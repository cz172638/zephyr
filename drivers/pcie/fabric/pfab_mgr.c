/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe fabric manager on the root complex. After the enumeration it
 * walks the tree (through the bridges, as the AER helpers do), takes
 * every type 0 function whose BAR0 starts with the PFAB magic as a
 * node, and hands out the cards:
 *
 *  1. withdraw the old cards (commit 0), the nodes stop,
 *  2. node IDs by DSN (1..n; 0 is the root complex's own node), roles
 *     from the table or the lowest role a node offers,
 *  3. switch ports: ACS P2P Request/Completion Redirect and Upstream
 *     Forwarding off, so peer traffic turns at the switch,
 *  4. Memory Space and Bus Master Enable on every node,
 *  5. per node the card: its ID and role, every other node with the
 *     bus address and geometry of its region; commit = epoch last,
 *  6. wait until every node reports RUNNING with this epoch.
 *
 * The root complex's node lives in RAM; the endpoints write there by
 * inbound DMA (bus address = physical + CONFIG_PCIE_FABRIC_RC_INBOUND_
 * OFFSET), the node writes into the endpoints' BARs through the CPU
 * windows.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pcie/aer.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>
#ifdef CONFIG_MMU
#include <zephyr/kernel/internal/mm.h>
#endif

LOG_MODULE_DECLARE(pcie_fabric, CONFIG_PCIE_FABRIC_LOG_LEVEL);

#ifndef CONFIG_PCIE_FABRIC_RC_INBOUND_OFFSET
#define CONFIG_PCIE_FABRIC_RC_INBOUND_OFFSET 0
#endif

#ifdef BUILD_VERSION
#define FW_STRING STRINGIFY(BUILD_VERSION)
#else
#define FW_STRING KERNEL_VERSION_STRING
#endif

#define MAX_EPS (CONFIG_PCIE_FABRIC_MAX_NODES - 1)

#define CFG_CMD     0x04
#define CFG_HDR     0x0c
#define CFG_BAR0    0x10
#define CMD_MEM     BIT(1)
#define CMD_BME     BIT(2)
#define EXT_ID_ACS  0x000d
#define ACS_RR      BIT(2) /* P2P Request Redirect */
#define ACS_CR      BIT(3) /* P2P Completion Redirect */
#define ACS_UF      BIT(4) /* Upstream Forwarding */

static struct pfab_mgr_ent ents[MAX_EPS];
static int n_ents;
static uint32_t epoch;
static struct k_mutex mgr_lock = Z_MUTEX_INITIALIZER(mgr_lock);

static struct {
	uint64_t dsn;
	uint16_t role;
	bool used;
} role_tab[CONFIG_PCIE_FABRIC_MAX_NODES];

/* ------------------------------------------------------------------------ */
/* Mapping (weak: tests run without an MMU and with RAM "BARs")              */

__weak volatile uint8_t *pfab_mgr_map(uintptr_t phys, size_t size)
{
#ifdef CONFIG_MMU
	uint8_t *v;

	k_mem_map_phys_bare(&v, phys, size, K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	return v;
#else
	ARG_UNUSED(size);
	return (volatile uint8_t *)phys;
#endif
}

__weak void pfab_mgr_unmap(volatile uint8_t *virt, size_t size)
{
#ifdef CONFIG_MMU
	k_mem_unmap_phys_bare((uint8_t *)virt, size);
#else
	ARG_UNUSED(virt);
	ARG_UNUSED(size);
#endif
}

/* ------------------------------------------------------------------------ */
/* The root complex's node                                                   */

#ifdef CONFIG_PCIE_FABRIC_RC_NODE
static uint8_t rc_region[CONFIG_PCIE_FABRIC_RC_REGION_SIZE] __aligned(4096);
static struct pfab_node rc_node;
static uint64_t rc_bus;
static bool rc_ready;
static struct k_thread rc_thread;
static K_KERNEL_STACK_DEFINE(rc_stack, CONFIG_PCIE_FABRIC_STACK_SIZE);

static void rc_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (rc_node.running) {
			(void)pfab_service(&rc_node);
			pfab_service_wait(&rc_node);
		} else {
			k_msleep(50);
		}
	}
}

static int rc_node_init(void)
{
	struct pfab_geom g;
	volatile uint8_t *v;
	uintptr_t phys;
	int ret;

	if (rc_ready) {
		return 0;
	}
	ret = pfab_geom_default(&g, sizeof(rc_region));
	if (ret != 0) {
		return ret;
	}
#ifdef CONFIG_MMU
	phys = k_mem_phys_addr(rc_region);
	/* written by DMA, read uncached: no line of the cached alias may linger */
	(void)sys_cache_data_flush_and_invd_range(rc_region, sizeof(rc_region));
	v = pfab_mgr_map(phys, sizeof(rc_region));
#else
	phys = (uintptr_t)rc_region;
	v = rc_region;
#endif
	rc_bus = (uint64_t)phys + CONFIG_PCIE_FABRIC_RC_INBOUND_OFFSET;
	pfab_region_init(v, &g, 0, PFAB_CAP_MSG | PFAB_CAP_RMA, BIT(PFAB_ROLE_MANAGER), FW_STRING,
			 CONFIG_BOARD);
	pfab_node_init(&rc_node, "rc", v, &g);
	pfab_node_register(&rc_node);
	k_thread_create(&rc_thread, rc_stack, K_KERNEL_STACK_SIZEOF(rc_stack), rc_thread_fn, NULL,
			NULL, NULL, CONFIG_PCIE_FABRIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&rc_thread, "pfab_rc");
	rc_ready = true;
	return 0;
}

struct pfab_node *pfab_mgr_node(void)
{
	return rc_ready ? &rc_node : NULL;
}
#else
struct pfab_node *pfab_mgr_node(void)
{
	return NULL;
}
#endif

/* ------------------------------------------------------------------------ */
/* Discovery                                                                 */

static inline uint32_t crd(const struct device *c, pcie_bdf_t bdf, uint32_t off)
{
	return pcie_ctrl_conf_read(c, bdf, off / 4);
}

static inline void cwr(const struct device *c, pcie_bdf_t bdf, uint32_t off, uint32_t v)
{
	pcie_ctrl_conf_write(c, bdf, off / 4, v);
}

static void rd_str(volatile const uint8_t *r, uint32_t off, char *s)
{
	for (int i = 0; i < PFAB_STR_LEN; i += 4) {
		uint32_t w = pfab_rd(r, off + i);

		memcpy(&s[i], &w, 4);
	}
	s[PFAB_STR_LEN] = '\0';
}

static void ent_refresh(struct pfab_mgr_ent *e)
{
	uint32_t node = pfab_rd(e->cpu, PFAB_INFO_NODE);

	e->state = pfab_rd(e->cpu, PFAB_INFO_STATE);
	e->dsn = pfab_rd64(e->cpu, PFAB_INFO_DSN);
	e->roles = pfab_rd(e->cpu, PFAB_INFO_ROLES);
	e->caps = pfab_rd(e->cpu, PFAB_INFO_CAPS);
	if (e->state == PFAB_STATE_RUNNING) {
		e->id = node & 0xffff;
		e->role = node >> 16;
	}
}

static void scan_fn(const struct device *c, pcie_bdf_t bdf, void *user)
{
	uint32_t bar, cmd;
	uint64_t bus;
	uintptr_t phys;
	bool mem64;
	volatile uint8_t *v;
	struct pfab_geom g;
	struct pfab_mgr_ent *e;

	ARG_UNUSED(user);
	if (n_ents >= MAX_EPS || ((crd(c, bdf, CFG_HDR) >> 16) & 0x7f) != 0) {
		return;
	}
	bar = crd(c, bdf, CFG_BAR0);
	if (bar == 0xffffffff || (bar & BIT(0)) != 0) {
		return; /* absent or I/O */
	}
	mem64 = ((bar >> 1) & 3) == 2;
	bus = bar & ~0xfULL;
	if (mem64) {
		bus |= (uint64_t)crd(c, bdf, CFG_BAR0 + 4) << 32;
	}
	if (bus == 0 || !pcie_ctrl_region_translate(c, bdf, true, mem64, (uintptr_t)bus, &phys)) {
		return;
	}
	/* the BAR must decode before we look */
	cmd = crd(c, bdf, CFG_CMD) & 0xffff;
	if ((cmd & CMD_MEM) == 0) {
		cwr(c, bdf, CFG_CMD, cmd | CMD_MEM);
	}
	v = pfab_mgr_map(phys, 0x1000);
	if (v == NULL) {
		return;
	}
	if (pfab_region_geom(v, &g) != 0) {
		pfab_mgr_unmap(v, 0x1000);
		if ((cmd & CMD_MEM) == 0) {
			cwr(c, bdf, CFG_CMD, cmd);
		}
		return;
	}
	pfab_mgr_unmap(v, 0x1000);
	e = &ents[n_ents];
	memset(e, 0, sizeof(*e));
	e->bdf = bdf;
	e->bus = bus;
	e->g = g;
	e->cpu = pfab_mgr_map(phys, g.size);
	if (e->cpu == NULL) {
		return;
	}
	rd_str(e->cpu, PFAB_INFO_FW, e->fw);
	rd_str(e->cpu, PFAB_INFO_BOARD, e->board);
	ent_refresh(e);
	n_ents++;
	LOG_INF("fabric: %02x:%02x.%x node, region 0x%x at 0x%llx, DSN %016llx, %s",
		PCIE_BDF_TO_BUS(bdf), PCIE_BDF_TO_DEV(bdf), PCIE_BDF_TO_FUNC(bdf), g.size,
		(unsigned long long)bus, (unsigned long long)e->dsn, e->board);
}

int pfab_mgr_scan(const struct device *ctrl, uint32_t root)
{
	int ret;

	k_mutex_lock(&mgr_lock, K_FOREVER);
	for (int i = 0; i < n_ents; i++) {
		pfab_mgr_unmap(ents[i].cpu, ents[i].g.size);
	}
	n_ents = 0;
	ret = pcie_aer_walk(ctrl, root, scan_fn, NULL);
	ret = ret < 0 ? ret : n_ents;
	k_mutex_unlock(&mgr_lock);
	return ret;
}

const struct pfab_mgr_ent *pfab_mgr_get(int i)
{
	if (i < 0 || i >= n_ents) {
		return NULL;
	}
	ent_refresh(&ents[i]);
	return &ents[i];
}

uint32_t pfab_mgr_epoch(void)
{
	return epoch;
}

int pfab_mgr_role_set(uint64_t dsn, uint16_t role)
{
	int free = -1;

	for (int i = 0; i < ARRAY_SIZE(role_tab); i++) {
		if (role_tab[i].used && role_tab[i].dsn == dsn) {
			role_tab[i].role = role;
			return 0;
		}
		if (!role_tab[i].used && free < 0) {
			free = i;
		}
	}
	if (free < 0) {
		return -ENOMEM;
	}
	role_tab[free].dsn = dsn;
	role_tab[free].role = role;
	role_tab[free].used = true;
	return 0;
}

static uint16_t role_for(const struct pfab_mgr_ent *e)
{
	for (int i = 0; i < ARRAY_SIZE(role_tab); i++) {
		if (role_tab[i].used && role_tab[i].dsn == e->dsn) {
			return role_tab[i].role;
		}
	}
	return e->roles != 0 ? (uint16_t)(find_lsb_set(e->roles) - 1) : PFAB_ROLE_NONE;
}

/* ------------------------------------------------------------------------ */
/* Switch ports: peer traffic turns at the switch                           */

static uint16_t find_ext(const struct device *c, pcie_bdf_t bdf, uint16_t id)
{
	uint32_t p = 0x100;

	for (int n = 0; n < 96 && p >= 0x100 && p < 0x1000; n++) {
		uint32_t v = crd(c, bdf, p);

		if (v == 0 || v == 0xffffffff) {
			return 0;
		}
		if ((v & 0xffff) == id) {
			return p;
		}
		p = (v >> 20) & 0xffc;
	}
	return 0;
}

static void acs_fn(const struct device *c, pcie_bdf_t bdf, void *user)
{
	int *n = user;
	uint16_t acs;
	uint32_t v;

	if (((crd(c, bdf, CFG_HDR) >> 16) & 0x7f) != 1) {
		return;
	}
	acs = find_ext(c, bdf, EXT_ID_ACS);
	if (acs == 0) {
		return;
	}
	v = crd(c, bdf, acs + 4); /* capability [15:0], control [31:16] */
	if (((v >> 16) & (ACS_RR | ACS_CR | ACS_UF)) != 0) {
		cwr(c, bdf, acs + 4, v & ~((uint32_t)(ACS_RR | ACS_CR | ACS_UF) << 16));
		(*n)++;
	}
}

/* ------------------------------------------------------------------------ */
/* Cards                                                                     */

struct peer_desc {
	uint16_t id, role;
	uint64_t dsn, bus;
	struct pfab_geom g;
	volatile uint8_t *cpu;
};

static void wr_peer(volatile uint8_t *e, const struct peer_desc *p)
{
	pfab_wr(e, PFAB_PEER_NODE, p->id | ((uint32_t)p->role << 16));
	pfab_wr64(e, PFAB_PEER_DSN, p->dsn);
	pfab_wr64(e, PFAB_PEER_BUS, p->bus);
	pfab_wr(e, PFAB_PEER_SIZE_OFF, p->g.size);
	pfab_wr(e, PFAB_PEER_GEOM, p->g.max_nodes | ((uint32_t)p->g.nslots << 16));
	pfab_wr(e, PFAB_PEER_SLOT, p->g.slot_size);
	pfab_wr(e, PFAB_PEER_RX_OFF, p->g.rx_off);
	pfab_wr(e, PFAB_PEER_CRED_OFF, p->g.cred_off);
	pfab_wr(e, PFAB_PEER_RMA_OFF, p->g.rma_off);
	pfab_wr(e, PFAB_PEER_RMA_SIZE, p->g.rma_size);
	pfab_wr(e, PFAB_PEER_FLAGS, PFAB_PEER_VALID);
}

static void withdraw(void)
{
	int64_t end = k_uptime_get() + 100;

	for (int i = 0; i < n_ents; i++) {
		pfab_wr(ents[i].cpu, PFAB_CARD_COMMIT, 0);
	}
#ifdef CONFIG_PCIE_FABRIC_RC_NODE
	if (rc_ready) {
		pfab_node_stop(&rc_node, NULL);
	}
#endif
	/* the nodes drop their peers before anyone gets new addresses */
	for (int i = 0; i < n_ents; i++) {
		while (pfab_rd(ents[i].cpu, PFAB_INFO_STATE) == PFAB_STATE_RUNNING &&
		       k_uptime_get() < end) {
			k_msleep(1);
		}
	}
}

void pfab_mgr_down(void)
{
	k_mutex_lock(&mgr_lock, K_FOREVER);
	withdraw();
	k_mutex_unlock(&mgr_lock);
}

int pfab_mgr_up(const struct device *ctrl, uint32_t root, k_timeout_t timeout)
{
	struct peer_desc pd[CONFIG_PCIE_FABRIC_MAX_NODES];
	k_timepoint_t end;
	int np = 0, running = 0, acs = 0;
	int ret;

	if (n_ents == 0) {
		ret = pfab_mgr_scan(ctrl, root);
		if (ret <= 0) {
			return ret;
		}
	}
	k_mutex_lock(&mgr_lock, K_FOREVER);
#ifdef CONFIG_PCIE_FABRIC_RC_NODE
	ret = rc_node_init();
	if (ret != 0) {
		goto out;
	}
#endif
	withdraw();

	/* node IDs by DSN: the same boards get the same IDs */
	for (int i = 0; i < n_ents; i++) {
		for (int j = i + 1; j < n_ents; j++) {
			if (ents[j].dsn < ents[i].dsn) {
				struct pfab_mgr_ent t = ents[i];

				ents[i] = ents[j];
				ents[j] = t;
			}
		}
	}
#ifdef CONFIG_PCIE_FABRIC_RC_NODE
	pd[np++] = (struct peer_desc){
		.id = 0, .role = PFAB_ROLE_MANAGER, .bus = rc_bus, .g = rc_node.g,
		.cpu = rc_node.local,
	};
#endif
	for (int i = 0; i < n_ents; i++) {
		ents[i].id = i + 1;
		ents[i].role = role_for(&ents[i]);
		pd[np++] = (struct peer_desc){
			.id = ents[i].id, .role = ents[i].role, .dsn = ents[i].dsn,
			.bus = ents[i].bus, .g = ents[i].g, .cpu = ents[i].cpu,
		};
	}

	(void)pcie_aer_walk(ctrl, root, acs_fn, &acs);
	epoch = epoch == 0 ? (k_cycle_get_32() | 1U) : epoch + 1;
	if (epoch == 0) {
		epoch = 1;
	}

	for (int i = 0; i < n_ents; i++) {
		struct pfab_mgr_ent *e = &ents[i];
		uint32_t cmd = crd(ctrl, e->bdf, CFG_CMD) & 0xffff;
		int k = 0;

		cwr(ctrl, e->bdf, CFG_CMD, cmd | CMD_MEM | CMD_BME);
		pfab_wr(e->cpu, PFAB_CARD_MAGIC_OFF, PFAB_CARD_MAGIC);
		pfab_wr(e->cpu, PFAB_CARD_EPOCH, epoch);
		pfab_wr(e->cpu, PFAB_CARD_NODE, e->id | ((uint32_t)e->role << 16));
		for (int j = 0; j < np; j++) {
			if (pd[j].id == e->id) {
				continue;
			}
			wr_peer(e->cpu + PFAB_CARD_PEERS + k * PFAB_PEER_SIZE, &pd[j]);
			k++;
		}
		pfab_wr(e->cpu, PFAB_CARD_NPEERS, k);
		barrier_dmem_fence_full();
		pfab_wr(e->cpu, PFAB_CARD_COMMIT, epoch);
	}

#ifdef CONFIG_PCIE_FABRIC_RC_NODE
	{
		struct pfab_peer peers[CONFIG_PCIE_FABRIC_MAX_NODES];
		int k = 0;

		for (int j = 0; j < np; j++) {
			if (pd[j].id == 0) {
				continue;
			}
			peers[k++] = (struct pfab_peer){
				.valid = true, .id = pd[j].id, .role = pd[j].role,
				.dsn = pd[j].dsn, .bus = pd[j].bus, .g = pd[j].g,
				.base = pd[j].cpu,
			};
		}
		pfab_node_start(&rc_node, 0, PFAB_ROLE_MANAGER, epoch, peers, k);
	}
#endif

	end = sys_timepoint_calc(timeout);
	do {
		running = 0;
		for (int i = 0; i < n_ents; i++) {
			if (pfab_rd(ents[i].cpu, PFAB_INFO_STATE) == PFAB_STATE_RUNNING &&
			    pfab_rd(ents[i].cpu, PFAB_INFO_EPOCH) == epoch) {
				running++;
			}
		}
		if (running == n_ents || sys_timepoint_expired(end)) {
			break;
		}
		k_msleep(1);
	} while (true);

	for (int i = 0; i < n_ents; i++) {
		ent_refresh(&ents[i]);
	}
	LOG_INF("fabric: epoch %08x, %d of %d endpoint(s) running, %d switch port(s) set "
		"for P2P", epoch, running, n_ents, acs);
	ret = running;
out:
	k_mutex_unlock(&mgr_lock);
	return ret;
}

/* ------------------------------------------------------------------------ */
/* After an enumeration (controller drivers call this)                       */

#ifdef CONFIG_PCIE_FABRIC_MGR_AUTO
static const struct device *auto_ctrl;
static int auto_tries;
/* bringing the fabric up waits for the nodes: not on the system work queue */
static struct k_work_q auto_q;
static K_KERNEL_STACK_DEFINE(auto_stack, CONFIG_PCIE_FABRIC_STACK_SIZE);
static bool auto_q_started;

static void auto_fn(struct k_work *w)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(w);
	int n = pfab_mgr_scan(auto_ctrl, PCIE_BDF(0, 0, 0));

	if (n > 0) {
		n = pfab_mgr_up(auto_ctrl, PCIE_BDF(0, 0, 0), K_MSEC(1000));
		if (n == n_ents) {
			return;
		}
	}
	/* endpoints still booting: a few more looks */
	if (++auto_tries < 10) {
		k_work_reschedule_for_queue(&auto_q, dw, K_MSEC(500));
	}
}

static K_WORK_DELAYABLE_DEFINE(auto_work, auto_fn);

void pfab_mgr_rescanned(const struct device *ctrl)
{
	if (!auto_q_started) {
		const struct k_work_queue_config qc = {.name = "pfab_mgr"};

		k_work_queue_start(&auto_q, auto_stack, K_KERNEL_STACK_SIZEOF(auto_stack),
				   CONFIG_PCIE_FABRIC_THREAD_PRIORITY, &qc);
		auto_q_started = true;
	}
	auto_ctrl = ctrl;
	auto_tries = 0;
	k_work_reschedule_for_queue(&auto_q, &auto_work, K_MSEC(200));
}
#else
void pfab_mgr_rescanned(const struct device *ctrl)
{
	ARG_UNUSED(ctrl);
}
#endif
