/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * vNTB endpoint function: a Non-Transparent Bridge between the PCIe host
 * and this Zephyr, the counterpart of the Linux pci-epf-vntb function
 * (drivers/pci/endpoint/functions/pci-epf-vntb.c). The host runs the
 * stock ntb_hw_epf driver and any NTB client on top of it (ntb_tool,
 * ntb_transport + ntb_netdev, ...); this side gets the same constructs
 * through the Zephyr NTB API (zephyr/drivers/ntb.h).
 *
 * BARs (layout picked in DT, it must be one ntb_hw_epf knows for the
 * vendor/device ID):
 *  - config BAR: the control region (struct epf_ntb_ctrl of Linux),
 *    the MSI-X table and the scratchpads; the host's own scratchpads
 *    first, then ours (the host reads ours as its peer scratchpads),
 *  - doorbell BAR: one 32-bit slot per doorbell; the host writes
 *    db_data[n] into slot n, this side polls the slots (there is no MSI
 *    doorbell to receive inbound writes as interrupts),
 *  - memory window BARs: local memory the host writes into.
 *
 * The host sends commands through the control region (configure the
 * doorbell interrupts, map a memory window of host memory, link up/down);
 * they are polled together with the doorbells. A host memory window is
 * mapped through an outbound window and is this side's "peer" window.
 *
 * Interrupt vectors toward the host (1-based): 1 = link event, 2 unused
 * (kept by the protocol), 3.. = doorbell 0... Doorbell n rung by the host
 * lands in slot n + 2.
 *
 * Only pcie_epc.h is used: the function knows nothing about the
 * controller.
 */

#define DT_DRV_COMPAT zephyr_pcie_epf_vntb

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/ntb.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>
#ifdef CONFIG_PCIE_AARDVARK_MODE
#include <zephyr/drivers/pcie/aardvark.h>
#endif
#ifdef CONFIG_MMU
#include <zephyr/kernel/internal/mm.h>
#endif

LOG_MODULE_REGISTER(pcie_epf_vntb, CONFIG_PCIE_EP_LOG_LEVEL);

/* Control region, Linux struct epf_ntb_ctrl (packed) */
#define CTRL_COMMAND       0x00
#define CTRL_ARGUMENT      0x04
#define CTRL_CMD_STATUS    0x08
#define CTRL_LINK_STATUS   0x0a
#define CTRL_TOPOLOGY      0x0c
#define CTRL_ADDR          0x10
#define CTRL_SIZE          0x18
#define CTRL_NUM_MWS       0x20
#define CTRL_MW1_OFFSET    0x24
#define CTRL_SPAD_OFFSET   0x28
#define CTRL_SPAD_COUNT    0x2c
#define CTRL_DB_ENTRY_SIZE 0x30
#define CTRL_DB_DATA(n)    (0x34 + 4 * (n))
#define CTRL_DB_OFFSET(n)  (0xb4 + 4 * (n))
#define CTRL_END           0x134

#define CMD_CONFIGURE_DOORBELL 1
#define CMD_TEARDOWN_DOORBELL  2
#define CMD_CONFIGURE_MW       3
#define CMD_TEARDOWN_MW        4
#define CMD_LINK_UP            5
#define CMD_LINK_DOWN          6

#define CMD_STATUS_OK    1
#define CMD_STATUS_ERROR 2

#define LINK_STATUS_UP BIT(0)
#define ARG_MSIX       BIT(16)
#define ARG_DB_COUNT   GENMASK(15, 0)

/* interrupt slots: 0 link, 1 unused, 2.. doorbells */
#define IRQ_LINK     0
#define IRQ_DB_START 2
#define MIN_DB_COUNT 3
#define MAX_DB_COUNT 32

#define MAX_MWS 4

#define MSIX_ENTRY 16U

/* Configuration space */
#define CFG_COMMAND     0x04
#define CFG_COMMAND_BME BIT(2)

struct epf_vntb_config {
	const struct device *epc;
	uint8_t func;
	struct pcie_epc_header hdr;
	uintptr_t mem_base;
	size_t mem_size;
	/* memory-region: physical memory to map; else a static buffer */
	bool mem_map;
	uint8_t ctrl_bar;
	uint8_t db_bar;
	uint8_t n_mws;
	uint8_t mw_bar[MAX_MWS];
	uint64_t mw_size[MAX_MWS];
	uint16_t spad_count;
	uint8_t db_count;
	k_thread_entry_t thread_fn;
};

struct epf_vntb_stats {
	uint32_t cmds;
	uint32_t cmd_errors;
	uint32_t db_in;
	uint32_t db_out;
	uint32_t irq_errors;
	uint32_t resets;
};

struct epf_vntb_data {
	const struct device *dev;
	struct pcie_epc_listener listener;
	struct k_mutex lock;
	struct k_sem kick;
	uintptr_t mem_virt;

	struct pcie_epc_bar bar[PCIE_EPC_NUM_BARS];
	volatile uint8_t *ctrl;
	volatile uint32_t *db;
	uint32_t ctrl_size;
	uint32_t msix_offset;
	uint32_t spad_offset;

	/* inbound windows: the BAR backing unless retargeted */
	bool mw_retargeted[MAX_MWS];
	/* outbound windows: host memory set up by CMD_CONFIGURE_MW */
	struct pcie_epc_host_map peer[MAX_MWS];
	uint64_t peer_addr[MAX_MWS];
	size_t peer_size[MAX_MWS];
	bool peer_mapped[MAX_MWS];

	bool pcie_link;
	bool bme;
	bool host_link;
	bool local_link;
	bool link_up;
	bool peer_msix;
	uint16_t host_vectors;

	atomic_t db_bits;
	atomic_t reset_req;
	atomic_t link_evt;

	const struct ntb_callbacks *cb;
	void *cb_data;

	struct epf_vntb_stats st;

	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_PCIE_EPF_VNTB_STACK_SIZE);
};

static inline uint32_t ctrl_rd(struct epf_vntb_data *d, uint32_t off)
{
	return sys_read32((mem_addr_t)(d->ctrl + off));
}

static inline void ctrl_wr(struct epf_vntb_data *d, uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)(d->ctrl + off));
}

static inline uint16_t ctrl_rd16(struct epf_vntb_data *d, uint32_t off)
{
	return sys_read16((mem_addr_t)(d->ctrl + off));
}

static inline void ctrl_wr16(struct epf_vntb_data *d, uint32_t off, uint16_t v)
{
	sys_write16(v, (mem_addr_t)(d->ctrl + off));
}

static inline uint64_t ctrl_rd64(struct epf_vntb_data *d, uint32_t off)
{
	return ((uint64_t)ctrl_rd(d, off + 4) << 32) | ctrl_rd(d, off);
}

/* host's own scratchpads (our peer ones) first, then ours */
static inline mem_addr_t spad_addr(struct epf_vntb_data *d, int idx, bool own)
{
	const struct epf_vntb_config *cfg = d->dev->config;

	return (mem_addr_t)(d->ctrl + d->spad_offset +
			    (own ? cfg->spad_count * 4U : 0U) + (uint32_t)idx * 4U);
}

static void bme_update(struct epf_vntb_data *d)
{
	const struct epf_vntb_config *cfg = d->dev->config;
	uint32_t v = 0;

	(void)pcie_epc_cfg_read(cfg->epc, cfg->func, CFG_COMMAND, &v);
	d->bme = (v & CFG_COMMAND_BME) != 0U;
}

/* number of interrupt slots in use: ours, or what the host allocated */
static uint32_t db_slots(struct epf_vntb_data *d)
{
	const struct epf_vntb_config *cfg = d->dev->config;

	return MIN(MAX((uint32_t)cfg->db_count, d->host_vectors), MAX_DB_COUNT);
}

static uint64_t vntb_db_valid_mask(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	uint32_t n = cfg->db_count;

	if (d->host_vectors != 0U) {
		n = MIN(n, d->host_vectors);
	}
	return n > IRQ_DB_START ? BIT64(n - IRQ_DB_START) - 1U : 0U;
}

/* raise interrupt slot @p slot (0-based) at the host */
static int raise_slot(struct epf_vntb_data *d, uint32_t slot)
{
	const struct epf_vntb_config *cfg = d->dev->config;
	int ret;

	if (!d->pcie_link || !d->bme) {
		return -EACCES;
	}
	if (d->host_vectors != 0U && slot >= d->host_vectors) {
		return -EINVAL;
	}
	ret = pcie_epc_raise_irq(cfg->epc, cfg->func,
				 d->peer_msix ? PCIE_EPC_IRQ_MSIX : PCIE_EPC_IRQ_MSI,
				 (uint16_t)(slot + 1U));
	if (ret != 0) {
		d->st.irq_errors++;
		LOG_WRN("interrupt %u to the host failed (%d)", slot + 1U, ret);
	}
	return ret;
}

/* with d->lock held */
static void update_link(struct epf_vntb_data *d)
{
	bool up = d->host_link && d->local_link;
	uint16_t st = ctrl_rd16(d, CTRL_LINK_STATUS);

	ctrl_wr16(d, CTRL_LINK_STATUS, up ? (st | LINK_STATUS_UP) : (st & ~LINK_STATUS_UP));
	if (up == d->link_up) {
		return;
	}
	d->link_up = up;
	LOG_INF("NTB link %s", up ? "up" : "down");
	atomic_set(&d->link_evt, 1);
	k_sem_give(&d->kick);
	if (d->host_vectors != 0U) {
		(void)raise_slot(d, IRQ_LINK);
	}
}

static void unmap_peer(struct epf_vntb_data *d, uint32_t idx)
{
	const struct epf_vntb_config *cfg = d->dev->config;

	if (d->peer_mapped[idx]) {
		pcie_epc_unmap_host(cfg->epc, &d->peer[idx]);
		d->peer_mapped[idx] = false;
		LOG_INF("peer mw %u unmapped", idx);
	}
}

static int configure_mw(struct epf_vntb_data *d, uint32_t idx)
{
	const struct epf_vntb_config *cfg = d->dev->config;
	uint64_t addr = ctrl_rd64(d, CTRL_ADDR);
	uint64_t size = ctrl_rd64(d, CTRL_SIZE);
	int ret;

	if (idx >= cfg->n_mws || size == 0U || size > SIZE_MAX) {
		LOG_WRN("configure mw %u: 0x%llx bytes at 0x%llx refused", idx,
			(unsigned long long)size, (unsigned long long)addr);
		return -EINVAL;
	}
	unmap_peer(d, idx);
	ret = pcie_epc_map_host(cfg->epc, cfg->func, addr, (size_t)size, &d->peer[idx]);
	if (ret != 0) {
		LOG_ERR("peer mw %u: mapping 0x%llx bytes at 0x%llx failed (%d)", idx,
			(unsigned long long)size, (unsigned long long)addr, ret);
		return ret;
	}
	d->peer_addr[idx] = addr;
	d->peer_size[idx] = (size_t)size;
	d->peer_mapped[idx] = true;
	LOG_INF("peer mw %u: %llu KiB of host memory at 0x%llx", idx,
		(unsigned long long)(size / 1024U), (unsigned long long)addr);
	return 0;
}

static int run_command(struct epf_vntb_data *d, uint32_t cmd, uint32_t arg)
{
	const struct epf_vntb_config *cfg = d->dev->config;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);

	switch (cmd) {
	case CMD_CONFIGURE_DOORBELL: {
		bool msix = (arg & ARG_MSIX) != 0U;

		if (feat == NULL || (msix && !feat->msix_capable) ||
		    (!msix && !feat->msi_capable)) {
			return -ENOTSUP;
		}
		d->peer_msix = msix;
		d->host_vectors = (uint16_t)MIN(arg & ARG_DB_COUNT, MAX_DB_COUNT);
		LOG_INF("host: %u %s vectors", d->host_vectors, msix ? "MSI-X" : "MSI");
		return 0;
	}
	case CMD_TEARDOWN_DOORBELL:
		d->host_vectors = 0;
		return 0;
	case CMD_CONFIGURE_MW:
		return configure_mw(d, arg);
	case CMD_TEARDOWN_MW:
		if (arg < cfg->n_mws) {
			unmap_peer(d, arg);
		}
		return 0;
	case CMD_LINK_UP:
	case CMD_LINK_DOWN:
		d->host_link = cmd == CMD_LINK_UP;
		update_link(d);
		return 0;
	default:
		LOG_WRN("unknown command %u (argument 0x%x)", cmd, arg);
		return -EINVAL;
	}
}

static void poll_commands(struct epf_vntb_data *d)
{
	uint32_t cmd = ctrl_rd(d, CTRL_COMMAND);
	uint32_t arg;
	int ret;

	if (cmd == 0U) {
		return;
	}
	arg = ctrl_rd(d, CTRL_ARGUMENT);
	ctrl_wr(d, CTRL_COMMAND, 0);
	ctrl_wr(d, CTRL_ARGUMENT, 0);

	k_mutex_lock(&d->lock, K_FOREVER);
	ret = run_command(d, cmd, arg);
	k_mutex_unlock(&d->lock);

	d->st.cmds++;
	if (ret != 0) {
		d->st.cmd_errors++;
	}
	/* the host polls the status, then clears it */
	barrier_dmem_fence_full();
	ctrl_wr16(d, CTRL_CMD_STATUS, ret == 0 ? CMD_STATUS_OK : CMD_STATUS_ERROR);
}

static void poll_doorbells(struct epf_vntb_data *d)
{
	uint32_t n = db_slots(d);

	for (uint32_t s = IRQ_DB_START; s < n; s++) {
		int vector = (int)(s - IRQ_DB_START);

		if (d->db[s] == 0U) {
			continue;
		}
		d->db[s] = 0;
		d->st.db_in++;
		atomic_or(&d->db_bits, (atomic_val_t)BIT(vector));
		if (d->cb != NULL && d->cb->db_event != NULL) {
			d->cb->db_event(d->dev, vector, d->cb_data);
		}
	}
}

/* host gone (link down, hot reset, FLR, PERST#): back to the reset state */
static void reset_state(struct epf_vntb_data *d)
{
	const struct epf_vntb_config *cfg = d->dev->config;

	k_mutex_lock(&d->lock, K_FOREVER);
	for (uint32_t i = 0; i < cfg->n_mws; i++) {
		unmap_peer(d, i);
	}
	d->host_vectors = 0;
	d->peer_msix = false;
	d->host_link = false;
	update_link(d);
	ctrl_wr(d, CTRL_COMMAND, 0);
	ctrl_wr(d, CTRL_ARGUMENT, 0);
	ctrl_wr16(d, CTRL_CMD_STATUS, 0);
	for (uint32_t s = 0; s < MAX_DB_COUNT; s++) {
		d->db[s] = 0;
	}
	/* scratchpads the host wrote belong to the host that went away */
	for (int i = 0; i < cfg->spad_count; i++) {
		sys_write32(0, spad_addr(d, i, false));
	}
	k_mutex_unlock(&d->lock);
	d->st.resets++;
}

static void vntb_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct epf_vntb_data *d = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (atomic_cas(&d->reset_req, 1, 0)) {
			reset_state(d);
		}
		if (d->pcie_link) {
			bme_update(d);
			poll_commands(d);
			poll_doorbells(d);
		}
		if (atomic_cas(&d->link_evt, 1, 0) && d->cb != NULL &&
		    d->cb->link_event != NULL) {
			d->cb->link_event(dev, d->cb_data);
		}
		(void)k_sem_take(&d->kick, d->pcie_link ? K_USEC(CONFIG_PCIE_EPF_VNTB_POLL_US)
							: K_MSEC(100));
	}
}

static void vntb_event(const struct device *epc, struct pcie_epc_listener *l,
		       enum pcie_epc_event evt, uint8_t func)
{
	struct epf_vntb_data *d = CONTAINER_OF(l, struct epf_vntb_data, listener);

	ARG_UNUSED(epc);
	ARG_UNUSED(func);

	switch (evt) {
	case PCIE_EPC_EVT_LINK_UP:
		d->pcie_link = true;
		break;
	case PCIE_EPC_EVT_LINK_DOWN:
		d->pcie_link = false;
		atomic_set(&d->reset_req, 1);
		break;
	case PCIE_EPC_EVT_HOT_RESET:
	case PCIE_EPC_EVT_FLR:
	case PCIE_EPC_EVT_PERST_ASSERT:
		atomic_set(&d->reset_req, 1);
		break;
	default:
		break;
	}
	k_sem_give(&d->kick);
}

/*
 * NTB API
 */
static int vntb_set_callbacks(const struct device *dev, const struct ntb_callbacks *cb,
			      void *user_data)
{
	struct epf_vntb_data *d = dev->data;

	if (cb != NULL && d->cb != NULL && d->cb != cb) {
		return -EBUSY;
	}
	d->cb_data = user_data;
	d->cb = cb;
	return 0;
}

static int vntb_link_enable(const struct device *dev, bool enable)
{
	struct epf_vntb_data *d = dev->data;

	k_mutex_lock(&d->lock, K_FOREVER);
	d->local_link = enable;
	update_link(d);
	k_mutex_unlock(&d->lock);
	return 0;
}

static bool vntb_link_is_up(const struct device *dev)
{
	struct epf_vntb_data *d = dev->data;

	return d->link_up;
}

static int vntb_spad_count(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;

	return cfg->spad_count;
}

static uint32_t spad_rd(const struct device *dev, int idx, bool own)
{
	const struct epf_vntb_config *cfg = dev->config;

	if (idx < 0 || idx >= cfg->spad_count) {
		return 0;
	}
	return sys_read32(spad_addr(dev->data, idx, own));
}

static int spad_wr(const struct device *dev, int idx, uint32_t val, bool own)
{
	const struct epf_vntb_config *cfg = dev->config;

	if (idx < 0 || idx >= cfg->spad_count) {
		return -EINVAL;
	}
	sys_write32(val, spad_addr(dev->data, idx, own));
	return 0;
}

static uint32_t vntb_spad_read(const struct device *dev, int idx)
{
	return spad_rd(dev, idx, true);
}

static int vntb_spad_write(const struct device *dev, int idx, uint32_t val)
{
	return spad_wr(dev, idx, val, true);
}

static uint32_t vntb_peer_spad_read(const struct device *dev, int idx)
{
	return spad_rd(dev, idx, false);
}

static int vntb_peer_spad_write(const struct device *dev, int idx, uint32_t val)
{
	return spad_wr(dev, idx, val, false);
}

static uint64_t vntb_db_read(const struct device *dev)
{
	struct epf_vntb_data *d = dev->data;

	return (uint64_t)atomic_get(&d->db_bits);
}

static int vntb_db_clear(const struct device *dev, uint64_t bits)
{
	struct epf_vntb_data *d = dev->data;

	atomic_and(&d->db_bits, ~(atomic_val_t)bits);
	return 0;
}

static int vntb_peer_db_set(const struct device *dev, uint64_t bits)
{
	struct epf_vntb_data *d = dev->data;
	int ret = 0;

	if ((bits & ~vntb_db_valid_mask(dev)) != 0U || bits == 0U) {
		return -EINVAL;
	}
	k_mutex_lock(&d->lock, K_FOREVER);
	if (d->host_vectors == 0U) {
		ret = -ENOTCONN;
	}
	for (uint32_t b = 0; b < 64U && ret == 0; b++) {
		if ((bits & BIT64(b)) != 0U) {
			ret = raise_slot(d, b + IRQ_DB_START);
			if (ret == 0) {
				d->st.db_out++;
			}
		}
	}
	k_mutex_unlock(&d->lock);
	return ret;
}

static int vntb_mw_count(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;

	return cfg->n_mws;
}

static int vntb_mw_get_align(const struct device *dev, int idx, struct ntb_mw_align *align)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	size_t sz;

	if (idx < 0 || idx >= cfg->n_mws) {
		return -EINVAL;
	}
	sz = d->bar[cfg->mw_bar[idx]].size;
	/* inbound translation of a BAR: aligned to the BAR size */
	align->addr_align = MAX(sz, feat != NULL ? feat->align : 0U);
	align->size_align = 1;
	align->size_max = sz;
	return 0;
}

static int vntb_mw_set_trans(const struct device *dev, int idx, uint64_t phys, size_t size)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	struct pcie_epc_bar bar;
	int ret;

	if (idx < 0 || idx >= cfg->n_mws) {
		return -EINVAL;
	}
	bar = d->bar[cfg->mw_bar[idx]];
	if (size > bar.size) {
		return -EINVAL;
	}
	/* the host sized the BAR at enumeration: only the backing moves */
	if (phys != 0U) {
		bar.phys = phys;
		bar.virt = NULL;
	}
	ret = pcie_epc_set_bar(cfg->epc, cfg->func, &bar);
	if (ret == 0) {
		d->mw_retargeted[idx] = phys != 0U;
	}
	return ret;
}

static int vntb_mw_get_local(const struct device *dev, int idx, void **virt, uint64_t *phys,
			     size_t *size)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	const struct pcie_epc_bar *bar;

	if (idx < 0 || idx >= cfg->n_mws) {
		return -EINVAL;
	}
	if (d->mw_retargeted[idx]) {
		return -ENODATA;
	}
	bar = &d->bar[cfg->mw_bar[idx]];
	if (virt != NULL) {
		*virt = bar->virt;
	}
	if (phys != NULL) {
		*phys = bar->phys;
	}
	if (size != NULL) {
		*size = bar->size;
	}
	return 0;
}

static int vntb_peer_mw_get_addr(const struct device *dev, int idx, volatile void **virt,
				 size_t *size)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;

	if (idx < 0 || idx >= cfg->n_mws) {
		return -EINVAL;
	}
	if (!d->peer_mapped[idx]) {
		return -ENODATA;
	}
	if (virt != NULL) {
		*virt = d->peer[idx].ptr;
	}
	if (size != NULL) {
		*size = d->peer_size[idx];
	}
	return 0;
}

static int vntb_peer_mw_xfer(const struct device *dev, int idx, size_t offset, void *local,
			     size_t len, bool to_peer)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	uint64_t pci;
	int ret = -ENOTSUP;

	if (idx < 0 || idx >= cfg->n_mws) {
		return -EINVAL;
	}
	if (!d->peer_mapped[idx]) {
		return -ENODATA;
	}
	if (offset > d->peer_size[idx] || len > d->peer_size[idx] - offset) {
		return -EINVAL;
	}
	if (!d->bme) {
		/* outbound requests without bus mastering abort on some controllers */
		return -EACCES;
	}
	pci = d->peer_addr[idx] + offset;
	if (IS_ENABLED(CONFIG_PCIE_EPF_VNTB_DMA)) {
		ret = pcie_epc_xfer_dma(cfg->epc, cfg->func, pci, local, len, to_peer);
	}
	if (ret == -ENOTSUP) {
		ret = pcie_epc_xfer_memcpy(cfg->epc, cfg->func, pci, local, len, to_peer);
	}
	return ret;
}

static DEVICE_API(ntb, vntb_api) = {
	.set_callbacks = vntb_set_callbacks,
	.link_enable = vntb_link_enable,
	.link_is_up = vntb_link_is_up,
	.spad_count = vntb_spad_count,
	.spad_read = vntb_spad_read,
	.spad_write = vntb_spad_write,
	.peer_spad_read = vntb_peer_spad_read,
	.peer_spad_write = vntb_peer_spad_write,
	.db_valid_mask = vntb_db_valid_mask,
	.db_read = vntb_db_read,
	.db_clear = vntb_db_clear,
	.peer_db_set = vntb_peer_db_set,
	.mw_count = vntb_mw_count,
	.mw_get_align = vntb_mw_get_align,
	.mw_set_trans = vntb_mw_set_trans,
	.mw_get_local = vntb_mw_get_local,
	.peer_mw_count = vntb_mw_count,
	.peer_mw_get_addr = vntb_peer_mw_get_addr,
	.peer_mw_xfer = vntb_peer_mw_xfer,
};

/*
 * Setup
 */
struct bar_req {
	uint8_t barno;
	size_t size;
};

static int check_bars(const struct epf_vntb_config *cfg, const struct pcie_epc_features *feat)
{
	uint8_t used = BIT(cfg->ctrl_bar) | BIT(cfg->db_bar);
	uint8_t n = 2;

	if (cfg->ctrl_bar == cfg->db_bar) {
		LOG_ERR("config and doorbell BAR must differ");
		return -EINVAL;
	}
	for (uint8_t i = 0; i < cfg->n_mws; i++) {
		used |= BIT(cfg->mw_bar[i]);
		n++;
	}
	if (POPCOUNT(used) != n) {
		LOG_ERR("a BAR is used twice");
		return -EINVAL;
	}
	for (uint8_t b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		if ((used & BIT(b)) == 0U) {
			continue;
		}
		if (feat != NULL && (feat->reserved_bar & BIT(b)) != 0U) {
			LOG_ERR("BAR%u is not available on %s", b, cfg->epc->name);
			return -EINVAL;
		}
		/* the upper half of a 64-bit BAR below */
		if (b > 0 && (used & BIT(b - 1)) != 0U && feat != NULL &&
		    (feat->bar_fixed_64bit & BIT(b - 1)) != 0U) {
			LOG_ERR("BAR%u is the upper half of 64-bit BAR%u", b, b - 1);
			return -EINVAL;
		}
	}
	return 0;
}

static int setup_bars(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	size_t align = feat != NULL ? MAX(feat->align, 128U) : 128U;
	struct bar_req req[2 + MAX_MWS];
	uintptr_t base = cfg->mem_base;
	uint8_t used = 0;
	int n = 0, ret;

	ret = check_bars(cfg, feat);
	if (ret != 0) {
		return ret;
	}

	/* control region, MSI-X table and PBA, both halves of the scratchpads */
	d->ctrl_size = CTRL_END;
	if (feat != NULL && feat->msix_capable) {
		d->msix_offset = ROUND_UP(CTRL_END, 8);
		d->ctrl_size = d->msix_offset + cfg->db_count * MSIX_ENTRY +
			       DIV_ROUND_UP(cfg->db_count, 64) * 8U;
	}
	d->spad_offset = ROUND_UP(d->ctrl_size, 4);

	req[n++] = (struct bar_req){cfg->ctrl_bar,
				    d->spad_offset + 2U * cfg->spad_count * 4U};
	req[n++] = (struct bar_req){cfg->db_bar, MAX_DB_COUNT * 4U};
	for (int i = 0; i < cfg->n_mws; i++) {
		if (cfg->mw_size[i] > SIZE_MAX) {
			return -EINVAL;
		}
		req[n++] = (struct bar_req){cfg->mw_bar[i], (size_t)cfg->mw_size[i]};
	}
	for (int i = 0; i < n; i++) {
		size_t sz = pcie_epf_bar_size(feat, req[i].barno, req[i].size);

		if (sz == 0 || sz < req[i].size) {
			LOG_ERR("BAR%u: 0x%zx bytes not possible", req[i].barno, req[i].size);
			return -EINVAL;
		}
		req[i].size = sz;
	}

	/* largest first, each aligned to its size: no holes for the windows */
	for (int i = 0; i < n; i++) {
		for (int j = i + 1; j < n; j++) {
			if (req[j].size > req[i].size) {
				struct bar_req t = req[i];

				req[i] = req[j];
				req[j] = t;
			}
		}
	}
	for (int i = 0; i < n; i++) {
		struct pcie_epc_bar *bar = &d->bar[req[i].barno];
		bool b64 = feat != NULL && (feat->bar_fixed_64bit & BIT(req[i].barno)) != 0U;

		base = ROUND_UP(base, MAX(req[i].size, align));
		if (base + req[i].size > cfg->mem_base + cfg->mem_size) {
			LOG_ERR("memory-region too small: BAR%u needs 0x%zx at 0x%lx",
				req[i].barno, req[i].size, (unsigned long)base);
			return -ENOMEM;
		}
		*bar = (struct pcie_epc_bar){
			.barno = req[i].barno,
			.flags = b64 ? PCIE_EPC_BAR_MEM64 : PCIE_EPC_BAR_MEM32,
			.phys = pcie_epc_cpu_to_axi(cfg->epc, base),
			.size = req[i].size,
			.virt = (void *)(d->mem_virt + (base - cfg->mem_base)),
		};
		memset(bar->virt, 0, bar->size);
		base += req[i].size;
	}

	d->ctrl = d->bar[cfg->ctrl_bar].virt;
	d->db = d->bar[cfg->db_bar].virt;
	ctrl_wr(d, CTRL_NUM_MWS, cfg->n_mws);
	ctrl_wr(d, CTRL_SPAD_OFFSET, d->spad_offset);
	ctrl_wr(d, CTRL_SPAD_COUNT, cfg->spad_count);
	ctrl_wr(d, CTRL_DB_ENTRY_SIZE, sizeof(uint32_t));
	for (uint32_t i = 0; i < MAX_DB_COUNT; i++) {
		ctrl_wr(d, CTRL_DB_DATA(i), i + 1U);
		ctrl_wr(d, CTRL_DB_OFFSET(i), 0);
	}

	for (int i = 0; i < n; i++) {
		ret = pcie_epc_set_bar(cfg->epc, cfg->func, &d->bar[req[i].barno]);
		if (ret != 0) {
			LOG_ERR("set_bar %u failed (%d)", req[i].barno, ret);
			return ret;
		}
		used |= BIT(req[i].barno);
	}
	/* whatever the controller enables at reset goes away */
	for (uint8_t b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		if ((used & BIT(b)) == 0U &&
		    (feat == NULL || (feat->reserved_bar & BIT(b)) == 0U)) {
			pcie_epc_clear_bar(cfg->epc, cfg->func, b);
		}
	}
	return 0;
}

static int setup_irqs(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	bool any = false;

	if (feat == NULL || feat->msi_capable) {
		if (pcie_epc_set_msi(cfg->epc, cfg->func,
				     (uint8_t)MIN(1U << LOG2CEIL(cfg->db_count), 32U)) == 0) {
			any = true;
		}
	}
	if (d->msix_offset != 0U &&
	    pcie_epc_set_msix(cfg->epc, cfg->func, cfg->db_count, cfg->ctrl_bar,
			      d->msix_offset) == 0) {
		any = true;
	} else {
		d->msix_offset = 0;
	}
	if (!any) {
		LOG_ERR("doorbells toward the host need MSI or MSI-X");
		return -ENOTSUP;
	}
	return 0;
}

static int epf_vntb_init(const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	int ret;

	d->dev = dev;
	k_mutex_init(&d->lock);
	k_sem_init(&d->kick, 0, 1);

	if (!device_is_ready(cfg->epc)) {
#if defined(CONFIG_PCIE_AARDVARK_MODE)
		if (advk_mode_get() == ADVK_MODE_RC) {
			LOG_INF("%s: root complex mode, function off", dev->name);
			return -ENODEV;
		}
#endif
		LOG_ERR("controller %s not ready", cfg->epc->name);
		return -ENODEV;
	}
	if (cfg->db_count < MIN_DB_COUNT || cfg->db_count > MAX_DB_COUNT ||
	    cfg->n_mws == 0U || cfg->n_mws > MAX_MWS || cfg->spad_count == 0U) {
		LOG_ERR("db-count %u..%u, 1..%u windows, spad-count > 0", MIN_DB_COUNT,
			MAX_DB_COUNT, MAX_MWS);
		return -EINVAL;
	}

	ret = pcie_epc_claim_function(cfg->epc, cfg->func);
	if (ret != 0) {
		LOG_ERR("function %u: claim failed (%d)", cfg->func, ret);
		return ret;
	}
	ret = pcie_epc_write_header(cfg->epc, cfg->func, &cfg->hdr);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_MMU
	if (cfg->mem_map) {
		/* outside the kernel image, possibly at physical 0: no 1:1 map */
		k_mem_map_phys_bare((uint8_t **)&d->mem_virt, cfg->mem_base, cfg->mem_size,
				    K_MEM_PERM_RW |
#ifdef K_MEM_ARM_NORMAL_NC
				    K_MEM_ARM_NORMAL_NC
#else
				    K_MEM_CACHE_NONE
#endif
		);
	} else {
		d->mem_virt = cfg->mem_base;
	}
#else
	d->mem_virt = cfg->mem_base;
#endif

	ret = setup_bars(dev);
	if (ret != 0) {
		return ret;
	}
	ret = setup_irqs(dev);
	if (ret != 0) {
		return ret;
	}

	d->pcie_link = pcie_epc_link_up(cfg->epc);
	d->listener.func = cfg->func;
	d->listener.event = vntb_event;
	pcie_epc_add_listener(cfg->epc, &d->listener);

	k_thread_create(&d->thread, d->stack, K_KERNEL_STACK_SIZEOF(d->stack), cfg->thread_fn,
			(void *)dev, NULL, NULL, CONFIG_PCIE_EPF_VNTB_THREAD_PRIORITY, 0,
			K_NO_WAIT);
	k_thread_name_set(&d->thread, dev->name);

	LOG_INF("%s: fn%u %04x:%04x on %s, config BAR%u, doorbell BAR%u, %u window(s), "
		"%u scratchpads, %u doorbells", dev->name, cfg->func, cfg->hdr.vendor_id,
		cfg->hdr.device_id, cfg->epc->name, cfg->ctrl_bar, cfg->db_bar, cfg->n_mws,
		cfg->spad_count, cfg->db_count - IRQ_DB_START);
	return 0;
}

#if defined(CONFIG_PCIE_EPF_VNTB_SHELL)
#include <zephyr/shell/shell.h>

static void vntb_status(const struct shell *sh, const struct device *dev)
{
	const struct epf_vntb_config *cfg = dev->config;
	struct epf_vntb_data *d = dev->data;
	const struct epf_vntb_stats *s = &d->st;
	uint32_t pending = 0;

	shell_print(sh, "%s: fn%u on %s, pcie link %s, bus master %s", dev->name, cfg->func,
		    cfg->epc->name, d->pcie_link ? "up" : "down", d->bme ? "on" : "off");
	for (uint8_t b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		const struct pcie_epc_bar *bar = &d->bar[b];
		const char *what = b == cfg->ctrl_bar ? "config" : b == cfg->db_bar ? "doorbell"
										  : "window";

		if (bar->size != 0U) {
			shell_print(sh, "  BAR%u %-8s 0x%08zx bytes at 0x%llx%s", b, what,
				    bar->size, (unsigned long long)bar->phys,
				    bar->flags & PCIE_EPC_BAR_MEM64 ? " (64-bit)" : "");
		}
	}
	shell_print(sh, "  ctrl: command %u argument 0x%x status %u link 0x%x, MSI-X table %s0x%x, "
		    "scratchpads at 0x%x", ctrl_rd(d, CTRL_COMMAND), ctrl_rd(d, CTRL_ARGUMENT),
		    ctrl_rd16(d, CTRL_CMD_STATUS), ctrl_rd16(d, CTRL_LINK_STATUS),
		    d->msix_offset != 0U ? "" : "off ", d->msix_offset, d->spad_offset);
	for (uint32_t i = 0; i < MAX_DB_COUNT; i++) {
		if (d->db[i] != 0U) {
			pending |= BIT(i);
		}
	}
	shell_print(sh, "  link: host %s, local %s -> %s; host %u %s vectors; slots set 0x%08x",
		    d->host_link ? "on" : "off", d->local_link ? "on" : "off",
		    d->link_up ? "up" : "down", d->host_vectors, d->peer_msix ? "MSI-X" : "MSI",
		    pending);
	for (uint8_t i = 0; i < cfg->n_mws; i++) {
		if (d->peer_mapped[i]) {
			shell_print(sh, "  peer mw %u: host 0x%llx, %zu bytes, cpu %p", i,
				    (unsigned long long)d->peer_addr[i], d->peer_size[i],
				    (void *)d->peer[i].ptr);
		}
	}
	shell_print(sh, "  commands %u (failed %u), doorbells in %u out %u, interrupt errors %u, "
		    "resets %u", s->cmds, s->cmd_errors, s->db_in, s->db_out, s->irq_errors,
		    s->resets);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	STRUCT_SECTION_FOREACH(device, dev) {
		if (dev->api == &vntb_api && device_is_ready(dev)) {
			vntb_status(sh, dev);
		}
	}
	return 0;
}

SHELL_CMD_REGISTER(vntb, NULL, "vNTB endpoint function state", cmd_status);
#endif

/*
 * Without memory-region (targets where any RAM is reachable from the
 * controller), the BARs live in a static buffer.
 */
#define EPF_VNTB_MEM(n)                                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),                                       \
		    (.mem_base = DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region)),                   \
		     .mem_size = DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region)),                   \
		     .mem_map = true,),                                                            \
		    (.mem_base = (uintptr_t)epf_vntb_mem_##n,                                      \
		     .mem_size = sizeof(epf_vntb_mem_##n),))

#define EPF_VNTB_MEM_BUF(n)                                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region), (),                                   \
		    (static uint8_t epf_vntb_mem_##n[CONFIG_PCIE_EPF_VNTB_STATIC_MEM_SIZE]         \
		     __aligned(CONFIG_PCIE_EPF_VNTB_STATIC_MEM_SIZE);))

#define EPF_VNTB_MW_SIZE(node, prop, idx) DT_PROP_BY_IDX(node, prop, idx),

#define EPF_VNTB_DEFINE(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, mw_bars) == DT_INST_PROP_LEN(n, mw_sizes),               \
		     "one mw-sizes entry per mw-bars entry");                                      \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, mw_bars) <= MAX_MWS, "4 memory windows at most");         \
	EPF_VNTB_MEM_BUF(n)                                                                        \
	static void epf_vntb_thread_##n(void *p1, void *p2, void *p3)                              \
	{                                                                                          \
		vntb_thread(p1, p2, p3);                                                           \
	}                                                                                          \
	static const struct epf_vntb_config epf_vntb_cfg_##n = {                                   \
		.epc = PCIE_EPF_DT_INST_EPC(n),                                                    \
		.func = PCIE_EPF_DT_INST_FUNC(n),                                                  \
		.hdr = PCIE_EPF_DT_HEADER(DT_DRV_INST(n)),                                         \
		EPF_VNTB_MEM(n)                                                                    \
		.ctrl_bar = DT_INST_PROP(n, ctrl_bar),                                             \
		.db_bar = DT_INST_PROP(n, db_bar),                                                 \
		.n_mws = DT_INST_PROP_LEN(n, mw_bars),                                             \
		.mw_bar = DT_INST_PROP(n, mw_bars),                                                \
		.mw_size = {DT_INST_FOREACH_PROP_ELEM(n, mw_sizes, EPF_VNTB_MW_SIZE)},             \
		.spad_count = DT_INST_PROP(n, spad_count),                                         \
		.db_count = DT_INST_PROP(n, db_count),                                             \
		.thread_fn = epf_vntb_thread_##n,                                                  \
	};                                                                                         \
	static struct epf_vntb_data epf_vntb_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, epf_vntb_init, NULL, &epf_vntb_data_##n, &epf_vntb_cfg_##n,       \
			      POST_KERNEL, CONFIG_PCIE_EPF_INIT_PRIORITY, &vntb_api);

DT_INST_FOREACH_STATUS_OKAY(EPF_VNTB_DEFINE)
