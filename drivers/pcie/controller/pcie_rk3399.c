/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 PCIe controller as root complex.
 *
 * Cadence core behind the Rockchip client wrapper (apb + 0x800000); the
 * root port is real hardware: its configuration space is at core + 0
 * (normal view) and core + 0x200000 (privileged view: class, link
 * control, ...). Downstream configuration requests go through outbound
 * region 0 (AXI 0xf8000000, 32 MiB, bus << 20 | devfn << 12), switched
 * between type 0 and type 1 per access. Regions 1..32 decode the next
 * 32 MiB in 1 MiB pieces: memory and I/O from "ranges".
 *
 * Same controller as pcie_epc_rk3399.c; the role comes from
 * zephyr/drivers/pcie/role.h, bring-up is shared (pcie_rk3399_port.c).
 * Sequence after Linux drivers/pci/controller/pcie-rockchip-host.c.
 *
 * Not yet: INTx/MSI (MSI needs the GIC ITS), AER, ASPM/L1SS.
 */

#define DT_DRV_COMPAT rockchip_rk3399_pcie

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/pcie.h>
#if defined(CONFIG_PCIE_ROLE)
#include <zephyr/drivers/pcie/role.h>
#endif
#if defined(CONFIG_PCIE_CTRL_SHELL)
#include <zephyr/drivers/pcie/pcie_shell.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "../pcie_rk3399_port.h"

LOG_MODULE_REGISTER(pcie_rk3399_rc, LOG_LEVEL_INF);

/* Inside the Cadence core (core = apb + RK_CORE_OFFSET) */
#define LM_BASE              0x100000U
#define LM_CTRL              (LM_BASE + 0x000U)
#define  LM_CTRL_LANE        GENMASK(2, 1)
#define  LM_CTRL_SPEED       GENMASK(4, 3)
#define  LM_CTRL_SPEED_5G    0x1U
#define LM_PLC1              (LM_BASE + 0x004U)
#define  LM_PLC1_FTS         GENMASK(23, 8)
#define LM_VENDOR            (LM_BASE + 0x044U)
#define LM_RC_BAR_CONF       (LM_BASE + 0x300U)

#define RP_PRIV              0x200000U
#define RP_ID                (RP_PRIV + 0x000U)
#define RP_RID_CCR           (RP_PRIV + 0x008U)
#define RP_DCSR              (RP_PRIV + 0x0c8U)
#define  RP_DCSR_MPS         GENMASK(7, 5)
#define RP_LCS               (RP_PRIV + 0x0d0U)
#define  LNKCTL_RCB          BIT(3)
#define  LNKCTL_RL           BIT(5)
#define  LNKSTA_SLC          BIT(12 + 16)
#define RP_THP_CAP           (RP_PRIV + 0x274U)
#define  THP_CAP_NEXT        GENMASK(31, 20)

#define AT_BASE              0x400000U
#define AT_OB_ADDR0(r)       (AT_BASE + (r) * 0x20U + 0x00U)
#define AT_OB_ADDR1(r)       (AT_BASE + (r) * 0x20U + 0x04U)
#define AT_OB_DESC0(r)       (AT_BASE + (r) * 0x20U + 0x08U)
#define AT_OB_DESC1(r)       (AT_BASE + (r) * 0x20U + 0x0cU)
#define  OB_DESC0_RC_TRANS   BIT(23)
#define  OB_TYPE_MEM         0x2U
#define  OB_TYPE_IO          0x6U
#define  OB_TYPE_CFG0        0xaU
#define  OB_TYPE_CFG1        0xbU
#define AT_IB_RP_ADDR0(r)    (AT_BASE + 0x800U + (r) * 8U)
#define AT_IB_RP_ADDR1(r)    (AT_BASE + 0x804U + (r) * 8U)

#define CFG_REGION_BITS      25U         /* 32 MiB: buses 0..31 */
#define OB_REGION            MB(1)
#define OB_REGIONS           32U
#define ROCKCHIP_VID         0x1d87U
#define CLASS_BRIDGE_PCI     0x060400U

#define LINK_TIMEOUT_MS      500
#define T_PVPERL_MS          100         /* power stable to PERST# release */
#define T_RRS_MS             100         /* PERST# release to config request */

enum region_type {
	REGION_MEM,
	REGION_IO,
	REGION_MAX,
};

struct region {
	uint64_t bus_start;
	uint64_t phys_start;
	uint64_t size;
	uint64_t allocation_offset;
};

struct rk_rc_config {
	DEVICE_MMIO_NAMED_ROM(apb);
	const struct pcie_ctrl_config *common;
	uintptr_t cfg_phys;
	size_t cfg_size;
	struct rk3399_pcie_port_cfg port;
	struct gpio_dt_spec perst;
	struct gpio_dt_spec slot_power;
};

struct rk_rc_data {
	DEVICE_MMIO_NAMED_RAM(apb);
	mm_reg_t core;
	mm_reg_t cfg;
	struct k_spinlock lock;
	uint32_t cfg_type;
	struct region regions[REGION_MAX];
	uint32_t ob_regions;
	bool active;
	bool hw_ready;
	bool link;
	int64_t link_up_ms;
	unsigned int scans;
};

#define DEV_CFG(_dev)  ((const struct rk_rc_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct rk_rc_data *)(_dev)->data)

static inline mm_reg_t apb(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, apb);
}

static inline uint32_t crd(const struct device *dev, uint32_t off)
{
	return sys_read32(((struct rk_rc_data *)dev->data)->core + off);
}

static inline void cwr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, ((struct rk_rc_data *)dev->data)->core + off);
}

static inline void crmw(const struct device *dev, uint32_t off, uint32_t mask, uint32_t v)
{
	cwr(dev, off, (crd(dev, off) & ~mask) | (v & mask));
}

static bool role_rc(void)
{
#if defined(CONFIG_PCIE_ROLE)
	return pcie_role_get() == PCIE_ROLE_RC;
#else
	return true;
#endif
}

static bool link_up(const struct device *dev)
{
	return ((struct rk_rc_data *)dev->data)->hw_ready && rk3399_pcie_link_up(apb(dev));
}

/* ------------------------------------------------------------------------ */
/* Configuration access                                                     */

static void cfg_region_type(const struct device *dev, uint32_t type)
{
	struct rk_rc_data *data = dev->data;

	if (data->cfg_type != type) {
		cwr(dev, AT_OB_DESC0(0), OB_DESC0_RC_TRANS | type);
		data->cfg_type = type;
	}
}

static bool bus_routes(const struct device *dev, pcie_bdf_t bdf, bool *type1)
{
	uint32_t bus_reg = crd(dev, PCIE_BUS_NUMBER * 4U);
	unsigned int bus = PCIE_BDF_TO_BUS(bdf);
	unsigned int sec = PCIE_BUS_SECONDARY_NUMBER(bus_reg);
	unsigned int sub = PCIE_BUS_SUBORDINATE_NUMBER(bus_reg);

	if (sec == 0 || bus < sec || bus > sub || bus >= BIT(CFG_REGION_BITS - 20U) ||
	    !link_up(dev)) {
		return false;
	}
	/* point to point: only device 0 directly below the root port */
	if (bus == sec && PCIE_BDF_TO_DEV(bdf) != 0) {
		return false;
	}
	*type1 = bus != sec;
	return true;
}

static inline uint32_t ecam_off(pcie_bdf_t bdf, unsigned int reg)
{
	return (PCIE_BDF_TO_BUS(bdf) << 20) | (PCIE_BDF_TO_DEV(bdf) << 15) |
	       (PCIE_BDF_TO_FUNC(bdf) << 12) | (reg << 2);
}

static uint32_t rk_rc_conf_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	struct rk_rc_data *data = dev->data;
	k_spinlock_key_t key;
	bool type1;
	uint32_t v;

	if (!data->hw_ready || reg >= 1024U) {
		return 0xffffffffU;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		return bdf == PCIE_BDF(0, 0, 0) ? crd(dev, reg * 4U) : 0xffffffffU;
	}
	if (!bus_routes(dev, bdf, &type1)) {
		return 0xffffffffU;
	}
	key = k_spin_lock(&data->lock);
	cfg_region_type(dev, type1 ? OB_TYPE_CFG1 : OB_TYPE_CFG0);
	v = sys_read32(data->cfg + ecam_off(bdf, reg));
	k_spin_unlock(&data->lock, key);
	return v;
}

static void rk_rc_conf_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg,
			     uint32_t v)
{
	struct rk_rc_data *data = dev->data;
	k_spinlock_key_t key;
	bool type1;

	if (!data->hw_ready || reg >= 1024U) {
		return;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		if (bdf == PCIE_BDF(0, 0, 0)) {
			cwr(dev, reg * 4U, v);
		}
		return;
	}
	if (!bus_routes(dev, bdf, &type1)) {
		return;
	}
	key = k_spin_lock(&data->lock);
	cfg_region_type(dev, type1 ? OB_TYPE_CFG1 : OB_TYPE_CFG0);
	sys_write32(v, data->cfg + ecam_off(bdf, reg));
	k_spin_unlock(&data->lock, key);
}

/* ------------------------------------------------------------------------ */
/* BAR space                                                                */

static bool rk_rc_region_allocate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				  bool mem64, size_t bar_size, uintptr_t *bar_bus_addr)
{
	struct rk_rc_data *data = dev->data;
	struct region *r = &data->regions[mem ? REGION_MEM : REGION_IO];
	uint64_t addr;

	ARG_UNUSED(bdf);
	ARG_UNUSED(mem64);
	if (r->size == 0) {
		return false;
	}
	addr = ((r->bus_start + r->allocation_offset - 1) | (bar_size - 1)) + 1;
	if (addr - r->bus_start + bar_size > r->size) {
		return false;
	}
	*bar_bus_addr = addr;
	r->allocation_offset = addr - r->bus_start + bar_size;
	return true;
}

static bool rk_rc_region_get_allocate_base(const struct device *dev, pcie_bdf_t bdf, bool mem,
					   bool mem64, size_t align, uintptr_t *bar_base_addr)
{
	struct rk_rc_data *data = dev->data;
	struct region *r = &data->regions[mem ? REGION_MEM : REGION_IO];

	ARG_UNUSED(bdf);
	/* one 32-bit memory window: bridges' prefetchable windows stay closed */
	if ((mem && mem64) || r->size == 0) {
		return false;
	}
	*bar_base_addr = ((r->bus_start + r->allocation_offset - 1) | (align - 1)) + 1;
	r->allocation_offset = *bar_base_addr - r->bus_start;
	return true;
}

static bool rk_rc_region_translate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				   bool mem64, uintptr_t bar_bus_addr, uintptr_t *bar_addr)
{
	struct rk_rc_data *data = dev->data;
	struct region *r = &data->regions[mem ? REGION_MEM : REGION_IO];

	ARG_UNUSED(bdf);
	ARG_UNUSED(mem64);
	if (bar_bus_addr == 0) {
		return false;
	}
	*bar_addr = r->phys_start + (bar_bus_addr - r->bus_start);
	return true;
}

static DEVICE_API(pcie_ctrl, rk_rc_api) = {
	.conf_read = rk_rc_conf_read,
	.conf_write = rk_rc_conf_write,
	.region_allocate = rk_rc_region_allocate,
	.region_get_allocate_base = rk_rc_region_get_allocate_base,
	.region_translate = rk_rc_region_translate,
};

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                 */

static int parse_ranges(const struct device *dev)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	uint64_t win = cfg->cfg_phys + BIT64(CFG_REGION_BITS);

	memset(data->regions, 0, sizeof(data->regions));
	for (size_t i = 0; i < cfg->common->ranges_count; i++) {
		const uint32_t sp = (cfg->common->ranges[i].flags >> 24) & 0x3;
		struct region *r;

		if (sp == 0) {
			continue;
		}
		r = &data->regions[sp == 1 ? REGION_IO : REGION_MEM];
		r->bus_start = cfg->common->ranges[i].pcie_bus_addr;
		r->phys_start = cfg->common->ranges[i].host_map_addr;
		r->size = cfg->common->ranges[i].map_length;
		if (r->phys_start < win || r->phys_start + r->size > win + OB_REGIONS * OB_REGION ||
		    ((r->phys_start | r->size) & (OB_REGION - 1)) != 0) {
			LOG_ERR("range 0x%llx+0x%llx outside the 1 MiB regions at 0x%llx",
				(unsigned long long)r->phys_start, (unsigned long long)r->size,
				(unsigned long long)win);
			return -EINVAL;
		}
	}
	return data->regions[REGION_MEM].size != 0 ? 0 : -EINVAL;
}

static void ob_program(const struct device *dev, uint32_t region, uint64_t pci, uint32_t type)
{
	cwr(dev, AT_OB_ADDR0(region), (19U & 0x3fU) | ((uint32_t)pci & 0xffffff00U));
	cwr(dev, AT_OB_ADDR1(region), (uint32_t)(pci >> 32));
	cwr(dev, AT_OB_DESC0(region), OB_DESC0_RC_TRANS | type);
	cwr(dev, AT_OB_DESC1(region), 0);
}

static void atu_setup(const struct device *dev)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	uint64_t win = cfg->cfg_phys + BIT64(CFG_REGION_BITS);

	/* root port BARs off; region 0: configuration requests */
	cwr(dev, LM_RC_BAR_CONF, 0);
	cwr(dev, AT_OB_ADDR0(0), CFG_REGION_BITS - 1U);
	cwr(dev, AT_OB_ADDR1(0), 0);
	cwr(dev, AT_OB_DESC0(0), OB_DESC0_RC_TRANS | OB_TYPE_CFG0);
	cwr(dev, AT_OB_DESC1(0), 0);
	data->cfg_type = OB_TYPE_CFG0;

	data->ob_regions = 0;
	for (int t = 0; t < REGION_MAX; t++) {
		const struct region *r = &data->regions[t];

		for (uint64_t off = 0; off < r->size; off += OB_REGION) {
			uint32_t region = 1U + (uint32_t)((r->phys_start + off - win) / OB_REGION);

			ob_program(dev, region, r->bus_start + off,
				   t == REGION_IO ? OB_TYPE_IO : OB_TYPE_MEM);
			data->ob_regions |= BIT(region - 1U);
		}
	}

	/* inbound: everything (32 bits passed) to address 0 */
	cwr(dev, AT_IB_RP_ADDR0(2), 32U - 1U);
	cwr(dev, AT_IB_RP_ADDR1(2), 0);
}

static void perst(const struct device *dev, bool assert)
{
	const struct rk_rc_config *cfg = dev->config;

	if (cfg->perst.port != NULL) {
		(void)gpio_pin_set_dt(&cfg->perst, assert ? 1 : 0);
	}
}

static int hw_setup(const struct device *dev)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	int ret;

	data->hw_ready = false;
	perst(dev, true);
	ret = rk3399_pcie_port_init(&cfg->port, apb(dev), true);
	if (ret != 0) {
		LOG_ERR("%s: port init failed: %d", dev->name, ret);
		return ret;
	}
	rk_client_write(apb(dev), RK_CLIENT_CONFIG, RK_WM16(RK_CLIENT_CONF_ENABLE, 1U));

	/* FTS count to leave L0s */
	crmw(dev, LM_PLC1, LM_PLC1_FTS, FIELD_PREP(LM_PLC1_FTS, 0xffffU));
	/* common clock, RCB 128 */
	cwr(dev, RP_LCS, crd(dev, RP_LCS) | LNKSTA_SLC | LNKCTL_RCB);
	/* identity of a PCI-PCI bridge */
	cwr(dev, LM_VENDOR, ROCKCHIP_VID);
	cwr(dev, RP_RID_CCR, (CLASS_BRIDGE_PCI << 8) | (crd(dev, RP_RID_CCR) & 0xffU));
	/* hide L1 substates (next pointer after the TPH capability) */
	crmw(dev, RP_THP_CAP, THP_CAP_NEXT, 0);
	/* MPS 256 */
	crmw(dev, RP_DCSR, RP_DCSR_MPS, FIELD_PREP(RP_DCSR_MPS, 1U));

	atu_setup(dev);
	data->hw_ready = true;
	return 0;
}

static bool link_start(const struct device *dev)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	unsigned int gen, width;
	int64_t t0;

	perst(dev, true);
	rk_client_write(apb(dev), RK_CLIENT_CONFIG, RK_WM16(RK_CLIENT_LINK_TRAIN, 0U));
	k_msleep(10);
	rk_client_write(apb(dev), RK_CLIENT_CONFIG, RK_WM16(RK_CLIENT_LINK_TRAIN, 1U));
	k_msleep(T_PVPERL_MS);
	perst(dev, false);
	k_msleep(T_RRS_MS);

	t0 = k_uptime_get();
	while (!rk3399_pcie_link_up(apb(dev))) {
		if (k_uptime_get() - t0 > LINK_TIMEOUT_MS) {
			uint8_t s = rk3399_pcie_ltssm(apb(dev));

			LOG_WRN("%s: no link (ltssm 0x%02x %s)", dev->name, s,
				rk3399_pcie_ltssm_name(s));
			data->link = false;
			return false;
		}
		k_msleep(1);
	}

	/* trained at 2.5 GT/s; Gen2 by retraining */
	if (cfg->port.link_gen > 1U) {
		cwr(dev, RP_LCS, crd(dev, RP_LCS) | LNKCTL_RL);
		t0 = k_uptime_get();
		while (FIELD_GET(LM_CTRL_SPEED, crd(dev, LM_CTRL)) != LM_CTRL_SPEED_5G) {
			if (k_uptime_get() - t0 > LINK_TIMEOUT_MS) {
				LOG_INF("%s: Gen2 retrain failed, staying at Gen1", dev->name);
				break;
			}
			k_msleep(1);
		}
	}
	data->link_up_ms = k_uptime_get();
	data->link = true;
	rk3399_pcie_link_info(apb(dev), &gen, &width);
	LOG_INF("%s: link up, Gen%u x%u (LM lanes x%lu)", dev->name, gen, width,
		BIT(FIELD_GET(LM_CTRL_LANE, crd(dev, LM_CTRL))));
	return true;
}

static void rk_rc_scan(const struct device *dev)
{
	struct rk_rc_data *data = dev->data;

	for (int t = 0; t < REGION_MAX; t++) {
		data->regions[t].allocation_offset = 0;
	}
	/* fresh bus numbers: generic enumeration assigns them again */
	cwr(dev, PCIE_BUS_NUMBER * 4U, 0);
	pcie_generic_ctrl_enumerate(dev, PCIE_BDF(0, 0, 0));
	(void)pcie_dev_scan();
	data->scans++;
}

#if defined(CONFIG_PCIE_CTRL_SHELL)
static const struct pcie_shell_ctrl rc_shell;
#endif

static int rk_rc_init(const struct device *dev)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	int ret;

#if defined(CONFIG_PCIE_CTRL_SHELL)
	pcie_shell_register(&rc_shell);
#endif
	if (!role_rc()) {
		LOG_INF("%s: endpoint role this boot, root complex off", dev->name);
		return 0; /* stays inactive; the shell reports it */
	}

	ret = parse_ranges(dev);
	if (ret != 0) {
		return ret;
	}
	DEVICE_MMIO_NAMED_MAP(dev, apb, K_MEM_CACHE_NONE);
	data->core = apb(dev) + RK_CORE_OFFSET;
	device_map(&data->cfg, cfg->cfg_phys, cfg->cfg_size, K_MEM_CACHE_NONE);

	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}
		(void)gpio_pin_configure_dt(&cfg->perst, GPIO_OUTPUT_ACTIVE);
	}
	if (cfg->slot_power.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->slot_power)) {
			return -ENODEV;
		}
		(void)gpio_pin_configure_dt(&cfg->slot_power, GPIO_OUTPUT_ACTIVE);
	}

	ret = rk3399_pcie_port_prepare(&cfg->port);
	if (ret == 0) {
		ret = hw_setup(dev);
	}
	if (ret != 0) {
		return ret;
	}
	data->active = true;
	if (link_start(dev)) {
		rk_rc_scan(dev);
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
/* pcie shell backend                                                       */

#if defined(CONFIG_PCIE_CTRL_SHELL)
static bool sh_active(const struct device *dev)
{
	ARG_UNUSED(dev);
	return role_rc();
}

static void sh_info(const struct device *dev, const struct shell *sh)
{
	const struct rk_rc_config *cfg = dev->config;
	struct rk_rc_data *data = dev->data;
	uint32_t bus;
	uint8_t s;

	if (!data->active) {
		shell_print(sh, "root complex idle (endpoint role this boot)");
		return;
	}
	s = rk3399_pcie_ltssm(apb(dev));
	shell_print(sh, "ltssm      0x%02x %s, link %s", s, rk3399_pcie_ltssm_name(s),
		    link_up(dev) ? "up" : "down");
	if (link_up(dev)) {
		unsigned int gen, width;

		rk3399_pcie_link_info(apb(dev), &gen, &width);
		shell_print(sh, "link       Gen%u x%u (of x%u, max Gen%u), up for %lld ms", gen,
			    width, cfg->port.lanes, cfg->port.link_gen,
			    k_uptime_get() - data->link_up_ms);
	}
	bus = crd(dev, PCIE_BUS_NUMBER * 4U);
	shell_print(sh, "root port  %08x class %08x, buses %u..%u, scans %u", crd(dev, 0),
		    crd(dev, 8), (unsigned int)PCIE_BUS_SECONDARY_NUMBER(bus),
		    (unsigned int)PCIE_BUS_SUBORDINATE_NUMBER(bus),
		    data->scans);
	shell_print(sh, "lnkctl/sta %08x  devctl/sta %08x", crd(dev, RP_LCS), crd(dev, RP_DCSR));
	shell_print(sh, "config     0x%lx (region 0, type %x)", (unsigned long)cfg->cfg_phys,
		    data->cfg_type);
	for (int t = 0; t < REGION_MAX; t++) {
		const struct region *r = &data->regions[t];

		if (r->size != 0) {
			shell_print(sh, "%-10s 0x%llx -> pci 0x%llx, 0x%llx bytes, used 0x%llx",
				    t == REGION_IO ? "io" : "mem",
				    (unsigned long long)r->phys_start,
				    (unsigned long long)r->bus_start, (unsigned long long)r->size,
				    (unsigned long long)r->allocation_offset);
		}
	}
	shell_print(sh, "ob regions %08x (bit n = region n+1)", data->ob_regions);
}

static int sh_link(const struct device *dev, const struct shell *sh, bool retrain)
{
	struct rk_rc_data *data = dev->data;

	if (!data->active) {
		return -EBUSY;
	}
	if (retrain) {
		if (link_start(dev)) {
			rk_rc_scan(dev);
		}
	}
	shell_print(sh, "link %s, ltssm %s", link_up(dev) ? "up" : "down",
		    rk3399_pcie_ltssm_name(rk3399_pcie_ltssm(apb(dev))));
	return 0;
}

static int sh_scan(const struct device *dev, const struct shell *sh)
{
	struct rk_rc_data *data = dev->data;

	if (!data->active) {
		return -EBUSY;
	}
	if (!link_up(dev)) {
		shell_print(sh, "no link");
		return -ENOTCONN;
	}
	rk_rc_scan(dev);
	return 0;
}

static int sh_id_get(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id)
{
	struct rk_rc_data *data = dev->data;
	uint32_t v;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -EINVAL;
	}
	v = crd(dev, 0);
	*id = (struct pcie_shell_id){
		.vid = v & 0xffff,
		.did = v >> 16,
		.class_code = crd(dev, 8) >> 8,
	};
	return 0;
}

/* applied now, not kept */
static int sh_id_set(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id)
{
	struct rk_rc_data *data = dev->data;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -EINVAL;
	}
	cwr(dev, LM_VENDOR, id->vid);
	cwr(dev, RP_ID, ((uint32_t)id->did << 16) | id->vid);
	if (id->class_code != UINT32_MAX) {
		cwr(dev, RP_RID_CCR, (id->class_code << 8) | (crd(dev, RP_RID_CCR) & 0xffU));
	}
	return 0;
}

#if defined(CONFIG_PCIE_ROLE)
static int sh_mode_set(const struct device *dev, enum pcie_shell_role role)
{
	ARG_UNUSED(dev);
	return pcie_role_save(role == PCIE_SHELL_ROLE_RC ? PCIE_ROLE_RC : PCIE_ROLE_EP);
}

static bool sh_mode_saved(const struct device *dev, enum pcie_shell_role *role)
{
	enum pcie_role r;

	ARG_UNUSED(dev);
	if (!pcie_role_saved(&r)) {
		return false;
	}
	*role = r == PCIE_ROLE_RC ? PCIE_SHELL_ROLE_RC : PCIE_SHELL_ROLE_EP;
	return true;
}
#endif

static const struct pcie_shell_ctrl rc_shell = {
	.name = "rk3399",
	.dev = DEVICE_DT_INST_GET(0),
	.role = PCIE_SHELL_ROLE_RC,
	.active = sh_active,
	.info = sh_info,
#if defined(CONFIG_PCIE_ROLE)
	.mode_set = sh_mode_set,
	.mode_saved = sh_mode_saved,
#endif
	.link = sh_link,
	.scan = sh_scan,
	.id_get = sh_id_get,
	.id_set = sh_id_set,
	.cfg_read = rk_rc_conf_read,
	.cfg_write = rk_rc_conf_write,
};
#endif /* CONFIG_PCIE_CTRL_SHELL */

#define RK_CLK(n, name)                                                                            \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, name)),                        \
		.id = DT_INST_CLOCKS_CELL_BY_NAME(n, name, clkid),                                 \
	}

#define RK_RST(n, name)                                                                            \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(n, name)),                         \
		.id = DT_INST_RESET_CELL_BY_NAME(n, name, id),                                     \
	}

#define RK_RC_DEFINE(n)                                                                            \
	static const struct pcie_ctrl_config rk_rc_ctrl_cfg_##n = {                                \
		.cfg_addr = DT_INST_REG_ADDR_BY_NAME(n, cfg),                                 \
		.cfg_size = DT_INST_REG_SIZE_BY_NAME(n, cfg),                                 \
		.ranges_count = DT_NUM_RANGES(DT_DRV_INST(n)),                                     \
		.ranges = {DT_FOREACH_RANGE(DT_DRV_INST(n), PCIE_RANGE_FORMAT)},                   \
	};                                                                                         \
	static const struct rk_rc_config rk_rc_cfg_##n = {                                         \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(apb, DT_DRV_INST(n)),                           \
		.common = &rk_rc_ctrl_cfg_##n,                                                     \
		.cfg_phys = DT_INST_REG_ADDR_BY_NAME(n, cfg),                                 \
		.cfg_size = DT_INST_REG_SIZE_BY_NAME(n, cfg),                                 \
		.port = {                                                                          \
			.clks = {                                                                  \
				[RK_CLK_ACLK] = RK_CLK(n, aclk),                                   \
				[RK_CLK_ACLK_PERF] = RK_CLK(n, aclk_perf),                         \
				[RK_CLK_HCLK] = RK_CLK(n, hclk),                                   \
				[RK_CLK_PM] = RK_CLK(n, pm),                                       \
			},                                                                         \
			.rsts = {                                                                  \
				[RK_RST_CORE] = RK_RST(n, core),                                   \
				[RK_RST_MGMT] = RK_RST(n, mgmt),                                   \
				[RK_RST_MGMT_STICKY] = RK_RST(n, mgmt_sticky),                     \
				[RK_RST_PIPE] = RK_RST(n, pipe),                                   \
				[RK_RST_PM] = RK_RST(n, pm),                                       \
				[RK_RST_PCLK] = RK_RST(n, pclk),                                   \
				[RK_RST_ACLK] = RK_RST(n, aclk),                                   \
				[RK_RST_PHY] = RK_RST(n, phy),                                     \
			},                                                                         \
			.grf = DEVICE_DT_GET(DT_INST_PHANDLE(n, rockchip_grf)),                    \
			.lanes = DT_INST_PROP(n, num_lanes),                                       \
			.link_gen = DT_INST_PROP(n, max_link_speed),                               \
		},                                                                                 \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.slot_power = GPIO_DT_SPEC_INST_GET_OR(n, slot_power_gpios, {0}),                  \
	};                                                                                         \
	static struct rk_rc_data rk_rc_data_##n;                                                   \
	DEVICE_DT_INST_DEFINE(n, rk_rc_init, NULL, &rk_rc_data_##n, &rk_rc_cfg_##n, POST_KERNEL,   \
			      CONFIG_PCIE_RK3399_INIT_PRIORITY, &rk_rc_api);

DT_INST_FOREACH_STATUS_OKAY(RK_RC_DEFINE)
