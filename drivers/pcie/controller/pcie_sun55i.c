/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Allwinner A523/A527/T527 PCIe controller as root complex.
 *
 * Synopsys DesignWare core (16c3:abcd), root complex only, PCIe 2.1 x1
 * through the INNO PCIe/USB 3.0 combo PHY. The root port is the DWC's own
 * type 1 header in the DBI; downstream configuration requests go through
 * outbound iATU region 0, retargeted (bus << 24 | dev << 19 | fn << 16,
 * CFG0 or CFG1) per access. Region 1 decodes the memory window, region 2
 * the I/O window from "ranges". The iATU sees AXI addresses relative to
 * the controller's 256 MiB window at 0x20000000. Inbound requests pass
 * untranslated (no inbound region): bus address = CPU physical address.
 *
 * The SoC has no clock, reset or power domain drivers yet, so bring-up is
 * done here: PCK-600 PCIe power domain, CCU reference/AUX clocks and the
 * sub-system reset, combo sub-system clocks and resets, the PHY's 100 MHz
 * internal reference setup, root complex device type, LTSSM. Sequence
 * after the Allwinner BSP (pcie-sunxi-rc.c, sunxi-inno-combophy.c) and
 * Linux drivers/pmdomain/sunxi/sun55i-pck600.c.
 *
 * Not yet: INTx/MSI (the DWC MSI controller on "msi"), eDMA, ASPM.
 */

#define DT_DRV_COMPAT allwinner_sun55i_a523_pcie

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/pcie.h>
#if defined(CONFIG_PCIE_CTRL_SHELL)
#include <zephyr/drivers/pcie/pcie_shell.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pcie_sun55i, LOG_LEVEL_INF);

/* SoC blocks the driver programs directly */
#define CCU_PHYS             0x02001000U
#define CCU_MBUS_GATE        0x804U
#define  CCU_MBUS_USB3       BIT(6)
#define CCU_USB3_REF_CLK     0xa84U      /* [31] gate, [26:24] src, [4:0] M */
#define  CCU_REF_PERI0_200M  (1U << 24)
#define CCU_PCIE_AUX_CLK     0xaa0U      /* [31] gate, [24] 0 = HOSC, [4:0] M */
#define CCU_PCIE_BGR         0xaacU      /* [16] sub-system reset, 1 = released */
#define  CCU_PCIE_RST        BIT(16)

#define R_CCU_PHYS           0x07010000U
#define R_CCU_PPU_BGR        0x1acU      /* [0] PPU0 gate, [16] PPU0 reset */

#define PPU_PHYS             0x07060000U
#define PPU_PD_PCIE          7U
#define PPU_PWPR             0x000U
#define PPU_PWSR             0x008U
#define PPU_DCDR0            0x170U
#define PPU_DCDR1            0x174U
#define PPU_LPS0_DELAY       0xc00U
#define PPU_LPS1_DELAY       0xc04U
#define PPU_OFF2ON           0xc10U
#define PPU_MODE             GENMASK(3, 0)
#define PPU_MODE_ON          0x8U

/* "combo": sub-system at 0, INNO PHY at 0x80000 */
#define COMBO_PCIEBGR        0x004U
#define  PCIEBGR_ACLK        BIT(17)
#define  PCIEBGR_HCLK        BIT(16)
#define  PCIEBGR_PERSTN      BIT(1)
#define  PCIEBGR_PWRUP_RSTN  BIT(0)
#define COMBO_PHYCTL         0x010U
#define  PHYCTL_USB3         BIT(31)
#define  PHYCTL_EXT_REFCLK   BIT(30)
#define  PHYCTL_PIPE_RST     GENMASK(9, 8)
#define  PHYCTL_SYS_RSTN     BIT(1)
#define  PHYCTL_PHY_RSTN     BIT(0)
#define COMBO_PHYSTS0        0x080U
#define PHY_OFFSET           0x80000U
#define PHY_SIZE             0x2000U

/* DBI: DWC configuration header and port logic */
#define DBI_CMD              0x004U
#define DBI_CLASS            0x008U
#define DBI_BAR0             0x010U
#define DBI_BAR1             0x014U
#define DBI_BUS              0x018U
#define DBI_PREF_BASE_LIMIT  0x024U
#define DBI_PREF_BASE_UPPER  0x028U
#define DBI_PREF_LIMIT_UPPER 0x02cU
#define DBI_CAP_PTR          0x034U
#define CAP_ID_EXP           0x10U
#define EXP_LNKCAP           0x0cU
#define EXP_LNKCTL           0x10U
#define  LNKSTA_SPEED        GENMASK(19, 16)
#define  LNKSTA_WIDTH        GENMASK(25, 20)
#define EXP_LNKCTL2          0x30U
#define  LNKCTL2_TLS         GENMASK(3, 0)
#define DBI_PORT_LINK_CTRL   0x710U
#define  LINK_MODE           GENMASK(21, 16)
#define DBI_PL_DEBUG0        0x728U
#define  PL_LTSSM            GENMASK(5, 0)
#define  LTSSM_RCVRY_LOCK    0x0dU
#define  LTSSM_L0            0x11U
#define  LTSSM_L2_WAKE       0x16U
#define DBI_WIDTH_SPEED_CTRL 0x80cU
#define  SPEED_CHANGE        BIT(17)
#define  LINK_WIDTH          GENMASK(16, 8)
#define DBI_MISC_CTRL1       0x8bcU
#define  DBI_RO_WR_EN        BIT(0)

/* unrolled iATU */
#define ATU_BASE             0x300000U
#define ATU_OB(r, off)       (ATU_BASE + (r) * 0x200U + (off))
#define ATU_CTRL1            0x00U
#define  ATU_TYPE_MEM        0x0U
#define  ATU_TYPE_IO         0x2U
#define  ATU_TYPE_CFG0       0x4U
#define  ATU_TYPE_CFG1       0x5U
#define ATU_CTRL2            0x04U
#define  ATU_ENABLE          BIT(31)
#define ATU_LOWER_BASE       0x08U
#define ATU_UPPER_BASE       0x0cU
#define ATU_LIMIT            0x10U
#define ATU_LOWER_TARGET     0x14U
#define ATU_UPPER_TARGET     0x18U
#define ATU_CFG_SIZE         0x1000U
#define ATU_REGION_CFG       0U
#define ATU_REGION_MEM       1U
#define ATU_REGION_IO        2U

/* Allwinner application registers */
#define APP_BASE             0x400000U
#define APP_LTSSM_CTRL       (APP_BASE + 0xc00U)
#define  LTSSM_ENABLE        BIT(0)
#define  DEVICE_TYPE         GENMASK(7, 4)
#define  DEVICE_TYPE_RC      BIT(6)
#define APP_INT_ENABLE_CLR   (APP_BASE + 0xe04U)
#define APP_LINK_STAT        (APP_BASE + 0xe0cU)
#define  RDLH_LINK_UP        BIT(1)
#define  SMLH_LINK_UP        BIT(0)

/* the controller's AXI slave window; the iATU sees offsets into it */
#define AXI_WINDOW           0x20000000U
#define AXI_WINDOW_SIZE      0x10000000U

#define CLASS_BRIDGE_PCI     0x060400U
#define LINK_TIMEOUT_MS      500
#define SPEED_TIMEOUT_MS     100
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

struct sun55i_rc_config {
	DEVICE_MMIO_NAMED_ROM(dbi);
	const struct pcie_ctrl_config *common;
	uintptr_t cfg_phys;
	uintptr_t combo_phys;
	uint8_t lanes;
	uint8_t link_gen;
	struct gpio_dt_spec perst;
	struct gpio_dt_spec slot_power;
};

struct sun55i_rc_data {
	DEVICE_MMIO_NAMED_RAM(dbi);
	mm_reg_t cfg;
	mm_reg_t combo;
	mm_reg_t phy;
	mm_reg_t ccu;
	mm_reg_t r_ccu;
	mm_reg_t ppu;
	struct k_spinlock lock;
	struct region regions[REGION_MAX];
	uint8_t exp_cap;
	uint32_t cfg_target;
	uint32_t cfg_type;
	bool active;
	bool link;
	int64_t link_up_ms;
	unsigned int scans;
	/* link monitor */
	const struct device *dev;
	struct k_mutex op_lock;
	struct k_work_delayable mon_work;
	uint8_t up_polls;
	unsigned int link_downs;
	unsigned int core_resets;
};

#define DEV_CFG(_dev)  ((const struct sun55i_rc_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct sun55i_rc_data *)(_dev)->data)

static inline mm_reg_t dbi(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, dbi);
}

static inline uint32_t drd(const struct device *dev, uint32_t off)
{
	return sys_read32(dbi(dev) + off);
}

static inline void dwr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, dbi(dev) + off);
}

static inline void drmw(const struct device *dev, uint32_t off, uint32_t mask, uint32_t v)
{
	dwr(dev, off, (drd(dev, off) & ~mask) | (v & mask));
}

static inline void rmw(mm_reg_t base, uint32_t off, uint32_t clr, uint32_t set)
{
	sys_write32((sys_read32(base + off) & ~clr) | set, base + off);
}

static uint8_t ltssm(const struct device *dev)
{
	return FIELD_GET(PL_LTSSM, drd(dev, DBI_PL_DEBUG0));
}

/*
 * Both link up bits, and the LTSSM in recovery or a link state: the
 * status bits stay set for a while when the partner drops out and the
 * LTSSM falls back to polling.
 */
static bool link_up(const struct device *dev)
{
	const struct sun55i_rc_data *data = dev->data;
	uint32_t v;
	uint8_t s;

	if (!data->active) {
		return false;
	}
	v = drd(dev, APP_LINK_STAT);
	if ((v & (RDLH_LINK_UP | SMLH_LINK_UP)) != (RDLH_LINK_UP | SMLH_LINK_UP)) {
		return false;
	}
	s = ltssm(dev);
	return s >= LTSSM_RCVRY_LOCK && s <= LTSSM_L2_WAKE;
}

static const char *ltssm_name(uint8_t s)
{
	static const char *const names[] = {
		[0x00] = "detect.quiet", [0x01] = "detect.act",
		[0x02] = "poll.active", [0x03] = "poll.compliance",
		[0x04] = "poll.config", [0x05] = "pre.detect.quiet",
		[0x06] = "detect.wait", [0x07] = "cfg.linkwd.start",
		[0x08] = "cfg.linkwd.accept", [0x09] = "cfg.lanenum.wait",
		[0x0a] = "cfg.lanenum.accept", [0x0b] = "cfg.complete",
		[0x0c] = "cfg.idle", [0x0d] = "rcvry.lock",
		[0x0e] = "rcvry.speed", [0x0f] = "rcvry.rcvrcfg",
		[0x10] = "rcvry.idle", [0x11] = "L0",
		[0x12] = "L0s", [0x13] = "L123.send.eidle",
		[0x14] = "L1.idle", [0x15] = "L2.idle",
		[0x16] = "L2.wake", [0x17] = "disabled.entry",
		[0x18] = "disabled.idle", [0x19] = "disabled",
		[0x1a] = "lpbk.entry", [0x1b] = "lpbk.active",
		[0x1c] = "lpbk.exit", [0x1d] = "lpbk.exit.timeout",
		[0x1e] = "hot.reset.entry", [0x1f] = "hot.reset",
	};

	return s < ARRAY_SIZE(names) && names[s] != NULL ? names[s] : "?";
}

static void link_info(const struct device *dev, unsigned int *gen, unsigned int *width)
{
	const struct sun55i_rc_data *data = dev->data;
	uint32_t v = drd(dev, data->exp_cap + EXP_LNKCTL);

	*gen = FIELD_GET(LNKSTA_SPEED, v);
	*width = FIELD_GET(LNKSTA_WIDTH, v);
}

/* ------------------------------------------------------------------------ */
/* iATU                                                                     */

static void atu_ob(const struct device *dev, uint32_t r, uint32_t type, uint64_t cpu,
		   uint64_t pci, uint64_t size)
{
	uint64_t axi = cpu - AXI_WINDOW;

	dwr(dev, ATU_OB(r, ATU_LOWER_BASE), (uint32_t)axi);
	dwr(dev, ATU_OB(r, ATU_UPPER_BASE), (uint32_t)(axi >> 32));
	dwr(dev, ATU_OB(r, ATU_LIMIT), (uint32_t)(axi + size - 1U));
	dwr(dev, ATU_OB(r, ATU_LOWER_TARGET), (uint32_t)pci);
	dwr(dev, ATU_OB(r, ATU_UPPER_TARGET), (uint32_t)(pci >> 32));
	dwr(dev, ATU_OB(r, ATU_CTRL1), type);
	dwr(dev, ATU_OB(r, ATU_CTRL2), ATU_ENABLE);
	/* the region is used right after: wait for the enable to land */
	(void)drd(dev, ATU_OB(r, ATU_CTRL2));
}

static void cfg_target(const struct device *dev, uint32_t target, uint32_t type)
{
	struct sun55i_rc_data *data = dev->data;

	if (data->cfg_target != target) {
		dwr(dev, ATU_OB(ATU_REGION_CFG, ATU_LOWER_TARGET), target);
		data->cfg_target = target;
	}
	if (data->cfg_type != type) {
		dwr(dev, ATU_OB(ATU_REGION_CFG, ATU_CTRL1), type);
		data->cfg_type = type;
	}
	(void)drd(dev, ATU_OB(ATU_REGION_CFG, ATU_CTRL1));
}

/* ------------------------------------------------------------------------ */
/* Configuration access                                                     */

static bool bus_routes(const struct device *dev, pcie_bdf_t bdf, uint32_t *type)
{
	uint32_t bus_reg = drd(dev, DBI_BUS);
	unsigned int bus = PCIE_BDF_TO_BUS(bdf);
	unsigned int sec = PCIE_BUS_SECONDARY_NUMBER(bus_reg);
	unsigned int sub = PCIE_BUS_SUBORDINATE_NUMBER(bus_reg);

	if (sec == 0 || bus < sec || bus > sub || !link_up(dev)) {
		return false;
	}
	/* point to point: only device 0 directly below the root port */
	if (bus == sec && PCIE_BDF_TO_DEV(bdf) != 0) {
		return false;
	}
	*type = bus == sec ? ATU_TYPE_CFG0 : ATU_TYPE_CFG1;
	return true;
}

static inline uint32_t cfg_bdf(pcie_bdf_t bdf)
{
	return (PCIE_BDF_TO_BUS(bdf) << 24) | (PCIE_BDF_TO_DEV(bdf) << 19) |
	       (PCIE_BDF_TO_FUNC(bdf) << 16);
}

static uint32_t sun55i_rc_conf_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	struct sun55i_rc_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t type;
	uint32_t v;

	if (!data->active || reg >= 1024U) {
		return 0xffffffffU;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		return bdf == PCIE_BDF(0, 0, 0) ? drd(dev, reg * 4U) : 0xffffffffU;
	}
	if (!bus_routes(dev, bdf, &type)) {
		return 0xffffffffU;
	}
	key = k_spin_lock(&data->lock);
	cfg_target(dev, cfg_bdf(bdf), type);
	v = sys_read32(data->cfg + reg * 4U);
	k_spin_unlock(&data->lock, key);
	return v;
}

static void sun55i_rc_conf_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg,
				 uint32_t v)
{
	struct sun55i_rc_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t type;

	if (!data->active || reg >= 1024U) {
		return;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		if (bdf == PCIE_BDF(0, 0, 0)) {
			dwr(dev, reg * 4U, v);
		}
		return;
	}
	if (!bus_routes(dev, bdf, &type)) {
		return;
	}
	key = k_spin_lock(&data->lock);
	cfg_target(dev, cfg_bdf(bdf), type);
	sys_write32(v, data->cfg + reg * 4U);
	k_spin_unlock(&data->lock, key);
}

/* ------------------------------------------------------------------------ */
/* BAR space                                                                */

static bool sun55i_rc_region_allocate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				      bool mem64, size_t bar_size, uintptr_t *bar_bus_addr)
{
	struct sun55i_rc_data *data = dev->data;
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

static bool sun55i_rc_region_get_allocate_base(const struct device *dev, pcie_bdf_t bdf,
					       bool mem, bool mem64, size_t align,
					       uintptr_t *bar_base_addr)
{
	struct sun55i_rc_data *data = dev->data;
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

static bool sun55i_rc_region_translate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				       bool mem64, uintptr_t bar_bus_addr, uintptr_t *bar_addr)
{
	struct sun55i_rc_data *data = dev->data;
	struct region *r = &data->regions[mem ? REGION_MEM : REGION_IO];

	ARG_UNUSED(bdf);
	ARG_UNUSED(mem64);
	if (bar_bus_addr == 0) {
		return false;
	}
	*bar_addr = r->phys_start + (bar_bus_addr - r->bus_start);
	return true;
}

static DEVICE_API(pcie_ctrl, sun55i_rc_api) = {
	.conf_read = sun55i_rc_conf_read,
	.conf_write = sun55i_rc_conf_write,
	.region_allocate = sun55i_rc_region_allocate,
	.region_get_allocate_base = sun55i_rc_region_get_allocate_base,
	.region_translate = sun55i_rc_region_translate,
};

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                 */

static int parse_ranges(const struct device *dev)
{
	const struct sun55i_rc_config *cfg = dev->config;
	struct sun55i_rc_data *data = dev->data;

	if (cfg->cfg_phys != AXI_WINDOW) {
		LOG_ERR("config window 0x%lx: must start the AXI window at 0x%x",
			(unsigned long)cfg->cfg_phys, AXI_WINDOW);
		return -EINVAL;
	}
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
		if (r->phys_start < AXI_WINDOW + cfg->common->cfg_size ||
		    r->phys_start + r->size > AXI_WINDOW + AXI_WINDOW_SIZE) {
			LOG_ERR("range 0x%llx+0x%llx outside the AXI window",
				(unsigned long long)r->phys_start, (unsigned long long)r->size);
			return -EINVAL;
		}
	}
	return data->regions[REGION_MEM].size != 0 ? 0 : -EINVAL;
}

/* PCIe domain of the PCK-600; usually already on after boot */
static int power_on(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;
	mm_reg_t pd = data->ppu;
	int64_t t0;

	rmw(data->r_ccu, R_CCU_PPU_BGR, 0, BIT(16) | BIT(0));
	if ((sys_read32(pd + PPU_PWSR) & PPU_MODE) == PPU_MODE_ON) {
		return 0;
	}
	sys_write32(0xffffffU, pd + PPU_DCDR0);
	sys_write32(0xffffU, pd + PPU_DCDR1);
	sys_write32(0x08080808U, pd + PPU_LPS0_DELAY);
	sys_write32(0x0808U, pd + PPU_LPS1_DELAY);
	sys_write32(0x8U, pd + PPU_OFF2ON);
	rmw(pd, PPU_PWPR, PPU_MODE, PPU_MODE_ON);
	t0 = k_uptime_get();
	while ((sys_read32(pd + PPU_PWSR) & PPU_MODE) != PPU_MODE_ON) {
		if (k_uptime_get() - t0 > 10) {
			LOG_ERR("%s: power domain stays off, PWSR %08x", dev->name,
				sys_read32(pd + PPU_PWSR));
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/* reference 100 MHz (PERI0_200M / 2), AUX from HOSC, MBUS gate, sub-system reset */
static void clocks_on(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;

	sys_write32(BIT(31) | CCU_REF_PERI0_200M | 1U, data->ccu + CCU_USB3_REF_CLK);
	rmw(data->ccu, CCU_PCIE_AUX_CLK, BIT(24) | GENMASK(4, 0), BIT(31));
	rmw(data->ccu, CCU_MBUS_GATE, 0, CCU_MBUS_USB3);
	/* a pulse: the controller starts from its reset state */
	rmw(data->ccu, CCU_PCIE_BGR, CCU_PCIE_RST, 0);
	k_busy_wait(10);
	rmw(data->ccu, CCU_PCIE_BGR, 0, CCU_PCIE_RST);
}

/* INNO combo PHY for PCIe, 100 MHz internal reference (BSP v1 sequence) */
static void phy_init(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;
	mm_reg_t phy = data->phy;

	rmw(phy, 0x1004, (0x3U << 3) | BIT(0), BIT(0) | BIT(2) | BIT(4));
	rmw(phy, 0x1018, 0x3U << 4, 0x3U << 4);
	rmw(phy, 0x101c, 0x0fffffffU, 0);
	rmw(phy, 0x107c, 0x3ffffU, (0x2U << 12) | 0x32U);
	rmw(phy, 0x1030, 0x3U << 20, 0);
	rmw(phy, 0x1050, 0x7U << 5, 0x1U << 5);
	rmw(phy, 0x1054, 0x7U << 5, 0x1U << 5);
	rmw(phy, 0x0804, 0xfU << 4, 0xcU << 4);
	rmw(phy, 0x109c, 0x3U << 8, BIT(1));
	sys_write32(0x80540a0aU, phy + 0x1418);

	/* PHY for PCIe, internal reference; assert, then release the resets */
	rmw(data->combo, COMBO_PHYCTL,
	    PHYCTL_USB3 | PHYCTL_EXT_REFCLK | PHYCTL_PIPE_RST | PHYCTL_SYS_RSTN | PHYCTL_PHY_RSTN,
	    0);
	rmw(data->combo, COMBO_PHYCTL, 0, PHYCTL_PHY_RSTN);
	rmw(data->combo, COMBO_PHYCTL, 0, PHYCTL_SYS_RSTN);
	k_busy_wait(100);
}

static uint8_t find_exp_cap(const struct device *dev)
{
	uint8_t ptr = drd(dev, DBI_CAP_PTR) & 0xfcU;

	for (int i = 0; i < 48 && ptr >= 0x40U; i++) {
		uint32_t v = drd(dev, ptr);

		if ((v & 0xffU) == CAP_ID_EXP) {
			return ptr;
		}
		ptr = (v >> 8) & 0xfcU;
	}
	return 0;
}

static void ltssm_enable(const struct device *dev, bool on)
{
	drmw(dev, APP_LTSSM_CTRL, LTSSM_ENABLE, on ? LTSSM_ENABLE : 0);
}

static void setup_rc(const struct device *dev)
{
	const struct sun55i_rc_config *cfg = dev->config;
	struct sun55i_rc_data *data = dev->data;
	const struct region *r;
	uint32_t v;

	dwr(dev, DBI_MISC_CTRL1, drd(dev, DBI_MISC_CTRL1) | DBI_RO_WR_EN);

	/* target link speed and lanes */
	drmw(dev, data->exp_cap + EXP_LNKCTL2, LNKCTL2_TLS, cfg->link_gen);
	drmw(dev, DBI_PORT_LINK_CTRL, LINK_MODE, FIELD_PREP(LINK_MODE, BIT(cfg->lanes) - 1U));
	drmw(dev, DBI_WIDTH_SPEED_CTRL, LINK_WIDTH, FIELD_PREP(LINK_WIDTH, cfg->lanes));

	/* root port BARs off, identity of a PCI-PCI bridge */
	dwr(dev, DBI_BAR0, 0);
	dwr(dev, DBI_BAR1, 0);
	v = drd(dev, DBI_CLASS);
	dwr(dev, DBI_CLASS, (CLASS_BRIDGE_PCI << 8) | (v & 0xffU));
	dwr(dev, DBI_CMD, (drd(dev, DBI_CMD) & 0xffff0000U) | PCIE_CONF_CMDSTAT_IO |
			      PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER | BIT(8));
	/*
	 * No prefetchable window (only one 32-bit memory range): base above
	 * limit, otherwise the reset value 0 forwards 0..0xfffff downstream.
	 */
	dwr(dev, DBI_PREF_BASE_LIMIT, 0x0000fff0U);
	dwr(dev, DBI_PREF_BASE_UPPER, 0);
	dwr(dev, DBI_PREF_LIMIT_UPPER, 0);

	dwr(dev, DBI_MISC_CTRL1, drd(dev, DBI_MISC_CTRL1) & ~DBI_RO_WR_EN);

	/* events are not used yet */
	dwr(dev, APP_INT_ENABLE_CLR, 0xffffffffU);

	/* outbound: configuration, memory, I/O */
	atu_ob(dev, ATU_REGION_CFG, ATU_TYPE_CFG0, cfg->cfg_phys, 0, ATU_CFG_SIZE);
	data->cfg_target = 0;
	data->cfg_type = ATU_TYPE_CFG0;
	r = &data->regions[REGION_MEM];
	atu_ob(dev, ATU_REGION_MEM, ATU_TYPE_MEM, r->phys_start, r->bus_start, r->size);
	r = &data->regions[REGION_IO];
	if (r->size != 0) {
		atu_ob(dev, ATU_REGION_IO, ATU_TYPE_IO, r->phys_start, r->bus_start, r->size);
	}
}

static int hw_setup(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;
	uint32_t id;
	int ret;

	ret = power_on(dev);
	if (ret != 0) {
		return ret;
	}
	clocks_on(dev);
	/* controller clocks on, its resets held until the device type is set */
	rmw(data->combo, COMBO_PCIEBGR, 0, PCIEBGR_ACLK | PCIEBGR_HCLK);
	drmw(dev, APP_LTSSM_CTRL, DEVICE_TYPE | LTSSM_ENABLE, DEVICE_TYPE_RC);
	rmw(data->combo, COMBO_PCIEBGR, 0, PCIEBGR_PERSTN | PCIEBGR_PWRUP_RSTN);
	phy_init(dev);

	id = drd(dev, 0);
	data->exp_cap = find_exp_cap(dev);
	if (id == 0 || id == 0xffffffffU || data->exp_cap == 0) {
		LOG_ERR("%s: core does not answer (id %08x, PHYSTS0 %08x)", dev->name, id,
			sys_read32(data->combo + COMBO_PHYSTS0));
		return -EIO;
	}
	setup_rc(dev);
	LOG_DBG("%s: core %08x, lnkcap %08x", dev->name, id,
		drd(dev, data->exp_cap + EXP_LNKCAP));
	return 0;
}

static void perst(const struct device *dev, bool assert)
{
	const struct sun55i_rc_config *cfg = dev->config;

	if (cfg->perst.port != NULL) {
		(void)gpio_pin_set_dt(&cfg->perst, assert ? 1 : 0);
	}
}

static void speed_change(const struct device *dev)
{
	const struct sun55i_rc_config *cfg = dev->config;
	struct sun55i_rc_data *data = dev->data;
	int64_t t0;

	if (cfg->link_gen < 2U) {
		return;
	}
	/* directed speed change: the DWC clears the bit when it is done */
	drmw(dev, DBI_WIDTH_SPEED_CTRL, SPEED_CHANGE, 0);
	drmw(dev, DBI_WIDTH_SPEED_CTRL, SPEED_CHANGE, SPEED_CHANGE);
	t0 = k_uptime_get();
	while ((drd(dev, DBI_WIDTH_SPEED_CTRL) & SPEED_CHANGE) != 0) {
		if (k_uptime_get() - t0 > SPEED_TIMEOUT_MS) {
			LOG_INF("%s: speed change timed out", dev->name);
			break;
		}
		k_msleep(1);
	}
	/* the new speed shows up after recovery */
	t0 = k_uptime_get();
	while (FIELD_GET(LNKSTA_SPEED, drd(dev, data->exp_cap + EXP_LNKCTL)) < 2U) {
		if (k_uptime_get() - t0 > SPEED_TIMEOUT_MS) {
			break;
		}
		k_msleep(1);
	}
}

static bool link_start(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;
	unsigned int gen, width;
	int64_t t0;

	data->link = false;
	ltssm_enable(dev, false);
	perst(dev, true);
	k_msleep(T_PVPERL_MS);
	perst(dev, false);
	ltssm_enable(dev, true);

	t0 = k_uptime_get();
	while (!link_up(dev)) {
		if (k_uptime_get() - t0 > LINK_TIMEOUT_MS) {
			uint8_t s = ltssm(dev);

			LOG_WRN("%s: no link (ltssm 0x%02x %s)", dev->name, s, ltssm_name(s));
			return false;
		}
		k_msleep(1);
	}
	speed_change(dev);
	/* the endpoint answers configuration requests T_RRS after PERST# */
	k_msleep(T_RRS_MS);

	data->link_up_ms = k_uptime_get();
	data->link = true;
	link_info(dev, &gen, &width);
	LOG_INF("%s: link up, Gen%u x%u", dev->name, gen, width);
	return true;
}

static void sun55i_rc_scan(const struct device *dev)
{
	struct sun55i_rc_data *data = dev->data;

	if (!link_up(dev)) {
		return;
	}

	for (int t = 0; t < REGION_MAX; t++) {
		data->regions[t].allocation_offset = 0;
	}
	/* fresh bus numbers: generic enumeration assigns them again */
	dwr(dev, DBI_BUS, drd(dev, DBI_BUS) & 0xff000000U);
	pcie_generic_ctrl_enumerate(dev, PCIE_BDF(0, 0, 0));
	(void)pcie_dev_scan();
	data->scans++;
}

/*
 * The core resets its non-sticky registers (root port header, iATU) when
 * the link goes down, and the endpoint loses its configuration: put the
 * root port back and enumerate again once the link is up in L0.
 */
static void mon_handler(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct sun55i_rc_data *data = CONTAINER_OF(dw, struct sun55i_rc_data, mon_work);
	const struct device *dev = data->dev;
	bool up, reset;

	if (k_mutex_lock(&data->op_lock, K_NO_WAIT) != 0) {
		goto out;
	}
	up = link_up(dev);
	reset = (drd(dev, ATU_OB(ATU_REGION_CFG, ATU_CTRL2)) & ATU_ENABLE) == 0 ||
		(data->link && PCIE_BUS_SECONDARY_NUMBER(drd(dev, DBI_BUS)) == 0U);
	if (reset) {
		data->core_resets++;
		LOG_WRN("%s: core reset (ltssm %s), restoring the root port", dev->name,
			ltssm_name(ltssm(dev)));
		setup_rc(dev);
	}
	if (data->link && !up) {
		uint8_t s = ltssm(dev);

		data->link = false;
		data->link_downs++;
		LOG_WRN("%s: link down (ltssm 0x%02x %s)", dev->name, s, ltssm_name(s));
	}
	data->up_polls = up && ltssm(dev) == LTSSM_L0 ? MIN(data->up_polls + 1U, 255U) : 0U;
	if (!data->link && data->up_polls >= 3U) {
		unsigned int gen, width;

		k_msleep(T_RRS_MS);
		data->link_up_ms = k_uptime_get();
		data->link = true;
		link_info(dev, &gen, &width);
		LOG_INF("%s: link up again, Gen%u x%u, scanning", dev->name, gen, width);
		sun55i_rc_scan(dev);
	} else if (reset && data->link) {
		sun55i_rc_scan(dev);
	}
	k_mutex_unlock(&data->op_lock);
out:
	k_work_schedule(dw, K_MSEC(CONFIG_PCIE_SUN55I_POLL_MS));
}

#if defined(CONFIG_PCIE_CTRL_SHELL)
static const struct pcie_shell_ctrl rc_shell;
#endif

static int sun55i_rc_init(const struct device *dev)
{
	const struct sun55i_rc_config *cfg = dev->config;
	struct sun55i_rc_data *data = dev->data;
	int ret;

#if defined(CONFIG_PCIE_CTRL_SHELL)
	pcie_shell_register(&rc_shell);
#endif
	ret = parse_ranges(dev);
	if (ret != 0) {
		return ret;
	}
	DEVICE_MMIO_NAMED_MAP(dev, dbi, K_MEM_CACHE_NONE);
	device_map(&data->cfg, cfg->cfg_phys, ATU_CFG_SIZE, K_MEM_CACHE_NONE);
	device_map(&data->combo, cfg->combo_phys, 0x1000, K_MEM_CACHE_NONE);
	device_map(&data->phy, cfg->combo_phys + PHY_OFFSET, PHY_SIZE, K_MEM_CACHE_NONE);
	device_map(&data->ccu, CCU_PHYS, 0x1000, K_MEM_CACHE_NONE);
	device_map(&data->r_ccu, R_CCU_PHYS, 0x1000, K_MEM_CACHE_NONE);
	device_map(&data->ppu, PPU_PHYS + PPU_PD_PCIE * 0x1000U, 0x1000, K_MEM_CACHE_NONE);

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

	ret = hw_setup(dev);
	if (ret != 0) {
		return ret;
	}
	data->active = true;
	if (link_start(dev)) {
		sun55i_rc_scan(dev);
	}
	data->dev = dev;
	k_mutex_init(&data->op_lock);
	k_work_init_delayable(&data->mon_work, mon_handler);
	if (CONFIG_PCIE_SUN55I_POLL_MS > 0) {
		k_work_schedule(&data->mon_work, K_MSEC(CONFIG_PCIE_SUN55I_POLL_MS));
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
/* pcie shell backend                                                       */

#if defined(CONFIG_PCIE_CTRL_SHELL)
static bool sh_active(const struct device *dev)
{
	ARG_UNUSED(dev);
	return true;
}

static void sh_info(const struct device *dev, const struct shell *sh)
{
	const struct sun55i_rc_config *cfg = dev->config;
	struct sun55i_rc_data *data = dev->data;
	uint32_t bus;
	uint8_t s;

	if (!data->active) {
		shell_print(sh, "root complex not initialized");
		return;
	}
	s = ltssm(dev);
	shell_print(sh, "ltssm      0x%02x %s, link %s (stat %08x, ctrl %08x)", s, ltssm_name(s),
		    link_up(dev) ? "up" : "down", drd(dev, APP_LINK_STAT),
		    drd(dev, APP_LTSSM_CTRL));
	if (link_up(dev)) {
		unsigned int gen, width;

		link_info(dev, &gen, &width);
		shell_print(sh, "link       Gen%u x%u (of x%u, max Gen%u), up for %lld ms", gen,
			    width, cfg->lanes, cfg->link_gen, k_uptime_get() - data->link_up_ms);
	}
	bus = drd(dev, DBI_BUS);
	shell_print(sh, "root port  %08x class %08x, buses %u..%u, scans %u", drd(dev, 0),
		    drd(dev, DBI_CLASS), (unsigned int)PCIE_BUS_SECONDARY_NUMBER(bus),
		    (unsigned int)PCIE_BUS_SUBORDINATE_NUMBER(bus), data->scans);
	shell_print(sh, "events     link downs %u, core resets %u", data->link_downs,
		    data->core_resets);
	shell_print(sh, "lnkcap     %08x  lnkctl/sta %08x  lnkctl2 %08x",
		    drd(dev, data->exp_cap + EXP_LNKCAP), drd(dev, data->exp_cap + EXP_LNKCTL),
		    drd(dev, data->exp_cap + EXP_LNKCTL2));
	shell_print(sh, "phy        PHYCTL %08x PHYSTS0 %08x",
		    sys_read32(data->combo + COMBO_PHYCTL),
		    sys_read32(data->combo + COMBO_PHYSTS0));
	shell_print(sh, "config     0x%lx (region 0, target %08x type %u)",
		    (unsigned long)cfg->cfg_phys, data->cfg_target, data->cfg_type);
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
}

static int sh_link(const struct device *dev, const struct shell *sh, bool retrain)
{
	struct sun55i_rc_data *data = dev->data;

	if (!data->active) {
		return -EBUSY;
	}
	if (retrain) {
		(void)k_mutex_lock(&data->op_lock, K_FOREVER);
		if (link_start(dev)) {
			sun55i_rc_scan(dev);
		}
		k_mutex_unlock(&data->op_lock);
	}
	shell_print(sh, "link %s, ltssm %s", link_up(dev) ? "up" : "down", ltssm_name(ltssm(dev)));
	return 0;
}

static int sh_link_trace(const struct device *dev, const struct shell *sh, uint32_t ms)
{
	struct sun55i_rc_data *data = dev->data;
	int64_t t0 = k_uptime_get();
	uint8_t last = 0xff;

	if (!data->active) {
		return -EBUSY;
	}
	while (k_uptime_get() - t0 < ms) {
		uint8_t s = ltssm(dev);

		if (s != last) {
			shell_print(sh, "%6lld ms  0x%02x %s", k_uptime_get() - t0, s,
				    ltssm_name(s));
			last = s;
		}
		k_busy_wait(50);
	}
	return 0;
}

static int sh_scan(const struct device *dev, const struct shell *sh)
{
	struct sun55i_rc_data *data = dev->data;

	if (!data->active) {
		return -EBUSY;
	}
	if (!link_up(dev)) {
		shell_print(sh, "no link");
		return -ENOTCONN;
	}
	(void)k_mutex_lock(&data->op_lock, K_FOREVER);
	sun55i_rc_scan(dev);
	k_mutex_unlock(&data->op_lock);
	return 0;
}

static int sh_id_get(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id)
{
	struct sun55i_rc_data *data = dev->data;
	uint32_t v;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -EINVAL;
	}
	v = drd(dev, 0);
	*id = (struct pcie_shell_id){
		.vid = v & 0xffff,
		.did = v >> 16,
		.class_code = drd(dev, DBI_CLASS) >> 8,
	};
	return 0;
}

/* applied now, not kept */
static int sh_id_set(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id)
{
	struct sun55i_rc_data *data = dev->data;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -EINVAL;
	}
	dwr(dev, DBI_MISC_CTRL1, drd(dev, DBI_MISC_CTRL1) | DBI_RO_WR_EN);
	dwr(dev, 0, ((uint32_t)id->did << 16) | id->vid);
	if (id->class_code != UINT32_MAX) {
		dwr(dev, DBI_CLASS, (id->class_code << 8) | (drd(dev, DBI_CLASS) & 0xffU));
	}
	dwr(dev, DBI_MISC_CTRL1, drd(dev, DBI_MISC_CTRL1) & ~DBI_RO_WR_EN);
	return 0;
}

static const struct pcie_shell_ctrl rc_shell = {
	.name = "a523",
	.dev = DEVICE_DT_INST_GET(0),
	.role = PCIE_SHELL_ROLE_RC,
	.active = sh_active,
	.info = sh_info,
	.link = sh_link,
	.link_trace = sh_link_trace,
	.scan = sh_scan,
	.id_get = sh_id_get,
	.id_set = sh_id_set,
	.cfg_read = sun55i_rc_conf_read,
	.cfg_write = sun55i_rc_conf_write,
};
#endif /* CONFIG_PCIE_CTRL_SHELL */

#define SUN55I_RC_DEFINE(n)                                                                        \
	static const struct pcie_ctrl_config sun55i_rc_ctrl_cfg_##n = {                            \
		.cfg_addr = DT_INST_REG_ADDR_BY_NAME(n, config),                                   \
		.cfg_size = DT_INST_REG_SIZE_BY_NAME(n, config),                                   \
		.ranges_count = DT_NUM_RANGES(DT_DRV_INST(n)),                                     \
		.ranges = {DT_FOREACH_RANGE(DT_DRV_INST(n), PCIE_RANGE_FORMAT)},                   \
	};                                                                                         \
	static const struct sun55i_rc_config sun55i_rc_cfg_##n = {                                 \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(dbi, DT_DRV_INST(n)),                           \
		.common = &sun55i_rc_ctrl_cfg_##n,                                                 \
		.cfg_phys = DT_INST_REG_ADDR_BY_NAME(n, config),                                   \
		.combo_phys = DT_INST_REG_ADDR_BY_NAME(n, combo),                                  \
		.lanes = DT_INST_PROP(n, num_lanes),                                               \
		.link_gen = DT_INST_PROP(n, max_link_speed),                                       \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.slot_power = GPIO_DT_SPEC_INST_GET_OR(n, slot_power_gpios, {0}),                  \
	};                                                                                         \
	static struct sun55i_rc_data sun55i_rc_data_##n;                                           \
	DEVICE_DT_INST_DEFINE(n, sun55i_rc_init, NULL, &sun55i_rc_data_##n, &sun55i_rc_cfg_##n,    \
			      POST_KERNEL, CONFIG_PCIE_SUN55I_INIT_PRIORITY, &sun55i_rc_api);

DT_INST_FOREACH_STATUS_OKAY(SUN55I_RC_DEFINE)
