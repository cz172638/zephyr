/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3700 PCIe controller (Aardvark) as root complex.
 *
 * Configuration requests to the link go through the PIO block (0x4000),
 * one dword at a time. The controller's own config space (0x000-0xfff) is
 * not a usable type 1 header in root complex mode, so the root port at
 * 00:00.0 is a software bridge: the header (0x00-0x3f) is kept in RAM,
 * the PCI Express capability (0xc0) and the extended capabilities (AER at
 * 0x100, DSN, L1SS, LTR, Secondary PCIe) are the hardware's, passed
 * through; only the port type is reported as Root Port. The PM, MSI and
 * MSI-X capabilities of the endpoint personality are left out of the
 * chain.
 *
 * Memory space: outbound windows (0x4c00) map the "ranges" of the node
 * 1:1 in power-of-two pieces. Inbound DMA reaches DRAM through the PCIe
 * IO decode window that TF-A sets up for all of DRAM.
 *
 * Board glue (A3700 only): peripheral clock, COMPHY lane through the TF-A
 * SiP service with the 100 MHz reference output on, PERST# on a GPIO.
 */

#define DT_DRV_COMPAT marvell_armada_3700_pcie

#include <stdlib.h>
#include <string.h>

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pcie/aardvark.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <soc.h>

LOG_MODULE_REGISTER(pcie_aardvark, LOG_LEVEL_INF);

/* Controller's own config space */
#define CFG_CMD_STATUS 0x04
#define CFG_REV_CLASS  0x08
#define CFG_PCIE_CAP   0xc0
#define CFG_DEVCTL     (CFG_PCIE_CAP + 0x08)
#define CFG_LNKCTL     (CFG_PCIE_CAP + 0x10)
#define CFG_LNKCTL2    (CFG_PCIE_CAP + 0x30)
#define CFG_EXT_START  0x100
#define CFG_SIZE       0x1000

/* PIO: configuration (and I/O) requests */
#define PIO_CTRL                 0x4000
#define PIO_CTRL_TYPE_MASK       GENMASK(3, 0)
#define PIO_CTRL_ADDR_WIN_DIS    BIT(24)
#define PIO_STAT                 0x4004
#define PIO_STAT_COMP_MASK       GENMASK(9, 7)
#define PIO_STAT_NON_POSTED      BIT(10)
#define PIO_STAT_ERR             BIT(11)
#define PIO_ADDR_LS              0x4008
#define PIO_ADDR_MS              0x400c
#define PIO_WR_DATA              0x4010
#define PIO_WR_DATA_STRB         0x4014
#define PIO_RD_DATA              0x4018
#define PIO_START                0x401c
#define PIO_ISR                  0x4020
#define PIO_ISRM                 0x4024
#define PIO_CFG_RD_TYPE0         0x8
#define PIO_CFG_RD_TYPE1         0x9
#define PIO_CFG_WR_TYPE0         0xa
#define PIO_CFG_WR_TYPE1         0xb
#define PIO_COMP_OK              0
#define PIO_COMP_UR              1
#define PIO_COMP_CRS             2
#define PIO_COMP_CA              4

/* Core control */
#define CTRL0                0x4800
#define CTRL0_GEN_MASK       GENMASK(1, 0)
#define CTRL0_IS_RC          BIT(2)
#define CTRL0_LANES_MASK     GENMASK(4, 3)
#define CTRL0_LINK_TRAIN     BIT(6)
#define CTRL2                0x4808
#define CTRL2_RESERVED       0x7
#define CTRL2_TD             BIT(4)
#define CTRL2_OB_WIN_EN      BIT(6)
#define REF_CLK              0x4814
#define REF_CLK_TX           BIT(1)
#define REF_CLK_RX           BIT(2)
#define ISR0                 0x4840
#define ISR0_MASK            0x4844
#define ISR1                 0x4848
#define ISR1_MASK            0x484c
#define OB_WIN_COUNT         8
#define OB_WIN(w, off)       (0x4c00 + 0x20 * (w) + (off))
#define OB_DEFAULT_ACTIONS   (OB_WIN(OB_WIN_COUNT - 1, 0x18) + 4)
#define OB_WIN_ENABLE        BIT(0)
#define OB_TYPE_MEM          0x0
#define OB_TYPE_IO           0x4
#define OB_MIN_SIZE          0x10000

/* Local management (Cadence LM layout) */
#define LMI_CFG              0x6000
#define LMI_LTSSM_SHIFT      24
#define LMI_LTSSM_MASK       0x3f
#define LMI_VENDOR_ID        0x6044
#define LTSSM_L0             0x10
#define LTSSM_DISABLED       0x20

#define CTRL_CONFIG          0x18000
#define CTRL_MODE_CMD        BIT(0)
#define HOST_INT_STATUS      0x1b000
#define HOST_INT_MASK        0x1b004

/* TF-A SiP COMPHY service */
#define SIP_COMPHY_POWER_ON  0x82000001
#define COMPHY_MODE_PCIE     0x6
#define COMPHY_SPEED_5G      0x3
#define COMPHY_PARAM(invert, width)                                                                \
	(((uint32_t)COMPHY_MODE_PCIE << 12) | ((uint32_t)COMPHY_SPEED_5G << 2) |                   \
	 ((uint32_t)(width) << 18) | ((uint32_t)(invert) & 0x3))
#define COMPHY_MISC_CTRL0    (0x4f * 2)
#define COMPHY_CLK100M_EN    BIT(4)

#define VENDOR_MARVELL       0x11ab

enum region_type {
	REGION_IO,
	REGION_MEM,
	REGION_MEM64,
	REGION_MAX,
};

struct advk_rc_config {
	const struct pcie_ctrl_config *common;
	uintptr_t reg_phys;
	size_t reg_size;
	uintptr_t comphy_phys;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
	struct gpio_dt_spec perst;
	uint8_t comphy_lane;
	uint8_t comphy_invert;
	uint8_t num_lanes;
	uint8_t max_speed;
};

/* Software type 1 header of the root port, 0x00-0x3f */
struct root_port {
	uint16_t cmd;
	uint32_t bus;      /* 0x18 */
	uint32_t io;       /* 0x1c, secondary status in the upper half */
	uint32_t mem;      /* 0x20 */
	uint32_t pref;     /* 0x24 */
	uint32_t pref_hi;  /* 0x28 */
	uint32_t pref_lim; /* 0x2c */
	uint32_t io_hi;    /* 0x30 */
	uint32_t bridge;   /* 0x3c: interrupt line/pin, bridge control */
};

struct advk_rc_data {
	mm_reg_t reg;
	mm_reg_t comphy;
	bool active;
	long comphy_ret;
	struct k_mutex pio_lock;
	struct root_port rp;
	struct {
		uintptr_t phys_start;
		uintptr_t bus_start;
		size_t size;
		size_t allocation_offset;
	} regions[REGION_MAX];
	uint32_t pio_err, pio_ur, pio_ca, pio_crs, pio_timeout;
	int64_t link_up_ms;
};

#define REG(dev) (((struct advk_rc_data *)(dev)->data)->reg)

static inline uint32_t rd(const struct device *dev, uint32_t off)
{
	return sys_read32(REG(dev) + off);
}

static inline void wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, REG(dev) + off);
}

static inline void rmw(const struct device *dev, uint32_t off, uint32_t mask, uint32_t v)
{
	wr(dev, off, (rd(dev, off) & ~mask) | (v & mask));
}

static uint8_t ltssm(const struct device *dev)
{
	return (rd(dev, LMI_CFG) >> LMI_LTSSM_SHIFT) & LMI_LTSSM_MASK;
}

static bool link_up(const struct device *dev)
{
	uint8_t s = ltssm(dev);

	return s >= LTSSM_L0 && s < LTSSM_DISABLED;
}

/*
 * Bring-up checkpoints: probe-read a few controller registers (a bus
 * error is caught, not fatal) and print them. False when the core
 * registers abort: the init stops there instead of taking the system down.
 */
static bool step(const struct device *dev, const char *what)
{
	uintptr_t base = REG(dev);
	uint32_t c0 = 0, cc = 0, id = 0;
	int e0 = a3700_probe_read32(base + CTRL0, &c0);
	int e1 = a3700_probe_read32(base + CTRL_CONFIG, &cc);
	int e2 = a3700_probe_read32(base + 0x0, &id);

	if (IS_ENABLED(CONFIG_PCIE_AARDVARK_TRACE_INIT) || e0 != 0 || e1 != 0 || e2 != 0) {
		printk("advk_rc: %-24s ctrl0 %s%08x  ctrl_config %s%08x  id %s%08x\n", what,
		       e0 ? "ABORT " : "", c0, e1 ? "ABORT " : "", cc, e2 ? "ABORT " : "", id);
	}
	return e0 == 0 && e1 == 0 && e2 == 0;
}

/* ------------------------------------------------------------------------ */
/* PIO configuration requests                                                */

static int pio_wait(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;

	/* a completion takes a few us; a timeout of the link takes ms */
	for (int i = 0; i < 50000; i++) {
		if (rd(dev, PIO_START) == 0 && rd(dev, PIO_ISR) != 0) {
			return 0;
		}
		k_busy_wait(1);
	}
	data->pio_timeout++;
	return -ETIMEDOUT;
}

/* 0, -EAGAIN (CRS), -EIO (UR/CA/error), -ETIMEDOUT */
static int pio_status(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;
	uint32_t st = rd(dev, PIO_STAT);

	switch (FIELD_GET(PIO_STAT_COMP_MASK, st)) {
	case PIO_COMP_OK:
		if ((st & PIO_STAT_ERR) != 0) {
			data->pio_err++;
			return -EIO;
		}
		return 0;
	case PIO_COMP_CRS:
		data->pio_crs++;
		return -EAGAIN;
	case PIO_COMP_UR:
		data->pio_ur++;
		return -EIO;
	case PIO_COMP_CA:
		data->pio_ca++;
		return -EIO;
	default:
		data->pio_err++;
		return -EIO;
	}
}

static int pio_cfg(const struct device *dev, bool write, bool type1, pcie_bdf_t bdf,
		   unsigned int reg, uint32_t *val)
{
	struct advk_rc_data *data = dev->data;
	uint32_t addr = (PCIE_BDF_TO_BUS(bdf) << 20) | (PCIE_BDF_TO_DEV(bdf) << 15) |
			(PCIE_BDF_TO_FUNC(bdf) << 12) | (reg << 2);
	int ret = -EIO;

	k_mutex_lock(&data->pio_lock, K_FOREVER);
	for (int retry = 0; retry < 100; retry++) {
		if (rd(dev, PIO_START) != 0) {
			/* a previous request never completed */
			ret = -EBUSY;
			break;
		}
		wr(dev, PIO_ISR, BIT(0));
		rmw(dev, PIO_CTRL, PIO_CTRL_TYPE_MASK | PIO_CTRL_ADDR_WIN_DIS,
		    (write ? (type1 ? PIO_CFG_WR_TYPE1 : PIO_CFG_WR_TYPE0)
			   : (type1 ? PIO_CFG_RD_TYPE1 : PIO_CFG_RD_TYPE0)) |
			    PIO_CTRL_ADDR_WIN_DIS);
		wr(dev, PIO_ADDR_LS, addr);
		wr(dev, PIO_ADDR_MS, 0);
		if (write) {
			wr(dev, PIO_WR_DATA, *val);
			wr(dev, PIO_WR_DATA_STRB, 0xf);
		}
		wr(dev, PIO_START, 1);

		ret = pio_wait(dev);
		if (ret == 0) {
			ret = pio_status(dev);
		}
		if (ret == 0 && !write) {
			*val = rd(dev, PIO_RD_DATA);
		}
		/* configuration request retry status: device still initializing */
		if (ret != -EAGAIN) {
			break;
		}
		k_busy_wait(1000);
	}
	k_mutex_unlock(&data->pio_lock);
	return ret;
}

/* ------------------------------------------------------------------------ */
/* Root port at 00:00.0                                                      */

static uint32_t rp_read(const struct device *dev, unsigned int reg)
{
	struct advk_rc_data *data = dev->data;
	struct root_port *rp = &data->rp;
	unsigned int off = reg * 4;
	uint32_t v;

	switch (off) {
	case PCIE_CONF_ID * 4:
		return rd(dev, 0x00);
	case PCIE_CONF_CMDSTAT * 4:
		/* status: capabilities list */
		return rp->cmd | ((uint32_t)BIT(4) << 16);
	case PCIE_CONF_CLASSREV * 4:
		return (rd(dev, CFG_REV_CLASS) & 0xff) | 0x06040000;
	case PCIE_CONF_TYPE * 4:
		return 0x00010000; /* header type 1 */
	case 0x18:
		return rp->bus;
	case 0x1c:
		return rp->io;
	case 0x20:
		return rp->mem;
	case 0x24:
		return rp->pref;
	case 0x28:
		return rp->pref_hi;
	case 0x2c:
		return rp->pref_lim;
	case 0x30:
		return rp->io_hi;
	case 0x34:
		return CFG_PCIE_CAP;
	case 0x3c:
		return rp->bridge;
	default:
		break;
	}

	if (off < 0x40) {
		return 0; /* BARs, expansion ROM */
	}
	if (off < CFG_PCIE_CAP) {
		return 0; /* PM, MSI, MSI-X of the endpoint personality: hidden */
	}
	if (off >= CFG_SIZE) {
		return 0xffffffff;
	}

	v = rd(dev, off);
	if (off == CFG_PCIE_CAP) {
		/* Root Port, slot implemented, end of the legacy chain */
		v = (v & ~(GENMASK(23, 20) | GENMASK(15, 8))) | (0x4U << 20) | BIT(24);
	}
	return v;
}

static void rp_write(const struct device *dev, unsigned int reg, uint32_t v)
{
	struct advk_rc_data *data = dev->data;
	struct root_port *rp = &data->rp;
	unsigned int off = reg * 4;

	switch (off) {
	case PCIE_CONF_CMDSTAT * 4:
		rp->cmd = v & (PCIE_CONF_CMDSTAT_IO | PCIE_CONF_CMDSTAT_MEM |
			       PCIE_CONF_CMDSTAT_MASTER | BIT(6) | BIT(8) | BIT(10));
		return;
	case 0x18:
		rp->bus = v & 0x00ffffff;
		return;
	case 0x1c:
		/* secondary status is RW1C, nothing latched here */
		rp->io = v & 0x0000ffff;
		return;
	case 0x20:
		rp->mem = v & 0xfff0fff0;
		return;
	case 0x24:
		rp->pref = v & 0xfff0fff0;
		return;
	case 0x28:
		rp->pref_hi = v;
		return;
	case 0x2c:
		rp->pref_lim = v;
		return;
	case 0x30:
		rp->io_hi = v;
		return;
	case 0x3c:
		rp->bridge = (rp->bridge & 0x0000ff00) | (v & 0xffff00ff);
		return;
	default:
		break;
	}

	/* header rest, hidden capabilities, the PCIe capability register itself */
	if (off <= CFG_PCIE_CAP || off >= CFG_SIZE) {
		return;
	}
	wr(dev, off, v);
}

/* ------------------------------------------------------------------------ */
/* pcie_ctrl API                                                             */

static bool bus_routes(const struct device *dev, pcie_bdf_t bdf, bool *type1)
{
	struct advk_rc_data *data = dev->data;
	unsigned int bus = PCIE_BDF_TO_BUS(bdf);
	unsigned int sec = PCIE_BUS_SECONDARY_NUMBER(data->rp.bus);
	unsigned int sub = PCIE_BUS_SUBORDINATE_NUMBER(data->rp.bus);

	if (sec == 0 || bus < sec || bus > sub || !link_up(dev)) {
		return false;
	}
	/* point to point: only device 0 directly below the root port */
	if (bus == sec && PCIE_BDF_TO_DEV(bdf) != 0) {
		return false;
	}
	*type1 = bus != sec;
	return true;
}

static uint32_t advk_rc_conf_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	struct advk_rc_data *data = dev->data;
	uint32_t v = 0xffffffff;
	bool type1;

	if (!data->active || reg >= CFG_SIZE / 4) {
		return 0xffffffff;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		return bdf == PCIE_BDF(0, 0, 0) ? rp_read(dev, reg) : 0xffffffff;
	}
	if (!bus_routes(dev, bdf, &type1) || pio_cfg(dev, false, type1, bdf, reg, &v) != 0) {
		return 0xffffffff;
	}
	return v;
}

static void advk_rc_conf_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg,
			       uint32_t v)
{
	struct advk_rc_data *data = dev->data;
	bool type1;

	if (!data->active || reg >= CFG_SIZE / 4) {
		return;
	}
	if (PCIE_BDF_TO_BUS(bdf) == 0) {
		if (bdf == PCIE_BDF(0, 0, 0)) {
			rp_write(dev, reg, v);
		}
		return;
	}
	if (bus_routes(dev, bdf, &type1)) {
		(void)pio_cfg(dev, true, type1, bdf, reg, &v);
	}
}

static enum region_type region_pick(struct advk_rc_data *data, bool mem, bool mem64)
{
	if (!mem) {
		return REGION_IO;
	}
	if ((mem64 && data->regions[REGION_MEM64].size != 0) ||
	    data->regions[REGION_MEM].size == 0) {
		return REGION_MEM64;
	}
	return REGION_MEM;
}

static bool advk_rc_region_allocate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				    bool mem64, size_t bar_size, uintptr_t *bar_bus_addr)
{
	struct advk_rc_data *data = dev->data;
	enum region_type t = region_pick(data, mem, mem64);
	uintptr_t addr;

	ARG_UNUSED(bdf);

	if (data->regions[t].size == 0) {
		return false;
	}
	addr = ((data->regions[t].bus_start + data->regions[t].allocation_offset - 1) |
		(bar_size - 1)) + 1;
	if (addr - data->regions[t].bus_start + bar_size > data->regions[t].size) {
		return false;
	}
	*bar_bus_addr = addr;
	data->regions[t].allocation_offset = addr - data->regions[t].bus_start + bar_size;
	return true;
}

static bool advk_rc_region_get_allocate_base(const struct device *dev, pcie_bdf_t bdf, bool mem,
					     bool mem64, size_t align, uintptr_t *bar_base_addr)
{
	struct advk_rc_data *data = dev->data;
	enum region_type t;

	ARG_UNUSED(bdf);

	/*
	 * No separate prefetchable region: leave the bridges' prefetchable
	 * windows closed, prefetchable BARs come out of the memory window.
	 */
	if (mem && mem64 && data->regions[REGION_MEM64].size == 0) {
		return false;
	}
	t = region_pick(data, mem, mem64);
	if (data->regions[t].size == 0) {
		return false;
	}
	*bar_base_addr = ((data->regions[t].bus_start + data->regions[t].allocation_offset - 1) |
			  (align - 1)) + 1;
	data->regions[t].allocation_offset = *bar_base_addr - data->regions[t].bus_start;
	return true;
}

static bool advk_rc_region_translate(const struct device *dev, pcie_bdf_t bdf, bool mem,
				     bool mem64, uintptr_t bar_bus_addr, uintptr_t *bar_addr)
{
	struct advk_rc_data *data = dev->data;
	enum region_type t = region_pick(data, mem, mem64);

	ARG_UNUSED(bdf);

	if (bar_bus_addr == 0) {
		return false;
	}
	*bar_addr = data->regions[t].phys_start + (bar_bus_addr - data->regions[t].bus_start);
	return true;
}

static DEVICE_API(pcie_ctrl, advk_rc_api) = {
	.conf_read = advk_rc_conf_read,
	.conf_write = advk_rc_conf_write,
	.region_allocate = advk_rc_region_allocate,
	.region_get_allocate_base = advk_rc_region_get_allocate_base,
	.region_translate = advk_rc_region_translate,
};

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

static int parse_ranges(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct advk_rc_data *data = dev->data;

	memset(data->regions, 0, sizeof(data->regions));
	for (size_t i = 0; i < cfg->common->ranges_count; i++) {
		enum region_type t;

		switch ((cfg->common->ranges[i].flags >> 24) & 0x3) {
		case 0x1:
			t = REGION_IO;
			break;
		case 0x2:
			t = REGION_MEM;
			break;
		case 0x3:
			t = REGION_MEM64;
			break;
		default:
			continue;
		}
		data->regions[t].bus_start = cfg->common->ranges[i].pcie_bus_addr;
		data->regions[t].phys_start = cfg->common->ranges[i].host_map_addr;
		data->regions[t].size = cfg->common->ranges[i].map_length;
	}
	return data->regions[REGION_MEM].size != 0 || data->regions[REGION_MEM64].size != 0
		       ? 0
		       : -EINVAL;
}

static void ob_set(const struct device *dev, int w, uint64_t axi, uint64_t pci, uint64_t size,
		   uint32_t type)
{
	uint64_t mask = ~(size - 1);

	wr(dev, OB_WIN(w, 0x00), 0);
	wr(dev, OB_WIN(w, 0x04), (uint32_t)(axi >> 32));
	wr(dev, OB_WIN(w, 0x08), (uint32_t)pci);
	wr(dev, OB_WIN(w, 0x0c), (uint32_t)(pci >> 32));
	wr(dev, OB_WIN(w, 0x10), (uint32_t)mask);
	wr(dev, OB_WIN(w, 0x14), (uint32_t)(mask >> 32));
	wr(dev, OB_WIN(w, 0x18), type);
	wr(dev, OB_WIN(w, 0x00), (uint32_t)axi | OB_WIN_ENABLE);
}

/* Map each range 1:1 (or as given) in naturally aligned power-of-two windows */
static int ob_setup(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;
	static const enum region_type order[] = {REGION_MEM64, REGION_MEM, REGION_IO};
	int w = 0;

	for (int i = 0; i < OB_WIN_COUNT; i++) {
		for (uint32_t off = 0; off <= 0x18; off += 4) {
			wr(dev, OB_WIN(i, off), 0);
		}
	}

	for (size_t r = 0; r < ARRAY_SIZE(order); r++) {
		uint64_t axi = data->regions[order[r]].phys_start;
		uint64_t pci = data->regions[order[r]].bus_start;
		uint64_t left = data->regions[order[r]].size;

		while (left >= OB_MIN_SIZE) {
			uint64_t sz = BIT64(63 - __builtin_clzll(left));
			uint64_t x = axi | pci;
			uint64_t align = x & (~x + 1); /* lowest set bit, 0 for 0 */

			if (align != 0 && sz > align) {
				sz = align;
			}
			if (w == OB_WIN_COUNT) {
				LOG_ERR("out of outbound windows, 0x%llx bytes of %s unmapped",
					(unsigned long long)left,
					order[r] == REGION_IO ? "I/O" : "memory");
				return -ENOSPC;
			}
			ob_set(dev, w++, axi, pci, sz,
			       order[r] == REGION_IO ? OB_TYPE_IO : OB_TYPE_MEM);
			axi += sz;
			pci += sz;
			left -= sz;
		}
	}
	LOG_DBG("%d outbound windows", w);
	return 0;
}

static int hw_setup(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	uint32_t lanes = cfg->num_lanes >= 4 ? 2 : (cfg->num_lanes == 2 ? 1 : 0);
	uint32_t gen = cfg->max_speed >= 3 ? 2 : (cfg->max_speed == 2 ? 1 : 0);

	if (!step(dev, "mapped")) {
		return -EIO;
	}
	/* Direct mode, reference clock driven to the slot */
	rmw(dev, CTRL_CONFIG, CTRL_MODE_CMD, 0);
	if (!step(dev, "direct mode")) {
		return -EIO;
	}
	rmw(dev, REF_CLK, REF_CLK_TX | REF_CLK_RX, REF_CLK_TX);
	if (!step(dev, "refclk tx")) {
		return -EIO;
	}

	/* Root complex, lanes, speed, training off */
	rmw(dev, CTRL0, CTRL0_IS_RC | CTRL0_LANES_MASK | CTRL0_GEN_MASK | CTRL0_LINK_TRAIN,
	    CTRL0_IS_RC | FIELD_PREP(CTRL0_LANES_MASK, lanes) | FIELD_PREP(CTRL0_GEN_MASK, gen));

	if (!step(dev, "root complex")) {
		return -EIO;
	}

	/* Vendor (and subsystem vendor) ID: erratum, reset value is wrong */
	wr(dev, LMI_VENDOR_ID, ((uint32_t)VENDOR_MARVELL << 16) | VENDOR_MARVELL);

	/* PCI-PCI bridge class, root bridge decodes nothing by itself */
	wr(dev, CFG_REV_CLASS, (rd(dev, CFG_REV_CLASS) & 0xff) | 0x06040000);
	rmw(dev, CFG_CMD_STATUS, PCIE_CONF_CMDSTAT_IO | PCIE_CONF_CMDSTAT_MEM |
				      PCIE_CONF_CMDSTAT_MASTER, 0);

	/*
	 * Device Control of the root port: MPS 128 B, MRRS 512 B, no relaxed
	 * ordering, no no-snoop; error reporting on (AER of the port)
	 */
	rmw(dev, CFG_DEVCTL, GENMASK(14, 0),
	    BIT(0) | BIT(1) | BIT(2) | BIT(3) | (0U << 5) | (2U << 12));

	/* Target link speed */
	rmw(dev, CFG_LNKCTL2, GENMASK(3, 0), cfg->max_speed != 0 ? cfg->max_speed : 2);

	/* Outbound windows on, everything else as memory */
	wr(dev, CTRL2, CTRL2_RESERVED | CTRL2_TD | CTRL2_OB_WIN_EN);
	wr(dev, OB_DEFAULT_ACTIONS, OB_TYPE_MEM);

	/* No interrupts yet: all masked, latched status cleared */
	wr(dev, ISR0_MASK, UINT32_MAX);
	wr(dev, ISR1_MASK, UINT32_MAX);
	wr(dev, HOST_INT_MASK, UINT32_MAX);
	wr(dev, ISR0, UINT32_MAX);
	wr(dev, ISR1, UINT32_MAX);
	wr(dev, HOST_INT_STATUS, UINT32_MAX);
	wr(dev, PIO_ISRM, UINT32_MAX);
	wr(dev, PIO_ISR, UINT32_MAX);
	return step(dev, "setup done") ? 0 : -EIO;
}

static void perst(const struct device *dev, bool assert)
{
	const struct advk_rc_config *cfg = dev->config;

	if (cfg->perst.port != NULL) {
		(void)gpio_pin_set_dt(&cfg->perst, assert ? 1 : 0);
	}
}

/* PERST# pulse, training, wait for L0; true when the link is up */
static bool advk_rc_link_start(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;
	int64_t t0;

	if (!step(dev, "link start")) {
		return false;
	}
	rmw(dev, CTRL0, CTRL0_LINK_TRAIN, 0);
	perst(dev, true);
	k_msleep(CONFIG_PCIE_AARDVARK_PERST_MS);
	perst(dev, false);
	if (!step(dev, "perst released")) {
		return false;
	}

	rmw(dev, CTRL0, CTRL0_LINK_TRAIN, CTRL0_LINK_TRAIN);
	t0 = k_uptime_get();
	while (!link_up(dev)) {
		if (k_uptime_get() - t0 > CONFIG_PCIE_AARDVARK_LINK_TIMEOUT_MS) {
			LOG_WRN("%s: no link (ltssm 0x%02x)", dev->name, ltssm(dev));
			return false;
		}
		k_msleep(1);
	}
	data->link_up_ms = k_uptime_get();

	/* PCIe CEM: 100 ms from PERST# deassertion to the first config request */
	k_msleep(100);

	uint32_t sta = rd(dev, CFG_LNKCTL) >> 16;

	LOG_INF("%s: link up in %lld ms, Gen%u x%u", dev->name, data->link_up_ms - t0,
		sta & 0xf, (sta >> 4) & 0x3f);
	return true;
}

static void advk_rc_scan(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;

	for (int t = 0; t < REGION_MAX; t++) {
		data->regions[t].allocation_offset = 0;
	}
	memset(&data->rp, 0, sizeof(data->rp));
	data->rp.bridge = 0x00000100; /* INTA */
	pcie_generic_ctrl_enumerate(dev, PCIE_BDF(0, 0, 0));
}

static long comphy_power_on(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct arm_smccc_res res;

	arm_smccc_smc(SIP_COMPHY_POWER_ON, cfg->comphy_lane,
		      COMPHY_PARAM(cfg->comphy_invert, cfg->num_lanes), 0, 0, 0, 0, 0, &res);
	return (long)res.a0;
}

static int advk_rc_init(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct advk_rc_data *data = dev->data;
	int ret;

	k_mutex_init(&data->pio_lock);

	if (advk_mode_get() != ADVK_MODE_RC) {
		LOG_INF("%s: endpoint mode selected, root complex idle", dev->name);
		return 0;
	}

	ret = parse_ranges(dev);
	if (ret != 0) {
		LOG_ERR("%s: no memory range", dev->name);
		return ret;
	}

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	ret = clock_control_on(cfg->clk_dev, cfg->clk_id);
	if (ret != 0) {
		return ret;
	}
	device_map(&data->reg, cfg->reg_phys, cfg->reg_size, K_MEM_CACHE_NONE);
	/* informational: the core may only answer once the PHY runs */
	(void)step(dev, "clock on");

	data->comphy_ret = comphy_power_on(dev);
	if (data->comphy_ret != 0) {
		LOG_ERR("%s: COMPHY lane %u PCIe power on failed (%ld)", dev->name,
			cfg->comphy_lane, data->comphy_ret);
	}
	device_map(&data->comphy, cfg->comphy_phys, 0x400, K_MEM_CACHE_NONE);
	/* the slot needs the 100 MHz reference (an endpoint build turns it off) */
	sys_write16(sys_read16(data->comphy + COMPHY_MISC_CTRL0) | COMPHY_CLK100M_EN,
		    data->comphy + COMPHY_MISC_CTRL0);
	if (IS_ENABLED(CONFIG_PCIE_AARDVARK_TRACE_INIT)) {
		printk("advk_rc: comphy SiP returned %ld, misc_ctrl0 %04x\n", data->comphy_ret,
		       sys_read16(data->comphy + COMPHY_MISC_CTRL0));
	}
	/*
	 * The core registers abort for a while after the clock and the PHY
	 * come up (the endpoint driver's printk checkpoints used to cover
	 * it): wait until the core answers.
	 */
	{
		int64_t t0 = k_uptime_get();
		uint32_t v;

		while (a3700_probe_read32(REG(dev) + CTRL0, &v) != 0) {
			if (k_uptime_get() - t0 > CONFIG_PCIE_AARDVARK_CORE_TIMEOUT_MS) {
				break;
			}
			k_msleep(1);
		}
		LOG_INF("%s: core answers %lld ms after the PHY", dev->name,
			k_uptime_get() - t0);
	}
	if (!step(dev, "comphy")) {
		return -EIO;
	}

	ret = hw_setup(dev);
	if (ret != 0) {
		LOG_ERR("%s: controller registers abort, root complex not started", dev->name);
		return ret;
	}
	ret = ob_setup(dev);
	if (ret != 0) {
		return ret;
	}

	/* PCIE1_RESETn as GPIO: PERST# is ours, asserted until training */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}
	}
	if (!step(dev, "pinctrl")) {
		return -EIO;
	}
	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_OUTPUT_ACTIVE);
		if (ret != 0) {
			return ret;
		}
	}
	if (!step(dev, "perst asserted")) {
		return -EIO;
	}
	data->active = true;

	(void)advk_rc_link_start(dev);
	advk_rc_scan(dev);
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Shell                                                                     */

#if defined(CONFIG_PCIE_AARDVARK_SHELL)
#define RC_DEV DEVICE_DT_INST_GET(0)

static bool rc_active(const struct shell *sh)
{
	struct advk_rc_data *data = RC_DEV->data;

	if (!data->active) {
		shell_error(sh, "root complex not active (advk mode rc, then reboot)");
		return false;
	}
	return true;
}

static int cmd_rc_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = RC_DEV;
	struct advk_rc_data *data = dev->data;
	uint32_t lnk;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_active(sh)) {
		return -ENODEV;
	}
	lnk = rd(dev, CFG_LNKCTL) >> 16;
	shell_print(sh, "ltssm 0x%02x (%s), lnksta %04x Gen%u x%u, ctrl0 %08x, comphy %ld",
		    ltssm(dev), link_up(dev) ? "up" : "down", lnk, lnk & 0xf, (lnk >> 4) & 0x3f,
		    rd(dev, CTRL0), data->comphy_ret);
	shell_print(sh, "root port: bus %06x mem %08x pref %08x cmd %04x", data->rp.bus,
		    data->rp.mem, data->rp.pref, data->rp.cmd);
	shell_print(sh, "PIO: ur %u ca %u crs %u err %u timeout %u", data->pio_ur, data->pio_ca,
		    data->pio_crs, data->pio_err, data->pio_timeout);
	for (int t = 0; t < REGION_MAX; t++) {
		if (data->regions[t].size != 0) {
			shell_print(sh, "%-5s cpu 0x%lx bus 0x%lx size 0x%zx used 0x%zx",
				    t == REGION_IO ? "io" : t == REGION_MEM ? "mem" : "mem64",
				    data->regions[t].phys_start, data->regions[t].bus_start,
				    data->regions[t].size, data->regions[t].allocation_offset);
		}
	}
	for (int w = 0; w < OB_WIN_COUNT; w++) {
		uint32_t m = rd(dev, OB_WIN(w, 0x00));

		if ((m & OB_WIN_ENABLE) != 0) {
			shell_print(sh, "ob%d: axi %08x -> pci %08x mask %08x type %x", w,
				    (uint32_t)(m & ~OB_WIN_ENABLE), rd(dev, OB_WIN(w, 0x08)),
				    rd(dev, OB_WIN(w, 0x10)), rd(dev, OB_WIN(w, 0x18)));
		}
	}
	return 0;
}

static int cmd_rc_retrain(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_active(sh)) {
		return -ENODEV;
	}
	shell_print(sh, "%s", advk_rc_link_start(RC_DEV) ? "link up" : "no link");
	advk_rc_scan(RC_DEV);
	shell_print(sh, "rescanned, see: pcie ls");
	return 0;
}

static int cmd_rc_scan(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_active(sh)) {
		return -ENODEV;
	}
	advk_rc_scan(RC_DEV);
	shell_print(sh, "rescanned, see: pcie ls");
	return 0;
}

static int parse_bdf(const struct shell *sh, const char *s, pcie_bdf_t *bdf)
{
	unsigned long b, d, f;
	char *e;

	b = strtoul(s, &e, 16);
	if (*e != ':') {
		goto bad;
	}
	d = strtoul(e + 1, &e, 16);
	if (*e != '.') {
		goto bad;
	}
	f = strtoul(e + 1, &e, 16);
	if (*e != '\0' || b > 255 || d > 31 || f > 7) {
		goto bad;
	}
	*bdf = PCIE_BDF(b, d, f);
	return 0;
bad:
	shell_error(sh, "bus:dev.fn, hex");
	return -EINVAL;
}

/* cfg <b:d.f> [<off> [<len>]]: dump config space, like lspci -xxx */
static int cmd_rc_cfg(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long off = 0, len = 0x100;
	pcie_bdf_t bdf;

	if (!rc_active(sh) || parse_bdf(sh, argv[1], &bdf) != 0) {
		return -EINVAL;
	}
	if (argc > 2) {
		off = strtoul(argv[2], NULL, 0) & ~3UL;
	}
	if (argc > 3) {
		len = strtoul(argv[3], NULL, 0);
	}
	for (unsigned long o = off; o < MIN(off + len, (unsigned long)CFG_SIZE); o += 16) {
		uint32_t v[4];

		for (int i = 0; i < 4; i++) {
			v[i] = advk_rc_conf_read(RC_DEV, bdf, (o >> 2) + i);
		}
		shell_print(sh, "%03lx: %08x %08x %08x %08x", o, v[0], v[1], v[2], v[3]);
	}
	return 0;
}

/* cfgw <b:d.f> <off> <val>: write one config dword */
static int cmd_rc_cfgw(const struct shell *sh, size_t argc, char **argv)
{
	pcie_bdf_t bdf;
	unsigned long off;

	ARG_UNUSED(argc);
	if (!rc_active(sh) || parse_bdf(sh, argv[1], &bdf) != 0) {
		return -EINVAL;
	}
	off = strtoul(argv[2], NULL, 0);
	advk_rc_conf_write(RC_DEV, bdf, off >> 2, strtoul(argv[3], NULL, 0));
	shell_print(sh, "%03lx: %08x", off & ~3UL, advk_rc_conf_read(RC_DEV, bdf, off >> 2));
	return 0;
}



#if defined(CONFIG_PCIE_AARDVARK_NVME_SHELL)
/*
 * nvme <b:d.f> [<lba>]: minimal polled NVMe bring-up, no interrupts.
 * Admin queues in DDR, Identify Controller and Namespace 1, one I/O queue
 * pair, read one block. Checks the outbound path (registers, doorbells)
 * and the inbound one (the device DMAs queues, completions and data into
 * DDR). Cache maintenance around every DMA, whatever the coherency.
 */
#define NV_QSIZE   16
#define NV_CAP     0x00
#define NV_VS      0x08
#define NV_CC      0x14
#define NV_CSTS    0x1c
#define NV_AQA     0x24
#define NV_ASQ     0x28
#define NV_ACQ     0x30
#define NV_DB      0x1000

struct nv_sqe {
	uint32_t dw0, nsid, rsvd[2];
	uint64_t mptr, prp1, prp2;
	uint32_t cdw10, cdw11, cdw12, cdw13, cdw14, cdw15;
};

struct nv_cqe {
	uint32_t result, rsvd;
	uint16_t sqhd, sqid;
	uint16_t cid, status; /* status bit 0: phase */
};

BUILD_ASSERT(sizeof(struct nv_sqe) == 64 && sizeof(struct nv_cqe) == 16);

static struct nv_sqe nv_asq[NV_QSIZE] __aligned(4096);
static struct nv_cqe nv_acq[NV_QSIZE] __aligned(4096);
static struct nv_sqe nv_iosq[NV_QSIZE] __aligned(4096);
static struct nv_cqe nv_iocq[NV_QSIZE] __aligned(4096);
static uint8_t nv_buf[4096] __aligned(4096);

struct nv_q {
	struct nv_sqe *sq;
	struct nv_cqe *cq;
	uint16_t tail, head, cid;
	uint8_t phase;
	uint32_t sq_db, cq_db;
};

struct nv_ctx {
	const struct shell *sh;
	mm_reg_t bar;
	uint32_t stride;
	uint32_t to_ms;
	struct nv_q admin, io;
};

static uint64_t nv_pa(const void *p)
{
	return (uint64_t)k_mem_phys_addr((void *)p);
}

static void nv_wr64(struct nv_ctx *c, uint32_t off, uint64_t v)
{
	sys_write32((uint32_t)v, c->bar + off);
	sys_write32((uint32_t)(v >> 32), c->bar + off + 4);
}

static int nv_wait_rdy(struct nv_ctx *c, bool rdy)
{
	int64_t t0 = k_uptime_get();

	while (((sys_read32(c->bar + NV_CSTS) & BIT(0)) != 0) != rdy) {
		if ((sys_read32(c->bar + NV_CSTS) & BIT(1)) != 0) {
			shell_error(c->sh, "CSTS.CFS: controller fatal status");
			return -EIO;
		}
		if (k_uptime_get() - t0 > c->to_ms) {
			shell_error(c->sh, "timeout waiting for CSTS.RDY=%d (CSTS %08x)", rdy,
				    sys_read32(c->bar + NV_CSTS));
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}
	return 0;
}

/* Submit one command, poll its completion; returns the NVMe status or -errno */
static int nv_cmd(struct nv_ctx *c, struct nv_q *q, struct nv_sqe *cmd, uint32_t *result)
{
	struct nv_cqe *e;
	int64_t t0;
	uint16_t cid = q->cid++;

	cmd->dw0 = (cmd->dw0 & 0xffff) | ((uint32_t)cid << 16);
	q->sq[q->tail] = *cmd;
	sys_cache_data_flush_range(&q->sq[q->tail], sizeof(*cmd));
	q->tail = (q->tail + 1) % NV_QSIZE;
	barrier_dsync_fence_full();
	sys_write32(q->tail, c->bar + q->sq_db);

	t0 = k_uptime_get();
	e = &q->cq[q->head];
	while (true) {
		sys_cache_data_invd_range(e, sizeof(*e));
		if ((e->status & 1) == q->phase) {
			break;
		}
		if (k_uptime_get() - t0 > 2000) {
			shell_error(c->sh, "command %02x: no completion (CSTS %08x)",
				    cmd->dw0 & 0xff, sys_read32(c->bar + NV_CSTS));
			return -ETIMEDOUT;
		}
		k_busy_wait(10);
	}
	if (result != NULL) {
		*result = e->result;
	}
	q->head = (q->head + 1) % NV_QSIZE;
	if (q->head == 0) {
		q->phase ^= 1;
	}
	sys_write32(q->head, c->bar + q->cq_db);
	if (e->cid != cid) {
		shell_warn(c->sh, "completion for cid %u, expected %u", e->cid, cid);
	}
	return e->status >> 1;
}

static void nv_str(const struct shell *sh, const char *what, const uint8_t *p, size_t n)
{
	char s[48];

	n = MIN(n, sizeof(s) - 1);
	memcpy(s, p, n);
	s[n] = '\0';
	for (int i = n - 1; i >= 0 && s[i] == ' '; i--) {
		s[i] = '\0';
	}
	shell_print(sh, "  %-9s %s", what, s);
}

static int nv_identify(struct nv_ctx *c, uint32_t cns, uint32_t nsid)
{
	struct nv_sqe cmd = {.dw0 = 0x06, .nsid = nsid, .prp1 = nv_pa(nv_buf), .cdw10 = cns};

	memset(nv_buf, 0, sizeof(nv_buf));
	sys_cache_data_flush_and_invd_range(nv_buf, sizeof(nv_buf));
	int st = nv_cmd(c, &c->admin, &cmd, NULL);

	sys_cache_data_invd_range(nv_buf, sizeof(nv_buf));
	return st;
}

static int cmd_rc_nvme(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = RC_DEV;
	struct nv_ctx c = {.sh = sh};
	unsigned long long lba = argc > 2 ? strtoull(argv[2], NULL, 0) : 0;
	uint64_t cap, nsze, bar_phys;
	uint32_t lo, hi, bs, flbas;
	pcie_bdf_t bdf;
	int st;

	if (!rc_active(sh) || parse_bdf(sh, argv[1], &bdf) != 0) {
		return -EINVAL;
	}
	lo = advk_rc_conf_read(dev, bdf, PCIE_CONF_BAR0);
	hi = advk_rc_conf_read(dev, bdf, PCIE_CONF_BAR1);
	if (lo == 0xffffffff || (lo & 0x1) != 0) {
		shell_error(sh, "no memory BAR0");
		return -ENODEV;
	}
	bar_phys = (lo & ~0xfULL) | (((lo & 0x6) == 0x4) ? ((uint64_t)hi << 32) : 0);
	/* memory space and bus master on */
	advk_rc_conf_write(dev, bdf, PCIE_CONF_CMDSTAT,
			   (advk_rc_conf_read(dev, bdf, PCIE_CONF_CMDSTAT) & 0xffff) |
				   PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER);

	device_map(&c.bar, bar_phys, 0x2000, K_MEM_CACHE_NONE);
	cap = sys_read32(c.bar + NV_CAP) | ((uint64_t)sys_read32(c.bar + NV_CAP + 4) << 32);
	c.stride = 4U << ((cap >> 32) & 0xf);
	c.to_ms = MAX(((cap >> 24) & 0xff) * 500U, 500U);
	shell_print(sh, "BAR0 0x%llx, CAP %016llx, VS %08x, MQES %u, doorbell stride %u",
		    (unsigned long long)bar_phys, (unsigned long long)cap,
		    sys_read32(c.bar + NV_VS), (uint32_t)(cap & 0xffff) + 1, c.stride);
	if (((cap >> 48) & 0xf) != 0) {
		shell_error(sh, "MPSMIN above 4 KiB not handled");
		goto out;
	}

	/* disable, set up the admin queues, enable */
	sys_write32(sys_read32(c.bar + NV_CC) & ~BIT(0), c.bar + NV_CC);
	if (nv_wait_rdy(&c, false) != 0) {
		goto out;
	}
	memset(nv_asq, 0, sizeof(nv_asq));
	memset(nv_acq, 0, sizeof(nv_acq));
	memset(nv_iosq, 0, sizeof(nv_iosq));
	memset(nv_iocq, 0, sizeof(nv_iocq));
	sys_cache_data_flush_and_invd_range(nv_asq, sizeof(nv_asq));
	sys_cache_data_flush_and_invd_range(nv_acq, sizeof(nv_acq));
	sys_cache_data_flush_and_invd_range(nv_iosq, sizeof(nv_iosq));
	sys_cache_data_flush_and_invd_range(nv_iocq, sizeof(nv_iocq));
	c.admin = (struct nv_q){.sq = nv_asq, .cq = nv_acq, .phase = 1,
				.sq_db = NV_DB, .cq_db = NV_DB + c.stride};
	c.io = (struct nv_q){.sq = nv_iosq, .cq = nv_iocq, .phase = 1,
			     .sq_db = NV_DB + 2 * c.stride, .cq_db = NV_DB + 3 * c.stride};
	sys_write32(((NV_QSIZE - 1) << 16) | (NV_QSIZE - 1), c.bar + NV_AQA);
	nv_wr64(&c, NV_ASQ, nv_pa(nv_asq));
	nv_wr64(&c, NV_ACQ, nv_pa(nv_acq));
	/* NVM command set, 4 KiB pages, round robin, SQE 64 B, CQE 16 B */
	sys_write32((6U << 16) | (4U << 20) | BIT(0), c.bar + NV_CC);
	if (nv_wait_rdy(&c, true) != 0) {
		goto out;
	}
	shell_print(sh, "enabled, admin queues at 0x%llx / 0x%llx",
		    (unsigned long long)nv_pa(nv_asq), (unsigned long long)nv_pa(nv_acq));

	st = nv_identify(&c, 1, 0);
	if (st != 0) {
		shell_error(sh, "Identify Controller: status 0x%x", st);
		goto out;
	}
	shell_print(sh, "controller: PCI VID %02x%02x", nv_buf[1], nv_buf[0]);
	nv_str(sh, "model", &nv_buf[24], 40);
	nv_str(sh, "serial", &nv_buf[4], 20);
	nv_str(sh, "firmware", &nv_buf[64], 8);
	shell_print(sh, "  MDTS %u, namespaces %u", nv_buf[77], sys_get_le32(&nv_buf[516]));

	st = nv_identify(&c, 0, 1);
	if (st != 0) {
		shell_error(sh, "Identify Namespace 1: status 0x%x", st);
		goto out;
	}
	nsze = sys_get_le64(&nv_buf[0]);
	flbas = nv_buf[26] & 0xf;
	bs = 1U << nv_buf[128 + 4 * flbas + 2];
	shell_print(sh, "namespace 1: %llu blocks of %u B = %llu MiB",
		    (unsigned long long)nsze, bs, (unsigned long long)(nsze * bs >> 20));
	if (bs > sizeof(nv_buf)) {
		shell_error(sh, "block size above 4 KiB not handled");
		goto out;
	}

	/* I/O queue pair 1, no interrupts */
	struct nv_sqe ccq = {.dw0 = 0x05, .prp1 = nv_pa(nv_iocq),
			     .cdw10 = ((NV_QSIZE - 1) << 16) | 1, .cdw11 = BIT(0)};
	struct nv_sqe csq = {.dw0 = 0x01, .prp1 = nv_pa(nv_iosq),
			     .cdw10 = ((NV_QSIZE - 1) << 16) | 1, .cdw11 = (1U << 16) | BIT(0)};

	st = nv_cmd(&c, &c.admin, &ccq, NULL);
	if (st == 0) {
		st = nv_cmd(&c, &c.admin, &csq, NULL);
	}
	if (st != 0) {
		shell_error(sh, "creating the I/O queues: status 0x%x", st);
		goto out;
	}

	memset(nv_buf, 0xa5, sizeof(nv_buf));
	sys_cache_data_flush_and_invd_range(nv_buf, sizeof(nv_buf));
	struct nv_sqe rd = {.dw0 = 0x02, .nsid = 1, .prp1 = nv_pa(nv_buf),
			    .cdw10 = (uint32_t)lba, .cdw11 = (uint32_t)(lba >> 32), .cdw12 = 0};
	int64_t t0 = k_uptime_get();

	st = nv_cmd(&c, &c.io, &rd, NULL);
	sys_cache_data_invd_range(nv_buf, sizeof(nv_buf));
	if (st != 0) {
		shell_error(sh, "read LBA %llu: status 0x%x", lba, st);
		goto out;
	}
	shell_print(sh, "LBA %llu (%lld ms):", lba, k_uptime_get() - t0);
	shell_hexdump(sh, nv_buf, MIN(bs, 128U));
	if (bs >= 512 && nv_buf[510] == 0x55 && nv_buf[511] == 0xaa) {
		shell_print(sh, "  55aa at 510: MBR / protective MBR");
	}
out:
	k_mem_unmap_phys_bare((uint8_t *)c.bar, 0x2000);
	return 0;
}
#endif /* CONFIG_PCIE_AARDVARK_NVME_SHELL */

/*
 * Bring-up experiments, also when the init stopped: the steps one by
 * one, each followed by a probe of the core registers.
 *   advk_rc probe | pinctrl | clkout on|off | perst assert|release|input
 *   | comphy | start
 */
static bool rc_mapped(const struct shell *sh)
{
	if (REG(RC_DEV) == 0) {
		shell_error(sh, "not mapped (endpoint mode?)");
		return false;
	}
	return true;
}

static int cmd_rc_probe(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_mapped(sh)) {
		return -ENODEV;
	}
	shell_print(sh, "core %s", step(RC_DEV, "probe") ? "answers" : "ABORTS");
	return 0;
}

static int cmd_rc_pinctrl(const struct shell *sh, size_t argc, char **argv)
{
	const struct advk_rc_config *cfg = RC_DEV->config;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_mapped(sh) || cfg->pcfg == NULL) {
		return -ENODEV;
	}
	shell_print(sh, "pinctrl %d", pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT));
	(void)step(RC_DEV, "pinctrl");
	return 0;
}

static int cmd_rc_clkout(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_rc_data *data = RC_DEV->data;
	mem_addr_t r;
	uint16_t v;

	ARG_UNUSED(argc);
	if (!rc_mapped(sh) || data->comphy == 0) {
		return -ENODEV;
	}
	r = data->comphy + COMPHY_MISC_CTRL0;
	v = sys_read16(r);
	v = strcmp(argv[1], "on") == 0 ? (v | COMPHY_CLK100M_EN) : (v & ~COMPHY_CLK100M_EN);
	sys_write16(v, r);
	shell_print(sh, "misc_ctrl0 %04x", sys_read16(r));
	(void)step(RC_DEV, "clkout");
	return 0;
}

static int cmd_rc_perst(const struct shell *sh, size_t argc, char **argv)
{
	const struct advk_rc_config *cfg = RC_DEV->config;
	int ret;

	ARG_UNUSED(argc);
	if (!rc_mapped(sh) || cfg->perst.port == NULL) {
		return -ENODEV;
	}
	if (strcmp(argv[1], "input") == 0) {
		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_INPUT);
	} else {
		ret = gpio_pin_configure_dt(&cfg->perst, strcmp(argv[1], "assert") == 0
								 ? GPIO_OUTPUT_ACTIVE
								 : GPIO_OUTPUT_INACTIVE);
	}
	shell_print(sh, "perst %s: %d", argv[1], ret);
	(void)step(RC_DEV, "perst");
	return 0;
}

static int cmd_rc_comphy(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_mapped(sh)) {
		return -ENODEV;
	}
	shell_print(sh, "SiP %ld", comphy_power_on(RC_DEV));
	(void)step(RC_DEV, "comphy");
	return 0;
}

/* controller setup, windows, training and scan from the current state */
static int cmd_rc_start(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_rc_data *data = RC_DEV->data;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_mapped(sh)) {
		return -ENODEV;
	}
	if (hw_setup(RC_DEV) != 0 || ob_setup(RC_DEV) != 0) {
		shell_error(sh, "setup failed, see above");
		return -EIO;
	}
	data->active = true;
	shell_print(sh, "%s", advk_rc_link_start(RC_DEV) ? "link up" : "no link");
	advk_rc_scan(RC_DEV);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(rc_cmds,
	SHELL_CMD_ARG(status, NULL, "Link, root port windows, PIO errors, outbound windows",
		      cmd_rc_status, 1, 0),
	SHELL_CMD_ARG(retrain, NULL, "PERST# pulse, train, rescan", cmd_rc_retrain, 1, 0),
	SHELL_CMD_ARG(scan, NULL, "Enumerate again (bus numbers, BARs, windows)", cmd_rc_scan,
		      1, 0),
	SHELL_CMD_ARG(cfg, NULL, "<b:d.f> [<off> [<len>]] Dump config space", cmd_rc_cfg, 2, 2),
	SHELL_CMD_ARG(cfgw, NULL, "<b:d.f> <off> <val> Write a config dword", cmd_rc_cfgw, 4,
		      0),
	IF_ENABLED(CONFIG_PCIE_AARDVARK_NVME_SHELL,
		   (SHELL_CMD_ARG(nvme, NULL, "<b:d.f> [<lba>] Polled NVMe test: identify, read "
				  "one block", cmd_rc_nvme, 2, 1),))
	SHELL_CMD_ARG(probe, NULL, "Do the core registers answer?", cmd_rc_probe, 1, 0),
	SHELL_CMD_ARG(pinctrl, NULL, "Apply the default pin state", cmd_rc_pinctrl, 1, 0),
	SHELL_CMD_ARG(clkout, NULL, "on|off COMPHY 100 MHz output", cmd_rc_clkout, 2, 0),
	SHELL_CMD_ARG(perst, NULL, "assert|release|input PERST# GPIO", cmd_rc_perst, 2, 0),
	SHELL_CMD_ARG(comphy, NULL, "COMPHY power on again (SiP)", cmd_rc_comphy, 1, 0),
	SHELL_CMD_ARG(start, NULL, "Setup, windows, training, scan from the current state",
		      cmd_rc_start, 1, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(advk_rc, &rc_cmds, "Armada 3700 PCIe root complex", NULL);
#endif /* CONFIG_PCIE_AARDVARK_SHELL */

/* ------------------------------------------------------------------------ */

#define ADVK_RC_HAS_PINCTRL(n) DT_INST_PINCTRL_HAS_NAME(n, default)

#define ADVK_RC_DEFINE(n)                                                                          \
	IF_ENABLED(ADVK_RC_HAS_PINCTRL(n), (PINCTRL_DT_INST_DEFINE(n);))                          \
	static const struct pcie_ctrl_config advk_rc_ctrl_cfg_##n = {                              \
		.cfg_addr = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                     \
		.cfg_size = DT_INST_REG_SIZE_BY_NAME(n, ctrl),                                     \
		.ranges_count = DT_NUM_RANGES(DT_DRV_INST(n)),                                     \
		.ranges = {DT_FOREACH_RANGE(DT_DRV_INST(n), PCIE_RANGE_FORMAT)},                   \
	};                                                                                         \
	static const struct advk_rc_config advk_rc_cfg_##n = {                                     \
		.common = &advk_rc_ctrl_cfg_##n,                                                   \
		.reg_phys = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                     \
		.reg_size = DT_INST_REG_SIZE_BY_NAME(n, ctrl),                                     \
		.comphy_phys = DT_INST_REG_ADDR_BY_NAME(n, comphy),                                \
		.pcfg = COND_CODE_1(ADVK_RC_HAS_PINCTRL(n), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),   \
				    (NULL)),                                                       \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, index),                   \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.comphy_lane = DT_INST_PROP(n, marvell_comphy_lane),                               \
		.comphy_invert = DT_INST_PROP(n, marvell_comphy_polarity_invert),                  \
		.num_lanes = DT_INST_PROP_OR(n, num_lanes, 1),                                     \
		.max_speed = DT_INST_PROP_OR(n, max_link_speed, 2),                                \
	};                                                                                         \
	static struct advk_rc_data advk_rc_data_##n;                                               \
	DEVICE_DT_INST_DEFINE(n, advk_rc_init, NULL, &advk_rc_data_##n, &advk_rc_cfg_##n,          \
			      POST_KERNEL, CONFIG_PCIE_AARDVARK_INIT_PRIORITY, &advk_rc_api);

DT_INST_FOREACH_STATUS_OKAY(ADVK_RC_DEFINE)
