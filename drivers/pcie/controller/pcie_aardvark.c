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
#include <zephyr/drivers/pcie/aer.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/pcie_shell.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/spi_nor_ex.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/random/random.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
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
#define CTRL1                0x4804
#define CTRL1_HOT_RESET      BIT(0)
#define CTRL2                0x4808
#define CTRL2_RESERVED       0x7
#define CTRL2_TD             BIT(4)
#define CTRL2_OB_WIN_EN      BIT(6)
#define REF_CLK              0x4814
#define REF_CLK_TX           BIT(1)
#define REF_CLK_RX           BIT(2)
#define CTRL2_MSI_EN         BIT(10)
#define MSG_LOG              0x4830 /* last inbound message, requester ID [31:16] */
#define ISR0                 0x4840
#define ISR0_PME             BIT(7)
#define ISR0_CORR_ERR        BIT(11)
#define ISR0_NFAT_ERR        BIT(12)
#define ISR0_FAT_ERR         BIT(13)
#define ISR0_ERR_MASK        GENMASK(13, 11)
#define ISR0_INTX_ASSERT(n)  BIT(16 + (n))
#define ISR0_INTX_MASK       GENMASK(19, 16)
#define ISR0_MSI_PENDING     BIT(24)
#define MSI_ADDR_LO          0x4850
#define MSI_ADDR_HI          0x4854
#define MSI_STATUS           0x4858
#define MSI_MASK             0x485c
#define MSI_PAYLOAD          0x489c
#define MSI_VECTORS          32
#define HOST_INT_CORE        BIT(16)
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
#define LTSSM_CONFIG_LW_START 0x05
#define LTSSM_CONFIG_IDLE    0x0a
#define LTSSM_L0             0x10
#define LTSSM_DISABLED       0x20
#define LTSSM_RECOVERY_EQ0   0x28
#define LTSSM_RECOVERY_EQ3   0x2b

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
	void (*irq_config)(const struct device *dev);
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
	uint16_t secsta;   /* 0x1e: from the PIO completions */
	uint16_t rootctl;  /* PCIe cap + 0x1c: PME interrupt, CRS visibility */
	uint32_t rootsta;  /* PCIe cap + 0x20: PME requester and status */
};

/* Root port identity: Vendor/Device ID, Subsystem (cap 0x0d at 0x40) */
struct rp_id {
	uint16_t vid, did, svid, ssid;
};

#define CFG_SSVID_CAP 0x40
#define RCID_KEY      "advk/rcid"
/* the root port's own setting, the endpoint keeps pcie/ep/dsn<n> */
#define RCDSN_KEY     "pcie/rc/dsn0"
#define CFG_DSN_LO    0x154
#define CFG_DSN_HI    0x158

struct advk_rc_data {
	mm_reg_t reg;
	struct rp_id id;
	uint64_t dsn;
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
	/* interrupts: MSI receiver, error messages, INTx */
	uint64_t msi_doorbell __aligned(64);
	uint32_t msi_used;
	struct {
		void (*routine)(const void *parameter);
		const void *parameter;
		uint32_t count;
	} msi[MSI_VECTORS];
	uint32_t irq_count, msi_spurious, err_cor, err_nfat, err_fat, intx[4], pme;
	uint32_t last_isr0, last_payload;
	int64_t link_up_ms;
#if defined(CONFIG_PCIE_AARDVARK_AER)
	const struct device *self;
	struct k_work aer_work;
	uint32_t aer_events;
#endif
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

/* Data Link Layer Link Active: Configuration.Idle, Recovery, L0, L0s, L1, L2 */
static bool link_active(const struct device *dev)
{
	uint8_t s = ltssm(dev);

	return s >= LTSSM_CONFIG_IDLE && s < LTSSM_DISABLED;
}

/* Link Training: Configuration and Recovery states */
static bool link_training(const struct device *dev)
{
	uint8_t s = ltssm(dev);

	return (s >= LTSSM_CONFIG_LW_START && s < LTSSM_L0) ||
	       (s >= LTSSM_RECOVERY_EQ0 && s <= LTSSM_RECOVERY_EQ3);
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
		data->rp.secsta |= BIT(13); /* Received Master Abort */
		return -EIO;
	case PIO_COMP_CA:
		data->pio_ca++;
		data->rp.secsta |= BIT(12); /* Received Target Abort */
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
	for (int retry = 0; retry < CONFIG_PCIE_AARDVARK_CRS_TIMEOUT_MS; retry++) {
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
		return ((uint32_t)data->id.did << 16) | data->id.vid;
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
		return rp->io | ((uint32_t)rp->secsta << 16);
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
		return CFG_SSVID_CAP;
	case CFG_SSVID_CAP:
		/* Subsystem ID capability of a bridge, then the PCIe capability */
		return 0x0d | (CFG_PCIE_CAP << 8);
	case CFG_SSVID_CAP + 4:
		return ((uint32_t)data->id.ssid << 16) | data->id.svid;
	case 0x3c:
		/*
		 * Bridge Control as in Linux: SERR# Enable is the error message
		 * interrupt (ISR0 bits 11-13 unmasked), Secondary Bus Reset is
		 * the hot reset generator
		 */
		v = rp->bridge & ~(BIT(16 + 1) | BIT(16 + 6));
		if ((rd(dev, ISR0_MASK) & ISR0_ERR_MASK) == 0) {
			v |= BIT(16 + 1);
		}
		if ((rd(dev, CTRL1) & CTRL1_HOT_RESET) != 0) {
			v |= BIT(16 + 6);
		}
		return v;
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

	switch (off - CFG_PCIE_CAP) {
	case 0x14:
		/* Slot Capabilities: physical slot 1, nothing else */
		return 1U << 19;
	case 0x18:
		/* Slot Status: Presence Detect State, no hot plug */
		return BIT(16 + 6);
	case 0x1c:
		/* Root Control (PME interrupt, CRS visibility), Root Capabilities: CRS SV */
		return rp->rootctl | BIT(16);
	case 0x20:
		return rp->rootsta;
	default:
		break;
	}

	v = rd(dev, off);
	if (off == CFG_PCIE_CAP) {
		/* Root Port, slot implemented, end of the legacy chain */
		v = (v & ~(GENMASK(23, 20) | GENMASK(15, 8))) | (0x4U << 20) | BIT(24);
	} else if (off == CFG_PCIE_CAP + 0x0c) {
		/* Link Capabilities: DL Active reporting (emulated from the LTSSM) */
		v |= BIT(20);
	} else if (off == CFG_PCIE_CAP + 0x10) {
		v &= ~(BIT(16 + 11) | BIT(16 + 13));
		if (link_training(dev)) {
			v |= BIT(16 + 11);
		}
		if (link_active(dev)) {
			v |= BIT(16 + 13);
		}
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
		/*
		 * Bus master of a root port gates requests from below toward
		 * memory (device DMA): the hardware's own bit has to follow
		 * (as Linux does since "PCI: aardvark: Fix support for bus
		 * mastering bit on emulated bridge").
		 */
		rmw(dev, CFG_CMD_STATUS, PCIE_CONF_CMDSTAT_MASTER, v & PCIE_CONF_CMDSTAT_MASTER);
		return;
	case 0x18:
		rp->bus = v & 0x00ffffff;
		return;
	case 0x1c:
		/* secondary status is RW1C */
		rp->io = v & 0x0000ffff;
		rp->secsta &= ~(v >> 16);
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
		rmw(dev, ISR0_MASK, ISR0_ERR_MASK, (v & BIT(16 + 1)) ? 0 : ISR0_ERR_MASK);
		rmw(dev, CTRL1, CTRL1_HOT_RESET, (v & BIT(16 + 6)) ? CTRL1_HOT_RESET : 0);
		return;
	default:
		break;
	}

	switch (off - CFG_PCIE_CAP) {
	case 0x14:
	case 0x18:
		return; /* slot: nothing to control */
	case 0x1c:
		rp->rootctl = v & (BIT(3) | BIT(4)); /* PME IE, CRS SV enable */
		return;
	case 0x20:
		if ((v & BIT(16)) != 0) {
			rp->rootsta = 0; /* PME Status, RW1C */
		}
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
/* Interrupts: MSI receiver, error messages, INTx                            */

/*
 * MSIs are memory writes to the doorbell address; the controller takes
 * them out of the inbound stream, sets bit (data & 31) in MSI_STATUS and
 * raises ISR0.MSI_PENDING. One GIC SPI for everything.
 */
static void advk_rc_isr(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;
	uint32_t host = rd(dev, HOST_INT_STATUS);
	uint32_t isr0 = rd(dev, ISR0) & ~rd(dev, ISR0_MASK);
	uint32_t isr1 = rd(dev, ISR1) & ~rd(dev, ISR1_MASK);

	data->irq_count++;
	data->last_isr0 = isr0;

	/* PME first: the message log holds only the last inbound message */
	if ((isr0 & ISR0_PME) != 0) {
		data->pme++;
		if ((data->rp.rootsta & BIT(16)) == 0) {
			data->rp.rootsta = (rd(dev, MSG_LOG) >> 16) | BIT(16);
		}
	}

	if ((isr0 & ISR0_MSI_PENDING) != 0) {
		uint32_t st = rd(dev, MSI_STATUS) & ~rd(dev, MSI_MASK);

		data->last_payload = rd(dev, MSI_PAYLOAD);
		wr(dev, MSI_STATUS, st);
		while (st != 0) {
			int v = find_lsb_set(st) - 1;

			st &= ~BIT(v);
			data->msi[v].count++;
			if (data->msi[v].routine != NULL) {
				data->msi[v].routine(data->msi[v].parameter);
			} else {
				data->msi_spurious++;
			}
		}
	}
	if ((isr0 & ISR0_CORR_ERR) != 0) {
		data->err_cor++;
	}
	if ((isr0 & ISR0_NFAT_ERR) != 0) {
		data->err_nfat++;
	}
	if ((isr0 & ISR0_FAT_ERR) != 0) {
		data->err_fat++;
	}
#if defined(CONFIG_PCIE_AARDVARK_AER)
	/* source and status take config requests: from a thread */
	if ((isr0 & ISR0_ERR_MASK) != 0 && data->self != NULL) {
		k_work_submit(&data->aer_work);
	}
#endif
	for (int i = 0; i < 4; i++) {
		if ((isr0 & ISR0_INTX_ASSERT(i)) != 0) {
			data->intx[i]++;
		}
	}

	wr(dev, ISR0, isr0);
	wr(dev, ISR1, isr1);
	wr(dev, HOST_INT_STATUS, host);
}

#if defined(CONFIG_PCIE_AARDVARK_AER)
static void aer_report(const struct device *dev, const struct pcie_aer_event *ev)
{
	static const char *const sev[] = {"correctable", "non-fatal", "fatal"};
	const struct pcie_aer_stat *st = NULL;
	uint32_t n = 0;
	char bits[96];

	for (int i = 0; (st = pcie_aer_stat_get(i)) != NULL; i++) {
		if (st->bdf == ev->bdf) {
			n = st->cnt[0] + st->cnt[1] + st->cnt[2];
			break;
		}
	}
	if (n > CONFIG_PCIE_AER_LOG_LIMIT) {
		return; /* counted only, see pcie aer */
	}
	if (ev->sev == PCIE_AER_COR) {
		pcie_aer_bits_str(bits, sizeof(bits), ev->ce, pcie_aer_ce_name);
	} else {
		pcie_aer_bits_str(bits, sizeof(bits), ev->ue, pcie_aer_ue_name);
	}
	if (ev->bdf == PCIE_BDF_NONE) {
		LOG_WRN("%s: AER %s error, source unknown", dev->name, sev[ev->sev]);
	} else {
		LOG_WRN("%s: AER %s error from %02x:%02x.%x%s: %s", dev->name, sev[ev->sev],
			PCIE_BDF_TO_BUS(ev->bdf), PCIE_BDF_TO_DEV(ev->bdf),
			PCIE_BDF_TO_FUNC(ev->bdf), ev->multiple ? " (and more)" : "",
			bits[0] != '\0' ? bits : "(no AER status)");
	}
	if (ev->hdr_valid) {
		LOG_WRN("  TLP header %08x %08x %08x %08x", ev->hdr[0], ev->hdr[1], ev->hdr[2],
			ev->hdr[3]);
	}
	if (n == CONFIG_PCIE_AER_LOG_LIMIT) {
		LOG_WRN("  further errors of this source only counted (pcie aer)");
	}
}

static void aer_work_fn(struct k_work *w)
{
	struct advk_rc_data *data = CONTAINER_OF(w, struct advk_rc_data, aer_work);
	int n;

	if (!data->active) {
		return;
	}
	n = pcie_aer_root_service(data->self, PCIE_BDF(0, 0, 0), aer_report);
	if (n > 0) {
		data->aer_events += n;
	}
}

static void aer_setup(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;
	uint32_t flags = (IS_ENABLED(CONFIG_PCIE_AER_UR_EP) ? PCIE_AER_UR_EP : 0) |
			 (IS_ENABLED(CONFIG_PCIE_AER_UR_BRIDGE) ? PCIE_AER_UR_BRIDGE : 0) |
			 (IS_ENABLED(CONFIG_PCIE_AER_ECRC) ? PCIE_AER_ECRC : 0);
	int n;

	if (!data->active) {
		return;
	}
	n = pcie_aer_setup(dev, PCIE_BDF(0, 0, 0), flags);
	/* anything latched meanwhile went with the setup's clear */
	wr(dev, ISR0, ISR0_ERR_MASK);
	LOG_INF("%s: AER on %d function(s)%s%s", dev->name, n,
		(flags & PCIE_AER_ECRC) ? ", ECRC" : "",
		(flags & PCIE_AER_UR_EP) ? ", UR on endpoints" : "");
}
#endif

static void irq_setup(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct advk_rc_data *data = dev->data;
	uint64_t db = k_mem_phys_addr(&data->msi_doorbell);

#if defined(CONFIG_PCIE_AARDVARK_AER)
	if (data->self == NULL) {
		k_work_init(&data->aer_work, aer_work_fn);
		data->self = dev;
	}
#endif
	wr(dev, MSI_ADDR_LO, (uint32_t)db);
	wr(dev, MSI_ADDR_HI, (uint32_t)(db >> 32));
	wr(dev, MSI_MASK, ~data->msi_used);
	wr(dev, MSI_STATUS, UINT32_MAX);
	rmw(dev, CTRL2, CTRL2_MSI_EN, CTRL2_MSI_EN);

	/* MSI, error messages and INTx assertions; nothing from ISR1 */
	wr(dev, ISR0, UINT32_MAX);
	wr(dev, ISR1, UINT32_MAX);
	wr(dev, ISR0_MASK,
	   (uint32_t)~(ISR0_MSI_PENDING | ISR0_PME | ISR0_ERR_MASK | ISR0_INTX_MASK));
	wr(dev, ISR1_MASK, UINT32_MAX);
	wr(dev, HOST_INT_STATUS, UINT32_MAX);
	wr(dev, HOST_INT_MASK, (uint32_t)~HOST_INT_CORE);

	cfg->irq_config(dev);
}

#if defined(CONFIG_PCIE_MSI)
static uint8_t advk_rc_msi_device_setup(const struct device *dev, unsigned int priority,
					msi_vector_t *vectors, uint8_t n_vector)
{
	struct advk_rc_data *data = dev->data;
	uint64_t db = k_mem_phys_addr(&data->msi_doorbell);
	uint8_t n = 0;

	if (!data->active) {
		return 0;
	}
	for (int v = 0; v < MSI_VECTORS && n < n_vector; v++) {
		if ((data->msi_used & BIT(v)) != 0) {
			continue;
		}
		data->msi_used |= BIT(v);
		vectors[n].arch.irq = v;
		vectors[n].arch.address = (uint32_t)db;
		vectors[n].arch.eventid = v;
		vectors[n].arch.priority = priority;
		n++;
	}
	wr(dev, MSI_MASK, ~data->msi_used);
	return n;
}

static bool advk_rc_msi_vector_connect(const struct device *dev, msi_vector_t *vector,
				       void (*routine)(const void *parameter),
				       const void *parameter, uint32_t flags)
{
	struct advk_rc_data *data = dev->data;
	unsigned int v = vector->arch.eventid;

	ARG_UNUSED(flags);

	if (v >= MSI_VECTORS || (data->msi_used & BIT(v)) == 0) {
		return false;
	}
	data->msi[v].parameter = parameter;
	data->msi[v].routine = routine;
	return true;
}
#endif

/* For in-driver users (shell tests): a free vector, its doorbell and data */
static int advk_rc_msi_alloc(const struct device *dev, void (*routine)(const void *),
			     const void *parameter, uint32_t *addr, uint16_t *msg)
{
	struct advk_rc_data *data = dev->data;

	for (int v = MSI_VECTORS - 1; v >= 0; v--) {
		if ((data->msi_used & BIT(v)) == 0) {
			data->msi_used |= BIT(v);
			data->msi[v].routine = routine;
			data->msi[v].parameter = parameter;
			*addr = (uint32_t)k_mem_phys_addr(&data->msi_doorbell);
			*msg = v;
			wr(dev, MSI_MASK, ~data->msi_used);
			return v;
		}
	}
	return -ENOSPC;
}

static void advk_rc_msi_free(const struct device *dev, int v)
{
	struct advk_rc_data *data = dev->data;

	data->msi[v].routine = NULL;
	data->msi_used &= ~BIT(v);
	wr(dev, MSI_MASK, ~data->msi_used);
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
#if defined(CONFIG_PCIE_MSI)
	.msi_device_setup = advk_rc_msi_device_setup,
	.msi_vector_connect = advk_rc_msi_vector_connect,
#endif
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


/* Root port IDs into the hardware too (vendor through the LMI, device in cfg) */
static void id_apply(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;

	wr(dev, LMI_VENDOR_ID, ((uint32_t)data->id.svid << 16) | data->id.vid);
	rmw(dev, 0x00, 0xffff0000, (uint32_t)data->id.did << 16);
}

static int id_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		      void *param)
{
	struct rp_id *id = param;
	struct rp_id v;

	ARG_UNUSED(key);
	if (len == sizeof(v) && read_cb(cb_arg, &v, len) == (ssize_t)len) {
		*id = v;
	}
	return 0;
}

static void id_load(const struct device *dev)
{
	struct advk_rc_data *data = dev->data;

	data->id = (struct rp_id){.vid = VENDOR_MARVELL, .did = rd(dev, 0x00) >> 16,
				  .svid = VENDOR_MARVELL, .ssid = 0};
	if (IS_ENABLED(CONFIG_SETTINGS) && settings_subsys_init() == 0) {
		(void)settings_load_subtree_direct(RCID_KEY, id_load_cb, &data->id);
	}
	id_apply(dev);
}


/*
 * Device Serial Number of the root port: the hardware's DSN capability
 * (0x150, passed through), kept in its own setting (pcie/rc/dsn0). When
 * nothing is saved, the SPI NOR unique ID (locally administered EUI-64,
 * the endpoint makes the same one) or else a random value is written to
 * the setting on the first start, so the value stays put from then on.
 */
static void dsn_write(const struct device *dev, uint64_t v)
{
	struct advk_rc_data *data = dev->data;

	data->dsn = v;
	wr(dev, CFG_DSN_LO, (uint32_t)v);
	wr(dev, CFG_DSN_HI, (uint32_t)(v >> 32));
}

static uint64_t dsn_local(uint64_t v)
{
	return (v & ~(0x01ULL << 56)) | (0x02ULL << 56);
}

static uint64_t dsn_generate(const char **src)
{
#if DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor) && defined(CONFIG_FLASH_EX_OP_ENABLED)
	const struct device *nor = DEVICE_DT_GET_ANY(jedec_spi_nor);
	uint8_t uid[SPI_NOR_UID_LEN];
	uint64_t u = 0;

	if (device_is_ready(nor) && flash_ex_op(nor, FLASH_EX_OP_SPI_NOR_READ_UID, 0, uid) == 0) {
		for (int b = 0; b < SPI_NOR_UID_LEN; b++) {
			u = (u << 8) | uid[b];
		}
		if (u != 0 && u != UINT64_MAX) {
			*src = "SPI NOR unique ID";
			return dsn_local(u);
		}
	}
#endif
	*src = "random";
	return dsn_local(sys_rand64_get() ^ (k_cycle_get_64() * 0x9e3779b97f4a7c15ULL));
}

static int dsn_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
		       void *param)
{
	uint64_t *v = param;

	ARG_UNUSED(key);
	if (len == sizeof(*v)) {
		(void)read_cb(cb_arg, v, len);
	}
	return 0;
}

static void dsn_load(const struct device *dev)
{
	const char *src = "saved";
	bool settings_ok = IS_ENABLED(CONFIG_SETTINGS) && settings_subsys_init() == 0;
	uint64_t v = 0;

	if (settings_ok) {
		(void)settings_load_subtree_direct(RCDSN_KEY, dsn_load_cb, &v);
	}
	if (v == 0) {
		v = dsn_generate(&src);
		if (settings_ok) {
			int ret = settings_save_one(RCDSN_KEY, &v, sizeof(v));

			if (ret != 0) {
				LOG_WRN("DSN not saved (%d)", ret);
			}
		}
	}
	dsn_write(dev, v);
	LOG_INF("root port DSN %016llx (%s)", (unsigned long long)v, src);
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
	/* devicetree PCIe devices (e.g. nvme-controller) found only now */
	(void)pcie_dev_scan();
#if defined(CONFIG_PCIE_AARDVARK_AER)
	aer_setup(dev);
#endif
}

static long comphy_power_on(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct arm_smccc_res res;

	arm_smccc_smc(SIP_COMPHY_POWER_ON, cfg->comphy_lane,
		      COMPHY_PARAM(cfg->comphy_invert, cfg->num_lanes), 0, 0, 0, 0, 0, &res);
	return (long)res.a0;
}

#if defined(CONFIG_PCIE_AARDVARK_SHELL)
static const struct pcie_shell_ctrl rc_shell;
#endif

static int advk_rc_init(const struct device *dev)
{
	const struct advk_rc_config *cfg = dev->config;
	struct advk_rc_data *data = dev->data;
	int ret;

	k_mutex_init(&data->pio_lock);
#if defined(CONFIG_PCIE_AARDVARK_SHELL)
	pcie_shell_register(&rc_shell);
#endif

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
	id_load(dev);
	dsn_load(dev);
	ret = ob_setup(dev);
	if (ret != 0) {
		return ret;
	}
	irq_setup(dev);

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
		shell_error(sh, "root complex not active (pcie controller mode rc, then reboot)");
		return false;
	}
	return true;
}

static int parse_bdf(const struct shell *sh, const char *s, pcie_bdf_t *bdf)
{
	if (pcie_shell_parse_bdf(s, bdf) != 0) {
		shell_error(sh, "bus:dev.fn, hex");
		return -EINVAL;
	}
	return 0;
}

/* --- pcie shell backend ---------------------------------------------------- */

static bool sh_active(const struct device *dev)
{
	ARG_UNUSED(dev);
	return advk_mode_get() == ADVK_MODE_RC;
}

static void sh_info(const struct device *dev, const struct shell *sh)
{
	struct advk_rc_data *data = dev->data;
	uint32_t lnk;

	if (!data->active) {
		shell_print(sh, "root complex not started (see pcie debug bringup)");
		return;
	}
	lnk = rd(dev, CFG_LNKCTL) >> 16;
	shell_print(sh, "ltssm 0x%02x (%s), lnksta %04x Gen%u x%u, ctrl0 %08x, comphy %ld",
		    ltssm(dev), link_up(dev) ? "up" : "down", lnk, lnk & 0xf, (lnk >> 4) & 0x3f,
		    rd(dev, CTRL0), data->comphy_ret);
	shell_print(sh, "root port %04x:%04x: bus %06x mem %08x pref %08x cmd %04x",
		    data->id.vid, data->id.did, data->rp.bus, data->rp.mem, data->rp.pref,
		    data->rp.cmd);
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
}

static void sh_irq(const struct device *dev, const struct shell *sh)
{
	struct advk_rc_data *data = dev->data;

	if (!data->active) {
		return;
	}
	shell_print(sh, "irqs %u, last isr0 %08x; now isr0 %08x mask %08x, isr1 %08x, host %08x",
		    data->irq_count, data->last_isr0, rd(dev, ISR0), rd(dev, ISR0_MASK),
		    rd(dev, ISR1), rd(dev, HOST_INT_STATUS));
	shell_print(sh, "MSI: doorbell 0x%llx, used %08x, mask %08x, status %08x, payload %08x "
			"(last %08x), spurious %u",
		    (unsigned long long)k_mem_phys_addr(&data->msi_doorbell), data->msi_used,
		    rd(dev, MSI_MASK), rd(dev, MSI_STATUS), rd(dev, MSI_PAYLOAD),
		    data->last_payload, data->msi_spurious);
	for (int v = 0; v < MSI_VECTORS; v++) {
		if (data->msi[v].count != 0 || (data->msi_used & BIT(v)) != 0) {
			shell_print(sh, "  vector %2d: %u%s", v, data->msi[v].count,
				    data->msi[v].routine != NULL ? "" : " (no handler)");
		}
	}
	shell_print(sh, "error messages: cor %u nonfatal %u fatal %u; INTx a %u b %u c %u d %u",
		    data->err_cor, data->err_nfat, data->err_fat, data->intx[0], data->intx[1],
		    data->intx[2], data->intx[3]);
#if defined(CONFIG_PCIE_AARDVARK_AER)
	shell_print(sh, "AER events serviced: %u (pcie aer)", data->aer_events);
#endif
	shell_print(sh, "PME messages %u, root status %08x (requester %02x:%02x.%x)", data->pme,
		    data->rp.rootsta, (data->rp.rootsta >> 8) & 0xff, (data->rp.rootsta >> 3) & 0x1f,
		    data->rp.rootsta & 7);
}

static int sh_link(const struct device *dev, const struct shell *sh, bool retrain)
{
	struct advk_rc_data *data = dev->data;
	uint32_t lnk;

	if (!data->active) {
		return -ENODEV;
	}
	if (retrain) {
		shell_print(sh, "%s", advk_rc_link_start(dev) ? "link up" : "no link");
		advk_rc_scan(dev);
		shell_print(sh, "rescanned, see: pcie ls");
		return 0;
	}
	lnk = rd(dev, CFG_LNKCTL) >> 16;
	shell_print(sh, "%s, ltssm 0x%02x, Gen%u x%u", link_up(dev) ? "up" : "down", ltssm(dev),
		    lnk & 0xf, (lnk >> 4) & 0x3f);
	return 0;
}

static int sh_scan(const struct device *dev, const struct shell *sh)
{
	struct advk_rc_data *data = dev->data;

	if (!data->active) {
		return -ENODEV;
	}
	advk_rc_scan(dev);
	shell_print(sh, "rescanned, see: pcie ls");
	return 0;
}

static int sh_id_get(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id)
{
	struct advk_rc_data *data = dev->data;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -ENOTSUP; /* only the root port is ours */
	}
	*id = (struct pcie_shell_id){.vid = data->id.vid, .did = data->id.did,
				     .svid = data->id.svid, .ssid = data->id.ssid,
				     .class_code = 0x060400};
	return 0;
}

static int sh_id_set(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id)
{
	struct advk_rc_data *data = dev->data;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -ENOTSUP;
	}
	data->id = (struct rp_id){.vid = id->vid, .did = id->did, .svid = id->svid,
				  .ssid = id->ssid};
	id_apply(dev);
	return IS_ENABLED(CONFIG_SETTINGS) ? settings_save_one(RCID_KEY, &data->id,
							       sizeof(data->id))
					   : 0;
}

static int sh_id_reset(const struct device *dev, pcie_bdf_t bdf)
{
	ARG_UNUSED(dev);
	if (bdf != PCIE_BDF(0, 0, 0)) {
		return -ENOTSUP;
	}
	return IS_ENABLED(CONFIG_SETTINGS) ? settings_delete(RCID_KEY) : -ENOTSUP;
}

static int sh_dsn(const struct device *dev, pcie_bdf_t bdf, const char *arg, uint64_t *v,
		  const char **src)
{
	struct advk_rc_data *data = dev->data;
	int ret = 0;

	if (!data->active || bdf != PCIE_BDF(0, 0, 0)) {
		return -ENOTSUP;
	}
	if (arg == NULL) {
		/* show */
	} else {
		uint64_t n;

		/* reset: back to the start-up default, saved as on the first start */
		if (strcmp(arg, "uid") == 0 || strcmp(arg, "reset") == 0) {
			n = dsn_generate(src);
		} else if (strcmp(arg, "new") == 0) {
			n = dsn_local(sys_rand64_get() ^ (k_cycle_get_64() * 0x9e3779b97f4a7c15ULL));
		} else {
			char *end;

			n = strtoull(arg, &end, 16);
			if (*end != '\0') {
				return -EINVAL;
			}
		}
		dsn_write(dev, n);
		if (IS_ENABLED(CONFIG_SETTINGS)) {
			ret = settings_save_one(RCDSN_KEY, &n, sizeof(n));
		}
	}
	*v = data->dsn;
	return ret;
}

#if defined(CONFIG_PCIE_AARDVARK_MODE)
static int sh_mode_set(const struct device *dev, enum pcie_shell_role role)
{
	ARG_UNUSED(dev);
	return advk_mode_save(role == PCIE_SHELL_ROLE_RC ? ADVK_MODE_RC : ADVK_MODE_EP);
}

static bool sh_mode_saved(const struct device *dev, enum pcie_shell_role *role)
{
	enum advk_mode m;

	ARG_UNUSED(dev);
	if (!advk_mode_saved(&m)) {
		return false;
	}
	*role = m == ADVK_MODE_RC ? PCIE_SHELL_ROLE_RC : PCIE_SHELL_ROLE_EP;
	return true;
}
#endif

static const struct pcie_shell_ctrl rc_shell = {
	.name = "aardvark",
	.dev = RC_DEV,
	.role = PCIE_SHELL_ROLE_RC,
	.active = sh_active,
	.info = sh_info,
#if defined(CONFIG_PCIE_AARDVARK_MODE)
	.mode_set = sh_mode_set,
	.mode_saved = sh_mode_saved,
#endif
	.link = sh_link,
	.scan = sh_scan,
	.irq = sh_irq,
	.id_get = sh_id_get,
	.id_set = sh_id_set,
	.id_reset = sh_id_reset,
	.dsn = sh_dsn,
	.cfg_read = advk_rc_conf_read,
	.cfg_write = advk_rc_conf_write,
};



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
	bool cache;
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
	if (c->cache) {
		sys_cache_data_flush_range(&q->sq[q->tail], sizeof(*cmd));
	}
	q->tail = (q->tail + 1) % NV_QSIZE;
	barrier_dsync_fence_full();
	sys_write32(q->tail, c->bar + q->sq_db);

	t0 = k_uptime_get();
	e = &q->cq[q->head];
	while (true) {
		if (c->cache) {
			sys_cache_data_invd_range(e, sizeof(*e));
		}
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

	if (c->cache) {
		sys_cache_data_invd_range(nv_buf, sizeof(nv_buf));
	}
	return st;
}


static atomic_t nv_msi_count;

static void nv_msi_handler(const void *arg)
{
	ARG_UNUSED(arg);
	atomic_inc(&nv_msi_count);
}

/* MSI-X entry 0 of the device to one of our vectors; returns the vector */
static int nv_msix_on(struct nv_ctx *c, const struct device *dev, pcie_bdf_t bdf,
		      uint64_t bar_phys, uint32_t *cap_out)
{
	uint32_t cap = advk_rc_conf_read(dev, bdf, PCIE_CONF_CAPPTR) & 0xfc;
	uint32_t tbl, addr;
	uint16_t msg;
	uint8_t *va;
	int v;

	for (int ttl = 48; cap >= 0x40 && ttl > 0; ttl--) {
		uint32_t w = advk_rc_conf_read(dev, bdf, cap >> 2);

		if ((w & 0xff) == 0x11) {
			break;
		}
		cap = (w >> 8) & 0xfc;
	}
	if (cap < 0x40) {
		shell_error(c->sh, "no MSI-X capability");
		return -ENODEV;
	}
	tbl = advk_rc_conf_read(dev, bdf, (cap >> 2) + 1);
	if ((tbl & 0x7) != 0) {
		shell_error(c->sh, "MSI-X table in BAR%u, only BAR0 handled", tbl & 0x7);
		return -ENOTSUP;
	}
	v = advk_rc_msi_alloc(dev, nv_msi_handler, NULL, &addr, &msg);
	if (v < 0) {
		return v;
	}
	tbl &= ~0x7U;
	k_mem_map_phys_bare(&va, (bar_phys + tbl) & ~0xfffULL, 0x1000,
			    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	volatile uint32_t *e = (volatile uint32_t *)(va + ((bar_phys + tbl) & 0xfff));

	e[0] = addr;
	e[1] = 0;
	e[2] = msg;
	e[3] = 0; /* unmasked */
	k_mem_unmap_phys_bare(va, 0x1000);

	/* MSI-X enable, function mask off */
	advk_rc_conf_write(dev, bdf, cap >> 2,
			   (advk_rc_conf_read(dev, bdf, cap >> 2) & ~BIT(30)) | BIT(31));
	*cap_out = cap;
	atomic_set(&nv_msi_count, 0);
	shell_print(c->sh, "MSI-X cap 0x%02x, table BAR0+0x%x, entry 0 -> vector %d (addr 0x%x)",
		    cap, tbl, v, addr);
	return v;
}

static int cmd_rc_nvme(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = RC_DEV;
	struct nv_ctx c = {.sh = sh, .cache = true};
	unsigned long long lba = 0;
	bool use_msi = false;
	int msi_v = -1;
	uint32_t msix_cap = 0;

	for (size_t i = 2; i < argc; i++) {
		if (strcmp(argv[i], "msi") == 0) {
			use_msi = true;
		} else if (strcmp(argv[i], "nocache") == 0) {
			c.cache = false;
		} else {
			lba = strtoull(argv[i], NULL, 0);
		}
	}
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
	if (!c.cache) {
		shell_print(sh, "no cache maintenance on queues and data (coherency test)");
	}
	if (use_msi) {
		msi_v = nv_msix_on(&c, dev, bdf, bar_phys, &msix_cap);
		if (msi_v < 0) {
			goto out;
		}
	}

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
	if (c.cache) {
		sys_cache_data_invd_range(nv_buf, sizeof(nv_buf));
	}
	if (st != 0) {
		shell_error(sh, "read LBA %llu: status 0x%x", lba, st);
		goto out;
	}
	shell_print(sh, "LBA %llu (%lld ms):", lba, k_uptime_get() - t0);
	shell_hexdump(sh, nv_buf, MIN(bs, 128U));
	if (bs >= 512 && nv_buf[510] == 0x55 && nv_buf[511] == 0xaa) {
		shell_print(sh, "  55aa at 510: MBR / protective MBR");
	}
	if (use_msi) {
		/* the admin completions interrupt on vector 0; I/O queue 1 has IEN off */
		shell_print(sh, "MSI-X: %ld interrupts for %u admin commands",
			    (long)atomic_get(&nv_msi_count), (unsigned int)c.admin.cid);
	}
out:
	if (msi_v >= 0) {
		advk_rc_conf_write(dev, bdf, msix_cap >> 2,
				   advk_rc_conf_read(dev, bdf, msix_cap >> 2) & ~BIT(31));
		advk_rc_msi_free(dev, msi_v);
	}
	k_mem_unmap_phys_bare((uint8_t *)c.bar, 0x2000);
	return 0;
}
#endif /* CONFIG_PCIE_AARDVARK_NVME_SHELL */

#if defined(CONFIG_PCIE_AARDVARK_EPFTEST_SHELL)
/*
 * Host side of the endpoint test function (Linux pci_endpoint_test /
 * pcitest, the same register block as Linux pci-epf-test and Zephyr
 * pcie_epf_test): BAR patterns, MSI and MSI-X into the root complex's own
 * MSI receiver, and the read / write / copy transfers into RC memory,
 * memcpy or DMA on the endpoint's side, checked with the same CRC32.
 *
 *   pcie debug epftest <b:d.f> [info|bar|msi|msix|read|write|copy|all]
 *                      [<size>] [dma] [<count>]
 *
 * Naming as pcitest: "write" = the host writes, the endpoint reads
 * (COMMAND_READ) and checks; "read" = the endpoint writes into host
 * memory (COMMAND_WRITE) and the host checks.
 */
#define EPT_MAGIC      0x00
#define EPT_COMMAND    0x04
#define EPT_STATUS     0x08
#define EPT_SRC_LO     0x0c
#define EPT_SRC_HI     0x10
#define EPT_DST_LO     0x14
#define EPT_DST_HI     0x18
#define EPT_SIZE       0x1c
#define EPT_CHECKSUM   0x20
#define EPT_IRQ_TYPE   0x24
#define EPT_IRQ_NUMBER 0x28
#define EPT_FLAGS      0x2c
#define EPT_CAPS       0x30

#define EPT_CMD_RAISE_MSI  BIT(1)
#define EPT_CMD_RAISE_MSIX BIT(2)
#define EPT_CMD_READ       BIT(3)
#define EPT_CMD_WRITE      BIT(4)
#define EPT_CMD_COPY       BIT(5)

#define EPT_STS_READ_OK    BIT(0)
#define EPT_STS_READ_FAIL  BIT(1)
#define EPT_STS_WRITE_OK   BIT(2)
#define EPT_STS_WRITE_FAIL BIT(3)
#define EPT_STS_COPY_OK    BIT(4)
#define EPT_STS_COPY_FAIL  BIT(5)
#define EPT_STS_IRQ        BIT(6)
#define EPT_STS_SRC_INVAL  BIT(7)
#define EPT_STS_DST_INVAL  BIT(8)

#define EPT_IRQ_MSI  1
#define EPT_IRQ_MSIX 2

#define EPT_BUF (CONFIG_PCIE_AARDVARK_EPFTEST_BUF_KB * 1024)

static uint8_t ept_src[EPT_BUF] __aligned(4096);
static uint8_t ept_dst[EPT_BUF] __aligned(4096);
static atomic_t ept_irqs;

struct ept {
	const struct shell *sh;
	const struct device *dev;
	pcie_bdf_t bdf;
	uint64_t bar[6];
	uint64_t bar_size[6];
	uint8_t *reg;     /* test register BAR (BAR0), mapped */
	uint32_t irq_type; /* 0: polled */
	int vec;          /* our MSI vector, -1 none */
	uint32_t msix_cap, msi_cap;
};

static void ept_isr(const void *arg)
{
	ARG_UNUSED(arg);
	atomic_inc(&ept_irqs);
}

static uint32_t ept_rd(struct ept *t, uint32_t off)
{
	return sys_read32((mem_addr_t)t->reg + off);
}

static void ept_wr(struct ept *t, uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)t->reg + off);
}

static uint32_t ept_find_cap(struct ept *t, uint8_t id)
{
	uint32_t cap = advk_rc_conf_read(t->dev, t->bdf, PCIE_CONF_CAPPTR) & 0xfc;

	for (int ttl = 48; cap >= 0x40 && ttl > 0; ttl--) {
		uint32_t w = advk_rc_conf_read(t->dev, t->bdf, cap >> 2);

		if ((w & 0xff) == id) {
			return cap;
		}
		cap = (w >> 8) & 0xfc;
	}
	return 0;
}

/* BAR addresses as assigned, sizes by the usual all-ones probe */
static void ept_bars(struct ept *t)
{
	uint32_t cmd = advk_rc_conf_read(t->dev, t->bdf, PCIE_CONF_CMDSTAT);

	/* decoding off while sizing */
	advk_rc_conf_write(t->dev, t->bdf, PCIE_CONF_CMDSTAT, cmd & 0xfff8);
	for (int i = 0; i < 6; i++) {
		unsigned int r = PCIE_CONF_BAR0 + i;
		uint32_t lo = advk_rc_conf_read(t->dev, t->bdf, r), hi = 0, slo, shi = 0xffffffff;
		bool b64 = (lo & 0x7) == 0x4;

		t->bar[i] = 0;
		t->bar_size[i] = 0;
		if ((lo & 1) != 0) {
			continue; /* I/O */
		}
		advk_rc_conf_write(t->dev, t->bdf, r, 0xffffffff);
		slo = advk_rc_conf_read(t->dev, t->bdf, r);
		advk_rc_conf_write(t->dev, t->bdf, r, lo);
		if (b64) {
			hi = advk_rc_conf_read(t->dev, t->bdf, r + 1);
			advk_rc_conf_write(t->dev, t->bdf, r + 1, 0xffffffff);
			shi = advk_rc_conf_read(t->dev, t->bdf, r + 1);
			advk_rc_conf_write(t->dev, t->bdf, r + 1, hi);
		}
		if ((slo & ~0xfU) != 0) {
			uint64_t mask = ((uint64_t)(b64 ? shi : 0xffffffff) << 32) | (slo & ~0xfULL);

			t->bar_size[i] = ~mask + 1;
			t->bar[i] = ((uint64_t)hi << 32) | (lo & ~0xfULL);
		}
		if (b64) {
			i++;
		}
	}
	advk_rc_conf_write(t->dev, t->bdf, PCIE_CONF_CMDSTAT,
			   (cmd & 0xffff) | PCIE_CONF_CMDSTAT_MEM | PCIE_CONF_CMDSTAT_MASTER);
}

static int ept_open(struct ept *t)
{
	if (t->bar_size[0] < 0x40 || t->bar[0] == 0) {
		shell_error(t->sh, "BAR0 (test registers) not assigned");
		return -ENODEV;
	}
	k_mem_map_phys_bare(&t->reg, t->bar[0], 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	t->msix_cap = ept_find_cap(t, 0x11);
	t->msi_cap = ept_find_cap(t, 0x05);
	t->vec = -1;
	return 0;
}

static void ept_close(struct ept *t)
{
	if (t->vec >= 0) {
		if (t->irq_type == EPT_IRQ_MSIX) {
			advk_rc_conf_write(t->dev, t->bdf, t->msix_cap >> 2,
					   advk_rc_conf_read(t->dev, t->bdf, t->msix_cap >> 2) &
						   ~BIT(31));
		} else if (t->irq_type == EPT_IRQ_MSI) {
			advk_rc_conf_write(t->dev, t->bdf, t->msi_cap >> 2,
					   advk_rc_conf_read(t->dev, t->bdf, t->msi_cap >> 2) &
						   ~BIT(16));
		}
		advk_rc_msi_free(t->dev, t->vec);
	}
	k_mem_unmap_phys_bare(t->reg, 0x1000);
}

/* MSI-X: entry <n-1> of the device's table to our vector */
static int ept_msix_setup(struct ept *t, uint32_t n)
{
	uint32_t tbl, addr, ctrl;
	uint16_t msg;
	uint8_t *va;
	uint64_t pa;

	if (t->msix_cap == 0) {
		return -ENODEV;
	}
	ctrl = advk_rc_conf_read(t->dev, t->bdf, t->msix_cap >> 2);
	tbl = advk_rc_conf_read(t->dev, t->bdf, (t->msix_cap >> 2) + 1);
	if (n == 0 || n > ((ctrl >> 16) & 0x7ff) + 1 || (tbl & 7) > 5 || t->bar[tbl & 7] == 0) {
		return -EINVAL;
	}
	if (t->vec < 0) {
		t->vec = advk_rc_msi_alloc(t->dev, ept_isr, NULL, &addr, &msg);
		if (t->vec < 0) {
			return t->vec;
		}
	} else {
		addr = (uint32_t)k_mem_phys_addr(&((struct advk_rc_data *)t->dev->data)->msi_doorbell);
		msg = t->vec;
	}
	pa = t->bar[tbl & 7] + (tbl & ~7U) + (n - 1) * 16;
	k_mem_map_phys_bare(&va, pa & ~0xfffULL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	volatile uint32_t *e = (volatile uint32_t *)(va + (pa & 0xfff));

	e[0] = addr;
	e[1] = 0;
	e[2] = msg;
	e[3] = 0;
	k_mem_unmap_phys_bare(va, 0x1000);
	advk_rc_conf_write(t->dev, t->bdf, t->msix_cap >> 2, (ctrl & ~BIT(30)) | BIT(31));
	t->irq_type = EPT_IRQ_MSIX;
	return 0;
}

/* MSI: one message, data = our vector (the endpoint adds irq - 1) */
static int ept_msi_setup(struct ept *t)
{
	uint32_t ctrl, addr;
	uint16_t msg;
	unsigned int c;

	if (t->msi_cap == 0) {
		return -ENODEV;
	}
	if (t->vec < 0) {
		t->vec = advk_rc_msi_alloc(t->dev, ept_isr, NULL, &addr, &msg);
		if (t->vec < 0) {
			return t->vec;
		}
	} else {
		addr = (uint32_t)k_mem_phys_addr(&((struct advk_rc_data *)t->dev->data)->msi_doorbell);
		msg = t->vec;
	}
	c = t->msi_cap >> 2;
	ctrl = advk_rc_conf_read(t->dev, t->bdf, c);
	advk_rc_conf_write(t->dev, t->bdf, c + 1, addr);
	if ((ctrl & BIT(23)) != 0) {
		advk_rc_conf_write(t->dev, t->bdf, c + 2, 0);
		advk_rc_conf_write(t->dev, t->bdf, c + 3, msg);
	} else {
		advk_rc_conf_write(t->dev, t->bdf, c + 2, msg);
	}
	/* MME = 0 (one vector), enable */
	advk_rc_conf_write(t->dev, t->bdf, c, (ctrl & ~(0x7U << 20)) | BIT(16));
	t->irq_type = EPT_IRQ_MSI;
	return 0;
}

/* Run one command, wait for a status bit; returns the status */
static uint32_t ept_run(struct ept *t, uint32_t cmd, uint32_t irq_num, uint32_t done_mask,
			uint32_t ms, int64_t *us, bool *irq_seen)
{
	atomic_val_t i0 = atomic_get(&ept_irqs);
	int64_t t0, end;
	uint32_t st;

	ept_wr(t, EPT_IRQ_TYPE, t->irq_type == 0 ? EPT_IRQ_MSIX : t->irq_type);
	ept_wr(t, EPT_IRQ_NUMBER, irq_num);
	ept_wr(t, EPT_STATUS, 0);
	barrier_dsync_fence_full();
	t0 = k_uptime_ticks();
	end = k_uptime_get() + ms;
	ept_wr(t, EPT_COMMAND, cmd);
	do {
		st = ept_rd(t, EPT_STATUS);
		if ((st & done_mask) != 0) {
			break;
		}
		k_busy_wait(20);
	} while (k_uptime_get() < end);
	*us = k_ticks_to_us_floor64(k_uptime_ticks() - t0);
	/* the interrupt follows the status write */
	for (int w = 0; w < 100 && t->irq_type != 0 && atomic_get(&ept_irqs) == i0; w++) {
		k_busy_wait(100);
	}
	if (irq_seen != NULL) {
		*irq_seen = atomic_get(&ept_irqs) != i0;
	}
	return st;
}

static void ept_fill(uint8_t *p, size_t n)
{
	static uint32_t x = 2463534242U;

	x ^= (uint32_t)k_cycle_get_32();
	for (size_t i = 0; i < n; i++) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		p[i] = (uint8_t)x;
	}
}

static const char *ept_irq_str(struct ept *t, bool seen)
{
	if (t->irq_type == 0) {
		return "";
	}
	return seen ? ", irq ok" : ", NO irq";
}

static int ept_xfer(struct ept *t, const char *what, size_t size, bool dma)
{
	uint64_t src = k_mem_phys_addr(ept_src), dst = k_mem_phys_addr(ept_dst);
	uint32_t cmd, ok, fail, st;
	bool irq_seen;
	int64_t us;
	int ret = 0;

	if (size == 0 || size > EPT_BUF) {
		shell_error(t->sh, "size 1..%u", EPT_BUF);
		return -EINVAL;
	}
	ept_wr(t, EPT_FLAGS, dma ? BIT(0) : 0);
	ept_wr(t, EPT_SIZE, (uint32_t)size);
	if (strcmp(what, "write") == 0) {
		/* host -> endpoint: the endpoint reads src and checks the CRC */
		ept_fill(ept_src, size);
		sys_cache_data_flush_range(ept_src, size);
		ept_wr(t, EPT_SRC_LO, (uint32_t)src);
		ept_wr(t, EPT_SRC_HI, (uint32_t)(src >> 32));
		ept_wr(t, EPT_CHECKSUM, ~crc32_ieee(ept_src, size));
		cmd = EPT_CMD_READ;
		ok = EPT_STS_READ_OK;
		fail = EPT_STS_READ_FAIL;
	} else if (strcmp(what, "read") == 0) {
		/* endpoint -> host: it fills dst and publishes the CRC */
		memset(ept_dst, 0, size);
		sys_cache_data_flush_and_invd_range(ept_dst, size);
		ept_wr(t, EPT_DST_LO, (uint32_t)dst);
		ept_wr(t, EPT_DST_HI, (uint32_t)(dst >> 32));
		cmd = EPT_CMD_WRITE;
		ok = EPT_STS_WRITE_OK;
		fail = EPT_STS_WRITE_FAIL;
	} else {
		ept_fill(ept_src, size);
		memset(ept_dst, 0, size);
		sys_cache_data_flush_range(ept_src, size);
		sys_cache_data_flush_and_invd_range(ept_dst, size);
		ept_wr(t, EPT_SRC_LO, (uint32_t)src);
		ept_wr(t, EPT_SRC_HI, (uint32_t)(src >> 32));
		ept_wr(t, EPT_DST_LO, (uint32_t)dst);
		ept_wr(t, EPT_DST_HI, (uint32_t)(dst >> 32));
		cmd = EPT_CMD_COPY;
		ok = EPT_STS_COPY_OK;
		fail = EPT_STS_COPY_FAIL;
	}

	st = ept_run(t, cmd, 1, ok | fail | EPT_STS_SRC_INVAL | EPT_STS_DST_INVAL, 10000, &us,
		     &irq_seen);
	if ((st & ok) == 0) {
		shell_error(t->sh, "%-5s %7zu B %-6s FAIL status %08x%s", what, size,
			    dma ? "dma" : "memcpy", st, (st & (ok | fail)) == 0 ? " (timeout)" : "");
		return -EIO;
	}
	if (cmd == EPT_CMD_WRITE) {
		sys_cache_data_invd_range(ept_dst, size);
		if (~crc32_ieee(ept_dst, size) != ept_rd(t, EPT_CHECKSUM)) {
			ret = -EIO;
		}
	} else if (cmd == EPT_CMD_COPY) {
		sys_cache_data_invd_range(ept_dst, size);
		if (memcmp(ept_src, ept_dst, size) != 0) {
			ret = -EIO;
		}
	}
	shell_print(t->sh, "%-5s %7zu B %-6s %s, %lld us, %llu kB/s%s", what, size,
		    dma ? "dma" : "memcpy", ret == 0 ? "ok" : "DATA MISMATCH", us,
		    us > 0 ? (unsigned long long)(size * 1000ULL / (uint64_t)us) : 0ULL,
		    ept_irq_str(t, irq_seen));
	return ret;
}

static int ept_bar_test(struct ept *t, size_t max)
{
	static const uint32_t pat[6] = {0xa0a0a0a0, 0xa1a1a1a1, 0xa2a2a2a2,
					0xa3a3a3a3, 0xa4a4a4a4, 0xa5a5a5a5};
	int fails = 0;

	for (int i = 0; i < 6; i++) {
		size_t n;
		uint8_t *va;
		uint32_t bad = 0, first_bad = 0;

		if (t->bar_size[i] == 0) {
			continue;
		}
		/* the test register BAR: only its first word (as pci_endpoint_test) */
		n = i == 0 ? 4 : (size_t)MIN((uint64_t)max, t->bar_size[i]);
		k_mem_map_phys_bare(&va, t->bar[i], ROUND_UP(n, 4096),
				    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
		uint32_t keep = i == 0 ? sys_read32((mem_addr_t)va) : 0;

		for (size_t o = 0; o < n; o += 4) {
			sys_write32(pat[i], (mem_addr_t)va + o);
		}
		for (size_t o = 0; o < n; o += 4) {
			uint32_t v = sys_read32((mem_addr_t)va + o);

			if (v != pat[i]) {
				if (bad++ == 0) {
					first_bad = (uint32_t)o;
				}
			}
		}
		if (i == 0) {
			sys_write32(keep, (mem_addr_t)va);
		}
		k_mem_unmap_phys_bare(va, ROUND_UP(n, 4096));
		if (bad != 0) {
			shell_error(t->sh, "BAR%d 0x%llx: %u of %zu words wrong (first +0x%x)", i,
				    (unsigned long long)t->bar[i], bad, n / 4, first_bad);
			fails++;
		} else {
			shell_print(t->sh, "BAR%d 0x%llx (%llu KiB): %zu B pattern ok", i,
				    (unsigned long long)t->bar[i],
				    (unsigned long long)(t->bar_size[i] / 1024), n);
		}
	}
	return fails != 0 ? -EIO : 0;
}

static int ept_irq_test(struct ept *t, bool msix, uint32_t count)
{
	int ret = msix ? ept_msix_setup(t, 1) : ept_msi_setup(t);
	uint32_t got = 0;

	if (ret != 0) {
		shell_error(t->sh, "%s setup: %d", msix ? "MSI-X" : "MSI", ret);
		return ret;
	}
	for (uint32_t n = 1; n <= count; n++) {
		bool seen;
		int64_t us;
		uint32_t st;

		if (msix && n > 1) {
			ret = ept_msix_setup(t, n);
			if (ret != 0) {
				break;
			}
		}
		st = ept_run(t, msix ? EPT_CMD_RAISE_MSIX : EPT_CMD_RAISE_MSI, msix ? n : 1,
			     EPT_STS_IRQ, 1000, &us, &seen);
		if ((st & EPT_STS_IRQ) != 0 && seen) {
			got++;
		} else {
			shell_error(t->sh, "%s %u: status %08x, %s", msix ? "MSI-X" : "MSI", n, st,
				    seen ? "irq" : "no irq");
		}
	}
	shell_print(t->sh, "%s: %u/%u interrupts received (vector %d)", msix ? "MSI-X" : "MSI",
		    got, count, t->vec);
	return got == count ? 0 : -EIO;
}

static int cmd_rc_epftest(const struct shell *sh, size_t argc, char **argv)
{
	struct ept t = {.sh = sh, .dev = RC_DEV};
	const char *what = argc > 2 ? argv[2] : "info";
	unsigned long size = 0, count = 0;
	bool dma = false;
	int ret = 0;

	if (!rc_active(sh) || parse_bdf(sh, argv[1], &t.bdf) != 0) {
		return -EINVAL;
	}
	for (size_t i = 3; i < argc; i++) {
		if (strcmp(argv[i], "dma") == 0) {
			dma = true;
		} else if (size == 0) {
			size = strtoul(argv[i], NULL, 0);
		} else {
			count = strtoul(argv[i], NULL, 0);
		}
	}
	if (advk_rc_conf_read(RC_DEV, t.bdf, PCIE_CONF_ID) == 0xffffffff) {
		shell_error(sh, "no device at %s", argv[1]);
		return -ENODEV;
	}
	ept_bars(&t);
	ret = ept_open(&t);
	if (ret != 0) {
		return ret;
	}

	if (strcmp(what, "info") == 0) {
		shell_print(sh, "magic %08x caps %08x status %08x", ept_rd(&t, EPT_MAGIC),
			    ept_rd(&t, EPT_CAPS), ept_rd(&t, EPT_STATUS));
		for (int i = 0; i < 6; i++) {
			if (t.bar_size[i] != 0) {
				shell_print(sh, "BAR%d 0x%llx, %llu KiB", i,
					    (unsigned long long)t.bar[i],
					    (unsigned long long)(t.bar_size[i] / 1024));
			}
		}
		shell_print(sh, "MSI cap 0x%02x, MSI-X cap 0x%02x, host buffers 2 x %u KiB at 0x%llx",
			    t.msi_cap, t.msix_cap, EPT_BUF / 1024,
			    (unsigned long long)k_mem_phys_addr(ept_src));
	} else if (strcmp(what, "bar") == 0) {
		ret = ept_bar_test(&t, size != 0 ? size : 65536);
	} else if (strcmp(what, "msi") == 0 || strcmp(what, "msix") == 0) {
		ret = ept_irq_test(&t, what[3] == 'x', size != 0 ? size : 1);
	} else if (strcmp(what, "read") == 0 || strcmp(what, "write") == 0 ||
		   strcmp(what, "copy") == 0) {
		(void)ept_msix_setup(&t, 1);
		for (unsigned long i = 0; i < MAX(count, 1UL) && ret == 0; i++) {
			ret = ept_xfer(&t, what, size != 0 ? size : 65536, dma);
		}
	} else if (strcmp(what, "all") == 0) {
		static const size_t sizes[] = {1, 1024, 65536, 1024 * 1024};
		static const char *const ops[] = {"read", "write", "copy"};
		int fails = 0;

		fails += ept_bar_test(&t, 65536) != 0;
		fails += ept_irq_test(&t, false, 1) != 0;
		ept_close(&t);
		(void)ept_open(&t);
		fails += ept_irq_test(&t, true, 16) != 0;
		for (int d = 0; d < 2; d++) {
			for (size_t o = 0; o < ARRAY_SIZE(ops); o++) {
				for (size_t s = 0; s < ARRAY_SIZE(sizes); s++) {
					if (sizes[s] <= EPT_BUF) {
						fails += ept_xfer(&t, ops[o], sizes[s], d) != 0;
					}
				}
			}
		}
		shell_print(sh, "all: %d failed", fails);
		ret = fails != 0 ? -EIO : 0;
	} else {
		shell_error(sh, "info|bar|msi|msix|read|write|copy|all");
		ret = -EINVAL;
	}
	ept_close(&t);
	return ret;
}
#endif /* CONFIG_PCIE_AARDVARK_EPFTEST_SHELL */


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
	irq_setup(RC_DEV);
	id_load(RC_DEV);
	dsn_load(RC_DEV);
	data->active = true;
	shell_print(sh, "%s", advk_rc_link_start(RC_DEV) ? "link up" : "no link");
	advk_rc_scan(RC_DEV);
	return 0;
}

/*
 * Hot reset of the link (Secondary Bus Reset of the root port): TS1 with
 * the Hot Reset bit, the device resets without PERST#. Then the link
 * trains again, 100 ms for the device, and the tree is enumerated again.
 */
static int cmd_rc_hotreset(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = RC_DEV;
	int64_t t0;
	uint32_t bc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (!rc_active(sh)) {
		return -ENODEV;
	}
	bc = advk_rc_conf_read(dev, PCIE_BDF(0, 0, 0), 0x3c / 4);
	advk_rc_conf_write(dev, PCIE_BDF(0, 0, 0), 0x3c / 4, bc | BIT(16 + 6));
	k_msleep(2); /* Trst >= 1 ms */
	shell_print(sh, "hot reset: ltssm 0x%02x", ltssm(dev));
	advk_rc_conf_write(dev, PCIE_BDF(0, 0, 0), 0x3c / 4, bc & ~BIT(16 + 6));
	t0 = k_uptime_get();
	while (!link_up(dev)) {
		if (k_uptime_get() - t0 > CONFIG_PCIE_AARDVARK_LINK_TIMEOUT_MS) {
			shell_error(sh, "no link after %d ms, ltssm 0x%02x",
				    CONFIG_PCIE_AARDVARK_LINK_TIMEOUT_MS, ltssm(dev));
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}
	shell_print(sh, "link up in %lld ms", k_uptime_get() - t0);
	k_msleep(100);
	advk_rc_scan(dev);
	shell_print(sh, "rescanned, see: pcie ls");
	return 0;
}

/* pcie debug bringup ...: the init steps one by one */
SHELL_STATIC_SUBCMD_SET_CREATE(rc_bringup_cmds,
	SHELL_CMD_ARG(probe, NULL, "Do the core registers answer?", cmd_rc_probe, 1, 0),
	SHELL_CMD_ARG(pinctrl, NULL, "Apply the default pin state", cmd_rc_pinctrl, 1, 0),
	SHELL_CMD_ARG(clkout, NULL, "on|off COMPHY 100 MHz output", cmd_rc_clkout, 2, 0),
	SHELL_CMD_ARG(perst, NULL, "assert|release|input PERST# GPIO", cmd_rc_perst, 2, 0),
	SHELL_CMD_ARG(comphy, NULL, "COMPHY power on again (SiP)", cmd_rc_comphy, 1, 0),
	SHELL_CMD_ARG(start, NULL, "Setup, windows, training, scan from the current state",
		      cmd_rc_start, 1, 0),
	SHELL_SUBCMD_SET_END);
SHELL_SUBCMD_ADD((pcie_debug), bringup, &rc_bringup_cmds,
		 "Root complex bring-up step by step", NULL, 1, 0);
SHELL_SUBCMD_ADD((pcie_debug), hotreset, NULL,
		 "Hot reset of the link (Secondary Bus Reset), retrain, rescan",
		 cmd_rc_hotreset, 1, 0);
#if defined(CONFIG_PCIE_AARDVARK_NVME_SHELL)
SHELL_SUBCMD_ADD((pcie_debug), nvme, NULL,
		 "<b:d.f> [<lba>] [msi] [nocache] Polled NVMe test: identify, read one block; "
		 "msi: count MSI-X of the admin queue; nocache: no cache maintenance",
		 cmd_rc_nvme, 2, 3);
#endif
#if defined(CONFIG_PCIE_AARDVARK_EPFTEST_SHELL)
SHELL_SUBCMD_ADD((pcie_debug), epftest, NULL,
		 "<b:d.f> [info|bar|msi|msix|read|write|copy|all] [<size>|<count>] [dma] [<count>] "
		 "Host side of the endpoint test function (as Linux pcitest)",
		 cmd_rc_epftest, 2, 4);
#endif
#endif /* CONFIG_PCIE_AARDVARK_SHELL */

/* ------------------------------------------------------------------------ */

#define ADVK_RC_HAS_PINCTRL(n) DT_INST_PINCTRL_HAS_NAME(n, default)

#define ADVK_RC_DEFINE(n)                                                                          \
	IF_ENABLED(ADVK_RC_HAS_PINCTRL(n), (PINCTRL_DT_INST_DEFINE(n);))                          \
	static void advk_rc_irq_config_##n(const struct device *dev)                               \
	{                                                                                          \
		static bool done;                                                                  \
		if (!done) {                                                                       \
			IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), advk_rc_isr,         \
				    DEVICE_DT_INST_GET(n), DT_INST_IRQ(n, flags));                 \
			done = true;                                                               \
		}                                                                                  \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
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
		.irq_config = advk_rc_irq_config_##n,                                              \
	};                                                                                         \
	static struct advk_rc_data advk_rc_data_##n;                                               \
	DEVICE_DT_INST_DEFINE(n, advk_rc_init, NULL, &advk_rc_data_##n, &advk_rc_cfg_##n,          \
			      POST_KERNEL, CONFIG_PCIE_AARDVARK_INIT_PRIORITY, &advk_rc_api);

DT_INST_FOREACH_STATUS_OKAY(ADVK_RC_DEFINE)
