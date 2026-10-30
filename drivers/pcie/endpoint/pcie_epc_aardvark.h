/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 3700 "Aardvark" PCIe controller in endpoint mode.
 *
 * Register offsets are the ones Linux pci-aardvark.c uses for the root
 * complex. Marvell published no endpoint documentation, so everything
 * endpoint specific (BAR sizing, inbound translation, INTx generation)
 * is marked as such and expected to be found on hardware with the shell.
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

/* Core: the function's own configuration space as the host sees it */
#define ADVK_CFG_ID          0x00
#define ADVK_CFG_CMD_STATUS  0x04
#define ADVK_CFG_REV_CLASS   0x08
#define ADVK_CFG_BIST_HDR    0x0c
#define ADVK_CFG_BAR(n)      (0x10 + 4 * (n))
#define ADVK_CFG_SSID        0x2c
#define ADVK_CFG_CAP_PTR     0x34
#define ADVK_CFG_INTR        0x3c
#define ADVK_CFG_PCIE_CAP    0xc0
/* Device Serial Number extended capability (0x150): lower, upper dword */
#define ADVK_CFG_DSN_LO      0x154
#define ADVK_CFG_DSN_HI      0x158
#define ADVK_CFG_LNKCTL_STA  (ADVK_CFG_PCIE_CAP + 0x10)
#define ADVK_CFG_LNKCTL2     (ADVK_CFG_PCIE_CAP + 0x30)

#define ADVK_CMD_MEM    BIT(1)
#define ADVK_CMD_MASTER BIT(2)
#define ADVK_STS_CAPLIST BIT(20)

#define ADVK_CAP_ID_MSI 0x05
#define ADVK_CAP_ID_MSIX 0x11
/* MSI-X capability (0xb0 on the A3720): table and PBA live in a BAR, i.e.
 * in local memory the host writes; locally all of it is writable.
 */
#define ADVK_MSIX_CTRL_EN    BIT(31)
#define ADVK_MSIX_CTRL_FMASK BIT(30)
#define ADVK_MSIX_CTRL_TSIZE GENMASK(26, 16)
#define ADVK_MSIX_MAX        2048
#define ADVK_MSI_CTRL_EN    BIT(16)
#define ADVK_MSI_CTRL_MMC   GENMASK(19, 17)
#define ADVK_MSI_CTRL_MME   GENMASK(22, 20)
#define ADVK_MSI_CTRL_64BIT BIT(23)

/* Control */
#define ADVK_CTRL0            0x4800
#define ADVK_CTRL0_GEN_MASK   GENMASK(1, 0)
#define ADVK_CTRL0_IS_RC      BIT(2)
#define ADVK_CTRL0_LANES_MASK GENMASK(4, 3)
#define ADVK_CTRL0_LINK_TRAIN BIT(6)
/* Defined but unused by the RC driver; presumably the EP INTx wires */
#define ADVK_CTRL0_INTA       BIT(28)
#define ADVK_CTRL1            0x4804
#define ADVK_CTRL2            0x4808
#define ADVK_CTRL2_RESERVED   0x7
#define ADVK_CTRL2_TD         BIT(4)
#define ADVK_CTRL2_OB_WIN_EN  BIT(6)
#define ADVK_REF_CLK          0x4814
#define ADVK_REF_CLK_TX       BIT(1)
#define ADVK_REF_CLK_RX       BIT(2)
#define ADVK_MSG_LOG          0x4830
#define ADVK_ISR0             0x4840
#define ADVK_ISR0_MASK        0x4844
#define ADVK_ISR1             0x4848
#define ADVK_ISR1_MASK        0x484c

/* Outbound windows: 8 x 0x20, then the default actions */
#define ADVK_OB_WIN_COUNT        8
#define ADVK_OB_WIN(w, off)      (0x4c00 + 0x20 * (w) + (off))
#define ADVK_OB_MATCH_LS(w)      ADVK_OB_WIN(w, 0x00)
#define ADVK_OB_MATCH_MS(w)      ADVK_OB_WIN(w, 0x04)
#define ADVK_OB_REMAP_LS(w)      ADVK_OB_WIN(w, 0x08)
#define ADVK_OB_REMAP_MS(w)      ADVK_OB_WIN(w, 0x0c)
#define ADVK_OB_MASK_LS(w)       ADVK_OB_WIN(w, 0x10)
#define ADVK_OB_MASK_MS(w)       ADVK_OB_WIN(w, 0x14)
#define ADVK_OB_ACTIONS(w)       ADVK_OB_WIN(w, 0x18)
#define ADVK_OB_DEFAULT_ACTIONS  (ADVK_OB_ACTIONS(ADVK_OB_WIN_COUNT - 1) + 4)
#define ADVK_OB_WIN_ENABLE       BIT(0)
#define ADVK_OB_TYPE_MEM         0x0
#define ADVK_OB_FUNC_NUM_ENABLE  BIT(23)
#define ADVK_OB_FUNC_NUM_SHIFT   24

/*
 * Inbound translation, found on hardware (2026-10-29): per memory BAR a
 * 64-bit AXI base, low word bits 31:7 (+ bits 2:1 of unknown use), high
 * word bits 31:7. BAR offset bits below the BAR size come from the TLP,
 * so the base is effectively aligned down to the BAR size. Reset value
 * 0: BAR offset N = AXI address N.
 *   0x5800/0x5804 BAR0, 0x5808/0x580c BAR2 (I/O, untested), 0x5810/0x5814 BAR4
 * The other entries in 0x5800-0x5fff (+0x20, +0x40, +0x80, +0xa0, and the
 * blocks at 0x5a00, 0x5c00, 0x5e00) have the same shape; not understood.
 */
#define ADVK_IB_BAR_LO(bar)   (0x5800 + 4 * (bar))
#define ADVK_IB_BAR_HI(bar)   (0x5800 + 4 * (bar) + 4)
#define ADVK_IB_ADDR_MASK     0xffffff80U

/* Local management interface */
#define ADVK_LMI_CFG          0x6000
#define ADVK_LMI_LTSSM_SHIFT  24
#define ADVK_LMI_LTSSM_MASK   0x3f
#define ADVK_LMI_VENDOR_ID    0x6044

#define ADVK_LTSSM_L0          0x10
#define ADVK_LTSSM_DISABLED    0x20
#define ADVK_LTSSM_HOT_RESET   0x27

/* Core control */
#define ADVK_CTRL_CONFIG      0x18000
#define ADVK_CTRL_MODE_CMD    BIT(0)

/* Central interrupt */
#define ADVK_HOST_INT_STATUS  0x1b000
#define ADVK_HOST_INT_MASK    0x1b004
#define ADVK_HOST_INT_EP_STATUS BIT(23)

#define ADVK_REG_SPACE        0x20000

/* TF-A SiP service, encoding as in TF-A phy-comphy-common.h */
#define ADVK_SIP_COMPHY_POWER_ON  0x82000001
#define ADVK_SIP_COMPHY_POWER_OFF 0x82000002
#define ADVK_COMPHY_MODE_PCIE     0x6
#define ADVK_COMPHY_SPEED_5G      0x3
#define ADVK_COMPHY_PARAM(invert, width)                                                           \
	(((ADVK_COMPHY_MODE_PCIE) << 12) | ((ADVK_COMPHY_SPEED_5G) << 2) | ((width) << 18) |       \
	 ((invert) & 0x3))

/*
 * COMPHY lane 1 (PCIe/GbE0) registers, 16 bit at reg * 2. MISC_CTRL0
 * bit 4 enables the PHY's 100/125 MHz clock output, which drives the
 * REFCLK pins; TF-A turns it on, an endpoint must not drive the clock
 * (ESPRESSObin wiki "Enabling ESPRESSObin board as PCIe Endpoint").
 */
#define ADVK_COMPHY_PWR_PLL_CTRL     (0x01 * 2)
#define ADVK_COMPHY_PU_PLL_RX_TX     (BIT(14) | BIT(13) | BIT(12))
#define ADVK_COMPHY_MISC_CTRL0       (0x4f * 2)
#define ADVK_COMPHY_CLK100M_125M_EN  BIT(4)

/* Minimal size of an outbound window */
#define ADVK_OB_MIN_SIZE 0x10000

#define ADVK_LTSSM_HIST 16

struct advk_ep {
	/* filled before advk_ep_init() */
	mm_reg_t reg;
	uintptr_t ob_virt;
	uint64_t ob_phys;
	size_t ob_size;
	uint8_t num_lanes;
	uint8_t max_speed;
	/* REFCLK pins as input instead of the RC default (TX on, RX off) */
	bool refclk_in;

	/* private */
	struct k_spinlock lock;
	uintptr_t ob_cpu[ADVK_OB_WIN_COUNT];
	uint8_t msi_cap;
	struct pcie_epc_features features;
	struct pcie_epc_header hdr;
	bool hdr_valid;
	uint64_t dsn;
	bool dsn_valid;
	struct pcie_epc_bar bar[PCIE_EPC_NUM_BARS];
	/* hardware BAR sizes, from the writable bits at init */
	size_t bar_hw_size[PCIE_EPC_NUM_BARS];
	uint8_t msi_mmc;
	uint32_t msi_sent;
	/* MSI-X: capability, layout set by set_msix, local view of table + PBA */
	uint8_t msix_cap;
	uint8_t msix_bir;
	uint16_t msix_nr;
	uint32_t msix_off;
	volatile uint32_t *msix_tbl;
	uint8_t *msix_va;
	size_t msix_map_size;
	uint32_t msix_sent;

	/* link state seen by the last poll */
	uint8_t ltssm;
	bool link_up;
	bool bme;
	uint32_t link_ups;
	uint32_t link_downs;
	uint32_t hot_resets;
	uint32_t reapplies;
	struct {
		uint8_t state;
		uint32_t ms;
	} hist[ADVK_LTSSM_HIST];
	uint8_t hist_head;
};

/* dev->data of the driver; the EPC core needs its part first */
struct advk_ep_data {
	struct pcie_epc_data common;
	struct advk_ep ep;
};

static inline struct advk_ep *advk_ep(const struct device *dev)
{
	return &((struct advk_ep_data *)dev->data)->ep;
}

static inline uint32_t advk_rd(const struct advk_ep *ep, uint32_t off)
{
	return sys_read32(ep->reg + off);
}

static inline void advk_wr(const struct advk_ep *ep, uint32_t off, uint32_t val)
{
	sys_write32(val, ep->reg + off);
}

static inline uint8_t advk_ltssm(const struct advk_ep *ep)
{
	return (advk_rd(ep, ADVK_LMI_CFG) >> ADVK_LMI_LTSSM_SHIFT) & ADVK_LMI_LTSSM_MASK;
}

static inline bool advk_ltssm_is_up(uint8_t s)
{
	return s >= ADVK_LTSSM_L0 && s < ADVK_LTSSM_DISABLED;
}

const char *advk_ltssm_name(uint8_t state);

/*
 * Outbound window registers for AXI address axi, PCI address pci and a
 * power of two size, as Linux advk_pcie_set_ob_win(). Returns -EINVAL
 * for a size that is not a power of two >= ADVK_OB_MIN_SIZE or a
 * misaligned address.
 */
int advk_ob_encode(uint64_t axi, uint64_t pci, size_t size, uint32_t regs[7]);

/* Controller programming without the SoC glue (clock, COMPHY, IRQ) */
void advk_ep_hw_setup(const struct device *dev);
/* Controller setup again plus the header and MSI capability last written */
void advk_ep_restore(const struct device *dev);
int advk_ep_init(const struct device *dev, bool autostart);

/* One poll of the link: LTSSM history, events to the EPC core */
void advk_ep_poll(const struct device *dev, uint32_t now_ms);

/* pcie_epc_driver_api */
int advk_ep_start(const struct device *dev);
void advk_ep_stop(const struct device *dev);
int advk_ep_write_header(const struct device *dev, uint8_t func,
			 const struct pcie_epc_header *hdr);
int advk_ep_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar);
void advk_ep_clear_bar(const struct device *dev, uint8_t func, uint8_t barno);
int advk_ep_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
		     uint64_t pci_addr, size_t size);
void advk_ep_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr);
int advk_ep_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs);
int advk_ep_get_msi(const struct device *dev, uint8_t func);
int advk_ep_set_msix(const struct device *dev, uint8_t func, uint16_t nr_irqs, uint8_t bir,
		     uint32_t offset);
int advk_ep_get_msix(const struct device *dev, uint8_t func);
int advk_ep_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
		      uint16_t irq_num);
int advk_ep_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val);
int advk_ep_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val);
const struct pcie_epc_features *advk_ep_get_features(const struct device *dev, uint8_t func);
bool advk_ep_link_up(const struct device *dev);
int advk_ep_set_dsn(const struct device *dev, uint8_t func, uint64_t dsn);
uint64_t advk_ep_cpu_to_axi(const struct device *dev, uintptr_t cpu_addr);

#endif /* ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_H_ */
