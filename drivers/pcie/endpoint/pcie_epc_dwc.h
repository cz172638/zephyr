/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synopsys DesignWare PCIe controller in endpoint mode: the part shared by
 * the SoC glue drivers (configuration header, BARs through the DBI2 shadow,
 * inbound and outbound iATU regions, MSI, link state).
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_DWC_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_DWC_H_

#include <zephyr/device.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

/* Type 0 header */
#define DWC_CFG_ID         0x00
#define DWC_CFG_CMD_STATUS 0x04
#define DWC_CMD_MEM        BIT(1)
#define DWC_CMD_MASTER     BIT(2)
#define DWC_STS_CAPLIST    BIT(20)
#define DWC_CFG_REV_CLASS  0x08
#define DWC_CFG_BIST_HDR   0x0c
#define DWC_CFG_BAR(n)     (0x10 + 4 * (n))
#define DWC_CFG_SSID       0x2c
#define DWC_CFG_CAP_PTR    0x34
#define DWC_CFG_INTR       0x3c

#define DWC_CAP_ID_MSI  0x05
#define DWC_CAP_ID_EXP  0x10
#define DWC_MSI_CTRL_EN    BIT(16)
#define DWC_MSI_CTRL_MMC   GENMASK(19, 17)
#define DWC_MSI_CTRL_MME   GENMASK(22, 20)
#define DWC_MSI_CTRL_64BIT BIT(23)
#define DWC_EXP_LNKCAP     0x0c
#define DWC_EXP_LNKCTL_STA 0x10
#define DWC_EXP_LNKCTL2    0x30

/* Port logic */
#define DWC_PL_DEBUG0           0x728
#define DWC_PL_DEBUG0_LTSSM     GENMASK(5, 0)
#define DWC_PL_DEBUG1           0x72c
#define DWC_PL_DEBUG1_LINK_UP   BIT(4)
#define DWC_PL_DEBUG1_TRAINING  BIT(29)
#define DWC_MISC_CTRL1          0x8bc
#define DWC_MISC_DBI_RO_WR_EN   BIT(0)
#define DWC_VERSION_NUMBER      0x8f8

/* iATU, viewport (cores before 4.80a) or unrolled */
#define DWC_ATU_VIEWPORT        0x900
#define DWC_ATU_VIEWPORT_BASE   0x904
#define DWC_ATU_DIR_IB          BIT(31)
#define DWC_ATU_CTRL1           0x00
#define DWC_ATU_CTRL1_FUNC(f)   ((uint32_t)(f) << 20)
#define DWC_ATU_CTRL2           0x04
#define DWC_ATU_ENABLE          BIT(31)
#define DWC_ATU_BAR_MODE        BIT(30)
#define DWC_ATU_BAR(b)          ((uint32_t)(b) << 8)
#define DWC_ATU_LOWER_BASE      0x08
#define DWC_ATU_UPPER_BASE      0x0c
#define DWC_ATU_LIMIT           0x10
#define DWC_ATU_LOWER_TARGET    0x14
#define DWC_ATU_UPPER_TARGET    0x18
#define DWC_ATU_UNROLL_OFFSET   0x300000
#define DWC_ATU_UNROLL(dir, i)  (((uint32_t)(i) << 9) | ((dir) != 0 ? BIT(8) : 0))
#define DWC_ATU_MAX_REGIONS     16
#define DWC_ATU_MIN_SIZE        0x1000

/* LTSSM states of PL_DEBUG0 */
#define DWC_LTSSM_L0        0x11
#define DWC_LTSSM_HOT_RESET 0x1f

struct dwc_ep {
	/* filled by the glue before dwc_ep_init() */
	mm_reg_t dbi;
	mm_reg_t dbi2;
	/* unrolled iATU, 0 to find it at DBI + 0x300000 if the viewport is absent */
	mm_reg_t atu;
	uintptr_t ob_virt;
	uint64_t ob_phys;
	size_t ob_size;
	uint8_t max_speed;
	/* BARs that are 64-bit (and prefetchable) in hardware */
	uint8_t bar_fixed_64bit;

	/* private */
	struct k_spinlock lock;
	struct k_mutex irq_lock;
	struct pcie_epc_features features;
	bool unrolled;
	uint8_t num_ib;
	uint8_t num_ob;
	uint8_t msi_cap;
	uint8_t exp_cap;
	uint32_t lnkcap;
	uintptr_t ob_cpu[DWC_ATU_MAX_REGIONS];
	struct pcie_epc_bar bar[PCIE_EPC_NUM_BARS];
	struct pcie_epc_header hdr;
	bool hdr_valid;
	uint8_t msi_mmc;
	struct pcie_epc_host_map irq_map;
	uint64_t irq_map_page;
	bool irq_map_valid;
	bool irq_map_stale;
	uint32_t msi_sent;
	/* link poll */
	uint8_t ltssm;
	bool link_up;
	bool bme;
	uint32_t link_ups;
	uint32_t link_downs;
	uint32_t hot_resets;
};

/* dev->data of a DWC glue driver starts with this */
struct dwc_ep_data {
	struct pcie_epc_data common;
	struct dwc_ep ep;
};

static inline struct dwc_ep *dwc_ep(const struct device *dev)
{
	return &((struct dwc_ep_data *)dev->data)->ep;
}

static inline uint32_t dwc_rd(struct dwc_ep *ep, uint32_t off)
{
	return sys_read32(ep->dbi + off);
}

static inline void dwc_wr(struct dwc_ep *ep, uint32_t off, uint32_t v)
{
	sys_write32(v, ep->dbi + off);
}

static inline uint8_t dwc_ltssm(struct dwc_ep *ep)
{
	return FIELD_GET(DWC_PL_DEBUG0_LTSSM, dwc_rd(ep, DWC_PL_DEBUG0));
}

const char *dwc_ltssm_name(uint8_t s);
int dwc_ep_init(const struct device *dev, bool autostart);
void dwc_ep_restore_lnkcap(const struct device *dev);
/* link state, bus master enable and hot reset; called periodically by the glue */
void dwc_ep_poll(const struct device *dev);

int dwc_ep_write_header(const struct device *dev, uint8_t func,
			const struct pcie_epc_header *hdr);
int dwc_ep_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar);
void dwc_ep_clear_bar(const struct device *dev, uint8_t func, uint8_t barno);
int dwc_ep_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr, uint64_t pci_addr,
		    size_t size);
void dwc_ep_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr);
int dwc_ep_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs);
int dwc_ep_get_msi(const struct device *dev, uint8_t func);
int dwc_ep_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
		     uint16_t irq_num);
int dwc_ep_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val);
int dwc_ep_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val);
const struct pcie_epc_features *dwc_ep_get_features(const struct device *dev, uint8_t func);
bool dwc_ep_link_up(const struct device *dev);
uint64_t dwc_ep_cpu_to_axi(const struct device *dev, uintptr_t cpu_addr);

#endif /* ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_DWC_H_ */
