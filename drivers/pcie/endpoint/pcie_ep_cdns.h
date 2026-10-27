/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Cadence PCIe controller (LGA register layout), endpoint mode.
 *
 * Offsets are relative to the base of the Cadence core register space
 * (Linux drivers/pci/controller/cadence/pcie-cadence-lga-regs.h). On the
 * RK3399 that space starts at apb + 0x800000.
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/pcie/endpoint/pcie_ep_cdns.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* Endpoint function configuration space */
#define CDNS_EP_FUNC_BASE(fn)		(((uint32_t)(fn) << 12) & GENMASK(19, 12))
#define CDNS_EP_FUNC_MSI_CAP		0x90U
#define CDNS_EP_FUNC_MSIX_CAP		0xb0U
#define CDNS_EP_FUNC_PCIE_CAP		0xc0U

/* Local management */
#define CDNS_LM_BASE			0x100000U
#define CDNS_LM_PL_CONFIG0		(CDNS_LM_BASE + 0x000U)
#define  CDNS_LM_PL_CONFIG0_LS_READY	BIT(0)
#define CDNS_LM_ID			(CDNS_LM_BASE + 0x044U)
#define CDNS_LM_EP_FUNC_BAR_CFG(fn, bar) \
	(CDNS_LM_BASE + 0x240U + ((uint32_t)(fn) * 8U) + (((bar) < 4U) ? 0U : 4U))
#define  CDNS_LM_BAR_APERTURE_MASK(b)	(GENMASK(4, 0) << ((b) * 8U))
#define  CDNS_LM_BAR_CTRL_MASK(b)	(GENMASK(7, 5) << ((b) * 8U))
#define  CDNS_LM_BAR_APERTURE(b, a) \
	(((uint32_t)(a) << ((b) * 8U)) & CDNS_LM_BAR_APERTURE_MASK(b))
#define  CDNS_LM_BAR_CTRL(b, c) \
	(((uint32_t)(c) << (((b) * 8U) + 5U)) & CDNS_LM_BAR_CTRL_MASK(b))
#define  CDNS_LM_BAR_CTRL_DISABLED	0x0U
#define  CDNS_LM_BAR_CTRL_IO_32		0x1U
#define  CDNS_LM_BAR_CTRL_MEM_32	0x4U
#define  CDNS_LM_BAR_CTRL_PF_MEM_32	0x5U
#define  CDNS_LM_BAR_CTRL_MEM_64	0x6U
#define  CDNS_LM_BAR_CTRL_PF_MEM_64	0x7U
#define CDNS_LM_EP_FUNC_CFG		(CDNS_LM_BASE + 0x2c0U)

/* Root port configuration space (also the local view of function 0) */
#define CDNS_RP_BASE			0x200000U

/* Address translation */
#define CDNS_AT_BASE			0x400000U
#define CDNS_AT_OB_PCI_ADDR0(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x00U)
#define CDNS_AT_OB_PCI_ADDR1(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x04U)
#define CDNS_AT_OB_DESC0(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x08U)
#define CDNS_AT_OB_DESC1(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x0cU)
#define CDNS_AT_OB_CPU_ADDR0(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x18U)
#define CDNS_AT_OB_CPU_ADDR1(base, r)	(CDNS_AT_BASE + (base) + ((r) * 0x20U) + 0x1cU)
#define  CDNS_AT_OB_NBITS(n)		(((n) - 1U) & GENMASK(5, 0))
#define  CDNS_AT_OB_DESC0_TYPE_MEM	0x2U
#define  CDNS_AT_OB_DESC0_TYPE_MSG	0xcU
#define  CDNS_AT_OB_DESC0_DEVFN(fn)	(((uint32_t)(fn) << 24) & GENMASK(31, 24))
#define CDNS_AT_IB_EP_BAR_ADDR0(base, fn, bar) \
	(CDNS_AT_BASE + (base) + ((uint32_t)(fn) * 0x40U) + ((uint32_t)(bar) * 8U))
#define CDNS_AT_IB_EP_BAR_ADDR1(base, fn, bar) (CDNS_AT_IB_EP_BAR_ADDR0(base, fn, bar) + 4U)

/* MSI capability, dword view */
#define CDNS_MSI_CTRL_ME		BIT(16)
#define CDNS_MSI_CTRL_MMC_SHIFT		17U
#define CDNS_MSI_CTRL_MMC_MASK		GENMASK(19, 17)
#define CDNS_MSI_CTRL_MME_SHIFT		20U
#define CDNS_MSI_CTRL_MME_MASK		GENMASK(22, 20)
#define CDNS_MSI_CTRL_64BIT		BIT(23)
#define CDNS_MSI_CTRL_MASKBIT		BIT(24)
#define CDNS_MSI_CTRL_NEXT_MASK		GENMASK(15, 8)
#define CDNS_MSI_ADDR_LO		0x04U
#define CDNS_MSI_ADDR_HI		0x08U
#define CDNS_MSI_DATA_64		0x0cU

#define CDNS_EP_MAX_FUNCS		8U
#define CDNS_EP_BAR_MIN_SIZE		128U

/**
 * Differences between revisions of the Cadence IP.
 *
 * RK3399 (older revision):
 *  - outbound region r registers start at AT + 0x20 (region 0 belongs to the
 *    root port), region selected by AXI address bits [24:20], no
 *    CPU_ADDR0/1 registers, at most 1 MiB per region;
 *  - inbound EP BAR table at AT + 0x828;
 *  - device ID written through the root port bank.
 * TI K3 / upstream Cadence:
 *  - outbound regions at AT + 0x00 with CPU_ADDR0/1 registers;
 *  - inbound EP BAR table at AT + 0x840.
 */
struct cdns_ep_variant {
	uint32_t ob_region_offset;
	uint32_t ib_bar_offset;
	uint8_t ob_max_nbits;
	bool ob_has_cpu_addr;
	bool did_via_rp_bank;
};

struct cdns_ep_core {
	mm_reg_t base;
	const struct cdns_ep_variant *variant;
};

static inline uint32_t cdns_ep_read(const struct cdns_ep_core *core, uint32_t reg)
{
	return sys_read32(core->base + reg);
}

static inline void cdns_ep_write(const struct cdns_ep_core *core, uint32_t reg, uint32_t val)
{
	sys_write32(val, core->base + reg);
}

int cdns_ep_write_header(const struct cdns_ep_core *core, uint8_t fn,
			 const struct pcie_ep_cdns_header *hdr);
int cdns_ep_set_bar(const struct cdns_ep_core *core, uint8_t fn, uint8_t bar, uintptr_t phys,
		    uint64_t size, uint32_t flags);
int cdns_ep_clear_bar(const struct cdns_ep_core *core, uint8_t fn, uint8_t bar);
int cdns_ep_set_msi(const struct cdns_ep_core *core, uint8_t fn, uint8_t nr_vectors);
uint8_t cdns_ep_ob_nbits(const struct cdns_ep_core *core, uint64_t pci_addr, uint64_t size);
void cdns_ep_ob_program(const struct cdns_ep_core *core, uint8_t fn, uint32_t region,
			uint64_t cpu_addr, uint64_t pci_addr, uint8_t nbits);
void cdns_ep_ob_clear(const struct cdns_ep_core *core, uint32_t region);
int cdns_ep_msi_target(const struct cdns_ep_core *core, uint8_t fn, uint32_t irq_num,
		       uint64_t *pci_addr, uint16_t *data);
void cdns_ep_hide_msix_cap(const struct cdns_ep_core *core);

#endif /* ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_ */
