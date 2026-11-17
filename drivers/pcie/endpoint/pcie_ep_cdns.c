/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Cadence PCIe controller, endpoint mode: register level helpers shared by
 * the SoC glue drivers.
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/sys/barrier.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

#include "pcie_ep_cdns.h"

#define PCI_VENDOR_ID_REG	0x00U
#define PCI_CLASS_REVISION_REG	0x08U
#define PCI_SUBSYSTEM_REG	0x2cU
#define PCI_INTERRUPT_REG	0x3cU

int cdns_ep_write_header(const struct cdns_ep_core *core, uint8_t fn,
			 const struct pcie_ep_cdns_header *hdr)
{
	uint32_t func = CDNS_EP_FUNC_BASE(fn);
	uint32_t did_reg;
	uint32_t val;

	if (fn >= CDNS_EP_MAX_RID) {
		return -EINVAL;
	}

	/* Vendor and subsystem vendor IDs are common to all functions. */
	if (fn == 0U) {
		cdns_ep_write(core, CDNS_LM_ID,
			      (uint32_t)hdr->vendor_id | ((uint32_t)hdr->subsys_vendor_id << 16));
	}

	did_reg = core->variant->did_via_rp_bank ? CDNS_RP_BASE : (func + PCI_VENDOR_ID_REG);
	val = cdns_ep_read(core, did_reg);
	val = (val & 0xffffU) | ((uint32_t)hdr->device_id << 16);
	cdns_ep_write(core, did_reg, val);

	cdns_ep_write(core, func + PCI_CLASS_REVISION_REG,
		      (uint32_t)hdr->revision_id | ((hdr->class_code & 0xffffffU) << 8));
	cdns_ep_write(core, func + PCI_SUBSYSTEM_REG, (uint32_t)hdr->subsys_id << 16);
	cdns_ep_write(core, func + PCI_INTERRUPT_REG, (uint32_t)hdr->interrupt_pin << 8);

	return 0;
}

/* aperture (size 2^(a+7), 128 B .. 4 GiB) and control field of a BAR setting */
static int cdns_ep_bar_fields(uintptr_t phys, uint64_t size, uint32_t flags, uint32_t *aperture,
			      uint32_t *ctrl)
{
	uint64_t sz = MAX(size, (uint64_t)CDNS_EP_BAR_MIN_SIZE);

	sz = 1ULL << (64U - u64_count_leading_zeros(sz - 1U));
	*aperture = (uint32_t)u64_count_trailing_zeros(sz) - 7U;
	if (*aperture > 25U || (phys & (sz - 1U)) != 0U) {
		return -EINVAL;
	}
	if ((flags & PCIE_EP_CDNS_BAR_IO) != 0U) {
		*ctrl = CDNS_LM_BAR_CTRL_IO_32;
	} else if ((flags & PCIE_EP_CDNS_BAR_64BIT) != 0U) {
		*ctrl = ((flags & PCIE_EP_CDNS_BAR_PREFETCH) != 0U) ? CDNS_LM_BAR_CTRL_PF_MEM_64
								    : CDNS_LM_BAR_CTRL_MEM_64;
	} else {
		*ctrl = ((flags & PCIE_EP_CDNS_BAR_PREFETCH) != 0U) ? CDNS_LM_BAR_CTRL_PF_MEM_32
								    : CDNS_LM_BAR_CTRL_MEM_32;
	}
	return 0;
}

int cdns_ep_set_bar(const struct cdns_ep_core *core, uint8_t fn, uint8_t bar, uintptr_t phys,
		    uint64_t size, uint32_t flags)
{
	uint32_t reg = CDNS_LM_EP_FUNC_BAR_CFG(fn, bar);
	uint32_t b = (bar < 4U) ? bar : (bar - 4U);
	uint32_t aperture;
	uint32_t ctrl;
	uint32_t cfg;
	int ret;

	if ((fn >= CDNS_EP_MAX_FUNCS) || (bar > 5U) || (size == 0U)) {
		return -EINVAL;
	}

	if (((flags & PCIE_EP_CDNS_BAR_64BIT) != 0U) && ((bar & 1U) != 0U)) {
		return -EINVAL;
	}

	ret = cdns_ep_bar_fields(phys, size, flags, &aperture, &ctrl);
	if (ret != 0) {
		return ret;
	}

	cfg = cdns_ep_read(core, reg);
	cfg &= ~(CDNS_LM_BAR_APERTURE_MASK(b) | CDNS_LM_BAR_CTRL_MASK(b));
	cfg |= CDNS_LM_BAR_APERTURE(b, aperture) | CDNS_LM_BAR_CTRL(b, ctrl);
	cdns_ep_write(core, reg, cfg);

	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR0(core->variant->ib_bar_offset, fn, bar),
		      (uint32_t)phys);
	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR1(core->variant->ib_bar_offset, fn, bar),
		      (uint32_t)((uint64_t)phys >> 32));

	return 0;
}

int cdns_ep_clear_bar(const struct cdns_ep_core *core, uint8_t fn, uint8_t bar)
{
	uint32_t reg = CDNS_LM_EP_FUNC_BAR_CFG(fn, bar);
	uint32_t b = (bar < 4U) ? bar : (bar - 4U);
	uint32_t cfg;

	if ((fn >= CDNS_EP_MAX_FUNCS) || (bar > 5U)) {
		return -EINVAL;
	}

	cfg = cdns_ep_read(core, reg);
	cfg &= ~(CDNS_LM_BAR_APERTURE_MASK(b) | CDNS_LM_BAR_CTRL_MASK(b));
	cfg |= CDNS_LM_BAR_CTRL(b, CDNS_LM_BAR_CTRL_DISABLED);
	cdns_ep_write(core, reg, cfg);

	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR0(core->variant->ib_bar_offset, fn, bar), 0U);
	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR1(core->variant->ib_bar_offset, fn, bar), 0U);

	return 0;
}

int cdns_ep_set_msi(const struct cdns_ep_core *core, uint8_t fn, uint8_t nr_vectors)
{
	uint32_t reg = CDNS_EP_FUNC_BASE(fn) + CDNS_EP_FUNC_MSI_CAP;
	uint32_t mmc;
	uint32_t ctrl;

	if ((fn >= CDNS_EP_MAX_RID) || (nr_vectors == 0U) || (nr_vectors > 32U) ||
	    !IS_POWER_OF_TWO(nr_vectors)) {
		return -EINVAL;
	}

	mmc = (uint32_t)u32_count_trailing_zeros(nr_vectors);

	ctrl = cdns_ep_read(core, reg);
	ctrl &= ~(CDNS_MSI_CTRL_MMC_MASK | CDNS_MSI_CTRL_MASKBIT);
	ctrl |= (mmc << CDNS_MSI_CTRL_MMC_SHIFT) | CDNS_MSI_CTRL_64BIT;
	cdns_ep_write(core, reg, ctrl);

	return 0;
}

uint8_t cdns_ep_ob_nbits(const struct cdns_ep_core *core, uint64_t pci_addr, uint64_t size)
{
	uint64_t diff = pci_addr ^ (pci_addr + size - 1U);
	uint32_t nbits = (diff == 0U) ? 0U : (64U - u64_count_leading_zeros(diff));

	return (uint8_t)CLAMP(nbits, 8U, (uint32_t)core->variant->ob_max_nbits);
}

void cdns_ep_ob_program(const struct cdns_ep_core *core, uint8_t fn, uint32_t region,
			uint64_t cpu_addr, uint64_t pci_addr, uint8_t nbits)
{
	uint32_t off = core->variant->ob_region_offset;

	cdns_ep_write(core, CDNS_AT_OB_PCI_ADDR0(off, region),
		      CDNS_AT_OB_NBITS((uint32_t)nbits) | ((uint32_t)pci_addr & GENMASK(31, 8)));
	cdns_ep_write(core, CDNS_AT_OB_PCI_ADDR1(off, region), (uint32_t)(pci_addr >> 32));
	cdns_ep_write(core, CDNS_AT_OB_DESC0(off, region),
		      CDNS_AT_OB_DESC0_DEVFN(fn) | CDNS_AT_OB_DESC0_TYPE_MEM);
	cdns_ep_write(core, CDNS_AT_OB_DESC1(off, region), 0U);

	if (core->variant->ob_has_cpu_addr) {
		cdns_ep_write(core, CDNS_AT_OB_CPU_ADDR0(off, region),
			      CDNS_AT_OB_NBITS((uint32_t)nbits) |
				      ((uint32_t)cpu_addr & GENMASK(31, 8)));
		cdns_ep_write(core, CDNS_AT_OB_CPU_ADDR1(off, region),
			      (uint32_t)(cpu_addr >> 32));
	}

	/*
	 * The window is a different slave than the AT registers: with both
	 * mapped Device, a write through the window right after this can get
	 * to the controller before the region does (SLVERR). Read back, then
	 * wait for completion.
	 */
	(void)cdns_ep_read(core, CDNS_AT_OB_DESC0(off, region));
	barrier_dsync_fence_full();
}

void cdns_ep_ob_clear(const struct cdns_ep_core *core, uint32_t region)
{
	uint32_t off = core->variant->ob_region_offset;

	cdns_ep_write(core, CDNS_AT_OB_PCI_ADDR0(off, region), 0U);
	cdns_ep_write(core, CDNS_AT_OB_PCI_ADDR1(off, region), 0U);
	cdns_ep_write(core, CDNS_AT_OB_DESC0(off, region), 0U);
	cdns_ep_write(core, CDNS_AT_OB_DESC1(off, region), 0U);

	if (core->variant->ob_has_cpu_addr) {
		cdns_ep_write(core, CDNS_AT_OB_CPU_ADDR0(off, region), 0U);
		cdns_ep_write(core, CDNS_AT_OB_CPU_ADDR1(off, region), 0U);
	}
}

int cdns_ep_msi_target(const struct cdns_ep_core *core, uint8_t fn, uint32_t irq_num,
		       uint64_t *pci_addr, uint16_t *data)
{
	uint32_t cap = CDNS_EP_FUNC_BASE(fn) + CDNS_EP_FUNC_MSI_CAP;
	uint32_t ctrl = cdns_ep_read(core, cap);
	uint32_t count;
	uint32_t mask;
	uint32_t val;

	if ((ctrl & CDNS_MSI_CTRL_ME) == 0U) {
		return -EINVAL;
	}

	count = 1U << ((ctrl & CDNS_MSI_CTRL_MME_MASK) >> CDNS_MSI_CTRL_MME_SHIFT);
	if ((irq_num == 0U) || (irq_num > count)) {
		return -EINVAL;
	}

	mask = count - 1U;
	val = cdns_ep_read(core, cap + CDNS_MSI_DATA_64) & 0xffffU;
	*data = (uint16_t)((val & ~mask) | ((irq_num - 1U) & mask));

	*pci_addr = ((uint64_t)cdns_ep_read(core, cap + CDNS_MSI_ADDR_HI) << 32) |
		    cdns_ep_read(core, cap + CDNS_MSI_ADDR_LO);
	*pci_addr &= ~(uint64_t)3U;

	return 0;
}

int cdns_ep_set_msix(const struct cdns_ep_core *core, uint8_t fn, uint16_t nr, uint8_t bir,
		     uint32_t offset)
{
	uint32_t base = CDNS_EP_FUNC_BASE(fn);
	uint32_t ctrl;

	if (nr == 0U || nr > 2048U || bir > 5U || (offset & 7U) != 0U) {
		return -EINVAL;
	}
	ctrl = cdns_ep_read(core, base + CDNS_EP_FUNC_MSIX_CAP);
	ctrl &= ~(CDNS_MSIX_CTRL_TSIZE | CDNS_MSIX_CTRL_EN | CDNS_MSIX_CTRL_FMASK);
	ctrl |= FIELD_PREP(CDNS_MSIX_CTRL_TSIZE, nr - 1U);
	cdns_ep_write(core, base + CDNS_EP_FUNC_MSIX_CAP, ctrl);
	cdns_ep_write(core, base + CDNS_MSIX_TABLE, offset | bir);
	cdns_ep_write(core, base + CDNS_MSIX_PBA, (offset + nr * 16U) | bir);
	return 0;
}

void cdns_ep_hide_msix_cap(const struct cdns_ep_core *core)
{
	uint32_t msi = cdns_ep_read(core, CDNS_RP_BASE + CDNS_EP_FUNC_MSI_CAP);
	uint32_t msix = cdns_ep_read(core, CDNS_RP_BASE + CDNS_EP_FUNC_MSIX_CAP);

	/* Link the MSI capability straight to whatever follows MSI-X. */
	msi = (msi & ~CDNS_MSI_CTRL_NEXT_MASK) | (msix & CDNS_MSI_CTRL_NEXT_MASK);
	cdns_ep_write(core, CDNS_RP_BASE + CDNS_EP_FUNC_MSI_CAP, msi);
}

/*
 * The core advertises Function Level Reset, but nothing on the endpoint side
 * completes one: a host that resets the function (vfio-pci does when it
 * opens the device) waits for a function that never becomes ready again.
 * Cleared through the privileged view, like the MSI-X link.
 */
bool cdns_ep_hide_flr(const struct cdns_ep_core *core, uint8_t fn)
{
	uint32_t reg = CDNS_EP_FUNC_BASE(fn) + CDNS_EP_FUNC_PCIE_CAP + CDNS_EXP_DEVCAP;
	uint32_t cap = cdns_ep_read(core, CDNS_RP_BASE + reg);

	cdns_ep_write(core, CDNS_RP_BASE + reg, cap & ~CDNS_EXP_DEVCAP_FLR);
	return (cdns_ep_read(core, reg) & CDNS_EXP_DEVCAP_FLR) == 0U;
}

int cdns_ep_rebar_program(const struct cdns_ep_core *core, uint8_t fn,
			  const struct cdns_ep_rebar *ent, uint8_t n)
{
	uint32_t reg = CDNS_LM_EP_FUNC_BAR_CFG(fn, 4U);
	uint32_t cfg = cdns_ep_read(core, reg);

	if (n > CDNS_EP_REBAR_MAX) {
		return -EINVAL;
	}
	if (n == 0U) {
		cdns_ep_write(core, reg, cfg & ~CDNS_LM_BAR_CFG1_ERBC);
		return 0;
	}
	for (uint8_t i = 0; i < n; i++) {
		uint32_t ctrl = FIELD_PREP(CDNS_EP_REBAR_CTRL_BAR, ent[i].bar) |
				FIELD_PREP(CDNS_EP_REBAR_CTRL_SIZE, ent[i].size);

		if (i == 0U) {
			ctrl |= FIELD_PREP(CDNS_EP_REBAR_CTRL_NBAR, n); /* first entry only */
		}
		cdns_ep_write(core, CDNS_EP_FUNC_BASE(fn) + CDNS_EP_REBAR_CAP(i),
			      (ent[i].sizes << 4) & GENMASK(23, 4));
		cdns_ep_write(core, CDNS_EP_FUNC_BASE(fn) + CDNS_EP_REBAR_CTRL(i), ctrl);
	}
	cdns_ep_write(core, reg, cfg | CDNS_LM_BAR_CFG1_ERBC);
	return 0;
}

/*
 * SR-IOV: the BAR layout (aperture, type) is one setting per physical
 * function for all its VFs, in the VF BAR configuration registers; the
 * inbound translation is per routing ID (vf_fn = PF + First VF Offset +
 * index), in the same AT bank as the PF's (verified on RK3399, where the
 * TRM only lists function 0). A VF BAR is set when its layout is unset
 * or identical; -EBUSY when another VF already set it differently.
 */
int cdns_ep_set_vf_bar(const struct cdns_ep_core *core, uint8_t pf, uint8_t vf_fn, uint8_t bar,
		       uintptr_t phys, uint64_t size, uint32_t flags)
{
	uint32_t reg = CDNS_LM_EP_VFUNC_BAR_CFG(pf, bar);
	uint32_t b = (bar < 4U) ? bar : (bar - 4U);
	uint32_t aperture, ctrl, cfg, want;
	int ret;

	if ((pf >= CDNS_EP_MAX_FUNCS) || (vf_fn >= CDNS_EP_MAX_RID) || (bar > 5U) ||
	    (size == 0U) || ((flags & PCIE_EP_CDNS_BAR_IO) != 0U)) {
		return -EINVAL;
	}
	if (((flags & PCIE_EP_CDNS_BAR_64BIT) != 0U) && (((bar & 1U) != 0U))) {
		return -EINVAL;
	}
	ret = cdns_ep_bar_fields(phys, size, flags, &aperture, &ctrl);
	if (ret != 0) {
		return ret;
	}

	cfg = cdns_ep_read(core, reg);
	want = CDNS_LM_BAR_APERTURE(b, aperture) | CDNS_LM_BAR_CTRL(b, ctrl);
	if ((cfg & CDNS_LM_BAR_CTRL_MASK(b)) != CDNS_LM_BAR_CTRL(b, CDNS_LM_BAR_CTRL_DISABLED) &&
	    (cfg & (CDNS_LM_BAR_APERTURE_MASK(b) | CDNS_LM_BAR_CTRL_MASK(b))) != want) {
		return -EBUSY;
	}
	cfg &= ~(CDNS_LM_BAR_APERTURE_MASK(b) | CDNS_LM_BAR_CTRL_MASK(b));
	cdns_ep_write(core, reg, cfg | want);

	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR0(core->variant->ib_bar_offset, vf_fn, bar),
		      (uint32_t)phys);
	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR1(core->variant->ib_bar_offset, vf_fn, bar),
		      (uint32_t)((uint64_t)phys >> 32));
	return 0;
}

/* Drop the VF's translation; @p last also disables the layout for all VFs. */
int cdns_ep_clear_vf_bar(const struct cdns_ep_core *core, uint8_t pf, uint8_t vf_fn, uint8_t bar,
			 bool last)
{
	uint32_t reg = CDNS_LM_EP_VFUNC_BAR_CFG(pf, bar);
	uint32_t b = (bar < 4U) ? bar : (bar - 4U);
	uint32_t cfg;

	if ((pf >= CDNS_EP_MAX_FUNCS) || (vf_fn >= CDNS_EP_MAX_RID) || (bar > 5U)) {
		return -EINVAL;
	}
	if (last) {
		cfg = cdns_ep_read(core, reg);
		cfg &= ~(CDNS_LM_BAR_APERTURE_MASK(b) | CDNS_LM_BAR_CTRL_MASK(b));
		cfg |= CDNS_LM_BAR_CTRL(b, CDNS_LM_BAR_CTRL_DISABLED);
		cdns_ep_write(core, reg, cfg);
	}
	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR0(core->variant->ib_bar_offset, vf_fn, bar), 0U);
	cdns_ep_write(core, CDNS_AT_IB_EP_BAR_ADDR1(core->variant->ib_bar_offset, vf_fn, bar), 0U);
	return 0;
}

/* SR-IOV capability of PF @p pf: Initial = Total VFs, VF Device ID. */
void cdns_ep_sriov_setup(const struct cdns_ep_core *core, uint8_t pf, uint16_t total_vfs)
{
	uint32_t cap = CDNS_EP_FUNC_BASE(pf) + CDNS_EP_FUNC_SRIOV_CAP;

	cdns_ep_write(core, cap + CDNS_SRIOV_TOTAL_VFS,
		      (uint32_t)total_vfs | ((uint32_t)total_vfs << 16));
}

void cdns_ep_sriov_set_vf_did(const struct cdns_ep_core *core, uint8_t pf, uint16_t did)
{
	uint32_t reg = CDNS_EP_FUNC_BASE(pf) + CDNS_EP_FUNC_SRIOV_CAP + CDNS_SRIOV_VF_DID;

	cdns_ep_write(core, reg, (cdns_ep_read(core, reg) & 0xffffU) | ((uint32_t)did << 16));
}
