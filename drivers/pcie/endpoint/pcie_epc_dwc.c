/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synopsys DesignWare PCIe controller in endpoint mode, without the SoC
 * glue. Follows the Linux pcie-designware-ep driver: read-only header
 * fields and BAR types are written with DBI_RO_WR_EN, BAR sizes through
 * the masks in the DBI2 shadow, inbound translation with iATU regions in
 * BAR match mode, outbound windows with address match regions. MSI is a
 * memory write through an outbound window to the address the host set.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>

#include "pcie_epc_dwc.h"

LOG_MODULE_REGISTER(pcie_epc_dwc, CONFIG_PCIE_EP_LOG_LEVEL);

const char *dwc_ltssm_name(uint8_t s)
{
	static const char *const names[] = {
		[0x00] = "detect-quiet",
		[0x01] = "detect-act",
		[0x02] = "poll-active",
		[0x03] = "poll-compliance",
		[0x04] = "poll-config",
		[0x05] = "pre-detect-quiet",
		[0x06] = "detect-wait",
		[0x07] = "cfg-linkwd-start",
		[0x08] = "cfg-linkwd-accept",
		[0x09] = "cfg-lanenum-wait",
		[0x0a] = "cfg-lanenum-accept",
		[0x0b] = "cfg-complete",
		[0x0c] = "cfg-idle",
		[0x0d] = "rcvry-lock",
		[0x0e] = "rcvry-speed",
		[0x0f] = "rcvry-rcvrcfg",
		[0x10] = "rcvry-idle",
		[0x11] = "L0",
		[0x12] = "L0s",
		[0x13] = "L123-send-eidle",
		[0x14] = "L1-idle",
		[0x15] = "L2-idle",
		[0x16] = "L2-wake",
		[0x17] = "disabled-entry",
		[0x18] = "disabled-idle",
		[0x19] = "disabled",
		[0x1a] = "lpbk-entry",
		[0x1b] = "lpbk-active",
		[0x1c] = "lpbk-exit",
		[0x1d] = "lpbk-exit-timeout",
		[0x1e] = "hot-reset-entry",
		[0x1f] = "hot-reset",
		[0x20] = "rcvry-eq0",
		[0x21] = "rcvry-eq1",
		[0x22] = "rcvry-eq2",
		[0x23] = "rcvry-eq3",
	};

	if (s < ARRAY_SIZE(names) && names[s] != NULL) {
		return names[s];
	}
	return "?";
}

static void rmw(struct dwc_ep *ep, uint32_t off, uint32_t mask, uint32_t val)
{
	k_spinlock_key_t key = k_spin_lock(&ep->lock);

	dwc_wr(ep, off, (dwc_rd(ep, off) & ~mask) | (val & mask));
	k_spin_unlock(&ep->lock, key);
}

static void ro_wr(struct dwc_ep *ep, bool en)
{
	rmw(ep, DWC_MISC_CTRL1, DWC_MISC_DBI_RO_WR_EN, en ? DWC_MISC_DBI_RO_WR_EN : 0);
}

/* Registers of one iATU region; with the viewport the caller holds ep->lock */
static mm_reg_t atu_region(struct dwc_ep *ep, bool ib, uint8_t i)
{
	if (ep->unrolled) {
		return ep->atu + DWC_ATU_UNROLL(ib, i);
	}
	dwc_wr(ep, DWC_ATU_VIEWPORT, (ib ? DWC_ATU_DIR_IB : 0) | i);
	return ep->dbi + DWC_ATU_VIEWPORT_BASE;
}

static void atu_disable(struct dwc_ep *ep, bool ib, uint8_t i)
{
	k_spinlock_key_t key = k_spin_lock(&ep->lock);

	sys_write32(0, atu_region(ep, ib, i) + DWC_ATU_CTRL2);
	k_spin_unlock(&ep->lock, key);
}

static uint8_t atu_count(struct dwc_ep *ep, bool ib)
{
	uint8_t n;

	for (n = 0; n < DWC_ATU_MAX_REGIONS; n++) {
		mm_reg_t r;

		if (ep->unrolled) {
			r = atu_region(ep, ib, n);
			sys_write32(0x11110000U, r + DWC_ATU_LOWER_TARGET);
			if (sys_read32(r + DWC_ATU_LOWER_TARGET) != 0x11110000U) {
				break;
			}
			sys_write32(0, r + DWC_ATU_LOWER_TARGET);
		} else {
			dwc_wr(ep, DWC_ATU_VIEWPORT, (ib ? DWC_ATU_DIR_IB : 0) | n);
			if (dwc_rd(ep, DWC_ATU_VIEWPORT) != ((ib ? DWC_ATU_DIR_IB : 0) | n)) {
				break;
			}
		}
	}
	return n;
}

static uint8_t find_cap(struct dwc_ep *ep, uint8_t id)
{
	uint8_t pos;
	int ttl = 48;

	if ((dwc_rd(ep, DWC_CFG_CMD_STATUS) & DWC_STS_CAPLIST) == 0) {
		return 0;
	}
	pos = dwc_rd(ep, DWC_CFG_CAP_PTR) & 0xfc;
	while (pos >= 0x40 && ttl-- > 0) {
		uint32_t v = dwc_rd(ep, pos);

		if ((v & 0xff) == id) {
			return pos;
		}
		pos = (v >> 8) & 0xfc;
	}
	return 0;
}

void dwc_ep_restore_lnkcap(const struct device *dev)
{
	struct dwc_ep *ep = dwc_ep(dev);

	if (ep->exp_cap == 0) {
		return;
	}
	ro_wr(ep, true);
	dwc_wr(ep, ep->exp_cap + DWC_EXP_LNKCAP, ep->lnkcap);
	ro_wr(ep, false);
}

int dwc_ep_init(const struct device *dev, bool autostart)
{
	struct dwc_ep *ep = dwc_ep(dev);
	struct pcie_epc_ob_window ob = {
		.cpu_base = ep->ob_virt,
		.axi_base = ep->ob_phys,
		.size = ep->ob_size,
		.page_size = DWC_ATU_MIN_SIZE,
	};
	uint32_t ver;
	int ret;

	ret = pcie_epc_init_common(dev, &ob, autostart);
	if (ret != 0) {
		return ret;
	}
	k_mutex_init(&ep->irq_lock);

	ver = dwc_rd(ep, DWC_VERSION_NUMBER);
	ep->unrolled = dwc_rd(ep, DWC_ATU_VIEWPORT) == UINT32_MAX;
	if (ep->unrolled && ep->atu == 0) {
		ep->atu = ep->dbi + DWC_ATU_UNROLL_OFFSET;
	}
	ep->num_ib = atu_count(ep, true);
	ep->num_ob = atu_count(ep, false);
	for (uint8_t i = 0; i < ep->num_ib; i++) {
		atu_disable(ep, true, i);
	}
	for (uint8_t i = 0; i < ep->num_ob; i++) {
		atu_disable(ep, false, i);
		ep->ob_cpu[i] = 0;
	}

	/* every BAR off (reset values enable some, BAR1 64 MiB on LS1012A) */
	ro_wr(ep, true);
	for (uint8_t b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		sys_write32(0, ep->dbi2 + DWC_CFG_BAR(b));
		dwc_wr(ep, DWC_CFG_BAR(b), 0);
	}
	ro_wr(ep, false);

	ep->msi_cap = find_cap(ep, DWC_CAP_ID_MSI);
	ep->exp_cap = find_cap(ep, DWC_CAP_ID_EXP);
	if (ep->exp_cap != 0) {
		if (ep->max_speed != 0) {
			uint32_t cap = dwc_rd(ep, ep->exp_cap + DWC_EXP_LNKCAP);

			ro_wr(ep, true);
			dwc_wr(ep, ep->exp_cap + DWC_EXP_LNKCAP,
			       (cap & ~GENMASK(3, 0)) | MIN(cap & GENMASK(3, 0), ep->max_speed));
			ro_wr(ep, false);
			rmw(ep, ep->exp_cap + DWC_EXP_LNKCTL2, GENMASK(3, 0), ep->max_speed);
		}
		ep->lnkcap = dwc_rd(ep, ep->exp_cap + DWC_EXP_LNKCAP);
	}

	ep->features = (struct pcie_epc_features){
		.linkup_notifier = true,
		.msi_capable = ep->msi_cap != 0,
		.bar_fixed_64bit = ep->bar_fixed_64bit,
		/* upper halves of the 64-bit BARs */
		.reserved_bar = (uint8_t)(ep->bar_fixed_64bit << 1),
		.align = DWC_ATU_MIN_SIZE,
		.max_functions = 1,
	};

	LOG_INF("%s: id %08x class %08x, DWC %08x, iATU %s %u IB / %u OB, MSI cap 0x%02x, "
		"LnkCap %08x, LTSSM %s",
		dev->name, dwc_rd(ep, DWC_CFG_ID), dwc_rd(ep, DWC_CFG_REV_CLASS), ver,
		ep->unrolled ? "unrolled" : "viewport", ep->num_ib, ep->num_ob, ep->msi_cap,
		ep->lnkcap, dwc_ltssm_name(dwc_ltssm(ep)));
	return 0;
}

static void write_header(struct dwc_ep *ep, const struct pcie_epc_header *hdr)
{
	uint32_t class = ((uint32_t)hdr->baseclass << 16) | ((uint32_t)hdr->subclass << 8) |
			 hdr->prog_if;

	ro_wr(ep, true);
	dwc_wr(ep, DWC_CFG_ID, ((uint32_t)hdr->device_id << 16) | hdr->vendor_id);
	dwc_wr(ep, DWC_CFG_REV_CLASS, (class << 8) | hdr->revision_id);
	dwc_wr(ep, DWC_CFG_SSID, ((uint32_t)hdr->subsys_id << 16) | hdr->subsys_vendor_id);
	rmw(ep, DWC_CFG_BIST_HDR, 0xff, hdr->cache_line_size);
	rmw(ep, DWC_CFG_INTR, 0xff00, (uint32_t)hdr->interrupt_pin << 8);
	ro_wr(ep, false);

	if (dwc_rd(ep, DWC_CFG_ID) != (((uint32_t)hdr->device_id << 16) | hdr->vendor_id)) {
		LOG_WRN("ID reads %08x, wanted %04x:%04x", dwc_rd(ep, DWC_CFG_ID), hdr->vendor_id,
			hdr->device_id);
	}
}

int dwc_ep_write_header(const struct device *dev, uint8_t func, const struct pcie_epc_header *hdr)
{
	struct dwc_ep *ep = dwc_ep(dev);

	if (func != 0) {
		return -EINVAL;
	}
	ep->hdr = *hdr;
	ep->hdr_valid = true;
	write_header(ep, hdr);
	return 0;
}

static int ib_program(struct dwc_ep *ep, uint8_t func, uint8_t barno, uint64_t phys)
{
	k_spinlock_key_t key;
	mm_reg_t r;
	int i;

	key = k_spin_lock(&ep->lock);
	r = atu_region(ep, true, barno);
	sys_write32((uint32_t)phys, r + DWC_ATU_LOWER_TARGET);
	sys_write32((uint32_t)(phys >> 32), r + DWC_ATU_UPPER_TARGET);
	sys_write32(DWC_ATU_CTRL1_FUNC(func), r + DWC_ATU_CTRL1);
	sys_write32(DWC_ATU_ENABLE | DWC_ATU_BAR_MODE | DWC_ATU_BAR(barno), r + DWC_ATU_CTRL2);
	for (i = 0; i < 5; i++) {
		if ((sys_read32(r + DWC_ATU_CTRL2) & DWC_ATU_ENABLE) != 0) {
			break;
		}
	}
	k_spin_unlock(&ep->lock, key);
	return i < 5 ? 0 : -EIO;
}

int dwc_ep_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint8_t b = bar->barno;
	bool is64 = (bar->flags & PCIE_EPC_BAR_MEM64) != 0 || (ep->bar_fixed_64bit & BIT(b)) != 0;
	uint32_t flags;
	size_t size = MAX(bar->size, (size_t)DWC_ATU_MIN_SIZE);
	uint64_t mask;
	int ret;

	if (func != 0 || b >= PCIE_EPC_NUM_BARS || (bar->flags & PCIE_EPC_BAR_IO) != 0) {
		return -EINVAL;
	}
	if ((ep->features.reserved_bar & BIT(b)) != 0 || (is64 && b == PCIE_EPC_NUM_BARS - 1)) {
		LOG_ERR("BAR%u is not available", b);
		return -EINVAL;
	}
	if (b >= ep->num_ib) {
		LOG_ERR("BAR%u: only %u inbound regions", b, ep->num_ib);
		return -ENOSPC;
	}
	if (!IS_POWER_OF_TWO(size) || (bar->phys & (size - 1)) != 0) {
		LOG_ERR("BAR%u 0x%zx at 0x%llx: power of two, aligned to its size", b, bar->size,
			(unsigned long long)bar->phys);
		return -EINVAL;
	}

	flags = (is64 ? BIT(2) : 0) |
		((bar->flags & PCIE_EPC_BAR_PREFETCH) != 0 || (ep->bar_fixed_64bit & BIT(b)) != 0
			 ? BIT(3)
			 : 0);
	mask = (uint64_t)size - 1;

	ret = ib_program(ep, func, b, bar->phys);
	if (ret != 0) {
		LOG_ERR("BAR%u: inbound region does not enable", b);
		return ret;
	}

	ro_wr(ep, true);
	/* mask bit 0 is the BAR enable */
	sys_write32((uint32_t)mask, ep->dbi2 + DWC_CFG_BAR(b));
	dwc_wr(ep, DWC_CFG_BAR(b), flags);
	if (is64) {
		sys_write32((uint32_t)(mask >> 32), ep->dbi2 + DWC_CFG_BAR(b + 1));
		dwc_wr(ep, DWC_CFG_BAR(b + 1), 0);
	}
	ro_wr(ep, false);

	ep->bar[b] = *bar;
	ep->bar[b].flags = is64 ? (bar->flags | PCIE_EPC_BAR_MEM64) : bar->flags;
	LOG_INF("BAR%u: 0x%zx bytes%s at AXI 0x%llx", b, size, is64 ? " 64-bit" : "",
		(unsigned long long)bar->phys);
	return 0;
}

void dwc_ep_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	struct dwc_ep *ep = dwc_ep(dev);
	bool is64;

	if (func != 0 || barno >= PCIE_EPC_NUM_BARS || ep->bar[barno].size == 0) {
		return;
	}
	is64 = (ep->bar[barno].flags & PCIE_EPC_BAR_MEM64) != 0;
	atu_disable(ep, true, barno);
	ro_wr(ep, true);
	sys_write32(0, ep->dbi2 + DWC_CFG_BAR(barno));
	dwc_wr(ep, DWC_CFG_BAR(barno), 0);
	if (is64) {
		sys_write32(0, ep->dbi2 + DWC_CFG_BAR(barno + 1));
		dwc_wr(ep, DWC_CFG_BAR(barno + 1), 0);
	}
	ro_wr(ep, false);
	memset(&ep->bar[barno], 0, sizeof(ep->bar[barno]));
}

uint64_t dwc_ep_cpu_to_axi(const struct device *dev, uintptr_t cpu_addr)
{
	struct dwc_ep *ep = dwc_ep(dev);

	if (cpu_addr >= ep->ob_virt && cpu_addr - ep->ob_virt < ep->ob_size) {
		return ep->ob_phys + (cpu_addr - ep->ob_virt);
	}
	return cpu_addr;
}

int dwc_ep_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr, uint64_t pci_addr,
		    size_t size)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint64_t axi = dwc_ep_cpu_to_axi(dev, cpu_addr);
	uint64_t limit = axi + size - 1;
	k_spinlock_key_t key;
	mm_reg_t r;
	int w;

	if (func != 0 || size == 0 || (axi & (DWC_ATU_MIN_SIZE - 1)) != 0 ||
	    (size & (DWC_ATU_MIN_SIZE - 1)) != 0) {
		return -EINVAL;
	}
	/* the limit register holds the lower 32 bits only */
	if ((axi >> 32) != (limit >> 32)) {
		return -EINVAL;
	}

	key = k_spin_lock(&ep->lock);
	for (w = 0; w < ep->num_ob; w++) {
		if (ep->ob_cpu[w] == 0) {
			ep->ob_cpu[w] = cpu_addr;
			break;
		}
	}
	if (w == ep->num_ob) {
		k_spin_unlock(&ep->lock, key);
		return -ENOSPC;
	}
	r = atu_region(ep, false, w);
	sys_write32((uint32_t)axi, r + DWC_ATU_LOWER_BASE);
	sys_write32((uint32_t)(axi >> 32), r + DWC_ATU_UPPER_BASE);
	sys_write32((uint32_t)limit, r + DWC_ATU_LIMIT);
	sys_write32((uint32_t)pci_addr, r + DWC_ATU_LOWER_TARGET);
	sys_write32((uint32_t)(pci_addr >> 32), r + DWC_ATU_UPPER_TARGET);
	sys_write32(DWC_ATU_CTRL1_FUNC(func), r + DWC_ATU_CTRL1);
	sys_write32(DWC_ATU_ENABLE, r + DWC_ATU_CTRL2);
	(void)sys_read32(r + DWC_ATU_CTRL2);
	k_spin_unlock(&ep->lock, key);
	barrier_dmem_fence_full();
	return 0;
}

void dwc_ep_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr)
{
	struct dwc_ep *ep = dwc_ep(dev);

	ARG_UNUSED(func);

	for (uint8_t w = 0; w < ep->num_ob; w++) {
		if (ep->ob_cpu[w] == cpu_addr) {
			atu_disable(ep, false, w);
			ep->ob_cpu[w] = 0;
			return;
		}
	}
}

int dwc_ep_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint8_t mmc = 0;

	if (func != 0 || nr_irqs == 0 || nr_irqs > 32) {
		return -EINVAL;
	}
	if (ep->msi_cap == 0) {
		return -ENOTSUP;
	}
	while ((1U << mmc) < nr_irqs) {
		mmc++;
	}
	ep->msi_mmc = mmc;
	ro_wr(ep, true);
	rmw(ep, ep->msi_cap, DWC_MSI_CTRL_MMC, FIELD_PREP(DWC_MSI_CTRL_MMC, mmc));
	ro_wr(ep, false);
	return 0;
}

int dwc_ep_get_msi(const struct device *dev, uint8_t func)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint32_t ctrl;

	if (func != 0 || ep->msi_cap == 0) {
		return -EINVAL;
	}
	ctrl = dwc_rd(ep, ep->msi_cap);
	if ((ctrl & DWC_MSI_CTRL_EN) == 0) {
		return -EINVAL;
	}
	return 1 << FIELD_GET(DWC_MSI_CTRL_MME, ctrl);
}

/* One 32-bit write to host memory; the page stays mapped for the next one */
static int irq_write(const struct device *dev, uint64_t addr, uint32_t data)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint64_t page = addr & ~(uint64_t)(DWC_ATU_MIN_SIZE - 1);
	int ret;

	if (ep->irq_map_valid && (ep->irq_map_stale || page != ep->irq_map_page)) {
		pcie_epc_unmap_host(dev, &ep->irq_map);
		ep->irq_map_valid = false;
	}
	if (!ep->irq_map_valid) {
		ret = pcie_epc_map_host(dev, 0, page, DWC_ATU_MIN_SIZE, &ep->irq_map);
		if (ret != 0) {
			return ret;
		}
		ep->irq_map_page = page;
		ep->irq_map_valid = true;
		ep->irq_map_stale = false;
	}
	sys_write32(data, (mem_addr_t)ep->irq_map.ptr + (mem_addr_t)(addr - page));
	barrier_dsync_fence_full();
	return 0;
}

static int raise_msi(const struct device *dev, uint16_t irq_num)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint32_t ctrl = dwc_rd(ep, ep->msi_cap);
	uint64_t addr = dwc_rd(ep, ep->msi_cap + 4);
	uint16_t data;
	int vectors, ret;

	if ((ctrl & DWC_MSI_CTRL_EN) == 0) {
		return -EINVAL;
	}
	if ((dwc_rd(ep, DWC_CFG_CMD_STATUS) & DWC_CMD_MASTER) == 0) {
		return -EACCES;
	}
	vectors = 1 << FIELD_GET(DWC_MSI_CTRL_MME, ctrl);
	if (irq_num == 0 || irq_num > vectors) {
		return -EINVAL;
	}
	if ((ctrl & DWC_MSI_CTRL_64BIT) != 0) {
		addr |= (uint64_t)dwc_rd(ep, ep->msi_cap + 8) << 32;
		data = dwc_rd(ep, ep->msi_cap + 12) & 0xffff;
	} else {
		data = dwc_rd(ep, ep->msi_cap + 8) & 0xffff;
	}
	data = (data & ~(vectors - 1)) | (irq_num - 1);

	k_mutex_lock(&ep->irq_lock, K_FOREVER);
	ret = irq_write(dev, addr & ~3ULL, data);
	k_mutex_unlock(&ep->irq_lock);
	if (ret == 0 && ep->msi_sent++ == 0) {
		LOG_INF("MSI %u: 0x%04x -> 0x%llx (first one)", irq_num, data,
			(unsigned long long)addr);
	}
	return ret;
}

int dwc_ep_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
		     uint16_t irq_num)
{
	struct dwc_ep *ep = dwc_ep(dev);

	if (func != 0) {
		return -EINVAL;
	}
	if (type == PCIE_EPC_IRQ_MSI && ep->msi_cap != 0) {
		return raise_msi(dev, irq_num);
	}
	return -ENOTSUP;
}

int dwc_ep_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val)
{
	if (func != 0 || offset >= 0x1000) {
		return -EINVAL;
	}
	*val = dwc_rd(dwc_ep(dev), offset & ~3U);
	return 0;
}

int dwc_ep_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val)
{
	if (func != 0 || offset >= 0x1000) {
		return -EINVAL;
	}
	dwc_wr(dwc_ep(dev), offset & ~3U, val);
	return 0;
}

const struct pcie_epc_features *dwc_ep_get_features(const struct device *dev, uint8_t func)
{
	ARG_UNUSED(func);
	return &dwc_ep(dev)->features;
}

bool dwc_ep_link_up(const struct device *dev)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint32_t d1 = dwc_rd(ep, DWC_PL_DEBUG1);

	return (d1 & DWC_PL_DEBUG1_LINK_UP) != 0 && (d1 & DWC_PL_DEBUG1_TRAINING) == 0;
}

void dwc_ep_poll(const struct device *dev)
{
	struct dwc_ep *ep = dwc_ep(dev);
	uint8_t s = dwc_ltssm(ep);
	uint8_t prev = ep->ltssm;
	bool up = dwc_ep_link_up(dev);
	bool bme;

	ep->ltssm = s;
	if (s == DWC_LTSSM_HOT_RESET && prev != DWC_LTSSM_HOT_RESET) {
		ep->hot_resets++;
		ep->irq_map_stale = true;
		pcie_epc_notify(dev, PCIE_EPC_EVT_HOT_RESET, 0);
	}
	if (up != ep->link_up) {
		ep->link_up = up;
		if (up) {
			ep->link_ups++;
			LOG_INF("%s: link up, LnkSta %04x", dev->name,
				ep->exp_cap != 0 ? dwc_rd(ep, ep->exp_cap + DWC_EXP_LNKCTL_STA) >> 16
						 : 0);
		} else {
			ep->link_downs++;
			ep->irq_map_stale = true;
			LOG_INF("%s: link down (%s)", dev->name, dwc_ltssm_name(s));
		}
		pcie_epc_notify(dev, up ? PCIE_EPC_EVT_LINK_UP : PCIE_EPC_EVT_LINK_DOWN, 0);
	}
	bme = up && (dwc_rd(ep, DWC_CFG_CMD_STATUS) & DWC_CMD_MASTER) != 0;
	if (bme != ep->bme) {
		ep->bme = bme;
		pcie_epc_notify(dev, bme ? PCIE_EPC_EVT_BME : PCIE_EPC_EVT_BME_CLEARED, 0);
	}
}
