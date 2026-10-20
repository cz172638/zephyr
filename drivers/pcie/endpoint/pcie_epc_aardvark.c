/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Aardvark (Marvell Armada 3700) PCIe endpoint controller, register
 * programming without the SoC glue.
 *
 * Known from the Linux root complex driver: mode and lane selection in
 * CTRL0, reference clock direction, direct mode, the outbound windows,
 * the LTSSM state in the LMI, the vendor ID override and the writable
 * class code. Endpoint only and undocumented: BAR sizing and inbound
 * translation (left alone, the host sees the reset defaults), INTx
 * (CTRL0 bits 28..31, a guess). MSI is sent the generic way, a memory
 * write through an outbound window to the address the host programmed
 * in the MSI capability.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_epc_aardvark.h"

LOG_MODULE_REGISTER(pcie_epc_aardvark, CONFIG_PCIE_EP_LOG_LEVEL);

static void rmw(struct advk_ep *ep, uint32_t off, uint32_t mask, uint32_t val)
{
	k_spinlock_key_t key = k_spin_lock(&ep->lock);

	advk_wr(ep, off, (advk_rd(ep, off) & ~mask) | (val & mask));
	k_spin_unlock(&ep->lock, key);
}

const char *advk_ltssm_name(uint8_t s)
{
	static const char *const names[] = {
		[0x00] = "detect-quiet",
		[0x01] = "detect-active",
		[0x02] = "polling-active",
		[0x03] = "polling-compliance",
		[0x04] = "polling-config",
		[0x05] = "config-linkwidth-start",
		[0x06] = "config-linkwidth-accept",
		[0x07] = "config-lanenum-accept",
		[0x08] = "config-lanenum-wait",
		[0x09] = "config-complete",
		[0x0a] = "config-idle",
		[0x0b] = "recovery-rcvr-lock",
		[0x0c] = "recovery-speed",
		[0x0d] = "recovery-rcvr-cfg",
		[0x0e] = "recovery-idle",
		[0x10] = "L0",
		[0x11] = "rx-L0s-entry",
		[0x12] = "rx-L0s-idle",
		[0x13] = "rx-L0s-fts",
		[0x14] = "tx-L0s-entry",
		[0x15] = "tx-L0s-idle",
		[0x16] = "tx-L0s-fts",
		[0x17] = "L1-entry",
		[0x18] = "L1-idle",
		[0x19] = "L2-idle",
		[0x1a] = "L2-transmit-wake",
		[0x20] = "disabled",
		[0x21] = "loopback-entry-master",
		[0x22] = "loopback-active-master",
		[0x23] = "loopback-exit-master",
		[0x24] = "loopback-entry-slave",
		[0x25] = "loopback-active-slave",
		[0x26] = "loopback-exit-slave",
		[0x27] = "hot-reset",
		[0x28] = "recovery-eq0",
		[0x29] = "recovery-eq1",
		[0x2a] = "recovery-eq2",
		[0x2b] = "recovery-eq3",
	};

	if (s < ARRAY_SIZE(names) && names[s] != NULL) {
		return names[s];
	}
	return "?";
}

int advk_ob_encode(uint64_t axi, uint64_t pci, size_t size, uint32_t regs[7])
{
	uint64_t mask;

	if (size < ADVK_OB_MIN_SIZE || !IS_POWER_OF_TWO(size)) {
		return -EINVAL;
	}
	if ((axi & (size - 1)) != 0 || (pci & (size - 1)) != 0) {
		return -EINVAL;
	}

	mask = ~((uint64_t)size - 1);
	regs[0] = (uint32_t)axi | ADVK_OB_WIN_ENABLE;
	regs[1] = (uint32_t)(axi >> 32);
	regs[2] = (uint32_t)pci;
	regs[3] = (uint32_t)(pci >> 32);
	regs[4] = (uint32_t)mask;
	regs[5] = (uint32_t)(mask >> 32);
	regs[6] = ADVK_OB_TYPE_MEM;
	return 0;
}

static void ob_disable(struct advk_ep *ep, int w)
{
	for (uint32_t off = 0; off <= 0x18; off += 4) {
		advk_wr(ep, ADVK_OB_WIN(w, off), 0);
	}
}

static uint8_t find_cap(struct advk_ep *ep, uint8_t id)
{
	uint8_t pos;
	int ttl = 48;

	if ((advk_rd(ep, ADVK_CFG_CMD_STATUS) & ADVK_STS_CAPLIST) == 0) {
		return 0;
	}

	pos = advk_rd(ep, ADVK_CFG_CAP_PTR) & 0xfc;
	while (pos >= 0x40 && ttl-- > 0) {
		uint32_t v = advk_rd(ep, pos);

		if ((v & 0xff) == id) {
			return pos;
		}
		pos = (v >> 8) & 0xfc;
	}
	return 0;
}

void advk_ep_hw_setup(const struct device *dev)
{
	struct advk_ep *ep = advk_ep(dev);
	uint32_t lanes = ep->num_lanes >= 4 ? 2 : (ep->num_lanes == 2 ? 1 : 0);
	uint32_t gen = ep->max_speed >= 3 ? 2 : (ep->max_speed == 2 ? 1 : 0);

	/* Direct mode, no command queue */
	rmw(ep, ADVK_CTRL_CONFIG, ADVK_CTRL_MODE_CMD, 0);

	/*
	 * Reference clock: the known working endpoint setup (ESPRESSObin wiki)
	 * keeps the RC value, TX on and RX off, with the PHY's clock output
	 * disabled and the REFCLK lines not connected, each side runs on its
	 * own clock. refclk_in tries the pins as input instead.
	 */
	rmw(ep, ADVK_REF_CLK, ADVK_REF_CLK_TX | ADVK_REF_CLK_RX,
	    ep->refclk_in ? ADVK_REF_CLK_RX : ADVK_REF_CLK_TX);

	/* Endpoint, lanes, speed; training stays off until start */
	rmw(ep, ADVK_CTRL0,
	    ADVK_CTRL0_IS_RC | ADVK_CTRL0_LANES_MASK | ADVK_CTRL0_GEN_MASK |
		    ADVK_CTRL0_LINK_TRAIN | GENMASK(31, 28),
	    FIELD_PREP(ADVK_CTRL0_LANES_MASK, lanes) | FIELD_PREP(ADVK_CTRL0_GEN_MASK, gen));

	/* Target link speed in Link Control 2 */
	rmw(ep, ADVK_CFG_LNKCTL2, GENMASK(3, 0), ep->max_speed != 0 ? ep->max_speed : 2);

	/* Outbound: windows only, transparent for anything else */
	advk_wr(ep, ADVK_CTRL2, ADVK_CTRL2_RESERVED | ADVK_CTRL2_TD | ADVK_CTRL2_OB_WIN_EN);
	advk_wr(ep, ADVK_OB_DEFAULT_ACTIONS, ADVK_OB_TYPE_MEM);
	for (int w = 0; w < ADVK_OB_WIN_COUNT; w++) {
		ob_disable(ep, w);
		ep->ob_cpu[w] = 0;
	}

	/* Everything masked, latched status cleared; the glue polls */
	advk_wr(ep, ADVK_ISR0_MASK, UINT32_MAX);
	advk_wr(ep, ADVK_ISR1_MASK, UINT32_MAX);
	advk_wr(ep, ADVK_HOST_INT_MASK, UINT32_MAX);
	advk_wr(ep, ADVK_ISR0, UINT32_MAX);
	advk_wr(ep, ADVK_ISR1, UINT32_MAX);
	advk_wr(ep, ADVK_HOST_INT_STATUS, UINT32_MAX);

	ep->msi_cap = find_cap(ep, ADVK_CAP_ID_MSI);
}

int advk_ep_init(const struct device *dev, bool autostart)
{
	struct advk_ep *ep = advk_ep(dev);
	struct pcie_epc_ob_window ob = {
		.cpu_base = ep->ob_virt,
		.axi_base = ep->ob_phys,
		.size = ep->ob_size,
		.page_size = ADVK_OB_MIN_SIZE,
	};
	int ret;

	ret = pcie_epc_init_common(dev, &ob, autostart);
	if (ret != 0) {
		return ret;
	}

	advk_ep_hw_setup(dev);

	ep->features = (struct pcie_epc_features){
		.linkup_notifier = true,
		.msi_capable = ep->msi_cap != 0,
		.intx_capable = true,
		.align = ADVK_OB_MIN_SIZE,
		.max_functions = 1,
	};

	LOG_INF("%s: id %08x class %08x ctrl0 %08x ltssm %s, MSI cap 0x%02x", dev->name,
		advk_rd(ep, ADVK_CFG_ID), advk_rd(ep, ADVK_CFG_REV_CLASS),
		advk_rd(ep, ADVK_CTRL0), advk_ltssm_name(advk_ltssm(ep)), ep->msi_cap);
	return 0;
}

int advk_ep_start(const struct device *dev)
{
	rmw(advk_ep(dev), ADVK_CTRL0, ADVK_CTRL0_LINK_TRAIN, ADVK_CTRL0_LINK_TRAIN);
	return 0;
}

void advk_ep_stop(const struct device *dev)
{
	rmw(advk_ep(dev), ADVK_CTRL0, ADVK_CTRL0_LINK_TRAIN, 0);
}

static void write_header(struct advk_ep *ep, const struct pcie_epc_header *hdr)
{
	uint32_t class = ((uint32_t)hdr->baseclass << 16) | ((uint32_t)hdr->subclass << 8) |
			 hdr->prog_if;
	uint32_t v;

	/* Vendor ID override (erratum 4.1); also the subsystem vendor */
	advk_wr(ep, ADVK_LMI_VENDOR_ID, ((uint32_t)hdr->subsys_vendor_id << 16) | hdr->vendor_id);
	advk_wr(ep, ADVK_CFG_ID, ((uint32_t)hdr->device_id << 16) | hdr->vendor_id);
	advk_wr(ep, ADVK_CFG_REV_CLASS, (class << 8) | hdr->revision_id);
	advk_wr(ep, ADVK_CFG_SSID, ((uint32_t)hdr->subsys_id << 16) | hdr->subsys_vendor_id);
	rmw(ep, ADVK_CFG_BIST_HDR, 0xff, hdr->cache_line_size);
	rmw(ep, ADVK_CFG_INTR, 0xff00, (uint32_t)hdr->interrupt_pin << 8);

	v = advk_rd(ep, ADVK_CFG_ID);
	if (v != (((uint32_t)hdr->device_id << 16) | hdr->vendor_id)) {
		LOG_WRN("ID reads %08x, wanted %04x:%04x (read-only part?)", v, hdr->vendor_id,
			hdr->device_id);
	}
	v = advk_rd(ep, ADVK_CFG_REV_CLASS);
	if ((v >> 8) != class) {
		LOG_WRN("class reads %06x, wanted %06x", v >> 8, class);
	}
}

int advk_ep_write_header(const struct device *dev, uint8_t func,
			 const struct pcie_epc_header *hdr)
{
	struct advk_ep *ep = advk_ep(dev);

	if (func != 0) {
		return -EINVAL;
	}
	ep->hdr = *hdr;
	ep->hdr_valid = true;
	write_header(ep, hdr);
	return 0;
}

int advk_ep_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar)
{
	struct advk_ep *ep = advk_ep(dev);

	if (func != 0 || bar->barno >= PCIE_EPC_NUM_BARS) {
		return -EINVAL;
	}

	/*
	 * No known register sets the BAR size or translates inbound accesses
	 * to bar->phys; remember the request for the shell and leave the
	 * reset defaults the host will size.
	 */
	ep->bar[bar->barno] = *bar;
	LOG_WRN("BAR%u: 0x%zx bytes at 0x%llx requested, inbound translation unknown, "
		"not programmed",
		bar->barno, bar->size, (unsigned long long)bar->phys);
	return 0;
}

void advk_ep_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	struct advk_ep *ep = advk_ep(dev);

	if (func == 0 && barno < PCIE_EPC_NUM_BARS) {
		memset(&ep->bar[barno], 0, sizeof(ep->bar[barno]));
	}
}

uint64_t advk_ep_cpu_to_axi(const struct device *dev, uintptr_t cpu_addr)
{
	struct advk_ep *ep = advk_ep(dev);

	if (cpu_addr >= ep->ob_virt && cpu_addr - ep->ob_virt < ep->ob_size) {
		return ep->ob_phys + (cpu_addr - ep->ob_virt);
	}
	return cpu_addr;
}

int advk_ep_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
		     uint64_t pci_addr, size_t size)
{
	struct advk_ep *ep = advk_ep(dev);
	uint32_t regs[7];
	k_spinlock_key_t key;
	int w, ret;

	if (func != 0) {
		return -EINVAL;
	}

	ret = advk_ob_encode(advk_ep_cpu_to_axi(dev, cpu_addr), pci_addr, size, regs);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&ep->lock);
	for (w = 0; w < ADVK_OB_WIN_COUNT; w++) {
		if (ep->ob_cpu[w] == 0) {
			ep->ob_cpu[w] = cpu_addr;
			break;
		}
	}
	k_spin_unlock(&ep->lock, key);
	if (w == ADVK_OB_WIN_COUNT) {
		return -ENOSPC;
	}

	/* enable bit in MATCH_LS last */
	for (int i = 6; i >= 0; i--) {
		advk_wr(ep, ADVK_OB_WIN(w, 4 * i), regs[i]);
	}
	barrier_dmem_fence_full();
	return 0;
}

void advk_ep_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr)
{
	struct advk_ep *ep = advk_ep(dev);

	ARG_UNUSED(func);

	for (int w = 0; w < ADVK_OB_WIN_COUNT; w++) {
		if (ep->ob_cpu[w] == cpu_addr) {
			ob_disable(ep, w);
			ep->ob_cpu[w] = 0;
			return;
		}
	}
}

int advk_ep_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs)
{
	struct advk_ep *ep = advk_ep(dev);
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
	rmw(ep, ep->msi_cap, ADVK_MSI_CTRL_MMC, FIELD_PREP(ADVK_MSI_CTRL_MMC, mmc));
	return 0;
}

int advk_ep_get_msi(const struct device *dev, uint8_t func)
{
	struct advk_ep *ep = advk_ep(dev);
	uint32_t ctrl;

	if (func != 0 || ep->msi_cap == 0) {
		return -EINVAL;
	}
	ctrl = advk_rd(ep, ep->msi_cap);
	if ((ctrl & ADVK_MSI_CTRL_EN) == 0) {
		return -EINVAL;
	}
	return 1 << FIELD_GET(ADVK_MSI_CTRL_MME, ctrl);
}

static int raise_msi(const struct device *dev, uint16_t irq_num)
{
	struct advk_ep *ep = advk_ep(dev);
	struct pcie_epc_host_map map;
	uint32_t ctrl = advk_rd(ep, ep->msi_cap);
	uint64_t addr = advk_rd(ep, ep->msi_cap + 4);
	uint16_t data;
	int vectors, ret;

	if ((ctrl & ADVK_MSI_CTRL_EN) == 0) {
		return -EINVAL;
	}
	vectors = 1 << FIELD_GET(ADVK_MSI_CTRL_MME, ctrl);
	if (irq_num == 0 || irq_num > vectors) {
		return -EINVAL;
	}

	if ((ctrl & ADVK_MSI_CTRL_64BIT) != 0) {
		addr |= (uint64_t)advk_rd(ep, ep->msi_cap + 8) << 32;
		data = advk_rd(ep, ep->msi_cap + 12) & 0xffff;
	} else {
		data = advk_rd(ep, ep->msi_cap + 8) & 0xffff;
	}
	data = (data & ~(vectors - 1)) | (irq_num - 1);

	ret = pcie_epc_map_host(dev, 0, addr & ~3ULL, 4, &map);
	if (ret != 0) {
		return ret;
	}
	sys_write32(data, (mem_addr_t)map.ptr);
	barrier_dmem_fence_full();
	(void)sys_read32((mem_addr_t)map.ptr);
	pcie_epc_unmap_host(dev, &map);
	return 0;
}

int advk_ep_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
		      uint16_t irq_num)
{
	struct advk_ep *ep = advk_ep(dev);

	if (func != 0) {
		return -EINVAL;
	}

	switch (type) {
	case PCIE_EPC_IRQ_INTX: {
		uint8_t pin = (advk_rd(ep, ADVK_CFG_INTR) >> 8) & 0xff;
		uint32_t bit = ADVK_CTRL0_INTA << (pin >= 1 && pin <= 4 ? pin - 1 : 0);

		rmw(ep, ADVK_CTRL0, bit, bit);
		k_busy_wait(1000);
		rmw(ep, ADVK_CTRL0, bit, 0);
		return 0;
	}
	case PCIE_EPC_IRQ_MSI:
		return ep->msi_cap != 0 ? raise_msi(dev, irq_num) : -ENOTSUP;
	default:
		return -ENOTSUP;
	}
}

int advk_ep_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val)
{
	if (func != 0 || offset >= 0x1000) {
		return -EINVAL;
	}
	*val = advk_rd(advk_ep(dev), offset & ~3U);
	return 0;
}

int advk_ep_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val)
{
	if (func != 0 || offset >= 0x1000) {
		return -EINVAL;
	}
	advk_wr(advk_ep(dev), offset & ~3U, val);
	return 0;
}

const struct pcie_epc_features *advk_ep_get_features(const struct device *dev, uint8_t func)
{
	ARG_UNUSED(func);
	return &advk_ep(dev)->features;
}

bool advk_ep_link_up(const struct device *dev)
{
	return advk_ltssm_is_up(advk_ltssm(advk_ep(dev)));
}

/*
 * The core may reset its registers on hot reset or link down; put the
 * header and MSI capability back when the class code went away.
 */
static void reapply(const struct device *dev)
{
	struct advk_ep *ep = advk_ep(dev);
	uint32_t want = ((uint32_t)ep->hdr.baseclass << 16) | ((uint32_t)ep->hdr.subclass << 8) |
			ep->hdr.prog_if;
	bool training = (advk_rd(ep, ADVK_CTRL0) & ADVK_CTRL0_LINK_TRAIN) != 0;

	if (!ep->hdr_valid || (advk_rd(ep, ADVK_CFG_REV_CLASS) >> 8) == want) {
		return;
	}

	ep->reapplies++;
	LOG_WRN("%s: registers lost (class %06x), reprogramming", dev->name,
		advk_rd(ep, ADVK_CFG_REV_CLASS) >> 8);
	advk_ep_hw_setup(dev);
	write_header(ep, &ep->hdr);
	if (ep->msi_cap != 0) {
		rmw(ep, ep->msi_cap, ADVK_MSI_CTRL_MMC, FIELD_PREP(ADVK_MSI_CTRL_MMC, ep->msi_mmc));
	}
	if (training) {
		advk_ep_start(dev);
	}
}

void advk_ep_poll(const struct device *dev, uint32_t now_ms)
{
	struct advk_ep *ep = advk_ep(dev);
	uint8_t s = advk_ltssm(ep);
	uint8_t prev = ep->ltssm;
	bool up = advk_ltssm_is_up(s);
	bool bme;

	if (s != ep->hist[ep->hist_head].state || ep->hist[ep->hist_head].ms == 0) {
		ep->hist_head = (ep->hist_head + 1) % ADVK_LTSSM_HIST;
		ep->hist[ep->hist_head].state = s;
		ep->hist[ep->hist_head].ms = MAX(now_ms, 1);
	}
	ep->ltssm = s;

	if (s == ADVK_LTSSM_HOT_RESET && prev != ADVK_LTSSM_HOT_RESET) {
		ep->hot_resets++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_HOT_RESET, 0);
	}

	if (up != ep->link_up) {
		ep->link_up = up;
		if (up) {
			ep->link_ups++;
			LOG_INF("%s: link up, lnksta %04x", dev->name,
				advk_rd(ep, ADVK_CFG_LNKCTL_STA) >> 16);
		} else {
			ep->link_downs++;
			LOG_INF("%s: link down (%s)", dev->name, advk_ltssm_name(s));
		}
		pcie_epc_notify(dev, up ? PCIE_EPC_EVT_LINK_UP : PCIE_EPC_EVT_LINK_DOWN, 0);
	}

	if (!up) {
		reapply(dev);
	}

	bme = up && (advk_rd(ep, ADVK_CFG_CMD_STATUS) & ADVK_CMD_MASTER) != 0;
	if (bme != ep->bme) {
		ep->bme = bme;
		pcie_epc_notify(dev, bme ? PCIE_EPC_EVT_BME : PCIE_EPC_EVT_BME_CLEARED, 0);
	}
}
