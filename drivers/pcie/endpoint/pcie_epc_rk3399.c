/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 PCIe endpoint controller on the EPC API (pcie_epc.h):
 * Cadence core behind the Rockchip client wrapper, PHY in the GRF. The
 * same controller is the root complex in the other role (pcie_rk3399.c,
 * role from zephyr/drivers/pcie/role.h); bring-up shared in
 * pcie_rk3399_port.c.
 *
 * Outbound: the 32 MiB "mem" window is 32 Cadence regions of 1 MiB;
 * region 0 is kept for MSI writes, the rest is the EPC core's window.
 * MSI-X is advertised by the hardware but does not work (hidden).
 */

#define DT_DRV_COMPAT rockchip_rk3399_pcie_ep

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep_cdns.h>
#if defined(CONFIG_PCIE_ROLE)
#include <zephyr/drivers/pcie/role.h>
#endif
#if defined(CONFIG_PCIE_CTRL_SHELL)
#include <zephyr/drivers/pcie/pcie_shell.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_ep_cdns.h"
#include "../pcie_rk3399_port.h"

LOG_MODULE_REGISTER(pcie_epc_rk3399, CONFIG_PCIE_EP_LOG_LEVEL);

#define PCI_COMMAND_REG          0x04U
#define  PCI_COMMAND_MASTER      BIT(2)
#define  PCI_COMMAND_INTX_DIS    BIT(10)
#define PCI_EXP_LNKCTL           0x10U
#define  PCI_EXP_LNKCTL_RL       BIT(5)
#define CLIENT_INT_HOT_RST       BIT(13)

#define OB_REGIONS    32U
#define OB_REGION     MB(1)
#define OB_IRQ_REGION 0U
#define POLL_DOWN_MS  5
#define POLL_UP_MS    20

struct epc_rk3399_config {
	DEVICE_MMIO_NAMED_ROM(apb);
	uintptr_t mem_base;
	size_t mem_size;
	struct rk3399_pcie_port_cfg port;
	struct gpio_dt_spec perst;
	struct gpio_dt_spec slot_power;
	struct pcie_epc_header hdr;
	bool has_ids;
	uint8_t max_functions;
	uint8_t max_vfs; /* SR-IOV VFs of function 0, routing functions 1..max_vfs */
	uint8_t msi_vectors;
	bool autostart;
};

struct epc_rk3399_data {
	struct pcie_epc_data epc; /* must be first */
	DEVICE_MMIO_NAMED_RAM(apb);
	struct cdns_ep_core core;
	const struct device *dev;
	struct k_spinlock lock;
	uint32_t ob_map;
	uint8_t ob_count[OB_REGIONS];
	mm_reg_t irq_va;
	uint64_t irq_pci_addr;
	uint8_t irq_pci_fn;
	struct k_work_delayable link_work;
	struct gpio_callback perst_cb;
	struct pcie_epc_features features;
	struct pcie_epc_features vf_features; /* no INTx */
	bool active;
	bool perst_asserted;
	bool link_up;
	uint16_t bme; /* per routing function, PF and VFs */
	uint16_t vf_bar_users[6]; /* VFs holding the shared layout of each VF BAR */
	struct k_timer sriov_timer; /* ARI Capable Hierarchy for the VFs, see link_work */
	atomic_t arie_set;
	uint32_t ups, downs, hot_resets;
	int64_t last_up_log; /* uptime of the last "link up" INF line */
};

#define DEV_CFG(_dev)  ((const struct epc_rk3399_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct epc_rk3399_data *)(_dev)->data)

/* routing functions: the PF(s), then the SR-IOV VFs of function 0 */
static inline uint8_t nfuncs(const struct device *dev)
{
	return DEV_CFG(dev)->max_functions + DEV_CFG(dev)->max_vfs;
}

static inline bool is_vf(const struct device *dev, uint8_t func)
{
	return func >= DEV_CFG(dev)->max_functions && func < nfuncs(dev);
}

static const struct cdns_ep_variant rk3399_cdns_variant = {
	.ob_region_offset = 0x20U,
	.ib_bar_offset = 0x828U,
	.ob_max_nbits = 20U,
	.ob_has_cpu_addr = false,
	.did_via_rp_bank = true,
};

static inline mm_reg_t apb(const struct device *dev)
{
	return DEVICE_MMIO_NAMED_GET(dev, apb);
}

static bool active_role(void)
{
#if defined(CONFIG_PCIE_ROLE)
	return pcie_role_get() == PCIE_ROLE_EP;
#else
	return true;
#endif
}

/* --- link supervision ------------------------------------------------------ */

static void link_down(const struct device *dev)
{
	struct epc_rk3399_data *d = dev->data;

	if (d->link_up) {
		d->link_up = false;
		d->bme = 0;
		d->downs++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_LINK_DOWN, 0);
	}
}

/* 1 ms, ISR context: VF Enable without ARI Capable Hierarchy -> set it */
static void sriov_timer_fn(struct k_timer *t)
{
	struct epc_rk3399_data *d = CONTAINER_OF(t, struct epc_rk3399_data, sriov_timer);
	uint32_t reg = CDNS_EP_FUNC_SRIOV_CAP + CDNS_SRIOV_CTRL;
	uint32_t ctl = cdns_ep_read(&d->core, reg);

	if ((ctl & CDNS_SRIOV_CTRL_VFE) != 0U && (ctl & CDNS_SRIOV_CTRL_ARIE) == 0U) {
		cdns_ep_write(&d->core, reg, ctl | CDNS_SRIOV_CTRL_ARIE);
		atomic_set(&d->arie_set, 1);
	}
}

static void link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct epc_rk3399_data *d = CONTAINER_OF(dwork, struct epc_rk3399_data, link_work);
	const struct device *dev = d->dev;
	uint32_t ist;

	if (d->perst_asserted || !d->epc.started) {
		return;
	}

	ist = rk_client_read(apb(dev), RK_CLIENT_INT_STATUS);
	if (ist & CLIENT_INT_HOT_RST) {
		rk_client_write(apb(dev), RK_CLIENT_INT_STATUS, CLIENT_INT_HOT_RST);
		d->hot_resets++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_HOT_RESET, 0);
	}

	if (!rk3399_pcie_link_up(apb(dev)) ||
	    (cdns_ep_read(&d->core, CDNS_LM_PL_CONFIG0) & CDNS_LM_PL_CONFIG0_LS_READY) == 0U) {
		/*
		 * A data link down resets the function's configuration space
		 * (BARs, Command, SR-IOV Control): always a real link down.
		 */
		link_down(dev);
		(void)k_work_reschedule(&d->link_work, K_MSEC(POLL_DOWN_MS));
		return;
	}

	if (!d->link_up) {
		unsigned int gen, width;

		rk3399_pcie_link_info(apb(dev), &gen, &width);
		/* a flapping link would flood the console: one line per 10 s at most */
		if (d->last_up_log == 0 || k_uptime_get() - d->last_up_log > 10000) {
			LOG_INF("%s: link up, Gen%u x%u (ups %u downs %u)", dev->name, gen,
				width, d->ups + 1U, d->downs);
			d->last_up_log = k_uptime_get();
		} else {
			LOG_DBG("%s: link up, Gen%u x%u", dev->name, gen, width);
		}
		d->link_up = true;
		d->ups++;
		pcie_epc_notify(dev, PCIE_EPC_EVT_LINK_UP, 0);
	}

	/*
	 * The core needs ARI Capable Hierarchy whenever VF Enable is set (TRM,
	 * SR-IOV Control), but a host whose root port lacks ARI forwarding
	 * never sets it: the VFs then answer config requests with garbage.
	 * For VFs 1..7 the ARI and legacy routing IDs are the same (device 0),
	 * so sriov_timer sets it from the local side, within the 100 ms Linux
	 * waits after VF Enable before it scans the VFs; this only reports it.
	 */
	if (atomic_cas(&d->arie_set, 1, 0)) {
		LOG_INF("%s: VFs enabled (%u), ARI Capable Hierarchy set locally", dev->name,
				cdns_ep_read(&d->core, CDNS_EP_FUNC_SRIOV_CAP + CDNS_SRIOV_NUM_VFS) &
					0xffffU);
	}

	/* Bus Master Enable of every enabled function */
	for (uint8_t fn = 0; fn < nfuncs(dev); fn++) {
		bool on;

		if ((d->epc.func_map & BIT(fn)) == 0U && fn != 0U) {
			continue;
		}
		on = (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + PCI_COMMAND_REG) &
		      PCI_COMMAND_MASTER) != 0U;
		if (on != ((d->bme & BIT(fn)) != 0U)) {
			WRITE_BIT(d->bme, fn, on);
			pcie_epc_notify(dev, on ? PCIE_EPC_EVT_BME : PCIE_EPC_EVT_BME_CLEARED, fn);
		}
	}
	(void)k_work_reschedule(&d->link_work, K_MSEC(POLL_UP_MS));
}

static void retrain(struct epc_rk3399_data *d)
{
	uint32_t reg = CDNS_RP_BASE + CDNS_EP_FUNC_PCIE_CAP + PCI_EXP_LNKCTL;

	cdns_ep_write(&d->core, reg, cdns_ep_read(&d->core, reg) | PCI_EXP_LNKCTL_RL);
}

/* Rockchip GPIO v1 has no both-edge trigger: arm the opposite edge each time. */
static void perst_arm(const struct epc_rk3399_config *cfg, bool asserted)
{
	(void)gpio_pin_interrupt_configure_dt(&cfg->perst, asserted ? GPIO_INT_EDGE_TO_INACTIVE
								  : GPIO_INT_EDGE_TO_ACTIVE);
}

static void perst_handler(const struct device *port, struct gpio_callback *cb,
			  gpio_port_pins_t pins)
{
	struct epc_rk3399_data *d = CONTAINER_OF(cb, struct epc_rk3399_data, perst_cb);
	const struct device *dev = d->dev;
	const struct epc_rk3399_config *cfg = dev->config;
	int asserted = gpio_pin_get_dt(&cfg->perst);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	if (asserted < 0) {
		return;
	}
	perst_arm(cfg, asserted != 0);

	if (asserted != 0) {
		if (!d->perst_asserted) {
			d->perst_asserted = true;
			(void)k_work_cancel_delayable(&d->link_work);
			link_down(dev);
			pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_ASSERT, 0);
		}
		return;
	}
	if (d->perst_asserted) {
		d->perst_asserted = false;
		pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_DEASSERT, 0);
		if (d->epc.started) {
			retrain(d);
			(void)k_work_reschedule(&d->link_work, K_NO_WAIT);
		}
	}
}

/* --- EPC operations -------------------------------------------------------- */

static int epc_start(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	int ret;

	if (!d->active) {
		return -EBUSY;
	}
	cdns_ep_write(&d->core, CDNS_LM_EP_FUNC_CFG,
		      (d->epc.func_map | BIT(0)) & BIT_MASK(cfg->max_functions));
	rk_client_write(apb(dev), RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_LINK_TRAIN, 1U) | RK_WM16(RK_CLIENT_CONF_ENABLE, 1U));

	if (cfg->max_vfs != 0U) {
		k_timer_start(&d->sriov_timer, K_MSEC(1), K_MSEC(1));
	}
	if (cfg->perst.port != NULL) {
		ret = gpio_pin_get_dt(&cfg->perst);
		if (ret < 0) {
			return ret;
		}
		d->perst_asserted = (ret != 0);
		perst_arm(cfg, d->perst_asserted);
		if (d->perst_asserted) {
			LOG_INF("%s: waiting for PERST# deassertion", dev->name);
			return 0;
		}
	}
	(void)k_work_reschedule(&d->link_work, K_NO_WAIT);
	return 0;
}

static void epc_stop(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;

	if (!d->active) {
		return;
	}
	if (cfg->perst.port != NULL) {
		(void)gpio_pin_interrupt_configure_dt(&cfg->perst, GPIO_INT_DISABLE);
	}
	k_timer_stop(&d->sriov_timer);
	(void)k_work_cancel_delayable(&d->link_work);
	link_down(dev);
	rk_client_write(apb(dev), RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_LINK_TRAIN, 0U) | RK_WM16(RK_CLIENT_CONF_ENABLE, 0U));
}

static int epc_write_header(const struct device *dev, uint8_t func,
			    const struct pcie_epc_header *hdr)
{
	struct epc_rk3399_data *d = dev->data;
	struct pcie_ep_cdns_header h = {
		.vendor_id = hdr->vendor_id,
		.device_id = hdr->device_id,
		.revision_id = hdr->revision_id,
		.class_code = ((uint32_t)hdr->baseclass << 16) | ((uint32_t)hdr->subclass << 8) |
			      hdr->prog_if,
		.subsys_vendor_id = hdr->subsys_vendor_id,
		.subsys_id = hdr->subsys_id,
		.interrupt_pin = hdr->interrupt_pin,
	};

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	if (is_vf(dev, func)) {
		uint32_t base = CDNS_EP_FUNC_BASE(func);

		/*
		 * A VF reads Vendor ID 0xffff and the VF Device ID of its PF
		 * (one for all VFs); the rest of the header is per VF.
		 */
		cdns_ep_sriov_set_vf_did(&d->core, 0, hdr->device_id);
		cdns_ep_write(&d->core, base + 0x08U,
			      (uint32_t)h.revision_id | ((h.class_code & 0xffffffU) << 8));
		cdns_ep_write(&d->core, base + 0x2cU,
			      (uint32_t)h.subsys_vendor_id | ((uint32_t)h.subsys_id << 16));
		return 0;
	}
	return cdns_ep_write_header(&d->core, func, &h);
}

static int epc_set_bar(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t flags = 0;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	if (bar->flags & PCIE_EPC_BAR_IO) {
		flags |= PCIE_EP_CDNS_BAR_IO;
	}
	if (bar->flags & PCIE_EPC_BAR_MEM64) {
		flags |= PCIE_EP_CDNS_BAR_64BIT;
	}
	if (bar->flags & PCIE_EPC_BAR_PREFETCH) {
		flags |= PCIE_EP_CDNS_BAR_PREFETCH;
	}
	if (is_vf(dev, func)) {
		int ret = cdns_ep_set_vf_bar(&d->core, 0, func, bar->barno, (uintptr_t)bar->phys,
					     bar->size, flags);

		if (ret == 0) {
			d->vf_bar_users[bar->barno] |= BIT(func);
		}
		return ret;
	}
	return cdns_ep_set_bar(&d->core, func, bar->barno, (uintptr_t)bar->phys, bar->size,
			       flags);
}

static void epc_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func) && barno < 6U) {
		d->vf_bar_users[barno] &= ~BIT(func);
		(void)cdns_ep_clear_vf_bar(&d->core, 0, func, barno, d->vf_bar_users[barno] == 0U);
		return;
	}
	if (func < nfuncs(dev)) {
		(void)cdns_ep_clear_bar(&d->core, func, barno);
	}
}

/* cpu_addr: physical address in the window; up to 1 MiB per region */
static int epc_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
			uint64_t pci_addr, size_t size)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	uint32_t first, n;
	k_spinlock_key_t key;

	if (cpu_addr < cfg->mem_base + OB_REGION || size == 0 ||
	    cpu_addr + size > cfg->mem_base + cfg->mem_size) {
		return -EINVAL;
	}
	first = (cpu_addr - cfg->mem_base) / OB_REGION;
	n = DIV_ROUND_UP((cpu_addr - cfg->mem_base) % OB_REGION + size, OB_REGION);
	if (size < OB_REGION && ((cpu_addr | pci_addr) & (size - 1)) != 0) {
		return -EINVAL; /* translation replaces the low bits */
	}

	key = k_spin_lock(&d->lock);
	for (uint32_t i = 0; i < n; i++) {
		uint64_t chunk = MIN((uint64_t)size, (uint64_t)OB_REGION);
		uint8_t nbits = cdns_ep_ob_nbits(&d->core, pci_addr + i * OB_REGION, chunk);

		cdns_ep_ob_program(&d->core, func, first + i,
				   cfg->mem_base + (first + i) * OB_REGION,
				   pci_addr + (uint64_t)i * OB_REGION, nbits);
		d->ob_map |= BIT(first + i);
	}
	d->ob_count[first] = (uint8_t)n;
	k_spin_unlock(&d->lock, key);
	return 0;
}

static void epc_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	k_spinlock_key_t key;
	uint32_t first;

	ARG_UNUSED(func);
	if (cpu_addr < cfg->mem_base + OB_REGION || cpu_addr >= cfg->mem_base + cfg->mem_size) {
		return;
	}
	/*
	 * The window is Normal-NC: writes through it may still sit in the CPU
	 * write buffer. Drain them before the region goes, or they reach the
	 * core with no translation.
	 */
	barrier_dsync_fence_full();
	first = (cpu_addr - cfg->mem_base) / OB_REGION;
	key = k_spin_lock(&d->lock);
	for (uint32_t i = 0; i < MAX(d->ob_count[first], 1U) && first + i < OB_REGIONS; i++) {
		cdns_ep_ob_clear(&d->core, first + i);
		d->ob_map &= ~BIT(first + i);
	}
	d->ob_count[first] = 0;
	k_spin_unlock(&d->lock, key);
}

static int epc_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs)
{
	struct epc_rk3399_data *d = dev->data;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	return cdns_ep_set_msi(&d->core, func, nr_irqs);
}

static int epc_get_msi(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;
	uint32_t ctrl;

	if (func >= nfuncs(dev)) {
		return -EINVAL;
	}
	ctrl = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + CDNS_EP_FUNC_MSI_CAP);
	if ((ctrl & CDNS_MSI_CTRL_ME) == 0U) {
		return -EINVAL;
	}
	return 1 << ((ctrl & CDNS_MSI_CTRL_MME_MASK) >> CDNS_MSI_CTRL_MME_SHIFT);
}

static int raise_intx(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func)) {
		return -ENOTSUP; /* VFs have no INTx (SR-IOV) */
	}
	if (cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + PCI_COMMAND_REG) &
	    PCI_COMMAND_INTX_DIS) {
		return -EINVAL;
	}
	/* The TRM asks for some AHB clock cycles between edges; 1 ms is ample. */
	rk_client_write(apb(dev), RK_CLIENT_LEGACY_INT_CTRL,
			RK_WM16(RK_CLIENT_INT_IN, 1U) | RK_WM16(RK_CLIENT_INT_PEND_ST, 1U));
	k_busy_wait(1000);
	rk_client_write(apb(dev), RK_CLIENT_LEGACY_INT_CTRL,
			RK_WM16(RK_CLIENT_INT_IN, 0U) | RK_WM16(RK_CLIENT_INT_PEND_ST, 0U));
	return 0;
}

static int raise_msi(const struct device *dev, uint8_t func, uint16_t irq_num)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	k_spinlock_key_t key;
	uint64_t pci_addr, base;
	uint16_t data;
	int ret;

	ret = cdns_ep_msi_target(&d->core, func, irq_num, &pci_addr, &data);
	if (ret != 0) {
		return ret;
	}
	base = pci_addr & ~(uint64_t)0xffU;

	barrier_dsync_fence_full(); /* data written through the window goes first */
	key = k_spin_lock(&d->lock);
	if (d->irq_pci_addr != base || d->irq_pci_fn != func) {
		cdns_ep_ob_program(&d->core, func, OB_IRQ_REGION, cfg->mem_base, base, 8U);
		d->irq_pci_addr = base;
		d->irq_pci_fn = func;
	}
	sys_write16(data, d->irq_va + (mm_reg_t)(pci_addr & 0xffU));
	barrier_dsync_fence_full(); /* normal NC mapping: push the MSI write out */
	k_spin_unlock(&d->lock, key);
	return 0;
}

static int epc_raise_irq(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
			 uint16_t irq_num)
{
	struct epc_rk3399_data *d = dev->data;

	if (!d->link_up) {
		return -ENOTCONN;
	}
	if ((type == PCIE_EPC_IRQ_MSI || type == PCIE_EPC_IRQ_MSIX) && !(d->bme & BIT(func))) {
		return -EACCES; /* no Bus Master Enable: the write would be dropped */
	}
	switch (type) {
	case PCIE_EPC_IRQ_INTX:
		return raise_intx(dev, func);
	case PCIE_EPC_IRQ_MSI:
		return raise_msi(dev, func, irq_num);
	default:
		return -ENOTSUP;
	}
}

static int epc_cfg_read(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val)
{
	struct epc_rk3399_data *d = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U || func >= nfuncs(dev)) {
		return -EINVAL;
	}
	*val = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(func) + offset);
	return 0;
}

static int epc_cfg_write(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val)
{
	struct epc_rk3399_data *d = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U || func >= nfuncs(dev)) {
		return -EINVAL;
	}
	cdns_ep_write(&d->core, CDNS_EP_FUNC_BASE(func) + offset, val);
	return 0;
}

static const struct pcie_epc_features *epc_get_features(const struct device *dev, uint8_t func)
{
	struct epc_rk3399_data *d = dev->data;

	if (is_vf(dev, func)) {
		return &d->vf_features;
	}
	return func < nfuncs(dev) ? &d->features : NULL;
}

static bool epc_link_up(const struct device *dev)
{
	return DEV_DATA(dev)->link_up;
}

static DEVICE_API(pcie_epc, epc_rk3399_api) = {
	.start = epc_start,
	.stop = epc_stop,
	.write_header = epc_write_header,
	.set_bar = epc_set_bar,
	.clear_bar = epc_clear_bar,
	.map_addr = epc_map_addr,
	.unmap_addr = epc_unmap_addr,
	.set_msi = epc_set_msi,
	.get_msi = epc_get_msi,
	.raise_irq = epc_raise_irq,
	.cfg_read = epc_cfg_read,
	.cfg_write = epc_cfg_write,
	.get_features = epc_get_features,
	.link_up = epc_link_up,
};

/* --- pcie shell backend ---------------------------------------------------- */

#if defined(CONFIG_PCIE_CTRL_SHELL)
static bool sh_active(const struct device *dev)
{
	ARG_UNUSED(dev);
	return active_role();
}

static void sh_info(const struct device *dev, const struct shell *sh)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	uint8_t s;

	if (!d->active) {
		shell_print(sh, "endpoint idle (root complex role this boot)");
		return;
	}
	s = rk3399_pcie_ltssm(apb(dev));
	shell_print(sh, "ltssm      0x%02x %s, link %s, started %d, perst %s", s,
		    rk3399_pcie_ltssm_name(s), d->link_up ? "up" : "down", d->epc.started,
		    cfg->perst.port == NULL ? "-" : (d->perst_asserted ? "asserted" : "released"));
	if (d->link_up) {
		unsigned int gen, width;

		rk3399_pcie_link_info(apb(dev), &gen, &width);
		shell_print(sh, "link       Gen%u x%u (of x%u, max Gen%u)", gen, width,
			    cfg->port.lanes, cfg->port.link_gen);
	}
	shell_print(sh, "id         %08x  cmd/sts %08x  class %08x", 
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0)),
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 4),
		    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 8));
	shell_print(sh, "functions  %02x enabled, bme %02x, ob regions %08x", d->epc.func_map,
		    d->bme, d->ob_map);
	for (int b = 0; b < 6; b++) {
		shell_print(sh, "BAR%d       %08x", b,
			    cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(0) + 0x10 + 4 * b));
	}
	shell_print(sh, "events     up %u down %u hot-reset %u", d->ups, d->downs, d->hot_resets);
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

static int sh_link(const struct device *dev, const struct shell *sh, bool do_retrain)
{
	struct epc_rk3399_data *d = dev->data;

	if (!d->active) {
		return -EBUSY;
	}
	if (do_retrain) {
		retrain(d);
	}
	shell_print(sh, "link %s, ltssm %s", d->link_up ? "up" : "down",
		    rk3399_pcie_ltssm_name(rk3399_pcie_ltssm(apb(dev))));
	return 0;
}

static int sh_id_get(const struct device *dev, pcie_bdf_t bdf, struct pcie_shell_id *id)
{
	struct epc_rk3399_data *d = dev->data;
	uint8_t fn = PCIE_BDF_TO_FUNC(bdf);
	uint32_t v;

	if (!d->active || fn >= nfuncs(dev)) {
		return -EINVAL;
	}
	v = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn));
	id->vid = v & 0xffff;
	id->did = v >> 16;
	v = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + 0x2c);
	id->svid = v & 0xffff;
	id->ssid = v >> 16;
	id->class_code = cdns_ep_read(&d->core, CDNS_EP_FUNC_BASE(fn) + 8) >> 8;
	return 0;
}

/* applied now; nothing kept (no settings storage on this board yet) */
static int sh_id_set(const struct device *dev, pcie_bdf_t bdf, const struct pcie_shell_id *id)
{
	struct pcie_epc_header h = {
		.vendor_id = id->vid,
		.device_id = id->did,
		.subsys_vendor_id = id->svid,
		.subsys_id = id->ssid,
		.interrupt_pin = 1,
	};
	struct pcie_shell_id cur;
	uint32_t cc;

	if (sh_id_get(dev, bdf, &cur) != 0) {
		return -EINVAL;
	}
	cc = id->class_code == UINT32_MAX ? cur.class_code : id->class_code;
	h.baseclass = cc >> 16;
	h.subclass = cc >> 8;
	h.prog_if = cc;
	return epc_write_header(dev, PCIE_BDF_TO_FUNC(bdf), &h);
}

static uint32_t sh_cfg_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	uint32_t v;

	if (!DEV_DATA(dev)->active ||
	    epc_cfg_read(dev, PCIE_BDF_TO_FUNC(bdf), reg * 4U, &v) != 0) {
		return UINT32_MAX;
	}
	return v;
}

static void sh_cfg_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg, uint32_t v)
{
	if (DEV_DATA(dev)->active) {
		(void)epc_cfg_write(dev, PCIE_BDF_TO_FUNC(bdf), reg * 4U, v);
	}
}
#endif

/* --- init ------------------------------------------------------------------ */

static int epc_rk3399_init(const struct device *dev)
{
	const struct epc_rk3399_config *cfg = dev->config;
	struct epc_rk3399_data *d = dev->data;
	struct pcie_epc_ob_window ob = {
		.cpu_base = cfg->mem_base + OB_REGION,
		.axi_base = cfg->mem_base + OB_REGION,
		.size = cfg->mem_size - OB_REGION,
		.page_size = OB_REGION,
	};
	int ret;

	d->dev = dev;
#if defined(CONFIG_PCIE_CTRL_SHELL)
	static const struct pcie_shell_ctrl shell_ctrl = {
		.name = "rk3399",
		.dev = DEVICE_DT_INST_GET(0),
		.role = PCIE_SHELL_ROLE_EP,
		.active = sh_active,
		.info = sh_info,
#if defined(CONFIG_PCIE_ROLE)
		.mode_set = sh_mode_set,
		.mode_saved = sh_mode_saved,
#endif
		.link = sh_link,
		.id_get = sh_id_get,
		.id_set = sh_id_set,
		.cfg_read = sh_cfg_read,
		.cfg_write = sh_cfg_write,
	};

	pcie_shell_register(&shell_ctrl);
#endif

	if (!active_role()) {
		LOG_INF("%s: root complex role this boot, endpoint off", dev->name);
		return -EBUSY;
	}

	DEVICE_MMIO_NAMED_MAP(dev, apb, K_MEM_CACHE_NONE);
	d->core.base = apb(dev) + RK_CORE_OFFSET;
	d->core.variant = &rk3399_cdns_variant;
	d->irq_pci_addr = UINT64_MAX;
	d->ob_map = BIT(OB_IRQ_REGION);
	k_work_init_delayable(&d->link_work, link_work);
	k_timer_init(&d->sriov_timer, sriov_timer_fn, NULL);

	/* the slot's 3.3 V belongs to the host now */
	if (cfg->slot_power.port != NULL) {
		(void)gpio_pin_configure_dt(&cfg->slot_power, GPIO_OUTPUT_INACTIVE);
	}

	ret = rk3399_pcie_port_prepare(&cfg->port);
	if (ret == 0) {
		ret = rk3399_pcie_port_init(&cfg->port, apb(dev), false);
	}
	if (ret != 0) {
		LOG_ERR("%s: port init failed: %d", dev->name, ret);
		return ret;
	}
	LOG_INF("%s: core up, reset VID/SSVID 0x%08x", dev->name,
		cdns_ep_read(&d->core, CDNS_LM_ID));

	/*
	 * The core comes out of reset with BARs of its own enabled (BAR2/3
	 * 64-bit 4 MiB, BAR5 4 MiB on RK3399): disable all, the functions
	 * set the ones they use.
	 */
	for (uint8_t fn = 0; fn < cfg->max_functions; fn++) {
		for (uint8_t bar = 0; bar < 6U; bar++) {
			(void)cdns_ep_clear_bar(&d->core, fn, bar);
		}
	}
	/* the VF BARs too (reset: 3 x 64-bit 4 MiB), and their translations */
	for (uint8_t fn = 1; fn <= 8U; fn++) {
		for (uint8_t bar = 0; bar < 6U; bar++) {
			(void)cdns_ep_clear_vf_bar(&d->core, 0, fn, bar, true);
		}
	}
	if (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_SRIOV)) {
		cdns_ep_sriov_setup(&d->core, 0, cfg->max_vfs);
	}

	/* MSI-X is advertised but not functional on this revision. */
	cdns_ep_hide_msix_cap(&d->core);
	cdns_ep_write(&d->core, CDNS_LM_EP_FUNC_CFG, BIT(0));
	if (cfg->has_ids) {
		(void)epc_write_header(dev, 0, &cfg->hdr);
	}
	ret = cdns_ep_set_msi(&d->core, 0, cfg->msi_vectors);
	if (ret != 0) {
		return ret;
	}
	/*
	 * The whole outbound window is identity mapped (normal non-cacheable)
	 * by soc/rockchip/rk3399/mmu_regions.c, as pcie_epc_core hands out
	 * cpu addresses as pointers; mapping region 0 again would collide.
	 */
	d->irq_va = (mm_reg_t)(cfg->mem_base + OB_IRQ_REGION * OB_REGION);

	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
		gpio_init_callback(&d->perst_cb, perst_handler, BIT(cfg->perst.pin));
		ret = gpio_add_callback_dt(&cfg->perst, &d->perst_cb);
		if (ret != 0) {
			return ret;
		}
	}

	d->features = (struct pcie_epc_features){
		.linkup_notifier = true,
		.msi_capable = true,
		.msix_capable = false,
		.intx_capable = true,
		.align = 256,
		.max_functions = cfg->max_functions,
		.max_vfs = cfg->max_vfs,
		.ob_mmio_only = true, /* Normal-NC bursts into the window raise an SError */
	};
	d->vf_features = d->features;
	d->vf_features.intx_capable = false;

	ret = pcie_epc_init_common(dev, &ob, cfg->autostart);
	if (ret != 0) {
		return ret;
	}
	d->active = true;
	return 0;
}

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

#define EPC_RK3399_INIT(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP(n, max_functions) >= 1 &&                                        \
		     DT_INST_PROP(n, max_functions) <= CDNS_EP_MAX_FUNCS);                         \
	BUILD_ASSERT(DT_INST_PROP(n, max_virtual_functions) == 0 ||                               \
		     (IS_ENABLED(CONFIG_PCIE_EPC_RK3399_SRIOV) &&                                    \
		      DT_INST_PROP(n, max_functions) == 1 &&                                         \
		      DT_INST_PROP(n, max_virtual_functions) <= 8),                                \
		     "max-virtual-functions: 1..8 VFs of the one PF, needs PCIE_EPC_RK3399_SRIOV"); \
	static const struct epc_rk3399_config epc_rk3399_config_##n = {                            \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(apb, DT_DRV_INST(n)),                           \
		.mem_base = DT_INST_REG_ADDR_BY_NAME(n, mem),                                      \
		.mem_size = DT_INST_REG_SIZE_BY_NAME(n, mem),                                      \
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
		.has_ids = DT_INST_NODE_HAS_PROP(n, vendor_id) &&                                  \
			   DT_INST_NODE_HAS_PROP(n, device_id),                                    \
		.hdr = {                                                                           \
			.vendor_id = DT_INST_PROP_OR(n, vendor_id, 0),                             \
			.device_id = DT_INST_PROP_OR(n, device_id, 0),                             \
			.baseclass = (DT_INST_PROP(n, class_code) >> 16) & 0xff,                   \
			.subclass = (DT_INST_PROP(n, class_code) >> 8) & 0xff,                     \
			.prog_if = DT_INST_PROP(n, class_code) & 0xff,                             \
			.subsys_vendor_id = DT_INST_PROP(n, subsystem_vendor_id),                  \
			.subsys_id = DT_INST_PROP(n, subsystem_id),                                \
			.interrupt_pin = 1,                                                        \
		},                                                                                 \
		.max_functions = DT_INST_PROP(n, max_functions),                                   \
		.max_vfs = DT_INST_PROP(n, max_virtual_functions),                                 \
		.msi_vectors = DT_INST_PROP(n, msi_vectors),                                       \
		.autostart = !DT_INST_PROP(n, zephyr_no_autostart),                                \
	};                                                                                         \
	static struct epc_rk3399_data epc_rk3399_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, epc_rk3399_init, NULL, &epc_rk3399_data_##n,                      \
			      &epc_rk3399_config_##n, POST_KERNEL, CONFIG_PCIE_EPC_INIT_PRIORITY,  \
			      &epc_rk3399_api);

DT_INST_FOREACH_STATUS_OKAY(EPC_RK3399_INIT)
