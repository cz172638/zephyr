/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP Layerscape glue of the DesignWare endpoint (Linux pci-layerscape-ep):
 * the role comes from the RCW (HOST_AGT_PEX), the outbound windows live in
 * the controller's address space, and the PF block (big-endian on LS1012A)
 * holds CFG_READY, which lets the host's configuration requests complete,
 * and the link event register.
 */

#define DT_DRV_COMPAT fsl_ls_pcie_ep

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>

#include "pcie_epc_dwc.h"

LOG_MODULE_REGISTER(pcie_epc_layerscape, CONFIG_PCIE_EP_LOG_LEVEL);

#define LS_PF_CONFIG       0x14
#define LS_PF_CFG_READY    BIT(0)
#define LS_PF_PME_MES_DR   0x20
#define LS_PF_PME_MES_LUD  BIT(7)
#define LS_PF_PME_MES_LDD  BIT(9)
#define LS_PF_PME_MES_HRD  BIT(10)
#define LS_PF_PME_MES_IER  0x28

struct ls_ep_config {
	uintptr_t regs_phys;
	size_t regs_size;
	uint64_t addr_phys;
	size_t ob_size;
	uint32_t dbi2_offset;
	uint32_t pf_offset;
	bool big_endian;
	uint8_t max_speed;
	uint8_t bar_fixed_64bit;
	bool autostart;
};

struct ls_ep_data {
	struct dwc_ep_data base; /* must be first */
	const struct device *dev;
	struct k_work_delayable poll_work;
	uint32_t events;
};

static uint32_t pf_rd(const struct device *dev, uint32_t off)
{
	const struct ls_ep_config *cfg = dev->config;
	uint32_t v = sys_read32(dwc_ep(dev)->dbi + cfg->pf_offset + off);

	return cfg->big_endian ? sys_be32_to_cpu(v) : v;
}

static void pf_wr(const struct device *dev, uint32_t off, uint32_t v)
{
	const struct ls_ep_config *cfg = dev->config;

	sys_write32(cfg->big_endian ? sys_cpu_to_be32(v) : v,
		    dwc_ep(dev)->dbi + cfg->pf_offset + off);
}

static void cfg_ready(const struct device *dev)
{
	pf_wr(dev, LS_PF_CONFIG, pf_rd(dev, LS_PF_CONFIG) | LS_PF_CFG_READY);
}

static void poll_handler(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct ls_ep_data *data = CONTAINER_OF(dw, struct ls_ep_data, poll_work);
	const struct device *dev = data->dev;
	struct pcie_epc_data *core = dev->data;
	uint32_t ev = pf_rd(dev, LS_PF_PME_MES_DR);

	if (ev != 0) {
		pf_wr(dev, LS_PF_PME_MES_DR, ev);
		data->events |= ev;
		if ((ev & LS_PF_PME_MES_LUD) != 0) {
			/*
			 * Max Link Width and Supported Link Speed are lost with
			 * a link down or hot reset: put back the RCW values.
			 */
			dwc_ep_restore_lnkcap(dev);
			if (core->started) {
				cfg_ready(dev);
			}
		}
		LOG_DBG("%s: PF events %08x", dev->name, ev);
	}
	dwc_ep_poll(dev);
	k_work_schedule(dw, K_MSEC(CONFIG_PCIE_EPC_DWC_POLL_MS));
}

static int ls_ep_start(const struct device *dev)
{
	cfg_ready(dev);
	return 0;
}

static void ls_ep_stop(const struct device *dev)
{
	pf_wr(dev, LS_PF_CONFIG, pf_rd(dev, LS_PF_CONFIG) & ~LS_PF_CFG_READY);
}

static int ls_ep_init(const struct device *dev)
{
	const struct ls_ep_config *cfg = dev->config;
	struct ls_ep_data *data = dev->data;
	struct dwc_ep *ep = dwc_ep(dev);
	mm_reg_t ob;
	uint8_t hdr;
	int ret;

	data->dev = dev;
	device_map(&ep->dbi, cfg->regs_phys, cfg->regs_size, K_MEM_CACHE_NONE);
	ep->dbi2 = ep->dbi + cfg->dbi2_offset;

	hdr = (dwc_rd(ep, DWC_CFG_BIST_HDR) >> 16) & 0x7f;
	if (hdr != 0) {
		LOG_INF("%s: header type %u, the RCW selects host mode (HOST_AGT_PEX = 0)",
			dev->name, hdr);
		return -EBUSY;
	}

	device_map(&ob, cfg->addr_phys, cfg->ob_size, K_MEM_CACHE_NONE);
	ep->ob_virt = ob;
	ep->ob_phys = cfg->addr_phys;
	ep->ob_size = cfg->ob_size;
	ep->max_speed = cfg->max_speed;
	ep->bar_fixed_64bit = cfg->bar_fixed_64bit;

	/* events are polled; masked at the PF, status cleared */
	pf_wr(dev, LS_PF_PME_MES_IER, 0);
	pf_wr(dev, LS_PF_PME_MES_DR, pf_rd(dev, LS_PF_PME_MES_DR));

	ret = dwc_ep_init(dev, cfg->autostart);
	if (ret != 0) {
		return ret;
	}

	k_work_init_delayable(&data->poll_work, poll_handler);
	k_work_schedule(&data->poll_work, K_MSEC(CONFIG_PCIE_EPC_DWC_POLL_MS));
	return 0;
}

static DEVICE_API(pcie_epc, ls_ep_api) = {
	.start = ls_ep_start,
	.stop = ls_ep_stop,
	.write_header = dwc_ep_write_header,
	.set_bar = dwc_ep_set_bar,
	.clear_bar = dwc_ep_clear_bar,
	.map_addr = dwc_ep_map_addr,
	.unmap_addr = dwc_ep_unmap_addr,
	.set_msi = dwc_ep_set_msi,
	.get_msi = dwc_ep_get_msi,
	.raise_irq = dwc_ep_raise_irq,
	.cfg_read = dwc_ep_cfg_read,
	.cfg_write = dwc_ep_cfg_write,
	.get_features = dwc_ep_get_features,
	.link_up = dwc_ep_link_up,
	.cpu_to_axi = dwc_ep_cpu_to_axi,
};

#define LS_EP_DEFINE(n)                                                                            \
	static const struct ls_ep_config ls_ep_cfg_##n = {                                         \
		.regs_phys = DT_INST_REG_ADDR_BY_NAME(n, regs),                                    \
		.regs_size = DT_INST_REG_SIZE_BY_NAME(n, regs),                                    \
		.addr_phys = DT_INST_REG_ADDR_BY_NAME_U64(n, addr_space),                          \
		.ob_size = MIN((uint64_t)DT_INST_REG_SIZE_BY_NAME(n, addr_space),                   \
			       (uint64_t)CONFIG_PCIE_EPC_DWC_OB_SIZE),                             \
		.dbi2_offset = DT_INST_PROP(n, dbi2_offset),                                       \
		.pf_offset = DT_INST_PROP(n, pf_offset),                                           \
		.big_endian = DT_INST_PROP(n, big_endian),                                         \
		.max_speed = DT_INST_PROP_OR(n, max_link_speed, 0),                                \
		.bar_fixed_64bit = DT_INST_PROP(n, bar_fixed_64bit),                               \
		.autostart = !DT_INST_PROP(n, zephyr_no_autostart),                                \
	};                                                                                         \
	static struct ls_ep_data ls_ep_data_##n;                                                   \
	DEVICE_DT_INST_DEFINE(n, ls_ep_init, NULL, &ls_ep_data_##n, &ls_ep_cfg_##n, POST_KERNEL,   \
			      CONFIG_PCIE_EPC_INIT_PRIORITY, &ls_ep_api);

DT_INST_FOREACH_STATUS_OKAY(LS_EP_DEFINE)
