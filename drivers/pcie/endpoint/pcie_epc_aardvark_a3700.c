/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3700 glue of the Aardvark endpoint: peripheral clock, COMPHY
 * lane 1 through the TF-A SiP service, the outbound memory window,
 * link polling, optional PERST# input and a bring-up shell.
 */

#define DT_DRV_COMPAT marvell_armada_3700_pcie_ep

#include <stdlib.h>
#include <string.h>

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>

#include <soc.h>

#include "pcie_epc_aardvark.h"
#include "pcie_epc_aardvark_a3700_xor.h"

LOG_MODULE_REGISTER(pcie_epc_a3700, CONFIG_PCIE_EP_LOG_LEVEL);

/* printk checkpoints: visible even when the init hangs with deferred logging */
#define TRACE(fmt, ...)                                                                            \
	do {                                                                                       \
		if (IS_ENABLED(CONFIG_PCIE_EPC_AARDVARK_TRACE_INIT)) {                             \
			printk("advk_ep: " fmt "\n", ##__VA_ARGS__);                               \
		}                                                                                  \
	} while (0)

struct a3700_ep_config {
	uintptr_t reg_phys;
	size_t reg_size;
	uintptr_t mem_phys;
	size_t mem_size;
	uintptr_t comphy_phys;
	/* DDR range inbound TLPs may reach (memory-region), size 0: Kconfig limit */
	uint64_t inbound_base;
	uint64_t inbound_size;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
	struct gpio_dt_spec perst;
	uint8_t comphy_lane;
	uint8_t comphy_invert;
	bool comphy_skip;
	bool refclk_in;
	uint8_t num_lanes;
	uint8_t max_speed;
	bool autostart;
};

struct a3700_ep_data {
	struct advk_ep_data base; /* must be first */
	const struct device *dev;
	struct k_work_delayable poll_work;
	bool perst_asserted;
	long comphy_ret;
	mm_reg_t comphy;
};

/*
 * Inbound: a TLP hitting a BAR goes through the BAR's inbound translation
 * (set_bar) to an AXI address, then through the PCIe unit's IO decode
 * window 0 (0xd000c500, TF-A sets it to all of DRAM, no remap) into DDR.
 * Shrinking that window to the BAR backing store keeps the host out of
 * the rest of memory: anything else gets a completer abort.
 */
#define IODEC_PCIE_WIN0 0xd000c500UL

static void inbound_window(uint64_t base, uint64_t size)
{
	uint8_t *va;
	volatile uint32_t *r;
	uint32_t ctrl;

	if (size == 0) {
		return;
	}
	if ((base & 0xffff) != 0 || (size & 0xffff) != 0) {
		LOG_ERR("inbound window 0x%llx+0x%llx not in 64 KiB units, TF-A's kept",
			(unsigned long long)base, (unsigned long long)size);
		return;
	}
	k_mem_map_phys_bare(&va, IODEC_PCIE_WIN0 & ~0xfffUL, 0x1000,
			    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	r = (volatile uint32_t *)(va + (IODEC_PCIE_WIN0 & 0xfff));
	/* window 1 off, window 0: attr and target as TF-A set them */
	r[4] = r[4] & ~1U;
	ctrl = (r[0] & 0xfff0) | ((uint32_t)((size >> 16) - 1) << 16);
	r[0] = r[0] & ~1U;
	r[1] = (uint32_t)(base >> 16) << 16;
	r[2] = 0;
	r[0] = ctrl;
	r[0] = ctrl | 1;
	LOG_INF("inbound limited to DDR 0x%llx-0x%llx (IO decode ctrl %08x base %08x)",
		(unsigned long long)base, (unsigned long long)(base + size - 1), r[0], r[1]);
	k_mem_unmap_phys_bare(va, 0x1000);
}

/* TF-A enables the PHY clock output to the REFCLK pins; an EP must not drive it */
static void comphy_clkout_off(const struct device *dev)
{
	struct a3700_ep_data *data = dev->data;
	mem_addr_t r = data->comphy + ADVK_COMPHY_MISC_CTRL0;

	sys_write16(sys_read16(r) & ~ADVK_COMPHY_CLK100M_125M_EN, r);
}

static long comphy_power_on(const struct device *dev)
{
	const struct a3700_ep_config *cfg = dev->config;
	struct arm_smccc_res res;

	arm_smccc_smc(ADVK_SIP_COMPHY_POWER_ON, cfg->comphy_lane,
		      ADVK_COMPHY_PARAM(cfg->comphy_invert, cfg->num_lanes), 0, 0, 0, 0, 0, &res);
	return (long)res.a0;
}

static void perst_poll(const struct device *dev)
{
	const struct a3700_ep_config *cfg = dev->config;
	struct a3700_ep_data *data = dev->data;
	struct pcie_epc_data *core = dev->data;
	int v;

	if (cfg->perst.port == NULL) {
		return;
	}

	v = gpio_pin_get_dt(&cfg->perst);
	if (v < 0 || (v != 0) == data->perst_asserted) {
		return;
	}

	data->perst_asserted = v != 0;
	if (data->perst_asserted) {
		LOG_INF("%s: PERST# asserted", dev->name);
		advk_ep_stop(dev);
		pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_ASSERT, 0);
	} else {
		LOG_INF("%s: PERST# deasserted", dev->name);
		if (core->started) {
			advk_ep_start(dev);
		}
		pcie_epc_notify(dev, PCIE_EPC_EVT_PERST_DEASSERT, 0);
	}
}

static void poll_handler(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct a3700_ep_data *data = CONTAINER_OF(dw, struct a3700_ep_data, poll_work);

	perst_poll(data->dev);
	advk_ep_poll(data->dev, k_uptime_get_32());
	k_work_schedule(dw, K_MSEC(CONFIG_PCIE_EPC_AARDVARK_POLL_MS));
}

static int a3700_ep_start(const struct device *dev)
{
	struct a3700_ep_data *data = dev->data;

	/* with PERST# wired, training starts on its deassertion */
	if (data->perst_asserted) {
		return 0;
	}
	return advk_ep_start(dev);
}

static int a3700_ep_init(const struct device *dev)
{
	const struct a3700_ep_config *cfg = dev->config;
	struct a3700_ep_data *data = dev->data;
	struct advk_ep *ep = advk_ep(dev);
	mm_reg_t ob;
	int ret;

	data->dev = dev;
	TRACE("init");

	/* PCIE1_RESETn is an output of the controller: GPIO in EP mode */
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}
	}

	TRACE("pinctrl done, clock on");
	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	ret = clock_control_on(cfg->clk_dev, cfg->clk_id);
	if (ret != 0) {
		LOG_ERR("clock on failed (%d)", ret);
		return ret;
	}

	TRACE("clock on, COMPHY SiP call");
	if (!cfg->comphy_skip) {
		data->comphy_ret = comphy_power_on(dev);
		if (data->comphy_ret != 0) {
			/* keep going: the registers are reachable for the shell */
			LOG_ERR("COMPHY lane %u PCIe power on failed (%ld)", cfg->comphy_lane,
				data->comphy_ret);
		}
	}

	TRACE("COMPHY returned %ld, clock output off", data->comphy_ret);
	device_map(&data->comphy, cfg->comphy_phys, 0x400, K_MEM_CACHE_NONE);
	if (!cfg->comphy_skip) {
		comphy_clkout_off(dev);
	}

	TRACE("misc_ctrl0 %04x, mapping controller and OB window",
	      sys_read16(data->comphy + ADVK_COMPHY_MISC_CTRL0));
	device_map(&ep->reg, cfg->reg_phys, cfg->reg_size, K_MEM_CACHE_NONE);
	device_map(&ob, cfg->mem_phys, cfg->mem_size, K_MEM_CACHE_NONE);
	TRACE("ctrl at 0x%lx, OB at 0x%lx; ctrl0 %08x", (unsigned long)ep->reg,
	      (unsigned long)ob, sys_read32(ep->reg + ADVK_CTRL0));
	ep->ob_virt = ob;
	ep->ob_phys = cfg->mem_phys;
	ep->ob_size = cfg->mem_size;
	ep->num_lanes = cfg->num_lanes;
	ep->max_speed = cfg->max_speed;
	ep->refclk_in = cfg->refclk_in;

	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}
		data->perst_asserted = gpio_pin_get_dt(&cfg->perst) > 0;
	}

	if (cfg->inbound_size != 0) {
		inbound_window(cfg->inbound_base, cfg->inbound_size);
	} else {
		inbound_window(0, CONFIG_PCIE_EPC_AARDVARK_INBOUND_LIMIT);
	}

#ifdef CONFIG_PCIE_EPC_AARDVARK_DMA
	a3700_xor_pcie_window(cfg->mem_phys, cfg->mem_size, CONFIG_PCIE_EPC_AARDVARK_DMA_ATTR);
#endif

	TRACE("controller setup");
	ret = advk_ep_init(dev, cfg->autostart);
	if (ret != 0) {
		return ret;
	}
	TRACE("ready, ltssm 0x%02x", advk_ltssm(ep));

	k_work_init_delayable(&data->poll_work, poll_handler);
	k_work_schedule(&data->poll_work, K_MSEC(CONFIG_PCIE_EPC_AARDVARK_POLL_MS));

	LOG_INF("%s: x%u Gen%u, OB 0x%lx+0x%zx, comphy %ld%s", dev->name, cfg->num_lanes,
		cfg->max_speed, (unsigned long)cfg->mem_phys, cfg->mem_size, data->comphy_ret,
		cfg->perst.port != NULL ? ", PERST# on GPIO" : "");
	return 0;
}

#ifdef CONFIG_PCIE_EPC_AARDVARK_DMA
/*
 * System DMA through the SoC XOR engine. The host side goes through an
 * outbound window as for memcpy; the engine reaches it through the DMA
 * unit's IO decode window set up at init. Short copies stay on the CPU.
 */
static int a3700_ep_xfer_dma(const struct device *dev, uint8_t func, uint64_t pci_addr,
			     void *local, size_t size, bool to_host)
{
	struct pcie_epc_host_map map;
	uint64_t host_axi, local_phys;
	int ret;

	if (size < 128) {
		return pcie_epc_xfer_memcpy(dev, func, pci_addr, local, size, to_host);
	}
	if (size >= 0x1000000) {
		return -EINVAL;
	}
	ret = pcie_epc_map_host(dev, func, pci_addr, size, &map);
	if (ret != 0) {
		return ret;
	}
	host_axi = advk_ep_cpu_to_axi(dev, (uintptr_t)map.ptr);
	local_phys = k_mem_phys_addr(local);
	if (((host_axi | local_phys) >> 32) != 0) {
		ret = -EINVAL;
		goto out;
	}

	sys_cache_data_flush_and_invd_range(local, size);
	if (to_host) {
		ret = a3700_xor_memcpy((uint32_t)host_axi, (uint32_t)local_phys, size, NULL);
		/* non-posted read: the writes are at the host before an IRQ */
		(void)sys_read8((mem_addr_t)map.ptr);
	} else {
		ret = a3700_xor_memcpy((uint32_t)local_phys, (uint32_t)host_axi, size, NULL);
		sys_cache_data_invd_range(local, size);
	}
out:
	pcie_epc_unmap_host(dev, &map);
	return ret;
}
#endif

static DEVICE_API(pcie_epc, a3700_ep_api) = {
	.start = a3700_ep_start,
	.stop = advk_ep_stop,
	.write_header = advk_ep_write_header,
	.set_bar = advk_ep_set_bar,
	.clear_bar = advk_ep_clear_bar,
	.map_addr = advk_ep_map_addr,
	.unmap_addr = advk_ep_unmap_addr,
	.set_msi = advk_ep_set_msi,
	.get_msi = advk_ep_get_msi,
	.raise_irq = advk_ep_raise_irq,
	.cfg_read = advk_ep_cfg_read,
	.cfg_write = advk_ep_cfg_write,
	.get_features = advk_ep_get_features,
	.link_up = advk_ep_link_up,
	.cpu_to_axi = advk_ep_cpu_to_axi,
#ifdef CONFIG_PCIE_EPC_AARDVARK_DMA
	.xfer_dma = a3700_ep_xfer_dma,
#endif
};

#define A3700_EP_HAS_PINCTRL(n) DT_INST_PINCTRL_HAS_NAME(n, default)

#define A3700_EP_DEFINE(n)                                                                         \
	IF_ENABLED(A3700_EP_HAS_PINCTRL(n), (PINCTRL_DT_INST_DEFINE(n);))                         \
	static const struct a3700_ep_config a3700_ep_cfg_##n = {                                   \
		.pcfg = COND_CODE_1(A3700_EP_HAS_PINCTRL(n), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),  \
				    (NULL)),                                                       \
		.reg_phys = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                     \
		.reg_size = DT_INST_REG_SIZE_BY_NAME(n, ctrl),                                     \
		.mem_phys = DT_INST_REG_ADDR_BY_NAME(n, mem),                                      \
		.mem_size = DT_INST_REG_SIZE_BY_NAME(n, mem),                                      \
		.comphy_phys = DT_INST_REG_ADDR_BY_NAME(n, comphy),                                \
		.inbound_base = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),              \
				(DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region))), (0)),            \
		.inbound_size = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),              \
				(DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region))), (0)),            \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, index),                   \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.comphy_lane = DT_INST_PROP(n, marvell_comphy_lane),                               \
		.comphy_invert = DT_INST_PROP(n, marvell_comphy_polarity_invert),                  \
		.comphy_skip = DT_INST_PROP(n, marvell_comphy_skip),                               \
		.refclk_in = DT_INST_PROP(n, marvell_refclk_in),                                   \
		.num_lanes = DT_INST_PROP_OR(n, num_lanes, 1),                                     \
		.max_speed = DT_INST_PROP_OR(n, max_link_speed, 2),                                \
		.autostart = !DT_INST_PROP(n, zephyr_no_autostart),                                \
	};                                                                                         \
	static struct a3700_ep_data a3700_ep_data_##n;                                             \
	DEVICE_DT_INST_DEFINE(n, a3700_ep_init, NULL, &a3700_ep_data_##n, &a3700_ep_cfg_##n,       \
			      POST_KERNEL, CONFIG_PCIE_EPC_INIT_PRIORITY, &a3700_ep_api);

DT_INST_FOREACH_STATUS_OKAY(A3700_EP_DEFINE)

#ifdef CONFIG_PCIE_EPC_AARDVARK_SHELL

#define EP_DEV DEVICE_DT_INST_GET(0)

static int parse_u(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

static void dump_words(const struct shell *sh, struct advk_ep *ep, uint32_t off, uint32_t len)
{
	off &= ~3U;
	for (uint32_t o = off; o < off + len && o < ADVK_REG_SPACE; o += 16) {
		shell_fprintf(sh, SHELL_NORMAL, "%05x:", o);
		for (uint32_t i = 0; i < 16 && o + i < off + len; i += 4) {
			uint32_t v;

			/* holes in the register space abort: show them, go on */
			if (a3700_probe_read32(ep->reg + o + i, &v) < 0) {
				shell_fprintf(sh, SHELL_NORMAL, " ????????");
			} else {
				shell_fprintf(sh, SHELL_NORMAL, " %08x", v);
			}
		}
		shell_fprintf(sh, SHELL_NORMAL, "\n");
	}
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = EP_DEV;
	struct advk_ep *ep = advk_ep(dev);
	struct a3700_ep_data *data = dev->data;
	struct pcie_epc_data *core = dev->data;
	uint32_t lnk = advk_rd(ep, ADVK_CFG_LNKCTL_STA) >> 16;
	uint8_t s = advk_ltssm(ep);

	shell_print(sh, "ltssm      0x%02x %s, link %s, started %d, perst %s", s,
		    advk_ltssm_name(s), advk_ltssm_is_up(s) ? "up" : "down", core->started,
		    data->perst_asserted ? "asserted" : "-");
	shell_print(sh, "lnksta     %04x: Gen%u x%u", lnk, lnk & 0xf, (lnk >> 4) & 0x3f);
	shell_print(sh, "id         %08x  cmd/sts %08x  class %08x  ssid %08x",
		    advk_rd(ep, ADVK_CFG_ID), advk_rd(ep, ADVK_CFG_CMD_STATUS),
		    advk_rd(ep, ADVK_CFG_REV_CLASS), advk_rd(ep, ADVK_CFG_SSID));
	shell_print(sh, "lmi vendor %08x  lmi cfg %08x", advk_rd(ep, ADVK_LMI_VENDOR_ID),
		    advk_rd(ep, ADVK_LMI_CFG));
	shell_print(sh, "ctrl0      %08x  ctrl2 %08x  refclk %08x  mode %08x",
		    advk_rd(ep, ADVK_CTRL0), advk_rd(ep, ADVK_CTRL2), advk_rd(ep, ADVK_REF_CLK),
		    advk_rd(ep, ADVK_CTRL_CONFIG));
	shell_print(sh, "isr0       %08x  isr1 %08x  host int %08x  msg log %08x",
		    advk_rd(ep, ADVK_ISR0), advk_rd(ep, ADVK_ISR1),
		    advk_rd(ep, ADVK_HOST_INT_STATUS), advk_rd(ep, ADVK_MSG_LOG));
	for (int b = 0; b < PCIE_EPC_NUM_BARS; b++) {
		shell_print(sh, "BAR%d       %08x  wanted 0x%zx @ 0x%llx", b,
			    advk_rd(ep, ADVK_CFG_BAR(b)), ep->bar[b].size,
			    (unsigned long long)ep->bar[b].phys);
	}
	if (ep->msi_cap != 0) {
		shell_print(sh, "MSI @%02x    %08x %08x %08x %08x", ep->msi_cap,
			    advk_rd(ep, ep->msi_cap), advk_rd(ep, ep->msi_cap + 4),
			    advk_rd(ep, ep->msi_cap + 8), advk_rd(ep, ep->msi_cap + 12));
	} else {
		shell_print(sh, "MSI        no capability found");
	}
	for (int w = 0; w < ADVK_OB_WIN_COUNT; w++) {
		if (ep->ob_cpu[w] != 0) {
			shell_print(sh, "OB%d        cpu 0x%lx match %08x remap %08x%08x mask %08x",
				    w, (unsigned long)ep->ob_cpu[w],
				    advk_rd(ep, ADVK_OB_MATCH_LS(w)),
				    advk_rd(ep, ADVK_OB_REMAP_MS(w)),
				    advk_rd(ep, ADVK_OB_REMAP_LS(w)),
				    advk_rd(ep, ADVK_OB_MASK_LS(w)));
		}
	}
	shell_print(sh, "events     up %u down %u hot-reset %u reprogrammed %u, comphy %ld",
		    ep->link_ups, ep->link_downs, ep->hot_resets, ep->reapplies,
		    data->comphy_ret);
	return 0;
}

static int cmd_hist(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);

	for (int i = 1; i <= ADVK_LTSSM_HIST; i++) {
		int k = (ep->hist_head + i) % ADVK_LTSSM_HIST;

		if (ep->hist[k].ms != 0) {
			shell_print(sh, "%8u ms  0x%02x %s", ep->hist[k].ms, ep->hist[k].state,
				    advk_ltssm_name(ep->hist[k].state));
		}
	}
	return 0;
}

static int cmd_trace(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long ms = 100;
	struct {
		uint8_t s;
		uint32_t us;
	} t[64];
	int n = 0;
	uint32_t t0 = k_cycle_get_32();
	uint32_t end_us;
	uint8_t last = 0xff;

	if (argc > 1 && parse_u(sh, argv[1], &ms) != 0) {
		return -EINVAL;
	}
	end_us = (uint32_t)MIN(ms, 10000ULL) * 1000U;

	for (;;) {
		uint32_t us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);
		uint8_t s = advk_ltssm(ep);

		if (s != last && n < ARRAY_SIZE(t)) {
			t[n].s = s;
			t[n].us = us;
			n++;
			last = s;
		}
		if (us >= end_us) {
			break;
		}
		k_busy_wait(5);
	}

	for (int i = 0; i < n; i++) {
		shell_print(sh, "%8u us  0x%02x %s", t[i].us, t[i].s, advk_ltssm_name(t[i].s));
	}
	if (n == ARRAY_SIZE(t)) {
		shell_print(sh, "(truncated)");
	}
	return 0;
}

static const struct {
	const char *name;
	uint32_t off;
	uint32_t len;
} regions[] = {
	{"cfg", 0x0, 0x200},	{"ctrl", 0x4800, 0xa0},	   {"ob", 0x4c00, 0x100},
	{"lmi", 0x6000, 0x200}, {"core", 0x18000, 0x40}, {"irq", 0x1b000, 0x20},
};

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long off, len = 0x40;

	if (argc < 2) {
		for (int i = 0; i < ARRAY_SIZE(regions); i++) {
			shell_print(sh, "-- %s", regions[i].name);
			dump_words(sh, ep, regions[i].off, regions[i].len);
		}
		return 0;
	}
	for (int i = 0; i < ARRAY_SIZE(regions); i++) {
		if (strcmp(argv[1], regions[i].name) == 0) {
			dump_words(sh, ep, regions[i].off, regions[i].len);
			return 0;
		}
	}
	if (parse_u(sh, argv[1], &off) != 0 || (argc > 2 && parse_u(sh, argv[2], &len) != 0)) {
		return -EINVAL;
	}
	if (off >= ADVK_REG_SPACE) {
		shell_error(sh, "offset beyond 0x%x", ADVK_REG_SPACE);
		return -EINVAL;
	}
	dump_words(sh, ep, off, len);
	return 0;
}

static int cmd_rd(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long off;

	if (parse_u(sh, argv[1], &off) != 0 || off >= ADVK_REG_SPACE) {
		return -EINVAL;
	}
	{
		uint32_t v;

		if (a3700_probe_read32(advk_ep(EP_DEV)->reg + (off & ~3U), &v) < 0) {
			shell_print(sh, "%05llx: abort", off & ~3ULL);
			return -EIO;
		}
		shell_print(sh, "%05llx: %08x", off & ~3ULL, v);
	}
	return 0;
}

static int cmd_wr(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long off, val;

	if (parse_u(sh, argv[1], &off) != 0 || parse_u(sh, argv[2], &val) != 0 ||
	    off >= ADVK_REG_SPACE) {
		return -EINVAL;
	}
	{
		uint32_t v;

		if (a3700_probe_write32(ep->reg + (off & ~3U), (uint32_t)val) < 0) {
			shell_print(sh, "%05llx: write aborted", off & ~3ULL);
			return -EIO;
		}
		if (a3700_probe_read32(ep->reg + (off & ~3U), &v) < 0) {
			shell_print(sh, "%05llx: read back aborted", off & ~3ULL);
			return -EIO;
		}
		shell_print(sh, "%05llx: %08x", off & ~3ULL, v);
	}
	return 0;
}

/* Which bits of a register can be changed: write the inverse, read, restore */
static int cmd_probe(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long off, len = 4;

	if (parse_u(sh, argv[1], &off) != 0 || (argc > 2 && parse_u(sh, argv[2], &len) != 0) ||
	    off >= ADVK_REG_SPACE) {
		return -EINVAL;
	}

	for (uint32_t o = off & ~3U; o < off + len && o < ADVK_REG_SPACE; o += 4) {
		uintptr_t r = ep->reg + o;
		uint32_t orig, inv, ones, zeros, now;

		if (a3700_probe_read32(r, &orig) < 0) {
			shell_print(sh, "%05x: read aborts", o);
			continue;
		}
		if (a3700_probe_write32(r, ~orig) < 0 || a3700_probe_read32(r, &inv) < 0 ||
		    a3700_probe_write32(r, UINT32_MAX) < 0 || a3700_probe_read32(r, &ones) < 0 ||
		    a3700_probe_write32(r, 0) < 0 || a3700_probe_read32(r, &zeros) < 0) {
			(void)a3700_probe_write32(r, orig);
			shell_print(sh, "%05x: %08x  write aborts (read only?)", o, orig);
			continue;
		}
		(void)a3700_probe_write32(r, orig);
		(void)a3700_probe_read32(r, &now);
		shell_print(sh, "%05x: %08x  rw %08x  ~ %08x  1s %08x  0s %08x  now %08x", o, orig,
			    inv ^ orig, inv, ones, zeros, now);
	}
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	return pcie_epc_start(EP_DEV);
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	pcie_epc_stop(EP_DEV);
	return 0;
}

static int cmd_setup(const struct shell *sh, size_t argc, char **argv)
{
	advk_ep_hw_setup(EP_DEV);
	shell_print(sh, "controller reprogrammed, training off; header/BARs need the EPF again");
	return 0;
}

/*
 * map [start end]: which words of the register space read without an
 * abort; prints readable ranges and how many of their words are non-zero.
 * Reads only (read-to-clear registers get cleared).
 */
static int cmd_map(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long start = 0, end = ADVK_REG_SPACE;
	uint32_t run = 0, nz = 0;
	bool in = false;

	if (argc > 2 && (parse_u(sh, argv[1], &start) != 0 || parse_u(sh, argv[2], &end) != 0)) {
		return -EINVAL;
	}
	end = MIN(end, (unsigned long long)ADVK_REG_SPACE);
	for (uint32_t o = start & ~3U; o <= end; o += 4) {
		uint32_t v = 0;
		bool ok = o < end && a3700_probe_read32(ep->reg + o, &v) == 0;

		if (ok && !in) {
			in = true;
			run = o;
			nz = 0;
		}
		if (ok && v != 0) {
			nz++;
		}
		if (!ok && in) {
			in = false;
			shell_print(sh, "%05x-%05x  %5u words, %u non-zero", run, o - 1, (o - run) / 4,
				    nz);
		}
	}
	return 0;
}

/*
 * xor <dst> <src> <len>: one memcpy with the SoC XOR engine (unit 0,
 * channel 0), as Linux mv_xor does on the A3700: an address prefixed
 * with h: is host memory (PCI address), reached through an outbound
 * window. Probing whether a system DMA master gets to PCIe at all.
 */

static int xor_addr(const struct shell *sh, const char *arg, size_t len, uint64_t *axi,
		    struct pcie_epc_host_map *map, bool *mapped)
{
	unsigned long long v;
	bool host = strncmp(arg, "h:", 2) == 0;

	if (parse_u(sh, host ? arg + 2 : arg, &v) != 0) {
		return -EINVAL;
	}
	*mapped = false;
	if (!host) {
		*axi = v;
		return 0;
	}
	if (pcie_epc_map_host(EP_DEV, 0, v, len, map) != 0) {
		shell_error(sh, "no outbound window for host 0x%llx (bus mastering on?)", v);
		return -ENOSPC;
	}
	*mapped = true;
	*axi = pcie_epc_cpu_to_axi(EP_DEV, (uintptr_t)map->ptr);
	return 0;
}

static int cmd_xor(const struct shell *sh, size_t argc, char **argv)
{
	struct pcie_epc_host_map dmap, smap;
	bool dm = false, sm = false;
	unsigned long long len;
	uint64_t dst, src;
	struct a3700_xor_result r;
	int64_t t0;
	int ret = -EINVAL;

	if (parse_u(sh, argv[3], &len) != 0 || len < 128 || len >= 0x1000000) {
		shell_error(sh, "len 128..16M-1");
		return -EINVAL;
	}
	if (xor_addr(sh, argv[1], len, &dst, &dmap, &dm) != 0 ||
	    xor_addr(sh, argv[2], len, &src, &smap, &sm) != 0) {
		goto out;
	}
	if ((dst | src) >> 32) {
		shell_error(sh, "XOR addresses are 32 bit");
		goto out;
	}

	t0 = k_uptime_get();
	ret = a3700_xor_memcpy((uint32_t)dst, (uint32_t)src, (uint32_t)len, &r);
	shell_print(sh, "xor 0x%llx <- 0x%llx, %llu bytes: %d, cause %08x error %08x @%08x, "
		    "desc status %08x, %lld ms", (unsigned long long)dst, (unsigned long long)src,
		    len, ret, r.cause, r.err, r.err_addr, r.status, k_uptime_get() - t0);
out:
	if (dm) {
		pcie_epc_unmap_host(EP_DEV, &dmap);
	}
	if (sm) {
		pcie_epc_unmap_host(EP_DEV, &smap);
	}
	return ret;
}

/*
 * CPU address decode windows (0xd000cf00, 0x10 each: ctrl = attr << 8 |
 * target << 4 | enable, size = (size/64K - 1), base and remap in 64K units).
 * Targets per TF-A dram_win.c: 0 DRAM, 1 internal regs, 2 PCIe, 3 PCIe over
 * MCI, 4 BootROM, 5 MCI external, 7 rWTM RAM, 8 CCI-400 regs.
 */
#define CPU_DEC_WIN 0xd000cf00UL

static int cmd_cpuwin(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t *va;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	k_mem_map_phys_bare(&va, CPU_DEC_WIN & ~0xfffUL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	for (int i = 0; i < 16; i++) {
		uint32_t v[4];
		bool ok = true;

		for (int j = 0; j < 4; j++) {
			ok &= a3700_probe_read32((uintptr_t)va + (CPU_DEC_WIN & 0xfff) + i * 0x10 + j * 4,
						 &v[j]) == 0;
		}
		if (!ok) {
			shell_print(sh, "win %2d: abort", i);
			continue;
		}
		if ((v[0] | v[1] | v[2] | v[3]) == 0) {
			continue;
		}
		shell_print(sh, "win %2d: %08x %08x %08x %08x  %s target %u attr 0x%02x "
			    "0x%08llx+0x%llx remap 0x%08llx", i, v[0], v[1], v[2], v[3],
			    (v[0] & 1) ? "on " : "off", (v[0] >> 4) & 0xf, (v[0] >> 8) & 0xff,
			    (unsigned long long)(v[2] & 0xffff) << 16,
			    (unsigned long long)((v[1] & 0xffff) + 1) << 16,
			    (unsigned long long)(v[3] & 0xffff) << 16);
	}
	k_mem_unmap_phys_bare(va, 0x1000);
	return 0;
}

/* bar <n> [axi]: show / set the inbound AXI base of BAR n (0x5800 + 8*n/2) */
static int cmd_bar(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long n, axi;
	uint32_t lo, hi;

	if (parse_u(sh, argv[1], &n) != 0 || n > 5) {
		return -EINVAL;
	}
	if (argc > 2) {
		if (parse_u(sh, argv[2], &axi) != 0 || (axi & ~(uint64_t)ADVK_IB_ADDR_MASK & 0xffffffffULL) != 0) {
			shell_error(sh, "axi must be 128-byte aligned");
			return -EINVAL;
		}
		lo = advk_rd(ep, ADVK_IB_BAR_LO(n));
		advk_wr(ep, ADVK_IB_BAR_HI(n), (uint32_t)(axi >> 32));
		advk_wr(ep, ADVK_IB_BAR_LO(n), ((uint32_t)axi & ADVK_IB_ADDR_MASK) |
						 (lo & ~ADVK_IB_ADDR_MASK));
	}
	lo = advk_rd(ep, ADVK_IB_BAR_LO(n));
	hi = advk_rd(ep, ADVK_IB_BAR_HI(n));
	shell_print(sh, "BAR%llu: %08x %08x -> axi 0x%llx (hw size 0x%zx)", n, lo, hi,
		    ((unsigned long long)hi << 32) | (lo & ADVK_IB_ADDR_MASK),
		    ep->bar_hw_size[n]);
	return 0;
}

/* copy <dst> <src> <len>: word copy with probe reads (aborts become 0xdeadbeef) */
static int cmd_copy(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long dst, src, len;
	uint8_t *d, *s;
	uint32_t bad = 0;

	ARG_UNUSED(argc);
	if (parse_u(sh, argv[1], &dst) != 0 || parse_u(sh, argv[2], &src) != 0 ||
	    parse_u(sh, argv[3], &len) != 0 || ((dst | src | len) & 3) != 0 || len == 0 ||
	    len > 0x100000) {
		return -EINVAL;
	}
	k_mem_map_phys_bare(&d, dst & ~0xfffULL, ROUND_UP(len + (dst & 0xfff), 0x1000),
			    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	k_mem_map_phys_bare(&s, src & ~0xfffULL, ROUND_UP(len + (src & 0xfff), 0x1000),
			    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	for (unsigned long long o = 0; o < len; o += 4) {
		uint32_t v;

		if (a3700_probe_read32((uintptr_t)s + (src & 0xfff) + o, &v) != 0) {
			v = 0xdeadbeef;
			bad++;
		}
		*(volatile uint32_t *)(d + (dst & 0xfff) + o) = v;
	}
	barrier_dsync_fence_full();
	k_mem_unmap_phys_bare(s, ROUND_UP(len + (src & 0xfff), 0x1000));
	k_mem_unmap_phys_bare(d, ROUND_UP(len + (dst & 0xfff), 0x1000));
	shell_print(sh, "copied 0x%llx bytes 0x%llx -> 0x%llx, %u aborted words", len, src, dst,
		    bad);
	return 0;
}

/*
 * wmap <start> <end>: writable bits of every word (write the complement,
 * read back, restore). Skips 0x18000-0x18fff (core control: mode bit)
 * unless asked for explicitly. Prints only words with writable bits.
 */
static int cmd_wmap(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long start, end;
	uint32_t n = 0;

	ARG_UNUSED(argc);
	if (parse_u(sh, argv[1], &start) != 0 || parse_u(sh, argv[2], &end) != 0) {
		return -EINVAL;
	}
	end = MIN(end, (unsigned long long)ADVK_REG_SPACE);
	for (uint32_t o = start & ~3U; o < end; o += 4) {
		uint32_t v, w;

		if (o >= 0x18000 && o < 0x19000 && !(start >= 0x18000 && start < 0x19000)) {
			o = 0x19000 - 4;
			continue;
		}
		if (a3700_probe_read32(ep->reg + o, &v) != 0) {
			continue;
		}
		if (a3700_probe_write32(ep->reg + o, ~v) != 0 ||
		    a3700_probe_read32(ep->reg + o, &w) != 0) {
			shell_print(sh, "%05x: abort on write", o);
			continue;
		}
		(void)a3700_probe_write32(ep->reg + o, v);
		if ((v ^ w) != 0) {
			uint32_t now = 0;

			(void)a3700_probe_read32(ep->reg + o, &now);
			shell_print(sh, "%05x: reset %08x  writable %08x%s", o, v, v ^ w,
				    now != v ? "  (did not restore)" : "");
			n++;
		}
	}
	shell_print(sh, "%u words with writable bits", n);
	return 0;
}

/* nz <start> <end>: the non-zero words of a range (probe reads, holes skipped) */
static int cmd_nz(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long start, end;
	uint32_t n = 0;

	if (parse_u(sh, argv[1], &start) != 0 || parse_u(sh, argv[2], &end) != 0) {
		return -EINVAL;
	}
	end = MIN(end, (unsigned long long)ADVK_REG_SPACE);
	for (uint32_t o = start & ~3U; o < end; o += 4) {
		uint32_t v;

		if (a3700_probe_read32(ep->reg + o, &v) == 0 && v != 0) {
			shell_print(sh, "%05x: %08x", o, v);
			n++;
		}
	}
	shell_print(sh, "%u non-zero words", n);
	return 0;
}

/* Wait up to ms for the LTSSM to be (or not be) in L0..L2 */
static bool wait_link(struct advk_ep *ep, bool up, uint32_t ms)
{
	int64_t end = k_uptime_get() + ms;

	do {
		if (advk_ltssm_is_up(advk_ltssm(ep)) == up) {
			return true;
		}
		k_msleep(1);
	} while (k_uptime_get() < end);
	return false;
}

/*
 * down: link training off, the link goes to detect. On the host remove
 * the device first (echo 1 > .../remove), it would see a surprise down.
 */
static int cmd_down(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	bool ok;

	struct a3700_ep_data *data = EP_DEV->data;
	mem_addr_t pwr = data->comphy + ADVK_COMPHY_PWR_PLL_CTRL;

	/*
	 * Clearing the training enable does not leave L0 once the link is
	 * up: power the PHY PLL / Rx / Tx down (TF-A has no PCIe power off),
	 * the host sees electrical idle and drops the link. up powers it on
	 * again through the SiP call.
	 */
	pcie_epc_stop(EP_DEV);
	sys_write16(sys_read16(pwr) & ~ADVK_COMPHY_PU_PLL_RX_TX, pwr);
	ok = wait_link(ep, false, 200);
	shell_print(sh, "training off, PHY down (pwr %04x), ltssm 0x%02x %s%s", sys_read16(pwr),
		    advk_ltssm(ep), advk_ltssm_name(advk_ltssm(ep)), ok ? "" : " (link still up)");
	return 0;
}

/*
 * up: controller set up again as at boot (EP mode, header, MSI
 * capability; outbound windows cleared, IO decode window kept), then
 * link training. Then rescan on the host.
 */
static int cmd_up(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	bool ok;

	struct a3700_ep_data *data = EP_DEV->data;

	pcie_epc_stop(EP_DEV);
	(void)wait_link(ep, false, 200);
	if ((sys_read16(data->comphy + ADVK_COMPHY_PWR_PLL_CTRL) & ADVK_COMPHY_PU_PLL_RX_TX) !=
	    ADVK_COMPHY_PU_PLL_RX_TX) {
		data->comphy_ret = comphy_power_on(EP_DEV);
		comphy_clkout_off(EP_DEV);
		shell_print(sh, "PHY on: %ld", data->comphy_ret);
	}
	advk_ep_restore(EP_DEV);
	pcie_epc_start(EP_DEV);
	ok = wait_link(ep, true, 1000);
	shell_print(sh, "%s, ltssm 0x%02x %s, lnksta %04x", ok ? "link up" : "no link within 1 s",
		    advk_ltssm(ep), advk_ltssm_name(advk_ltssm(ep)),
		    advk_rd(ep, ADVK_CFG_LNKCTL_STA) >> 16);
	return 0;
}

static int cmd_comphy(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = EP_DEV;
	struct a3700_ep_data *data = dev->data;

	if (argc < 2 || strcmp(argv[1], "on") == 0) {
		data->comphy_ret = comphy_power_on(dev);
		comphy_clkout_off(dev);
		shell_print(sh, "COMPHY power on: %ld", data->comphy_ret);
	} else if (strcmp(argv[1], "clkout") == 0) {
		sys_write16(sys_read16(data->comphy + ADVK_COMPHY_MISC_CTRL0) |
				    ADVK_COMPHY_CLK100M_125M_EN,
			    data->comphy + ADVK_COMPHY_MISC_CTRL0);
	} else if (strcmp(argv[1], "noclkout") == 0) {
		comphy_clkout_off(dev);
	} else if (strcmp(argv[1], "show") != 0) {
		shell_error(sh, "on | clkout | noclkout | show");
		return -EINVAL;
	}
	shell_print(sh, "misc_ctrl0 %04x (100/125M out %s), refclk reg %08x",
		    sys_read16(data->comphy + ADVK_COMPHY_MISC_CTRL0),
		    (sys_read16(data->comphy + ADVK_COMPHY_MISC_CTRL0) &
		     ADVK_COMPHY_CLK100M_125M_EN) ? "on" : "off",
		    advk_rd(advk_ep(dev), ADVK_REF_CLK));
	return 0;
}

static int cmd_refclk(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	uint32_t v = advk_rd(ep, ADVK_REF_CLK) & ~(ADVK_REF_CLK_TX | ADVK_REF_CLK_RX);

	if (strcmp(argv[1], "in") == 0) {
		v |= ADVK_REF_CLK_RX;
	} else if (strcmp(argv[1], "out") == 0) {
		v |= ADVK_REF_CLK_TX;
	} else if (strcmp(argv[1], "off") != 0) {
		shell_error(sh, "in | out | off");
		return -EINVAL;
	}
	advk_wr(ep, ADVK_REF_CLK, v);
	shell_print(sh, "refclk %08x", advk_rd(ep, ADVK_REF_CLK));
	return 0;
}

static int cmd_intx(const struct shell *sh, size_t argc, char **argv)
{
	return pcie_epc_raise_irq(EP_DEV, 0, PCIE_EPC_IRQ_INTX, 0);
}

static int cmd_msi(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long n = 1;
	int ret;

	if (argc > 1 && parse_u(sh, argv[1], &n) != 0) {
		return -EINVAL;
	}
	ret = pcie_epc_raise_irq(EP_DEV, 0, PCIE_EPC_IRQ_MSI, (uint16_t)n);
	if (ret != 0) {
		shell_error(sh, "MSI %llu: %d", n, ret);
	}
	return ret;
}

/* Read (or write one word of) host memory through an outbound window */
static int cmd_host(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = EP_DEV;
	struct pcie_epc_host_map map;
	unsigned long long addr, arg = 64;
	bool write = argc > 2 && argv[2][0] == '=';
	int ret;

	if (parse_u(sh, argv[1], &addr) != 0 ||
	    (argc > 2 && parse_u(sh, argv[2] + (write ? 1 : 0), &arg) != 0)) {
		return -EINVAL;
	}

	ret = pcie_epc_map_host(dev, 0, addr, write ? 4 : MIN(arg, 4096ULL), &map);
	if (ret != 0) {
		shell_error(sh, "map 0x%llx: %d", addr, ret);
		return ret;
	}
	struct advk_ep *ep = advk_ep(dev);
	uint32_t isr0 = advk_rd(ep, ADVK_ISR0), isr1 = advk_rd(ep, ADVK_ISR1);

	shell_print(sh, "host 0x%llx via cpu %p, axi 0x%llx", addr, map.ptr,
		    (unsigned long long)advk_ep_cpu_to_axi(dev, (uintptr_t)map.ptr));
	/* a completion error from the host aborts the access: probe it */
	if (write) {
		uint32_t v = 0;

		ret = a3700_probe_write32((uintptr_t)map.ptr, (uint32_t)arg);
		if (ret == 0) {
			ret = a3700_probe_read32((uintptr_t)map.ptr, &v);
		}
		shell_print(sh, "0x%llx <- %08x: %s, reads %08x", addr, (uint32_t)arg,
			    ret == 0 ? "ok" : "bus error", v);
	} else {
		for (uint32_t o = 0; o < MIN(arg, 4096ULL); o += 16) {
			uint32_t w[4] = {0};
			int r = 0;

			for (int i = 0; i < 4 && r == 0; i++) {
				r = a3700_probe_read32((uintptr_t)map.ptr + o + 4 * i, &w[i]);
			}
			if (r != 0) {
				shell_print(sh, "%08llx: bus error", addr + o);
				ret = r;
				break;
			}
			shell_print(sh, "%08llx: %08x %08x %08x %08x", addr + o, w[0], w[1], w[2],
				    w[3]);
		}
	}
	shell_print(sh, "isr0 %08x -> %08x, isr1 %08x -> %08x", isr0, advk_rd(ep, ADVK_ISR0), isr1,
		    advk_rd(ep, ADVK_ISR1));
	pcie_epc_unmap_host(dev, &map);
	return ret;
}

/*
 * Inbound translation hunt. The BARs come up mapping AXI address 0 (BAR
 * offset = DDR address), the register that moves them is unknown. Phase 1
 * lists the registers in [start, end) that read 0 and keep a written
 * 0x20000000; phase 2 writes 0x20000000 + i MiB into candidate i for a
 * while, with DDR at that address holding 0xc0de0000 | i and the register
 * offset. A host polling BAR0+0 / BAR4+0 then reads the culprit directly.
 */
#include <zephyr/kernel/internal/mm.h>

#define IB_BASE     0x20000000UL
#define IB_STEP     0x100000UL
#define IB_MAX      64
#define IB_MAGIC    0xc0de0000U

static bool ib_skip(uint32_t o)
{
	return o < 0x1000 ||                            /* config space, BARs */
	       (o >= ADVK_CTRL0 && o < ADVK_CTRL0 + 0x100) || /* control, IRQ */
	       (o >= 0x4c00 && o < 0x4d00) ||           /* outbound windows */
	       (o >= ADVK_LMI_CFG && o < ADVK_LMI_CFG + 0x48) ||
	       o >= ADVK_CTRL_CONFIG;                   /* core control, IRQ */
}

static int cmd_ibscan(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long start, end, hold = 1000;
	uint32_t cand[IB_MAX];
	int n = 0;
	uint8_t *ddr;
	uint32_t err_start = 0, faults = 0;
	bool in_err = false;

	if (parse_u(sh, argv[1], &start) != 0 || parse_u(sh, argv[2], &end) != 0 ||
	    (argc > 3 && parse_u(sh, argv[3], &hold) != 0) || end > ADVK_REG_SPACE ||
	    start >= end) {
		return -EINVAL;
	}

	for (uint32_t o = start & ~3U; o < end && n < IB_MAX; o += 4) {
		uint32_t rb, v;

		if (ib_skip(o)) {
			continue;
		}
		/* holes in the register space abort: probe before touching */
		if (a3700_probe_read32(ep->reg + o, &v) < 0) {
			if (!in_err) {
				err_start = o;
				in_err = true;
			}
			faults++;
			continue;
		}
		if (in_err) {
			shell_print(sh, "  %05x..%05x: bus error", err_start, o - 4);
			in_err = false;
		}
		if (v != 0) {
			continue;
		}
		if (a3700_probe_write32(ep->reg + o, IB_BASE) < 0) {
			shell_print(sh, "  %05x: write aborts", o);
			faults++;
			continue;
		}
		rb = advk_rd(ep, o);
		(void)a3700_probe_write32(ep->reg + o, 0);
		if (rb != 0) {
			shell_print(sh, "candidate %2d: %05x keeps %08x", n, o, rb);
			cand[n++] = o;
		}
	}
	if (in_err) {
		shell_print(sh, "  %05x..%05x: bus error", err_start, (uint32_t)end - 4);
	}
	shell_print(sh, "%d candidates, %u aborted reads", n, faults);
	if (n == 0) {
		return 0;
	}

	for (int i = 0; i < n; i++) {
		volatile uint32_t *p;

		k_mem_map_phys_bare(&ddr, IB_BASE + i * IB_STEP, 0x1000,
				    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
		p = (volatile uint32_t *)ddr;
		p[0] = IB_MAGIC | i;
		p[1] = cand[i];
		barrier_dsync_fence_full();
		k_mem_unmap_phys_bare(ddr, 0x1000);
	}

	for (int i = 0; i < n; i++) {
		shell_print(sh, "[%lld ms] %2d: %05x = %08lx", k_uptime_get(), i, cand[i],
			    IB_BASE + i * IB_STEP);
		advk_wr(ep, cand[i], IB_BASE + i * IB_STEP);
		k_msleep(hold);
		advk_wr(ep, cand[i], 0);
	}
	shell_print(sh, "done");
	return 0;
}

/*
 * Guess: 0x4d00 holds inbound windows laid out like the outbound ones at
 * 0x4c00 (8 x 0x20: match | enable, remap, mask, actions, 64-bit pairs),
 * with PCI address as match and AXI address as remap. Puts a tagged
 * pattern at the AXI address, so a host reading the BAR sees the hit.
 */
#define IB_WIN(w, off) (0x4d00 + 0x20 * (w) + (off))

static int cmd_ibwin(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long long w, pci, axi, size, act = 0;
	uint64_t mask;
	uint32_t regs[7];
	uint8_t *ddr;
	volatile uint32_t *p;
	int ret = 0;

	if (parse_u(sh, argv[1], &w) != 0 || w > 7) {
		return -EINVAL;
	}
	if (argc == 2) {
		for (uint32_t o = 0; o < 0x1c; o += 4) {
			(void)a3700_probe_write32(ep->reg + IB_WIN(w, o), 0);
		}
		shell_print(sh, "window %llu cleared", w);
		return 0;
	}
	if (argc < 5 || parse_u(sh, argv[2], &pci) != 0 || parse_u(sh, argv[3], &axi) != 0 ||
	    parse_u(sh, argv[4], &size) != 0 || (argc > 5 && parse_u(sh, argv[5], &act) != 0) ||
	    size == 0 || (size & (size - 1)) != 0) {
		return -EINVAL;
	}

	k_mem_map_phys_bare(&ddr, axi & ~0xfffULL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	p = (volatile uint32_t *)(ddr + (axi & 0xfff));
	p[0] = IB_MAGIC | 0xff00 | (uint32_t)w;
	p[1] = IB_WIN(w, 0);
	barrier_dsync_fence_full();
	k_mem_unmap_phys_bare(ddr, 0x1000);

	mask = ~(size - 1);
	regs[0] = (uint32_t)pci | 1;
	regs[1] = (uint32_t)(pci >> 32);
	regs[2] = (uint32_t)axi;
	regs[3] = (uint32_t)(axi >> 32);
	regs[4] = (uint32_t)mask;
	regs[5] = (uint32_t)(mask >> 32);
	regs[6] = (uint32_t)act;
	/* enable (match) last */
	for (int i = 6; i >= 0 && ret == 0; i--) {
		ret = a3700_probe_write32(ep->reg + IB_WIN(w, 4 * i), regs[i]);
	}
	for (int i = 0; i < 7; i++) {
		shell_print(sh, "  %05x: %08x", (uint32_t)IB_WIN(w, 4 * i),
			    advk_rd(ep, (uint32_t)IB_WIN(w, 4 * i)));
	}
	shell_print(sh, "window %llu: pci 0x%llx -> axi 0x%llx, 0x%llx bytes%s, tag %08x at axi",
		    w, pci, axi, size, ret < 0 ? " (write aborted)" : "",
		    IB_MAGIC | 0xff00 | (uint32_t)w);
	return ret;
}

/*
 * The PCIe unit's IO address decode windows (TF-A io_addr_dec.c,
 * 0xd000c500, 2 windows x 0x10: ctrl = (size/64K - 1) << 16 | attr << 8 |
 * target << 4 | enable, base, remap). TF-A sets window 0 to DRAM 0..1 GiB,
 * which is why the BARs land on DDR 0: inbound TLPs go through here.
 */
#define IODEC_PCIE     0xd000c500UL
#define IODEC_ATTR     0x3d

static int cmd_iodec(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long w, base, size, remap, target = 0, attr = IODEC_ATTR;
	uint8_t *va;
	volatile uint32_t *r;

	if ((argc > 5 && parse_u(sh, argv[5], &target) != 0) ||
	    (argc > 6 && parse_u(sh, argv[6], &attr) != 0) || target > 0xf || attr > 0xff) {
		return -EINVAL;
	}
	k_mem_map_phys_bare(&va, IODEC_PCIE & ~0xfffUL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	r = (volatile uint32_t *)(va + (IODEC_PCIE & 0xfff));

	if (argc >= 5) {
		if (parse_u(sh, argv[1], &w) != 0 || w > 1 || parse_u(sh, argv[2], &base) != 0 ||
		    parse_u(sh, argv[3], &size) != 0 || parse_u(sh, argv[4], &remap) != 0 ||
		    size < 0x10000 || (size & 0xffff) != 0 || (base & 0xffff) != 0 ||
		    (remap & 0xffff) != 0) {
			k_mem_unmap_phys_bare(va, 0x1000);
			return -EINVAL;
		}
		uint32_t ctrl = (uint32_t)(((size >> 16) - 1) << 16) | ((uint32_t)attr << 8) |
				((uint32_t)target << 4);

		if (target == 0) {
			uint8_t *t;

			/* tag the remapped DRAM target */
			k_mem_map_phys_bare(&t, remap, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
			((volatile uint32_t *)t)[0] = IB_MAGIC | 0xee00 | (uint32_t)w;
			((volatile uint32_t *)t)[1] = (uint32_t)remap;
			barrier_dsync_fence_full();
			k_mem_unmap_phys_bare(t, 0x1000);
		}

		r[w * 4 + 0] = ctrl;
		r[w * 4 + 1] = (uint32_t)base;
		r[w * 4 + 2] = (uint32_t)remap;
		r[w * 4 + 0] = ctrl | 1;
		barrier_dsync_fence_full();
	} else if (argc == 2 && strcmp(argv[1], "reset") == 0) {
		/* TF-A's setting: window 0 = DRAM 0..1 GiB, window 1 off */
		r[4] = 0;
		r[0] = (uint32_t)(((0x40000000ULL >> 16) - 1) << 16) | (IODEC_ATTR << 8);
		r[1] = 0;
		r[2] = 0;
		r[0] |= 1;
	}

	for (int i = 0; i < 2; i++) {
		uint32_t c = r[i * 4 + 0];

		shell_print(sh, "win %d: ctrl %08x base %08x remap %08x  (%s, size 0x%x, attr 0x%02x,"
			    " target %u)", i, c, r[i * 4 + 1], r[i * 4 + 2], (c & 1) ? "on" : "off",
			    ((c >> 16) + 1) << 16, (c >> 8) & 0xff, (c >> 4) & 0xf);
	}
	k_mem_unmap_phys_bare(va, 0x1000);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_advk_ep, SHELL_CMD(status, NULL, "Link, header, BARs, windows, events", cmd_status),
	SHELL_CMD(hist, NULL, "LTSSM history of the link poll", cmd_hist),
	SHELL_CMD_ARG(trace, NULL, "[ms] Busy-poll LTSSM transitions (default 100)", cmd_trace,
		      1, 1),
	SHELL_CMD_ARG(dump, NULL, "[cfg|ctrl|ob|lmi|core|irq|<off> [len]] Register dump",
		      cmd_dump, 1, 2),
	SHELL_CMD_ARG(rd, NULL, "<off> Read a controller register", cmd_rd, 2, 0),
	SHELL_CMD_ARG(wr, NULL, "<off> <val> Write a controller register", cmd_wr, 3, 0),
	SHELL_CMD_ARG(probe, NULL, "<off> [len] Writable bits (writes, then restores!)",
		      cmd_probe, 2, 1),
	SHELL_CMD_ARG(cpuwin, NULL, "CPU address decode windows (0xd000cf00)", cmd_cpuwin, 1, 0),
	SHELL_CMD_ARG(bar, NULL, "<n> [axi] Inbound AXI base of BAR n", cmd_bar, 2, 1),
	SHELL_CMD_ARG(copy, NULL, "<dst> <src> <len> Word copy, aborts read as deadbeef",
		      cmd_copy, 4, 0),
	SHELL_CMD_ARG(wmap, NULL, "<start> <end> Writable bits of every word (writes, restores!)",
		      cmd_wmap, 3, 0),
	SHELL_CMD_ARG(nz, NULL, "<start> <end> Non-zero words of a range", cmd_nz, 3, 0),
	SHELL_CMD_ARG(map, NULL, "[start end] Readable ranges of the register space", cmd_map, 1,
		      2),
	SHELL_CMD_ARG(xor, NULL, "<dst> <src> <len> XOR engine memcpy, h:<pci addr> = host memory",
		      cmd_xor, 4, 0),
	SHELL_CMD(up, NULL, "Set up the controller again and train the link", cmd_up),
	SHELL_CMD(down, NULL, "Link training off (remove the device on the host first)",
		  cmd_down),
	SHELL_CMD(start, NULL, "Enable link training", cmd_start),
	SHELL_CMD(stop, NULL, "Disable link training", cmd_stop),
	SHELL_CMD(setup, NULL, "Reprogram the controller (EP mode, refclk in)", cmd_setup),
	SHELL_CMD_ARG(comphy, NULL, "[on|clkout|noclkout|show] COMPHY lane / clock output",
		      cmd_comphy, 1, 1),
	SHELL_CMD_ARG(refclk, NULL, "<in|out|off> REF_CLK pad direction (0x4814)", cmd_refclk,
		      2, 0),
	SHELL_CMD(intx, NULL, "Pulse INTx", cmd_intx),
	SHELL_CMD_ARG(msi, NULL, "[n] Send MSI n (1-based)", cmd_msi, 1, 1),
	SHELL_CMD_ARG(host, NULL, "<pci addr> [len|=val] Read / write host memory", cmd_host, 2,
		      1),
	SHELL_CMD_ARG(iodec, NULL,
		      "[<w> <base> <size> <remap> [target [attr]] | reset] PCIe IO decode windows"
		      " (0xd000c500)",
		      cmd_iodec, 1, 6),
	SHELL_CMD_ARG(ibwin, NULL,
		      "<w> [<pci> <axi> <size> [actions]] Guessed inbound window at 0x4d00",
		      cmd_ibwin, 2, 4),
	SHELL_CMD_ARG(ibscan, NULL,
		      "<start> <end> [hold ms] Hunt the inbound translation (writes, restores!)",
		      cmd_ibscan, 3, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(advk_ep, &sub_advk_ep, "Armada 3700 PCIe endpoint", NULL);

#endif /* CONFIG_PCIE_EPC_AARDVARK_SHELL */
