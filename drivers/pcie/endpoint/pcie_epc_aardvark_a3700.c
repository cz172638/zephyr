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

/* TF-A enables the PHY clock output to the REFCLK pins; an EP must not drive it */
/*
 * Inbound: a TLP hitting a BAR goes to AXI address (BAR offset) and on
 * through the PCIe unit's IO decode window 0 (0xd000c500, set up by TF-A
 * for DRAM 0..1 GiB, no remap). Shrinking that window to the BAR backing
 * area at DDR 0 keeps the host out of the rest of memory.
 */
#define IODEC_PCIE_WIN0 0xd000c500UL

static void inbound_limit(uint32_t size)
{
	uint8_t *va;
	volatile uint32_t *r;
	uint32_t ctrl;

	if (size == 0) {
		return;
	}
	k_mem_map_phys_bare(&va, IODEC_PCIE_WIN0 & ~0xfffUL, 0x1000,
			    K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	r = (volatile uint32_t *)(va + (IODEC_PCIE_WIN0 & 0xfff));
	ctrl = (r[0] & 0xffff) & ~1U;
	r[0] = ctrl;
	r[1] = 0;
	r[0] = (((size >> 16) - 1) << 16) | ctrl;
	r[0] |= 1;
	LOG_INF("inbound limited to DDR 0x0-0x%x (IO decode ctrl %08x)", size - 1, r[0]);
	k_mem_unmap_phys_bare(va, 0x1000);
}

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

	inbound_limit(CONFIG_PCIE_EPC_AARDVARK_INBOUND_LIMIT);

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

	pcie_epc_stop(EP_DEV);
	ok = wait_link(ep, false, 200);
	shell_print(sh, "training off, ltssm 0x%02x %s%s", advk_ltssm(ep),
		    advk_ltssm_name(advk_ltssm(ep)), ok ? "" : " (link still up)");
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

	pcie_epc_stop(EP_DEV);
	(void)wait_link(ep, false, 200);
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
	unsigned long long w, base, size, remap;
	uint8_t *va;
	volatile uint32_t *r;

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
		uint32_t ctrl = (uint32_t)(((size >> 16) - 1) << 16) | (IODEC_ATTR << 8);
		uint8_t *t;

		/* tag the remapped target */
		k_mem_map_phys_bare(&t, remap, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
		((volatile uint32_t *)t)[0] = IB_MAGIC | 0xee00 | (uint32_t)w;
		((volatile uint32_t *)t)[1] = (uint32_t)remap;
		barrier_dsync_fence_full();
		k_mem_unmap_phys_bare(t, 0x1000);

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
		      "[<w> <base> <size> <remap> | reset] PCIe IO decode windows (0xd000c500)",
		      cmd_iodec, 1, 4),
	SHELL_CMD_ARG(ibwin, NULL,
		      "<w> [<pci> <axi> <size> [actions]] Guessed inbound window at 0x4d00",
		      cmd_ibwin, 2, 4),
	SHELL_CMD_ARG(ibscan, NULL,
		      "<start> <end> [hold ms] Hunt the inbound translation (writes, restores!)",
		      cmd_ibscan, 3, 1),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(advk_ep, &sub_advk_ep, "Armada 3700 PCIe endpoint", NULL);

#endif /* CONFIG_PCIE_EPC_AARDVARK_SHELL */
