/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * CMSIS-DAP (SWD and JTAG on header 2) over TCP, for OpenOCD:
 *   adapter driver cmsis-dap
 *   cmsis-dap backend tcp
 *   cmsis-dap tcp host <probe>
 *
 * "dap loopback [kHz]" checks the JTAG path with TDI wired to TDO (header
 * pins 40 and 42): sequences of every length from 1 to 200 bits, then
 * 64 KiB through SPI for the throughput. Not while a client is connected.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/dap/dap_link.h>
#include <zephyr/device.h>
#include <zephyr/drivers/swdp.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include "probe.h"

DAP_LINK_CONTEXT_DEFINE(dap_ctx, DEVICE_DT_GET(DT_NODELABEL(dap)));

void probe_dap_start(void)
{
	int ret = dap_link_init(&dap_ctx);

	if (ret == 0) {
		ret = dap_link_backend_tcp_init(&dap_ctx);
	}
	if (ret != 0) {
		printk("dap: %d\n", ret);
		return;
	}
	printk("dap: CMSIS-DAP on TCP port %d\n", CONFIG_DAP_BACKEND_TCP_PORT);
}

static uint8_t lb_tdi[1024];
static uint8_t lb_tdo[1024];

static int lb_check(const struct shell *sh, const struct device *dev, uint32_t bits)
{
	size_t n = (bits + 7U) / 8U;
	int ret;

	sys_rand_get(lb_tdi, n);
	memset(lb_tdo, 0x5a, n);
	ret = swdp_jtag_sequence(dev, bits, false, lb_tdi, lb_tdo);
	if (ret != 0) {
		shell_error(sh, "%u bits: %d", bits, ret);
		return ret;
	}
	if (bits % 8U != 0U) {
		uint8_t mask = BIT(bits % 8U) - 1U;

		lb_tdi[n - 1U] &= mask;
		lb_tdo[n - 1U] &= mask;
	}
	if (memcmp(lb_tdi, lb_tdo, n) != 0) {
		for (size_t i = 0; i < n; i++) {
			if (lb_tdi[i] != lb_tdo[i]) {
				shell_error(sh, "%u bits: byte %zu sent %02x, read %02x", bits, i,
					    lb_tdi[i], lb_tdo[i]);
				break;
			}
		}
		return -EIO;
	}
	return 0;
}

static int cmd_loopback(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = dap_ctx.dev;
	uint32_t khz = argc > 1 ? strtoul(argv[1], NULL, 0) : 10000U;
	int64_t t0, ms;
	int ret;

	ret = swdp_port_on(dev);
	if (ret == 0) {
		ret = swdp_set_clock(dev, khz * 1000U);
	}
	if (ret != 0) {
		shell_error(sh, "port: %d", ret);
		return ret;
	}
	for (uint32_t bits = 1U; bits <= 200U && ret == 0; bits++) {
		ret = lb_check(sh, dev, bits);
	}
	if (ret == 0) {
		shell_print(sh, "1..200 bits at %u kHz: OK", khz);
		t0 = k_uptime_get();
		for (int i = 0; i < 64 && ret == 0; i++) {
			ret = lb_check(sh, dev, sizeof(lb_tdi) * 8U);
		}
		ms = k_uptime_get() - t0;
		if (ret == 0) {
			shell_print(sh, "64 KiB in %lld ms (%lld kbit/s)", ms,
				    ms > 0 ? (64LL * 8192LL) / ms : 0LL);
		}
	}
	(void)swdp_port_off(dev);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dap,
	SHELL_CMD_ARG(loopback, NULL, "[kHz]: JTAG check, TDI (pin 40) wired to TDO (pin 42)",
		      cmd_loopback, 1, 1),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(dap, &sub_dap, "CMSIS-DAP probe", NULL);
