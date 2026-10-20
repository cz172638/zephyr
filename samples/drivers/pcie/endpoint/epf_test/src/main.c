/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Everything happens in the EPC/EPF drivers; main() only reports link state
 * and offers a couple of shell commands for bring-up.
 */

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

#define EPC_NODE DT_PARENT(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_pcie_epf_test))

static const struct device *const epc = DEVICE_DT_GET(EPC_NODE);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t v;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "%s: link %s", epc->name, pcie_epc_link_up(epc) ? "up" : "down");
	for (uint32_t off = 0; off < 0x40; off += 4) {
		if (pcie_epc_cfg_read(epc, 0, off, &v) == 0) {
			shell_print(sh, "  cfg[0x%02x] = 0x%08x", off, v);
		}
	}
	return 0;
}

static int cmd_msi(const struct shell *sh, size_t argc, char **argv)
{
	int n = argc > 1 ? atoi(argv[1]) : 1;
	int ret = pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_MSI, n);

	shell_print(sh, "raise MSI %d: %d (host enabled %d vectors)", n, ret,
		    pcie_epc_get_msi(epc, 0));
	return 0;
}

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "start: %d", pcie_epc_start(epc));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ep,
	SHELL_CMD(status, NULL, "Link state and config header", cmd_status),
	SHELL_CMD_ARG(msi, NULL, "Raise MSI <n>", cmd_msi, 1, 1),
	SHELL_CMD(start, NULL, "Start link training", cmd_start),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(pcie_ep, &sub_ep, "PCIe endpoint", NULL);

int main(void)
{
	bool last = false;

	if (!device_is_ready(epc)) {
		printk("%s not ready\n", epc->name);
		return 0;
	}

	for (;;) {
		bool up = pcie_epc_link_up(epc);

		if (up != last) {
			printk("%s: link %s\n", epc->name, up ? "up" : "down");
			last = up;
		}
		k_sleep(K_MSEC(500));
	}
	return 0;
}
