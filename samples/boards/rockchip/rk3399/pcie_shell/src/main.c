/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe in either role, driven from the shell ("pcie ...").
 */

#include <zephyr/drivers/pcie/role.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	enum pcie_role saved;
	bool have = pcie_role_saved(&saved);

	printk("RK3399 PCIe %s (saved: %s, in %s); \"pcie controller mode rc|ep\", "
	       "then a warm reset (\"kernel reboot warm\")\n",
	       pcie_role_get() == PCIE_ROLE_RC ? "root complex" : "endpoint",
	       have ? (saved == PCIE_ROLE_RC ? "rc" : "ep") : "-", pcie_role_storage());
	return 0;
}
