/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * The A523 root complex comes up and enumerates during boot; the "pcie"
 * shell commands show the link and the devices behind the M.2 slot.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

int main(void)
{
	const struct device *pcie = DEVICE_DT_GET(DT_NODELABEL(pcie));

	printk("pcie: root complex %s %s\n", pcie->name,
	       device_is_ready(pcie) ? "ready" : "failed");
	return 0;
}
