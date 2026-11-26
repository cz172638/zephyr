/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3308 as a lab probe: USB composite device with CDC ACM ports, the first
 * one the probe's shell, the others bridged to UARTs (zephyr,uart-bridge);
 * the line coding the host sets on a bridged port is applied to its UART.
 * The FAT partition of a micro SD card is mounted at /SD:; "fwupdate"
 * writes the card's boot area. With net.conf: Ethernet with DHCP, mDNS
 * (rk3308-probe-<mac>.local), the shell over SSH and CMSIS-DAP over TCP
 * (SWD/JTAG on header 2).
 */

#include <ff.h>
#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart/uart_bridge.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usbd.h>

#include "probe.h"

LOG_MODULE_REGISTER(usb_probe, LOG_LEVEL_INF);

static const struct device *const bridges[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_uart_bridge, DEVICE_DT_GET_COMMA)
};

static FATFS sd_fat;
static struct fs_mount_t sd_mnt = {
	.type = FS_FATFS,
	.fs_data = &sd_fat,
	.mnt_point = "/SD:",
};

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING ||
	    msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		/* bridges not holding msg->dev skip it */
		for (size_t i = 0; i < ARRAY_SIZE(bridges); i++) {
			uart_bridge_settings_update(msg->dev, bridges[i]);
		}
		return;
	}
	LOG_INF("USB: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY && usbd_enable(ctx) != 0) {
			LOG_ERR("USB enable failed");
		}
		if (msg->type == USBD_MSG_VBUS_REMOVED && usbd_disable(ctx) != 0) {
			LOG_ERR("USB disable failed");
		}
	}
}

int main(void)
{
	int ret;
	struct usbd_context *usbd = sample_usbd_init_device(msg_cb);

	if (usbd == NULL) {
		LOG_ERR("USB device init failed");
		return -ENODEV;
	}
	if (!usbd_can_detect_vbus(usbd) && usbd_enable(usbd) != 0) {
		LOG_ERR("USB enable failed");
		return -EIO;
	}
	LOG_INF("USB probe: %zu UART bridges", ARRAY_SIZE(bridges));

	ret = fs_mount(&sd_mnt);
	printk("SD: %s (%d)\n", ret == 0 ? "mounted at /SD:" : "no card or no FAT", ret);

	if (IS_ENABLED(CONFIG_NET_DHCPV4) && net_if_get_default() != NULL) {
		struct net_linkaddr *ll = net_if_get_link_addr(net_if_get_default());

		printk("net: MAC %02x:%02x:%02x:%02x:%02x:%02x, DHCP\n", ll->addr[0],
		       ll->addr[1], ll->addr[2], ll->addr[3], ll->addr[4], ll->addr[5]);
		net_dhcpv4_start(net_if_get_default());
	}
	if (IS_ENABLED(CONFIG_SSH_SERVER)) {
		probe_ssh_start();
	}
	if (IS_ENABLED(CONFIG_DAP_BACKEND_TCP)) {
		probe_dap_start();
	}
	return 0;
}
