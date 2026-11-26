/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3308 as a lab probe: USB composite device with CDC ACM ports, the first
 * one the probe's shell, the others bridged to UARTs (zephyr,uart-bridge);
 * the line coding the host sets on a bridged port is applied to its UART.
 */

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart/uart_bridge.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(usb_probe, LOG_LEVEL_INF);

static const struct device *const bridges[] = {
	DT_FOREACH_STATUS_OKAY(zephyr_uart_bridge, DEVICE_DT_GET_COMMA)
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
	return 0;
}
