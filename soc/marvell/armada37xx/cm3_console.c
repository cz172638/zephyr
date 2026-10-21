/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART0 belongs to the CM3; the A53 console (zephyr,uart-ipc there)
 * arrives on the "console" endpoint and goes out here, the UART input
 * goes back to the A53. The CM3's own printk shares the UART.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ipc/ipc_service.h>

static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *const ipc = DEVICE_DT_GET(DT_NODELABEL(ipc0));

static struct ipc_ept ept;
static volatile bool bound;
static uint32_t rx_bytes, tx_bytes;

static void bridge_bound(void *priv)
{
	ARG_UNUSED(priv);
	bound = true;
}

static void bridge_received(const void *data, size_t len, void *priv)
{
	const uint8_t *p = data;

	ARG_UNUSED(priv);
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart, p[i]);
	}
	rx_bytes += len;
}

static struct ipc_ept_cfg ept_cfg = {
	.name = "console",
	.cb = {
		.bound = bridge_bound,
		.received = bridge_received,
	},
};

static void bridge_thread(void *a, void *b, void *c)
{
	uint8_t buf[32];
	unsigned char ch;
	size_t n;
	int ret;

	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	ret = ipc_service_open_instance(ipc);
	if (ret < 0 && ret != -EALREADY) {
		printk("cm3 console: ipc open %d\n", ret);
		return;
	}
	ret = ipc_service_register_endpoint(ipc, &ept, &ept_cfg);
	if (ret < 0) {
		printk("cm3 console: endpoint %d\n", ret);
		return;
	}

	for (;;) {
		/* the UART has no TX interrupt and its RX line is polled too */
		n = 0;
		while (n < sizeof(buf) && uart_poll_in(uart, &ch) == 0) {
			buf[n++] = ch;
		}
		if (n != 0 && bound) {
			while (ipc_service_send(&ept, buf, n) == -ENOMEM) {
				k_msleep(1);
			}
			tx_bytes += n;
		}
		k_msleep(CONFIG_SOC_ARMADA3720_CM3_CONSOLE_POLL_MS);
	}
}

K_THREAD_DEFINE(cm3_console, 1024, bridge_thread, NULL, NULL, NULL, 5, 0, 0);
