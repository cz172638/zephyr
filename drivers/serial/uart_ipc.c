/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART over an IPC service endpoint. The device is usable from
 * PRE_KERNEL_1 (console): output collects in the TX ring and leaves once
 * the endpoint binds. The interrupt API is emulated from the system work
 * queue.
 */

#define DT_DRV_COMPAT zephyr_uart_ipc

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/sys/ring_buffer.h>

struct uart_ipc_config {
	const char *ipc;
	const char *name;
};

struct uart_ipc_data {
	const struct device *dev;
	struct ipc_ept ept;
	struct ipc_ept_cfg ept_cfg;
	bool bound;

	struct k_spinlock lock;
	struct ring_buf tx;
	struct ring_buf rx;
	uint8_t tx_buf[CONFIG_UART_IPC_TX_BUF_SIZE];
	uint8_t rx_buf[CONFIG_UART_IPC_RX_BUF_SIZE];
	struct k_work_delayable flush;
	uint32_t tx_drops;
	uint32_t rx_drops;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	bool tx_en;
	bool rx_en;
	struct k_work irq;
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static bool uart_ipc_pending(struct uart_ipc_data *data)
{
	return (data->tx_en && ring_buf_space_get(&data->tx) > 0) ||
	       (data->rx_en && !ring_buf_is_empty(&data->rx));
}

static void uart_ipc_kick(struct uart_ipc_data *data)
{
	if (data->cb != NULL && uart_ipc_pending(data)) {
		k_work_submit(&data->irq);
	}
}

static void uart_ipc_irq_work(struct k_work *work)
{
	struct uart_ipc_data *data = CONTAINER_OF(work, struct uart_ipc_data, irq);

	for (int i = 0; i < 4 && data->cb != NULL && uart_ipc_pending(data); i++) {
		data->cb(data->dev, data->cb_data);
	}
	/* let other work run in between */
	uart_ipc_kick(data);
}
#else
static inline void uart_ipc_kick(struct uart_ipc_data *data)
{
	ARG_UNUSED(data);
}
#endif

static void uart_ipc_flush(struct k_work *work)
{
	struct k_work_delayable *dw = k_work_delayable_from_work(work);
	struct uart_ipc_data *data = CONTAINER_OF(dw, struct uart_ipc_data, flush);
	uint8_t *p;
	uint32_t n;
	int ret;

	while (data->bound) {
		n = MIN(ring_buf_get_ptr(&data->tx, &p, 0), CONFIG_UART_IPC_CHUNK);
		if (n == 0) {
			break;
		}
		ret = ipc_service_send(&data->ept, p, n);
		if (ret < 0) {
			/* shared ring full: try again shortly */
			k_work_reschedule(&data->flush, K_MSEC(1));
			break;
		}
		ring_buf_consume(&data->tx, n);
	}

	uart_ipc_kick(data);
}

static uint32_t uart_ipc_put(const struct device *dev, const uint8_t *buf, uint32_t len)
{
	struct uart_ipc_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t n = ring_buf_put(&data->tx, buf, len);

	k_spin_unlock(&data->lock, key);

	if (data->bound && !k_is_pre_kernel()) {
		k_work_schedule(&data->flush, K_NO_WAIT);
	}
	return n;
}

static void uart_ipc_poll_out(const struct device *dev, unsigned char c)
{
	struct uart_ipc_data *data = dev->data;

	if (uart_ipc_put(dev, &c, 1) == 0) {
		data->tx_drops++;
	}
}

static int uart_ipc_poll_in(const struct device *dev, unsigned char *c)
{
	struct uart_ipc_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t n = ring_buf_get(&data->rx, c, 1);

	k_spin_unlock(&data->lock, key);
	return n == 1 ? 0 : -1;
}

static void uart_ipc_received(const void *buf, size_t len, void *priv)
{
	struct uart_ipc_data *data = priv;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t n = ring_buf_put(&data->rx, buf, len);

	k_spin_unlock(&data->lock, key);
	data->rx_drops += len - n;
	uart_ipc_kick(data);
}

static void uart_ipc_bound(void *priv)
{
	struct uart_ipc_data *data = priv;

	data->bound = true;
	k_work_schedule(&data->flush, K_NO_WAIT);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_ipc_fifo_fill(const struct device *dev, const uint8_t *buf, int len)
{
	return uart_ipc_put(dev, buf, len);
}

static int uart_ipc_fifo_read(const struct device *dev, uint8_t *buf, const int len)
{
	struct uart_ipc_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t n = ring_buf_get(&data->rx, buf, len);

	k_spin_unlock(&data->lock, key);
	return n;
}

static void uart_ipc_irq_tx_enable(const struct device *dev)
{
	struct uart_ipc_data *data = dev->data;

	data->tx_en = true;
	uart_ipc_kick(data);
}

static void uart_ipc_irq_tx_disable(const struct device *dev)
{
	((struct uart_ipc_data *)dev->data)->tx_en = false;
}

static int uart_ipc_irq_tx_ready(const struct device *dev)
{
	struct uart_ipc_data *data = dev->data;

	return data->tx_en ? ring_buf_space_get(&data->tx) : 0;
}

static int uart_ipc_irq_tx_complete(const struct device *dev)
{
	return ring_buf_is_empty(&((struct uart_ipc_data *)dev->data)->tx);
}

static void uart_ipc_irq_rx_enable(const struct device *dev)
{
	struct uart_ipc_data *data = dev->data;

	data->rx_en = true;
	uart_ipc_kick(data);
}

static void uart_ipc_irq_rx_disable(const struct device *dev)
{
	((struct uart_ipc_data *)dev->data)->rx_en = false;
}

static int uart_ipc_irq_rx_ready(const struct device *dev)
{
	struct uart_ipc_data *data = dev->data;

	return data->rx_en && !ring_buf_is_empty(&data->rx);
}

static int uart_ipc_irq_is_pending(const struct device *dev)
{
	return uart_ipc_pending(dev->data);
}

static void uart_ipc_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void uart_ipc_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *user_data)
{
	struct uart_ipc_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;
}
#endif

static DEVICE_API(uart, uart_ipc_api) = {
	.poll_in = uart_ipc_poll_in,
	.poll_out = uart_ipc_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ipc_fifo_fill,
	.fifo_read = uart_ipc_fifo_read,
	.irq_tx_enable = uart_ipc_irq_tx_enable,
	.irq_tx_disable = uart_ipc_irq_tx_disable,
	.irq_tx_ready = uart_ipc_irq_tx_ready,
	.irq_tx_complete = uart_ipc_irq_tx_complete,
	.irq_rx_enable = uart_ipc_irq_rx_enable,
	.irq_rx_disable = uart_ipc_irq_rx_disable,
	.irq_rx_ready = uart_ipc_irq_rx_ready,
	.irq_is_pending = uart_ipc_irq_is_pending,
	.irq_update = uart_ipc_irq_update,
	.irq_callback_set = uart_ipc_irq_callback_set,
#endif
};

static int uart_ipc_init(const struct device *dev)
{
	struct uart_ipc_data *data = dev->data;

	data->dev = dev;
	ring_buf_init(&data->tx, sizeof(data->tx_buf), data->tx_buf);
	ring_buf_init(&data->rx, sizeof(data->rx_buf), data->rx_buf);
	k_work_init_delayable(&data->flush, uart_ipc_flush);
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	k_work_init(&data->irq, uart_ipc_irq_work);
#endif
	return 0;
}

/* POST_KERNEL: the IPC instance and the endpoint */
static int uart_ipc_open(const struct device *dev)
{
	const struct uart_ipc_config *cfg = dev->config;
	struct uart_ipc_data *data = dev->data;
	const struct device *ipc = device_get_binding(cfg->ipc);
	int ret;

	if (ipc == NULL) {
		return -ENODEV;
	}

	ret = ipc_service_open_instance(ipc);
	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}

	data->ept_cfg.name = cfg->name;
	data->ept_cfg.cb.bound = uart_ipc_bound;
	data->ept_cfg.cb.received = uart_ipc_received;
	data->ept_cfg.priv = data;

	return ipc_service_register_endpoint(ipc, &data->ept, &data->ept_cfg);
}

#define UART_IPC_DEFINE(n)                                                                         \
	static struct uart_ipc_data uart_ipc_data_##n;                                             \
	static const struct uart_ipc_config uart_ipc_config_##n = {                                \
		.ipc = DT_INST_PROP(n, ipc_instance),                                     \
		.name = DT_INST_PROP(n, endpoint_name),                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, uart_ipc_init, NULL, &uart_ipc_data_##n, &uart_ipc_config_##n,    \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &uart_ipc_api);           \
	static int uart_ipc_open_##n(void)                                                         \
	{                                                                                          \
		return uart_ipc_open(DEVICE_DT_INST_GET(n));                                       \
	}                                                                                          \
	SYS_INIT(uart_ipc_open_##n, POST_KERNEL, CONFIG_UART_IPC_INIT_PRIORITY);

DT_INST_FOREACH_STATUS_OKAY(UART_IPC_DEFINE)
