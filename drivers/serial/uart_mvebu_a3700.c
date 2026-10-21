/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 3700 "standard" UART (UART0 at 0xd0012000).
 *
 * The baud rate divider is left as programmed by the bootloader (TF-A /
 * WTMI configure UART0 for 115200 from the 25 MHz crystal) unless
 * clock-frequency is given, in which case divider = clk / (16 * baud).
 */

#define DT_DRV_COMPAT marvell_armada_3700_uart

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define UART_RBR   0x00
#define UART_TSH   0x04
#define UART_CTRL  0x08
#define UART_STAT  0x0c
#define UART_BRDV  0x10
#define UART_OSAMP 0x14

#define CTRL_TXFIFO_RST BIT(15)
#define CTRL_RXFIFO_RST BIT(14)
#define CTRL_TX_RDY_INT BIT(5)
#define CTRL_RX_RDY_INT BIT(4)
#define CTRL_ERR_INT    GENMASK(3, 0)

#define STAT_TX_FIFO_EMP BIT(13)
#define STAT_TX_FIFO_FUL BIT(11)
#define STAT_TX_EMP      BIT(6)
#define STAT_TX_RDY      BIT(5)
#define STAT_RX_RDY      BIT(4)
#define STAT_ERR         GENMASK(3, 0)

#define BRDV_BAUD_MASK 0x3ff
#define BRDV_NO_XTAL   BIT(19)

struct a3700_uart_config {
	DEVICE_MMIO_ROM;
	uint32_t clock_frequency;
	uint32_t current_speed;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config)(const struct device *dev);
#endif
};

struct a3700_uart_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	bool tx_en;
#endif
};

static inline mm_reg_t base(const struct device *dev)
{
	return DEVICE_MMIO_GET(dev);
}

static int a3700_poll_in(const struct device *dev, unsigned char *c)
{
	if (!(sys_read32(base(dev) + UART_STAT) & STAT_RX_RDY)) {
		return -1;
	}
	*c = sys_read32(base(dev) + UART_RBR) & 0xff;
	return 0;
}

static void a3700_poll_out(const struct device *dev, unsigned char c)
{
	struct a3700_uart_data *data = dev->data;
	k_spinlock_key_t key;

	while (sys_read32(base(dev) + UART_STAT) & STAT_TX_FIFO_FUL) {
	}
	key = k_spin_lock(&data->lock);
	sys_write32(c, base(dev) + UART_TSH);
	k_spin_unlock(&data->lock, key);
}

static int a3700_err_check(const struct device *dev)
{
	uint32_t st = sys_read32(base(dev) + UART_STAT);
	int err = 0;

	if (st & BIT(0)) {
		err |= UART_ERROR_OVERRUN;
	}
	if (st & BIT(1)) {
		err |= UART_ERROR_PARITY;
	}
	if (st & BIT(2)) {
		err |= UART_ERROR_FRAMING;
	}
	if (st & BIT(3)) {
		err |= UART_BREAK;
	}
	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static void ctrl_update(const struct device *dev, uint32_t clr, uint32_t set)
{
	struct a3700_uart_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	uint32_t v = sys_read32(base(dev) + UART_CTRL);

	sys_write32((v & ~clr) | set, base(dev) + UART_CTRL);
	k_spin_unlock(&data->lock, key);
}

static int a3700_fifo_fill(const struct device *dev, const uint8_t *buf, int len)
{
	int n = 0;

	while (n < len && !(sys_read32(base(dev) + UART_STAT) & STAT_TX_FIFO_FUL)) {
		sys_write32(buf[n++], base(dev) + UART_TSH);
	}
	return n;
}

static int a3700_fifo_read(const struct device *dev, uint8_t *buf, const int size)
{
	int n = 0;

	while (n < size && (sys_read32(base(dev) + UART_STAT) & STAT_RX_RDY)) {
		buf[n++] = sys_read32(base(dev) + UART_RBR) & 0xff;
	}
	return n;
}

static void a3700_irq_tx_enable(const struct device *dev)
{
	struct a3700_uart_data *data = dev->data;

	data->tx_en = true;
	ctrl_update(dev, 0, CTRL_TX_RDY_INT);
}

static void a3700_irq_tx_disable(const struct device *dev)
{
	struct a3700_uart_data *data = dev->data;

	data->tx_en = false;
	ctrl_update(dev, CTRL_TX_RDY_INT, 0);
}

static int a3700_irq_tx_ready(const struct device *dev)
{
	struct a3700_uart_data *data = dev->data;

	return data->tx_en && !(sys_read32(base(dev) + UART_STAT) & STAT_TX_FIFO_FUL);
}

static int a3700_irq_tx_complete(const struct device *dev)
{
	return !!(sys_read32(base(dev) + UART_STAT) & STAT_TX_EMP);
}

static void a3700_irq_rx_enable(const struct device *dev)
{
	ctrl_update(dev, 0, CTRL_RX_RDY_INT);
}

static void a3700_irq_rx_disable(const struct device *dev)
{
	ctrl_update(dev, CTRL_RX_RDY_INT, 0);
}

static int a3700_irq_rx_ready(const struct device *dev)
{
	return !!(sys_read32(base(dev) + UART_STAT) & STAT_RX_RDY);
}

static void a3700_irq_err_enable(const struct device *dev)
{
	ctrl_update(dev, 0, CTRL_ERR_INT);
}

static void a3700_irq_err_disable(const struct device *dev)
{
	ctrl_update(dev, CTRL_ERR_INT, 0);
}

static int a3700_irq_is_pending(const struct device *dev)
{
	return a3700_irq_tx_ready(dev) ||
	       ((sys_read32(base(dev) + UART_CTRL) & CTRL_RX_RDY_INT) && a3700_irq_rx_ready(dev));
}

static void a3700_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void a3700_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				   void *cb_data)
{
	struct a3700_uart_data *data = dev->data;

	data->cb = cb;
	data->cb_data = cb_data;
}

static void a3700_isr(const struct device *dev)
{
	struct a3700_uart_data *data = dev->data;

	if (data->cb != NULL) {
		data->cb(dev, data->cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int a3700_uart_init(const struct device *dev)
{
	const struct a3700_uart_config *cfg = dev->config;
	int __maybe_unused ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

#ifdef CONFIG_PINCTRL
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			return ret;
		}
	}
#endif

	if (cfg->clock_frequency != 0 && cfg->current_speed != 0) {
		uint32_t div = DIV_ROUND_CLOSEST(cfg->clock_frequency, 16 * cfg->current_speed);
		uint32_t v;

		/* wait for the bootloader's output to drain first */
		while (!(sys_read32(base(dev) + UART_STAT) & STAT_TX_EMP)) {
		}
		v = sys_read32(base(dev) + UART_BRDV);
		v &= ~(BRDV_BAUD_MASK | BRDV_NO_XTAL);
		v |= CLAMP(div, 1, BRDV_BAUD_MASK);
		sys_write32(v, base(dev) + UART_BRDV);
		sys_write32(0, base(dev) + UART_OSAMP);
	}

	/* interrupts off, FIFOs as left by the bootloader (no reset: keeps its tail) */
	sys_write32(sys_read32(base(dev) + UART_CTRL) &
			    ~(CTRL_TX_RDY_INT | CTRL_RX_RDY_INT | CTRL_ERR_INT),
		    base(dev) + UART_CTRL);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	cfg->irq_config(dev);
#endif
	return 0;
}

static DEVICE_API(uart, a3700_uart_api) = {
	.poll_in = a3700_poll_in,
	.poll_out = a3700_poll_out,
	.err_check = a3700_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = a3700_fifo_fill,
	.fifo_read = a3700_fifo_read,
	.irq_tx_enable = a3700_irq_tx_enable,
	.irq_tx_disable = a3700_irq_tx_disable,
	.irq_tx_ready = a3700_irq_tx_ready,
	.irq_tx_complete = a3700_irq_tx_complete,
	.irq_rx_enable = a3700_irq_rx_enable,
	.irq_rx_disable = a3700_irq_rx_disable,
	.irq_rx_ready = a3700_irq_rx_ready,
	.irq_err_enable = a3700_irq_err_enable,
	.irq_err_disable = a3700_irq_err_disable,
	.irq_is_pending = a3700_irq_is_pending,
	.irq_update = a3700_irq_update,
	.irq_callback_set = a3700_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define A3700_IRQ_CONFIG(n)                                                                        \
	static void a3700_uart_irq_config_##n(const struct device *dev)                            \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, uart_sum, irq),                                 \
			    DT_INST_IRQ_BY_NAME(n, uart_sum, priority), a3700_isr,                 \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ_BY_NAME(n, uart_sum, flags));       \
		irq_enable(DT_INST_IRQ_BY_NAME(n, uart_sum, irq));                                 \
	}
#define A3700_IRQ_CONFIG_REF(n) .irq_config = a3700_uart_irq_config_##n,
#else
#define A3700_IRQ_CONFIG(n)
#define A3700_IRQ_CONFIG_REF(n)
#endif

#define A3700_PINCTRL(n) IF_ENABLED(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEFINE(n);))
#define A3700_PINCTRL_REF(n)                                                                       \
	COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define A3700_UART_INIT(n)                                                                         \
	A3700_PINCTRL(n)                                                                           \
	A3700_IRQ_CONFIG(n)                                                                        \
	static const struct a3700_uart_config a3700_uart_cfg_##n = {                               \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clock_frequency = DT_INST_PROP_OR(n, clock_frequency, 0),                         \
		.current_speed = DT_INST_PROP_OR(n, current_speed, 0),                             \
		.pcfg = A3700_PINCTRL_REF(n), A3700_IRQ_CONFIG_REF(n)};                            \
	static struct a3700_uart_data a3700_uart_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, a3700_uart_init, NULL, &a3700_uart_data_##n, &a3700_uart_cfg_##n, \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &a3700_uart_api);

DT_INST_FOREACH_STATUS_OKAY(A3700_UART_INIT)
