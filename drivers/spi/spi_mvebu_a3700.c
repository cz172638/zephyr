/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 3700 SPI controller, legacy (byte) mode: single data
 * line in each direction, FIFO off, one byte per transfer, polled.
 *
 * Registers (offsets from the controller base):
 *   0x00 CTRL  [1] XFER_RDY, [19:16] SPI_EN (native chip selects)
 *   0x04 CFG   [4:0] clock prescaler, [5] 2-byte mode, [6] CPHA, [7] CPOL,
 *              [9] FIFO flush, [17] FIFO enable
 *   0x08 DOUT  writing starts a transfer
 *   0x0c DIN
 *
 * Semantics follow Linux spi-armada-3700.c and U-Boot mvebu_a3700_spi.c.
 */

#define DT_DRV_COMPAT marvell_armada_3700_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mvebu_a3700, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define A3700_CTRL      0x00
#define CTRL_XFER_RDY   BIT(1)
#define CTRL_SPI_EN(cs) BIT(16 + (cs))
#define A3700_CFG       0x04
#define CFG_PRESCALE    GENMASK(4, 0)
#define CFG_BYTE_LEN    BIT(5)
#define CFG_CLK_PHA     BIT(6)
#define CFG_CLK_POL     BIT(7)
#define CFG_FIFO_FLUSH  BIT(9)
#define CFG_FIFO_EN     BIT(17)
#define A3700_DOUT      0x08
#define A3700_DIN       0x0c

#define A3700_NUM_CS     4
#define A3700_TIMEOUT_US 1000

struct spi_a3700_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
};

struct spi_a3700_data {
	DEVICE_MMIO_RAM;
	struct spi_context ctx;
	uint32_t clk_hz;
	uint32_t cur_freq;
	uint16_t cur_op;
};

static inline uint32_t a3700_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static inline void a3700_write(const struct device *dev, uint32_t reg, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + reg);
}

static int a3700_wait_ready(const struct device *dev)
{
	if (!WAIT_FOR(a3700_read(dev, A3700_CTRL) & CTRL_XFER_RDY, A3700_TIMEOUT_US,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}
	return 0;
}

static void a3700_native_cs(const struct device *dev, unsigned int cs, bool on)
{
	uint32_t ctrl = a3700_read(dev, A3700_CTRL);

	ctrl = on ? (ctrl | CTRL_SPI_EN(cs)) : (ctrl & ~CTRL_SPI_EN(cs));
	a3700_write(dev, A3700_CTRL, ctrl);
}

/*
 * Prescaler: values 1..15 divide by the value; from 0x10 up the divider
 * is 2 * (value - 0x10), giving even dividers up to 30.
 */
static uint32_t a3700_prescale(uint32_t clk_hz, uint32_t hz)
{
	uint32_t div = DIV_ROUND_UP(clk_hz, hz);

	if (div > 0xf) {
		div = 0x10 + (div + 1) / 2;
	}
	return MIN(MAX(div, 1U), 0x1fU);
}

static int a3700_configure(const struct device *dev, const struct spi_config *config)
{
	struct spi_a3700_data *data = dev->data;
	uint32_t cfg;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		return -ENOTSUP;
	}
	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_CONTROLLER) {
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		return -ENOTSUP;
	}
	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -ENOTSUP;
	}
	if (config->operation & (SPI_TRANSFER_LSB | SPI_MODE_LOOP)) {
		return -ENOTSUP;
	}
	if (!spi_cs_is_gpio(config) && config->peripheral >= A3700_NUM_CS) {
		return -EINVAL;
	}
	if (config->frequency == 0) {
		return -EINVAL;
	}

	cfg = a3700_read(dev, A3700_CFG);
	cfg &= ~(CFG_PRESCALE | CFG_CLK_PHA | CFG_CLK_POL);
	cfg |= a3700_prescale(data->clk_hz, config->frequency);
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		cfg |= CFG_CLK_POL;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		cfg |= CFG_CLK_PHA;
	}
	a3700_write(dev, A3700_CFG, cfg);

	data->ctx.config = config;
	return 0;
}

static int a3700_xfer(const struct device *dev)
{
	struct spi_a3700_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {
		uint8_t out = spi_context_tx_buf_on(ctx) ? *ctx->tx_buf : 0;

		ret = a3700_wait_ready(dev);
		if (ret < 0) {
			return ret;
		}
		a3700_write(dev, A3700_DOUT, out);

		ret = a3700_wait_ready(dev);
		if (ret < 0) {
			return ret;
		}
		if (spi_context_rx_buf_on(ctx)) {
			*ctx->rx_buf = (uint8_t)a3700_read(dev, A3700_DIN);
		}

		spi_context_update_tx(ctx, 1, 1);
		spi_context_update_rx(ctx, 1, 1);
	}

	return 0;
}

static int spi_a3700_transceive(const struct device *dev, const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	struct spi_a3700_data *data = dev->data;
	bool native_cs = !spi_cs_is_gpio(config);
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = a3700_configure(dev, config);
	if (ret < 0) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (native_cs) {
		a3700_native_cs(dev, config->peripheral, true);
	} else {
		spi_context_cs_control(&data->ctx, true);
	}

	ret = a3700_xfer(dev);

	if (ret < 0 || !(config->operation & SPI_HOLD_ON_CS)) {
		if (native_cs) {
			(void)a3700_wait_ready(dev);
			a3700_native_cs(dev, config->peripheral, false);
		} else {
			spi_context_cs_control(&data->ctx, false);
		}
	}

out:
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_a3700_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_a3700_data *data = dev->data;

	if (!spi_cs_is_gpio(config) && config->peripheral < A3700_NUM_CS) {
		a3700_native_cs(dev, config->peripheral, false);
	} else {
		spi_context_cs_control(&data->ctx, false);
	}
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static int spi_a3700_init(const struct device *dev)
{
	const struct spi_a3700_config *cfg = dev->config;
	struct spi_a3700_data *data = dev->data;
	uint32_t val;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	ret = clock_control_on(cfg->clk_dev, cfg->clk_id);
	if (ret < 0) {
		return ret;
	}
	ret = clock_control_get_rate(cfg->clk_dev, cfg->clk_id, &data->clk_hz);
	if (ret < 0 || data->clk_hz == 0) {
		LOG_ERR("unknown SPI clock rate");
		return -EIO;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		return ret;
	}

	/* Legacy mode: flush and disable the FIFO, one byte per transfer */
	val = a3700_read(dev, A3700_CFG);
	a3700_write(dev, A3700_CFG, val | CFG_FIFO_FLUSH);
	if (!WAIT_FOR((a3700_read(dev, A3700_CFG) & CFG_FIFO_FLUSH) == 0, A3700_TIMEOUT_US,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}
	val &= ~(CFG_FIFO_EN | CFG_BYTE_LEN | CFG_FIFO_FLUSH);
	a3700_write(dev, A3700_CFG, val);

	/* All native chip selects inactive */
	a3700_write(dev, A3700_CTRL, a3700_read(dev, A3700_CTRL) & ~GENMASK(19, 16));

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("%s: input clock %u Hz", dev->name, data->clk_hz);

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_a3700_api) = {
	.transceive = spi_a3700_transceive,
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_a3700_release,
};

#define SPI_A3700_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct spi_a3700_config spi_a3700_config_##n = {                              \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                  \
		.clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, index),                   \
	};                                                                                         \
	static struct spi_a3700_data spi_a3700_data_##n = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_a3700_data_##n, ctx),                                    \
		SPI_CONTEXT_INIT_SYNC(spi_a3700_data_##n, ctx),                                    \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_a3700_init, NULL, &spi_a3700_data_##n,                    \
				  &spi_a3700_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
				  &spi_a3700_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_A3700_INIT)
