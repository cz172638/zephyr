/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip SPI controller (RK3066 and later, RK3399: 32 entry FIFOs),
 * master mode, 8-bit frames, polled. Register layout after Linux
 * drivers/spi/spi-rockchip.c.
 */

#define DT_DRV_COMPAT rockchip_rk3066_spi

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(spi_rockchip, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define CTRLR0   0x0000U
#define  CR0_DFS_8BIT     (1U << 0)
#define  CR0_SCPH         BIT(6)
#define  CR0_SCPOL        BIT(7)
#define  CR0_SSD_ONE      BIT(10)
#define  CR0_FBM_LSB      BIT(12)
#define  CR0_BHT_8BIT     BIT(13)
#define  CR0_RSD_SHIFT    14
#define  CR0_XFM_TR       (0U << 18)
#define  CR0_OPM_SLAVE    BIT(20)
#define CTRLR1   0x0004U
#define SSIENR   0x0008U
#define SER      0x000cU
#define BAUDR    0x0010U
#define TXFTLR   0x0014U
#define RXFTLR   0x0018U
#define TXFLR    0x001cU
#define RXFLR    0x0020U
#define SR       0x0024U
#define  SR_BUSY BIT(0)
#define  SR_TFE  BIT(2)
#define IMR      0x002cU
#define ICR      0x0038U
#define DMACR    0x003cU
#define VERSION  0x0048U
#define TXDR     0x0400U
#define RXDR     0x0800U

#define MAX_CS        2U
#define XFER_TIMEOUT  K_MSEC(1000)

struct spi_rk_config {
	DEVICE_MMIO_ROM;
	const struct device *clk_dev;
	clock_control_subsys_t sclk;
	clock_control_subsys_t pclk;
	uint32_t clock_frequency;
	uint8_t rx_sample_delay;
	const struct pinctrl_dev_config *pcfg;
};

struct spi_rk_data {
	DEVICE_MMIO_RAM;
	struct spi_context ctx;
	uint32_t sclk_hz;
	uint32_t fifo_len;
};

static inline uint32_t rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

static inline void wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, DEVICE_MMIO_GET(dev) + off);
}

static int spi_rk_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_rk_config *cfg = dev->config;
	struct spi_rk_data *data = dev->data;
	uint32_t ctrl, div;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}
	if (config->operation & (SPI_OP_MODE_PERIPHERAL | SPI_MODE_LOOP)) {
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(config->operation) != 8U) {
		return -ENOTSUP;
	}
	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		return -ENOTSUP;
	}
	if (!spi_cs_is_gpio(config) && config->peripheral >= MAX_CS) {
		return -EINVAL;
	}
	if (config->frequency == 0U) {
		return -EINVAL;
	}

	ctrl = CR0_DFS_8BIT | CR0_SSD_ONE | CR0_BHT_8BIT | CR0_XFM_TR |
	       ((uint32_t)cfg->rx_sample_delay << CR0_RSD_SHIFT);
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrl |= CR0_SCPH;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrl |= CR0_SCPOL;
	}
	if (config->operation & SPI_TRANSFER_LSB) {
		ctrl |= CR0_FBM_LSB;
	}

	/* sclk_out = sclk / BAUDR, BAUDR even, 2..65534 */
	div = DIV_ROUND_UP(data->sclk_hz, config->frequency);
	div = CLAMP(ROUND_UP(div, 2U), 2U, 65534U);

	wr(dev, SSIENR, 0);
	wr(dev, CTRLR0, ctrl);
	wr(dev, BAUDR, div);
	data->ctx.config = config;
	LOG_DBG("%s: mode %u, %u Hz (sclk %u / %u)", dev->name, SPI_MODE_GET(config->operation),
		data->sclk_hz / div, data->sclk_hz, div);
	return 0;
}

static void cs_set(const struct device *dev, const struct spi_config *config, bool on)
{
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(&((struct spi_rk_data *)dev->data)->ctx, on);
	} else {
		wr(dev, SER, on ? BIT(config->peripheral) : 0U);
	}
}

static int spi_rk_xfer(const struct device *dev)
{
	struct spi_rk_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	size_t total = MAX(spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx));
	size_t sent = 0, got = 0;
	k_timepoint_t end = sys_timepoint_calc(XFER_TIMEOUT);

	wr(dev, SSIENR, 1);
	while (got < total) {
		uint32_t n;

		while (sent < total && sent - got < data->fifo_len) {
			uint8_t b = 0;

			if (spi_context_tx_buf_on(ctx)) {
				b = *ctx->tx_buf;
			}
			if (spi_context_tx_on(ctx)) {
				spi_context_update_tx(ctx, 1, 1);
			}
			wr(dev, TXDR, b);
			sent++;
		}
		n = rd(dev, RXFLR);
		while (n-- > 0U) {
			uint8_t b = (uint8_t)rd(dev, RXDR);

			if (spi_context_rx_buf_on(ctx)) {
				*ctx->rx_buf = b;
			}
			if (spi_context_rx_on(ctx)) {
				spi_context_update_rx(ctx, 1, 1);
			}
			got++;
		}
		if (got < total && sys_timepoint_expired(end)) {
			LOG_ERR("%s: timeout, %zu/%zu bytes", dev->name, got, total);
			wr(dev, SSIENR, 0);
			return -ETIMEDOUT;
		}
	}
	while (rd(dev, SR) & SR_BUSY) {
		if (sys_timepoint_expired(end)) {
			wr(dev, SSIENR, 0);
			return -ETIMEDOUT;
		}
	}
	wr(dev, SSIENR, 0);
	return 0;
}

static int spi_rk_transceive(const struct device *dev, const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	struct spi_rk_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);
	ret = spi_rk_configure(dev, config);
	if (ret == 0) {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
		cs_set(dev, config, true);
		ret = spi_rk_xfer(dev);
		if (!(config->operation & SPI_HOLD_ON_CS) || ret != 0) {
			cs_set(dev, config, false);
		}
	}
	spi_context_release(&data->ctx, ret);
	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_rk_transceive_async(const struct device *dev, const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				   void *userdata)
{
	return -ENOTSUP;
}
#endif

static int spi_rk_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_rk_data *data = dev->data;

	if (!spi_context_configured(&data->ctx, config)) {
		return -EINVAL;
	}
	cs_set(dev, config, false);
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_rk_api) = {
	.transceive = spi_rk_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_rk_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_rk_release,
};

/* TXFTLR is FIFO depth - 1 wide: the highest value that sticks */
static uint32_t fifo_probe(const struct device *dev)
{
	uint32_t n;

	for (n = 2; n < 64U; n++) {
		wr(dev, TXFTLR, n);
		if (rd(dev, TXFTLR) != n) {
			break;
		}
	}
	wr(dev, TXFTLR, 0);
	return n;
}

static int spi_rk_init(const struct device *dev)
{
	const struct spi_rk_config *cfg = dev->config;
	struct spi_rk_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0 && ret != -ENOENT) {
		return ret;
	}
	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	(void)clock_control_on(cfg->clk_dev, cfg->pclk);
	if (cfg->clock_frequency != 0U) {
		ret = clock_control_set_rate(cfg->clk_dev, cfg->sclk,
					     (clock_control_subsys_rate_t)(uintptr_t)
						     cfg->clock_frequency);
		if (ret != 0) {
			LOG_WRN("%s: set_rate %u: %d", dev->name, cfg->clock_frequency, ret);
		}
	}
	(void)clock_control_on(cfg->clk_dev, cfg->sclk);
	ret = clock_control_get_rate(cfg->clk_dev, cfg->sclk, &data->sclk_hz);
	if (ret != 0 || data->sclk_hz == 0U) {
		LOG_ERR("%s: no sclk rate", dev->name);
		return -EIO;
	}

	wr(dev, SSIENR, 0);
	wr(dev, SER, 0);
	wr(dev, IMR, 0);
	wr(dev, DMACR, 0);
	wr(dev, ICR, 0xffffffffU);
	data->fifo_len = fifo_probe(dev);

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret != 0) {
		return ret;
	}
	spi_context_unlock_unconditionally(&data->ctx);
	LOG_INF("%s: version %08x, fifo %u, sclk %u Hz", dev->name, rd(dev, VERSION),
		data->fifo_len, data->sclk_hz);
	return 0;
}

#define SPI_RK_INIT(n)                                                                             \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct spi_rk_config spi_rk_config_##n = {                                    \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, spiclk)),                  \
		.sclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, spiclk, clkid),     \
		.pclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, apb_pclk, clkid),   \
		.clock_frequency = DT_INST_PROP_OR(n, clock_frequency, 0),                         \
		.rx_sample_delay = DT_INST_PROP(n, rx_sample_delay),                               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
	static struct spi_rk_data spi_rk_data_##n = {                                              \
		SPI_CONTEXT_INIT_LOCK(spi_rk_data_##n, ctx),                                       \
		SPI_CONTEXT_INIT_SYNC(spi_rk_data_##n, ctx),                                       \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_rk_init, NULL, &spi_rk_data_##n, &spi_rk_config_##n,      \
				  POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_rk_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_RK_INIT)
