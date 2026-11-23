/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SPI controller driver for the Allwinner SPI controllers found on the
 * A523/A527/T527 SoCs (register compatible with the sun50i-r329 SPI).
 *
 * The SoC has no clock controller driver yet: the CCU bus gate, reset line
 * and module clock of the controller are poked directly, with the module
 * clock kept on the always-on 24 MHz oscillator and divided in the CCU.
 * Transfers are polled, interrupts and DMA are not used. Timings already
 * set up by earlier firmware boot stages are not reconfigured.
 */

#define DT_DRV_COMPAT allwinner_sun55i_a523_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_allwinner);

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys/util.h>

#include "spi_context.h"

#define SPI_FIFO_DEPTH 64
#define SPI_MAX_XFER_SIZE 0xffffff

/* SPI registers */
#define SPI_GCR_REG 0x04
#define SPI_GCR_BUS_ENABLE BIT(0)
#define SPI_GCR_MASTER BIT(1)
#define SPI_GCR_TP BIT(7)
#define SPI_GCR_SRST BIT(31)

#define SPI_TCR_REG 0x08
#define SPI_TCR_CPHA BIT(0)
#define SPI_TCR_CPOL BIT(1)
#define SPI_TCR_SPOL BIT(2)
#define SPI_TCR_SS_MANUAL BIT(6)
#define SPI_TCR_SS_LEVEL BIT(7)
#define SPI_TCR_DHB BIT(8)
#define SPI_TCR_FBS BIT(12)
#define SPI_TCR_SDM BIT(13)
#define SPI_TCR_XCH BIT(31)

#define SPI_INT_STA_REG 0x14
#define SPI_INT_STA_TC BIT(12)

#define SPI_FCR_REG 0x18
#define SPI_FCR_RF_RST BIT(15)
#define SPI_FCR_TF_RST BIT(31)

#define SPI_FSR_REG 0x1c
#define SPI_FSR_RF_CNT GENMASK(7, 0)
#define SPI_FSR_TF_CNT GENMASK(23, 16)
#define SPI_FSR_TF_CNT_SHIFT 16

#define SPI_BC_REG 0x30
#define SPI_TC_REG 0x34
#define SPI_BCC_REG 0x38
#define SPI_BCC_STC_MASK GENMASK(23, 0)

#define SPI_TXD_REG 0x200
#define SPI_RXD_REG 0x300

/* A523 CCU: module clock plus bus gate and reset of the SPI controllers */
#define SPI_CCU_BASE 0x02001000
#define SPI_CCU_SIZE 0x1000
#define SPI_CCU_MOD_CLK_REG 0x940
#define SPI_CCU_MOD_CLK_M_MASK GENMASK(4, 0)
#define SPI_CCU_MOD_CLK_GATE BIT(31)
#define SPI_CCU_BUS_REG 0x96c

/* PIO (GPIO) block: pin configuration registers, per-port banks of 0x24 */
#define SPI_PIO_BASE 0x02000000
#define SPI_PIO_SIZE 0x1000
#define SPI_PIO_CFG_REG(port, idx) (((port) - 'A') * 0x24 + (idx) * 4)

#define SPI_TIMEOUT_MSEC(len, speed)                                                             \
	MIN(60000u, MAX(100u, (uint32_t)MIN((uint64_t)(len) * 40u * 1000 / (speed), 40000u)))

struct spi_allwinner_config {
	uintptr_t base_phys;
	uint32_t base_size;
	uint32_t clock_frequency;
	uint8_t ccu_instance;
};

struct spi_allwinner_data {
	struct spi_context ctx;
	mm_reg_t base;
	mm_reg_t ccu;
	mm_reg_t pio;
};

static inline uint32_t spi_allwinner_read(struct spi_allwinner_data *data, uint32_t reg)
{
	return sys_read32(data->base + reg);
}

static inline void spi_allwinner_write(struct spi_allwinner_data *data, uint32_t reg,
				       uint32_t value)
{
	sys_write32(value, data->base + reg);
}

static void spi_allwinner_ccu_enable(struct spi_allwinner_data *data, uint8_t ccu_instance)
{
	uint32_t reg;

	reg = sys_read32(data->ccu + SPI_CCU_BUS_REG);
	reg |= BIT(ccu_instance) | BIT(ccu_instance + 16);
	sys_write32(reg, data->ccu + SPI_CCU_BUS_REG);

	/* Module clock from the always-on 24 MHz oscillator, divided by one */
	sys_write32(SPI_CCU_MOD_CLK_GATE,
		    data->ccu + SPI_CCU_MOD_CLK_REG + ccu_instance * 4);
}

static int spi_allwinner_set_frequency(const struct spi_allwinner_config *cfg,
				       struct spi_allwinner_data *data, uint32_t frequency,
				       uint32_t *effective_speed)
{
	mm_reg_t clk_reg = data->ccu + SPI_CCU_MOD_CLK_REG + cfg->ccu_instance * 4;
	uint32_t div;

	if (frequency == 0 || frequency > cfg->clock_frequency) {
		return -EINVAL;
	}

	div = DIV_ROUND_UP(cfg->clock_frequency, frequency);
	if (div > SPI_CCU_MOD_CLK_M_MASK + 1) {
		return -ENOTSUP;
	}

	sys_write32((div - 1) | SPI_CCU_MOD_CLK_GATE, clk_reg);
	*effective_speed = cfg->clock_frequency / div;

	return 0;
}

#if defined(CONFIG_SPI_ALLWINNER_A523_SPI0_PC_PINMUX)
static void spi_allwinner_pinmux_spi0_pc(struct spi_allwinner_data *data)
{
	/* PC2, PC3 -> CFG0, PC4 -> CFG1, PC12 -> CFG2, function "spi0" */
	uint32_t reg;

	reg = sys_read32(data->pio + SPI_PIO_CFG_REG('C', 0));
	reg = (reg & ~(GENMASK(11, 8) | GENMASK(15, 12))) | 4u << 8 | 4u << 12;
	sys_write32(reg, data->pio + SPI_PIO_CFG_REG('C', 0));

	reg = sys_read32(data->pio + SPI_PIO_CFG_REG('C', 1));
	reg = (reg & ~GENMASK(3, 0)) | 4u;
	sys_write32(reg, data->pio + SPI_PIO_CFG_REG('C', 1));

	reg = sys_read32(data->pio + SPI_PIO_CFG_REG('C', 2));
	reg = (reg & ~GENMASK(19, 16)) | 4u << 16;
	sys_write32(reg, data->pio + SPI_PIO_CFG_REG('C', 2));
}
#endif

static int spi_allwinner_configure(const struct device *dev, const struct spi_config *config)
{
	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Dual/quad lines not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Word size must be 8");
		return -ENOTSUP;
	}

	return 0;
}

static void spi_allwinner_fill_fifo(struct spi_allwinner_data *data, const uint8_t **tx,
				    uint32_t *tx_left)
{
	uint32_t level = (spi_allwinner_read(data, SPI_FSR_REG) & SPI_FSR_TF_CNT) >>
			 SPI_FSR_TF_CNT_SHIFT;

	for (uint32_t count = MIN(SPI_FIFO_DEPTH - level, *tx_left); count != 0; count--) {
		uint8_t byte = (*tx != NULL) ? **tx : 0;

		sys_write8(byte, data->base + SPI_TXD_REG);
		if (*tx != NULL) {
			(*tx)++;
		}
		(*tx_left)--;
	}
}

static void spi_allwinner_drain_fifo(struct spi_allwinner_data *data, uint8_t **rx,
				     uint32_t *rx_left)
{
	for (uint32_t level = spi_allwinner_read(data, SPI_FSR_REG) & SPI_FSR_RF_CNT;
	     level != 0 && *rx_left != 0; level--) {
		**rx = sys_read8(data->base + SPI_RXD_REG);
		(*rx)++;
		(*rx_left)--;
	}
}

static int spi_allwinner_xfer(const struct device *dev, const struct spi_config *config)
{
	const struct spi_allwinner_config *cfg = dev->config;
	struct spi_allwinner_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	const uint8_t *tx = ctx->tx_buf;
	uint8_t *rx = ctx->rx_buf;
	/* Dummy zero bytes are sent for RX-only transfers */
	uint32_t len = MAX(ctx->tx_len, ctx->rx_len);
	uint32_t tx_left = len, rx_left = (rx != NULL) ? len : 0;
	uint32_t effective_speed, tcr, reg;
	k_timepoint_t end;
	int ret;

	if (len > SPI_MAX_XFER_SIZE - 1) {
		return -EMSGSIZE;
	}

	ret = spi_allwinner_set_frequency(cfg, data, config->frequency, &effective_speed);
	if (ret < 0) {
		return ret;
	}

	spi_allwinner_write(data, SPI_INT_STA_REG, SPI_INT_STA_TC);
	spi_allwinner_write(data, SPI_FCR_REG, SPI_FCR_RF_RST | SPI_FCR_TF_RST);

	tcr = spi_allwinner_read(data, SPI_TCR_REG);
	tcr &= ~(SPI_TCR_CPHA | SPI_TCR_CPOL | SPI_TCR_SPOL | SPI_TCR_DHB | SPI_TCR_FBS |
		 SPI_TCR_SDM | SPI_TCR_SS_LEVEL);
	if (config->operation & SPI_MODE_CPOL) {
		tcr |= SPI_TCR_CPOL;
	}
	if (config->operation & SPI_MODE_CPHA) {
		tcr |= SPI_TCR_CPHA;
	}
	if (config->operation & SPI_TRANSFER_LSB) {
		tcr |= SPI_TCR_FBS;
	}
	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		tcr |= SPI_TCR_SPOL;
	}
	/* Do not fill the RX FIFO with bogus data on TX-only transfers */
	if (rx == NULL) {
		tcr |= SPI_TCR_DHB;
	}
	/*
	 * Chip select is driven by the controller (hardware CS 0 only),
	 * asserted manually around the transfer.
	 */
	tcr |= SPI_TCR_SS_MANUAL;
	/* Normal sample mode: SCLK <= 24 MHz */
	tcr |= SPI_TCR_SDM;
	spi_allwinner_write(data, SPI_TCR_REG, tcr);

	reg = spi_allwinner_read(data, SPI_GCR_REG);
	spi_allwinner_write(data, SPI_GCR_REG, reg | SPI_GCR_BUS_ENABLE);

	spi_allwinner_write(data, SPI_BC_REG, len);
	spi_allwinner_write(data, SPI_TC_REG, len);
	spi_allwinner_write(data, SPI_BCC_REG, len & SPI_BCC_STC_MASK);

	spi_allwinner_fill_fifo(data, &tx, &tx_left);

	/* Assert CS and start the burst */
	spi_allwinner_write(data, SPI_TCR_REG, tcr | SPI_TCR_SS_LEVEL | SPI_TCR_XCH);

	end = sys_timepoint_calc(K_MSEC(SPI_TIMEOUT_MSEC(len, effective_speed)));
	while (!(spi_allwinner_read(data, SPI_INT_STA_REG) & SPI_INT_STA_TC)) {
		if (sys_timepoint_expired(end)) {
			LOG_ERR("Transfer timeout, %u bytes remaining", tx_left + rx_left);
			ret = -ETIMEDOUT;
			break;
		}

		spi_allwinner_fill_fifo(data, &tx, &tx_left);
		spi_allwinner_drain_fifo(data, &rx, &rx_left);
	}

	spi_allwinner_drain_fifo(data, &rx, &rx_left);

	/* De-assert CS */
	tcr = spi_allwinner_read(data, SPI_TCR_REG) & ~SPI_TCR_SS_LEVEL;
	spi_allwinner_write(data, SPI_TCR_REG, tcr);

	spi_allwinner_write(data, SPI_INT_STA_REG, SPI_INT_STA_TC);

	return ret;
}

static int spi_allwinner_transceive(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	struct spi_allwinner_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;

	spi_context_lock(ctx, false, NULL, NULL, config);

	ret = spi_allwinner_configure(dev, config);
	if (ret != 0) {
		spi_context_release(ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	do {
		uint32_t len = MAX(ctx->tx_len, ctx->rx_len);

		ret = spi_allwinner_xfer(dev, config);
		if (ret != 0) {
			break;
		}

		spi_context_update_tx(ctx, 1, len);
		spi_context_update_rx(ctx, 1, len);
	} while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));

	spi_context_release(ctx, ret);

	return ret;
}

static int spi_allwinner_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_allwinner_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_allwinner_driver_api) = {
	.transceive = spi_allwinner_transceive,
	.release = spi_allwinner_release,
};

static int spi_allwinner_init(const struct device *dev)
{
	const struct spi_allwinner_config *cfg = dev->config;
	struct spi_allwinner_data *data = dev->data;
	uint32_t reg, timeout = 1000;

	device_map(&data->base, cfg->base_phys, cfg->base_size, K_MEM_CACHE_NONE);
	device_map(&data->ccu, SPI_CCU_BASE, SPI_CCU_SIZE, K_MEM_CACHE_NONE);
	device_map(&data->pio, SPI_PIO_BASE, SPI_PIO_SIZE, K_MEM_CACHE_NONE);

	spi_allwinner_ccu_enable(data, cfg->ccu_instance);

#if defined(CONFIG_SPI_ALLWINNER_A523_SPI0_PC_PINMUX)
	if (cfg->ccu_instance == 0) {
		spi_allwinner_pinmux_spi0_pc(data);
	}
#endif

	/* Soft reset the controller */
	spi_allwinner_write(data, SPI_GCR_REG, SPI_GCR_SRST);
	do {
		reg = spi_allwinner_read(data, SPI_GCR_REG);
	} while ((reg & SPI_GCR_SRST) != 0 && --timeout != 0);
	if (timeout == 0) {
		LOG_ERR("Controller reset timed out");
		return -ETIMEDOUT;
	}

	spi_allwinner_write(data, SPI_GCR_REG, SPI_GCR_MASTER | SPI_GCR_TP);

	spi_context_cs_configure_all(&data->ctx);

	/* Keep CS de-asserted while the controller is idle */
	reg = spi_allwinner_read(data, SPI_TCR_REG) & ~SPI_TCR_SS_LEVEL;
	reg |= SPI_TCR_SS_MANUAL;
	spi_allwinner_write(data, SPI_TCR_REG, reg);

	return 0;
}

#define SPI_ALLWINNER_DEFINE(inst)                                                            \
	static struct spi_allwinner_data spi_allwinner_data_##inst = {                        \
		SPI_CONTEXT_INIT_LOCK(spi_allwinner_data_##inst, ctx),                        \
		SPI_CONTEXT_INIT_SYNC(spi_allwinner_data_##inst, ctx),                        \
	};                                                                                    \
	static const struct spi_allwinner_config spi_allwinner_cfg_##inst = {                 \
		.base_phys = DT_INST_REG_ADDR(inst),                                          \
		.base_size = DT_INST_REG_SIZE(inst),                                          \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                       \
		.ccu_instance = DT_INST_PROP(inst, allwinner_ccu_instance),                   \
	};                                                                                    \
	SPI_DEVICE_DT_INST_DEFINE(inst, spi_allwinner_init, NULL, &spi_allwinner_data_##inst, \
				  &spi_allwinner_cfg_##inst, POST_KERNEL,                     \
				  CONFIG_SPI_INIT_PRIORITY, &spi_allwinner_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_ALLWINNER_DEFINE)
