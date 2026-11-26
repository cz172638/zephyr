/*
 * Copyright (c) 2018-2019 PHYTEC Messtechnik GmbH
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This file is based on SW_DP.c from CMSIS-DAP Source (Revision:    V2.0.0)
 * https://github.com/ARM-software/CMSIS_5/tree/develop/CMSIS/DAP/Firmware
 * Copyright (c) 2013-2017, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 */


/*
 * Serial Wire Debug Port interface bit-bang driver, with JTAG sequences when
 * TDI and TDO are wired (TCK = SWCLK, TMS = SWDIO). Whole bytes of a JTAG
 * sequence can be shifted by an SPI controller instead (jtag-spi): its SCK,
 * MOSI and MISO pads are TCK, TDI and TDO, switched from GPIO to SPI by the
 * "spi" pinctrl state for the shift.
 */

#define DT_DRV_COMPAT zephyr_swdp_gpio

#include "swdp_common.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/swdp.h>

#include "swdp_ll_pin.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(swdp, CONFIG_DP_DRIVER_LOG_LEVEL);

#define MAX_SWJ_CLOCK(delay_cycles, port_write_cycles) \
	((CPU_CLOCK / 2U) / (port_write_cycles + delay_cycles))

/*
 * Default SWCLK frequency in Hz.
 * sw_clock can be used to overwrite this default value.
 */
#define SWDP_DEFAULT_SWCLK_FREQUENCY	1000000U

#define DELAY_FAST_CYCLES		2U
#define DELAY_SLOW_CYCLES		3U

#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>
/* SPI pad functions on TCK, TDI and TDO */
#define PINCTRL_STATE_SPI PINCTRL_STATE_PRIV_START
#else
#define PINCTRL_STATE_DEFAULT 0U
#define PINCTRL_STATE_SPI 1U
struct pinctrl_dev_config;
#endif

/* bytes per SPI transfer of a JTAG sequence */
#define JTAG_SPI_CHUNK 64U

struct sw_config {
	struct gpio_dt_spec clk;
	struct gpio_dt_spec tdi;
	struct gpio_dt_spec tdo;
	const struct device *spi;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec dout;
	struct gpio_dt_spec dio;
	struct gpio_dt_spec dnoe;
	void *dout_reg;
	void *dio_reg;
	void *dnoe_reg;
	struct gpio_dt_spec noe;
	struct gpio_dt_spec reset;
	bool keep_reset_deast;
	uint32_t port_write_cycles;
	void *clk_reg;
};

struct sw_cfg_data {
	uint32_t clock_delay;
	uint8_t turnaround;
	bool data_phase;
	bool fast_clock;
	/*
	 * SPI drivers reconfigure only for a new spi_config pointer: a clock
	 * change fills the other one
	 */
	struct spi_config spi_cfg[2];
	uint8_t spi_cfg_idx;
};

/* Set SWCLK DAP hardware output pin to high level */
static ALWAYS_INLINE void pin_swclk_set(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (FAST_BITBANG_HW_SUPPORT) {
		swdp_ll_pin_set(config->clk_reg, config->clk.pin);
	} else {
		gpio_pin_set_dt(&config->clk, 1);
	}
}

/* Set SWCLK DAP hardware output pin to low level */
static ALWAYS_INLINE void pin_swclk_clr(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (FAST_BITBANG_HW_SUPPORT) {
		swdp_ll_pin_clr(config->clk_reg, config->clk.pin);
	} else {
		gpio_pin_set_dt(&config->clk, 0);
	}
}

/* Set the SWDIO DAP hardware output pin to high level */
static ALWAYS_INLINE void pin_swdio_set(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (config->dout.port) {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_set(config->dout_reg, config->dout.pin);
		} else {
			gpio_pin_set_dt(&config->dout, 1);
		}
	} else {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_set(config->dio_reg, config->dio.pin);
		} else {
			gpio_pin_set_dt(&config->dio, 1);
		}
	}
}

/* Set the SWDIO DAP hardware output pin to low level */
static ALWAYS_INLINE void pin_swdio_clr(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (config->dout.port) {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_clr(config->dout_reg, config->dout.pin);
		} else {
			gpio_pin_set_dt(&config->dout, 0);
		}
	} else {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_clr(config->dio_reg, config->dio.pin);
		} else {
			gpio_pin_set_dt(&config->dio, 0);
		}
	}
}

/* Set the SWDIO DAP hardware output pin to bit level */
static ALWAYS_INLINE void pin_swdio_out(const struct device *dev,
					const uint32_t bit)
{
	if (bit & 1U) {
		pin_swdio_set(dev);
	} else {
		pin_swdio_clr(dev);
	}
}

/* Return current level of the SWDIO DAP hardware input pin */
static ALWAYS_INLINE uint32_t pin_swdio_in(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (FAST_BITBANG_HW_SUPPORT) {
		return swdp_ll_pin_get(config->dio_reg, config->dio.pin);
	} else {
		return gpio_pin_get_dt(&config->dio);
	}
}

/*
 * Configure the SWDIO DAP hardware to output mode.
 * This is default configuration for every transfer.
 */
static ALWAYS_INLINE void pin_swdio_out_enable(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (config->dnoe.port) {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_set(config->dnoe_reg, config->dnoe.pin);
		} else {
			gpio_pin_set_dt(&config->dnoe, 1);
		}
	} else {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_output(config->dio_reg, config->dio.pin);
		} else {
			gpio_pin_configure_dt(&config->dio, GPIO_OUTPUT_ACTIVE);
		}
	}
}

/*
 * Configure the SWDIO DAP hardware to input mode.
 */
static ALWAYS_INLINE void pin_swdio_out_disable(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	if (config->dnoe.port) {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_clr(config->dnoe_reg, config->dnoe.pin);
		} else {
			gpio_pin_set_dt(&config->dnoe, 0);
		}
	} else {
		if (FAST_BITBANG_HW_SUPPORT) {
			swdp_ll_pin_input(config->dio_reg, config->dio.pin);
		} else {
			gpio_pin_configure_dt(&config->dio, GPIO_INPUT);
		}
	}
}

#define SW_CLOCK_CYCLE(dev, delay)			\
	do {						\
		pin_swclk_clr(dev);			\
		pin_delay_asm(delay);			\
		pin_swclk_set(dev);			\
		pin_delay_asm(delay);			\
	} while (0)

#define SW_WRITE_BIT(dev, bit, delay)			\
	do {						\
		pin_swdio_out(dev, bit);		\
		pin_swclk_clr(dev);			\
		pin_delay_asm(delay);			\
		pin_swclk_set(dev);			\
		pin_delay_asm(delay);			\
	} while (0)

#define SW_READ_BIT(dev, bit, delay)			\
	do {						\
		pin_swclk_clr(dev);			\
		pin_delay_asm(delay);			\
		bit = pin_swdio_in(dev);		\
		pin_swclk_set(dev);			\
		pin_delay_asm(delay);			\
	} while (0)

static int sw_output_sequence(const struct device *dev, uint32_t count,
			      const uint8_t *data)
{
	struct sw_cfg_data *sw_data = dev->data;
	unsigned int key;
	uint32_t val = 0; /* current byte */
	uint32_t n = 0; /* bit counter */

	LOG_DBG("writing %u bits", count);
	LOG_HEXDUMP_DBG(data, count, "sequence bit data");
	key = irq_lock();

	pin_swdio_out_enable(dev);
	while (count--) {
		if (n == 0U) {
			val = *data++;
			n = 8U;
		}
		if (val & 1U) {
			pin_swdio_set(dev);
		} else {
			pin_swdio_clr(dev);
		}
		SW_CLOCK_CYCLE(dev, sw_data->clock_delay);
		val >>= 1;
		n--;
	}

	irq_unlock(key);

	return 0;
}

static int sw_input_sequence(const struct device *dev, uint32_t count,
			     uint8_t *data)
{
	struct sw_cfg_data *sw_data = dev->data;
	unsigned int key;
	uint32_t val = 0U; /* current byte */
	uint32_t n = 8U; /* bit counter */
	uint32_t bit;

	LOG_DBG("reading %u bits", count);
	key = irq_lock();

	pin_swdio_out_disable(dev);
	while (count--) {
		if (n == 0U) {
			*data++ = val;
			val = 0;
			n = 8U;
		}
		SW_READ_BIT(dev, bit, sw_data->clock_delay);
		LOG_DBG("Read bit: %d", bit);
		val = (val << 1 | bit);
		n--;
	}

	*data = val; /* write last byte */
	irq_unlock(key);

	return 0;
}

static ALWAYS_INLINE void sw_cycle_turnaround(const struct device *dev)
{
	struct sw_cfg_data *sw_data = dev->data;
	uint32_t n;

	for (n = sw_data->turnaround; n; n--) {
		SW_CLOCK_CYCLE(dev, sw_data->clock_delay);
	}
}

static int sw_transfer(const struct device *dev,
		       const uint8_t request, uint32_t *const data,
		       const uint8_t idle_cycles, uint8_t *const response)
{
	struct sw_cfg_data *sw_data = dev->data;
	unsigned int key;
	uint32_t ack;
	uint32_t bit;
	uint32_t val;
	uint32_t parity = 0;
	uint32_t n;

	pin_swdio_out_enable(dev);

	LOG_DBG("request 0x%02x idle %u", request, idle_cycles);
	if (!(request & SWDP_REQUEST_RnW)) {
		LOG_DBG("write data 0x%08x", *data);
		parity = swd_get32bit_parity(*data);
	}

	key = irq_lock();

	val = SWD_REQUEST_FROM_LUT(request);
	for (n = 8U; n; n--) {
		SW_WRITE_BIT(dev, val, sw_data->clock_delay);
		val >>= 1;
	}

	pin_swdio_out_disable(dev);
	sw_cycle_turnaround(dev);

	/* Acknowledge response */
	SW_READ_BIT(dev, bit, sw_data->clock_delay);
	ack = bit << 0;
	SW_READ_BIT(dev, bit, sw_data->clock_delay);
	ack |= bit << 1;
	SW_READ_BIT(dev, bit, sw_data->clock_delay);
	ack |= bit << 2;

	if (ack == SWDP_ACK_OK) {
		/* Data transfer */
		if (request & SWDP_REQUEST_RnW) {
			/* Read data */
			val = 0U;
			for (n = 32U; n; n--) {
				/* Read RDATA[0:31] */
				SW_READ_BIT(dev, bit, sw_data->clock_delay);
				val >>= 1;
				val |= bit << 31;
			}

			/* Read parity bit */
			SW_READ_BIT(dev, bit, sw_data->clock_delay);
			sw_cycle_turnaround(dev);
			pin_swdio_out_enable(dev);

			if ((swd_get32bit_parity(val) ^ bit) & 1U) {
				ack = SWDP_TRANSFER_ERROR;
			}

			if (data) {
				*data = val;
			}

		} else {
			sw_cycle_turnaround(dev);

			pin_swdio_out_enable(dev);
			/* Write data */
			val = *data;
			for (n = 32U; n; n--) {
				SW_WRITE_BIT(dev, val, sw_data->clock_delay);
				val >>= 1;
			}

			/* Write parity bit */
			SW_WRITE_BIT(dev, parity, sw_data->clock_delay);
		}
		/* Idle cycles */
		n = idle_cycles;
		if (n) {
			pin_swdio_out(dev, 0U);
			for (; n; n--) {
				SW_CLOCK_CYCLE(dev, sw_data->clock_delay);
			}
		}

		pin_swdio_out(dev, 1U);
		irq_unlock(key);
		if (request & SWDP_REQUEST_RnW) {
			LOG_DBG("read data 0x%08x", *data);
		}

		if (response) {
			*response = (uint8_t)ack;
		}

		return 0;
	}

	if ((ack == SWDP_ACK_WAIT) || (ack == SWDP_ACK_FAULT)) {
		/* WAIT OR fault response */
		if (sw_data->data_phase) {
			for (n = 32U + 1U + sw_data->turnaround; n; n--) {
				/* Dummy Read RDATA[0:31] + Parity */
				SW_CLOCK_CYCLE(dev, sw_data->clock_delay);
			}
		} else {
			sw_cycle_turnaround(dev);
		}

		pin_swdio_out_enable(dev);
		pin_swdio_out(dev, 1U);
		irq_unlock(key);
		LOG_DBG("Transfer wait or fault");
		if (response) {
			*response = (uint8_t)ack;
		}

		return 0;
	}

	/* Protocol error */
	for (n = sw_data->turnaround + 32U + 1U; n; n--) {
		/* Back off data phase */
		SW_CLOCK_CYCLE(dev, sw_data->clock_delay);
	}

	pin_swdio_out_enable(dev);
	pin_swdio_out(dev, 1U);
	irq_unlock(key);
	LOG_INF("Protocol error");
	if (response) {
		*response = (uint8_t)ack;
	}

	return 0;
}

static int sw_pinctrl(const struct sw_config *config, uint8_t state)
{
#if defined(CONFIG_PINCTRL)
	return pinctrl_apply_state(config->pcfg, state);
#else
	ARG_UNUSED(config);
	ARG_UNUSED(state);
	return -ENOTSUP;
#endif
}

static bool sw_jtag_spi(const struct device *dev)
{
	const struct sw_config *config = dev->config;

	return config->spi != NULL && config->pcfg != NULL;
}

/* whole bytes through the SPI controller; TCK low on both mux switches */
static int sw_jtag_spi_shift(const struct device *dev, uint32_t nbytes,
			     const uint8_t *tdi, uint8_t *tdo)
{
	const struct sw_config *config = dev->config;
	struct sw_cfg_data *sw_data = dev->data;
	int ret, ret2;

	gpio_pin_set_dt(&config->clk, 0);
	ret = sw_pinctrl(config, PINCTRL_STATE_SPI);
	if (ret != 0) {
		return ret;
	}
	while (nbytes > 0U && ret == 0) {
		uint32_t n = MIN(nbytes, JTAG_SPI_CHUNK);
		struct spi_buf tx = {.buf = (void *)tdi, .len = n};
		struct spi_buf rx = {.buf = tdo, .len = n};
		struct spi_buf_set txs = {.buffers = &tx, .count = 1};
		struct spi_buf_set rxs = {.buffers = &rx, .count = 1};

		ret = spi_transceive(config->spi, &sw_data->spi_cfg[sw_data->spi_cfg_idx], &txs,
				     tdo != NULL ? &rxs : NULL);
		tdi += n;
		if (tdo != NULL) {
			tdo += n;
		}
		nbytes -= n;
	}
	/* the SPI clock idles low: the GPIO takes over at the same level */
	ret2 = sw_pinctrl(config, PINCTRL_STATE_DEFAULT);
	return ret != 0 ? ret : ret2;
}

static int sw_jtag_sequence(const struct device *dev, uint32_t count, bool tms,
			    const uint8_t *tdi, uint8_t *tdo)
{
	const struct sw_config *config = dev->config;
	struct sw_cfg_data *sw_data = dev->data;
	uint32_t done = 0U;
	unsigned int key;
	int ret;

	if (config->tdi.port == NULL || config->tdo.port == NULL) {
		return -ENOTSUP;
	}
	if (count == 0U) {
		return 0;
	}
	/* TMS is SWDIO: an output for the whole sequence */
	pin_swdio_out(dev, tms ? 1U : 0U);
	pin_swdio_out_enable(dev);

	if (sw_jtag_spi(dev) && count >= 8U) {
		done = count & ~7U;
		ret = sw_jtag_spi_shift(dev, done / 8U, tdi, tdo);
		if (ret != 0) {
			return ret;
		}
	}

	key = irq_lock();
	for (uint32_t i = done; i < count; i++) {
		uint32_t bit;

		gpio_pin_set_dt(&config->tdi, (tdi[i / 8U] >> (i % 8U)) & 1U);
		pin_swclk_clr(dev);
		pin_delay_asm(sw_data->clock_delay);
		bit = (uint32_t)gpio_pin_get_dt(&config->tdo) & 1U;
		pin_swclk_set(dev);
		pin_delay_asm(sw_data->clock_delay);
		if (tdo != NULL) {
			if ((i % 8U) == 0U) {
				tdo[i / 8U] = 0U;
			}
			tdo[i / 8U] |= (uint8_t)(bit << (i % 8U));
		}
	}
	irq_unlock(key);

	return 0;
}

static int sw_set_pins(const struct device *dev,
		       const uint8_t pins, const uint8_t value)
{
	const struct sw_config *config = dev->config;

	LOG_DBG("pins 0x%02x value 0x%02x", pins, value);

	if (pins & BIT(SWDP_SWCLK_PIN)) {
		if (value & BIT(SWDP_SWCLK_PIN)) {
			gpio_pin_set_dt(&config->clk, 1);
		} else {
			gpio_pin_set_dt(&config->clk, 0);
		}
	}

	if (config->tdi.port != NULL && (pins & BIT(SWDP_TDI_PIN)) != 0U) {
		gpio_pin_set_dt(&config->tdi, (value & BIT(SWDP_TDI_PIN)) != 0U ? 1 : 0);
	}

	if (config->dout_reg != NULL) {
		if (pins & BIT(SWDP_SWDIO_PIN)) {
			if (value & BIT(SWDP_SWDIO_PIN)) {
				gpio_pin_set_dt(&config->dout, 1);
			} else {
				gpio_pin_set_dt(&config->dout, 0);
			}
		}
	} else {
		if (pins & BIT(SWDP_SWDIO_PIN)) {
			if (value & BIT(SWDP_SWDIO_PIN)) {
				gpio_pin_set_dt(&config->dio, 1);
			} else {
				gpio_pin_set_dt(&config->dio, 0);
			}
		}
	}

	if (config->reset.port) {
		if (pins & BIT(SWDP_nRESET_PIN)) {
			/*
			 * CMSIS-DAP DAP_SWJ_Pins uses the raw electrical level
			 * for the nRESET pin: bit=0 means drive low (asserted),
			 * bit=1 means drive high (deasserted). Zephyr GPIO
			 * polarity (GPIO_ACTIVE_LOW) requires us to translate
			 * the raw level into the logical level — bit=0 maps to
			 * logical-active (asserted), bit=1 to logical-inactive
			 * (deasserted). Without this inversion, GPIO_ACTIVE_LOW
			 * overlays double-invert the runtime assert/deassert.
			 */
			if (value & BIT(SWDP_nRESET_PIN)) {
				gpio_pin_set_dt(&config->reset, 0);
			} else {
				gpio_pin_set_dt(&config->reset, 1);
			}
		}
	}

	return 0;
}

static int sw_get_pins(const struct device *dev, uint8_t *const state)
{
	const struct sw_config *config = dev->config;
	uint32_t val;

	*state = 0;

	if (config->reset.port) {
		/*
		 * Symmetric to sw_set_pins(): the CMSIS-DAP nRESET bit
		 * reports the raw electrical level. gpio_pin_get_dt()
		 * returns the polarity-aware logical level — invert so
		 * logical-active (asserted) reports bit=0 and
		 * logical-inactive (deasserted) reports bit=1.
		 */
		val = gpio_pin_get_dt(&config->reset);
		*state |= val ? 0 : BIT(SWDP_nRESET_PIN);
	}

	val = gpio_pin_get_dt(&config->dio);
	*state |= val ? BIT(SWDP_SWDIO_PIN) : 0;

	if (config->tdi.port != NULL) {
		val = gpio_pin_get_dt(&config->tdi);
		*state |= val ? BIT(SWDP_TDI_PIN) : 0;
	}

	if (config->tdo.port != NULL) {
		val = gpio_pin_get_dt(&config->tdo);
		*state |= val ? BIT(SWDP_TDO_PIN) : 0;
	}

	val = gpio_pin_get_dt(&config->clk);
	*state |= val ? BIT(SWDP_SWCLK_PIN) : 0;

	LOG_DBG("pins state 0x%02x", *state);

	return 0;
}

static int sw_set_clock(const struct device *dev, const uint32_t clock)
{
	const struct sw_config *config = dev->config;
	struct sw_cfg_data *sw_data = dev->data;
	uint32_t delay;

	if (clock >= MAX_SWJ_CLOCK(DELAY_FAST_CYCLES, config->port_write_cycles)) {
		sw_data->fast_clock = true;
		delay = 1U;
	} else {
		sw_data->fast_clock = false;

		delay = ((CPU_CLOCK / 2U) + (clock - 1U)) / clock;
		if (delay > config->port_write_cycles) {
			delay -= config->port_write_cycles;
			delay = (delay + (DELAY_SLOW_CYCLES - 1U)) / DELAY_SLOW_CYCLES;
		} else {
			delay = 1U;
		}
	}

	sw_data->clock_delay = delay;
	sw_data->spi_cfg[sw_data->spi_cfg_idx ^ 1U] = sw_data->spi_cfg[sw_data->spi_cfg_idx];
	sw_data->spi_cfg_idx ^= 1U;
	sw_data->spi_cfg[sw_data->spi_cfg_idx].frequency = clock;

	LOG_DBG("cpu_clock %d, delay %d", CPU_CLOCK, sw_data->clock_delay);

	return 0;
}

static int sw_configure(const struct device *dev,
			const uint8_t turnaround, const bool data_phase)
{
	struct sw_cfg_data *sw_data = dev->data;

	sw_data->turnaround = turnaround;
	sw_data->data_phase = data_phase;

	LOG_INF("turnaround %d, data_phase %d",
		sw_data->turnaround, sw_data->data_phase);

	return 0;
}

static int sw_port_on(const struct device *dev)
{
	const struct sw_config *config = dev->config;
	int ret;

	if (config->dnoe.port) {
		ret = gpio_pin_configure_dt(&config->dnoe, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	if (config->dout.port) {
		ret = gpio_pin_configure_dt(&config->dout, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	ret = gpio_pin_configure_dt(&config->dio, GPIO_INPUT);
	if (ret) {
		return ret;
	}
	if (config->tdi.port != NULL) {
		ret = gpio_pin_configure_dt(&config->tdi, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	if (config->tdo.port != NULL) {
		ret = gpio_pin_configure_dt(&config->tdo, GPIO_INPUT);
		if (ret) {
			return ret;
		}
	}

	if (config->noe.port) {
		ret = gpio_pin_configure_dt(&config->noe, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}
	}

	ret = gpio_pin_configure_dt(&config->clk, GPIO_INPUT | GPIO_OUTPUT_ACTIVE);
	if (ret == -ENOTSUP) {
		LOG_DBG("Fall back to CLK output only");
		ret = gpio_pin_configure_dt(&config->clk, GPIO_OUTPUT_ACTIVE);
	}

	if (ret) {
		LOG_ERR("Failed to configure CLK GPIO");
		return ret;
	}

	if (config->reset.port) {
		ret = gpio_pin_configure_dt(&config->reset,
					    GPIO_INPUT | GPIO_OUTPUT_INACTIVE);
		if (ret == -ENOTSUP) {
			LOG_DBG("Fall back to RESET output only");
			ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		}

		if (ret) {
			LOG_ERR("Failed to configure RESET GPIO");
			return ret;
		}
	}

	return 0;
}

static int sw_port_off(const struct device *dev)
{
	const struct sw_config *config = dev->config;
	int ret;
	const gpio_flags_t off_flags = IS_ENABLED(CONFIG_SWDP_BITBANG_NO_DISCONNECT) ?
		GPIO_INPUT : GPIO_DISCONNECTED;

	/* If there is a transceiver connected to IO, pins should always be driven. */
	if (config->dnoe.port) {
		ret = gpio_pin_configure_dt(&config->dnoe, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}

		if (config->dout.port) {
			ret = gpio_pin_configure_dt(&config->dout, GPIO_OUTPUT_ACTIVE);
			if (ret) {
				return ret;
			}
		}

		ret = gpio_pin_configure_dt(&config->dio, GPIO_INPUT);
		if (ret) {
			return ret;
		}
	} else {
		if (config->dout.port) {
			ret = gpio_pin_configure_dt(&config->dout, off_flags);
			if (ret) {
				return ret;
			}
		}

		ret = gpio_pin_configure_dt(&config->dio, off_flags);
		if (ret) {
			return ret;
		}
	}

	/* If there is a transceiver connected to CLK, pins should always be driven. */
	if (config->noe.port) {
		ret = gpio_pin_configure_dt(&config->noe, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			return ret;
		}

		ret = gpio_pin_configure_dt(&config->clk, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			return ret;
		}

	} else {
		ret = gpio_pin_configure_dt(&config->clk, off_flags);
		if (ret) {
			return ret;
		}
	}

	if (config->tdi.port != NULL) {
		ret = gpio_pin_configure_dt(&config->tdi, off_flags);
		if (ret) {
			return ret;
		}
	}

	if (config->reset.port) {
		ret = gpio_pin_configure_dt(&config->reset,
					    config->keep_reset_deast
						    ? GPIO_OUTPUT_INACTIVE
						    : off_flags);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

static int sw_gpio_init(const struct device *dev)
{
	const struct sw_config *config = dev->config;
	struct sw_cfg_data *sw_data = dev->data;
	int ret;

	if (config->pcfg != NULL) {
		/* GPIO functions; the SPI controller may have muxed its pads */
		ret = sw_pinctrl(config, PINCTRL_STATE_DEFAULT);
		if (ret) {
			return ret;
		}
	}
	if (config->spi != NULL && !device_is_ready(config->spi)) {
		return -ENODEV;
	}
	/* JTAG over SPI: mode 0, LSB first, native chip select (not muxed) */
	sw_data->spi_cfg[0].operation = SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) | SPI_TRANSFER_LSB;
	sw_data->spi_cfg[0].peripheral = 0U;
	sw_data->spi_cfg_idx = 0U;

	/* start with the port turned off */
	ret = sw_port_off(dev);
	if (ret) {
		return ret;
	}

	sw_data->turnaround = 1U;
	sw_data->data_phase = false;
	sw_data->fast_clock = false;
	sw_set_clock(dev, SWDP_DEFAULT_SWCLK_FREQUENCY);

	return 0;
}

static struct swdp_api swdp_bitbang_api = {
	.swdp_output_sequence = sw_output_sequence,
	.swdp_input_sequence = sw_input_sequence,
	.swdp_transfer	= sw_transfer,
	.swdp_set_pins	= sw_set_pins,
	.swdp_get_pins	= sw_get_pins,
	.swdp_set_clock	= sw_set_clock,
	.swdp_configure	= sw_configure,
	.swdp_port_on	= sw_port_on,
	.swdp_port_off	= sw_port_off,
	.swdp_jtag_sequence = sw_jtag_sequence,
};

#define SW_GPIOS_GET_REG(n, gpios)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, gpios),					\
		    (INT_TO_POINTER(DT_REG_ADDR(DT_PHANDLE(DT_DRV_INST(n), gpios)))),	\
		    (NULL))

#define SW_PINCTRL_DEFINE(n)								\
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, spi), (PINCTRL_DT_INST_DEFINE(n);), ())

#define SW_PINCTRL_GET(n)								\
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, spi),					\
		    (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define SW_DEVICE_DEFINE(n)								\
	BUILD_ASSERT((DT_INST_NODE_HAS_PROP(n, dout_gpios)) ==				\
		     (DT_INST_NODE_HAS_PROP(n, dnoe_gpios)),				\
		     "Either the dout-gpios or dnoe-gpios property is missing.");	\
	BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(n, jtag_spi) ||				\
		     DT_INST_PINCTRL_HAS_NAME(n, spi),					\
		     "jtag-spi needs the \"default\" and \"spi\" pinctrl states.");	\
											\
	SW_PINCTRL_DEFINE(n)								\
	static const struct sw_config sw_cfg_##n = {					\
		.clk = GPIO_DT_SPEC_INST_GET(n, clk_gpios),				\
		.tdi = GPIO_DT_SPEC_INST_GET_OR(n, tdi_gpios, {0}),			\
		.tdo = GPIO_DT_SPEC_INST_GET_OR(n, tdo_gpios, {0}),			\
		.spi = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, jtag_spi),			\
				   (DEVICE_DT_GET(DT_INST_PHANDLE(n, jtag_spi))), (NULL)),	\
		.pcfg = SW_PINCTRL_GET(n),						\
		.clk_reg = SW_GPIOS_GET_REG(n, clk_gpios),				\
		.dio = GPIO_DT_SPEC_INST_GET(n, dio_gpios),				\
		.dio_reg = SW_GPIOS_GET_REG(n, dio_gpios),				\
		.dout = GPIO_DT_SPEC_INST_GET_OR(n, dout_gpios, {0}),			\
		.dout_reg = SW_GPIOS_GET_REG(n, dout_gpios),				\
		.dnoe = GPIO_DT_SPEC_INST_GET_OR(n, dnoe_gpios, {0}),			\
		.dnoe_reg = SW_GPIOS_GET_REG(n, dnoe_gpios),				\
		.noe = GPIO_DT_SPEC_INST_GET_OR(n, noe_gpios, {0}),			\
		.reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),			\
		.keep_reset_deast = DT_INST_PROP(n, keep_reset_deasserted),             \
		.port_write_cycles = DT_INST_PROP(n, port_write_cycles),		\
	};										\
											\
	static struct sw_cfg_data sw_data_##n;						\
											\
	DEVICE_DT_INST_DEFINE(n, sw_gpio_init, NULL,					\
			      &sw_data_##n, &sw_cfg_##n,				\
			      POST_KERNEL, CONFIG_DP_DRIVER_INIT_PRIO,			\
			      &swdp_bitbang_api);

DT_INST_FOREACH_STATUS_OKAY(SW_DEVICE_DEFINE)
