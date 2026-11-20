/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip I2C controller (RK3399 "rk3x", v1 timing), polled.
 *
 * Transfers go out in chunks of up to 32 bytes: a write fills the eight
 * TXDATA words (the first byte of a message being the address) and sets
 * MTXCNT; a read sets MRXCNT and takes the RXDATA words. A read starts in
 * "register TX" mode, where the controller itself sends the address from
 * MRXADDR, optionally followed by up to three register address bytes from
 * MRXRADDR: a write of up to three bytes followed by a read becomes one
 * such transaction.
 */

#define DT_DRV_COMPAT rockchip_rk3399_i2c

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "i2c-priv.h"

LOG_MODULE_REGISTER(i2c_rockchip, CONFIG_I2C_LOG_LEVEL);

#define I2C_CON      0x000U
#define I2C_CLKDIV   0x004U /* [15:0] SCL low, [31:16] SCL high, in 8 input clocks - 1 */
#define I2C_MRXADDR  0x008U
#define I2C_MRXRADDR 0x00cU
#define I2C_MTXCNT   0x010U
#define I2C_MRXCNT   0x014U
#define I2C_IEN      0x018U
#define I2C_IPD      0x01cU /* write 1 to clear */
#define I2C_TXDATA   0x100U
#define I2C_RXDATA   0x200U

#define CON_EN       BIT(0)
#define CON_MOD_TX   (0U << 1)
#define CON_MOD_RTX  (1U << 1) /* send MRXADDR (+ MRXRADDR), restart, receive */
#define CON_MOD_RX   (2U << 1)
#define CON_MOD_MASK (3U << 1)
#define CON_START    BIT(3)
#define CON_STOP     BIT(4)
#define CON_LASTACK  BIT(5) /* NACK the last byte of the chunk */
#define CON_ACTACK   BIT(6) /* stop the transfer on a NACK */

#define ADDR_VALID(n) BIT(24 + (n)) /* byte n of MRXADDR / MRXRADDR is sent */

#define IPD_MBTF  BIT(2)
#define IPD_MBRF  BIT(3)
#define IPD_START BIT(4)
#define IPD_STOP  BIT(5)
#define IPD_NAK   BIT(6)
#define IPD_ALL   0x7fU

#define CHUNK 32U

struct i2c_rk_config {
	DEVICE_MMIO_ROM;
	const struct device *clk;
	clock_control_subsys_t sclk;
	clock_control_subsys_t pclk;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
};

struct i2c_rk_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
	uint32_t clk_hz;
	uint32_t dev_config;
};

static inline uint32_t rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

static inline void wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, DEVICE_MMIO_GET(dev) + off);
}

/* wait for any of bits in IPD, then clear them; NAK ends the wait too */
static int wait_ipd(const struct device *dev, uint32_t bits)
{
	int64_t end = k_uptime_get() + 20; /* a 32-byte chunk at 100 kHz takes ~3 ms */
	uint32_t ipd;

	do {
		ipd = rd(dev, I2C_IPD);
		if ((ipd & IPD_NAK) != 0U) {
			wr(dev, I2C_IPD, IPD_ALL);
			return -EIO;
		}
		if ((ipd & bits) != 0U) {
			wr(dev, I2C_IPD, ipd & bits);
			return 0;
		}
		k_yield();
	} while (k_uptime_get() < end);
	return -ETIMEDOUT;
}

static int bus_start(const struct device *dev, uint32_t mode)
{
	wr(dev, I2C_IPD, IPD_ALL);
	wr(dev, I2C_CON, CON_EN | mode | CON_START | CON_ACTACK);
	if (wait_ipd(dev, IPD_START) != 0) {
		return -EIO;
	}
	wr(dev, I2C_CON, rd(dev, I2C_CON) & ~CON_START);
	return 0;
}

static void bus_stop(const struct device *dev)
{
	wr(dev, I2C_IPD, IPD_ALL);
	wr(dev, I2C_CON, (rd(dev, I2C_CON) & ~CON_START) | CON_STOP);
	(void)wait_ipd(dev, IPD_STOP);
	wr(dev, I2C_CON, 0U);
}

/* first: the address byte goes ahead of the data of the first chunk */
static int do_write(const struct device *dev, uint16_t addr, const uint8_t *buf, uint32_t len)
{
	uint32_t pos = 0;
	bool first = true;

	while (first || pos < len) {
		uint32_t word = 0, n = 0;

		for (uint32_t i = 0; i < CHUNK && (first || pos < len); i++) {
			uint8_t b;

			if (first) {
				b = (uint8_t)(addr << 1);
				first = false;
			} else {
				b = buf[pos++];
			}
			word |= (uint32_t)b << (8U * (n % 4U));
			n++;
			if (n % 4U == 0U) {
				wr(dev, I2C_TXDATA + n - 4U, word);
				word = 0;
			}
		}
		if (n % 4U != 0U) {
			wr(dev, I2C_TXDATA + (n & ~3U), word);
		}
		wr(dev, I2C_MTXCNT, n);
		if (wait_ipd(dev, IPD_MBTF) != 0) {
			return -EIO;
		}
	}
	return 0;
}

static int do_read(const struct device *dev, uint8_t *buf, uint32_t len)
{
	uint32_t pos = 0;

	while (pos < len) {
		uint32_t n = MIN(len - pos, CHUNK);
		uint32_t con = rd(dev, I2C_CON) & ~CON_START;

		/* NACK only the last byte of the message */
		con = (pos + n == len) ? (con | CON_LASTACK) : (con & ~CON_LASTACK);
		if (pos != 0U) {
			con = (con & ~CON_MOD_MASK) | CON_MOD_RX;
		}
		wr(dev, I2C_CON, con);
		wr(dev, I2C_MRXCNT, n);
		if (wait_ipd(dev, IPD_MBRF) != 0) {
			return -EIO;
		}
		for (uint32_t i = 0; i < n; i += 4U) {
			uint32_t word = rd(dev, I2C_RXDATA + i);

			for (uint32_t j = 0; j < 4U && i + j < n; j++) {
				buf[pos + i + j] = (uint8_t)(word >> (8U * j));
			}
		}
		pos += n;
	}
	return 0;
}

static int i2c_rk_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num,
			   uint16_t addr)
{
	struct i2c_rk_data *d = dev->data;
	int ret = 0;

	if (num == 0U) {
		return 0;
	}
	if ((addr & ~0x7fU) != 0U || (d->dev_config & I2C_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}
	k_mutex_lock(&d->lock, K_FOREVER);
	for (uint8_t i = 0; i < num && ret == 0; i++) {
		struct i2c_msg *m = &msgs[i];
		bool read = (m->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;

		if (!read && m->len <= 3U && i + 1U < num &&
		    (msgs[i + 1].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			/* register read: the controller sends address and register itself */
			uint32_t raddr = 0;

			for (uint32_t j = 0; j < m->len; j++) {
				raddr |= ((uint32_t)m->buf[j] << (8U * j)) | ADDR_VALID(j);
			}
			wr(dev, I2C_MRXADDR, ((uint32_t)addr << 1) | 1U | ADDR_VALID(0));
			wr(dev, I2C_MRXRADDR, raddr);
			ret = bus_start(dev, CON_MOD_RTX);
			i++;
			m = &msgs[i];
			if (ret == 0) {
				ret = do_read(dev, m->buf, m->len);
			}
		} else if (read) {
			wr(dev, I2C_MRXADDR, ((uint32_t)addr << 1) | 1U | ADDR_VALID(0));
			wr(dev, I2C_MRXRADDR, 0U);
			ret = bus_start(dev, CON_MOD_RTX);
			if (ret == 0) {
				ret = do_read(dev, m->buf, m->len);
			}
		} else {
			ret = bus_start(dev, CON_MOD_TX);
			if (ret == 0) {
				ret = do_write(dev, addr, m->buf, m->len);
			}
		}
		if (ret != 0 || (m->flags & I2C_MSG_STOP) != 0U || i + 1U == num) {
			bus_stop(dev);
		}
	}
	k_mutex_unlock(&d->lock);
	return ret;
}

/*
 * SCL high and low are 8 * (div + 1) input clocks each. Low gets the
 * larger share, as the specification wants a longer minimum low time;
 * start/stop setup and data hold stay at the reset tuning, which meets
 * standard and fast mode with these divisors.
 */
static int i2c_rk_set_rate(const struct device *dev, uint32_t hz)
{
	struct i2c_rk_data *d = dev->data;
	uint32_t total, low, high, low_ns, high_ns;

	if (hz == 0U || d->clk_hz == 0U) {
		return -EINVAL;
	}
	if (hz <= I2C_BITRATE_STANDARD) {
		low_ns = 4700U + 1000U; /* tLOW min + rise time */
		high_ns = 4000U;
	} else {
		low_ns = 1300U + 300U;
		high_ns = 600U;
	}
	total = DIV_ROUND_UP(d->clk_hz, 8U * hz);
	low = DIV_ROUND_UP((uint64_t)d->clk_hz * low_ns, 8ULL * NSEC_PER_SEC);
	high = DIV_ROUND_UP((uint64_t)d->clk_hz * high_ns, 8ULL * NSEC_PER_SEC);
	if (low + high < total) {
		low += (total - low - high + 1U) / 2U;
		high = total - low;
	}
	if (low == 0U || high == 0U || low > 0x10000U || high > 0x10000U) {
		return -ERANGE;
	}
	wr(dev, I2C_CLKDIV, ((high - 1U) << 16) | (low - 1U));
	LOG_DBG("%s: %u Hz input, SCL %u Hz (low %u high %u)", dev->name, d->clk_hz,
		d->clk_hz / (8U * (low + high)), low, high);
	return 0;
}

static int i2c_rk_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_rk_data *d = dev->data;
	uint32_t hz;
	int ret;

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		hz = I2C_BITRATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		hz = I2C_BITRATE_FAST;
		break;
	default:
		return -ENOTSUP;
	}
	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}
	k_mutex_lock(&d->lock, K_FOREVER);
	ret = i2c_rk_set_rate(dev, hz);
	if (ret == 0) {
		d->dev_config = dev_config;
	}
	k_mutex_unlock(&d->lock);
	return ret;
}

static int i2c_rk_get_config(const struct device *dev, uint32_t *dev_config)
{
	*dev_config = ((struct i2c_rk_data *)dev->data)->dev_config;
	return 0;
}

static int i2c_rk_init(const struct device *dev)
{
	const struct i2c_rk_config *cfg = dev->config;
	struct i2c_rk_data *d = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	k_mutex_init(&d->lock);
	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret != 0) {
			return ret;
		}
	}
	if (cfg->clk != NULL) {
		if (!device_is_ready(cfg->clk)) {
			return -ENODEV;
		}
		(void)clock_control_on(cfg->clk, cfg->pclk);
		(void)clock_control_on(cfg->clk, cfg->sclk);
		/* the controller is specified up to 200 MHz input */
		(void)clock_control_set_rate(cfg->clk, cfg->sclk,
					     (clock_control_subsys_rate_t)MHZ(200));
		ret = clock_control_get_rate(cfg->clk, cfg->sclk, &d->clk_hz);
		if (ret != 0) {
			LOG_ERR("%s: input clock rate: %d", dev->name, ret);
			return ret;
		}
	}
	wr(dev, I2C_CON, 0U);
	wr(dev, I2C_IEN, 0U);
	wr(dev, I2C_IPD, IPD_ALL);
	return i2c_rk_configure(dev, I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate));
}

static DEVICE_API(i2c, i2c_rk_api) = {
	.configure = i2c_rk_configure,
	.get_config = i2c_rk_get_config,
	.transfer = i2c_rk_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#define I2C_RK_INIT(n)                                                                             \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct i2c_rk_config i2c_rk_config_##n = {                                    \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR_BY_NAME(n, i2c)),                 \
		.sclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, i2c, clkid),           \
		.pclk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, clkid),          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
	};                                                                                         \
	static struct i2c_rk_data i2c_rk_data_##n;                                                 \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_rk_init, NULL, &i2c_rk_data_##n, &i2c_rk_config_##n,      \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, &i2c_rk_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_RK_INIT)
