/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 eMMC: Arasan SDHCI 5.1 with the Rockchip eMMC PHY in the GRF.
 * Polled, PIO through the buffer data port, 1/4/8-bit, legacy and high
 * speed SDR (52 MHz); no HS200/HS400 (tuning), no DDR, no DMA.
 *
 * Clocks: clk_xin (SCLK_EMMC) as the SDHCI base clock, set to the
 * highest GPLL division not above max-frequency (198 MHz) and written to
 * corecfg_baseclkfreq (GRF_EMMCCORE_CON0); the divider "0 = base" is not
 * usable on this core (Linux: SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN).
 *
 * PHY (GRF 0xf780): powered off around clock changes and on again above
 * 400 kHz with the DLL frequency range of the card clock, as U-Boot's
 * rockchip_sdhci.c does it.
 */

#define DT_DRV_COMPAT rockchip_rk3399_sdhci_5_1

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(sdhc_rk3399_emmc, CONFIG_SDHC_LOG_LEVEL);

/* standard SDHCI */
#define BLKSIZE      0x04U
#define BLKCNT       0x06U
#define ARG          0x08U
#define XFER_MODE    0x0cU
#define  XM_BLKCNT_EN   BIT(1)
#define  XM_AUTO_CMD12  (1U << 2)
#define  XM_READ        BIT(4)
#define  XM_MULTI       BIT(5)
#define CMD          0x0eU
#define  CMD_RESP_136   1U
#define  CMD_RESP_48    2U
#define  CMD_RESP_48B   3U
#define  CMD_CRC        BIT(3)
#define  CMD_INDEX      BIT(4)
#define  CMD_DATA       BIT(5)
#define  CMD_ABORT      (3U << 6)
#define RESP0        0x10U
#define BUFFER       0x20U
#define PRESENT      0x24U
#define  PS_CMD_INHIBIT BIT(0)
#define  PS_DAT_INHIBIT BIT(1)
#define  PS_DAT0        BIT(20)
#define HOST_CTRL1   0x28U
#define  HC1_4BIT       BIT(1)
#define  HC1_HS         BIT(2)
#define  HC1_8BIT       BIT(5)
#define POWER_CTRL   0x29U
#define  PWR_ON         BIT(0)
#define  PWR_330        (7U << 1)
#define  PWR_180        (5U << 1)
#define CLOCK_CTRL   0x2cU
#define  CLK_INT_EN     BIT(0)
#define  CLK_INT_STABLE BIT(1)
#define  CLK_SD_EN      BIT(2)
#define TIMEOUT_CTRL 0x2eU
#define SW_RESET     0x2fU
#define  RST_ALL        BIT(0)
#define  RST_CMD        BIT(1)
#define  RST_DAT        BIT(2)
#define INT_STATUS   0x30U   /* normal (16) | error (16) << 16 */
#define  INT_CMD_DONE   BIT(0)
#define  INT_XFER_DONE  BIT(1)
#define  INT_BUF_W      BIT(4)
#define  INT_BUF_R      BIT(5)
#define  INT_ERR        BIT(15)
#define  INT_CMD_TMO    BIT(16)
#define  INT_CMD_ERRS   (0xfU << 16)          /* timeout, crc, end bit, index */
#define  INT_DAT_TMO    BIT(20)
#define  INT_DAT_ERRS   (BIT(20) | BIT(21) | BIT(22) | BIT(24))
#define INT_ENABLE   0x34U
#define INT_SIGNAL   0x38U
#define HOST_CTRL2   0x3eU
#define CAPS         0x40U
#define  CAPS_8BIT      BIT(18)
#define  CAPS_330       BIT(24)
#define  CAPS_180       BIT(26)
#define HOST_VERSION 0xfeU

/* GRF */
#define GRF_EMMCCORE_CON0  0xf000U       /* [15:8] corecfg_baseclkfreq (MHz) */
#define GRF_EMMCCORE_CON11 0xf02cU       /* [7:0] corecfg_clockmultiplier */
#define GRF_EMMCPHY_CON0   0xf780U
#define GRF_EMMCPHY_CON2   0xf788U
#define GRF_EMMCPHY_CON6   0xf798U
#define GRF_EMMCPHY_STATUS 0xf7a0U
#define  PHY_CALDONE       BIT(6)
#define  PHY_DLLRDY        BIT(5)
#define HIWORD(mask, val)  (((uint32_t)(mask) << 16) | (val))

#define CMD_TIMEOUT_MS  500
#define DATA_TIMEOUT_MS 2000
#define F_MIN           400000U

struct emmc_config {
	DEVICE_MMIO_ROM;
	const struct device *clk_dev;
	clock_control_subsys_t xin;
	clock_control_subsys_t ahb;
	const struct device *grf;
	uint32_t max_freq;    /* base clock asked from the CRU */
	uint32_t max_bus_freq;
	uint8_t bus_width;
};

struct emmc_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
	uint32_t base_hz;
	uint32_t clock;
	bool phy_on;
	struct sdhc_io ios;
	struct sdhc_host_props props;
};

static inline uint32_t rd32(const struct device *d, uint32_t o) { return sys_read32(DEVICE_MMIO_GET(d) + o); }
static inline uint16_t rd16(const struct device *d, uint32_t o) { return sys_read16(DEVICE_MMIO_GET(d) + o); }
static inline uint8_t rd8(const struct device *d, uint32_t o) { return sys_read8(DEVICE_MMIO_GET(d) + o); }
static inline void wr32(const struct device *d, uint32_t o, uint32_t v) { sys_write32(v, DEVICE_MMIO_GET(d) + o); }
static inline void wr16(const struct device *d, uint32_t o, uint16_t v) { sys_write16(v, DEVICE_MMIO_GET(d) + o); }
static inline void wr8(const struct device *d, uint32_t o, uint8_t v) { sys_write8(v, DEVICE_MMIO_GET(d) + o); }

static void grf_wr(const struct device *dev, uint32_t off, uint32_t v)
{
	(void)syscon_write_reg(((const struct emmc_config *)dev->config)->grf, off, v);
}

static uint32_t grf_rd(const struct device *dev, uint32_t off)
{
	uint32_t v = 0;

	(void)syscon_read_reg(((const struct emmc_config *)dev->config)->grf, off, &v);
	return v;
}

static void phy_off(const struct device *dev)
{
	grf_wr(dev, GRF_EMMCPHY_CON6, HIWORD(BIT(0), 0));        /* pdb */
	grf_wr(dev, GRF_EMMCPHY_CON6, HIWORD(BIT(1), 0));        /* endll */
	((struct emmc_data *)dev->data)->phy_on = false;
}

static void phy_on(const struct device *dev, uint32_t clock)
{
	uint32_t freqsel;
	int64_t end;

	grf_wr(dev, GRF_EMMCPHY_CON6, HIWORD(7U << 4, 0));        /* drive 50 ohm */
	grf_wr(dev, GRF_EMMCPHY_CON0, HIWORD(BIT(11), BIT(11)));  /* otapdlyena */
	grf_wr(dev, GRF_EMMCPHY_CON0, HIWORD(0xfU << 7, 6U << 7)); /* otapdlysel */
	k_busy_wait(3);
	grf_wr(dev, GRF_EMMCPHY_CON6, HIWORD(BIT(0), BIT(0)));   /* power the I/O */
	k_busy_wait(500);
	if (!(grf_rd(dev, GRF_EMMCPHY_STATUS) & PHY_CALDONE)) {
		LOG_WRN("%s: PHY calibration not done", dev->name);
		return;
	}
	freqsel = clock < MHZ(75) ? 1U : clock < MHZ(125) ? 2U : clock < MHZ(175) ? 3U : 0U;
	grf_wr(dev, GRF_EMMCPHY_CON0, HIWORD(3U << 12, freqsel << 12));
	grf_wr(dev, GRF_EMMCPHY_CON6, HIWORD(BIT(1), BIT(1)));   /* DLL on */
	end = k_uptime_get() + 5;
	while (!(grf_rd(dev, GRF_EMMCPHY_STATUS) & PHY_DLLRDY)) {
		if (k_uptime_get() > end) {
			LOG_WRN("%s: PHY DLL not ready at %u Hz", dev->name, clock);
			break;
		}
	}
	((struct emmc_data *)dev->data)->phy_on = true;
}

static int sw_reset(const struct device *dev, uint8_t what)
{
	int64_t end = k_uptime_get() + 100;

	wr8(dev, SW_RESET, what);
	while (rd8(dev, SW_RESET) & what) {
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int set_clock(const struct device *dev, uint32_t hz)
{
	struct emmc_data *data = dev->data;
	uint32_t n, ctrl;
	int64_t end;

	if (data->phy_on && hz != data->clock) {
		phy_off(dev);
	}
	wr16(dev, CLOCK_CTRL, 0);
	data->clock = 0;
	if (hz == 0U) {
		return 0;
	}
	/* card clock = base / (2N), N >= 1 (N = 0 broken on this core) */
	n = MAX(DIV_ROUND_UP(data->base_hz, 2U * hz), 1U);
	if (n > 0x3ffU) {
		return -ERANGE;
	}
	ctrl = ((n & 0xffU) << 8) | (((n >> 8) & 0x3U) << 6) | CLK_INT_EN;
	wr16(dev, CLOCK_CTRL, ctrl);
	end = k_uptime_get() + 20;
	while (!(rd16(dev, CLOCK_CTRL) & CLK_INT_STABLE)) {
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
	}
	wr16(dev, CLOCK_CTRL, ctrl | CLK_SD_EN);
	data->clock = data->base_hz / (2U * n);
	if (data->clock > F_MIN) {
		phy_on(dev, data->clock);
	}
	LOG_DBG("%s: card clock %u Hz", dev->name, data->clock);
	return 0;
}

static int emmc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct emmc_config *cfg = dev->config;
	struct emmc_data *data = dev->data;
	uint8_t hc1;
	int ret = 0;

	if (ios->clock > data->props.f_max || (ios->clock != 0 && ios->clock < F_MIN)) {
		return -EINVAL;
	}
	switch (ios->timing) {
	case SDHC_TIMING_LEGACY:
	case SDHC_TIMING_HS:
	case SDHC_TIMING_SDR12:
	case SDHC_TIMING_SDR25:
		break;
	default:
		return -ENOTSUP;
	}
	if (ios->bus_width == SDHC_BUS_WIDTH8BIT && cfg->bus_width < 8U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (ios->power_mode != data->ios.power_mode) {
		uint32_t caps = rd32(dev, CAPS);

		wr8(dev, POWER_CTRL, ios->power_mode == SDHC_POWER_ON
					     ? (PWR_ON | ((caps & CAPS_330) ? PWR_330 : PWR_180))
					     : 0U);
		k_msleep(data->props.power_delay);
	}
	hc1 = rd8(dev, HOST_CTRL1) & ~(HC1_4BIT | HC1_8BIT | HC1_HS);
	if (ios->bus_width == SDHC_BUS_WIDTH8BIT) {
		hc1 |= HC1_8BIT;
	} else if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
		hc1 |= HC1_4BIT;
	}
	if (ios->timing == SDHC_TIMING_HS) {
		hc1 |= HC1_HS;
	}
	wr8(dev, HOST_CTRL1, hc1);
	if (ios->clock != data->clock) {
		ret = set_clock(dev, ios->clock);
	}
	if (ret == 0) {
		data->ios = *ios;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int wait_int(const struct device *dev, uint32_t mask, uint32_t errs, int ms, uint32_t *st)
{
	int64_t end = k_uptime_get() + ms;

	for (;;) {
		*st = rd32(dev, INT_STATUS);
		if (*st & errs) {
			return (*st & (INT_CMD_TMO | INT_DAT_TMO)) ? -ETIMEDOUT : -EIO;
		}
		if (*st & mask) {
			return 0;
		}
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
	}
}

static bool is_write(uint32_t op)
{
	return op == SD_WRITE_SINGLE_BLOCK || op == SD_WRITE_MULTIPLE_BLOCK;
}

static int pio(const struct device *dev, struct sdhc_data *d, bool write, int ms)
{
	uint8_t *buf = d->data;
	uint32_t st;
	int ret;

	for (uint32_t b = 0; b < d->blocks; b++) {
		ret = wait_int(dev, write ? INT_BUF_W : INT_BUF_R, INT_DAT_ERRS, ms, &st);
		if (ret != 0) {
			return ret;
		}
		wr32(dev, INT_STATUS, write ? INT_BUF_W : INT_BUF_R);
		for (uint32_t i = 0; i < d->block_size; i += 4U) {
			uint32_t w = 0;
			size_t c = MIN(4U, d->block_size - i);

			if (write) {
				memcpy(&w, buf + i, c);
				wr32(dev, BUFFER, sys_le32_to_cpu(w));
			} else {
				w = sys_cpu_to_le32(rd32(dev, BUFFER));
				memcpy(buf + i, &w, c);
			}
		}
		buf += d->block_size;
	}
	ret = wait_int(dev, INT_XFER_DONE, INT_DAT_ERRS, ms, &st);
	wr32(dev, INT_STATUS, INT_XFER_DONE);
	if (ret == 0) {
		d->bytes_xfered = d->block_size * d->blocks;
	}
	return ret;
}

static int emmc_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *d)
{
	struct emmc_data *data = dev->data;
	unsigned int tries = cmd->retries + 1U;
	uint32_t rt = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	uint16_t c = (uint16_t)((cmd->opcode & 0x3fU) << 8);
	uint16_t xm = 0;
	uint32_t st, inhibit;
	int64_t end;
	int ret = -EIO;

	switch (rt) {
	case SD_RSP_TYPE_NONE:
		break;
	case SD_RSP_TYPE_R2:
		c |= CMD_RESP_136 | CMD_CRC;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		c |= CMD_RESP_48;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		c |= CMD_RESP_48B | CMD_CRC | CMD_INDEX;
		break;
	default:
		c |= CMD_RESP_48 | CMD_CRC | CMD_INDEX;
		break;
	}
	if (cmd->opcode == SD_STOP_TRANSMISSION) {
		c |= CMD_ABORT;
	}
	if (d != NULL) {
		c |= CMD_DATA;
		xm = XM_BLKCNT_EN | (is_write(cmd->opcode) ? 0U : XM_READ);
		if (d->blocks > 1U) {
			xm |= XM_MULTI | XM_AUTO_CMD12;
		}
	}
	inhibit = PS_CMD_INHIBIT | ((d != NULL || (c & 3U) == CMD_RESP_48B) ? PS_DAT_INHIBIT : 0U);

	k_mutex_lock(&data->lock, K_FOREVER);
	while (tries-- > 0U) {
		end = k_uptime_get() + DATA_TIMEOUT_MS;
		while ((rd32(dev, PRESENT) & inhibit) && cmd->opcode != SD_STOP_TRANSMISSION) {
			if (k_uptime_get() > end) {
				break;
			}
		}
		wr32(dev, INT_STATUS, 0xffffffffU);
		if (d != NULL) {
			wr16(dev, BLKSIZE, (uint16_t)(d->block_size & 0xfffU));
			wr16(dev, BLKCNT, (uint16_t)d->blocks);
		}
		wr32(dev, ARG, cmd->arg);
		wr16(dev, XFER_MODE, xm);
		wr16(dev, CMD, c);

		ret = wait_int(dev, INT_CMD_DONE, INT_CMD_ERRS,
			       cmd->timeout_ms > 0 ? cmd->timeout_ms : CMD_TIMEOUT_MS, &st);
		wr32(dev, INT_STATUS, INT_CMD_DONE);
		if (ret != 0) {
			LOG_DBG("%s: CMD%u arg %08x: %08x", dev->name, cmd->opcode, cmd->arg, st);
			(void)sw_reset(dev, RST_CMD | RST_DAT);
			continue;
		}
		if ((c & 3U) == CMD_RESP_136) {
			/* SDHCI drops the CRC byte: shift back into the 128-bit layout */
			uint32_t r[4];

			for (int i = 0; i < 4; i++) {
				r[i] = rd32(dev, RESP0 + 4U * i);
			}
			cmd->response[0] = r[0] << 8;
			for (int i = 1; i < 4; i++) {
				cmd->response[i] = (r[i] << 8) | (r[i - 1] >> 24);
			}
		} else if (c & 3U) {
			cmd->response[0] = rd32(dev, RESP0);
		}
		if (d != NULL) {
			ret = pio(dev, d, is_write(cmd->opcode),
				  d->timeout_ms > 0 ? d->timeout_ms : DATA_TIMEOUT_MS);
		} else if ((c & 3U) == CMD_RESP_48B) {
			ret = wait_int(dev, INT_XFER_DONE, INT_DAT_ERRS,
				       cmd->timeout_ms > 0 ? cmd->timeout_ms : DATA_TIMEOUT_MS,
				       &st);
			wr32(dev, INT_STATUS, INT_XFER_DONE);
		}
		if (ret != 0) {
			(void)sw_reset(dev, RST_CMD | RST_DAT);
			continue;
		}
		break;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int emmc_get_card_present(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1; /* soldered / socketed module, no card detect */
}

static int emmc_card_busy(const struct device *dev)
{
	return (rd32(dev, PRESENT) & PS_DAT0) ? 0 : 1;
}

static int emmc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	*props = ((struct emmc_data *)dev->data)->props;
	return 0;
}

static int emmc_reset(const struct device *dev)
{
	struct emmc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_reset(dev, RST_CMD | RST_DAT);
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(sdhc, emmc_api) = {
	.reset = emmc_reset,
	.request = emmc_request,
	.set_io = emmc_set_io,
	.get_card_present = emmc_get_card_present,
	.card_busy = emmc_card_busy,
	.get_host_props = emmc_get_host_props,
};

static int emmc_init(const struct device *dev)
{
	const struct emmc_config *cfg = dev->config;
	struct emmc_data *data = dev->data;
	uint32_t caps;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	k_mutex_init(&data->lock);
	if (!device_is_ready(cfg->clk_dev) || !device_is_ready(cfg->grf)) {
		return -ENODEV;
	}
	(void)clock_control_on(cfg->clk_dev, cfg->ahb);
	(void)clock_control_set_rate(cfg->clk_dev, cfg->xin,
				     (clock_control_subsys_rate_t)(uintptr_t)cfg->max_freq);
	(void)clock_control_on(cfg->clk_dev, cfg->xin);
	ret = clock_control_get_rate(cfg->clk_dev, cfg->xin, &data->base_hz);
	if (ret != 0 || data->base_hz == 0U) {
		return -EIO;
	}
	/* whatever U-Boot SPL left: PHY off, then a clean controller */
	phy_off(dev);
	grf_wr(dev, GRF_EMMCCORE_CON0, HIWORD(0xffU << 8, (data->base_hz / MHZ(1)) << 8));
	grf_wr(dev, GRF_EMMCCORE_CON11, HIWORD(0xffU, 0));       /* no programmable clock */
	ret = sw_reset(dev, RST_ALL);
	if (ret != 0) {
		LOG_ERR("%s: reset stuck", dev->name);
		return ret;
	}
	wr32(dev, INT_ENABLE, 0xffffffffU & ~(INT_ERR | BIT(8)));   /* status only */
	wr32(dev, INT_SIGNAL, 0);                                  /* polled */
	wr8(dev, TIMEOUT_CTRL, 0xe);
	wr16(dev, HOST_CTRL2, 0);

	caps = rd32(dev, CAPS);
	data->props = (struct sdhc_host_props){
		.f_max = MIN(cfg->max_bus_freq, data->base_hz / 2U),
		.f_min = F_MIN,
		.power_delay = 10,
		.max_current_330 = 200,
		.max_current_180 = 200,
		.bus_4_bit_support = cfg->bus_width >= 4U,
		.is_spi = false,
	};
	data->props.host_caps.vol_330_support = (caps & CAPS_330) != 0U;
	data->props.host_caps.vol_180_support = (caps & CAPS_180) != 0U;
	data->props.host_caps.high_spd_support = 1;
	data->props.host_caps.bus_8_bit_support = cfg->bus_width == 8U && (caps & CAPS_8BIT);
	data->ios.power_mode = SDHC_POWER_OFF;

	LOG_INF("%s: SDHCI %u.%u, base %u Hz, caps %08x", dev->name,
		(rd16(dev, HOST_VERSION) & 0xffU) + 1U, rd16(dev, HOST_VERSION) >> 8,
		data->base_hz, caps);
	return set_clock(dev, F_MIN);
}

#define EMMC_INIT(n)                                                                               \
	static const struct emmc_config emmc_config_##n = {                                        \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, clk_xin)),                 \
		.xin = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, clk_xin, clkid),     \
		.ahb = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, clk_ahb, clkid),     \
		.grf = DEVICE_DT_GET(DT_INST_PHANDLE(n, rockchip_grf)),                            \
		.max_freq = DT_INST_PROP(n, max_frequency),                                        \
		.max_bus_freq = DT_INST_PROP(n, max_bus_freq),                                     \
		.bus_width = DT_INST_PROP(n, bus_width),                                           \
	};                                                                                         \
	static struct emmc_data emmc_data_##n;                                                     \
	DEVICE_DT_INST_DEFINE(n, emmc_init, NULL, &emmc_data_##n, &emmc_config_##n, POST_KERNEL,   \
			      CONFIG_SDHC_INIT_PRIORITY, &emmc_api);

DT_INST_FOREACH_STATUS_OKAY(EMMC_INIT)
