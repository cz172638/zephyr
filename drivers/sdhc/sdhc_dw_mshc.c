/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Synopsys DesignWare Mobile Storage Host (dw_mmc) in its Rockchip
 * integration (RK3399 SDMMC): one slot, polled, FIFO PIO, 3.3 V,
 * default and high speed (up to 50 MHz), 1/4-bit bus.
 *
 * Rockchip feeds the card interface unit through a fixed /2, so the CRU
 * clock is set to twice the card clock and CLKDIV stays 0 (bypass).
 */

#define DT_DRV_COMPAT rockchip_rk3399_dw_mshc

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(sdhc_dw_mshc, CONFIG_SDHC_LOG_LEVEL);

#define CTRL      0x000U
#define  CTRL_RESET      BIT(0)
#define  CTRL_FIFO_RESET BIT(1)
#define  CTRL_DMA_RESET  BIT(2)
#define  CTRL_INT_ENABLE BIT(4)
#define  CTRL_USE_IDMAC  BIT(25)
#define  CTRL_ALL_RESET  (CTRL_RESET | CTRL_FIFO_RESET | CTRL_DMA_RESET)
#define PWREN     0x004U
#define CLKDIV    0x008U
#define CLKSRC    0x00cU
#define CLKENA    0x010U
#define  CLKENA_ON       BIT(0)
#define TMOUT     0x014U
#define CTYPE     0x018U
#define  CTYPE_4BIT      BIT(0)
#define  CTYPE_8BIT      BIT(16)
#define BLKSIZ    0x01cU
#define BYTCNT    0x020U
#define INTMASK   0x024U
#define CMDARG    0x028U
#define CMD       0x02cU
#define  CMD_RESP_EXP    BIT(6)
#define  CMD_RESP_LONG   BIT(7)
#define  CMD_RESP_CRC    BIT(8)
#define  CMD_DAT_EXP     BIT(9)
#define  CMD_DAT_WR      BIT(10)
#define  CMD_SEND_STOP   BIT(12)
#define  CMD_PRV_DAT_WAIT BIT(13)
#define  CMD_STOP        BIT(14)
#define  CMD_INIT        BIT(15)
#define  CMD_UPD_CLK     BIT(21)
#define  CMD_USE_HOLD    BIT(29)
#define  CMD_START       BIT(31)
#define RESP0     0x030U
#define MINTSTS   0x040U
#define RINTSTS   0x044U
#define  INT_RE          BIT(1)
#define  INT_CMD_DONE    BIT(2)
#define  INT_DTO         BIT(3)
#define  INT_TXDR        BIT(4)
#define  INT_RXDR        BIT(5)
#define  INT_RCRC        BIT(6)
#define  INT_DCRC        BIT(7)
#define  INT_RTO         BIT(8)
#define  INT_DRTO        BIT(9)
#define  INT_HTO         BIT(10)
#define  INT_FRUN        BIT(11)
#define  INT_HLE         BIT(12)
#define  INT_SBE         BIT(13)
#define  INT_ACD         BIT(14)
#define  INT_EBE         BIT(15)
#define  INT_CMD_ERR     (INT_RE | INT_RCRC | INT_RTO | INT_HLE)
#define  INT_DATA_ERR    (INT_DCRC | INT_DRTO | INT_HTO | INT_FRUN | INT_SBE | INT_EBE)
#define STATUS    0x048U
#define  STATUS_DATA_BUSY BIT(9)
#define  STATUS_FCNT(s)  (((s) >> 17) & 0x1fffU)
#define FIFOTH    0x04cU
#define CDETECT   0x050U
#define WRTPRT    0x054U
#define VERID     0x06cU
#define HCON      0x070U
#define UHS_REG   0x074U
#define RST_N     0x078U
#define BMOD      0x080U

#define RK3399_GRF_SOC_CON7 0xe21cU
#define  GRF_FORCE_JTAG     BIT(12)

#define CMD_TIMEOUT_MS  500
#define BUSY_TIMEOUT_MS 2000

struct dw_mshc_config {
	DEVICE_MMIO_ROM;
	const struct device *clk_dev;
	clock_control_subsys_t biu;
	clock_control_subsys_t ciu;
	const struct device *grf;
	const struct pinctrl_dev_config *pcfg;
	uint32_t f_max;
	uint32_t f_min;
	uint32_t power_delay_ms;
	uint8_t bus_width;
	bool broken_cd;
};

struct dw_mshc_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
	uint32_t fifo_off;
	uint32_t fifo_depth;
	uint32_t clock;
	bool need_init;
	struct sdhc_io ios;
};

static inline uint32_t rd(const struct device *dev, uint32_t off)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + off);
}

static inline void wr(const struct device *dev, uint32_t off, uint32_t v)
{
	sys_write32(v, DEVICE_MMIO_GET(dev) + off);
}

static int wait_clear(const struct device *dev, uint32_t off, uint32_t mask, int ms)
{
	int64_t end = k_uptime_get() + ms;

	while (rd(dev, off) & mask) {
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
		k_yield();
	}
	return 0;
}

static int ctrl_reset(const struct device *dev, uint32_t what)
{
	wr(dev, CTRL, rd(dev, CTRL) | what);
	return wait_clear(dev, CTRL, what, 100);
}

/* clock register changes only take effect through a command */
static int clk_update(const struct device *dev)
{
	int64_t end = k_uptime_get() + CMD_TIMEOUT_MS;

	wr(dev, CMDARG, 0);
	wr(dev, CMD, CMD_START | CMD_UPD_CLK | CMD_PRV_DAT_WAIT);
	while (rd(dev, CMD) & CMD_START) {
		if (rd(dev, RINTSTS) & INT_HLE) {
			wr(dev, RINTSTS, INT_HLE);
			return -EIO;
		}
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int set_clock(const struct device *dev, uint32_t hz)
{
	const struct dw_mshc_config *cfg = dev->config;
	struct dw_mshc_data *data = dev->data;
	uint32_t rate = 0;
	int ret;

	wr(dev, CLKENA, 0);
	ret = clk_update(dev);
	if (ret != 0 || hz == 0U) {
		data->clock = 0;
		return ret;
	}
	ret = clock_control_set_rate(cfg->clk_dev, cfg->ciu,
				     (clock_control_subsys_rate_t)(uintptr_t)(hz * 2U));
	if (ret != 0) {
		return ret;
	}
	(void)clock_control_get_rate(cfg->clk_dev, cfg->ciu, &rate);
	wr(dev, CLKDIV, 0);
	wr(dev, CLKSRC, 0);
	ret = clk_update(dev);
	if (ret != 0) {
		return ret;
	}
	wr(dev, CLKENA, CLKENA_ON);
	ret = clk_update(dev);
	data->clock = rate / 2U;
	LOG_DBG("%s: card clock %u Hz (asked %u)", dev->name, data->clock, hz);
	return ret;
}

static int dw_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct dw_mshc_config *cfg = dev->config;
	struct dw_mshc_data *data = dev->data;
	int ret = 0;

	if (ios->signal_voltage == SD_VOL_1_8_V) {
		return -ENOTSUP; /* pads and vqmmc stay at 3.3 V */
	}
	if (ios->clock > cfg->f_max || (ios->clock != 0 && ios->clock < cfg->f_min)) {
		return -EINVAL;
	}
	if (ios->timing != SDHC_TIMING_LEGACY && ios->timing != SDHC_TIMING_HS &&
	    ios->timing != SDHC_TIMING_SDR12 && ios->timing != SDHC_TIMING_SDR25) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (ios->power_mode != data->ios.power_mode) {
		wr(dev, PWREN, ios->power_mode == SDHC_POWER_ON ? 1U : 0U);
		if (ios->power_mode == SDHC_POWER_ON) {
			data->need_init = true;
		}
		k_msleep(cfg->power_delay_ms);
	}
	switch (ios->bus_width) {
	case SDHC_BUS_WIDTH4BIT:
		wr(dev, CTYPE, CTYPE_4BIT);
		break;
	case SDHC_BUS_WIDTH8BIT:
		wr(dev, CTYPE, CTYPE_8BIT);
		break;
	default:
		wr(dev, CTYPE, 0);
		break;
	}
	if (ios->clock != data->clock) {
		ret = set_clock(dev, ios->clock);
	}
	if (ret == 0) {
		data->ios = *ios;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static void recover(const struct device *dev)
{
	struct dw_mshc_data *data = dev->data;
	uint32_t clock = data->clock;

	(void)ctrl_reset(dev, CTRL_ALL_RESET);
	wr(dev, RINTSTS, 0xffffffffU);
	data->clock = 0;
	if (clock != 0U) {
		(void)set_clock(dev, clock);
	}
}

static uint32_t cmd_flags(struct dw_mshc_data *data, const struct sdhc_command *cmd,
			  const struct sdhc_data *d)
{
	uint32_t f = CMD_START | CMD_USE_HOLD | (cmd->opcode & 0x3fU);

	switch (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		break;
	case SD_RSP_TYPE_R2:
		f |= CMD_RESP_EXP | CMD_RESP_LONG | CMD_RESP_CRC;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		f |= CMD_RESP_EXP;
		break;
	default:
		f |= CMD_RESP_EXP | CMD_RESP_CRC;
		break;
	}
	if (cmd->opcode == SD_STOP_TRANSMISSION) {
		f |= CMD_STOP;
	} else {
		f |= CMD_PRV_DAT_WAIT;
	}
	if (cmd->opcode == SD_GO_IDLE_STATE || data->need_init) {
		f |= CMD_INIT;
		data->need_init = false;
	}
	if (d != NULL) {
		f |= CMD_DAT_EXP;
		if (cmd->opcode == SD_WRITE_SINGLE_BLOCK || cmd->opcode == SD_WRITE_MULTIPLE_BLOCK) {
			f |= CMD_DAT_WR;
		}
		if (d->blocks > 1U && (cmd->opcode == SD_READ_MULTIPLE_BLOCK ||
				       cmd->opcode == SD_WRITE_MULTIPLE_BLOCK)) {
			f |= CMD_SEND_STOP;
		}
	}
	return f;
}

static int pio(const struct device *dev, struct sdhc_data *d, bool write, int timeout_ms)
{
	struct dw_mshc_data *data = dev->data;
	mm_reg_t fifo = DEVICE_MMIO_GET(dev) + data->fifo_off;
	size_t total = (size_t)d->block_size * d->blocks;
	uint8_t *buf = d->data;
	size_t done = 0;
	int64_t end = k_uptime_get() + timeout_ms;
	uint32_t st;

	for (;;) {
		st = rd(dev, RINTSTS);
		if (st & INT_DATA_ERR) {
			LOG_DBG("%s: data error %08x after %zu/%zu", dev->name, st, done, total);
			return (st & (INT_DRTO | INT_HTO)) ? -ETIMEDOUT : -EIO;
		}
		if (!write && (st & (INT_RXDR | INT_DTO))) {
			uint32_t n = STATUS_FCNT(rd(dev, STATUS));

			while (n-- > 0U && done < total) {
				uint32_t w = sys_read32(fifo);
				size_t c = MIN(sizeof(w), total - done);

				w = sys_cpu_to_le32(w);
				memcpy(buf + done, &w, c);
				done += c;
			}
			wr(dev, RINTSTS, INT_RXDR);
		}
		if (write && (st & INT_TXDR) && done < total) {
			uint32_t n = data->fifo_depth - STATUS_FCNT(rd(dev, STATUS));

			while (n-- > 0U && done < total) {
				uint32_t w = 0;
				size_t c = MIN(sizeof(w), total - done);

				memcpy(&w, buf + done, c);
				sys_write32(sys_le32_to_cpu(w), fifo);
				done += c;
			}
			wr(dev, RINTSTS, INT_TXDR);
		}
		if ((st & INT_DTO) && (write || done >= total ||
				       STATUS_FCNT(rd(dev, STATUS)) == 0U)) {
			wr(dev, RINTSTS, INT_DTO);
			break;
		}
		if (k_uptime_get() > end) {
			return -ETIMEDOUT;
		}
	}
	d->bytes_xfered = done;
	return done == total ? 0 : -EIO;
}

static int dw_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *d)
{
	struct dw_mshc_data *data = dev->data;
	uint32_t f, st;
	int64_t end;
	int ret = 0;
	unsigned int tries = cmd->retries + 1U;

	k_mutex_lock(&data->lock, K_FOREVER);
	while (tries-- > 0U) {
		if (cmd->opcode != SD_STOP_TRANSMISSION &&
		    wait_clear(dev, STATUS, STATUS_DATA_BUSY, BUSY_TIMEOUT_MS) != 0) {
			recover(dev);
			ret = -EBUSY;
			continue;
		}
		f = cmd_flags(data, cmd, d);
		if (d != NULL) {
			wr(dev, BLKSIZ, d->block_size);
			wr(dev, BYTCNT, d->block_size * d->blocks);
			(void)ctrl_reset(dev, CTRL_FIFO_RESET);
		}
		wr(dev, RINTSTS, 0xffffffffU);
		wr(dev, CMDARG, cmd->arg);
		wr(dev, CMD, f);

		end = k_uptime_get() + (cmd->timeout_ms > 0 ? cmd->timeout_ms : CMD_TIMEOUT_MS);
		do {
			st = rd(dev, RINTSTS);
			if (k_uptime_get() > end) {
				break;
			}
		} while (!(st & INT_CMD_DONE));

		if (!(st & INT_CMD_DONE) || (st & INT_RTO)) {
			ret = -ETIMEDOUT;
		} else if (st & INT_CMD_ERR) {
			ret = -EIO;
		} else {
			ret = 0;
		}
		wr(dev, RINTSTS, INT_CMD_DONE | INT_CMD_ERR);
		if (ret != 0) {
			LOG_DBG("%s: CMD%u arg %08x: %08x", dev->name, cmd->opcode, cmd->arg, st);
			if (d != NULL) {
				recover(dev);
			}
			continue;
		}

		if (f & CMD_RESP_LONG) {
			for (int i = 0; i < 4; i++) {
				cmd->response[i] = rd(dev, RESP0 + 4U * i);
			}
		} else if (f & CMD_RESP_EXP) {
			cmd->response[0] = rd(dev, RESP0);
		}

		if (d != NULL) {
			ret = pio(dev, d, (f & CMD_DAT_WR) != 0U,
				  d->timeout_ms > 0 ? d->timeout_ms : BUSY_TIMEOUT_MS);
			if (ret == 0 && (f & CMD_SEND_STOP)) {
				end = k_uptime_get() + CMD_TIMEOUT_MS;
				while (!(rd(dev, RINTSTS) & INT_ACD) && k_uptime_get() <= end) {
				}
			}
			if (ret == 0 && (f & CMD_DAT_WR)) {
				ret = wait_clear(dev, STATUS, STATUS_DATA_BUSY, BUSY_TIMEOUT_MS);
			}
			if (ret != 0) {
				recover(dev);
				continue;
			}
		}
		if ((cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) == SD_RSP_TYPE_R1b ||
		    (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) == SD_RSP_TYPE_R5b) {
			ret = wait_clear(dev, STATUS, STATUS_DATA_BUSY,
					 cmd->timeout_ms > 0 ? cmd->timeout_ms : BUSY_TIMEOUT_MS);
		}
		break;
	}
	k_mutex_unlock(&data->lock);
	return ret;
}

static int dw_get_card_present(const struct device *dev)
{
	const struct dw_mshc_config *cfg = dev->config;

	return cfg->broken_cd ? 1 : ((rd(dev, CDETECT) & BIT(0)) == 0U);
}

static int dw_card_busy(const struct device *dev)
{
	return (rd(dev, STATUS) & STATUS_DATA_BUSY) ? 1 : 0;
}

static int dw_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct dw_mshc_config *cfg = dev->config;

	memset(props, 0, sizeof(*props));
	props->f_max = cfg->f_max;
	props->f_min = cfg->f_min;
	props->power_delay = cfg->power_delay_ms;
	props->host_caps.vol_330_support = 1;
	props->host_caps.high_spd_support = cfg->f_max > 25000000U;
	props->host_caps.bus_8_bit_support = cfg->bus_width == 8U;
	props->bus_4_bit_support = cfg->bus_width >= 4U;
	props->max_current_330 = 800;
	return 0;
}

static int dw_reset(const struct device *dev)
{
	struct dw_mshc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = ctrl_reset(dev, CTRL_ALL_RESET);
	wr(dev, RINTSTS, 0xffffffffU);
	k_mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_API(sdhc, dw_mshc_api) = {
	.reset = dw_reset,
	.request = dw_request,
	.set_io = dw_set_io,
	.get_card_present = dw_get_card_present,
	.card_busy = dw_card_busy,
	.get_host_props = dw_get_host_props,
};

static int dw_mshc_init(const struct device *dev)
{
	const struct dw_mshc_config *cfg = dev->config;
	struct dw_mshc_data *data = dev->data;
	uint32_t ver, th;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	k_mutex_init(&data->lock);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0 && ret != -ENOENT) {
		return ret;
	}
	/* the SDMMC pins double as JTAG while this is set (reset default) */
	if (cfg->grf != NULL) {
		if (!device_is_ready(cfg->grf)) {
			return -ENODEV;
		}
		(void)syscon_write_reg(cfg->grf, RK3399_GRF_SOC_CON7, GRF_FORCE_JTAG << 16);
	}
	if (!device_is_ready(cfg->clk_dev)) {
		return -ENODEV;
	}
	(void)clock_control_on(cfg->clk_dev, cfg->biu);
	(void)clock_control_on(cfg->clk_dev, cfg->ciu);

	ver = rd(dev, VERID) & 0xffffU;
	data->fifo_off = ver < 0x240aU ? 0x100U : 0x200U;
	ret = ctrl_reset(dev, CTRL_ALL_RESET);
	if (ret != 0) {
		LOG_ERR("%s: reset stuck", dev->name);
		return ret;
	}
	/* polled: interrupts stay masked, raw status is enough */
	wr(dev, INTMASK, 0);
	wr(dev, RINTSTS, 0xffffffffU);
	wr(dev, CTRL, rd(dev, CTRL) & ~(CTRL_INT_ENABLE | CTRL_USE_IDMAC));
	wr(dev, BMOD, 0);
	wr(dev, TMOUT, 0xffffffffU);
	wr(dev, UHS_REG, 0);
	wr(dev, CTYPE, 0);

	th = rd(dev, FIFOTH);
	data->fifo_depth = ((th >> 16) & 0xfffU) + 1U;
	wr(dev, FIFOTH, (2U << 28) | ((data->fifo_depth / 2U - 1U) << 16) |
			       (data->fifo_depth / 2U));

	wr(dev, PWREN, 1);
	data->ios.power_mode = SDHC_POWER_ON;
	data->need_init = true;
	ret = set_clock(dev, cfg->f_min);
	if (ret != 0) {
		LOG_ERR("%s: clock: %d", dev->name, ret);
		return ret;
	}
	LOG_INF("%s: dw_mmc %04x, fifo %u words at 0x%x, card %s", dev->name, ver,
		data->fifo_depth, data->fifo_off, dw_get_card_present(dev) ? "present" : "absent");
	return 0;
}

#define DW_MSHC_INIT(n)                                                                            \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct dw_mshc_config dw_mshc_config_##n = {                                  \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, biu)),                     \
		.biu = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, biu, clkid),         \
		.ciu = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, ciu, clkid),         \
		.grf = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, rockchip_grf)),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.f_max = DT_INST_PROP(n, max_bus_freq),                                            \
		.f_min = DT_INST_PROP(n, min_bus_freq),                                            \
		.power_delay_ms = DT_INST_PROP(n, power_delay_ms),                                 \
		.bus_width = DT_INST_PROP(n, bus_width),                                           \
		.broken_cd = DT_INST_PROP(n, broken_cd),                                           \
	};                                                                                         \
	static struct dw_mshc_data dw_mshc_data_##n;                                               \
	DEVICE_DT_INST_DEFINE(n, dw_mshc_init, NULL, &dw_mshc_data_##n, &dw_mshc_config_##n,       \
			      POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, &dw_mshc_api);

DT_INST_FOREACH_STATUS_OKAY(DW_MSHC_INIT)
