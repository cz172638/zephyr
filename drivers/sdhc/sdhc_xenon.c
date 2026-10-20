/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Xenon SDHC as in the Armada 3700: a standard SDHCI v3 slot with
 * Xenon system registers and an eMMC PHY, after U-Boot xenon_sdhci.c and
 * Linux sdhci-xenon*.c. Polled, PIO only (no interrupts, no DMA).
 *
 * The Armada 3700 has a second register with the IO pad voltage of the
 * slot (0: 3.3 V, 1: 1.8 V). The SD slot of the ESPRESSObin switches its
 * IO voltage with a GPIO regulator (vqmmc-supply) and has the card detect
 * on a GPIO (cd-gpios).
 */

#define DT_DRV_COMPAT marvell_armada_3700_sdhci

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/kernel.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sdhc_xenon, CONFIG_SDHC_LOG_LEVEL);

/* SDHCI */
#define SDHCI_BLOCK_SIZE      0x04
#define SDHCI_BLOCK_COUNT     0x06
#define SDHCI_ARGUMENT        0x08
#define SDHCI_TRANSFER_MODE   0x0c
#define SDHCI_TRNS_BLK_CNT_EN BIT(1)
#define SDHCI_TRNS_AUTO_CMD12 BIT(2)
#define SDHCI_TRNS_READ       BIT(4)
#define SDHCI_TRNS_MULTI      BIT(5)
#define SDHCI_COMMAND         0x0e
#define SDHCI_CMD_RESP_NONE   0x00
#define SDHCI_CMD_RESP_136    0x01
#define SDHCI_CMD_RESP_48     0x02
#define SDHCI_CMD_RESP_48_BSY 0x03
#define SDHCI_CMD_CRC         BIT(3)
#define SDHCI_CMD_INDEX       BIT(4)
#define SDHCI_CMD_DATA        BIT(5)
#define SDHCI_RESPONSE        0x10
#define SDHCI_BUFFER          0x20
#define SDHCI_PRESENT_STATE   0x24
#define SDHCI_CMD_INHIBIT     BIT(0)
#define SDHCI_DATA_INHIBIT    BIT(1)
#define SDHCI_CARD_PRESENT    BIT(16)
#define SDHCI_DATA_0_LVL      BIT(20)
#define SDHCI_HOST_CONTROL    0x28
#define SDHCI_CTRL_4BITBUS    BIT(1)
#define SDHCI_CTRL_HISPD      BIT(2)
#define SDHCI_CTRL_8BITBUS    BIT(5)
#define SDHCI_POWER_CONTROL   0x29
#define SDHCI_POWER_ON        BIT(0)
#define SDHCI_POWER_180       0x0a
#define SDHCI_POWER_330       0x0e
#define SDHCI_CLOCK_CONTROL   0x2c
#define SDHCI_CLOCK_INT_EN    BIT(0)
#define SDHCI_CLOCK_INT_STABLE BIT(1)
#define SDHCI_CLOCK_CARD_EN   BIT(2)
#define SDHCI_TIMEOUT_CONTROL 0x2e
#define SDHCI_SOFTWARE_RESET  0x2f
#define SDHCI_RESET_ALL       BIT(0)
#define SDHCI_RESET_CMD       BIT(1)
#define SDHCI_RESET_DATA      BIT(2)
#define SDHCI_INT_STATUS      0x30
#define SDHCI_INT_RESPONSE    BIT(0)
#define SDHCI_INT_DATA_END    BIT(1)
#define SDHCI_INT_SPACE_AVAIL BIT(4)
#define SDHCI_INT_DATA_AVAIL  BIT(5)
#define SDHCI_INT_ERROR       BIT(15)
#define SDHCI_INT_ERROR_MASK  GENMASK(31, 16)
#define SDHCI_INT_ENABLE      0x34
#define SDHCI_SIGNAL_ENABLE   0x38
#define SDHCI_HOST_CONTROL2   0x3e
#define SDHCI_CTRL_UHS_MASK   GENMASK(2, 0)
#define SDHCI_CTRL_VDD_180    BIT(3)
#define SDHCI_CAPABILITIES    0x40
#define SDHCI_CAPABILITIES_1  0x44
#define SDHCI_HOST_VERSION    0xfe

/* Xenon */
#define XENON_SYS_OP_CTRL          0x108
#define XENON_AUTO_CLKGATE_DISABLE BIT(20)
#define XENON_SLOT_ENABLE(s)       BIT(s)
#define XENON_SYS_EXT_OP_CTRL      0x10c
#define XENON_MASK_CMD_CONFLICT    BIT(8)
#define XENON_SLOT_EMMC_CTRL       0x130
#define XENON_EMMC_VCCQ_MASK       GENMASK(1, 0)
#define XENON_EMMC_VCCQ_3_3V       0x3
#define XENON_SLOT_RETUNING_CTRL   0x144
#define XENON_RETUNING_COMPATIBLE  BIT(0)
#define XENON_RETUNE_EVT_INTSIG    BIT(12)
#define XENON_PHY_TIMING_ADJUST    0x170
#define XENON_OUTPUT_QSN_PHASE     BIT(17)
#define XENON_SAMPL_INV_QSP_PHASE  BIT(18)
#define XENON_PHY_SLOW_MODE        BIT(29)
#define XENON_PHY_INITIALIZATION   BIT(31)
#define XENON_PHY_FUNC_CONTROL     0x174
#define XENON_DQ_DDR_MODE          GENMASK(15, 8)
#define XENON_CMD_DDR_MODE         BIT(16)
#define XENON_PHY_PAD_CONTROL      0x178
#define XENON_FC_DQ_RECEN          BIT(24)
#define XENON_FC_CMD_RECEN         BIT(25)
#define XENON_FC_QSP_RECEN         BIT(26)
#define XENON_OEN_QSN              BIT(28)
#define XENON_AUTO_RECEN_CTRL      BIT(30)
#define XENON_PHY_PAD_CONTROL1     0x17c
#define XENON_FC_CMD_PD            BIT(8)
#define XENON_FC_DQ_PD             GENMASK(7, 0)
#define XENON_FC_CMD_PU            BIT(24)
#define XENON_FC_DQ_PU             GENMASK(23, 16)

#define XENON_SLOT 0

/* Armada 3700 pad control */
#define A3700_PAD_3_3V 0x0
#define A3700_PAD_1_8V 0x1

#define POLL_US      10
#define CMD_TIMEOUT  K_MSEC(1000)

struct xenon_config {
	DEVICE_MMIO_NAMED_ROM(base);
	DEVICE_MMIO_NAMED_ROM(pad);
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec cd_gpio;
	const struct device *vqmmc;
	uint32_t max_bus_freq;
	uint32_t min_bus_freq;
	uint32_t power_delay_ms;
	uint8_t bus_width;
	bool non_removable;
	bool pad_fixed_1v8;
};

struct xenon_data {
	DEVICE_MMIO_NAMED_RAM(base);
	DEVICE_MMIO_NAMED_RAM(pad);
	struct k_mutex lock;
	struct sdhc_host_props props;
	struct sdhc_io ios;
	uint32_t base_clk;
};

#define DEV_CFG(dev)  ((const struct xenon_config *)(dev)->config)
#define DEV_DATA(dev) ((struct xenon_data *)(dev)->data)
#define BASE(dev)     DEVICE_MMIO_NAMED_GET(dev, base)

static inline uint32_t rd32(const struct device *dev, uint32_t reg)
{
	return sys_read32(BASE(dev) + reg);
}

static inline uint16_t rd16(const struct device *dev, uint32_t reg)
{
	return sys_read16(BASE(dev) + reg);
}

static inline uint8_t rd8(const struct device *dev, uint32_t reg)
{
	return sys_read8(BASE(dev) + reg);
}

static inline void wr32(const struct device *dev, uint32_t reg, uint32_t v)
{
	sys_write32(v, BASE(dev) + reg);
}

static inline void wr16(const struct device *dev, uint32_t reg, uint16_t v)
{
	sys_write16(v, BASE(dev) + reg);
}

static inline void wr8(const struct device *dev, uint32_t reg, uint8_t v)
{
	sys_write8(v, BASE(dev) + reg);
}

static inline void set32(const struct device *dev, uint32_t reg, uint32_t mask, uint32_t v)
{
	wr32(dev, reg, (rd32(dev, reg) & ~mask) | v);
}

/* Poll reg until (value & mask) == want */
static int poll32(const struct device *dev, uint32_t reg, uint32_t mask, uint32_t want,
		  k_timeout_t timeout)
{
	k_timepoint_t end = sys_timepoint_calc(timeout);

	do {
		if ((rd32(dev, reg) & mask) == want) {
			return 0;
		}
		k_busy_wait(POLL_US);
	} while (!sys_timepoint_expired(end));

	return (rd32(dev, reg) & mask) == want ? 0 : -ETIMEDOUT;
}

static int xenon_sw_reset(const struct device *dev, uint8_t mask)
{
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(100));

	wr8(dev, SDHCI_SOFTWARE_RESET, mask);
	while ((rd8(dev, SDHCI_SOFTWARE_RESET) & mask) != 0) {
		if (sys_timepoint_expired(end)) {
			LOG_ERR("reset 0x%x timeout", mask);
			return -ETIMEDOUT;
		}
		k_busy_wait(POLL_US);
	}

	return 0;
}

/* Xenon eMMC PHY (U-Boot xenon_mmc_phy_init/_set) */
static int xenon_phy_init(const struct device *dev, enum sdhc_timing_mode timing)
{
	uint32_t v = rd32(dev, XENON_PHY_TIMING_ADJUST) | XENON_SAMPL_INV_QSP_PHASE;

	if (timing == SDHC_TIMING_LEGACY || timing == SDHC_TIMING_HS ||
	    timing == SDHC_TIMING_SDR12 || timing == SDHC_TIMING_SDR25 ||
	    timing == SDHC_TIMING_SDR50) {
		v |= XENON_PHY_SLOW_MODE;
	} else {
		v &= ~XENON_PHY_SLOW_MODE;
	}
	wr32(dev, XENON_PHY_TIMING_ADJUST, v);

	if (poll32(dev, SDHCI_CLOCK_CONTROL, SDHCI_CLOCK_INT_STABLE, SDHCI_CLOCK_INT_STABLE,
		   K_MSEC(10)) < 0) {
		LOG_ERR("internal clock not stable");
		return -ETIMEDOUT;
	}

	set32(dev, XENON_PHY_TIMING_ADJUST, 0, XENON_PHY_INITIALIZATION);
	if (poll32(dev, XENON_PHY_TIMING_ADJUST, XENON_PHY_INITIALIZATION, 0, K_MSEC(10)) < 0) {
		LOG_ERR("PHY init timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static int xenon_phy_set(const struct device *dev, enum sdhc_timing_mode timing)
{
	bool ddr = timing == SDHC_TIMING_DDR50 || timing == SDHC_TIMING_DDR52;
	uint16_t clk;

	set32(dev, XENON_PHY_PAD_CONTROL, 0,
	      XENON_AUTO_RECEN_CTRL | XENON_OEN_QSN | XENON_FC_QSP_RECEN | XENON_FC_CMD_RECEN |
		      XENON_FC_DQ_RECEN);
	set32(dev, XENON_PHY_PAD_CONTROL1, XENON_FC_CMD_PD | XENON_FC_DQ_PD,
	      XENON_FC_CMD_PU | XENON_FC_DQ_PU);

	if (timing != SDHC_TIMING_LEGACY && timing != SDHC_TIMING_HS &&
	    timing != SDHC_TIMING_SDR12) {
		set32(dev, XENON_PHY_TIMING_ADJUST, 0, XENON_OUTPUT_QSN_PHASE);
	}

	/* The card clock is off while the function control changes */
	clk = rd16(dev, SDHCI_CLOCK_CONTROL);
	wr16(dev, SDHCI_CLOCK_CONTROL, clk & ~SDHCI_CLOCK_CARD_EN);
	set32(dev, XENON_PHY_FUNC_CONTROL, XENON_DQ_DDR_MODE | XENON_CMD_DDR_MODE,
	      ddr ? XENON_DQ_DDR_MODE | XENON_CMD_DDR_MODE : 0);
	wr16(dev, SDHCI_CLOCK_CONTROL, clk);

	return xenon_phy_init(dev, timing);
}

static int xenon_set_clock(const struct device *dev, uint32_t hz)
{
	struct xenon_data *data = dev->data;
	uint32_t div = 0;
	uint16_t clk;

	wr16(dev, SDHCI_CLOCK_CONTROL, 0);
	if (hz == 0) {
		return 0;
	}

	/* SDHCI v3 10 bit divider: base / (2 * N), N = 0 is the base */
	if (data->base_clk > hz) {
		div = DIV_ROUND_UP(data->base_clk, 2 * hz);
		div = MIN(div, 0x3ff);
	}

	clk = ((div & 0xff) << 8) | (((div >> 8) & 0x3) << 6) | SDHCI_CLOCK_INT_EN;
	wr16(dev, SDHCI_CLOCK_CONTROL, clk);
	if (poll32(dev, SDHCI_CLOCK_CONTROL, SDHCI_CLOCK_INT_STABLE, SDHCI_CLOCK_INT_STABLE,
		   K_MSEC(20)) < 0) {
		LOG_ERR("clock %u Hz not stable", hz);
		return -ETIMEDOUT;
	}
	wr16(dev, SDHCI_CLOCK_CONTROL, clk | SDHCI_CLOCK_CARD_EN);

	LOG_DBG("clock %u Hz (base %u, div %u)", div ? data->base_clk / (2 * div) : data->base_clk,
		data->base_clk, div);

	return 0;
}

static int xenon_set_voltage(const struct device *dev, enum sd_voltage voltage)
{
	const struct xenon_config *cfg = dev->config;
	bool v18 = voltage == SD_VOL_1_8_V;
	uint16_t ctrl2;
	int ret = 0;

	if (v18 && cfg->vqmmc == NULL && !cfg->pad_fixed_1v8) {
		return -ENOTSUP;
	}

	if (cfg->vqmmc != NULL) {
		ret = regulator_set_voltage(cfg->vqmmc, v18 ? 1800000 : 3300000,
					    v18 ? 1800000 : 3300000);
		if (ret < 0) {
			LOG_ERR("vqmmc %s: %d", v18 ? "1.8 V" : "3.3 V", ret);
			return ret;
		}
	}

	if (!cfg->pad_fixed_1v8) {
		sys_write32(v18 ? A3700_PAD_1_8V : A3700_PAD_3_3V, DEVICE_MMIO_NAMED_GET(dev, pad));
	}

	ctrl2 = rd16(dev, SDHCI_HOST_CONTROL2);
	ctrl2 = v18 ? ctrl2 | SDHCI_CTRL_VDD_180 : ctrl2 & ~SDHCI_CTRL_VDD_180;
	wr16(dev, SDHCI_HOST_CONTROL2, ctrl2);
	k_msleep(5);

	return 0;
}

static int xenon_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct xenon_config *cfg = dev->config;
	struct xenon_data *data = dev->data;
	uint8_t ctrl;
	uint16_t ctrl2;
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (ios->power_mode != data->ios.power_mode) {
		if (ios->power_mode == SDHC_POWER_ON) {
			wr8(dev, SDHCI_POWER_CONTROL, SDHCI_POWER_330);
			wr8(dev, SDHCI_POWER_CONTROL, SDHCI_POWER_330 | SDHCI_POWER_ON);
		} else {
			wr8(dev, SDHCI_POWER_CONTROL, 0);
		}
		k_msleep(cfg->power_delay_ms);
	}

	if (ios->signal_voltage != data->ios.signal_voltage) {
		ret = xenon_set_voltage(dev, ios->signal_voltage);
		if (ret < 0) {
			goto out;
		}
	}

	ctrl = rd8(dev, SDHCI_HOST_CONTROL) &
	       ~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS | SDHCI_CTRL_HISPD);
	if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
		ctrl |= SDHCI_CTRL_4BITBUS;
	} else if (ios->bus_width == SDHC_BUS_WIDTH8BIT) {
		ctrl |= SDHCI_CTRL_8BITBUS;
	}
	if (ios->timing != SDHC_TIMING_LEGACY && ios->timing != SDHC_TIMING_SDR12) {
		ctrl |= SDHCI_CTRL_HISPD;
	}
	wr8(dev, SDHCI_HOST_CONTROL, ctrl);

	ctrl2 = rd16(dev, SDHCI_HOST_CONTROL2) & ~SDHCI_CTRL_UHS_MASK;
	switch (ios->timing) {
	case SDHC_TIMING_SDR25:
		ctrl2 |= 1;
		break;
	case SDHC_TIMING_SDR50:
		ctrl2 |= 2;
		break;
	case SDHC_TIMING_SDR104:
	case SDHC_TIMING_HS200:
		ctrl2 |= 3;
		break;
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
		ctrl2 |= 4;
		break;
	default:
		break;
	}
	wr16(dev, SDHCI_HOST_CONTROL2, ctrl2);

	if (ios->clock != data->ios.clock || ios->timing != data->ios.timing) {
		uint32_t hz = ios->clock;

		if (hz != 0) {
			hz = CLAMP(hz, data->props.f_min, data->props.f_max);
		}
		ret = xenon_set_clock(dev, hz);
		if (ret == 0 && hz != 0) {
			/* SD/SDIO signals at 3.3 V VCCQ in the slot, then the PHY */
			set32(dev, XENON_SLOT_EMMC_CTRL, XENON_EMMC_VCCQ_MASK, XENON_EMMC_VCCQ_3_3V);
			ret = xenon_phy_set(dev, ios->timing);
		}
	}

	if (ret == 0) {
		data->ios = *ios;
	}
out:
	k_mutex_unlock(&data->lock);

	return ret;
}

static void xenon_read_response(const struct device *dev, struct sdhc_command *cmd)
{
	uint32_t r[4];

	for (int i = 0; i < 4; i++) {
		r[i] = rd32(dev, SDHCI_RESPONSE + 4 * i);
	}

	switch (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		memset(cmd->response, 0, sizeof(cmd->response));
		break;
	case SD_RSP_TYPE_R2:
		/* REP[119:0] without the CRC byte: shift to [127:8] */
		cmd->response[3] = ((r[3] & GENMASK(23, 0)) << 8) | (r[2] >> 24);
		cmd->response[2] = (r[2] << 8) | (r[1] >> 24);
		cmd->response[1] = (r[1] << 8) | (r[0] >> 24);
		cmd->response[0] = r[0] << 8;
		break;
	default:
		cmd->response[0] = r[0];
		cmd->response[1] = cmd->response[2] = cmd->response[3] = 0;
		break;
	}
}

/* Wait for one of the status bits, report errors */
static int xenon_wait_int(const struct device *dev, uint32_t bits, int timeout_ms)
{
	k_timepoint_t end = sys_timepoint_calc(timeout_ms > 0 ? K_MSEC(timeout_ms) : CMD_TIMEOUT);
	uint32_t st;

	for (;;) {
		st = rd32(dev, SDHCI_INT_STATUS);
		if (st & SDHCI_INT_ERROR) {
			wr32(dev, SDHCI_INT_STATUS, st);
			LOG_DBG("error status 0x%08x", st);
			/* command timeout error alone: no response */
			return (st & BIT(16)) ? -ETIMEDOUT : -EIO;
		}
		if (st & bits) {
			wr32(dev, SDHCI_INT_STATUS, st & bits);
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -ETIMEDOUT;
		}
		k_busy_wait(POLL_US);
	}
}

static bool is_write(uint32_t opcode)
{
	return opcode == SD_WRITE_SINGLE_BLOCK || opcode == SD_WRITE_MULTIPLE_BLOCK;
}

static int xenon_pio(const struct device *dev, struct sdhc_data *dat, bool write)
{
	uint32_t *p = dat->data;
	int ret;

	for (unsigned int b = 0; b < dat->blocks; b++) {
		ret = xenon_wait_int(dev, write ? SDHCI_INT_SPACE_AVAIL : SDHCI_INT_DATA_AVAIL,
				     dat->timeout_ms);
		if (ret < 0) {
			LOG_DBG("block %u/%u: %d", b, dat->blocks, ret);
			return ret;
		}
		for (unsigned int i = 0; i < DIV_ROUND_UP(dat->block_size, 4); i++) {
			if (write) {
				wr32(dev, SDHCI_BUFFER, *p++);
			} else {
				*p++ = rd32(dev, SDHCI_BUFFER);
			}
		}
		dat->bytes_xfered += dat->block_size;
	}

	return xenon_wait_int(dev, SDHCI_INT_DATA_END, dat->timeout_ms);
}

static int xenon_send(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *dat)
{
	uint32_t type = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	bool busy = type == SD_RSP_TYPE_R1b || type == SD_RSP_TYPE_R5b;
	uint32_t inhibit = SDHCI_CMD_INHIBIT;
	uint16_t command, mode = 0;
	int ret;

	if (dat != NULL || busy) {
		inhibit |= SDHCI_DATA_INHIBIT;
	}
	if (poll32(dev, SDHCI_PRESENT_STATE, inhibit, 0, K_MSEC(100)) < 0) {
		LOG_ERR("CMD%u: lines busy (0x%08x)", cmd->opcode, rd32(dev, SDHCI_PRESENT_STATE));
		return -EBUSY;
	}

	wr32(dev, SDHCI_INT_STATUS, 0xffffffff);

	switch (type) {
	case SD_RSP_TYPE_NONE:
		command = SDHCI_CMD_RESP_NONE;
		break;
	case SD_RSP_TYPE_R2:
		command = SDHCI_CMD_RESP_136 | SDHCI_CMD_CRC;
		break;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		command = SDHCI_CMD_RESP_48;
		break;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		command = SDHCI_CMD_RESP_48_BSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
		break;
	default:
		command = SDHCI_CMD_RESP_48 | SDHCI_CMD_CRC | SDHCI_CMD_INDEX;
		break;
	}
	command |= (cmd->opcode & 0x3f) << 8;

	if (dat != NULL) {
		command |= SDHCI_CMD_DATA;
		dat->bytes_xfered = 0;
		/* 512 KiB SDMA boundary, unused in PIO */
		wr16(dev, SDHCI_BLOCK_SIZE, (7 << 12) | (dat->block_size & 0xfff));
		wr16(dev, SDHCI_BLOCK_COUNT, dat->blocks);
		if (!is_write(cmd->opcode)) {
			mode |= SDHCI_TRNS_READ;
		}
		if (dat->blocks > 1) {
			mode |= SDHCI_TRNS_BLK_CNT_EN | SDHCI_TRNS_MULTI | SDHCI_TRNS_AUTO_CMD12;
		}
	}
	wr16(dev, SDHCI_TRANSFER_MODE, mode);
	wr32(dev, SDHCI_ARGUMENT, cmd->arg);
	wr16(dev, SDHCI_COMMAND, command);

	ret = xenon_wait_int(dev, SDHCI_INT_RESPONSE, cmd->timeout_ms);
	if (ret < 0) {
		goto err;
	}
	xenon_read_response(dev, cmd);

	if (dat != NULL) {
		ret = xenon_pio(dev, dat, is_write(cmd->opcode));
	} else if (busy) {
		ret = xenon_wait_int(dev, SDHCI_INT_DATA_END,
				     cmd->timeout_ms > 0 ? cmd->timeout_ms : 1000);
	}
	if (ret == 0) {
		return 0;
	}
err:
	(void)xenon_sw_reset(dev, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	return ret;
}

static int xenon_request(const struct device *dev, struct sdhc_command *cmd,
			 struct sdhc_data *dat)
{
	struct xenon_data *data = dev->data;
	int ret = -EIO;

	k_mutex_lock(&data->lock, K_FOREVER);
	for (unsigned int i = 0; i <= cmd->retries; i++) {
		ret = xenon_send(dev, cmd, dat);
		if (ret == 0 || ret == -ETIMEDOUT) {
			break;
		}
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int xenon_get_card_present(const struct device *dev)
{
	const struct xenon_config *cfg = dev->config;

	if (cfg->non_removable) {
		return 1;
	}
	if (cfg->cd_gpio.port != NULL) {
		return gpio_pin_get_dt(&cfg->cd_gpio) > 0;
	}

	return (rd32(dev, SDHCI_PRESENT_STATE) & SDHCI_CARD_PRESENT) != 0;
}

static int xenon_card_busy(const struct device *dev)
{
	return (rd32(dev, SDHCI_PRESENT_STATE) & SDHCI_DATA_0_LVL) == 0;
}

static int xenon_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	struct xenon_data *data = dev->data;

	*props = data->props;
	return 0;
}

static int xenon_reset(const struct device *dev)
{
	struct xenon_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = xenon_sw_reset(dev, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	k_mutex_unlock(&data->lock);

	return ret;
}

static void xenon_init_props(const struct device *dev)
{
	const struct xenon_config *cfg = dev->config;
	struct xenon_data *data = dev->data;
	uint64_t caps = rd32(dev, SDHCI_CAPABILITIES) |
			((uint64_t)rd32(dev, SDHCI_CAPABILITIES_1) << 32);
	struct sdhc_host_props *p = &data->props;
	bool v18 = cfg->vqmmc != NULL || cfg->pad_fixed_1v8;

	memset(p, 0, sizeof(*p));
	memcpy(&p->host_caps, &caps, sizeof(p->host_caps));

	if (data->base_clk == 0) {
		/* Base clock in MHz, capabilities bits 15:8 */
		data->base_clk = p->host_caps.sd_base_clk * 1000000U;
	}

	/* PIO only; 1.8 V signalling needs the IO voltage regulator */
	p->host_caps.sdma_support = 0;
	p->host_caps.adma_2_support = 0;
	p->host_caps.vol_180_support = v18;
	if (!v18) {
		p->host_caps.sdr50_support = 0;
		p->host_caps.sdr104_support = 0;
		p->host_caps.ddr50_support = 0;
	}
	p->host_caps.bus_8_bit_support = cfg->bus_width == 8;
	p->bus_4_bit_support = cfg->bus_width >= 4;
	p->f_min = cfg->min_bus_freq;
	p->f_max = MIN(cfg->max_bus_freq, data->base_clk);
	p->power_delay = cfg->power_delay_ms;
	p->max_current_330 = 800;
	p->max_current_180 = 800;
}

static int xenon_init(const struct device *dev)
{
	const struct xenon_config *cfg = dev->config;
	struct xenon_data *data = dev->data;
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, pad, K_MEM_CACHE_NONE);
	k_mutex_init(&data->lock);

	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			return ret;
		}
	}

	if (cfg->clk_dev != NULL) {
		if (!device_is_ready(cfg->clk_dev)) {
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clk_dev, cfg->clk_id);
		if (ret < 0 && ret != -EALREADY) {
			return ret;
		}
		(void)clock_control_get_rate(cfg->clk_dev, cfg->clk_id, &data->base_clk);
	}

	if (cfg->cd_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->cd_gpio)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}

	if (cfg->vqmmc != NULL) {
		if (!device_is_ready(cfg->vqmmc)) {
			return -ENODEV;
		}
		(void)regulator_set_voltage(cfg->vqmmc, 3300000, 3300000);
		(void)regulator_enable(cfg->vqmmc);
	}

	/* Xenon slot setup, as U-Boot xenon_sdhci_probe() */
	set32(dev, XENON_SYS_OP_CTRL, 0, XENON_AUTO_CLKGATE_DISABLE);
	set32(dev, XENON_SYS_OP_CTRL, 0, XENON_SLOT_ENABLE(XENON_SLOT));
	sys_write32(cfg->pad_fixed_1v8 ? A3700_PAD_1_8V : A3700_PAD_3_3V,
		    DEVICE_MMIO_NAMED_GET(dev, pad));

	ret = xenon_sw_reset(dev, SDHCI_RESET_ALL);
	if (ret < 0) {
		return ret;
	}

	set32(dev, XENON_SYS_EXT_OP_CTRL, 0, BIT(XENON_SLOT)); /* parallel transfer */
	set32(dev, XENON_SLOT_RETUNING_CTRL, XENON_RETUNING_COMPATIBLE, 0);
	set32(dev, SDHCI_SIGNAL_ENABLE, XENON_RETUNE_EVT_INTSIG, 0);
	set32(dev, XENON_SYS_OP_CTRL, XENON_AUTO_CLKGATE_DISABLE, 0);
	set32(dev, XENON_SYS_EXT_OP_CTRL, 0, XENON_MASK_CMD_CONFLICT);

	/* All status bits, polled, no interrupt signals */
	wr32(dev, SDHCI_INT_ENABLE, 0xffffffff);
	wr32(dev, SDHCI_SIGNAL_ENABLE, 0);
	wr8(dev, SDHCI_TIMEOUT_CONTROL, 0x0e);

	xenon_init_props(dev);
	data->ios.power_mode = SDHC_POWER_OFF;
	data->ios.signal_voltage = SD_VOL_3_3_V;

	LOG_INF("%s: SDHCI v%u, base clock %u Hz, card %s", dev->name,
		(rd16(dev, SDHCI_HOST_VERSION) & 0xff) + 1, data->base_clk,
		xenon_get_card_present(dev) ? "present" : "absent");

	return 0;
}

static DEVICE_API(sdhc, xenon_api) = {
	.reset = xenon_reset,
	.request = xenon_request,
	.set_io = xenon_set_io,
	.get_card_present = xenon_get_card_present,
	.card_busy = xenon_card_busy,
	.get_host_props = xenon_get_host_props,
};

#define XENON_PINCTRL_DEFINE(n) COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), \
	(PINCTRL_DT_INST_DEFINE(n);), ())
#define XENON_PINCTRL_GET(n) COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), \
	(PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define XENON_INIT(n)                                                                              \
	XENON_PINCTRL_DEFINE(n)                                                                    \
	static const struct xenon_config xenon_config_##n = {                                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(base, DT_DRV_INST(n)),                         \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(pad, DT_DRV_INST(n)),                          \
		.clk_dev = COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(n, 0),                               \
				       (DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n))), (NULL)),           \
		.clk_id = COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(n, 0),                                \
				      ((clock_control_subsys_t)(uintptr_t)                         \
					       DT_INST_CLOCKS_CELL(n, index)), (NULL)),            \
		.pcfg = XENON_PINCTRL_GET(n),                                                      \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cd_gpios, {0}),                             \
		.vqmmc = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, vqmmc_supply),                       \
				     (DEVICE_DT_GET(DT_INST_PHANDLE(n, vqmmc_supply))), (NULL)),   \
		.max_bus_freq = DT_INST_PROP_OR(n, max_bus_freq, 50000000),                        \
		.min_bus_freq = DT_INST_PROP_OR(n, min_bus_freq, 400000),                          \
		.power_delay_ms = DT_INST_PROP_OR(n, power_delay_ms, 10),                          \
		.bus_width = DT_INST_PROP_OR(n, bus_width, 4),                                     \
		.non_removable = DT_INST_PROP(n, non_removable),                                   \
		.pad_fixed_1v8 = DT_INST_ENUM_IDX_OR(n, marvell_pad_type, 0) == 1,                 \
	};                                                                                         \
	static struct xenon_data xenon_data_##n;                                                   \
	DEVICE_DT_INST_DEFINE(n, xenon_init, NULL, &xenon_data_##n, &xenon_config_##n,             \
			      POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, &xenon_api);

DT_INST_FOREACH_STATUS_OKAY(XENON_INIT)
