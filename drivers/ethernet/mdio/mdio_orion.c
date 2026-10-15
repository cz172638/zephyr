/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Orion SMI (clause 22 MDIO) controller, as found on Kirkwood,
 * Armada 370/XP/375/38x/37xx. One 32-bit register:
 *
 *   [15:0]  data
 *   [20:16] PHY address
 *   [25:21] register
 *   [26]    opcode, 1 = read
 *   [27]    read valid
 *   [28]    busy
 */

#define DT_DRV_COMPAT marvell_orion_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_orion, CONFIG_MDIO_LOG_LEVEL);

#define ORION_SMI_DATA_MASK  GENMASK(15, 0)
#define ORION_SMI_PHY_SHIFT  16
#define ORION_SMI_REG_SHIFT  21
#define ORION_SMI_READ       BIT(26)
#define ORION_SMI_READ_VALID BIT(27)
#define ORION_SMI_BUSY       BIT(28)

#define ORION_SMI_TIMEOUT_US 10000

struct mdio_orion_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
};

struct mdio_orion_data {
	DEVICE_MMIO_RAM;
	struct k_mutex lock;
};

static inline uint32_t smi_read(const struct device *dev)
{
	return sys_read32(DEVICE_MMIO_GET(dev));
}

static inline void smi_write(const struct device *dev, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev));
}

static int mdio_orion_wait(const struct device *dev, uint32_t mask, uint32_t expect)
{
	if (!WAIT_FOR((smi_read(dev) & mask) == expect, ORION_SMI_TIMEOUT_US, k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int mdio_orion_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_orion_data *dev_data = dev->data;
	uint32_t val;
	int ret;

	if (prtad > 31 || regad > 31) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	ret = mdio_orion_wait(dev, ORION_SMI_BUSY, 0);
	if (ret < 0) {
		goto out;
	}

	smi_write(dev, ((uint32_t)prtad << ORION_SMI_PHY_SHIFT) |
			       ((uint32_t)regad << ORION_SMI_REG_SHIFT) | ORION_SMI_READ);

	ret = mdio_orion_wait(dev, ORION_SMI_READ_VALID, ORION_SMI_READ_VALID);
	if (ret < 0) {
		LOG_WRN("read timeout phy 0x%02x reg 0x%02x", prtad, regad);
		goto out;
	}

	val = smi_read(dev);
	*data = val & ORION_SMI_DATA_MASK;

out:
	k_mutex_unlock(&dev_data->lock);
	return ret;
}

static int mdio_orion_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_orion_data *dev_data = dev->data;
	int ret;

	if (prtad > 31 || regad > 31) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	ret = mdio_orion_wait(dev, ORION_SMI_BUSY, 0);
	if (ret < 0) {
		goto out;
	}

	smi_write(dev, ((uint32_t)prtad << ORION_SMI_PHY_SHIFT) |
			       ((uint32_t)regad << ORION_SMI_REG_SHIFT) | data);

	ret = mdio_orion_wait(dev, ORION_SMI_BUSY, 0);
	if (ret < 0) {
		LOG_WRN("write timeout phy 0x%02x reg 0x%02x", prtad, regad);
	}

out:
	k_mutex_unlock(&dev_data->lock);
	return ret;
}

static int mdio_orion_init(const struct device *dev)
{
	const struct mdio_orion_config *cfg = dev->config;
	struct mdio_orion_data *dev_data = dev->data;
	int ret;

	k_mutex_init(&dev_data->lock);

	/* On Armada 3700 the SMI register sits inside the GbE0 block */
	if (cfg->clk_dev != NULL) {
		if (!device_is_ready(cfg->clk_dev)) {
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clk_dev, cfg->clk_id);
		if (ret < 0) {
			return ret;
		}
	}

	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(mdio, mdio_orion_api) = {
	.read = mdio_orion_read,
	.write = mdio_orion_write,
};

#define MDIO_ORION_PCFG(n)                                                                         \
	COND_CODE_1(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)), (NULL))

#define MDIO_ORION_CLK_DEV(n)                                                                      \
	COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(n, 0), (DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n))), (NULL))

#define MDIO_ORION_CLK_ID(n)                                                                       \
	COND_CODE_1(DT_INST_CLOCKS_HAS_IDX(n, 0),                                                  \
		    ((clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, index)), (NULL))

#define MDIO_ORION_INIT(n)                                                                         \
	IF_ENABLED(DT_INST_PINCTRL_HAS_IDX(n, 0), (PINCTRL_DT_INST_DEFINE(n);))                    \
	static struct mdio_orion_data mdio_orion_data_##n;                                         \
	static const struct mdio_orion_config mdio_orion_config_##n = {                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.pcfg = MDIO_ORION_PCFG(n),                                                        \
		.clk_dev = MDIO_ORION_CLK_DEV(n),                                                  \
		.clk_id = MDIO_ORION_CLK_ID(n),                                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mdio_orion_init, NULL, &mdio_orion_data_##n,                      \
			      &mdio_orion_config_##n, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,      \
			      &mdio_orion_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_ORION_INIT)
