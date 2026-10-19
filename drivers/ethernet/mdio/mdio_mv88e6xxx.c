/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal MDIO bus of a Marvell 88E6xxx switch: clause 22 accesses to
 * the internal PHYs go through the Global 2 SMI PHY command/data
 * registers of the parent switch device.
 */

#define DT_DRV_COMPAT marvell_mv88e6xxx_mdio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>

#include "../phy/phy_marvell_mv88e6xxx.h"

struct mdio_mv88e6xxx_config {
	const struct device *sw;
};

static int mdio_mv88e6xxx_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			       uint16_t *data)
{
	const struct mdio_mv88e6xxx_config *cfg = dev->config;

	return mv88e6xxx_phy_read(cfg->sw, prtad, regad, data);
}

static int mdio_mv88e6xxx_write(const struct device *dev, uint8_t prtad, uint8_t regad,
				uint16_t data)
{
	const struct mdio_mv88e6xxx_config *cfg = dev->config;

	return mv88e6xxx_phy_write(cfg->sw, prtad, regad, data);
}

static int mdio_mv88e6xxx_init(const struct device *dev)
{
	const struct mdio_mv88e6xxx_config *cfg = dev->config;

	return device_is_ready(cfg->sw) ? 0 : -ENODEV;
}

static DEVICE_API(mdio, mdio_mv88e6xxx_api) = {
	.read = mdio_mv88e6xxx_read,
	.write = mdio_mv88e6xxx_write,
};

#define MDIO_MV88E6XXX_INIT(n)                                                                     \
	static const struct mdio_mv88e6xxx_config mdio_mv88e6xxx_config_##n = {                    \
		.sw = DEVICE_DT_GET(DT_INST_PARENT(n)),                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, mdio_mv88e6xxx_init, NULL, NULL, &mdio_mv88e6xxx_config_##n,      \
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY, &mdio_mv88e6xxx_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_MV88E6XXX_INIT)
