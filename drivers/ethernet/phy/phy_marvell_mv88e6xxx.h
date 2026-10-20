/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interface of the 88E6xxx switch driver for its MDIO bus and DSA driver.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_
#define ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_

#include <stdint.h>
#include <zephyr/device.h>

/* Default EtherType of the EDSA tag, as used by Linux */
#define MV88E6XXX_EDSA_ETHERTYPE 0xdada

enum mv88e6xxx_tag {
	MV88E6XXX_TAG_DSA,
	MV88E6XXX_TAG_EDSA,
};

/* Clause 22 access to the internal PHYs (Global 2 SMI PHY registers) */
int mv88e6xxx_phy_read(const struct device *dev, uint8_t phy, uint8_t reg, uint16_t *val);
int mv88e6xxx_phy_write(const struct device *dev, uint8_t phy, uint8_t reg, uint16_t val);

uint8_t mv88e6xxx_cpu_port(const struct device *dev);

/* Product number, e.g. 0x341 */
uint16_t mv88e6xxx_product(const struct device *dev);

/* AVB block "port" numbers for the global PTP and TAI registers */
#define MV88E6XXX_AVB_PORT_PTP (-1)
#define MV88E6XXX_AVB_PORT_TAI (-2)

/* PTP block registers through the Global 2 AVB command/data registers:
 * port is a switch port or MV88E6XXX_AVB_PORT_PTP/TAI, up to 4 words are
 * read starting at addr.
 */
int mv88e6xxx_avb_read(const struct device *dev, int port, uint8_t addr, uint16_t *buf, int len);
int mv88e6xxx_avb_write(const struct device *dev, int port, uint8_t addr, uint16_t val);

/* Switch the CPU port to tagged frames; the MAC carrier then follows the
 * CPU port link instead of the user ports.
 */
int mv88e6xxx_dsa_enable(const struct device *dev, enum mv88e6xxx_tag tag);

#endif /* ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_ */
