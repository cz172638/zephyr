/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interface of the 88E6xxx switch driver for its MDIO bus and DSA driver.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_
#define ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_

#include <stdbool.h>
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

/* Internal SMI device addresses, for mv88e6xxx_reg_rmw() */
#define MV88E6XXX_ADDR_PORT(p) (0x10 + (p))
#define MV88E6XXX_ADDR_GLOBAL1 0x1b
#define MV88E6XXX_ADDR_GLOBAL2 0x1c

/* Read-modify-write of a switch register under the driver lock; with
 * mask 0 only reads. The previous value goes to old if not NULL.
 */
int mv88e6xxx_reg_rmw(const struct device *dev, uint8_t addr, uint8_t reg, uint16_t mask,
		      uint16_t val, uint16_t *old);

/* ATU entry states for unicast addresses */
#define MV88E6XXX_ATU_STATE_UNUSED    0x0
#define MV88E6XXX_ATU_STATE_UC_STATIC 0xe

/* Load (or with MV88E6XXX_ATU_STATE_UNUSED purge) an address of database
 * fid, forwarding to the ports in portvec
 */
int mv88e6xxx_atu_load(const struct device *dev, uint16_t fid, const uint8_t mac[6],
		       uint16_t portvec, uint8_t state);

/* Forget all learned addresses */
int mv88e6xxx_atu_flush_dynamic(const struct device *dev);

/* Register blocks behind the Global 2 AVB command/data registers */
enum mv88e6xxx_avb_block {
	MV88E6XXX_AVB_BLOCK_PTP = 0,
	MV88E6XXX_AVB_BLOCK_AVB = 1,
	MV88E6XXX_AVB_BLOCK_QAV = 2,
	MV88E6XXX_AVB_BLOCK_QVB = 3,
};

/* AVB command "port" numbers: the global registers of a block (PTP, AVB
 * and Qav global share one number) and the TAI registers
 */
#define MV88E6XXX_AVB_PORT_GLOBAL (-1)
#define MV88E6XXX_AVB_PORT_PTP    MV88E6XXX_AVB_PORT_GLOBAL
#define MV88E6XXX_AVB_PORT_TAI    (-2)

/* True for the 88E6390 command layout (88E6341/6141, 88E6390 family),
 * false for the 88E6352 one
 */
bool mv88e6xxx_avb_is_6390(const struct device *dev);

/* Registers of an AVB block: port is a switch port or
 * MV88E6XXX_AVB_PORT_GLOBAL/TAI, up to 4 words are read starting at addr.
 */
int mv88e6xxx_avb_block_read(const struct device *dev, enum mv88e6xxx_avb_block block, int port,
			     uint8_t addr, uint16_t *buf, int len);
int mv88e6xxx_avb_block_write(const struct device *dev, enum mv88e6xxx_avb_block block, int port,
			      uint8_t addr, uint16_t val);

/* PTP block shortcuts */
static inline int mv88e6xxx_avb_read(const struct device *dev, int port, uint8_t addr,
				     uint16_t *buf, int len)
{
	return mv88e6xxx_avb_block_read(dev, MV88E6XXX_AVB_BLOCK_PTP, port, addr, buf, len);
}

static inline int mv88e6xxx_avb_write(const struct device *dev, int port, uint8_t addr,
				      uint16_t val)
{
	return mv88e6xxx_avb_block_write(dev, MV88E6XXX_AVB_BLOCK_PTP, port, addr, val);
}

/* Switch the CPU port to tagged frames; the MAC carrier then follows the
 * CPU port link instead of the user ports.
 */
int mv88e6xxx_dsa_enable(const struct device *dev, enum mv88e6xxx_tag tag);

#endif /* ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_H_ */
