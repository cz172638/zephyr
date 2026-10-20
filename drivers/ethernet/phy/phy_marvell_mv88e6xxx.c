/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal Marvell 88E6xxx (Link Street) switch bring-up, exposed to the
 * MAC as its PHY. Every user port can only forward to the CPU port, the
 * CPU port to all of them.
 *
 * Without DSA ("port expander") nothing is tagged and the MAC sees one
 * interface whose carrier is up while any user port has link. With the
 * DSA driver (dsa_mv88e6xxx.c) the CPU port speaks the Marvell DSA or
 * EDSA tag, each user port becomes its own interface and the MAC carrier
 * follows the fixed CPU port link. The internal PHYs are then reached
 * through the switch's MDIO bus (mdio_mv88e6xxx.c).
 *
 * Access is through the multi-chip SMI interface (switch strapped to a
 * non-zero SMI address): two registers at that address, command (0) and
 * data (1), are used to reach the internal per-port, Global 1/2 and PHY
 * registers. Internal PHYs are reached through the Global 2 SMI PHY
 * command/data registers.
 *
 * The register sequence follows U-Boot's espressobin_last_stage_init()
 * and Linux drivers/net/dsa/mv88e6xxx.
 */

#define DT_DRV_COMPAT marvell_mv88e6085

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_mv88e6xxx, CONFIG_PHY_LOG_LEVEL);

#include "phy_marvell_mv88e6xxx.h"

/* Multi-chip SMI: registers at the switch's own SMI address */
#define MC_SMI_CMD          0x00
#define MC_SMI_DATA         0x01
#define SMI_CMD_BUSY        BIT(15)
#define SMI_CMD_MODE_22     BIT(12)
#define SMI_CMD_OP_22_WRITE ((1 << 10) | SMI_CMD_MODE_22)
#define SMI_CMD_OP_22_READ  ((2 << 10) | SMI_CMD_MODE_22)
#define SMI_CMD_DEV(a)      (((a) & 0x1f) << 5)
#define SMI_CMD_REG(r)      ((r) & 0x1f)

/* Internal device addresses */
#define ADDR_PORT(p) (0x10 + (p))
#define ADDR_GLOBAL1 0x1b
#define ADDR_GLOBAL2 0x1c
#define ADDR_PHY(p)  (0x10 + (p))

/* Port registers */
#define PORT_STS            0x00
#define PORT_STS_LINK       BIT(11)
#define PORT_STS_DUPLEX     BIT(10)
#define PORT_STS_SPEED_MASK GENMASK(9, 8)
#define PORT_MAC_CTL        0x01
#define PORT_SWITCH_ID      0x03
#define PORT_SWITCH_ID_PROD GENMASK(15, 4)
#define PORT_CTL0           0x04
#define PORT_BASE_VLAN      0x06
#define PORT_CTL2           0x08
#define PORT_ETYPE          0x0f

/* Port control 0: state forwarding, flood unknown unicast and multicast,
 * initial priority from tag and IP, normal frame mode, egress unmodified.
 */
#define PORT_CTL0_FORWARDING       0x007f
#define PORT_CTL0_FRAME_MODE_MASK  GENMASK(9, 8)
#define PORT_CTL0_FRAME_MODE_DSA   0x0100
#define PORT_CTL0_FRAME_MODE_EDSA  0x0300
#define PORT_CTL0_EGRESS_MODE_MASK GENMASK(13, 12)
#define PORT_CTL0_EGRESS_MODE_EDSA 0x3000

/* Port control 2: maximum frame size, 2048 leaves room for the tag */
#define PORT_CTL2_JUMBO_MASK GENMASK(13, 12)
#define PORT_CTL2_JUMBO_2048 0x1000

/* Global 1 control 2: device number used in DSA tags */
#define G1_CTL2            0x1c
#define G1_CTL2_DEVICE_NUM GENMASK(4, 0)

/* Global 2 AVB/PTP indirect access (Linux global2_avb.c) */
#define G2_AVB_CMD           0x16
#define G2_AVB_DATA          0x17
#define AVB_CMD_BUSY         BIT(15)
#define AVB_CMD_BLOCK(b)     (((b) & 0x7) << 5)
/* 88E6352 layout */
#define AVB6352_OP_READ      0x4000
#define AVB6352_OP_READ_INCR 0x6000
#define AVB6352_OP_WRITE     0x3000
#define AVB6352_PORT_TAI     0xe
#define AVB6352_PORT_PTP     0xf
/* 88E6390/6341 layout */
#define AVB6390_OP_READ      0x0000
#define AVB6390_OP_READ_INCR 0x4000
#define AVB6390_OP_WRITE     0x6000
#define AVB6390_PORT_TAI     0x1e
#define AVB6390_PORT_PTP     0x1f

/* Global 2 SMI PHY access */
#define G2_SMI_PHY_CMD  0x18
#define G2_SMI_PHY_DATA 0x19

#define SMI_TIMEOUT_US 5000

struct mv88e6xxx_config {
	const struct device *mdio;
	uint8_t sw_addr;
	uint8_t cpu_port;
	uint32_t cpu_port_mac_ctl;
	uint8_t num_user_ports;
	const uint8_t *user_ports;
};

struct mv88e6xxx_data {
	const struct device *dev;
	struct k_mutex lock;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_work_delayable monitor;
	/* DSA tagging on the CPU port, carrier = CPU port link */
	bool dsa;
	/* Product number (port register 3, bits 15:4) */
	uint16_t product;
};

static int mc_wait(const struct mv88e6xxx_config *cfg)
{
	uint16_t val;
	int ret;

	for (int i = 0; i < SMI_TIMEOUT_US / 10; i++) {
		ret = mdio_read(cfg->mdio, cfg->sw_addr, MC_SMI_CMD, &val);
		if (ret < 0) {
			return ret;
		}
		if ((val & SMI_CMD_BUSY) == 0) {
			return 0;
		}
		k_busy_wait(10);
	}

	return -ETIMEDOUT;
}

static int sw_read(const struct mv88e6xxx_config *cfg, uint8_t addr, uint8_t reg, uint16_t *val)
{
	int ret;

	ret = mc_wait(cfg);
	if (ret == 0) {
		ret = mdio_write(cfg->mdio, cfg->sw_addr, MC_SMI_CMD,
				 SMI_CMD_BUSY | SMI_CMD_OP_22_READ | SMI_CMD_DEV(addr) |
					 SMI_CMD_REG(reg));
	}
	if (ret == 0) {
		ret = mc_wait(cfg);
	}
	if (ret == 0) {
		ret = mdio_read(cfg->mdio, cfg->sw_addr, MC_SMI_DATA, val);
	}

	return ret;
}

static int sw_write(const struct mv88e6xxx_config *cfg, uint8_t addr, uint8_t reg, uint16_t val)
{
	int ret;

	ret = mc_wait(cfg);
	if (ret == 0) {
		ret = mdio_write(cfg->mdio, cfg->sw_addr, MC_SMI_DATA, val);
	}
	if (ret == 0) {
		ret = mdio_write(cfg->mdio, cfg->sw_addr, MC_SMI_CMD,
				 SMI_CMD_BUSY | SMI_CMD_OP_22_WRITE | SMI_CMD_DEV(addr) |
					 SMI_CMD_REG(reg));
	}
	if (ret == 0) {
		ret = mc_wait(cfg);
	}

	return ret;
}

static int g2_phy_wait(const struct mv88e6xxx_config *cfg)
{
	uint16_t val;
	int ret;

	for (int i = 0; i < SMI_TIMEOUT_US / 10; i++) {
		ret = sw_read(cfg, ADDR_GLOBAL2, G2_SMI_PHY_CMD, &val);
		if (ret < 0) {
			return ret;
		}
		if ((val & SMI_CMD_BUSY) == 0) {
			return 0;
		}
		k_busy_wait(10);
	}

	return -ETIMEDOUT;
}

static int sw_phy_write(const struct mv88e6xxx_config *cfg, uint8_t phy, uint8_t reg, uint16_t val)
{
	int ret;

	ret = g2_phy_wait(cfg);
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_SMI_PHY_DATA, val);
	}
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_SMI_PHY_CMD,
			       SMI_CMD_BUSY | SMI_CMD_OP_22_WRITE | SMI_CMD_DEV(phy) |
				       SMI_CMD_REG(reg));
	}
	if (ret == 0) {
		ret = g2_phy_wait(cfg);
	}

	return ret;
}

static int sw_phy_read(const struct mv88e6xxx_config *cfg, uint8_t phy, uint8_t reg, uint16_t *val)
{
	int ret;

	ret = g2_phy_wait(cfg);
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_SMI_PHY_CMD,
			       SMI_CMD_BUSY | SMI_CMD_OP_22_READ | SMI_CMD_DEV(phy) |
				       SMI_CMD_REG(reg));
	}
	if (ret == 0) {
		ret = g2_phy_wait(cfg);
	}
	if (ret == 0) {
		ret = sw_read(cfg, ADDR_GLOBAL2, G2_SMI_PHY_DATA, val);
	}

	return ret;
}

/* Carrier of the MAC. The MAC side itself is always the fixed 1000/full
 * RGMII link to the CPU port. Without DSA it is up while at least one
 * user port has link, with DSA it is the CPU port link and the user ports
 * report their own.
 */
static int mv88e6xxx_poll(const struct device *dev, struct phy_link_state *state)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint16_t sts;
	int ret = 0;

	state->speed = LINK_FULL_1000BASE;
	state->is_up = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->dsa) {
		ret = sw_read(cfg, ADDR_PORT(cfg->cpu_port), PORT_STS, &sts);
		state->is_up = (ret == 0) && (sts & PORT_STS_LINK) != 0;
	} else {
		for (int i = 0; ret == 0 && i < cfg->num_user_ports; i++) {
			ret = sw_read(cfg, ADDR_PORT(cfg->user_ports[i]), PORT_STS, &sts);
			if (ret == 0 && (sts & PORT_STS_LINK)) {
				state->is_up = true;
			}
		}
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

static int mv88e6xxx_get_link(const struct device *dev, struct phy_link_state *state)
{
	struct mv88e6xxx_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*state = data->state;
	k_mutex_unlock(&data->lock);

	return 0;
}

static void mv88e6xxx_monitor(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mv88e6xxx_data *data = CONTAINER_OF(dwork, struct mv88e6xxx_data, monitor);
	struct phy_link_state state;
	bool changed = false;

	if (mv88e6xxx_poll(data->dev, &state) == 0) {
		k_mutex_lock(&data->lock, K_FOREVER);
		if (state.is_up != data->state.is_up || state.speed != data->state.speed) {
			data->state = state;
			changed = true;
		}
		k_mutex_unlock(&data->lock);
	}

	if (changed) {
		LOG_INF("%s: %s link %s", data->dev->name, data->dsa ? "CPU port" : "user ports",
			state.is_up ? "up" : "down");
		if (data->cb != NULL) {
			data->cb(data->dev, &state, data->cb_data);
		}
	}

	k_work_reschedule(&data->monitor, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int mv88e6xxx_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct mv88e6xxx_data *data = dev->data;
	struct phy_link_state state;

	data->cb = cb;
	data->cb_data = user_data;

	if (cb != NULL) {
		mv88e6xxx_get_link(dev, &state);
		cb(dev, &state, user_data);
	}

	return 0;
}

/* phy_read()/phy_write() reach the switch internal registers:
 * reg_addr[12:8] = internal device address, reg_addr[4:0] = register.
 */
static int mv88e6xxx_reg_read(const struct device *dev, uint16_t reg_addr, uint32_t *val)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint16_t v;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_read(cfg, (reg_addr >> 8) & 0x1f, reg_addr & 0x1f, &v);
	k_mutex_unlock(&data->lock);
	*val = v;

	return ret;
}

static int mv88e6xxx_reg_write(const struct device *dev, uint16_t reg_addr, uint32_t val)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_write(cfg, (reg_addr >> 8) & 0x1f, reg_addr & 0x1f, val);
	k_mutex_unlock(&data->lock);

	return ret;
}

int mv88e6xxx_phy_read(const struct device *dev, uint8_t phy, uint8_t reg, uint16_t *val)
{
	struct mv88e6xxx_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_phy_read(dev->config, phy, reg, val);
	k_mutex_unlock(&data->lock);

	return ret;
}

int mv88e6xxx_phy_write(const struct device *dev, uint8_t phy, uint8_t reg, uint16_t val)
{
	struct mv88e6xxx_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_phy_write(dev->config, phy, reg, val);
	k_mutex_unlock(&data->lock);

	return ret;
}

uint16_t mv88e6xxx_product(const struct device *dev)
{
	struct mv88e6xxx_data *data = dev->data;

	return data->product;
}

/* 88E6341/6141 and the 6390 family use the 6390 AVB command layout */
static bool avb_is_6390(const struct mv88e6xxx_data *data)
{
	switch (data->product) {
	case 0x341:
	case 0x141:
	case 0x390:
	case 0x190:
	case 0x191:
	case 0x290:
	case 0x393:
		return true;
	default:
		return false;
	}
}

static int avb_wait(const struct mv88e6xxx_config *cfg)
{
	uint16_t val;
	int ret;

	for (int i = 0; i < SMI_TIMEOUT_US / 10; i++) {
		ret = sw_read(cfg, ADDR_GLOBAL2, G2_AVB_CMD, &val);
		if (ret < 0) {
			return ret;
		}
		if ((val & AVB_CMD_BUSY) == 0) {
			return 0;
		}
		k_busy_wait(10);
	}

	return -ETIMEDOUT;
}

static uint16_t avb_port(const struct mv88e6xxx_data *data, int port)
{
	bool l6390 = avb_is_6390(data);

	if (port == MV88E6XXX_AVB_PORT_TAI) {
		port = l6390 ? AVB6390_PORT_TAI : AVB6352_PORT_TAI;
	} else if (port == MV88E6XXX_AVB_PORT_PTP) {
		port = l6390 ? AVB6390_PORT_PTP : AVB6352_PORT_PTP;
	}

	return (port & (l6390 ? 0x1f : 0x0f)) << 8;
}

bool mv88e6xxx_avb_is_6390(const struct device *dev)
{
	return avb_is_6390(dev->data);
}

int mv88e6xxx_avb_block_read(const struct device *dev, enum mv88e6xxx_avb_block block, int port,
			     uint8_t addr, uint16_t *buf, int len)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	bool l6390 = avb_is_6390(data);
	uint16_t op;
	int ret;

	/* The hardware snapshots at most four words */
	if (len < 1 || len > 4) {
		return -EINVAL;
	}
	if (len == 1) {
		op = l6390 ? AVB6390_OP_READ : AVB6352_OP_READ;
	} else {
		op = l6390 ? AVB6390_OP_READ_INCR : AVB6352_OP_READ_INCR;
	}
	op |= AVB_CMD_BUSY | avb_port(data, port) | AVB_CMD_BLOCK(block) | (addr & 0x1f);

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = avb_wait(cfg);
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_AVB_CMD, op);
	}
	if (ret == 0) {
		ret = avb_wait(cfg);
	}
	for (int i = 0; ret == 0 && i < len; i++) {
		ret = sw_read(cfg, ADDR_GLOBAL2, G2_AVB_DATA, &buf[i]);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

int mv88e6xxx_avb_block_write(const struct device *dev, enum mv88e6xxx_avb_block block, int port,
			      uint8_t addr, uint16_t val)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint16_t op = (avb_is_6390(data) ? AVB6390_OP_WRITE : AVB6352_OP_WRITE) | AVB_CMD_BUSY |
		      avb_port(data, port) | AVB_CMD_BLOCK(block) | (addr & 0x1f);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = avb_wait(cfg);
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_AVB_DATA, val);
	}
	if (ret == 0) {
		ret = sw_write(cfg, ADDR_GLOBAL2, G2_AVB_CMD, op);
	}
	if (ret == 0) {
		ret = avb_wait(cfg);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

uint8_t mv88e6xxx_cpu_port(const struct device *dev)
{
	const struct mv88e6xxx_config *cfg = dev->config;

	return cfg->cpu_port;
}

static int sw_rmw(const struct mv88e6xxx_config *cfg, uint8_t addr, uint8_t reg, uint16_t mask,
		  uint16_t val)
{
	uint16_t v;
	int ret;

	ret = sw_read(cfg, addr, reg, &v);
	if (ret == 0) {
		ret = sw_write(cfg, addr, reg, (v & ~mask) | val);
	}

	return ret;
}

int mv88e6xxx_reg_rmw(const struct device *dev, uint8_t addr, uint8_t reg, uint16_t mask,
		      uint16_t val, uint16_t *old)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint16_t v;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sw_read(cfg, addr, reg, &v);
	if (ret == 0 && mask != 0) {
		ret = sw_write(cfg, addr, reg, (v & ~mask) | (val & mask));
	}
	k_mutex_unlock(&data->lock);

	if (ret == 0 && old != NULL) {
		*old = v;
	}

	return ret;
}

/* As Linux mv88e6xxx_set_port_mode_dsa()/_edsa() for the CPU port */
int mv88e6xxx_dsa_enable(const struct device *dev, enum mv88e6xxx_tag tag)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint8_t cpu = ADDR_PORT(cfg->cpu_port);
	uint16_t ctl0 = PORT_CTL0_FORWARDING;
	int ret;

	switch (tag) {
	case MV88E6XXX_TAG_DSA:
		ctl0 |= PORT_CTL0_FRAME_MODE_DSA;
		break;
	case MV88E6XXX_TAG_EDSA:
		ctl0 |= PORT_CTL0_FRAME_MODE_EDSA | PORT_CTL0_EGRESS_MODE_EDSA;
		break;
	default:
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Single switch: device 0, as addressed by the tags */
	ret = sw_rmw(cfg, ADDR_GLOBAL1, G1_CTL2, G1_CTL2_DEVICE_NUM, 0);
	if (ret == 0) {
		ret = sw_write(cfg, cpu, PORT_CTL0, ctl0);
	}
	if (ret == 0 && tag == MV88E6XXX_TAG_EDSA) {
		ret = sw_write(cfg, cpu, PORT_ETYPE, MV88E6XXX_EDSA_ETHERTYPE);
	}
	/* Tagged frames are longer than 1522 bytes at full MTU */
	if (ret == 0) {
		ret = sw_rmw(cfg, cpu, PORT_CTL2, PORT_CTL2_JUMBO_MASK, PORT_CTL2_JUMBO_2048);
	}
	for (int i = 0; ret == 0 && i < cfg->num_user_ports; i++) {
		ret = sw_rmw(cfg, ADDR_PORT(cfg->user_ports[i]), PORT_CTL2, PORT_CTL2_JUMBO_MASK,
			     PORT_CTL2_JUMBO_2048);
	}
	if (ret == 0) {
		data->dsa = true;
	}

	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("%s: DSA setup failed (%d)", dev->name, ret);
		return ret;
	}

	LOG_INF("%s: CPU port %u in %s mode", dev->name, cfg->cpu_port,
		tag == MV88E6XXX_TAG_EDSA ? "EDSA" : "DSA");

	/* Carrier now follows the CPU port */
	k_work_reschedule(&data->monitor, K_NO_WAIT);

	return 0;
}

static int mv88e6xxx_init(const struct device *dev)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	struct mv88e6xxx_data *data = dev->data;
	uint16_t id, bmcr;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->monitor, mv88e6xxx_monitor);

	if (!device_is_ready(cfg->mdio)) {
		return -ENODEV;
	}

	ret = sw_read(cfg, ADDR_PORT(cfg->cpu_port), PORT_SWITCH_ID, &id);
	if (ret < 0) {
		LOG_ERR("no switch at SMI address %u (%d)", cfg->sw_addr, ret);
		return ret;
	}
	data->product = FIELD_GET(PORT_SWITCH_ID_PROD, id);
	LOG_INF("switch product 0x%03x rev %u at SMI address %u", data->product, id & 0xf,
		cfg->sw_addr);

	/* User ports may only talk to the CPU port */
	for (int i = 0; i < cfg->num_user_ports; i++) {
		ret = sw_write(cfg, ADDR_PORT(cfg->user_ports[i]), PORT_BASE_VLAN,
			       BIT(cfg->cpu_port));
		if (ret < 0) {
			return ret;
		}
	}

	/* Forwarding state on CPU and user ports */
	ret = sw_write(cfg, ADDR_PORT(cfg->cpu_port), PORT_CTL0, PORT_CTL0_FORWARDING);
	for (int i = 0; ret == 0 && i < cfg->num_user_ports; i++) {
		ret = sw_write(cfg, ADDR_PORT(cfg->user_ports[i]), PORT_CTL0, PORT_CTL0_FORWARDING);
	}
	if (ret < 0) {
		return ret;
	}

	/* CPU port MAC: RGMII RX/TX delays, speed */
	ret = sw_write(cfg, ADDR_PORT(cfg->cpu_port), PORT_MAC_CTL, cfg->cpu_port_mac_ctl);
	if (ret < 0) {
		return ret;
	}

	/* Power up the internal PHYs: autoneg on, full duplex, 1000 */
	for (int i = 0; i < cfg->num_user_ports; i++) {
		uint8_t phy = ADDR_PHY(cfg->user_ports[i]);

		ret = sw_phy_read(cfg, phy, MII_BMCR, &bmcr);
		if (ret < 0) {
			return ret;
		}
		bmcr &= ~MII_BMCR_POWER_DOWN;
		bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_DUPLEX_MODE | MII_BMCR_SPEED_1000;
		ret = sw_phy_write(cfg, phy, MII_BMCR, bmcr | MII_BMCR_RESET);
		if (ret < 0) {
			return ret;
		}
	}

	mv88e6xxx_poll(dev, &data->state);
	k_work_reschedule(&data->monitor, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));

	return 0;
}

static DEVICE_API(ethphy, mv88e6xxx_api) = {
	.get_link = mv88e6xxx_get_link,
	.link_cb_set = mv88e6xxx_link_cb_set,
	.read = mv88e6xxx_reg_read,
	.write = mv88e6xxx_reg_write,
};

#define MV88E6XXX_INIT(n)                                                                          \
	static const uint8_t mv88e6xxx_user_ports_##n[] = DT_INST_PROP(n, marvell_user_ports);     \
	static const struct mv88e6xxx_config mv88e6xxx_config_##n = {                              \
		.mdio = DEVICE_DT_GET(DT_INST_BUS(n)),                                             \
		.sw_addr = DT_INST_REG_ADDR(n),                                                    \
		.cpu_port = DT_INST_PROP(n, marvell_cpu_port),                                     \
		.cpu_port_mac_ctl = DT_INST_PROP(n, marvell_cpu_port_mac_ctl),                     \
		.num_user_ports = ARRAY_SIZE(mv88e6xxx_user_ports_##n),                            \
		.user_ports = mv88e6xxx_user_ports_##n,                                            \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_REG_ADDR(n) != 0, "only multi-chip addressing is supported");         \
	static struct mv88e6xxx_data mv88e6xxx_data_##n;                                           \
	DEVICE_DT_INST_DEFINE(n, mv88e6xxx_init, NULL, &mv88e6xxx_data_##n, &mv88e6xxx_config_##n, \
			      POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, &mv88e6xxx_api);

DT_INST_FOREACH_STATUS_OKAY(MV88E6XXX_INIT)
