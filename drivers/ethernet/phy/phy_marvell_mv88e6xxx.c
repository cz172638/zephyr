/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal Marvell 88E6xxx (Link Street) switch bring-up, exposed to the
 * MAC as its PHY. The switch is set up as a "port expander": every user
 * port can only forward to the CPU port, the CPU port to all of them, no
 * DSA tagging. The MAC sees one interface whose carrier is up while any
 * user port has link.
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

/* Port control 0: state forwarding, egress unmodified, all frame modes */
#define PORT_CTL0_FORWARDING 0x007f

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

/* Carrier of the MAC: up while at least one user port has link. The MAC
 * side itself is always the fixed 1000/full RGMII link to the CPU port.
 */
static int mv88e6xxx_poll(const struct device *dev, struct phy_link_state *state)
{
	const struct mv88e6xxx_config *cfg = dev->config;
	uint16_t sts;
	int ret;

	state->speed = LINK_FULL_1000BASE;
	state->is_up = false;

	for (int i = 0; i < cfg->num_user_ports; i++) {
		ret = sw_read(cfg, ADDR_PORT(cfg->user_ports[i]), PORT_STS, &sts);
		if (ret < 0) {
			return ret;
		}
		if (sts & PORT_STS_LINK) {
			state->is_up = true;
		}
	}

	return 0;
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
		LOG_INF("%s: user ports link %s", data->dev->name, state.is_up ? "up" : "down");
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
	LOG_INF("switch product 0x%03lx rev %u at SMI address %u",
		FIELD_GET(PORT_SWITCH_ID_PROD, id), id & 0xf, cfg->sw_addr);

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
