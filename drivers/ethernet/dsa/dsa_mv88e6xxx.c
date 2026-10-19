/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DSA driver for Marvell 88E6xxx switches. The switch itself (SMI access,
 * port isolation, CPU port MAC, internal PHY power) is handled by the
 * marvell,mv88e6085 driver, which stays the PHY of the conduit MAC. This
 * driver creates one interface per user port and switches the CPU port
 * to tagged frames once all ports exist. User port links come from their
 * PHYs (generic PHY driver on the switch's internal MDIO bus).
 *
 * Ports are standalone: every user port forwards only to the CPU port.
 */

#define DT_DRV_COMPAT marvell_mv88e6xxx_dsa

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dsa_mv88e6xxx, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/dt-bindings/ethernet/dsa_tag_proto.h>

#include "../phy/phy_marvell_mv88e6xxx.h"

BUILD_ASSERT(!IS_ENABLED(CONFIG_DSA_TAG_PROTOCOL_MV88E6XXX) || CONFIG_DSA_TAG_SIZE >= 4,
	     "DSA_TAG_SIZE must be 4 (DSA) or 8 (EDSA)");

struct dsa_mv88e6xxx_data {
	const struct device *sw;
	uint8_t cpu_port;
	int tag_proto;
};

static int dsa_mv88e6xxx_connect_tag_protocol(struct dsa_switch_context *ctx, int tag_proto)
{
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;

	if (tag_proto != DSA_TAG_PROTO_DSA && tag_proto != DSA_TAG_PROTO_EDSA) {
		LOG_ERR("tag protocol %d not supported", tag_proto);
		return -ENOTSUP;
	}
	if (tag_proto == DSA_TAG_PROTO_EDSA && CONFIG_DSA_TAG_SIZE < 8) {
		LOG_ERR("EDSA needs CONFIG_DSA_TAG_SIZE=8");
		return -EINVAL;
	}
	prv->tag_proto = tag_proto;

	return 0;
}

static int dsa_mv88e6xxx_switch_setup(const struct dsa_switch_context *ctx)
{
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;

	if (!device_is_ready(prv->sw)) {
		LOG_ERR("switch %s not ready", prv->sw->name);
		return -ENODEV;
	}
	if (mv88e6xxx_cpu_port(prv->sw) != prv->cpu_port) {
		LOG_ERR("CPU port %u differs from the switch's marvell,cpu-port %u", prv->cpu_port,
			mv88e6xxx_cpu_port(prv->sw));
		return -EINVAL;
	}

	return mv88e6xxx_dsa_enable(prv->sw, prv->tag_proto == DSA_TAG_PROTO_EDSA
						     ? MV88E6XXX_TAG_EDSA
						     : MV88E6XXX_TAG_DSA);
}

static enum ethernet_hw_caps dsa_mv88e6xxx_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE;
}

#define MV_DSA_PHANDLE_DEV(port, prop)                                                             \
	COND_CODE_1(DT_NODE_HAS_PROP(port, prop), (DEVICE_DT_GET(DT_PHANDLE(port, prop))), (NULL))

#define MV_DSA_PORT_INIT(port, n)                                                                  \
	static struct dsa_port_config dsa_mv88e6xxx_##n##_##port##_config = {                      \
		.mcfg = NET_ETH_MAC_DT_CONFIG_INIT(port),                                          \
		.port_idx = DT_REG_ADDR(port),                                                     \
		.phy_dev = MV_DSA_PHANDLE_DEV(port, phy_handle),                                   \
		.phy_mode = DT_PROP_OR(port, phy_connection_type, ""),                             \
		.tag_proto = DT_PROP_OR(port, dsa_tag_protocol, DSA_TAG_PROTO_NOTAG),              \
		.ethernet_connection = MV_DSA_PHANDLE_DEV(port, ethernet),                         \
	};                                                                                         \
	DSA_PORT_INST_INIT(port, n, &dsa_mv88e6xxx_##n##_##port##_config)

/* The CPU port is the child with an "ethernet" phandle */
#define MV_DSA_CPU_PORT_REG(port)                                                                  \
	COND_CODE_1(DT_NODE_HAS_PROP(port, ethernet), (DT_REG_ADDR(port)), ())

#define DSA_MV88E6XXX_DEVICE(n)                                                                    \
	static struct dsa_api dsa_mv88e6xxx_api_##n = {                                            \
		.switch_setup = dsa_mv88e6xxx_switch_setup,                                        \
		.connect_tag_protocol = dsa_mv88e6xxx_connect_tag_protocol,                        \
		.get_capabilities = dsa_mv88e6xxx_get_capabilities,                                \
	};                                                                                         \
	static struct dsa_mv88e6xxx_data dsa_mv88e6xxx_data_##n = {                                \
		.sw = DEVICE_DT_GET(DT_INST_PHANDLE(n, marvell_switch)),                           \
		.cpu_port = DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MV_DSA_CPU_PORT_REG),             \
	};                                                                                         \
	DSA_SWITCH_INST_INIT(n, &dsa_mv88e6xxx_api_##n, &dsa_mv88e6xxx_data_##n, MV_DSA_PORT_INIT);

DT_INST_FOREACH_STATUS_OKAY(DSA_MV88E6XXX_DEVICE)
