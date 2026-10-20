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
 *
 * With a ptp-clock on the ports (marvell,mv88e6xxx-ptp) PTP event messages
 * are time stamped by the switch: arrival time stamps are read from the
 * port's arrival registers when the tagger sees the message, departure
 * time stamps are polled after sending, as Linux hwtstamp.c does.
 */

#define DT_DRV_COMPAT marvell_mv88e6xxx_dsa

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dsa_mv88e6xxx, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/dt-bindings/ethernet/dsa_tag_proto.h>

#include "../phy/phy_marvell_mv88e6xxx.h"
#include "dsa_tag_mv88e6xxx.h"

#if defined(CONFIG_NET_L2_PTP_TIMESTAMPING) && defined(CONFIG_PTP_CLOCK_MV88E6XXX)
#define MV_DSA_PTP 1
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/byteorder.h>
#include "../../ptp_clock/ptp_clock_mv88e6xxx.h"

/* Port PTP registers */
#define PORT_PTP_CFG0                0x00
#define PORT_PTP_CFG0_NO_TSPEC_MATCH 0x0800
#define PORT_PTP_CFG2                0x02
#define PORT_PTP_ARR0_STS            0x08
#define PORT_PTP_ARR1_STS            0x0c
#define PORT_PTP_DEP_STS             0x10
#define PTP_TS_VALID                 BIT(0)
#define PTP_TS_STATUS_MASK           GENMASK(2, 1)

#define PTP_MSG_PDLY_RES 3
#define ETH_P_1588       0x88f7

/* Linux TX_TSTAMP_TIMEOUT */
#define TX_TSTAMP_TIMEOUT_MS 20
#define TX_TSTAMP_POLL       K_MSEC(1)

struct mv_dsa_tx_tstamp {
	struct k_work_delayable work;
	struct dsa_mv88e6xxx_data *prv;
	struct net_pkt *pkt;
	int64_t start;
	uint16_t seq_id;
	uint8_t port;
};
#endif

BUILD_ASSERT(!IS_ENABLED(CONFIG_DSA_TAG_PROTOCOL_MV88E6XXX) || CONFIG_DSA_TAG_SIZE >= 4,
	     "DSA_TAG_SIZE must be 4 (DSA) or 8 (EDSA)");

struct dsa_mv88e6xxx_data {
	const struct device *sw;
	uint8_t cpu_port;
	int tag_proto;
	struct dsa_tag_mv88e6xxx_data tag_data;
#if defined(MV_DSA_PTP)
	const struct dsa_switch_context *ctx;
	struct k_spinlock tx_lock;
	struct mv_dsa_tx_tstamp tx[DSA_PORT_MAX_COUNT];
#endif
};

#if defined(MV_DSA_PTP)
static const struct device *port_ptp_clock(const struct dsa_switch_context *ctx, uint8_t port)
{
	const struct dsa_port_config *cfg;

	if (port >= DSA_PORT_MAX_COUNT || ctx->iface_user[port] == NULL) {
		return NULL;
	}
	cfg = net_if_get_device(ctx->iface_user[port])->config;

	return cfg->ptp_clock;
}

/* Arrival time stamp of a received event message, from ARR0 or, for
 * Pdelay_Resp, ARR1. The registers hold one stamp each, cleared on use.
 */
static void dsa_mv88e6xxx_rx_tstamp(const struct dsa_switch_context *ctx, uint8_t port,
				    struct net_pkt *pkt, uint8_t msg_type, uint16_t seq_id)
{
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	const struct device *clk = port_ptp_clock(ctx, port);
	uint8_t reg = msg_type == PTP_MSG_PDLY_RES ? PORT_PTP_ARR1_STS : PORT_PTP_ARR0_STS;
	uint16_t ts[4];
	uint64_t ns;

	if (clk == NULL || mv88e6xxx_avb_read(prv->sw, port, reg, ts, ARRAY_SIZE(ts)) < 0 ||
	    (ts[0] & PTP_TS_VALID) == 0) {
		return;
	}

	(void)mv88e6xxx_avb_write(prv->sw, port, reg, 0);

	if ((ts[0] & PTP_TS_STATUS_MASK) != 0 || ts[3] != seq_id) {
		LOG_DBG("port %u: arrival stamp seq %u, frame %u", port, ts[3], seq_id);
		return;
	}

	if (ptp_clock_mv88e6xxx_cyc2ns(clk, ((uint32_t)ts[2] << 16) | ts[1], &ns) == 0) {
		net_pkt_set_timestamp_ns(pkt, ns);
	}
}

static void tx_tstamp_done(struct mv_dsa_tx_tstamp *t)
{
	k_spinlock_key_t key = k_spin_lock(&t->prv->tx_lock);
	struct net_pkt *pkt = t->pkt;

	t->pkt = NULL;
	k_spin_unlock(&t->prv->tx_lock, key);

	if (pkt != NULL) {
		net_pkt_unref(pkt);
	}
}

static void tx_tstamp_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mv_dsa_tx_tstamp *t = CONTAINER_OF(dwork, struct mv_dsa_tx_tstamp, work);
	struct dsa_mv88e6xxx_data *prv = t->prv;
	const struct device *clk = port_ptp_clock(prv->ctx, t->port);
	uint16_t ts[4];
	uint64_t ns;

	if (t->pkt == NULL || clk == NULL ||
	    mv88e6xxx_avb_read(prv->sw, t->port, PORT_PTP_DEP_STS, ts, ARRAY_SIZE(ts)) < 0) {
		tx_tstamp_done(t);
		return;
	}

	if ((ts[0] & PTP_TS_VALID) == 0) {
		if (k_uptime_get() - t->start < TX_TSTAMP_TIMEOUT_MS) {
			k_work_reschedule(dwork, TX_TSTAMP_POLL);
			return;
		}
		LOG_WRN("port %u: no departure time stamp", t->port);
		tx_tstamp_done(t);
		return;
	}

	(void)mv88e6xxx_avb_write(prv->sw, t->port, PORT_PTP_DEP_STS, 0);

	if ((ts[0] & PTP_TS_STATUS_MASK) != 0 || ts[3] != t->seq_id) {
		LOG_WRN("port %u: departure stamp status 0x%04x seq %u, expected %u", t->port,
			ts[0], ts[3], t->seq_id);
	} else if (ptp_clock_mv88e6xxx_cyc2ns(clk, ((uint32_t)ts[2] << 16) | ts[1], &ns) == 0) {
		net_pkt_set_timestamp_ns(t->pkt, ns);
		net_if_call_timestamp_cb(t->pkt);
	}

	tx_tstamp_done(t);
}

/* PTP sequence ID of an L2 PTP frame, as built by the Ethernet L2 */
static int ptp_seq_id(struct net_pkt *pkt, uint16_t *seq_id)
{
	struct net_pkt_cursor backup;
	uint16_t type;
	int ret;

	net_pkt_cursor_backup(pkt, &backup);
	net_pkt_cursor_init(pkt);

	ret = net_pkt_skip(pkt, 2 * NET_ETH_ADDR_LEN);
	if (ret == 0) {
		ret = net_pkt_read_be16(pkt, &type);
	}
	if (ret == 0 && type == NET_ETH_PTYPE_VLAN) {
		ret = net_pkt_skip(pkt, 2);
		if (ret == 0) {
			ret = net_pkt_read_be16(pkt, &type);
		}
	}
	if (ret == 0 && type != ETH_P_1588) {
		ret = -EINVAL;
	}
	if (ret == 0) {
		ret = net_pkt_skip(pkt, 30);
	}
	if (ret == 0) {
		ret = net_pkt_read_be16(pkt, seq_id);
	}

	net_pkt_cursor_restore(pkt, &backup);

	return ret;
}

/* Called before the frame goes to the conduit: remember it, then poll
 * the port's departure registers. One frame in flight per port.
 */
static int dsa_mv88e6xxx_port_txtstamp(const struct device *dev, struct net_pkt *pkt)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	struct mv_dsa_tx_tstamp *t;
	k_spinlock_key_t key;
	uint16_t seq_id;

	if (cfg->ptp_clock == NULL || cfg->port_idx >= DSA_PORT_MAX_COUNT ||
	    ptp_seq_id(pkt, &seq_id) < 0) {
		return 0;
	}

	t = &prv->tx[cfg->port_idx];
	key = k_spin_lock(&prv->tx_lock);
	if (t->pkt != NULL) {
		k_spin_unlock(&prv->tx_lock, key);
		LOG_WRN("port %d: departure time stamp still pending", cfg->port_idx);
		return 0;
	}
	t->pkt = net_pkt_ref(pkt);
	t->seq_id = seq_id;
	t->start = k_uptime_get();
	k_spin_unlock(&prv->tx_lock, key);

	k_work_reschedule(&t->work, TX_TSTAMP_POLL);

	return 0;
}

static int dsa_mv88e6xxx_ptp_setup(const struct dsa_switch_context *ctx)
{
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	int ret = 0;

	prv->ctx = ctx;

	for (uint8_t port = 0; ret == 0 && port < DSA_PORT_MAX_COUNT; port++) {
		prv->tx[port].prv = prv;
		prv->tx[port].port = port;
		k_work_init_delayable(&prv->tx[port].work, tx_tstamp_work);

		if (port_ptp_clock(ctx, port) == NULL) {
			continue;
		}
		/* PTP on, any transportSpecific (1588 and 802.1AS), no IRQs */
		ret = mv88e6xxx_avb_write(prv->sw, port, PORT_PTP_CFG0,
					  PORT_PTP_CFG0_NO_TSPEC_MATCH);
		if (ret == 0) {
			ret = mv88e6xxx_avb_write(prv->sw, port, PORT_PTP_CFG2, 0);
		}
	}

	if (ret < 0) {
		LOG_ERR("port PTP setup failed (%d)", ret);
	}

	return ret;
}
#endif /* MV_DSA_PTP */

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
#if defined(MV_DSA_PTP)
	prv->tag_data.rx_tstamp = dsa_mv88e6xxx_rx_tstamp;
#endif
	ctx->tagger_data = &prv->tag_data;

	return 0;
}

static int dsa_mv88e6xxx_switch_setup(const struct dsa_switch_context *ctx)
{
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	int ret;

	if (!device_is_ready(prv->sw)) {
		LOG_ERR("switch %s not ready", prv->sw->name);
		return -ENODEV;
	}
	if (mv88e6xxx_cpu_port(prv->sw) != prv->cpu_port) {
		LOG_ERR("CPU port %u differs from the switch's marvell,cpu-port %u", prv->cpu_port,
			mv88e6xxx_cpu_port(prv->sw));
		return -EINVAL;
	}

	ret = mv88e6xxx_dsa_enable(prv->sw, prv->tag_proto == DSA_TAG_PROTO_EDSA
						    ? MV88E6XXX_TAG_EDSA
						    : MV88E6XXX_TAG_DSA);
#if defined(MV_DSA_PTP)
	if (ret == 0) {
		ret = dsa_mv88e6xxx_ptp_setup(ctx);
	}
#endif

	return ret;
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
		IF_ENABLED(CONFIG_NET_L2_PTP_TIMESTAMPING,                                         \
			   (.ptp_clock = MV_DSA_PTP_CLOCK(port),)) }; \
	DSA_PORT_INST_INIT(port, n, &dsa_mv88e6xxx_##n##_##port##_config)

#if defined(MV_DSA_PTP)
#define MV_DSA_PTP_CLOCK(port) MV_DSA_PHANDLE_DEV(port, ptp_clock)
#else
#define MV_DSA_PTP_CLOCK(port) NULL
#endif

/* The CPU port is the child with an "ethernet" phandle */
#define MV_DSA_CPU_PORT_REG(port)                                                                  \
	COND_CODE_1(DT_NODE_HAS_PROP(port, ethernet), (DT_REG_ADDR(port)), ())

#define DSA_MV88E6XXX_DEVICE(n)                                                                    \
	static struct dsa_api dsa_mv88e6xxx_api_##n = {                                            \
		.switch_setup = dsa_mv88e6xxx_switch_setup,                                        \
		.connect_tag_protocol = dsa_mv88e6xxx_connect_tag_protocol,                        \
		.get_capabilities = dsa_mv88e6xxx_get_capabilities,                                \
		IF_ENABLED(MV_DSA_PTP, (.port_txtstamp = dsa_mv88e6xxx_port_txtstamp,)) };            \
	static struct dsa_mv88e6xxx_data dsa_mv88e6xxx_data_##n = {                                \
		.sw = DEVICE_DT_GET(DT_INST_PHANDLE(n, marvell_switch)),                           \
		.cpu_port = DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MV_DSA_CPU_PORT_REG),             \
	};                                                                                         \
	DSA_SWITCH_INST_INIT(n, &dsa_mv88e6xxx_api_##n, &dsa_mv88e6xxx_data_##n, MV_DSA_PORT_INIT);

DT_INST_FOREACH_STATUS_OKAY(DSA_MV88E6XXX_DEVICE)
