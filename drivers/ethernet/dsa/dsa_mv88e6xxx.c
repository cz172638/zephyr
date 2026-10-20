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
 * Ports are standalone, every user port forwards only to the CPU port,
 * until they join an Ethernet bridge: then the switch forwards between
 * the bridged ports itself (port based VLAN map, own address database
 * with a static entry for the bridge address on the CPU port), as Linux
 * does for a bridge without VLAN filtering.
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
#include <zephyr/net/phy.h>
#include <zephyr/net/net_if.h>
#include <zephyr/sys/util.h>
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

/* Qav block, per port: idle slope rate and hi limit of queue q */
#define QAV_PORT_RATE(q)    ((q) << 1)
#define QAV_PORT_HILIMIT(q) (((q) << 1) + 1)
/* 88E6352 family only: undocumented per port shaper enable (Linux, Luke
 * Howard's CBS series)
 */
#define QAV6352_PORT_CFG        0x08
#define QAV6352_PORT_CFG_ENABLE BIT(15)

/* Port egress rate control 2: scheduling, 3 = strict priority on all
 * queues, the reset value 0 is weighted round robin 8:4:2:1
 */
#define PORT_EGRESS_RATE_CTL2       0x0a
#define PORT_EGRESS_RATE_CTL2_SCHED GENMASK(13, 12)
#define SCHED_ALL_STRICT            (3 << 12)

/* Global 1 IEEE-PRI: 2 bits of queue per frame priority */
#define G1_IEEE_PRI 0x18

#define QAV_QUEUES 4

/* Largest frame that can delay a shaped one, 802.1Q 34.4: maximum frame
 * plus preamble and interframe gap, in bytes
 */
#define QAV_MAX_INTERFERENCE 1542

/* Zephyr priority -> traffic class for four classes (net_tc_mapping.h);
 * as the switch queue it keeps queue N = traffic class N
 */
#if defined(CONFIG_NET_TC_MAPPING_SR_CLASS_A_AND_B)
#define QAV_PRIO2QUEUE {0, 0, 2, 3, 1, 1, 1, 1}
#elif defined(CONFIG_NET_TC_MAPPING_SR_CLASS_B_ONLY)
#define QAV_PRIO2QUEUE {0, 0, 3, 0, 1, 1, 2, 2}
#else
#define QAV_PRIO2QUEUE {0, 0, 1, 1, 2, 2, 3, 3}
#endif

struct mv_qav_info {
	uint32_t rate_unit; /* bit/s per rate step */
	uint16_t rate_mask;
	uint16_t hilimit_mask;
	bool port_enable;   /* 88E6352: QAV6352_PORT_CFG_ENABLE */
};

/* Linux mv88e6341_qav_info / mv88e6352_qav_info */
static const struct mv_qav_info qav_6341 = {
	.rate_unit = 64000, .rate_mask = 0xffff, .hilimit_mask = 0x3fff};
static const struct mv_qav_info qav_6352 = {
	.rate_unit = 32000, .rate_mask = 0x7fff, .hilimit_mask = 0x7fff, .port_enable = true};

struct mv_qav_port {
	uint32_t idle_slope[QAV_QUEUES]; /* bit/s */
	uint8_t enabled;                 /* queue bitmap */
	bool active;                     /* shaping, strict scheduling */
	uint16_t sched;                  /* scheduling before shaping */
};

BUILD_ASSERT(!IS_ENABLED(CONFIG_DSA_TAG_PROTOCOL_MV88E6XXX) || CONFIG_DSA_TAG_SIZE >= 4,
	     "DSA_TAG_SIZE must be 4 (DSA) or 8 (EDSA)");

struct dsa_mv88e6xxx_data {
	const struct device *sw;
	uint8_t cpu_port;
	int tag_proto;
	struct dsa_tag_mv88e6xxx_data tag_data;
	const struct mv_qav_info *qav;
	struct k_mutex qav_lock;
	struct mv_qav_port qav_port[DSA_PORT_MAX_COUNT];
	struct k_mutex br_lock;
	struct net_if *bridge;
	uint16_t br_ports;
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

static const struct mv_qav_info *qav_info(uint16_t product)
{
	switch (product) {
	case 0x341: /* 88E6341 */
	case 0x141: /* 88E6141 */
		return &qav_6341;
	case 0x172: /* 88E6172 */
	case 0x176: /* 88E6176 */
	case 0x240: /* 88E6240 */
	case 0x320: /* 88E6320 */
	case 0x321: /* 88E6321 */
	case 0x350: /* 88E6350 */
	case 0x351: /* 88E6351 */
	case 0x352: /* 88E6352 */
		return &qav_6352;
	default:
		/* 88E6390 family: eight queues, scheduling elsewhere */
		return NULL;
	}
}

static uint64_t port_link_bps(const struct device *dev)
{
	const struct dsa_port_config *cfg = dev->config;
	struct phy_link_state st = {0};

	if (cfg->phy_dev != NULL && phy_get_link_state(cfg->phy_dev, &st) == 0 && st.is_up) {
		if (PHY_LINK_IS_SPEED_10M(st.speed)) {
			return 10000000ULL;
		}
		if (PHY_LINK_IS_SPEED_100M(st.speed)) {
			return 100000000ULL;
		}
	}

	return 1000000000ULL;
}

/* hiCredit for the queue, as 802.1Q L.3 for the highest SR class: one
 * maximum interfering frame sent at the port rate
 */
static uint16_t qav_hilimit(const struct mv_qav_info *qav, uint32_t idle_slope, uint64_t link)
{
	uint64_t h = DIV_ROUND_UP((uint64_t)QAV_MAX_INTERFERENCE * idle_slope, link);

	return CLAMP(h, 1, qav->hilimit_mask);
}

/* Program the shapers of a port from its state; with prv->qav_lock held */
static int qav_apply(struct dsa_mv88e6xxx_data *prv, const struct device *dev, uint8_t port)
{
	const struct mv_qav_info *qav = prv->qav;
	struct mv_qav_port *qp = &prv->qav_port[port];
	uint64_t link = port_link_bps(dev);
	bool active = false;
	int ret = 0;

	for (int q = 0; ret == 0 && q < QAV_QUEUES; q++) {
		uint32_t bps = qp->idle_slope[q];
		uint16_t rate = 0;
		uint16_t hilimit = qav->hilimit_mask;

		if ((qp->enabled & BIT(q)) != 0 && bps != 0) {
			rate = CLAMP(DIV_ROUND_UP(bps, qav->rate_unit), 1, qav->rate_mask);
			hilimit = qav_hilimit(qav, bps, link);
			active = true;
		}
		/* Hi limit first, the shaper starts with the rate */
		ret = mv88e6xxx_avb_block_write(prv->sw, MV88E6XXX_AVB_BLOCK_QAV, port,
						QAV_PORT_HILIMIT(q), hilimit);
		if (ret == 0) {
			ret = mv88e6xxx_avb_block_write(prv->sw, MV88E6XXX_AVB_BLOCK_QAV, port,
							QAV_PORT_RATE(q), rate);
		}
	}

	if (ret == 0 && qav->port_enable) {
		uint16_t cfg;

		ret = mv88e6xxx_avb_block_read(prv->sw, MV88E6XXX_AVB_BLOCK_QAV, port,
					       QAV6352_PORT_CFG, &cfg, 1);
		if (ret == 0) {
			cfg = active ? cfg | QAV6352_PORT_CFG_ENABLE
				     : cfg & ~QAV6352_PORT_CFG_ENABLE;
			ret = mv88e6xxx_avb_block_write(prv->sw, MV88E6XXX_AVB_BLOCK_QAV, port,
							QAV6352_PORT_CFG, cfg);
		}
	}

	/* Shaped queues need strict priority between the queues */
	if (ret == 0 && active != qp->active) {
		uint16_t old;

		ret = mv88e6xxx_reg_rmw(prv->sw, MV88E6XXX_ADDR_PORT(port), PORT_EGRESS_RATE_CTL2,
					PORT_EGRESS_RATE_CTL2_SCHED,
					active ? SCHED_ALL_STRICT : qp->sched, &old);
		if (ret == 0) {
			if (active) {
				qp->sched = old & PORT_EGRESS_RATE_CTL2_SCHED;
			}
			qp->active = active;
		}
	}

	if (ret < 0) {
		LOG_ERR("port %u: Qav setup failed (%d)", port, ret);
	}

	return ret;
}

static int qav_port(const struct device *dev, const struct ethernet_qav_param *p,
		    struct dsa_mv88e6xxx_data **prv, uint8_t *port)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;

	*prv = ctx->prv_data;
	*port = cfg->port_idx;

	if ((*prv)->qav == NULL || cfg->ethernet_connection != NULL ||
	    cfg->port_idx >= DSA_PORT_MAX_COUNT) {
		return -ENOTSUP;
	}
	if (p->queue_id < 0 || p->queue_id >= QAV_QUEUES) {
		return -EINVAL;
	}

	return 0;
}

static int dsa_mv88e6xxx_set_qav(const struct device *dev, const struct ethernet_qav_param *p)
{
	struct dsa_mv88e6xxx_data *prv;
	struct mv_qav_port *qp;
	uint8_t port;
	int ret = qav_port(dev, p, &prv, &port);

	if (ret < 0) {
		return ret;
	}
	qp = &prv->qav_port[port];

	k_mutex_lock(&prv->qav_lock, K_FOREVER);

	switch (p->type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		WRITE_BIT(qp->enabled, p->queue_id, p->enabled);
		break;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		if (p->idle_slope > port_link_bps(dev) ||
		    DIV_ROUND_UP(p->idle_slope, prv->qav->rate_unit) > prv->qav->rate_mask) {
			ret = -EINVAL;
			break;
		}
		qp->idle_slope[p->queue_id] = p->idle_slope;
		break;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		if (p->delta_bandwidth > 100) {
			ret = -EINVAL;
			break;
		}
		qp->idle_slope[p->queue_id] = port_link_bps(dev) / 100 * p->delta_bandwidth;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	if (ret == 0) {
		ret = qav_apply(prv, dev, port);
	}

	k_mutex_unlock(&prv->qav_lock);

	return ret;
}

static int dsa_mv88e6xxx_get_qav(const struct device *dev, struct ethernet_qav_param *p)
{
	struct dsa_mv88e6xxx_data *prv;
	const struct mv_qav_port *qp;
	uint16_t rate;
	uint8_t port;
	int ret = qav_port(dev, p, &prv, &port);

	if (ret < 0) {
		return ret;
	}
	qp = &prv->qav_port[port];

	switch (p->type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		p->enabled = (qp->enabled & BIT(p->queue_id)) != 0;
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		p->idle_slope = qp->idle_slope[p->queue_id];
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_OPER_IDLE_SLOPE:
		/* What the hardware runs with, from its rate register */
		ret = mv88e6xxx_avb_block_read(prv->sw, MV88E6XXX_AVB_BLOCK_QAV, port,
					       QAV_PORT_RATE(p->queue_id), &rate, 1);
		if (ret == 0) {
			p->oper_idle_slope = (rate & prv->qav->rate_mask) * prv->qav->rate_unit;
		}
		return ret;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		p->delta_bandwidth = DIV_ROUND_UP((uint64_t)qp->idle_slope[p->queue_id] * 100,
						  port_link_bps(dev));
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_TRAFFIC_CLASS:
		p->traffic_class = p->queue_id;
		return 0;
	default:
		return -ENOTSUP;
	}
}

/* The hi limits depend on the link speed */
static void dsa_mv88e6xxx_phylink_change(const struct device *phydev,
					 struct phy_link_state *state, const struct device *dev)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;

	ARG_UNUSED(phydev);

	if (prv->qav == NULL || !state->is_up || cfg->port_idx >= DSA_PORT_MAX_COUNT) {
		return;
	}

	k_mutex_lock(&prv->qav_lock, K_FOREVER);
	if (prv->qav_port[cfg->port_idx].active) {
		(void)qav_apply(prv, dev, cfg->port_idx);
	}
	k_mutex_unlock(&prv->qav_lock);
}

static int dsa_mv88e6xxx_qav_setup(struct dsa_mv88e6xxx_data *prv)
{
	static const uint8_t prio2queue[] = QAV_PRIO2QUEUE;
	uint16_t map = 0;
	int ret;

	k_mutex_init(&prv->qav_lock);
	prv->qav = qav_info(mv88e6xxx_product(prv->sw));
	if (prv->qav == NULL) {
		return 0;
	}

	for (int prio = 0; prio < ARRAY_SIZE(prio2queue); prio++) {
		map |= prio2queue[prio] << (2 * prio);
	}
	ret = mv88e6xxx_reg_rmw(prv->sw, MV88E6XXX_ADDR_GLOBAL1, G1_IEEE_PRI, 0xffff, map, NULL);
	if (ret < 0) {
		LOG_ERR("IEEE-PRI setup failed (%d)", ret);
		prv->qav = NULL;
	}

	return ret;
}

/* Port based VLAN map and default address database */
#define PORT_BASE_VLAN     0x06
#define PORT_BASE_VLAN_MAP GENMASK(10, 0)
#define PORT_BASE_VLAN_FID GENMASK(15, 12)

/* Database 0 for standalone ports, 1 for the bridge (Linux uses the same) */
#define FID_STANDALONE 0
#define FID_BRIDGED    1

static int br_port_set(struct dsa_mv88e6xxx_data *prv, uint8_t port, uint16_t map, uint16_t fid)
{
	return mv88e6xxx_reg_rmw(prv->sw, MV88E6XXX_ADDR_PORT(port), PORT_BASE_VLAN,
				 PORT_BASE_VLAN_MAP | PORT_BASE_VLAN_FID,
				 (map & PORT_BASE_VLAN_MAP) | FIELD_PREP(PORT_BASE_VLAN_FID, fid),
				 NULL);
}

/* Every bridged port may send to the CPU and the other bridged ports */
static int br_update(struct dsa_mv88e6xxx_data *prv)
{
	int ret = 0;

	for (uint8_t port = 0; ret == 0 && port < DSA_PORT_MAX_COUNT; port++) {
		if (prv->br_ports & BIT(port)) {
			ret = br_port_set(prv, port, BIT(prv->cpu_port) | (prv->br_ports & ~BIT(port)),
					  FID_BRIDGED);
		}
	}

	return ret;
}

static int dsa_mv88e6xxx_port_bridge_join(const struct device *dev, struct net_if *br)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	struct net_linkaddr *ll = net_if_get_link_addr(br);
	uint8_t port = cfg->port_idx;
	int ret;

	if (cfg->ethernet_connection != NULL || port >= DSA_PORT_MAX_COUNT) {
		return -EINVAL;
	}

	k_mutex_lock(&prv->br_lock, K_FOREVER);

	/* One address database for bridged ports: one bridge */
	if (prv->bridge != NULL && prv->bridge != br) {
		k_mutex_unlock(&prv->br_lock);
		return -EBUSY;
	}

	prv->br_ports |= BIT(port);
	ret = br_update(prv);
	/* Frames from the CPU are not learned: the bridge address is static */
	if (ret == 0 && prv->bridge == NULL && ll != NULL && ll->len == NET_ETH_ADDR_LEN) {
		ret = mv88e6xxx_atu_load(prv->sw, FID_BRIDGED, ll->addr, BIT(prv->cpu_port),
					 MV88E6XXX_ATU_STATE_UC_STATIC);
	}
	if (ret == 0) {
		ret = mv88e6xxx_atu_flush_dynamic(prv->sw);
	}

	if (ret == 0) {
		prv->bridge = br;
		LOG_INF("port %u joins the bridge, ports 0x%02x", port, prv->br_ports);
	} else {
		prv->br_ports &= ~BIT(port);
		(void)br_port_set(prv, port, BIT(prv->cpu_port), FID_STANDALONE);
		(void)br_update(prv);
		LOG_ERR("port %u: bridge offload failed (%d)", port, ret);
	}

	k_mutex_unlock(&prv->br_lock);

	return ret;
}

static void dsa_mv88e6xxx_port_bridge_leave(const struct device *dev, struct net_if *br)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	struct net_linkaddr *ll = net_if_get_link_addr(br);
	uint8_t port = cfg->port_idx;

	k_mutex_lock(&prv->br_lock, K_FOREVER);

	if (prv->bridge == br && (prv->br_ports & BIT(port)) != 0) {
		prv->br_ports &= ~BIT(port);
		(void)br_port_set(prv, port, BIT(prv->cpu_port), FID_STANDALONE);
		(void)br_update(prv);
		if (prv->br_ports == 0) {
			if (ll != NULL && ll->len == NET_ETH_ADDR_LEN) {
				(void)mv88e6xxx_atu_load(prv->sw, FID_BRIDGED, ll->addr, 0,
							 MV88E6XXX_ATU_STATE_UNUSED);
			}
			prv->bridge = NULL;
		}
		(void)mv88e6xxx_atu_flush_dynamic(prv->sw);
		LOG_INF("port %u leaves the bridge, ports 0x%02x", port, prv->br_ports);
	}

	k_mutex_unlock(&prv->br_lock);
}

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
	k_mutex_init(&prv->br_lock);
	if (ret == 0) {
		ret = dsa_mv88e6xxx_qav_setup(prv);
	}
#if defined(MV_DSA_PTP)
	if (ret == 0) {
		ret = dsa_mv88e6xxx_ptp_setup(ctx);
	}
#endif

	return ret;
}

static enum ethernet_hw_caps dsa_mv88e6xxx_get_capabilities(const struct device *dev)
{
	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;
	enum ethernet_hw_caps caps =
		ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE;

	if (prv->qav != NULL && cfg->ethernet_connection == NULL) {
		caps |= ETHERNET_PRIORITY_QUEUES | ETHERNET_QAV;
	}

	return caps;
}

/* The switch does not filter on the user port addresses (standalone
 * ports, promiscuous conduit), so a new address needs no hardware change;
 * the Ethernet management code updates the interface.
 */
static int dsa_mv88e6xxx_set_config(const struct device *dev, enum ethernet_config_type type,
				    const struct ethernet_config *config)
{
	const struct dsa_port_config *cfg = dev->config;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		return cfg->ethernet_connection == NULL ? 0 : -ENOTSUP;
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return dsa_mv88e6xxx_set_qav(dev, &config->qav_param);
	default:
		return -ENOTSUP;
	}
}

static int dsa_mv88e6xxx_get_config(const struct device *dev, enum ethernet_config_type type,
				    struct ethernet_config *config)
{
	struct dsa_switch_context *ctx = dev->data;
	struct dsa_mv88e6xxx_data *prv = ctx->prv_data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PRIORITY_QUEUES_NUM:
		if (prv->qav == NULL) {
			return -ENOTSUP;
		}
		config->priority_queues_num = QAV_QUEUES;
		return 0;
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return dsa_mv88e6xxx_get_qav(dev, &config->qav_param);
	default:
		return -ENOTSUP;
	}
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
		.label = DT_PROP_OR(port, label, NULL),                                            \
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
		.set_config = dsa_mv88e6xxx_set_config,                                            \
		.get_config = dsa_mv88e6xxx_get_config,                                            \
		.port_phylink_change = dsa_mv88e6xxx_phylink_change,                               \
		.port_bridge_join = dsa_mv88e6xxx_port_bridge_join,                                \
		.port_bridge_leave = dsa_mv88e6xxx_port_bridge_leave,                              \
		IF_ENABLED(MV_DSA_PTP, (.port_txtstamp = dsa_mv88e6xxx_port_txtstamp,)) };            \
	static struct dsa_mv88e6xxx_data dsa_mv88e6xxx_data_##n = {                                \
		.sw = DEVICE_DT_GET(DT_INST_PHANDLE(n, marvell_switch)),                           \
		.cpu_port = DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MV_DSA_CPU_PORT_REG),             \
	};                                                                                         \
	DSA_SWITCH_INST_INIT(n, &dsa_mv88e6xxx_api_##n, &dsa_mv88e6xxx_data_##n, MV_DSA_PORT_INIT);

DT_INST_FOREACH_STATUS_OKAY(DSA_MV88E6XXX_DEVICE)
