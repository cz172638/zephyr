/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell DSA/EDSA tagger: tag and untag frames between a fake conduit
 * and fake user port interfaces.
 */

#include <zephyr/ztest.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/net/dsa_tag.h>

#include "../../../../drivers/ethernet/dsa/dsa_tag_mv88e6xxx.h"

static const uint8_t addrs[12] = {
	0x02, 0x11, 0x22, 0x33, 0x44, 0x55, /* DA */
	0x02, 0x66, 0x77, 0x88, 0x99, 0xaa, /* SA */
};
static const uint8_t payload[] = {0x08, 0x00, 0x45, 0x00, 0x00, 0x1c, 0xde, 0xad};

static int fake_send(const struct device *dev, struct net_pkt *pkt)
{
	return 0;
}

static void fake_iface_init(struct net_if *iface)
{
	ethernet_init(iface);
}

static const struct ethernet_api fake_api = {
	.iface_api.init = fake_iface_init,
	.send = fake_send,
};

static struct dsa_switch_context ctx;

static struct dsa_port_config port_cfg[4] = {
	{.port_idx = 0},
	{.port_idx = 1},
	{.port_idx = 2},
	{.port_idx = 3},
};

ETH_NET_DEVICE_INIT(conduit, "conduit", NULL, NULL, NULL, NULL, CONFIG_ETH_INIT_PRIORITY, &fake_api,
		    NET_ETH_MTU);
ETH_NET_DEVICE_INIT(swp2, "swp2", NULL, NULL, &ctx, &port_cfg[2], CONFIG_ETH_INIT_PRIORITY,
		    &fake_api, NET_ETH_MTU);
ETH_NET_DEVICE_INIT(swp3, "swp3", NULL, NULL, &ctx, &port_cfg[3], CONFIG_ETH_INIT_PRIORITY,
		    &fake_api, NET_ETH_MTU);

static struct net_if *if_conduit, *if_swp2, *if_swp3;
static const struct dsa_tag_register *tag_dsa, *tag_edsa;

static void *setup(void)
{
	struct ethernet_context *eth_ctx;

	if_conduit = net_if_lookup_by_dev(DEVICE_GET(conduit));
	if_swp2 = net_if_lookup_by_dev(DEVICE_GET(swp2));
	if_swp3 = net_if_lookup_by_dev(DEVICE_GET(swp3));
	zassert_not_null(if_conduit);
	zassert_not_null(if_swp2);
	zassert_not_null(if_swp3);

	ctx.iface_user[2] = if_swp2;
	ctx.iface_user[3] = if_swp3;
	ctx.iface_conduit = if_conduit;
	ctx.num_ports = 4;
	eth_ctx = net_if_l2_data(if_conduit);
	eth_ctx->dsa_switch_ctx = &ctx;
	eth_ctx->dsa_port = DSA_CONDUIT_PORT;

	STRUCT_SECTION_FOREACH(dsa_tag_register, t) {
		if (t->proto == DSA_TAG_PROTO_DSA) {
			tag_dsa = t;
		} else if (t->proto == DSA_TAG_PROTO_EDSA) {
			tag_edsa = t;
		}
	}
	zassert_not_null(tag_dsa);
	zassert_not_null(tag_edsa);

	return NULL;
}

static struct net_pkt *rx_pkt(const uint8_t *hdr, size_t hlen)
{
	struct net_pkt *pkt =
		net_pkt_rx_alloc_with_buffer(if_conduit, 64, NET_AF_UNSPEC, 0, K_NO_WAIT);

	zassert_not_null(pkt);
	zassert_ok(net_pkt_write(pkt, addrs, sizeof(addrs)));
	zassert_ok(net_pkt_write(pkt, hdr, hlen));
	zassert_ok(net_pkt_write(pkt, payload, sizeof(payload)));
	return pkt;
}

static void check_frame(struct net_pkt *pkt, const uint8_t *mid, size_t mlen)
{
	uint8_t buf[64];
	size_t len = net_pkt_get_len(pkt);

	zassert_equal(len, sizeof(addrs) + mlen + sizeof(payload), "length %zu", len);
	net_pkt_cursor_init(pkt);
	zassert_ok(net_pkt_read(pkt, buf, len));
	zassert_mem_equal(buf, addrs, sizeof(addrs));
	zassert_mem_equal(&buf[sizeof(addrs)], mid, mlen);
	zassert_mem_equal(&buf[sizeof(addrs) + mlen], payload, sizeof(payload));
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_dsa_forward_untagged)
{
	const uint8_t tag[] = {0xc0, 2 << 3, 0x00, 0x00};
	struct net_pkt *pkt = rx_pkt(tag, sizeof(tag));

	zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_swp2);
	check_frame(pkt, NULL, 0);
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_dsa_tagged_cfi)
{
	/* FORWARD, tagged, port 3, CFI, PCP 5, VID 0x064 */
	const uint8_t tag[] = {0xe0, (3 << 3) | 1, 0xa0, 0x64};
	const uint8_t vlan[] = {0x81, 0x00, 0xb0, 0x64};
	struct net_pkt *pkt = rx_pkt(tag, sizeof(tag));

	zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_swp3);
	check_frame(pkt, vlan, sizeof(vlan));
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_edsa_to_cpu)
{
	/* TO_CPU, port 2, code 0 (management trap, e.g. BPDU) */
	const uint8_t tag[] = {0xda, 0xda, 0x00, 0x00, 0x00, 2 << 3, 0x00, 0x00};
	struct net_pkt *pkt = rx_pkt(tag, sizeof(tag));

	zassert_equal_ptr(tag_edsa->recv(if_conduit, pkt), if_swp2);
	check_frame(pkt, NULL, 0);
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_edsa_tagged)
{
	const uint8_t tag[] = {0xda, 0xda, 0x00, 0x00, 0xe0, 3 << 3, 0x20, 0x01};
	const uint8_t vlan[] = {0x81, 0x00, 0x20, 0x01};
	struct net_pkt *pkt = rx_pkt(tag, sizeof(tag));

	zassert_equal_ptr(tag_edsa->recv(if_conduit, pkt), if_swp3);
	check_frame(pkt, vlan, sizeof(vlan));
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_rejects)
{
	/* TO_CPU frame2reg, unknown port, FROM_CPU, wrong EDSA EtherType */
	const uint8_t f2r[] = {0x00, (2 << 3) | 0x0, 0x10, 0x00};
	const uint8_t noport[] = {0xc0, 5 << 3, 0x00, 0x00};
	const uint8_t fromcpu[] = {0x40, 2 << 3, 0x00, 0x00};
	const uint8_t badtype[] = {0x88, 0xa8, 0x00, 0x00, 0xc0, 2 << 3, 0x00, 0x00};
	const uint8_t *hdrs[] = {f2r, noport, fromcpu};
	struct net_pkt *pkt;

	for (int i = 0; i < ARRAY_SIZE(hdrs); i++) {
		pkt = rx_pkt(hdrs[i], 4);
		zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_conduit, "case %d", i);
		check_frame(pkt, hdrs[i], 4);
		net_pkt_unref(pkt);
	}

	pkt = rx_pkt(badtype, sizeof(badtype));
	zassert_equal_ptr(tag_edsa->recv(if_conduit, pkt), if_conduit);
	net_pkt_unref(pkt);
}

/* As the Ethernet L2 builds it: header in its own fragment */
static struct net_pkt *tx_pkt(struct net_if *iface, bool vlan)
{
	const uint8_t vtag[] = {0x81, 0x00, 0xb0, 0x64};
	struct net_pkt *pkt = net_pkt_alloc_on_iface(iface, K_NO_WAIT);
	struct net_buf *hdr, *data;

	zassert_not_null(pkt);
	hdr = net_pkt_get_frag(pkt, 18, K_NO_WAIT);
	data = net_pkt_get_frag(pkt, sizeof(payload) - 2, K_NO_WAIT);
	zassert_not_null(hdr);
	zassert_not_null(data);

	net_buf_add_mem(hdr, addrs, sizeof(addrs));
	if (vlan) {
		net_buf_add_mem(hdr, vtag, sizeof(vtag));
	}
	net_buf_add_mem(hdr, payload, 2);
	net_buf_add_mem(data, &payload[2], sizeof(payload) - 2);
	net_pkt_append_buffer(pkt, hdr);
	net_pkt_append_buffer(pkt, data);
	return pkt;
}

ZTEST(dsa_tag_mv88e6xxx, test_tx_dsa_untagged)
{
	const uint8_t tag[] = {0x40, 3 << 3, 0x00, 0x00};
	struct net_pkt *pkt = tx_pkt(if_swp3, false);

	zassert_equal_ptr(tag_dsa->xmit(if_swp3, pkt), pkt);
	check_frame(pkt, tag, sizeof(tag));
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_tx_dsa_vlan)
{
	/* 802.1Q PCP 5, CFI, VID 0x064 folded into the tag */
	const uint8_t tag[] = {0x60, (2 << 3) | 1, 0xa0, 0x64};
	struct net_pkt *pkt = tx_pkt(if_swp2, true);

	zassert_equal_ptr(tag_dsa->xmit(if_swp2, pkt), pkt);
	check_frame(pkt, tag, sizeof(tag));
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_tx_edsa)
{
	const uint8_t tag[] = {0xda, 0xda, 0x00, 0x00, 0x40, 2 << 3, 0x00, 0x00};
	struct net_pkt *pkt = tx_pkt(if_swp2, false);

	zassert_equal_ptr(tag_edsa->xmit(if_swp2, pkt), pkt);
	check_frame(pkt, tag, sizeof(tag));
	net_pkt_unref(pkt);
}

ZTEST(dsa_tag_mv88e6xxx, test_roundtrip_edsa_vlan)
{
	/* What the CPU sends tagged to port 3 comes back the same */
	const uint8_t vtag[] = {0x81, 0x00, 0xb0, 0x64};
	struct net_pkt *pkt = tx_pkt(if_swp3, true);
	uint8_t buf[64];
	size_t len;

	zassert_equal_ptr(tag_edsa->xmit(if_swp3, pkt), pkt);
	len = net_pkt_get_len(pkt);
	net_pkt_cursor_init(pkt);
	zassert_ok(net_pkt_read(pkt, buf, len));
	net_pkt_unref(pkt);

	/* The switch would turn FROM_CPU into FORWARD on the way back */
	buf[16] = (buf[16] & 0x3f) | 0xc0;
	pkt = net_pkt_rx_alloc_with_buffer(if_conduit, len, NET_AF_UNSPEC, 0, K_NO_WAIT);
	zassert_ok(net_pkt_write(pkt, buf, len));
	zassert_equal_ptr(tag_edsa->recv(if_conduit, pkt), if_swp3);
	check_frame(pkt, vtag, sizeof(vtag));
	net_pkt_unref(pkt);
}

static struct {
	int calls;
	uint8_t port;
	uint8_t msg;
	uint16_t seq;
	struct net_pkt *pkt;
} rx_ts;

static void fake_rx_tstamp(const struct dsa_switch_context *c, uint8_t port, struct net_pkt *pkt,
			   uint8_t msg_type, uint16_t seq_id)
{
	rx_ts.calls++;
	rx_ts.port = port;
	rx_ts.msg = msg_type;
	rx_ts.seq = seq_id;
	rx_ts.pkt = pkt;
}

static struct dsa_tag_mv88e6xxx_data tag_data = {.rx_tstamp = fake_rx_tstamp};

/* DA SA tag 88f7 + 34 byte PTP header */
static struct net_pkt *rx_ptp(const uint8_t *tag, size_t tlen, bool vlan, uint8_t msg, uint16_t seq)
{
	uint8_t hdr[34] = {0};
	const uint8_t vtag[] = {0x81, 0x00, 0x00, 0x05};
	const uint8_t etype[] = {0x88, 0xf7};
	struct net_pkt *pkt =
		net_pkt_rx_alloc_with_buffer(if_conduit, 96, NET_AF_UNSPEC, 0, K_NO_WAIT);

	hdr[0] = 0x10 | msg; /* transportSpecific 1 (802.1AS) */
	hdr[1] = 0x02;
	hdr[30] = seq >> 8;
	hdr[31] = seq & 0xff;

	zassert_not_null(pkt);
	zassert_ok(net_pkt_write(pkt, addrs, sizeof(addrs)));
	zassert_ok(net_pkt_write(pkt, tag, tlen));
	if (vlan) {
		zassert_ok(net_pkt_write(pkt, vtag, sizeof(vtag)));
	}
	zassert_ok(net_pkt_write(pkt, etype, sizeof(etype)));
	zassert_ok(net_pkt_write(pkt, hdr, sizeof(hdr)));
	return pkt;
}

ZTEST(dsa_tag_mv88e6xxx, test_rx_ptp_hook)
{
	const uint8_t fwd2[] = {0xc0, 2 << 3, 0x00, 0x00};
	const uint8_t trap3[] = {0x00, 3 << 3, 0x00, 0x00};
	struct net_pkt *pkt;

	ctx.tagger_data = &tag_data;
	memset(&rx_ts, 0, sizeof(rx_ts));

	/* Sync on port 2 */
	pkt = rx_ptp(fwd2, sizeof(fwd2), false, 0, 0x1234);
	zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_swp2);
	zassert_equal(rx_ts.calls, 1);
	zassert_equal(rx_ts.port, 2);
	zassert_equal(rx_ts.msg, 0);
	zassert_equal(rx_ts.seq, 0x1234);
	zassert_equal_ptr(rx_ts.pkt, pkt);
	net_pkt_unref(pkt);

	/* Pdelay_Resp behind an 802.1Q header on port 3 */
	pkt = rx_ptp(trap3, sizeof(trap3), true, 3, 0xbeef);
	zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_swp3);
	zassert_equal(rx_ts.calls, 2);
	zassert_equal(rx_ts.port, 3);
	zassert_equal(rx_ts.msg, 3);
	zassert_equal(rx_ts.seq, 0xbeef);
	net_pkt_unref(pkt);

	/* Follow_Up is a general message: no time stamp */
	pkt = rx_ptp(fwd2, sizeof(fwd2), false, 8, 1);
	zassert_equal_ptr(tag_dsa->recv(if_conduit, pkt), if_swp2);
	zassert_equal(rx_ts.calls, 2);
	net_pkt_unref(pkt);

	ctx.tagger_data = NULL;
}

ZTEST_SUITE(dsa_tag_mv88e6xxx, NULL, setup, NULL, NULL, NULL);
