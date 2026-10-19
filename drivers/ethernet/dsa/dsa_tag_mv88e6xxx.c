/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell DSA and Ethertype DSA (EDSA) tags, following Linux
 * net/dsa/tag_dsa.c.
 *
 * DSA:  DA SA | tag[4] | EtherType ...
 * EDSA: DA SA | 0xdada 0x0000 tag[4] | EtherType ...
 *
 * tag[0]: 7:6 command (0 TO_CPU, 1 FROM_CPU, 2 TO_SNIFFER, 3 FORWARD),
 *         5 frame was/is 802.1Q tagged, 4:0 switch device
 * tag[1]: 7:3 port, 2:1 TO_CPU code high bits / 2 FORWARD trunk, 0 CFI
 * tag[2]: 7:5 PCP, 4 TO_CPU code low bit, 3:0 VID 11:8
 * tag[3]: VID 7:0
 *
 * Frames from the CPU are sent FROM_CPU to the port of the interface; an
 * 802.1Q header is folded into the tag, as on received tagged frames the
 * tag is unfolded back into an 802.1Q header.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dsa_tag_mv88e6xxx, CONFIG_NET_DSA_LOG_LEVEL);

#include <string.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/net/dsa_tag.h>
#include <zephyr/sys/byteorder.h>

#define DSA_HLEN       4
#define EDSA_HLEN      8
#define EDSA_ETHERTYPE 0xdada
#define ADDRS_LEN      (2 * NET_ETH_ADDR_LEN)

#define CMD_TO_CPU   0
#define CMD_FROM_CPU 1
#define CMD_FORWARD  3

#define TAG0_CMD(t)    ((t)[0] >> 6)
#define TAG0_TAGGED    BIT(5)
#define TAG1_PORT(t)   (((t)[1] >> 3) & 0x1f)
#define TAG1_TRUNK     BIT(2)
#define TAG1_CFI       BIT(0)
#define TAG2_CFI_POS   BIT(4)
#define TO_CPU_CODE(t) (((t)[1] & 0x6) | (((t)[2] >> 4) & 1))

#define CODE_FRAME2REG 1

static struct net_if *mv_tag_recv(struct net_if *iface, struct net_pkt *pkt, bool edsa)
{
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	const struct dsa_switch_context *ctx = eth_ctx->dsa_switch_ctx;
	struct net_buf *frag = pkt->frags;
	size_t hlen = edsa ? EDSA_HLEN : DSA_HLEN;
	uint8_t *data;
	uint8_t *tag;
	struct net_if *dst;
	size_t drop;
	uint8_t port;

	if (frag == NULL || frag->len < ADDRS_LEN + hlen) {
		return iface;
	}

	data = frag->data;
	if (edsa && sys_get_be16(&data[ADDRS_LEN]) != EDSA_ETHERTYPE) {
		return iface;
	}
	tag = &data[ADDRS_LEN + hlen - DSA_HLEN];

	switch (TAG0_CMD(tag)) {
	case CMD_TO_CPU:
		if (TO_CPU_CODE(tag) == CODE_FRAME2REG) {
			return iface;
		}
		break;
	case CMD_FORWARD:
		if (tag[1] & TAG1_TRUNK) {
			return iface;
		}
		break;
	default:
		return iface;
	}

	port = TAG1_PORT(tag);
	if (port >= DSA_PORT_MAX_COUNT || ctx->iface_user[port] == NULL) {
		LOG_DBG("no interface for switch port %u", port);
		return iface;
	}
	dst = ctx->iface_user[port];

	if (tag[0] & TAG0_TAGGED) {
		/* Rebuild the 802.1Q header in place of the tag */
		uint8_t pcp_vid = tag[2] & ~TAG2_CFI_POS;

		if (tag[1] & TAG1_CFI) {
			pcp_vid |= TAG2_CFI_POS;
		}
		sys_put_be16(NET_ETH_PTYPE_VLAN, &tag[0]);
		tag[2] = pcp_vid;
		drop = hlen - DSA_HLEN;
	} else {
		drop = hlen;
	}

	if (drop != 0) {
		memmove(&data[drop], data, ADDRS_LEN);
		net_buf_pull(frag, drop);
	}

	return dst;
}

static struct net_pkt *mv_tag_xmit(struct net_if *iface, struct net_pkt *pkt, bool edsa)
{
	const struct device *dev = net_if_get_device(iface);
	const struct dsa_port_config *cfg = dev->config;
	struct net_buf *frag = pkt->frags;
	size_t hlen = edsa ? EDSA_HLEN : DSA_HLEN;
	struct net_buf *hdr;
	uint8_t *tag;
	bool vlan;

	if (frag == NULL || frag->len < ADDRS_LEN + sizeof(uint16_t)) {
		LOG_ERR("short Ethernet header");
		return pkt;
	}

	vlan = sys_get_be16(&frag->data[ADDRS_LEN]) == NET_ETH_PTYPE_VLAN &&
	       frag->len >= ADDRS_LEN + 2 * sizeof(uint16_t);

	hdr = net_buf_alloc_len(net_buf_pool_get(pkt->buffer->pool_id), ADDRS_LEN + hlen,
				K_NO_WAIT);
	if (hdr == NULL) {
		LOG_ERR("no buffer for the DSA tag");
		return pkt;
	}

	net_buf_add_mem(hdr, frag->data, ADDRS_LEN);
	if (edsa) {
		net_buf_add_be16(hdr, EDSA_ETHERTYPE);
		net_buf_add_be16(hdr, 0);
	}
	tag = net_buf_add(hdr, DSA_HLEN);

	tag[0] = CMD_FROM_CPU << 6;
	tag[1] = (cfg->port_idx & 0x1f) << 3;
	if (vlan) {
		/* Fold the 802.1Q header into the tag, CFI moves to tag[1] */
		tag[0] |= TAG0_TAGGED;
		tag[2] = frag->data[ADDRS_LEN + 2];
		tag[3] = frag->data[ADDRS_LEN + 3];
		if (tag[2] & TAG2_CFI_POS) {
			tag[1] |= TAG1_CFI;
			tag[2] &= ~TAG2_CFI_POS;
		}
		net_buf_pull(frag, ADDRS_LEN + 2 * sizeof(uint16_t));
	} else {
		tag[2] = 0;
		tag[3] = 0;
		net_buf_pull(frag, ADDRS_LEN);
	}

	hdr->frags = frag;
	pkt->frags = hdr;
	net_pkt_cursor_init(pkt);

	return pkt;
}

static struct net_if *dsa_tag_mv_dsa_recv(struct net_if *iface, struct net_pkt *pkt)
{
	return mv_tag_recv(iface, pkt, false);
}

static struct net_pkt *dsa_tag_mv_dsa_xmit(struct net_if *iface, struct net_pkt *pkt)
{
	return mv_tag_xmit(iface, pkt, false);
}

static struct net_if *dsa_tag_mv_edsa_recv(struct net_if *iface, struct net_pkt *pkt)
{
	return mv_tag_recv(iface, pkt, true);
}

static struct net_pkt *dsa_tag_mv_edsa_xmit(struct net_if *iface, struct net_pkt *pkt)
{
	return mv_tag_xmit(iface, pkt, true);
}

DSA_TAG_REGISTER(DSA_TAG_PROTO_DSA, dsa_tag_mv_dsa_recv, dsa_tag_mv_dsa_xmit);
DSA_TAG_REGISTER(DSA_TAG_PROTO_EDSA, dsa_tag_mv_edsa_recv, dsa_tag_mv_edsa_xmit);
