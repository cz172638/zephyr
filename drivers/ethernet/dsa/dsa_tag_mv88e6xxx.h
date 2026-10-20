/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_DSA_DSA_TAG_MV88E6XXX_H_
#define ZEPHYR_DRIVERS_ETHERNET_DSA_DSA_TAG_MV88E6XXX_H_

#include <zephyr/net/dsa_core.h>

/* Set by the switch driver as the tagger data of the switch context */
struct dsa_tag_mv88e6xxx_data {
#if defined(CONFIG_NET_L2_PTP_TIMESTAMPING)
	/* Called for untagged PTP event messages (types 0-3) received on a
	 * user port, to attach the arrival time stamp to the packet.
	 */
	void (*rx_tstamp)(const struct dsa_switch_context *ctx, uint8_t port, struct net_pkt *pkt,
			  uint8_t msg_type, uint16_t seq_id);
#endif
};

#endif /* ZEPHYR_DRIVERS_ETHERNET_DSA_DSA_TAG_MV88E6XXX_H_ */
