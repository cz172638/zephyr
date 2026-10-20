/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TCAM (ingress ACL) of the 88E6341 and 88E6390 family, 48 byte keys.
 *
 * The key is the frame as the switch sees it on ingress, always with a
 * 4 byte tag slot: DA 0..5, SA 6..11, 802.1Q tag 12..15, EtherType 16..17,
 * IPv4 header from 18 (protocol 27, source 30, destination 34, L4 ports
 * 38 and 40 without IP options), as in Linux tcflower.c.
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_TCAM_H_
#define ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_TCAM_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

#define MV88E6XXX_TCAM_KEY_SIZE 48

/* Key offsets */
#define MV88E6XXX_TCAM_OFF_DA        0
#define MV88E6XXX_TCAM_OFF_SA        6
#define MV88E6XXX_TCAM_OFF_TAG       12
#define MV88E6XXX_TCAM_OFF_ETHERTYPE 16
#define MV88E6XXX_TCAM_OFF_IP        18
#define MV88E6XXX_TCAM_OFF_IP_PROTO  27
#define MV88E6XXX_TCAM_OFF_IP_SRC    30
#define MV88E6XXX_TCAM_OFF_IP_DST    34
#define MV88E6XXX_TCAM_OFF_L4_SPORT  38
#define MV88E6XXX_TCAM_OFF_L4_DPORT  40

enum mv88e6xxx_tcam_action {
	/* Stop matching, forward normally (exception before a drop) */
	MV88E6XXX_TCAM_PERMIT,
	/* Destination ports replaced by none */
	MV88E6XXX_TCAM_DROP,
	/* ... by the CPU port */
	MV88E6XXX_TCAM_TRAP,
	/* ... by the given ports */
	MV88E6XXX_TCAM_REDIRECT,
	/* Given ports added to the destinations */
	MV88E6XXX_TCAM_MIRROR,
};

struct mv88e6xxx_tcam_rule {
	/* Unique, chosen by the user */
	uint16_t id;
	/* Lower first */
	uint16_t prio;
	/* Ingress ports the rule applies to */
	uint16_t ports;
	enum mv88e6xxx_tcam_action action;
	/* Ports for redirect and mirror */
	uint16_t action_ports;
	uint8_t data[MV88E6XXX_TCAM_KEY_SIZE];
	uint8_t mask[MV88E6XXX_TCAM_KEY_SIZE];
};

/* Set bytes of the key, data under mask */
void mv88e6xxx_tcam_match(struct mv88e6xxx_tcam_rule *rule, int offset, const uint8_t *data,
			  const uint8_t *mask, int len);

/* One register write of an entry: page 0..2, register 0..31 */
struct mv88e6xxx_tcam_write {
	uint8_t page;
	uint8_t reg;
	uint16_t val;
};

#define MV88E6XXX_TCAM_MAX_WRITES (3 + 2 + MV88E6XXX_TCAM_KEY_SIZE)

/*
 * Register writes for a rule, in the order they are loaded (action page,
 * key page 1, key page 0, which makes the entry valid), as Linux
 * mv88e6390_tcam_entry_add(). cpu_port is the trap destination.
 * Returns the number of writes.
 */
int mv88e6xxx_tcam_encode(const struct mv88e6xxx_tcam_rule *rule, uint8_t cpu_port,
			  uint16_t port_mask, struct mv88e6xxx_tcam_write *w);

/* Rule table of the switch, programmed in priority order */
int mv88e6xxx_tcam_add(const struct device *dev, const struct mv88e6xxx_tcam_rule *rule);
int mv88e6xxx_tcam_del(const struct device *dev, uint16_t id);
int mv88e6xxx_tcam_flush(const struct device *dev);

typedef void (*mv88e6xxx_tcam_cb_t)(const struct mv88e6xxx_tcam_rule *rule, int hw_idx,
				    void *user_data);
void mv88e6xxx_tcam_foreach(const struct device *dev, mv88e6xxx_tcam_cb_t cb, void *user_data);

/* Raw access for exploration: read the three pages of an entry */
int mv88e6xxx_tcam_read(const struct device *dev, uint8_t idx, uint16_t regs[3][32]);

#endif /* ZEPHYR_DRIVERS_ETHERNET_PHY_PHY_MARVELL_MV88E6XXX_TCAM_H_ */
