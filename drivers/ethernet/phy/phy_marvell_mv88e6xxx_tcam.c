/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 88E6341/88E6390 TCAM with the register layout of Linux
 * drivers/net/dsa/mv88e6xxx/tcam.c: at SMI address 0x1f, an operation
 * register (busy, operation, page, entry) and the page registers.
 *
 *   page 0: 2 keys 1 (frame type mask 15:14, source port vector mask
 *           10:8 and value 2:0 for ports 8..10), 3 keys 2 (source port
 *           vector mask 15:8 and value 7:0), 6..27 key bytes 0..21
 *   page 1: 2..27 key bytes 22..47
 *   page 2: 4 action 3 (DPV valid 11, DPV 10:0), 6 action 5 (DPV mode
 *           15:14: 1 AND, 2 OR, 3 replace)
 *
 * Key byte registers are mask << 8 | data. The first matching entry, the
 * lowest index, wins; an entry without action just stops the lookup.
 * The 88E6341 is not documented, its layout was checked by reading.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "phy_marvell_mv88e6xxx_tcam.h"

#define TCAM_PAGE0_BYTES 22
#define TCAM_PAGE1_BYTES 26

#define TCAM_KEYS1             0x02
#define TCAM_KEYS1_FT_MASK     0xc000
#define TCAM_KEYS1_SPV_MASK    0x0700
#define TCAM_KEYS1_SPV         0x0007
#define TCAM_KEYS2             0x03
#define TCAM_PAGE0_KEY         6
#define TCAM_PAGE1_KEY         2
#define TCAM_ACT3              0x04
#define TCAM_ACT3_DPV_VALID    BIT(11)
#define TCAM_ACT3_DPV          GENMASK(10, 0)
#define TCAM_ACT5              0x06
#define TCAM_DPV_MODE_OR       2
#define TCAM_DPV_MODE_REPLACE  3
#define TCAM_ACT5_DPV_MODE_POS 14

void mv88e6xxx_tcam_match(struct mv88e6xxx_tcam_rule *rule, int offset, const uint8_t *data,
			  const uint8_t *mask, int len)
{
	for (int i = 0; i < len && offset + i < MV88E6XXX_TCAM_KEY_SIZE; i++) {
		uint8_t m = mask != NULL ? mask[i] : 0xff;

		rule->mask[offset + i] |= m;
		rule->data[offset + i] = (rule->data[offset + i] & ~m) | (data[i] & m);
	}
}

int mv88e6xxx_tcam_encode(const struct mv88e6xxx_tcam_rule *rule, uint8_t cpu_port,
			  uint16_t port_mask, struct mv88e6xxx_tcam_write *w)
{
	uint16_t dpv = 0, spv_mask;
	uint8_t mode = 0;
	int n = 0;

	switch (rule->action) {
	case MV88E6XXX_TCAM_PERMIT:
		break;
	case MV88E6XXX_TCAM_DROP:
		mode = TCAM_DPV_MODE_REPLACE;
		break;
	case MV88E6XXX_TCAM_TRAP:
		mode = TCAM_DPV_MODE_REPLACE;
		dpv = BIT(cpu_port);
		break;
	case MV88E6XXX_TCAM_REDIRECT:
		mode = TCAM_DPV_MODE_REPLACE;
		dpv = rule->action_ports;
		break;
	case MV88E6XXX_TCAM_MIRROR:
		mode = TCAM_DPV_MODE_OR;
		dpv = rule->action_ports;
		break;
	default:
		return -EINVAL;
	}

	/* Action page */
	if (mode != 0) {
		w[n++] = (struct mv88e6xxx_tcam_write){
			2, TCAM_ACT3, TCAM_ACT3_DPV_VALID | (dpv & port_mask & TCAM_ACT3_DPV)};
		w[n++] = (struct mv88e6xxx_tcam_write){2, TCAM_ACT5,
							mode << TCAM_ACT5_DPV_MODE_POS};
	}

	/* Key bytes 22..47 */
	for (int i = TCAM_PAGE0_BYTES; i < MV88E6XXX_TCAM_KEY_SIZE; i++) {
		if (rule->mask[i] != 0) {
			w[n++] = (struct mv88e6xxx_tcam_write){
				1, TCAM_PAGE1_KEY + i - TCAM_PAGE0_BYTES,
				(rule->mask[i] << 8) | (rule->data[i] & rule->mask[i])};
		}
	}

	/* Key bytes 0..21 */
	for (int i = 0; i < TCAM_PAGE0_BYTES; i++) {
		if (rule->mask[i] != 0) {
			w[n++] = (struct mv88e6xxx_tcam_write){
				0, TCAM_PAGE0_KEY + i,
				(rule->mask[i] << 8) | (rule->data[i] & rule->mask[i])};
		}
	}

	/* The source port vector is one hot: ignore the bits of the wanted
	 * ports, require the others to be clear. The frame type mask must be
	 * set for a valid entry (normal frames).
	 */
	spv_mask = port_mask & ~rule->ports;
	w[n++] = (struct mv88e6xxx_tcam_write){0, TCAM_KEYS1,
					       TCAM_KEYS1_FT_MASK | (spv_mask & TCAM_KEYS1_SPV_MASK)};
	w[n++] = (struct mv88e6xxx_tcam_write){0, TCAM_KEYS2, (spv_mask << 8) & 0xff00};

	return n;
}

#if defined(CONFIG_PHY_MARVELL_MV88E6XXX_TCAM)
#include <zephyr/logging/log.h>
#include "phy_marvell_mv88e6xxx.h"

LOG_MODULE_REGISTER(mv88e6xxx_tcam, CONFIG_PHY_LOG_LEVEL);

#define TCAM_ADDR          0x1f
#define TCAM_OP            0x00
#define TCAM_OP_BUSY       BIT(15)
#define TCAM_OP_FLUSH_ALL  0x1000
#define TCAM_OP_FLUSH      0x2000
#define TCAM_OP_LOAD       0x3000
#define TCAM_OP_READ       0x5000
#define TCAM_OP_PAGE(p)    (((p) & 0x3) << 10)
#define TCAM_PORT_PRI      0x0d
#define TCAM_PORT_MODE     GENMASK(1, 0)
#define TCAM_PORT_MODE_48B 1

/* Switch ports (DPV and SPV bits) */
#define TCAM_PORT_MASK GENMASK(10, 0)

static K_MUTEX_DEFINE(tcam_lock);
static struct mv88e6xxx_tcam_rule rules[CONFIG_PHY_MARVELL_MV88E6XXX_TCAM_RULES];
static int num_rules;
static int hw_used;
static uint16_t ports_on;
static bool flushed;

static int tcam_op(const struct device *dev, uint16_t op)
{
	uint16_t v;
	int ret = mv88e6xxx_reg_rmw(dev, TCAM_ADDR, TCAM_OP, 0xffff, TCAM_OP_BUSY | op, NULL);

	for (int i = 0; ret == 0 && i < 500; i++) {
		ret = mv88e6xxx_reg_rmw(dev, TCAM_ADDR, TCAM_OP, 0, 0, &v);
		if (ret == 0 && (v & TCAM_OP_BUSY) == 0) {
			return 0;
		}
		k_busy_wait(10);
	}

	return ret < 0 ? ret : -ETIMEDOUT;
}

static int tcam_program(const struct device *dev, const struct mv88e6xxx_tcam_rule *rule,
			uint8_t idx)
{
	struct mv88e6xxx_tcam_write w[MV88E6XXX_TCAM_MAX_WRITES];
	int n = mv88e6xxx_tcam_encode(rule, mv88e6xxx_cpu_port(dev), TCAM_PORT_MASK, w);
	int ret, i = 0;

	if (n < 0) {
		return n;
	}

	ret = tcam_op(dev, TCAM_OP_FLUSH | idx);

	/* Pages in the order of the writes: 2, 1, then 0 */
	for (int page = 2; ret == 0 && page >= 0; page--) {
		ret = tcam_op(dev, TCAM_OP_READ | TCAM_OP_PAGE(page) | idx);
		for (; ret == 0 && i < n && w[i].page == page; i++) {
			ret = mv88e6xxx_reg_rmw(dev, TCAM_ADDR, w[i].reg, 0xffff, w[i].val, NULL);
		}
		if (ret == 0) {
			ret = tcam_op(dev, TCAM_OP_LOAD | TCAM_OP_PAGE(page) | idx);
		}
	}

	return ret;
}

/* TCAM lookups on the ingress ports of the rules */
static int tcam_ports(const struct device *dev)
{
	uint16_t want = 0;
	int ret = 0;

	for (int i = 0; i < num_rules; i++) {
		want |= rules[i].ports;
	}
	want &= TCAM_PORT_MASK;

	for (uint8_t port = 0; ret == 0 && port < 11; port++) {
		if (((want ^ ports_on) & BIT(port)) == 0) {
			continue;
		}
		ret = mv88e6xxx_reg_rmw(dev, MV88E6XXX_ADDR_PORT(port), TCAM_PORT_PRI,
					TCAM_PORT_MODE,
					(want & BIT(port)) ? TCAM_PORT_MODE_48B : 0, NULL);
		if (ret == 0) {
			WRITE_BIT(ports_on, port, (want & BIT(port)) != 0);
		}
	}

	return ret;
}

/* Rewrite the entries in rule order, then clear what is left over */
static int tcam_sync(const struct device *dev)
{
	int ret = 0;

	if (!flushed) {
		ret = tcam_op(dev, TCAM_OP_FLUSH_ALL);
		flushed = ret == 0;
	}
	for (int i = 0; ret == 0 && i < num_rules; i++) {
		ret = tcam_program(dev, &rules[i], i);
	}
	for (int i = num_rules; ret == 0 && i < hw_used; i++) {
		ret = tcam_op(dev, TCAM_OP_FLUSH | i);
	}
	if (ret == 0) {
		hw_used = num_rules;
		ret = tcam_ports(dev);
	}
	if (ret < 0) {
		LOG_ERR("TCAM update failed (%d)", ret);
	}

	return ret;
}

int mv88e6xxx_tcam_add(const struct device *dev, const struct mv88e6xxx_tcam_rule *rule)
{
	int ret, pos;

	k_mutex_lock(&tcam_lock, K_FOREVER);

	for (int i = 0; i < num_rules; i++) {
		if (rules[i].id == rule->id) {
			k_mutex_unlock(&tcam_lock);
			return -EEXIST;
		}
	}
	if (num_rules == ARRAY_SIZE(rules)) {
		k_mutex_unlock(&tcam_lock);
		return -ENOSPC;
	}

	/* After the rules of the same or a lower priority value */
	for (pos = num_rules; pos > 0 && rules[pos - 1].prio > rule->prio; pos--) {
		rules[pos] = rules[pos - 1];
	}
	rules[pos] = *rule;
	num_rules++;

	ret = tcam_sync(dev);

	k_mutex_unlock(&tcam_lock);

	return ret;
}

int mv88e6xxx_tcam_del(const struct device *dev, uint16_t id)
{
	int ret = -ENOENT;

	k_mutex_lock(&tcam_lock, K_FOREVER);

	for (int i = 0; i < num_rules; i++) {
		if (rules[i].id == id) {
			memmove(&rules[i], &rules[i + 1], (num_rules - i - 1) * sizeof(rules[0]));
			num_rules--;
			ret = tcam_sync(dev);
			break;
		}
	}

	k_mutex_unlock(&tcam_lock);

	return ret;
}

int mv88e6xxx_tcam_flush(const struct device *dev)
{
	int ret;

	k_mutex_lock(&tcam_lock, K_FOREVER);
	num_rules = 0;
	flushed = false;
	hw_used = 0;
	ret = tcam_sync(dev);
	k_mutex_unlock(&tcam_lock);

	return ret;
}

void mv88e6xxx_tcam_foreach(const struct device *dev, mv88e6xxx_tcam_cb_t cb, void *user_data)
{
	ARG_UNUSED(dev);

	k_mutex_lock(&tcam_lock, K_FOREVER);
	for (int i = 0; i < num_rules; i++) {
		cb(&rules[i], i, user_data);
	}
	k_mutex_unlock(&tcam_lock);
}

int mv88e6xxx_tcam_read(const struct device *dev, uint8_t idx, uint16_t regs[3][32])
{
	int ret = 0;

	k_mutex_lock(&tcam_lock, K_FOREVER);
	for (int page = 0; ret == 0 && page < 3; page++) {
		ret = tcam_op(dev, TCAM_OP_READ | TCAM_OP_PAGE(page) | idx);
		for (int r = 0; ret == 0 && r < 32; r++) {
			ret = mv88e6xxx_reg_rmw(dev, TCAM_ADDR, r, 0, 0, &regs[page][r]);
		}
	}
	k_mutex_unlock(&tcam_lock);

	return ret;
}
#endif /* CONFIG_PHY_MARVELL_MV88E6XXX_TCAM */
