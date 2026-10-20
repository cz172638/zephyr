/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register encoding of 88E6341/88E6390 TCAM rules (Linux tcam.c layout).
 */

#include <zephyr/ztest.h>
#include "phy_marvell_mv88e6xxx_tcam.h"

#define PORT_MASK 0x7ff
#define CPU_PORT  0

static struct mv88e6xxx_tcam_write w[MV88E6XXX_TCAM_MAX_WRITES];

static const struct mv88e6xxx_tcam_write *find(int n, uint8_t page, uint8_t reg)
{
	for (int i = 0; i < n; i++) {
		if (w[i].page == page && w[i].reg == reg) {
			return &w[i];
		}
	}
	return NULL;
}

ZTEST(mv88e6xxx_tcam, test_drop_icmp_on_port2)
{
	struct mv88e6xxx_tcam_rule r = {.ports = BIT(2), .action = MV88E6XXX_TCAM_DROP};
	const uint8_t et[2] = {0x08, 0x00}, icmp = 1;
	const struct mv88e6xxx_tcam_write *x;
	int n;

	mv88e6xxx_tcam_match(&r, MV88E6XXX_TCAM_OFF_ETHERTYPE, et, NULL, 2);
	mv88e6xxx_tcam_match(&r, MV88E6XXX_TCAM_OFF_IP_PROTO, &icmp, NULL, 1);
	n = mv88e6xxx_tcam_encode(&r, CPU_PORT, PORT_MASK, w);

	/* action: DPV valid, no ports, replace */
	zassert_equal(w[0].page, 2);
	zassert_equal(w[0].reg, 4);
	zassert_equal(w[0].val, 0x0800);
	zassert_equal(w[1].reg, 6);
	zassert_equal(w[1].val, 0xc000);

	/* page 1 (bytes 22..): protocol at 27 -> register 7 */
	x = find(n, 1, 2 + 27 - 22);
	zassert_not_null(x);
	zassert_equal(x->val, 0xff01);

	/* page 0: EtherType at 16/17 -> registers 22/23 */
	x = find(n, 0, 6 + 16);
	zassert_not_null(x);
	zassert_equal(x->val, 0xff08);
	x = find(n, 0, 6 + 17);
	zassert_not_null(x);
	zassert_equal(x->val, 0xff00);

	/* source port: all but port 2 must be clear, frame type mask set */
	zassert_equal(w[n - 2].page, 0);
	zassert_equal(w[n - 2].reg, 2);
	zassert_equal(w[n - 2].val, 0xc000 | (0x7fb & 0x0700));
	zassert_equal(w[n - 1].reg, 3);
	zassert_equal(w[n - 1].val, 0xfb00);

	/* page order 2, 1, 0 as they are loaded */
	for (int i = 1; i < n; i++) {
		zassert_true(w[i].page <= w[i - 1].page, "write %d out of page order", i);
	}
	zassert_equal(n, 2 + 1 + 2 + 2);
}

ZTEST(mv88e6xxx_tcam, test_actions)
{
	struct mv88e6xxx_tcam_rule r = {.ports = BIT(1) | BIT(3)};
	int n;

	r.action = MV88E6XXX_TCAM_PERMIT;
	n = mv88e6xxx_tcam_encode(&r, CPU_PORT, PORT_MASK, w);
	zassert_equal(n, 2, "permit writes only the keys");
	zassert_equal(w[1].val, 0xf500);

	r.action = MV88E6XXX_TCAM_TRAP;
	mv88e6xxx_tcam_encode(&r, 5, PORT_MASK, w);
	zassert_equal(w[0].val, 0x0800 | BIT(5));
	zassert_equal(w[1].val, 0xc000);

	r.action = MV88E6XXX_TCAM_MIRROR;
	r.action_ports = BIT(2);
	mv88e6xxx_tcam_encode(&r, CPU_PORT, PORT_MASK, w);
	zassert_equal(w[0].val, 0x0800 | BIT(2));
	zassert_equal(w[1].val, 0x8000, "mirror is OR");

	r.action = MV88E6XXX_TCAM_REDIRECT;
	r.action_ports = BIT(9);
	mv88e6xxx_tcam_encode(&r, CPU_PORT, PORT_MASK, w);
	zassert_equal(w[0].val, 0x0800 | BIT(9));
	zassert_equal(w[1].val, 0xc000);
}

ZTEST(mv88e6xxx_tcam, test_match_mask)
{
	struct mv88e6xxx_tcam_rule r = {0};
	const uint8_t net[4] = {192, 168, 10, 77}, m24[4] = {0xff, 0xff, 0xff, 0};
	int n;

	mv88e6xxx_tcam_match(&r, MV88E6XXX_TCAM_OFF_IP_SRC, net, m24, 4);
	zassert_equal(r.data[33], 0, "data outside the mask is cleared");
	zassert_equal(r.mask[33], 0);
	n = mv88e6xxx_tcam_encode(&r, CPU_PORT, PORT_MASK, w);
	/* 30..32 on page 1, nothing for 33 */
	zassert_not_null(find(n, 1, 2 + 30 - 22));
	zassert_is_null(find(n, 1, 2 + 33 - 22));
	zassert_equal(find(n, 1, 2 + 32 - 22)->val, 0xff0a);

	/* matching past the key is ignored */
	mv88e6xxx_tcam_match(&r, 46, net, NULL, 4);
	zassert_equal(r.mask[47], 0xff);
}

ZTEST_SUITE(mv88e6xxx_tcam, NULL, NULL, NULL, NULL, NULL);
