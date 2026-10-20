/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Software time counter of the 88E6xxx PTP clock.
 */

#include <zephyr/ztest.h>
#include "ptp_clock_mv88e6xxx_tc.h"

#define NS_PER_S 1000000000ULL

ZTEST(mv88e6xxx_tc, test_nominal_8ns)
{
	struct mv_tc tc;

	mv_tc_init(&tc, 8000, 1000, 5 * NS_PER_S);
	mv_tc_update(&tc, 1000 + 125000000);
	zassert_equal(tc.ns, 6 * NS_PER_S);
	zassert_equal(mv_tc_cyc2ns(&tc, 1000 + 125000000 + 10), 6 * NS_PER_S + 80);
	zassert_equal(mv_tc_cyc2ns(&tc, 1000 + 125000000 - 10), 6 * NS_PER_S - 80);
}

ZTEST(mv88e6xxx_tc, test_wrap)
{
	struct mv_tc tc;

	mv_tc_init(&tc, 8000, 0xffff0000, 0);
	mv_tc_update(&tc, 0x00010000);
	zassert_equal(tc.ns, 0x20000ULL * 8);
	/* a stamp latched just before the wrap, converted after it */
	zassert_equal(mv_tc_cyc2ns(&tc, 0xfffffff0), (0x20000ULL - 0x10010) * 8);
}

ZTEST(mv88e6xxx_tc, test_ratio)
{
	struct mv_tc tc;
	uint64_t want = 800000000ULL + 80000; /* 100 ppm fast over 0.8 s */

	mv_tc_init(&tc, 8000, 0, 0);
	mv_tc_set_ratio(&tc, 0, 1.0001);
	mv_tc_update(&tc, 100000000);
	zassert_within(tc.ns, want, 1, "%llu", (unsigned long long)tc.ns);

	/* fractions accumulate across many small updates */
	mv_tc_init(&tc, 8000, 0, 0);
	mv_tc_set_ratio(&tc, 0, 1.0000001);
	for (uint32_t c = 1; c <= 1000000; c++) {
		mv_tc_update(&tc, c * 100);
	}
	zassert_within(tc.ns, 800000080ULL, 1, "%llu", (unsigned long long)tc.ns);
}

ZTEST(mv88e6xxx_tc, test_set_adjust)
{
	struct mv_tc tc;

	mv_tc_init(&tc, 8000, 0, 0);
	mv_tc_set(&tc, 500, 42 * NS_PER_S);
	zassert_equal(mv_tc_cyc2ns(&tc, 500 + 125), 42 * NS_PER_S + 1000);
	mv_tc_adjust(&tc, 625, -1000);
	zassert_equal(tc.ns, 42 * NS_PER_S);
	mv_tc_adjust(&tc, 625, 250);
	zassert_equal(mv_tc_cyc2ns(&tc, 625), 42 * NS_PER_S + 250);
	mv_tc_adjust(&tc, 625, -(int64_t)(100 * NS_PER_S));
	zassert_equal(tc.ns, 0);
}

ZTEST(mv88e6xxx_tc, test_other_periods)
{
	struct mv_tc tc;

	mv_tc_init(&tc, 4000, 0, 0);
	mv_tc_update(&tc, 250000000);
	zassert_equal(tc.ns, NS_PER_S);
	mv_tc_init(&tc, 10000, 0, 0);
	mv_tc_update(&tc, 100000000);
	zassert_equal(tc.ns, NS_PER_S);
}

ZTEST_SUITE(mv88e6xxx_tc, NULL, NULL, NULL, NULL, NULL);
