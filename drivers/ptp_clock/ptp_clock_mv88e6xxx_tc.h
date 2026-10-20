/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Software time counter on a free running 32 bit cycle counter, like the
 * Linux timecounter: the time at the last update plus the cycles since,
 * scaled by a 32.32 fixed point ns-per-cycle rate. Must be updated at
 * least once per half counter period.
 */

#ifndef ZEPHYR_DRIVERS_PTP_CLOCK_MV88E6XXX_TC_H_
#define ZEPHYR_DRIVERS_PTP_CLOCK_MV88E6XXX_TC_H_

#include <stdint.h>

struct mv_tc {
	uint32_t last_cyc;
	uint32_t frac;
	uint64_t ns;
	uint64_t mult;
	uint64_t nominal;
};

/* delta * mult + frac, returns ns, fraction in *frac_out */
static inline uint64_t mv_tc_scale(uint32_t delta, uint64_t mult, uint32_t frac, uint32_t *frac_out)
{
	uint64_t lo = (uint64_t)delta * (mult & 0xffffffffU) + frac;

	*frac_out = (uint32_t)lo;
	return (uint64_t)delta * (mult >> 32) + (lo >> 32);
}

static inline void mv_tc_init(struct mv_tc *tc, uint32_t period_ps, uint32_t cyc, uint64_t ns)
{
	tc->nominal = ((uint64_t)period_ps << 32) / 1000U;
	tc->mult = tc->nominal;
	tc->last_cyc = cyc;
	tc->frac = 0;
	tc->ns = ns;
}

static inline void mv_tc_update(struct mv_tc *tc, uint32_t cyc)
{
	tc->ns += mv_tc_scale(cyc - tc->last_cyc, tc->mult, tc->frac, &tc->frac);
	tc->last_cyc = cyc;
}

/* Time of a cycle count within half a counter period of the last update */
static inline uint64_t mv_tc_cyc2ns(const struct mv_tc *tc, uint32_t cyc)
{
	int32_t delta = (int32_t)(cyc - tc->last_cyc);
	uint32_t frac;

	if (delta >= 0) {
		return tc->ns + mv_tc_scale((uint32_t)delta, tc->mult, tc->frac, &frac);
	}

	return tc->ns - mv_tc_scale((uint32_t)-delta, tc->mult, 0, &frac);
}

static inline void mv_tc_set(struct mv_tc *tc, uint32_t cyc, uint64_t ns)
{
	tc->last_cyc = cyc;
	tc->frac = 0;
	tc->ns = ns;
}

static inline void mv_tc_adjust(struct mv_tc *tc, uint32_t cyc, int64_t delta_ns)
{
	mv_tc_update(tc, cyc);
	if (delta_ns < 0 && (uint64_t)-delta_ns > tc->ns) {
		tc->ns = 0;
	} else {
		tc->ns += delta_ns;
	}
}

/* ratio: rate relative to the nominal counter rate */
static inline void mv_tc_set_ratio(struct mv_tc *tc, uint32_t cyc, double ratio)
{
	mv_tc_update(tc, cyc);
	tc->mult = (uint64_t)((double)tc->nominal * ratio + 0.5);
}

#endif /* ZEPHYR_DRIVERS_PTP_CLOCK_MV88E6XXX_TC_H_ */
