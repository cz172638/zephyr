/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PTP_CLOCK_PTP_CLOCK_MV88E6XXX_H_
#define ZEPHYR_DRIVERS_PTP_CLOCK_PTP_CLOCK_MV88E6XXX_H_

#include <stdint.h>
#include <zephyr/device.h>

/* Convert a raw TAI time stamp (port arrival/departure registers) */
int ptp_clock_mv88e6xxx_cyc2ns(const struct device *dev, uint32_t cyc, uint64_t *ns);

#endif
