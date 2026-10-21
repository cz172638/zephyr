/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Replacement for A3700-utils wtmi/sys_init/sys.h: what the Marvell
 * clock, AVS and DDR code needs, on Zephyr.
 */

#ifndef A3700_WTMI_SYS_H
#define A3700_WTMI_SYS_H

#include "autoconf.h"

#include "bits.h"
#include "types.h"
#include "regs.h"
#include "io.h"
#include "delay.h"

int a3700_wtmi_printf(const char *fmt, ...);
#define printf a3700_wtmi_printf

#endif /* A3700_WTMI_SYS_H */
