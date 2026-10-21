/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef A3700_WTMI_H
#define A3700_WTMI_H

#include <stdint.h>

/* Clock preset, AVS and DDR tuning as the Marvell WTMI sys_init */
int a3700_wtmi_sys_init(void);

/* Tell the A53 BootROM to start at entry (0x04100000: TF-A BL1) */
void a3700_wtmi_start_ap(uint32_t entry);

#endif /* A3700_WTMI_H */
