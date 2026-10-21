/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_MARVELL_ARMADA37XX_SOC_H_
#define ZEPHYR_SOC_MARVELL_ARMADA37XX_SOC_H_

#ifdef CONFIG_CPU_CORTEX_M
#include <cmsis_core_m_defaults.h>

/* Address map of the Cortex-M3 */
#define A3700_CM3_SOC_REGS  0xc0000000UL /* 0xd0000000 for the A53 */
#define A3700_CM3_OWN_REGS  0x40000000UL /* mailbox, eFuse, ... */
#define A3700_CM3_DDR       0x60000000UL /* DDR offset 0 */
#endif

#endif /* ZEPHYR_SOC_MARVELL_ARMADA37XX_SOC_H_ */
