/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Replacement for the A3700-utils generated wtmi/sys_init/autoconf.h,
 * values from Kconfig (SOC_ARMADA3720_CM3_*). The Marvell code uses
 * these CONFIG_ names itself.
 */

#ifndef A3700_WTMI_AUTOCONF_H
#define A3700_WTMI_AUTOCONF_H

#include <zephyr/autoconf.h>

#define WTMI_CLOCK       CONFIG_SOC_ARMADA3720_CM3_CLOCK_PRESET
#define CONFIG_DDR_TYPE  CONFIG_SOC_ARMADA3720_CM3_DDR_TYPE
#define CONFIG_BUS_WIDTH CONFIG_SOC_ARMADA3720_CM3_DDR_DEV_WIDTH
#define CONFIG_SPEED_BIN CONFIG_SOC_ARMADA3720_CM3_DDR_SPEED_BIN
#define CONFIG_CS_NUM    CONFIG_SOC_ARMADA3720_CM3_DDR_CS_NUM
#define CONFIG_DEV_CAP   CONFIG_SOC_ARMADA3720_CM3_DDR_DEV_CAP_MB

#endif /* A3700_WTMI_AUTOCONF_H */
