/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_MARVELL_ARMADA37XX_PINCTRL_SOC_H_
#define ZEPHYR_SOC_MARVELL_ARMADA37XX_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/armada37xx-pinctrl.h>

/*
 * Armada 37xx muxes pins in groups: a bit (or a few) in the north or south
 * bridge selection register switches a whole group. Each child node of a
 * pinctrl state is one group: marvell,bridge, marvell,mask, marvell,value.
 */
typedef struct pinctrl_soc_pin {
	uint8_t bridge;
	uint32_t mask;
	uint32_t value;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_A37XX_GROUP(group)                                                               \
	{                                                                                          \
		.bridge = DT_PROP(group, marvell_bridge),                                          \
		.mask = DT_PROP(group, marvell_mask),                                              \
		.value = DT_PROP(group, marvell_value),                                            \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_CHILD(DT_PHANDLE(node_id, prop), Z_PINCTRL_A37XX_GROUP)                 \
	}

#endif /* ZEPHYR_SOC_MARVELL_ARMADA37XX_PINCTRL_SOC_H_ */
