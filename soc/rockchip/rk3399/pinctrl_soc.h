/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ROCKCHIP_RK3399_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ROCKCHIP_RK3399_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/types.h>
#include <zephyr/dt-bindings/pinctrl/rk3399-pinctrl.h>

/* one RK_PINMUX() cell */
typedef uint32_t pinctrl_soc_pin_t;

#define Z_PINCTRL_RK_PIN(node_id, prop, idx) DT_PROP_BY_IDX(node_id, prop, idx),

/*
 * pinctrl-N names one state node; its children are groups with "pinmux"
 * (the macros of DT_FOREACH_PROP_ELEM and LISTIFY do not nest further).
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,          \
				Z_PINCTRL_RK_PIN)}

#endif
