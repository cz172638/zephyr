/*
 * Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ROCKCHIP_RK3308_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ROCKCHIP_RK3308_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/rk3308-pinctrl.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GRF_GPIO*_P values; RK3308_PULL_KEEP leaves the field alone */
#define RK3308_PULL_NONE 0U
#define RK3308_PULL_UP   1U
#define RK3308_PULL_DOWN 2U
#define RK3308_PULL_KEEP 0xffU

typedef struct pinctrl_soc_pin {
	uint32_t cfg;
	uint8_t pull;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_RK3308_PULL(node_id)                                                             \
	(DT_PROP(node_id, bias_pull_up)     ? RK3308_PULL_UP                                       \
	 : DT_PROP(node_id, bias_pull_down) ? RK3308_PULL_DOWN                                     \
	 : DT_PROP(node_id, bias_disable)   ? RK3308_PULL_NONE                                     \
					    : RK3308_PULL_KEEP)

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	{                                                                                          \
		.cfg = DT_PROP_BY_IDX(node_id, prop, idx),                                         \
		.pull = Z_PINCTRL_RK3308_PULL(node_id),                                            \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,            \
				Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ROCKCHIP_RK3308_PINCTRL_SOC_H_ */
