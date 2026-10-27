/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/arm64/arm-smccc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>

#include <soc.h>
#include <rk3399_cores.h>

/* TF-A plat/rockchip/rk3399 SiP service */
#define RK_SIP_M0_CTRL	0x82000020UL
#define RK_M0_NS_START	0UL
#define RK_M0_NS_STOP	1UL

#define SMC_UNK		0xffffffffUL

#define PSCI_0_2_FN64_CPU_ON	0xc4000003UL

static int rk3399_sip_to_errno(unsigned long ret)
{
	int32_t err = (int32_t)(uint32_t)ret;

	if ((uint32_t)ret == (uint32_t)SMC_UNK) {
		return -ENOTSUP;
	}

	return (err < 0) ? err : 0;
}

int rk3399_m0_start(enum rk3399_m0 core, uintptr_t dram_addr, uint32_t perilp_div)
{
	struct arm_smccc_res res;

	if ((core != RK3399_M0_PMU) && (core != RK3399_M0_PERILP)) {
		return -EINVAL;
	}

	if ((dram_addr & 0xfffUL) != 0UL) {
		return -EINVAL;
	}

	arm_smccc_smc(RK_SIP_M0_CTRL, RK_M0_NS_START, (unsigned long)core, dram_addr,
		      perilp_div, 0, 0, 0, &res);

	return rk3399_sip_to_errno(res.a0);
}

int rk3399_m0_stop(enum rk3399_m0 core)
{
	struct arm_smccc_res res;

	if ((core != RK3399_M0_PMU) && (core != RK3399_M0_PERILP)) {
		return -EINVAL;
	}

	arm_smccc_smc(RK_SIP_M0_CTRL, RK_M0_NS_STOP, (unsigned long)core, 0, 0, 0, 0, 0, &res);

	return rk3399_sip_to_errno(res.a0);
}

int rk3399_cpu_start(uint64_t mpidr, uintptr_t entry)
{
	struct arm_smccc_res res;
	int32_t ret;

	arm_smccc_smc(PSCI_0_2_FN64_CPU_ON, mpidr, entry, 0, 0, 0, 0, 0, &res);
	ret = (int32_t)res.a0;

	switch (ret) {
	case 0:
		return 0;
	case -2: /* INVALID_PARAMETERS */
		return -EINVAL;
	case -3: /* DENIED */
		return -EPERM;
	case -4: /* ALREADY_ON */
	case -5: /* ON_PENDING */
		return -EALREADY;
	default:
		return -EIO;
	}
}
