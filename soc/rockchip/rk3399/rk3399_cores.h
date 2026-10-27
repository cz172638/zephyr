/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ROCKCHIP_RK3399_CORES_H_
#define ZEPHYR_SOC_ROCKCHIP_RK3399_CORES_H_

#include <stdint.h>

/** RK3399 Cortex-M0 cores */
enum rk3399_m0 {
	RK3399_M0_PMU = 0,
	RK3399_M0_PERILP = 1,
};

/**
 * @brief Start a Cortex-M0 core.
 *
 * Asks the secure firmware (RK_SIP_M0_CTRL) to remap M0 address 0x0 to
 * @p dram_addr, enable the core clocks and release it from reset. The image
 * must already be in DRAM and cleaned to the point of coherency.
 *
 * @param core Core to start.
 * @param dram_addr 4 KiB aligned physical address of the M0 image.
 * @param perilp_div PERILP M0 only: fclk_cm0s = GPLL / (perilp_div + 1).
 *
 * @retval 0 Core started.
 * @retval -EINVAL Invalid core or misaligned address.
 * @retval -ENOTSUP Secure firmware does not implement RK_SIP_M0_CTRL.
 * @retval -EPERM Address rejected by the secure firmware.
 */
int rk3399_m0_start(enum rk3399_m0 core, uintptr_t dram_addr, uint32_t perilp_div);

/**
 * @brief Hold a Cortex-M0 core in reset and gate its clocks.
 *
 * @param core Core to stop.
 *
 * @retval 0 Core stopped.
 * @retval -EINVAL Invalid core.
 * @retval -ENOTSUP Secure firmware does not implement RK_SIP_M0_CTRL.
 */
int rk3399_m0_stop(enum rk3399_m0 core);

/**
 * @brief Start a Cortex-A core outside of this image's SMP set.
 *
 * Used to boot an independent image (AMP) on a core of the other cluster.
 *
 * @param mpidr MPIDR affinity of the core (RK3399_MPIDR_A53(n), RK3399_MPIDR_A72(n)).
 * @param entry Physical entry point of the image.
 *
 * @retval 0 Core started.
 * @retval <0 PSCI error translated to an errno value.
 */
int rk3399_cpu_start(uint64_t mpidr, uintptr_t entry);

#endif /* ZEPHYR_SOC_ROCKCHIP_RK3399_CORES_H_ */
