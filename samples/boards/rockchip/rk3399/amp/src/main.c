/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Runs on the little cluster and starts the other images, which a previous
 * boot stage (U-Boot, or the SPL FIT "loadables") has already placed in
 * DRAM and cleaned to the point of coherency.
 */

#include <stdio.h>

#include <zephyr/kernel.h>

#include <soc.h>
#include <rk3399_cores.h>

static void report(const char *what, int ret)
{
	printf("%-12s %s (%d)\n", what, (ret == 0) ? "started" : "FAILED", ret);
}

int main(void)
{
	printf("RK3399 AMP launcher on %s\n", CONFIG_BOARD_TARGET);

	if (CONFIG_AMP_M0_PMU_IMAGE != 0) {
		report("PMU M0", rk3399_m0_start(RK3399_M0_PMU, CONFIG_AMP_M0_PMU_IMAGE, 0));
	}

	if (CONFIG_AMP_M0_PERILP_IMAGE != 0) {
		report("PERILP M0", rk3399_m0_start(RK3399_M0_PERILP, CONFIG_AMP_M0_PERILP_IMAGE,
						    CONFIG_AMP_M0_PERILP_DIV));
	}

	if (CONFIG_AMP_A72_ENTRY != 0) {
		report("A72 CPU4", rk3399_cpu_start(RK3399_MPIDR_A72(0), CONFIG_AMP_A72_ENTRY));
	}

	return 0;
}
