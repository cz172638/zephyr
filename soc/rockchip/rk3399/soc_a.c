/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/toolchain.h>

#include <soc.h>

#define PSCI_0_2_FN64_CPU_ON	0xc4000003UL
#define PSCI_0_2_FN_CPU_OFF	0x84000002UL

#define MPIDR_CLUSTER(mpidr)	(((mpidr) >> 8) & 0xffUL)

#if defined(CONFIG_RK3399_A72_SELF_MIGRATE)

extern char __start[];

#if defined(CONFIG_SMP)
/*
 * arch/arm64/core/smp.c: the first member of arm64_cpu_boot_params is the
 * MPIDR of the core that won the primary core election.
 */
extern volatile uint64_t arm64_cpu_boot_params;
#endif

static inline unsigned long rk3399_psci_smc(unsigned long fn, unsigned long a1,
					    unsigned long a2, unsigned long a3)
{
	register unsigned long x0 __asm__("x0") = fn;
	register unsigned long x1 __asm__("x1") = a1;
	register unsigned long x2 __asm__("x2") = a2;
	register unsigned long x3 __asm__("x3") = a3;

	__asm__ volatile("smc #0"
			 : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3)
			 :
			 : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
			   "x14", "x15", "x16", "x17", "memory");

	return x0;
}

/*
 * Runs on every core at the highest EL, MMU and data cache off. When the big
 * cluster image is entered on an A53 (boot ROM, TF-A and U-Boot only use
 * CPU0), hand over to CPU4 and power the A53 off.
 */
void z_arm64_el_highest_plat_init(void)
{
	uint64_t mpidr = read_mpidr_el1();
	unsigned long ret;

	if (MPIDR_CLUSTER(mpidr) != 0UL) {
		return;
	}

#if defined(CONFIG_SMP)
	/* Undo the primary core election done by this A53. */
	arm64_cpu_boot_params = UINT64_MAX;
	barrier_dsync_fence_full();
#endif

	ret = rk3399_psci_smc(PSCI_0_2_FN64_CPU_ON, RK3399_MPIDR_A72(0), (uintptr_t)__start, 0);
	if (ret != 0UL) {
		/* CPU4 did not start, keep running on this A53. */
		return;
	}

	(void)rk3399_psci_smc(PSCI_0_2_FN_CPU_OFF, 0, 0, 0);

	for (;;) {
		__asm__ volatile("wfi");
	}
}

#endif /* CONFIG_RK3399_A72_SELF_MIGRATE */
