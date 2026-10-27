/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ROCKCHIP_RK3399_SOC_H_
#define ZEPHYR_SOC_ROCKCHIP_RK3399_SOC_H_

#include <zephyr/sys/util_macro.h>

/* Cortex-A physical addresses of system controllers */
#define RK3399_PMU_BASE		0xff310000UL
#define RK3399_PMUGRF_BASE	0xff320000UL
#define RK3399_SGRF_BASE	0xff330000UL
#define RK3399_PMUSRAM_BASE	0xff3b0000UL
#define RK3399_PMUCRU_BASE	0xff750000UL
#define RK3399_CRU_BASE		0xff760000UL
#define RK3399_GRF_BASE		0xff770000UL
#define RK3399_INTMEM_BASE	0xff8c0000UL

/*
 * The Cortex-M0 cores see the 0xf8000000-0xffffffff peripheral window at
 * 0x40000000; address 0x0 is remapped to a 4 KiB aligned DRAM address set up
 * by the secure firmware.
 */
#define RK3399_AP_MMIO_BASE	0xf8000000UL
#define RK3399_M0_MMIO_BASE	0x40000000UL
#define RK3399_AP_TO_M0(addr)	((addr) - RK3399_AP_MMIO_BASE + RK3399_M0_MMIO_BASE)

/* MPIDR affinity of the two clusters */
#define RK3399_MPIDR_A53(n)	(0x000UL + (n))
#define RK3399_MPIDR_A72(n)	(0x100UL + (n))

/* Rockchip "hiword" mask: upper 16 bits select which lower bits are written */
#define RK_HIWORD(val, mask, shift) \
	((((val) & (mask)) << (shift)) | (((mask) << (shift)) << 16U))

#if defined(CONFIG_CPU_CORTEX_M) && !defined(_ASMLANGUAGE)

/* CMSIS device configuration of the Cortex-M0 cores */
typedef enum IRQn {
	NonMaskableInt_IRQn = -14,
	HardFault_IRQn = -13,
	SVCall_IRQn = -5,
	PendSV_IRQn = -2,
	SysTick_IRQn = -1,
} IRQn_Type;

#define __CM0_REV		0x0000U
#define __NVIC_PRIO_BITS	NUM_IRQ_PRIO_BITS
#define __MPU_PRESENT		0U
#define __FPU_PRESENT		0U
#define __VTOR_PRESENT		0U
#define __Vendor_SysTickConfig	0U

#include <core_cm0.h>

#endif /* CONFIG_CPU_CORTEX_M && !_ASMLANGUAGE */

#endif /* ZEPHYR_SOC_ROCKCHIP_RK3399_SOC_H_ */
