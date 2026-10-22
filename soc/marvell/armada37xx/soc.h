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

/*
 * NVIC lines of the CM3 (from CZ.NIC mox-boot-builder wtmi/irq.h): the
 * A53 mailbox command arrives on SP, the UART only signals RX.
 */
#define A3700_CM3_IRQ_AP          0
#define A3700_CM3_IRQ_USB         1
#define A3700_CM3_IRQ_SP          2
#define A3700_CM3_IRQ_DMA         3
#define A3700_CM3_IRQ_MCT         4
#define A3700_CM3_IRQ_BCM_ACC_ERR 5
#define A3700_CM3_IRQ_EC_ZMODP    6
#define A3700_CM3_IRQ_HASH        7
#define A3700_CM3_IRQ_SATA        8
#define A3700_CM3_IRQ_CRYPTO      9
#define A3700_CM3_IRQ_EMMC_SDIO   10
#define A3700_CM3_IRQ_SPAD        11
#define A3700_CM3_IRQ_SPI         12
#define A3700_CM3_IRQ_UART_RX     13
#define A3700_CM3_IRQ_CTITRIGOUT2 14
#define A3700_CM3_IRQ_CTITRIGOUT3 15
#endif

#if defined(CONFIG_SOC_ARMADA3720) && !defined(_ASMLANGUAGE)
#include <stdint.h>
/* 32-bit read that returns -EIO on an external abort (probe.c) */
int a3700_probe_read32(uintptr_t va, uint32_t *val);
/* 32-bit write (and read back) that returns -EIO on a bus error */
int a3700_probe_write32(uintptr_t va, uint32_t val);
extern volatile uint32_t a3700_probe_faults;
#endif

#endif /* ZEPHYR_SOC_MARVELL_ARMADA37XX_SOC_H_ */
