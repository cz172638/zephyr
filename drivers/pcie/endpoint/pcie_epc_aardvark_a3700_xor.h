/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_A3700_XOR_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_A3700_XOR_H_

#include <stdint.h>

struct a3700_xor_result {
	uint32_t cause;
	uint32_t err;
	uint32_t err_addr;
	uint32_t status;
	/* time spent waiting for the descriptor write-back after EOC */
	uint32_t wb_wait_us;
};

/* DMA unit IO decode window 1 -> PCIe (target 2), 64K granularity */
void a3700_xor_pcie_window(uint32_t base, uint32_t size, uint8_t attr);

/* One memcpy, AXI addresses, 128 <= len < 16M; 0, -EIO or -ETIMEDOUT */
int a3700_xor_memcpy(uint32_t dst, uint32_t src, uint32_t len, struct a3700_xor_result *res);

#endif
