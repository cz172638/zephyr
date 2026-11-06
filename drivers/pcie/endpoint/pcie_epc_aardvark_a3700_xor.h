/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_A3700_XOR_H_
#define ZEPHYR_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_AARDVARK_A3700_XOR_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

/* Engine limits (Linux mv_xor: MV_XOR_MIN/MAX_BYTE_COUNT) */
#define A3700_XOR_MIN_LEN   128U
#define A3700_XOR_MAX_LEN   (0x1000000U - 1U)
#define A3700_XOR_MAX_SRC   8
#define A3700_XOR_MAX_SEGS  16
#define A3700_XOR_CHANNELS  2
#define A3700_XOR_ANY       (-1)

struct a3700_xor_result {
	uint32_t cause;    /* channel cause bits (EOD, EOC, stopped, errors) */
	uint32_t err;
	uint32_t err_addr;
	uint32_t status;   /* status of the last descriptor */
	/* time spent waiting for the descriptor write-back after EOC */
	uint32_t wb_wait_us;
	uint8_t chan;      /* channel that ran it */
	uint8_t failed;    /* first descriptor not successful, 0xff none */
	bool irq;          /* completion seen by interrupt (else polling) */
};

/* One piece of a chain: AXI addresses, A3700_XOR_MIN_LEN <= len <= MAX_LEN */
struct a3700_xor_seg {
	uint32_t dst;
	uint32_t src;
	uint32_t len;
};

struct a3700_xor_stats {
	uint32_t ops, descs, errors, timeouts, irqs, polls;
	uint64_t bytes;
};

/* DMA unit IO decode window 1 -> PCIe (target 2), 64K granularity */
void a3700_xor_pcie_window(uint32_t base, uint32_t size, uint8_t attr);

/*
 * The engine works on AXI addresses (DRAM, or PCIe through window 1),
 * not coherently: flush sources and invalidate destinations around a
 * transfer. Every call blocks until the chain ends (interrupt or
 * polling, 1 s timeout) and returns 0, -EINVAL, -EIO or -ETIMEDOUT.
 * chan: 0, 1 or A3700_XOR_ANY (the first free one).
 */

/* One memcpy */
int a3700_xor_memcpy(uint32_t dst, uint32_t src, uint32_t len, struct a3700_xor_result *res);

/* A chain of memcpys (scatter-gather), up to A3700_XOR_MAX_SEGS */
int a3700_xor_memcpy_sg(int chan, const struct a3700_xor_seg *seg, int n,
			struct a3700_xor_result *res);

/* dst = src[0] ^ ... ^ src[nsrc - 1], 2 <= nsrc <= A3700_XOR_MAX_SRC */
int a3700_xor_xor(int chan, uint32_t dst, const uint32_t *src, int nsrc, uint32_t len,
		  struct a3700_xor_result *res);

void a3700_xor_get_stats(int chan, struct a3700_xor_stats *st);

#endif
