/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3700 XOR engine (unit 0, channel 0) used as system DMA for the
 * Aardvark endpoint, programmed as Linux mv_xor does on the A3700: one
 * 4 GiB XOR window, the AXI level decoding the address. The DMA unit's
 * IO decode windows (0xd000c200, 2 x 0x10) only cover DRAM after TF-A;
 * window 1 is pointed at the PCIe outbound region (CPU window target 2),
 * else a transfer to or from host memory ends with ERR_MBUS.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_epc_aardvark_a3700_xor.h"

#define XOR_LOW  0xd0060900UL
#define XOR_HIGH 0xd0060b00UL

/* low: channel 0 */
#define XOR_CONFIG     0x10
#define XOR_ACTIVATION 0x20
#define XOR_CAUSE      0x30
#define XOR_ERR_CAUSE  0x50
#define XOR_ERR_ADDR   0x60
/* high: channel 0 */
#define XOR_NEXT_DESC  0x00
#define XOR_BAR_EN     0x40
#define XOR_WIN_BASE   0x50
#define XOR_WIN_SIZE   0x70
#define XOR_WIN_OVRD   0xa0

#define XOR_CAUSE_EOC  BIT(1)
#define XOR_CAUSE_ERRS GENMASK(9, 4)

#define XOR_DESC_OWNER   BIT(31)
#define XOR_DESC_SUCCESS BIT(30)
#define XOR_CMD_MEMCPY   (2U << 24)

/* DMA unit IO decode windows */
#define IODEC_DMA       0xd000c200UL
#define IODEC_WIN_DRAM  0
#define IODEC_WIN_PCIE  1
#define TARGET_PCIE     2

struct xor_desc {
	uint32_t status;
	uint32_t crc32;
	uint32_t command;
	uint32_t next;
	uint32_t byte_count;
	uint32_t dst;
	uint32_t src[8];
	uint32_t rsvd[2];
} __aligned(64);

static struct xor_desc desc;
static K_MUTEX_DEFINE(xor_lock);
static uint8_t *lo, *hi;

static void xor_map(void)
{
	if (lo == NULL) {
		k_mem_map_phys_bare(&lo, XOR_LOW, 0x100, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
		k_mem_map_phys_bare(&hi, XOR_HIGH, 0x100, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	}
}

void a3700_xor_pcie_window(uint32_t base, uint32_t size, uint8_t attr)
{
	uint8_t *va;
	volatile uint32_t *w;
	uint32_t ctrl = ((size >> 16) - 1) << 16 | (uint32_t)attr << 8 | TARGET_PCIE << 4;

	k_mem_map_phys_bare(&va, IODEC_DMA & ~0xfffUL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	w = (volatile uint32_t *)(va + (IODEC_DMA & 0xfff) + IODEC_WIN_PCIE * 0x10);
	w[0] = 0;
	w[1] = base & 0xffff0000U;
	w[0] = ctrl;
	w[0] = ctrl | 1;
	barrier_dsync_fence_full();
	k_mem_unmap_phys_bare(va, 0x1000);
}

int a3700_xor_memcpy(uint32_t dst, uint32_t src, uint32_t len, struct a3700_xor_result *res)
{
	struct a3700_xor_result r = {0};
	int64_t end;
	int ret;

	if (len < 128 || len >= 0x1000000) {
		return -EINVAL;
	}

	k_mutex_lock(&xor_lock, K_FOREVER);
	xor_map();

	for (int i = 0; i < 8; i++) {
		sys_write32(0, (mem_addr_t)hi + XOR_WIN_BASE + 4 * i);
		sys_write32(0, (mem_addr_t)hi + XOR_WIN_SIZE + 4 * i);
	}
	sys_write32(0xffff0000, (mem_addr_t)hi + XOR_WIN_SIZE);
	sys_write32(1 | (3 << 16), (mem_addr_t)hi + XOR_BAR_EN);
	sys_write32(0, (mem_addr_t)hi + XOR_WIN_OVRD);

	/* stopped, descriptor mode, no swap */
	sys_write32((sys_read32((mem_addr_t)lo + XOR_CONFIG) & ~(0x7U | BIT(14))) | 7,
		    (mem_addr_t)lo + XOR_CONFIG);
	sys_write32(0, (mem_addr_t)lo + XOR_CAUSE);

	memset(&desc, 0, sizeof(desc));
	desc.status = XOR_DESC_OWNER;
	desc.command = XOR_CMD_MEMCPY;
	desc.byte_count = len;
	desc.dst = dst;
	desc.src[0] = src;
	sys_cache_data_flush_range(&desc, sizeof(desc));
	barrier_dsync_fence_full();

	sys_write32((uint32_t)k_mem_phys_addr(&desc), (mem_addr_t)hi + XOR_NEXT_DESC);
	sys_write32(BIT(0), (mem_addr_t)lo + XOR_ACTIVATION);

	end = k_uptime_get() + 1000;
	do {
		r.cause = sys_read32((mem_addr_t)lo + XOR_CAUSE);
		if (r.cause & (XOR_CAUSE_EOC | XOR_CAUSE_ERRS)) {
			break;
		}
	} while (k_uptime_get() < end);
	r.err = sys_read32((mem_addr_t)lo + XOR_ERR_CAUSE);
	r.err_addr = sys_read32((mem_addr_t)lo + XOR_ERR_ADDR);
	sys_cache_data_invd_range(&desc, sizeof(desc));
	r.status = desc.status;

	/*
	 * End of chain can come before the descriptor status is written back
	 * (seen toward PCIe at Gen1: EOC, no error bits, owner still set):
	 * wait for the write-back.
	 */
	if ((r.cause & XOR_CAUSE_EOC) && !(r.cause & XOR_CAUSE_ERRS) &&
	    (r.status & XOR_DESC_OWNER)) {
		int64_t wb_end = k_uptime_get() + 100;
		uint64_t t0 = k_cycle_get_64();

		while ((r.status & XOR_DESC_OWNER) && k_uptime_get() < wb_end) {
			sys_cache_data_invd_range(&desc, sizeof(desc));
			r.status = desc.status;
		}
		r.wb_wait_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);
	}

	if (!(r.cause & XOR_CAUSE_EOC)) {
		ret = -ETIMEDOUT;
	} else if ((r.cause & XOR_CAUSE_ERRS) || !(r.status & XOR_DESC_SUCCESS)) {
		ret = -EIO;
	} else {
		ret = 0;
	}
	if (ret != 0) {
		sys_write32(BIT(1), (mem_addr_t)lo + XOR_ACTIVATION); /* stop */
	}
	sys_write32(0, (mem_addr_t)lo + XOR_CAUSE);
	k_mutex_unlock(&xor_lock);

	if (res != NULL) {
		*res = r;
	}
	return ret;
}
