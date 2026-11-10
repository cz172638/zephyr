/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie debug xortest: built into the firmware (CONFIG_PCIE_EPC_AARDVARK_XORTEST)
 * or as a lab debug module (zephyr/lab/mod.h).
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>

#include "pcie_epc_aardvark_a3700_xor.h"

static int parse_u(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

/*
 * pcie debug xortest [size] [loops]: the XOR engine on DRAM, no PCIe:
 * memcpy on both channels, a scatter-gather chain, XOR of eight
 * sources, both channels at once, throughput. Data checked by the CPU.
 */
#define XT_MAX 8192
#define XT_BUFS 10
static uint8_t xt_buf[XT_BUFS][XT_MAX] __aligned(64);

static uint32_t xt_phys(int i, size_t off)
{
	return (uint32_t)k_mem_phys_addr(&xt_buf[i][off]);
}

static void xt_fill(int i, size_t len, uint32_t seed)
{
	for (size_t k = 0; k < len; k++) {
		xt_buf[i][k] = (uint8_t)((k * 131 + seed * 7 + (k >> 8)) ^ seed);
	}
	sys_cache_data_flush_range(xt_buf[i], len);
}

static void xt_clear(int i, size_t len)
{
	memset(xt_buf[i], 0, len);
	sys_cache_data_flush_range(xt_buf[i], len);
}

static bool xt_check(const struct shell *sh, const char *what, int ret,
		     const struct a3700_xor_result *r, int dst, int src, size_t len)
{
	sys_cache_data_invd_range(xt_buf[dst], len);
	if (ret != 0) {
		shell_error(sh, "%s: %d (channel %u, cause %04x, error %08x @%08x, desc %u)", what,
			    ret, r->chan, r->cause, r->err, r->err_addr, r->failed);
		return false;
	}
	if (src >= 0 && memcmp(xt_buf[dst], xt_buf[src], len) != 0) {
		shell_error(sh, "%s: data differ", what);
		return false;
	}
	shell_print(sh, "%-28s ok  channel %u %s, write-back %u us", what, r->chan,
		    r->irq ? "irq " : "poll", r->wb_wait_us);
	return true;
}

/* channel 1 runs from the system work queue while we drive channel 0 */
static struct k_work xt_work;
static struct k_sem xt_done;
static volatile int xt_ret;
static struct a3700_xor_result xt_res;
static size_t xt_len;

static void xt_other(struct k_work *w)
{
	ARG_UNUSED(w);
	const struct a3700_xor_seg s = {.dst = xt_phys(3, 0), .src = xt_phys(2, 0),
					.len = (uint32_t)xt_len};

	xt_ret = a3700_xor_memcpy_sg(1, &s, 1, &xt_res);
	k_sem_give(&xt_done);
}

static int cmd_xortest(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long size = 4096, loops = 256;
	struct a3700_xor_result r;
	struct a3700_xor_seg seg[4];
	uint32_t src[A3700_XOR_MAX_SRC];
	struct a3700_xor_stats st;
	int ret, ok = 0, tests = 0;
	size_t len;

	if ((argc > 1 && parse_u(sh, argv[1], &size) != 0) ||
	    (argc > 2 && parse_u(sh, argv[2], &loops) != 0)) {
		return -EINVAL;
	}
	if (size < 4 * A3700_XOR_MIN_LEN || size > XT_MAX || (size & 3)) {
		shell_error(sh, "size %u..%u, multiple of 4", 4 * A3700_XOR_MIN_LEN, XT_MAX);
		return -EINVAL;
	}
	len = size;

	/* memcpy, each channel */
	for (int c = 0; c < A3700_XOR_CHANNELS; c++) {
		const struct a3700_xor_seg s = {.dst = xt_phys(1, 0), .src = xt_phys(0, 0),
						.len = (uint32_t)len};

		xt_fill(0, len, 0x10 + c);
		xt_clear(1, len);
		ret = a3700_xor_memcpy_sg(c, &s, 1, &r);
		ok += xt_check(sh, c ? "memcpy channel 1" : "memcpy channel 0", ret, &r, 1, 0,
			       len);
		tests++;
	}

	/* chain: four quarters, reversed order */
	xt_fill(0, len, 0x33);
	xt_clear(1, len);
	for (int i = 0; i < 4; i++) {
		size_t q = len / 4;

		seg[i] = (struct a3700_xor_seg){
			.dst = xt_phys(1, (3 - i) * q), .src = xt_phys(0, (3 - i) * q),
			.len = (uint32_t)q};
	}
	ret = a3700_xor_memcpy_sg(A3700_XOR_ANY, seg, 4, &r);
	ok += xt_check(sh, "chain of 4 descriptors", ret, &r, 1, 0, len);
	tests++;

	/* XOR of 8 sources, result checked against the CPU's */
	for (int i = 0; i < A3700_XOR_MAX_SRC; i++) {
		xt_fill(2 + i < XT_BUFS ? 2 + i : 0, len, 0x40 + i);
		src[i] = xt_phys(2 + i, 0);
	}
	xt_clear(1, len);
	ret = a3700_xor_xor(A3700_XOR_ANY, xt_phys(1, 0), src, A3700_XOR_MAX_SRC, (uint32_t)len,
			    &r);
	if (ret == 0) {
		memset(xt_buf[0], 0, len);
		for (int i = 0; i < A3700_XOR_MAX_SRC; i++) {
			for (size_t k = 0; k < len; k++) {
				xt_buf[0][k] ^= xt_buf[2 + i][k];
			}
		}
	}
	ok += xt_check(sh, "xor of 8 sources", ret, &r, 1, 0, len);
	tests++;

	/* both channels at once: channel 1 from a thread */
	xt_fill(0, len, 0x55);
	xt_fill(2, len, 0x66);
	xt_clear(1, len);
	xt_clear(3, len);
	xt_len = len;
	k_sem_init(&xt_done, 0, 1);
	k_work_init(&xt_work, xt_other);
	k_work_submit(&xt_work);
	{
		const struct a3700_xor_seg s = {.dst = xt_phys(1, 0), .src = xt_phys(0, 0),
						.len = (uint32_t)len};

		ret = a3700_xor_memcpy_sg(0, &s, 1, &r);
	}
	k_sem_take(&xt_done, K_FOREVER);
	ok += xt_check(sh, "parallel: channel 0", ret, &r, 1, 0, len);
	ok += xt_check(sh, "parallel: channel 1", xt_ret, &xt_res, 3, 2, len);
	tests += 2;

	/* throughput, DRAM to DRAM */
	{
		const struct a3700_xor_seg s = {.dst = xt_phys(1, 0), .src = xt_phys(0, 0),
						.len = (uint32_t)len};
		uint64_t t0 = k_cycle_get_64(), ns;

		for (unsigned long long i = 0; i < loops; i++) {
			ret = a3700_xor_memcpy_sg(A3700_XOR_ANY, &s, 1, &r);
			if (ret != 0) {
				break;
			}
		}
		ns = k_cyc_to_ns_floor64(k_cycle_get_64() - t0);
		if (ret == 0 && ns != 0) {
			shell_print(sh, "throughput: %llu x %zu B in %llu us = %llu kB/s, %llu us "
				    "per transfer", loops, len, ns / 1000,
				    (uint64_t)loops * len * 1000000ULL / ns, ns / 1000 / loops);
		}
	}

	for (int c = 0; c < A3700_XOR_CHANNELS; c++) {
		a3700_xor_get_stats(c, &st);
		shell_print(sh, "channel %d: %u ops, %u descriptors, %llu bytes, %u irqs, %u polls, "
			    "%u errors, %u timeouts", c, st.ops, st.descs, st.bytes, st.irqs,
			    st.polls, st.errors, st.timeouts);
	}
	shell_print(sh, "%d of %d tests passed", ok, tests);
	return ok == tests ? 0 : -EIO;
}

LAB_DBG_CMD((pcie_debug), "pcie", xortest, NULL,
	    "[size] [loops] XOR engine self-test on DRAM: both channels, chain, xor of 8, "
	    "parallel, throughput",
	    cmd_xortest, 1, 2, NULL, NULL);
