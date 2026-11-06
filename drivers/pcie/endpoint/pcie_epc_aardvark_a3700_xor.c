/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Armada 3700 XOR engine (one unit, two channels) used as system DMA for
 * the Aardvark endpoint, programmed as Linux mv_xor does on the A3700:
 * one 4 GiB XOR window per channel, the AXI level decoding the address.
 * The DMA unit's IO decode windows (0xd000c200, 2 x 0x10) only cover
 * DRAM after TF-A; window 1 is pointed at the PCIe outbound region (CPU
 * window target 2), else a transfer to or from host memory ends with
 * ERR_MBUS.
 *
 * Descriptor mode (operation in the descriptor): chains of memcpy
 * descriptors (scatter-gather) or one XOR descriptor with up to eight
 * sources. Completion by the channel's interrupt (GIC SPI 47 / 48 as in
 * Linux armada-37xx.dtsi) or by polling the cause register; then the
 * descriptors' write-back is awaited (it can come after end of chain).
 *
 * Register layout (Linux mv_xor.h): low block per channel at +4 * chan
 * (config, activation), cause and mask shared with 16 bits per channel
 * (written 0 to clear); high block next/current descriptor at
 * +4 * chan, windows shared, window enable and override per channel.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/dt-bindings/interrupt-controller/arm-gic.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_epc_aardvark_a3700_xor.h"

#define XOR_LOW  0xd0060900UL
#define XOR_HIGH 0xd0060b00UL

/* low block */
#define XOR_CONFIG(c)     (0x10 + 4 * (c))
#define XOR_ACTIVATION(c) (0x20 + 4 * (c))
#define XOR_CAUSE         0x30
#define XOR_MASK          0x40
#define XOR_ERR_CAUSE     0x50
#define XOR_ERR_ADDR      0x60
/* high block */
#define XOR_NEXT_DESC(c)  (0x00 + 4 * (c))
#define XOR_BAR_EN(c)     (0x40 + 4 * (c))
#define XOR_WIN_BASE(w)   (0x50 + 4 * (w))
#define XOR_WIN_SIZE(w)   (0x70 + 4 * (w))
#define XOR_WIN_REMAP(w)  (0x90 + 4 * (w))
#define XOR_WIN_OVRD(c)   (0xa0 + 4 * (c))

#define XOR_ACT_START   BIT(0)
#define XOR_ACT_STOP    BIT(1)
#define XOR_CFG_IN_DESC 7U

/* cause / mask, per channel, shifted by 16 * chan */
#define XOR_INT_EOC     BIT(1)
#define XOR_INT_STOPPED BIT(2)
#define XOR_INT_ERRS    GENMASK(9, 4)
#define XOR_INT_ALL     (XOR_INT_EOC | XOR_INT_STOPPED | XOR_INT_ERRS)

#define XOR_DESC_OWNER   BIT(31)
#define XOR_DESC_SUCCESS BIT(30)
#define XOR_OP_XOR       (0U << 24)
#define XOR_OP_MEMCPY    (2U << 24)

/* GIC SPI 47, 48: xor10, xor11 */
#define XOR_IRQ(c) (32 + 47 + (c))

#define XOR_TIMEOUT_MS 1000

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

struct xor_chan {
	struct k_mutex lock;
	struct k_sem done;
	volatile uint32_t cause; /* from the interrupt */
	struct xor_desc desc[A3700_XOR_MAX_SEGS];
	struct a3700_xor_stats st;
};

static struct xor_chan ch[A3700_XOR_CHANNELS];
static uint8_t *lo, *hi;
static struct k_spinlock reg_lock; /* shared cause / mask */
static bool ready;
static K_MUTEX_DEFINE(init_lock);

static inline uint32_t lrd(uint32_t off)
{
	return sys_read32((mem_addr_t)lo + off);
}

static inline void lwr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)lo + off);
}

static inline void hwr(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)hi + off);
}

static inline uint32_t chan_cause(int c)
{
	return (lrd(XOR_CAUSE) >> (16 * c)) & 0xffff;
}

/* the cause bits are cleared by writing 0, ones leave the rest alone */
static inline void chan_clear(int c, uint32_t bits)
{
	lwr(XOR_CAUSE, ~(bits << (16 * c)));
}

#ifdef CONFIG_PCIE_EPC_AARDVARK_XOR_IRQ
static void xor_isr(const void *arg)
{
	int c = (int)(uintptr_t)arg;
	k_spinlock_key_t key = k_spin_lock(&reg_lock);
	uint32_t cause = chan_cause(c) & XOR_INT_ALL;

	if (cause != 0) {
		chan_clear(c, cause);
		ch[c].cause |= cause;
		ch[c].st.irqs++;
	}
	k_spin_unlock(&reg_lock, key);
	if (cause != 0) {
		k_sem_give(&ch[c].done);
	}
}
#endif

static void xor_init(void)
{
	k_mutex_lock(&init_lock, K_FOREVER);
	if (ready) {
		k_mutex_unlock(&init_lock);
		return;
	}
	k_mem_map_phys_bare(&lo, XOR_LOW, 0x100, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	k_mem_map_phys_bare(&hi, XOR_HIGH, 0x100, K_MEM_CACHE_NONE | K_MEM_PERM_RW);

	/* as Linux mv_xor_conf_mbus_windows_a3700: one 4 GiB window */
	for (int w = 0; w < 8; w++) {
		hwr(XOR_WIN_BASE(w), 0);
		hwr(XOR_WIN_SIZE(w), 0);
		if (w < 4) {
			hwr(XOR_WIN_REMAP(w), 0);
		}
	}
	hwr(XOR_WIN_SIZE(0), 0xffff0000);
	for (int c = 0; c < A3700_XOR_CHANNELS; c++) {
		k_mutex_init(&ch[c].lock);
		k_sem_init(&ch[c].done, 0, 1);
		hwr(XOR_BAR_EN(c), 1 | (3 << 16));
		hwr(XOR_WIN_OVRD(c), 0);
		lwr(XOR_ACTIVATION(c), XOR_ACT_STOP);
		/* descriptor mode, no swap */
		lwr(XOR_CONFIG(c), (lrd(XOR_CONFIG(c)) & ~(0x7U | BIT(14))) | XOR_CFG_IN_DESC);
		chan_clear(c, 0xffff);
	}
#ifdef CONFIG_PCIE_EPC_AARDVARK_XOR_IRQ
	IRQ_CONNECT(XOR_IRQ(0), IRQ_DEFAULT_PRIORITY, xor_isr, (void *)0, IRQ_TYPE_LEVEL);
	IRQ_CONNECT(XOR_IRQ(1), IRQ_DEFAULT_PRIORITY, xor_isr, (void *)1, IRQ_TYPE_LEVEL);
	lwr(XOR_MASK, (XOR_INT_ALL << 16) | XOR_INT_ALL);
	irq_enable(XOR_IRQ(0));
	irq_enable(XOR_IRQ(1));
#else
	lwr(XOR_MASK, 0);
#endif
	ready = true;
	k_mutex_unlock(&init_lock);
}

void a3700_xor_pcie_window(uint32_t base, uint32_t size, uint8_t attr)
{
	uint8_t *va;
	volatile uint32_t *w;
	uint32_t ctrl = ((size >> 16) - 1) << 16 | (uint32_t)attr << 8 | 2U << 4;

	k_mem_map_phys_bare(&va, 0xd000c000UL, 0x1000, K_MEM_CACHE_NONE | K_MEM_PERM_RW);
	/* DMA unit IO decode window 1 (0xd000c210) -> PCIe (target 2) */
	w = (volatile uint32_t *)(va + 0x200 + 1 * 0x10);
	w[0] = 0;
	w[1] = base & 0xffff0000U;
	w[0] = ctrl;
	w[0] = ctrl | 1;
	barrier_dsync_fence_full();
	k_mem_unmap_phys_bare(va, 0x1000);
}

/* a free channel, locked */
static int chan_take(int want)
{
	if (want >= 0) {
		k_mutex_lock(&ch[want].lock, K_FOREVER);
		return want;
	}
	for (int c = 0; c < A3700_XOR_CHANNELS; c++) {
		if (k_mutex_lock(&ch[c].lock, K_NO_WAIT) == 0) {
			return c;
		}
	}
	/* both busy: spread the waiters */
	want = (int)(((uintptr_t)k_current_get() >> 6) & 1);
	k_mutex_lock(&ch[want].lock, K_FOREVER);
	return want;
}

/* run desc[0..n-1] on channel c (locked), wait, check the write-back */
static int chan_run(int c, int n, struct a3700_xor_result *res)
{
	struct xor_chan *x = &ch[c];
	struct a3700_xor_result r = {.chan = c, .failed = 0xff};
	k_spinlock_key_t key;
	int64_t end;
	int ret;

	for (int i = 0; i < n; i++) {
		x->desc[i].status = XOR_DESC_OWNER;
		x->desc[i].next = i + 1 < n ? (uint32_t)k_mem_phys_addr(&x->desc[i + 1]) : 0;
	}
	sys_cache_data_flush_range(x->desc, n * sizeof(x->desc[0]));
	barrier_dsync_fence_full();

	key = k_spin_lock(&reg_lock);
	chan_clear(c, 0xffff);
	x->cause = 0;
	k_spin_unlock(&reg_lock, key);
	k_sem_reset(&x->done);
	hwr(XOR_NEXT_DESC(c), (uint32_t)k_mem_phys_addr(&x->desc[0]));
	lwr(XOR_ACTIVATION(c), XOR_ACT_START);

#ifdef CONFIG_PCIE_EPC_AARDVARK_XOR_IRQ
	if (k_sem_take(&x->done, K_MSEC(XOR_TIMEOUT_MS)) == 0) {
		r.cause = x->cause;
		r.irq = true;
	}
#endif
	if (!r.irq) {
		/* no interrupt (or it never came): poll */
		end = k_uptime_get() + XOR_TIMEOUT_MS;
		do {
			r.cause = chan_cause(c) | x->cause;
			if (r.cause & XOR_INT_ALL) {
				break;
			}
		} while (k_uptime_get() < end);
		x->st.polls++;
	}
	r.err = lrd(XOR_ERR_CAUSE);
	r.err_addr = lrd(XOR_ERR_ADDR);

	/*
	 * End of chain can come before the descriptors' status is written
	 * back (seen toward PCIe at Gen1: EOC, no error bits, owner still
	 * set): wait for every descriptor.
	 */
	if ((r.cause & XOR_INT_EOC) && !(r.cause & XOR_INT_ERRS)) {
		int64_t wb_end = k_uptime_get() + 100;
		uint64_t t0 = k_cycle_get_64();
		bool pending;

		do {
			sys_cache_data_invd_range(x->desc, n * sizeof(x->desc[0]));
			pending = false;
			for (int i = 0; i < n; i++) {
				pending |= (x->desc[i].status & XOR_DESC_OWNER) != 0;
			}
		} while (pending && k_uptime_get() < wb_end);
		r.wb_wait_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - t0);
	} else {
		sys_cache_data_invd_range(x->desc, n * sizeof(x->desc[0]));
	}
	for (int i = 0; i < n; i++) {
		if ((x->desc[i].status & (XOR_DESC_OWNER | XOR_DESC_SUCCESS)) != XOR_DESC_SUCCESS) {
			r.failed = i;
			break;
		}
	}
	r.status = x->desc[n - 1].status;

	if (!(r.cause & XOR_INT_ALL)) {
		ret = -ETIMEDOUT;
		x->st.timeouts++;
	} else if ((r.cause & XOR_INT_ERRS) || r.failed != 0xff) {
		ret = -EIO;
		x->st.errors++;
	} else {
		ret = 0;
	}
	if (ret != 0) {
		lwr(XOR_ACTIVATION(c), XOR_ACT_STOP);
	}
	key = k_spin_lock(&reg_lock);
	chan_clear(c, 0xffff);
	k_spin_unlock(&reg_lock, key);
	x->st.ops++;
	x->st.descs += n;
	if (res != NULL) {
		*res = r;
	}
	return ret;
}

int a3700_xor_memcpy_sg(int chan, const struct a3700_xor_seg *seg, int n,
			struct a3700_xor_result *res)
{
	uint64_t bytes = 0;
	int c, ret;

	if (n < 1 || n > A3700_XOR_MAX_SEGS || chan >= A3700_XOR_CHANNELS) {
		return -EINVAL;
	}
	for (int i = 0; i < n; i++) {
		if (seg[i].len < A3700_XOR_MIN_LEN || seg[i].len > A3700_XOR_MAX_LEN) {
			return -EINVAL;
		}
		bytes += seg[i].len;
	}
	xor_init();
	c = chan_take(chan);
	for (int i = 0; i < n; i++) {
		struct xor_desc *d = &ch[c].desc[i];

		memset(d, 0, sizeof(*d));
		d->command = XOR_OP_MEMCPY | BIT(0);
		d->byte_count = seg[i].len;
		d->dst = seg[i].dst;
		d->src[0] = seg[i].src;
	}
	ret = chan_run(c, n, res);
	if (ret == 0) {
		ch[c].st.bytes += bytes;
	}
	k_mutex_unlock(&ch[c].lock);
	return ret;
}

int a3700_xor_memcpy(uint32_t dst, uint32_t src, uint32_t len, struct a3700_xor_result *res)
{
	const struct a3700_xor_seg s = {.dst = dst, .src = src, .len = len};

	return a3700_xor_memcpy_sg(A3700_XOR_ANY, &s, 1, res);
}

int a3700_xor_xor(int chan, uint32_t dst, const uint32_t *src, int nsrc, uint32_t len,
		  struct a3700_xor_result *res)
{
	struct xor_desc *d;
	int c, ret;

	if (nsrc < 2 || nsrc > A3700_XOR_MAX_SRC || len < A3700_XOR_MIN_LEN ||
	    len > A3700_XOR_MAX_LEN || chan >= A3700_XOR_CHANNELS) {
		return -EINVAL;
	}
	xor_init();
	c = chan_take(chan);
	d = &ch[c].desc[0];
	memset(d, 0, sizeof(*d));
	d->command = XOR_OP_XOR | GENMASK(nsrc - 1, 0);
	d->byte_count = len;
	d->dst = dst;
	for (int i = 0; i < nsrc; i++) {
		d->src[i] = src[i];
	}
	ret = chan_run(c, 1, res);
	if (ret == 0) {
		ch[c].st.bytes += len;
	}
	k_mutex_unlock(&ch[c].lock);
	return ret;
}

void a3700_xor_get_stats(int chan, struct a3700_xor_stats *st)
{
	if (chan < 0 || chan >= A3700_XOR_CHANNELS) {
		memset(st, 0, sizeof(*st));
		return;
	}
	*st = ch[chan].st;
}
