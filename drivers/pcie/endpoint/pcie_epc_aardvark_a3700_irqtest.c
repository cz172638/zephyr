/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie debug irqtest: which host actions set which bit of the Aardvark
 * central interrupt register (HOST_CTRL_INT_STATUS 0x1b000) in endpoint
 * mode, and whether the bit reaches the GIC (SPI 29).
 *
 * The bits of interest are RC_DBELL_DET (22), a doorbell from the host,
 * and DIR_WR_DDR_DET (14), presumably an inbound write to DDR. Linux
 * defines both and uses neither; Marvell documents neither.
 *
 * Three ways to arm the test:
 *   irq   bits unmasked in 0x1b004, GIC line enabled, the ISR logs and
 *         clears (W1C) what it finds
 *   open  bits unmasked, GIC line left disabled; a timer samples the
 *         status and the GIC pending bit (does the line assert at all?)
 *   poll  mask left as it is (masked); a timer samples the status
 *         (does the status latch while masked?)
 *
 * Every event keeps the raw status, ISR0/ISR1, the GIC pending bit and
 * up to IRQT_REGS words of a register range chosen with "irqtest regs"
 * (for finding where a doorbell value or address lands).
 *
 * The GIC line: without the root complex driver SPI 29 is connected
 * here statically. With it, the root complex driver owns the static
 * entry; with CONFIG_DYNAMIC_INTERRUPTS the entry is taken over at run
 * time (only in endpoint mode, where the root complex driver is idle),
 * otherwise only open and poll work.
 */

#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pcie/aardvark.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_GIC)
#include <zephyr/drivers/interrupt_controller/gic.h>
#endif

#include <soc.h>

#include "pcie_epc_aardvark.h"

#define EP_NODE      DT_INST(0, marvell_armada_3700_pcie_ep)
#define EP_DEV       DEVICE_DT_GET(EP_NODE)
#define EP_IRQ       DT_IRQN(EP_NODE)
#define EP_IRQ_PRIO  DT_IRQ(EP_NODE, priority)
#define EP_IRQ_FLAGS DT_IRQ(EP_NODE, flags)

#if !defined(CONFIG_PCIE_AARDVARK)
#define IRQT_LINE_STATIC 1
#define IRQT_HAVE_LINE   1
#define IRQT_LINE_HOW    "static"
#elif defined(CONFIG_DYNAMIC_INTERRUPTS) && !defined(CONFIG_SHARED_INTERRUPTS)
#define IRQT_LINE_DYNAMIC 1
#define IRQT_HAVE_LINE    1
#define IRQT_LINE_HOW     "taken over from the root complex driver"
#else
#define IRQT_HAVE_LINE 0
#define IRQT_LINE_HOW  "not available (root complex driver owns it, no DYNAMIC_INTERRUPTS)"
#endif

#define IRQT_LOG     32
#define IRQT_REGS    8
/* ISR entries per second above which the line is switched off */
#define IRQT_STORM   2000
#define IRQT_DEFAULT (BIT(22) | BIT(14))

/* 0x1b000 bits as Linux pci-aardvark.c names them, plus a short name */
static const struct {
	uint8_t bit;
	const char *name;
	const char *alias;
} irqt_names[] = {
	{0, "CMDQ_INT", "cmdq"},	   {1, "MSI_STATUS_INT", "msi"},
	{3, "CMD_SENT_DONE", "cmdsent"},   {4, "DMA_INT", "dma"},
	{5, "IB_DXFERDONE", "ibdone"},	   {6, "OB_DXFERDONE", "obdone"},
	{7, "OB_RXFERDONE", "obrdone"},	   {12, "COMPQ_INT", "compq"},
	{13, "DIR_RD_DDR_DET", "rdddr"},   {14, "DIR_WR_DDR_DET", "wrddr"},
	{16, "CORE_INT", "core"},	   {17, "CORE_INT_PIO", "corepio"},
	{18, "DPMU_INT", "dpmu"},	   {19, "PCIE_MIS_INT", "mis"},
	{20, "MSI_INT1_DET", "msi1"},	   {21, "MSI_INT2_DET", "msi2"},
	{22, "RC_DBELL_DET", "dbell"},	   {23, "EP_STATUS", "epstatus"},
};

enum irqt_how {
	IRQT_OFF,
	IRQT_IRQ,
	IRQT_OPEN,
	IRQT_POLL,
};

static const char *const irqt_how_name[] = {"off", "irq", "open", "poll"};

struct irqt_event {
	uint32_t us;
	uint32_t host;
	uint32_t isr0;
	uint32_t isr1;
	uint32_t regs[IRQT_REGS];
	char src;	/* i: ISR, p: timer, k: kick */
	bool gic;	/* SPI pending when sampled */
};

static struct {
	struct k_spinlock lock;
	enum irqt_how how;
	uint32_t bits;
	uint32_t mask_saved;
	uint32_t period_ms;
	bool line_on;
	bool connected;
	bool kick;
	bool storm;
	uint64_t t0;
	struct k_timer timer;
	struct k_sem ev;
	/* counters since arm */
	uint32_t cnt[32];
	uint32_t entries;
	uint32_t empty;
	uint32_t polls;
	uint32_t gic_seen;
	uint32_t events;
	uint32_t win_ms;
	uint32_t win_n;
	/* extra registers captured with every event */
	uint32_t reg_off;
	uint8_t reg_n;
	struct irqt_event log[IRQT_LOG];
} irqt;

static bool irqt_gic_pending(void)
{
#if defined(CONFIG_GIC)
	return arm_gic_irq_is_pending(EP_IRQ);
#else
	return false;
#endif
}

/* Log an event, clear (W1C) the armed bits that are set; lock held */
static void irqt_take(uint32_t host, char src, bool gic, bool force)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	uint32_t hit = host & irqt.bits;
	struct irqt_event *e;

	if (hit == 0 && !force) {
		return;
	}
	if (hit != 0) {
		advk_wr(ep, ADVK_HOST_INT_STATUS, hit);
	}
	for (uint32_t m = hit; m != 0; m &= m - 1) {
		irqt.cnt[find_lsb_set(m) - 1]++;
	}

	e = &irqt.log[irqt.events % IRQT_LOG];
	e->us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - irqt.t0);
	e->host = host;
	e->isr0 = advk_rd(ep, ADVK_ISR0);
	e->isr1 = advk_rd(ep, ADVK_ISR1);
	for (int i = 0; i < irqt.reg_n; i++) {
		e->regs[i] = advk_rd(ep, irqt.reg_off + 4 * i);
	}
	e->src = src;
	e->gic = gic;
	irqt.events++;
	k_sem_give(&irqt.ev);
}

static void irqt_tick(struct k_timer *t)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	k_spinlock_key_t key = k_spin_lock(&irqt.lock);
	uint32_t host;
	bool gic;

	ARG_UNUSED(t);
	if (irqt.how == IRQT_OFF) {
		k_spin_unlock(&irqt.lock, key);
		return;
	}
	host = advk_rd(ep, ADVK_HOST_INT_STATUS);
	gic = irqt_gic_pending();
	irqt.polls++;
	if (gic) {
		irqt.gic_seen++;
	}
	irqt_take(host, 'p', gic, false);
#if defined(CONFIG_GIC)
	/* level line, disabled: drop the pending state once the source is cleared */
	if (gic && !irqt.line_on) {
		arm_gic_irq_clear_pending(EP_IRQ);
	}
#endif
	k_spin_unlock(&irqt.lock, key);
}

#if IRQT_HAVE_LINE
static void irqt_isr(const void *arg)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	k_spinlock_key_t key = k_spin_lock(&irqt.lock);
	uint32_t host = advk_rd(ep, ADVK_HOST_INT_STATUS);
	uint32_t now = k_uptime_get_32();
	bool kick = irqt.kick;

	ARG_UNUSED(arg);
	irqt.entries++;
	irqt.kick = false;
	if ((host & irqt.bits) == 0 && !kick) {
		irqt.empty++;
	}
	irqt_take(host, kick ? 'k' : 'i', true, kick);

	if (now - irqt.win_ms >= 1000U) {
		irqt.win_ms = now;
		irqt.win_n = 0;
	}
	if (++irqt.win_n > IRQT_STORM) {
		/* line off, mask back, go on by sampling the status */
		irq_disable(EP_IRQ);
		irqt.line_on = false;
		advk_wr(ep, ADVK_HOST_INT_MASK,
			(advk_rd(ep, ADVK_HOST_INT_MASK) & ~irqt.bits) |
				(irqt.mask_saved & irqt.bits));
		irqt.storm = true;
		irqt.how = IRQT_POLL;
		k_timer_start(&irqt.timer, K_MSEC(irqt.period_ms), K_MSEC(irqt.period_ms));
	}
	k_spin_unlock(&irqt.lock, key);
}
#endif

static int irqt_connect(void)
{
#if defined(IRQT_LINE_STATIC)
	if (!irqt.connected) {
		IRQ_CONNECT(EP_IRQ, EP_IRQ_PRIO, irqt_isr, NULL, EP_IRQ_FLAGS);
		irqt.connected = true;
	}
	return 0;
#elif defined(IRQT_LINE_DYNAMIC)
	if (!irqt.connected) {
		int ret = irq_connect_dynamic(EP_IRQ, EP_IRQ_PRIO, irqt_isr, NULL, EP_IRQ_FLAGS);

		if (ret < 0) {
			return ret;
		}
		irqt.connected = true;
	}
	return 0;
#else
	return -ENOTSUP;
#endif
}

static void irqt_stop(void)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	k_spinlock_key_t key;

	k_timer_stop(&irqt.timer);
	key = k_spin_lock(&irqt.lock);
	if (irqt.line_on) {
		irq_disable(EP_IRQ);
		irqt.line_on = false;
	}
	if (irqt.how == IRQT_IRQ || irqt.how == IRQT_OPEN) {
		advk_wr(ep, ADVK_HOST_INT_MASK,
			(advk_rd(ep, ADVK_HOST_INT_MASK) & ~irqt.bits) |
				(irqt.mask_saved & irqt.bits));
	}
	irqt.how = IRQT_OFF;
	k_spin_unlock(&irqt.lock, key);
}

static void bits_print(const struct shell *sh, uint32_t bits)
{
	for (int i = 0; i < ARRAY_SIZE(irqt_names); i++) {
		if ((bits & BIT(irqt_names[i].bit)) != 0) {
			shell_fprintf(sh, SHELL_NORMAL, " %s", irqt_names[i].name);
			bits &= ~BIT(irqt_names[i].bit);
		}
	}
	for (; bits != 0; bits &= bits - 1) {
		shell_fprintf(sh, SHELL_NORMAL, " bit%d", find_lsb_set(bits) - 1);
	}
}

/* "dbell,wrddr", "RC_DBELL_DET", "all" or a number */
static int bits_parse(const struct shell *sh, const char *arg, uint32_t *out)
{
	char buf[128];
	char *save;
	uint32_t bits = 0;

	if (strlen(arg) >= sizeof(buf)) {
		return -EINVAL;
	}
	strcpy(buf, arg);
	for (char *t = strtok_r(buf, ",", &save); t != NULL; t = strtok_r(NULL, ",", &save)) {
		char *end;
		unsigned long v;
		bool found = false;

		if (strcasecmp(t, "all") == 0) {
			for (int i = 0; i < ARRAY_SIZE(irqt_names); i++) {
				bits |= BIT(irqt_names[i].bit);
			}
			continue;
		}
		for (int i = 0; i < ARRAY_SIZE(irqt_names); i++) {
			if (strcasecmp(t, irqt_names[i].name) == 0 ||
			    strcasecmp(t, irqt_names[i].alias) == 0) {
				bits |= BIT(irqt_names[i].bit);
				found = true;
				break;
			}
		}
		if (found) {
			continue;
		}
		v = strtoul(t, &end, 0);
		if (*end != '\0' || v == 0 || v > UINT32_MAX) {
			shell_error(sh, "unknown bit '%s'", t);
			return -EINVAL;
		}
		bits |= (uint32_t)v;
	}
	if (bits == 0) {
		return -EINVAL;
	}
	*out = bits;
	return 0;
}

static void event_print(const struct shell *sh, const struct irqt_event *e)
{
	shell_fprintf(sh, SHELL_NORMAL, "%10u us %c gic %d host %08x isr0 %08x isr1 %08x ", e->us,
		      e->src, e->gic, e->host, e->isr0, e->isr1);
	bits_print(sh, e->host & irqt.bits);
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	if (irqt.reg_n != 0) {
		shell_fprintf(sh, SHELL_NORMAL, "%15s %05x:", "", irqt.reg_off);
		for (int i = 0; i < irqt.reg_n; i++) {
			shell_fprintf(sh, SHELL_NORMAL, " %08x", e->regs[i]);
		}
		shell_fprintf(sh, SHELL_NORMAL, "\n");
	}
}

/* Endpoint mode, controller clocked, the central interrupt block answers */
static int irqt_usable(const struct shell *sh)
{
	uint32_t v;

	if (advk_mode_get() != ADVK_MODE_EP) {
		shell_error(sh, "controller is a root complex this boot");
		return -ENODEV;
	}
	if (!device_is_ready(EP_DEV)) {
		shell_error(sh, "endpoint controller not ready");
		return -ENODEV;
	}
	if (a3700_probe_read32(advk_ep(EP_DEV)->reg + ADVK_HOST_INT_STATUS, &v) < 0) {
		shell_error(sh, "0x1b000 read aborts (clock off?)");
		return -EIO;
	}
	return 0;
}

static void irqt_init_once(void)
{
	static bool done;

	if (!done) {
		k_timer_init(&irqt.timer, irqt_tick, NULL);
		k_sem_init(&irqt.ev, 0, 1);
		irqt.period_ms = 1;
		done = true;
	}
}

static int cmd_show(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep;
	uint32_t mask;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	irqt_init_once();
	ret = irqt_usable(sh);
	if (ret != 0) {
		return ret;
	}
	ep = advk_ep(EP_DEV);
	mask = advk_rd(ep, ADVK_HOST_INT_MASK);

	shell_fprintf(sh, SHELL_NORMAL, "armed      %s", irqt_how_name[irqt.how]);
	if (irqt.how == IRQT_OPEN || irqt.how == IRQT_POLL) {
		shell_fprintf(sh, SHELL_NORMAL, " every %u ms", irqt.period_ms);
	}
	shell_fprintf(sh, SHELL_NORMAL, ", bits %08x", irqt.bits);
	bits_print(sh, irqt.bits);
	shell_fprintf(sh, SHELL_NORMAL, "%s\n", irqt.storm ? "  (storm: line switched off)" : "");
	shell_print(sh, "line       SPI %d (intid %d) %s, enabled %d, pending %d", EP_IRQ - 32,
		    EP_IRQ, IRQT_LINE_HOW, irqt.line_on, irqt_gic_pending());
	shell_fprintf(sh, SHELL_NORMAL, "0x1b000    %08x", advk_rd(ep, ADVK_HOST_INT_STATUS));
	bits_print(sh, advk_rd(ep, ADVK_HOST_INT_STATUS));
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	shell_print(sh, "0x1b004    %08x%s", mask,
		    (irqt.how == IRQT_IRQ || irqt.how == IRQT_OPEN) && (mask & irqt.bits) != 0
			    ? "  armed bits masked again (controller set up anew?): arm again"
			    : "");
	shell_print(sh, "isr0       %08x mask %08x  isr1 %08x mask %08x", advk_rd(ep, ADVK_ISR0),
		    advk_rd(ep, ADVK_ISR0_MASK), advk_rd(ep, ADVK_ISR1),
		    advk_rd(ep, ADVK_ISR1_MASK));
	shell_print(sh, "counts     events %u, isr entries %u (without an armed bit %u), "
			"samples %u (gic pending in %u)",
		    irqt.events, irqt.entries, irqt.empty, irqt.polls, irqt.gic_seen);
	for (int b = 0; b < 32; b++) {
		if (irqt.cnt[b] != 0) {
			shell_fprintf(sh, SHELL_NORMAL, "%10u ", irqt.cnt[b]);
			bits_print(sh, BIT(b));
			shell_fprintf(sh, SHELL_NORMAL, "\n");
		}
	}
	if (irqt.reg_n != 0) {
		shell_print(sh, "regs       %05x + %u words with every event", irqt.reg_off,
			    irqt.reg_n);
	}
	return 0;
}

/* arm [irq|open|poll] [bits] [ms] */
static int cmd_arm(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep;
	enum irqt_how how = IRQT_HAVE_LINE ? IRQT_IRQ : IRQT_OPEN;
	uint32_t bits = IRQT_DEFAULT;
	unsigned long ms = 1;
	uint32_t latched;
	k_spinlock_key_t key;
	int ret;

	irqt_init_once();
	ret = irqt_usable(sh);
	if (ret != 0) {
		return ret;
	}
	if (argc > 1) {
		int i;

		for (i = IRQT_IRQ; i <= IRQT_POLL; i++) {
			if (strcmp(argv[1], irqt_how_name[i]) == 0) {
				break;
			}
		}
		if (i > IRQT_POLL) {
			shell_error(sh, "irq, open or poll");
			return -EINVAL;
		}
		how = i;
	}
	if (argc > 2 && bits_parse(sh, argv[2], &bits) != 0) {
		return -EINVAL;
	}
	if (argc > 3) {
		char *end;

		ms = strtoul(argv[3], &end, 0);
		if (*end != '\0' || ms == 0 || ms > 10000) {
			shell_error(sh, "period 1..10000 ms");
			return -EINVAL;
		}
	}
	if (how == IRQT_IRQ) {
		ret = irqt_connect();
		if (ret != 0) {
			shell_error(sh, "GIC line %s; use open or poll", IRQT_LINE_HOW);
			return ret;
		}
	}

	irqt_stop();
	ep = advk_ep(EP_DEV);
	key = k_spin_lock(&irqt.lock);
	memset(irqt.cnt, 0, sizeof(irqt.cnt));
	irqt.entries = irqt.empty = irqt.polls = irqt.gic_seen = irqt.events = 0;
	irqt.win_n = 0;
	irqt.win_ms = k_uptime_get_32();
	irqt.storm = false;
	irqt.kick = false;
	irqt.bits = bits;
	irqt.period_ms = ms;
	irqt.t0 = k_cycle_get_64();
	/* whatever latched before: report, then clear */
	latched = advk_rd(ep, ADVK_HOST_INT_STATUS);
	advk_wr(ep, ADVK_HOST_INT_STATUS, bits);
	irqt.mask_saved = advk_rd(ep, ADVK_HOST_INT_MASK);
	if (how != IRQT_POLL) {
		advk_wr(ep, ADVK_HOST_INT_MASK, irqt.mask_saved & ~bits);
	}
	irqt.how = how;
	k_sem_reset(&irqt.ev);
	k_spin_unlock(&irqt.lock, key);

#if defined(CONFIG_GIC)
	arm_gic_irq_clear_pending(EP_IRQ);
#endif
	if (how == IRQT_IRQ) {
		irqt.line_on = true;
		irq_enable(EP_IRQ);
	} else {
		k_timer_start(&irqt.timer, K_MSEC(ms), K_MSEC(ms));
	}

	shell_fprintf(sh, SHELL_NORMAL, "armed %s, bits", irqt_how_name[how]);
	bits_print(sh, bits);
	shell_fprintf(sh, SHELL_NORMAL, ", mask %08x -> %08x; latched before:",
		      irqt.mask_saved, advk_rd(ep, ADVK_HOST_INT_MASK));
	if ((latched & bits) == 0) {
		shell_fprintf(sh, SHELL_NORMAL, " -");
	}
	bits_print(sh, latched & bits);
	shell_fprintf(sh, SHELL_NORMAL, "\n");
	return 0;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	irqt_init_once();
	if (irqt_usable(sh) != 0) {
		return -ENODEV;
	}
	irqt_stop();
	shell_print(sh, "off, %u events, mask %08x", irqt.events,
		    advk_rd(advk_ep(EP_DEV), ADVK_HOST_INT_MASK));
	return 0;
}

static void log_print(const struct shell *sh, uint32_t from)
{
	struct irqt_event e;
	k_spinlock_key_t key;
	uint32_t to = irqt.events;

	if (to - from > IRQT_LOG) {
		shell_print(sh, "(%u older events dropped)", to - from - IRQT_LOG);
		from = to - IRQT_LOG;
	}
	for (uint32_t i = from; i != to; i++) {
		key = k_spin_lock(&irqt.lock);
		e = irqt.log[i % IRQT_LOG];
		k_spin_unlock(&irqt.lock, key);
		event_print(sh, &e);
	}
}

static int cmd_log(const struct shell *sh, size_t argc, char **argv)
{
	irqt_init_once();
	if (argc > 1) {
		k_spinlock_key_t key;

		if (strcmp(argv[1], "clear") != 0) {
			return -EINVAL;
		}
		key = k_spin_lock(&irqt.lock);
		memset(irqt.cnt, 0, sizeof(irqt.cnt));
		irqt.entries = irqt.empty = irqt.polls = irqt.gic_seen = irqt.events = 0;
		irqt.t0 = k_cycle_get_64();
		k_spin_unlock(&irqt.lock, key);
		return 0;
	}
	if (irqt.events == 0) {
		shell_print(sh, "no events");
		return 0;
	}
	log_print(sh, irqt.events > IRQT_LOG ? irqt.events - IRQT_LOG : 0);
	return 0;
}

/* wait [ms]: block until the next event (host side drives the test) */
static int cmd_wait(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long ms = 5000;
	uint32_t from;

	irqt_init_once();
	if (argc > 1) {
		char *end;

		ms = strtoul(argv[1], &end, 0);
		if (*end != '\0' || ms > 600000) {
			return -EINVAL;
		}
	}
	if (irqt.how == IRQT_OFF) {
		shell_error(sh, "not armed");
		return -EINVAL;
	}
	from = irqt.events;
	k_sem_reset(&irqt.ev);
	if (k_sem_take(&irqt.ev, K_MSEC(ms)) != 0 && irqt.events == from) {
		shell_print(sh, "nothing in %lu ms", ms);
		return -EAGAIN;
	}
	/* let a burst land */
	k_msleep(MAX(irqt.period_ms, 2U));
	log_print(sh, from);
	return 0;
}

/* regs <off> [n] | regs off */
static int cmd_regs(const struct shell *sh, size_t argc, char **argv)
{
	struct advk_ep *ep = advk_ep(EP_DEV);
	unsigned long off, n = 4;
	char *end;
	k_spinlock_key_t key;

	irqt_init_once();
	if (argc < 2) {
		if (irqt.reg_n == 0) {
			shell_print(sh, "none");
		} else {
			shell_print(sh, "%05x + %u words", irqt.reg_off, irqt.reg_n);
		}
		return 0;
	}
	if (strcmp(argv[1], "off") == 0) {
		key = k_spin_lock(&irqt.lock);
		irqt.reg_n = 0;
		k_spin_unlock(&irqt.lock, key);
		return 0;
	}
	if (irqt_usable(sh) != 0) {
		return -ENODEV;
	}
	off = strtoul(argv[1], &end, 0);
	if (*end != '\0' || off >= ADVK_REG_SPACE) {
		return -EINVAL;
	}
	if (argc > 2) {
		n = strtoul(argv[2], &end, 0);
		if (*end != '\0' || n == 0 || n > IRQT_REGS) {
			shell_error(sh, "1..%d words", IRQT_REGS);
			return -EINVAL;
		}
	}
	off &= ~3UL;
	if (off + 4 * n > ADVK_REG_SPACE) {
		return -EINVAL;
	}
	/* the ISR reads them plainly: no word may abort */
	for (unsigned long i = 0; i < n; i++) {
		uint32_t v;

		if (a3700_probe_read32(ep->reg + off + 4 * i, &v) < 0) {
			shell_error(sh, "%05lx aborts", off + 4 * i);
			return -EIO;
		}
	}
	key = k_spin_lock(&irqt.lock);
	irqt.reg_off = off;
	irqt.reg_n = n;
	k_spin_unlock(&irqt.lock, key);
	return 0;
}

/* kick: make SPI 29 pending in the GIC, the ISR path without the host */
static int cmd_kick(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	irqt_init_once();
#if defined(CONFIG_GIC)
	if (irqt.how != IRQT_IRQ || !irqt.line_on) {
		shell_error(sh, "arm irq first");
		return -EINVAL;
	}
	{
		uint32_t from = irqt.entries;

		irqt.kick = true;
		arm_gic_irq_set_pending(EP_IRQ);
		k_msleep(10);
		shell_print(sh, "isr entries %u -> %u", from, irqt.entries);
		return irqt.entries != from ? 0 : -EIO;
	}
#else
	shell_error(sh, "no GIC");
	return -ENOTSUP;
#endif
}

SHELL_STATIC_SUBCMD_SET_CREATE(irqtest_cmds,
	SHELL_CMD_ARG(show, NULL, "State, registers, counters per bit", cmd_show, 1, 0),
	SHELL_CMD_ARG(arm, NULL,
		      "[irq|open|poll] [bits] [ms]  Start: bits as names (dbell,wrddr = "
		      "default; all) or a mask; ms: sampling period of open/poll",
		      cmd_arm, 1, 3),
	SHELL_CMD_ARG(off, NULL, "Stop, mask restored", cmd_off, 1, 0),
	SHELL_CMD_ARG(log, NULL, "[clear] Last events", cmd_log, 1, 1),
	SHELL_CMD_ARG(wait, NULL, "[ms] Block until the next event, print it (default 5000)",
		      cmd_wait, 1, 1),
	SHELL_CMD_ARG(regs, NULL, "[<off> [n] | off] Register words captured with every event",
		      cmd_regs, 1, 2),
	SHELL_CMD_ARG(kick, NULL, "Set SPI 29 pending in the GIC (tests the ISR path)", cmd_kick,
		      1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((pcie_debug), irqtest, &irqtest_cmds,
		 "Central interrupt 0x1b000 (RC_DBELL_DET, DIR_WR_DDR_DET, ...): which host "
		 "action sets which bit, does it reach the GIC",
		 cmd_show, 1, 0);
