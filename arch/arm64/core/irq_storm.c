/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Interrupt storm guard: a PPI or SPI that raises more than
 * CONFIG_ARM64_IRQ_STORM_GUARD_COUNT interrupts within
 * CONFIG_ARM64_IRQ_STORM_GUARD_MS is disabled. Called from _isr_wrapper
 * with the INTID before the handler runs.
 */

#include <stdlib.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(irq_storm, CONFIG_KERNEL_LOG_LEVEL);

/* SGIs (0-15) are software generated; LPIs (8192+) are not tracked */
#define STORM_FIRST 16U
#define STORM_IRQS  MIN(CONFIG_NUM_IRQS, 1020)

struct storm_line {
	uint32_t start; /* uptime in ms when the window began */
	uint32_t count; /* interrupts in the window */
};

static struct storm_line lines[STORM_IRQS];
static ATOMIC_DEFINE(disabled, STORM_IRQS);
static uint32_t storms;

void z_arm64_irq_storm_check(unsigned int irq);

void z_arm64_irq_storm_check(unsigned int irq)
{
	struct storm_line *l;
	uint32_t now;

	if (irq < STORM_FIRST || irq >= STORM_IRQS) {
		return;
	}
	l = &lines[irq];
	if (++l->count < CONFIG_ARM64_IRQ_STORM_GUARD_COUNT) {
		return;
	}
	/* the clock is read once per COUNT interrupts only */
	now = k_uptime_get_32();
	if (now - l->start < CONFIG_ARM64_IRQ_STORM_GUARD_MS) {
		irq_disable(irq);
		atomic_set_bit(disabled, irq);
		storms++;
		LOG_ERR("IRQ %u: %u interrupts in %u ms, line disabled", irq, l->count,
			now - l->start);
	}
	l->count = 0U;
	l->start = now;
}

#if defined(CONFIG_SHELL)
static int cmd_storm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "limit %u interrupts in %u ms, %u storms",
		    CONFIG_ARM64_IRQ_STORM_GUARD_COUNT, CONFIG_ARM64_IRQ_STORM_GUARD_MS, storms);
	for (unsigned int irq = STORM_FIRST; irq < STORM_IRQS; irq++) {
		if (atomic_test_bit(disabled, irq)) {
			shell_print(sh, "IRQ %u disabled", irq);
		}
	}
	return 0;
}

static int cmd_storm_enable(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long irq = strtoul(argv[1], NULL, 0);

	ARG_UNUSED(argc);

	if (irq < STORM_FIRST || irq >= STORM_IRQS || !atomic_test_bit(disabled, irq)) {
		shell_error(sh, "IRQ %lu was not disabled by the guard", irq);
		return -EINVAL;
	}
	lines[irq].count = 0U;
	lines[irq].start = k_uptime_get_32();
	atomic_clear_bit(disabled, irq);
	irq_enable(irq);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_storm,
	SHELL_CMD_ARG(enable, NULL, "<irq>: enable a line the guard disabled", cmd_storm_enable,
		      2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_irq,
	SHELL_CMD(storm, &sub_storm, "Lines disabled by the interrupt storm guard", cmd_storm),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(irq, &sub_irq, "Interrupts", NULL);
#endif /* CONFIG_SHELL */
