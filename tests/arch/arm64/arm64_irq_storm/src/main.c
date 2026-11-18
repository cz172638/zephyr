/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/ztest.h>

/*
 * An SPI whose handler makes it pending again behaves like a level
 * interrupt that is never cleared: the guard must disable it. A line
 * pended a few times per millisecond must stay enabled.
 */

static volatile uint32_t hits;
static volatile bool repend;
static int test_irq = -1;

static void storm_isr(const void *arg)
{
	ARG_UNUSED(arg);

	hits++;
	if (repend) {
		arm_gic_irq_set_pending(test_irq);
	}
}

static int find_unused_spi(void)
{
	uint32_t itlines = sys_read32(GICD_TYPER) & GICD_TYPER_ITLINESNUM_MASK;
	int max_spi = MIN((int)(((itlines + 1U) << 5) - 1U), GIC_SPI_MAX_INTID);

	max_spi = MIN(max_spi, CONFIG_NUM_IRQS - 1);
	for (int irq = max_spi; irq >= GIC_SPI_INT_BASE; irq--) {
		if (!arm_gic_irq_is_enabled(irq)) {
			return irq;
		}
	}
	return -1;
}

static void *setup(void)
{
	test_irq = find_unused_spi();
	zassert_true(test_irq >= 0, "no free SPI");
	irq_connect_dynamic(test_irq, IRQ_DEFAULT_PRIORITY, storm_isr, NULL, 0);
	return NULL;
}

ZTEST(arm64_irq_storm, test_slow_line_stays_enabled)
{
	hits = 0;
	repend = false;
	irq_enable(test_irq);
	/* 300 interrupts over ~300 ms: far below 1000 per 100 ms */
	for (int i = 0; i < 300; i++) {
		arm_gic_irq_set_pending(test_irq);
		k_busy_wait(1000);
	}
	zassert_true(hits >= 290, "only %u interrupts taken", hits);
	zassert_true(arm_gic_irq_is_enabled(test_irq), "a slow line was disabled");
	irq_disable(test_irq);
}

ZTEST(arm64_irq_storm, test_storm_disabled)
{
	int64_t t0;

	hits = 0;
	repend = true;
	irq_enable(test_irq);
	arm_gic_irq_set_pending(test_irq);
	/* the storm keeps this thread from running until the guard steps in */
	t0 = k_uptime_get();
	while (arm_gic_irq_is_enabled(test_irq) && k_uptime_get() - t0 < 1000) {
	}
	repend = false;
	zassert_false(arm_gic_irq_is_enabled(test_irq), "storm not stopped, %u interrupts", hits);
	zassert_true(hits >= CONFIG_ARM64_IRQ_STORM_GUARD_COUNT, "%u interrupts", hits);
	TC_PRINT("storm stopped after %u interrupts in %lld ms\n", hits, k_uptime_get() - t0);
}

ZTEST_SUITE(arm64_irq_storm, NULL, setup, NULL, NULL, NULL);
