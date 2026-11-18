/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/interrupt_controller/gicv3_its.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

/*
 * its_map_intid() routes an LPI to the CPU that maps it; its_move_intid()
 * must deliver it to every other CPU in turn. its_send_int() raises the
 * LPI from the ITS like a device write to GITS_TRANSLATER would.
 */

#define DEVICE_ID 0x42U
#define EVENT_ID  0U

#define ITS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(arm_gic_v3_its)

static const struct device *const its = DEVICE_DT_GET(ITS_NODE);
static volatile int handler_cpu;
static struct k_sem lpi_sem;

static void lpi_isr(const void *arg)
{
	ARG_UNUSED(arg);

	handler_cpu = arch_curr_cpu()->id;
	k_sem_give(&lpi_sem);
}

ZTEST(arm64_gicv3_its_movi, test_move)
{
	unsigned int intid;

	zassert_true(device_is_ready(its), "ITS not ready");
	k_sem_init(&lpi_sem, 0, 1);
	zassert_ok(its_setup_deviceid(its, DEVICE_ID, 1));
	intid = its_alloc_intid(its);
	zassert_ok(its_map_intid(its, DEVICE_ID, EVENT_ID, intid));
	zassert_true(irq_connect_dynamic(intid, 0, lpi_isr, NULL, 0) >= 0);
	irq_enable(intid);

	for (unsigned int cpu = 0; cpu < arch_num_cpus(); cpu++) {
		handler_cpu = -1;
		zassert_ok(its_move_intid(its, DEVICE_ID, EVENT_ID, cpu));
		zassert_ok(its_send_int(its, DEVICE_ID, EVENT_ID));
		zassert_ok(k_sem_take(&lpi_sem, K_MSEC(100)), "no LPI after moving to CPU%u",
			   cpu);
		zassert_equal(handler_cpu, (int)cpu, "LPI on CPU%d, moved to CPU%u",
			      handler_cpu, cpu);
	}
	zassert_equal(its_move_intid(its, DEVICE_ID, EVENT_ID, arch_num_cpus()), -EINVAL);
}

ZTEST_SUITE(arm64_gicv3_its_movi, NULL, NULL, NULL, NULL, NULL);
