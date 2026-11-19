/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * With SMP the endpoint gets a CPU of its own: its interrupts are routed
 * to the CPU the test function's command work queue is pinned to.
 * APPLICATION level: the secondary CPUs run by then.
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#if defined(CONFIG_SMP) && defined(CONFIG_PCIE_EPF_TEST_CPU) && CONFIG_PCIE_EPF_TEST_CPU >= 0 &&  \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(pcie0_ep))
#include <zephyr/drivers/interrupt_controller/gic.h>

#define EP_NODE DT_NODELABEL(pcie0_ep)

static K_THREAD_STACK_DEFINE(mpidr_stack, 1024);
static struct k_thread mpidr_thread;
static volatile uint64_t ep_mpidr;

static void mpidr_read(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ep_mpidr = MPIDR_TO_CORE(GET_MPIDR());
}

/*
 * The endpoint's interrupts (link events, uDMA done) go to the CPU of the
 * test function's work queue, so a transfer completes where it was started
 * and nothing on the other CPUs delays it. The MPIDR is read on that CPU,
 * so this runs at the SMP level, after the secondary CPUs are started.
 */
static int ep_irq_affinity(void)
{
	k_thread_create(&mpidr_thread, mpidr_stack, K_THREAD_STACK_SIZEOF(mpidr_stack), mpidr_read,
			NULL, NULL, NULL, K_PRIO_COOP(0), 0, K_FOREVER);
	k_thread_cpu_pin(&mpidr_thread, CONFIG_PCIE_EPF_TEST_CPU);
	k_thread_start(&mpidr_thread);
	if (k_thread_join(&mpidr_thread, K_SECONDS(1)) != 0) {
		printk("CPU%d did not run, PCIe endpoint interrupts stay on CPU0\n",
		       CONFIG_PCIE_EPF_TEST_CPU);
		k_thread_abort(&mpidr_thread);
		return 0;
	}

	arm_gic_irq_set_affinity(DT_IRQ_BY_NAME(EP_NODE, sys, irq), ep_mpidr);
	arm_gic_irq_set_affinity(DT_IRQ_BY_NAME(EP_NODE, legacy, irq), ep_mpidr);
	arm_gic_irq_set_affinity(DT_IRQ_BY_NAME(EP_NODE, client, irq), ep_mpidr);
	printk("PCIe endpoint interrupts on CPU%d (MPIDR 0x%llx)\n", CONFIG_PCIE_EPF_TEST_CPU,
	       (unsigned long long)ep_mpidr);
	return 0;
}

SYS_INIT(ep_irq_affinity, SMP, 0);
#endif

