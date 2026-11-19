/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * With SMP the endpoint gets a CPU of its own: its interrupts are routed
 * to the CPU the test function's command work queue is pinned to, and the
 * threads that exist at boot (shell, network, logging, main) may no longer
 * run there. Threads created later are not restricted.
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
static unsigned int ep_moved, ep_busy;

/* everything but the idle threads and the threads pinned to the EP CPU */
static void ep_cpu_release(const struct k_thread *t, void *user_data)
{
	struct k_thread *th = (struct k_thread *)t;
	uint32_t cpu = BIT(CONFIG_PCIE_EPF_TEST_CPU);
	bool *retry = user_data;

	if (th == k_current_get() || th->base.prio == K_IDLE_PRIO ||
	    (th->base.cpu_mask & cpu) == 0U || th->base.cpu_mask == cpu) {
		return;
	}
	/* only a thread that cannot run right now may change its mask */
	if (k_thread_cpu_mask_disable(th, CONFIG_PCIE_EPF_TEST_CPU) == 0) {
		ep_moved++;
	} else {
		*retry = true;
	}
}

/*
 * Runs on the EP CPU while the init thread (later main) waits for it in
 * k_thread_join(), so that one cannot run either and gets restricted too.
 */
static void mpidr_read(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	ep_mpidr = MPIDR_TO_CORE(GET_MPIDR());

	for (int pass = 0; pass < 10; pass++) {
		bool retry = false;

		k_thread_foreach_unlocked(ep_cpu_release, &retry);
		if (!retry) {
			ep_busy = 0U;
			return;
		}
		ep_busy = 1U;
		k_msleep(1);
	}
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
	printk("PCIe endpoint interrupts on CPU%d (MPIDR 0x%llx), %u threads moved off it%s\n",
	       CONFIG_PCIE_EPF_TEST_CPU, (unsigned long long)ep_mpidr, ep_moved,
	       ep_busy != 0U ? ", some still allowed (they kept running)" : "");
	return 0;
}

SYS_INIT(ep_irq_affinity, SMP, 0);
#endif

