/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * "cpu": system registers, memory attributes of a few addresses and a
 * core clock / memory bandwidth estimate, to tell a slow clock from
 * memory that is not cached. With the PMU each benchmark also shows
 * core cycles, instructions, cache refills and exceptions taken.
 *
 * The PMU, the cache and the GIC CPU interface are per CPU: with SMP every
 * measurement runs in a thread pinned to one CPU, so the shell thread
 * moving between CPUs does not mix two of them.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/arch/pmu.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#define SYSREG(name)                                                                               \
	({                                                                                         \
		uint64_t v_;                                                                       \
		__asm__ volatile("mrs %0, " #name : "=r"(v_));                                     \
		v_;                                                                                \
	})

#define BENCH_BUF KB(256)

static uint8_t bench_buf[BENCH_BUF] __aligned(64);

static const struct {
	pmu_evt_t evt;
	const char *name;
} pmu_evts[] = {
	{PMU_EVT_INST_RETIRED, "insn"},     {PMU_EVT_L1I_CACHE_REFILL, "l1i-refill"},
	{PMU_EVT_L1D_CACHE_REFILL, "l1d-refill"}, {PMU_EVT_L2D_CACHE_REFILL, "l2-refill"},
	{PMU_EVT_EXC_TAKEN, "exc"},         {PMU_EVT_BUS_ACCESS, "bus"},
};

static bool pmu_ok;
static uint32_t pmu_n;

static void pmu_begin(void)
{
	if (!pmu_ok) {
		return;
	}
	pmu_stop();
	pmu_counter_reset_all();
	pmu_cycle_reset();
	pmu_start();
}

static void pmu_end(const struct shell *sh, uint64_t ns)
{
	uint64_t cyc, v[ARRAY_SIZE(pmu_evts)] = {0};

	if (!pmu_ok) {
		return;
	}
	pmu_stop();
	cyc = pmu_cycle_count();
	for (uint32_t i = 0; i < pmu_n; i++) {
		v[i] = pmu_counter_read(i);
	}
	/* insn per cycle, in hundredths */
	uint64_t ipc = (cyc != 0U && pmu_n > 0U) ? v[0] * 100U / cyc : 0U;

	shell_fprintf(sh, SHELL_NORMAL, "      cycles %llu (%llu MHz), IPC %llu.%02llu", cyc,
		      ns ? cyc * 1000U / ns : 0, ipc / 100U, ipc % 100U);
	for (uint32_t i = 0; i < pmu_n; i++) {
		shell_fprintf(sh, SHELL_NORMAL, ", %s %llu", pmu_evts[i].name, v[i]);
	}
	shell_fprintf(sh, SHELL_NORMAL, "\n");
}

static void pmu_setup(const struct shell *sh)
{
	if (pmu_init() != 0) {
		shell_print(sh, "PMU: not usable at this exception level");
		return;
	}
	pmu_n = MIN(pmu_num_counters(), (uint32_t)ARRAY_SIZE(pmu_evts));
	for (uint32_t i = 0; i < pmu_n; i++) {
		(void)pmu_counter_config(i, pmu_evts[i].evt);
	}
	pmu_counter_enable_all();
	pmu_ok = true;
	shell_print(sh, "PMU: %u counters, core clock %u MHz (cycle counter vs. CNTVCT)",
		    pmu_num_counters(), arch_pmu_cpu_freq_mhz());
}

/* stage 1 EL1 translation of va: PAR_EL1 (F, PA, ATTR) */
static uint64_t at_s1e1r(const void *va)
{
	uint64_t par;
	unsigned int key = irq_lock();

	__asm__ volatile("at s1e1r, %1\n\tisb\n\tmrs %0, par_el1" : "=r"(par) : "r"(va));
	irq_unlock(key);
	return par;
}

static void show_at(const struct shell *sh, const char *what, const void *va, uint64_t mair)
{
	uint64_t par = at_s1e1r(va);

	if ((par & 1U) != 0U) {
		shell_print(sh, "%-10s %p: fault, PAR %016llx", what, va, (unsigned long long)par);
		return;
	}
	uint8_t attr = (uint8_t)(par >> 56);
	uint32_t idx;

	for (idx = 0; idx < 8; idx++) {
		if ((uint8_t)(mair >> (8 * idx)) == attr) {
			break;
		}
	}
	shell_print(sh, "%-10s %p -> PA %llx, attr %02x (%s%s), sh %llu, ns %llu", what, va,
		    (unsigned long long)(par & GENMASK64(47, 12)), attr,
		    attr == 0xffU ? "normal WB RW-alloc" : (attr & 0xf0U) == 0U ? "device" :
		    attr == 0x44U ? "normal non-cacheable" : "other",
		    idx < 8 ? "" : ", not in MAIR", (unsigned long long)((par >> 7) & 3U),
		    (unsigned long long)((par >> 9) & 1U));
}

/* 8 dependent adds per iteration: >= 8 core cycles each */
static uint64_t bench_core(uint64_t iters)
{
	uint64_t acc = 0, n = iters, t0, t1;

	t0 = k_cycle_get_64();
	__asm__ volatile("1:\n\t"
			 "add %0, %0, #1\n\tadd %0, %0, #1\n\tadd %0, %0, #1\n\tadd %0, %0, #1\n\t"
			 "add %0, %0, #1\n\tadd %0, %0, #1\n\tadd %0, %0, #1\n\tadd %0, %0, #1\n\t"
			 "subs %1, %1, #1\n\tb.ne 1b"
			 : "+r"(acc), "+r"(n));
	t1 = k_cycle_get_64();
	return k_cyc_to_ns_floor64(t1 - t0);
}

static uint64_t bench_mem(bool write, unsigned int loops)
{
	volatile uint64_t *p = (volatile uint64_t *)bench_buf;
	uint64_t sum = 0, t0, t1;

	t0 = k_cycle_get_64();
	for (unsigned int l = 0; l < loops; l++) {
		for (size_t i = 0; i < BENCH_BUF / 8U; i++) {
			if (write) {
				p[i] = i;
			} else {
				sum += p[i];
			}
		}
	}
	t1 = k_cycle_get_64();
	(void)sum;
	return k_cyc_to_ns_floor64(t1 - t0);
}

static int cpu_info(const struct shell *sh, uint32_t arg)
{
	const uint64_t iters = 10000000ULL;
	uint64_t mair = SYSREG(mair_el1);
	uint64_t sctlr = SYSREG(sctlr_el1);
	uint64_t ns, ns_locked;
	unsigned int key;

	ARG_UNUSED(arg);

	shell_print(sh, "MIDR %08llx MPIDR %08llx CurrentEL %llu CNTFRQ %llu", SYSREG(midr_el1),
		    SYSREG(mpidr_el1), (SYSREG(CurrentEL) >> 2) & 3U, SYSREG(cntfrq_el0));
	shell_print(sh, "SCTLR_EL1 %08llx: M %llu C %llu I %llu", sctlr, sctlr & 1U,
		    (sctlr >> 2) & 1U, (sctlr >> 12) & 1U);
	shell_print(sh, "MAIR_EL1 %016llx TCR_EL1 %016llx", mair, SYSREG(tcr_el1));
	shell_print(sh, "CTR_EL0 %08llx CLIDR_EL1 %08llx", SYSREG(ctr_el0), SYSREG(clidr_el1));
	show_at(sh, "text", (const void *)cpu_info, mair);
	show_at(sh, "bss", bench_buf, mair);
	show_at(sh, "stack", &mair, mair);

	pmu_setup(sh);

	pmu_begin();
	ns = bench_core(iters);
	shell_print(sh, "core: %llu dependent adds in %llu us -> >= %llu MHz", 8ULL * iters,
		    ns / 1000U, ns ? 8000ULL * iters / ns : 0);
	pmu_end(sh, ns);
	key = irq_lock();
	pmu_begin();
	ns_locked = bench_core(iters);
	pmu_stop();
	irq_unlock(key);
	shell_print(sh, "core, irqs locked: %llu us -> >= %llu MHz", ns_locked / 1000U,
		    ns_locked ? 8000ULL * iters / ns_locked : 0);
	pmu_end(sh, ns_locked);

	memset(bench_buf, 0x5a, sizeof(bench_buf)); /* outside the counted period */
	pmu_begin();
	ns = bench_mem(false, 16);
	shell_print(sh, "read  %u KiB x16: %llu us, %llu MB/s", (unsigned int)(BENCH_BUF / 1024U),
		    ns / 1000U, ns ? 16ULL * BENCH_BUF * 1000U / ns : 0);
	pmu_end(sh, ns);
	memset(bench_buf, 0x5a, sizeof(bench_buf)); /* outside the counted period */
	pmu_begin();
	ns = bench_mem(true, 16);
	shell_print(sh, "write %u KiB x16: %llu us, %llu MB/s", (unsigned int)(BENCH_BUF / 1024U),
		    ns / 1000U, ns ? 16ULL * BENCH_BUF * 1000U / ns : 0);
	pmu_end(sh, ns);
	return 0;
}

/*
 * Which interrupt keeps the core busy: with IRQs masked at the core,
 * ICC_HPPIR1_EL1 names the highest priority pending Group 1 interrupt;
 * the PMU counts exceptions taken over a period with IRQs on.
 */
#define HPPIR_SAMPLES 64U
#define INTID_SPURIOUS 1023U

static int cpu_irq(const struct shell *sh, uint32_t ms)
{
	uint32_t ids[HPPIR_SAMPLES], cnt[HPPIR_SAMPLES] = {0}, nids = 0;

	for (uint32_t i = 0; i < HPPIR_SAMPLES; i++) {
		unsigned int key = irq_lock();
		uint64_t t0 = k_cycle_get_64();
		uint32_t id;

		/* 50..1650 us: long enough for a periodic source to become pending */
		while (k_cycle_get_64() - t0 < k_us_to_cyc_ceil64(50U + 25U * i)) {
		}
		id = (uint32_t)SYSREG(S3_0_C12_C12_2) & 0xffffffU; /* ICC_HPPIR1_EL1 */
		irq_unlock(key);

		uint32_t j;

		for (j = 0; j < nids && ids[j] != id; j++) {
		}
		if (j == nids) {
			ids[nids++] = id;
		}
		cnt[j]++;
	}
	shell_print(sh, "pending with IRQs masked, %u samples:", HPPIR_SAMPLES);
	for (uint32_t j = 0; j < nids; j++) {
		shell_print(sh, "  INTID %4u%s: %u", ids[j],
			    ids[j] == INTID_SPURIOUS ? " (none)"
			    : ids[j] >= 32U ? "" : ids[j] >= 16U ? " (PPI)" : " (SGI)", cnt[j]);
	}

	pmu_setup(sh);
	if (pmu_ok) {
		uint64_t exc;

		pmu_begin();
		k_busy_wait(ms * 1000U);
		pmu_stop();
		exc = pmu_counter_read(4);
		shell_print(sh, "exceptions taken in %u ms: %llu (%llu/s)", ms, exc,
			    exc * 1000U / MAX(ms, 1U));
	}
	return 0;
}

typedef int (*cpu_fn_t)(const struct shell *sh, uint32_t arg);

#if defined(CONFIG_SCHED_CPU_MASK) && (CONFIG_MP_MAX_NUM_CPUS > 1)
static K_THREAD_STACK_DEFINE(cpu_stack, 4096);
static struct k_thread cpu_thread;

struct cpu_job {
	cpu_fn_t fn;
	const struct shell *sh;
	uint32_t arg;
	int ret;
};

static void cpu_job_run(void *p1, void *p2, void *p3)
{
	struct cpu_job *job = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	job->ret = job->fn(job->sh, job->arg);
}

/* the shell thread is not pinned: run fn in a thread that stays on cpu */
static int run_on_cpu(int cpu, cpu_fn_t fn, const struct shell *sh, uint32_t arg)
{
	struct cpu_job job = {.fn = fn, .sh = sh, .arg = arg, .ret = -EIO};

	k_thread_create(&cpu_thread, cpu_stack, K_THREAD_STACK_SIZEOF(cpu_stack), cpu_job_run,
			&job, NULL, NULL, k_thread_priority_get(k_current_get()), 0, K_FOREVER);
	if (k_thread_cpu_pin(&cpu_thread, cpu) != 0) {
		k_thread_abort(&cpu_thread);
		return -EINVAL;
	}
	k_thread_start(&cpu_thread);
	(void)k_thread_join(&cpu_thread, K_FOREVER);
	return job.ret;
}
#else
static int run_on_cpu(int cpu, cpu_fn_t fn, const struct shell *sh, uint32_t arg)
{
	ARG_UNUSED(cpu);
	return fn(sh, arg);
}
#endif

static int parse_cpu(const struct shell *sh, const char *s, int *cpu)
{
	char *end;
	unsigned long v = strtoul(s, &end, 0);

	if (*end != '\0' || v >= arch_num_cpus()) {
		shell_error(sh, "CPU %s: 0..%u", s, arch_num_cpus() - 1U);
		return -EINVAL;
	}
	*cpu = (int)v;
	return 0;
}

static int cmd_cpu(const struct shell *sh, size_t argc, char **argv)
{
	unsigned int key = irq_lock();
	int cpu = (int)arch_curr_cpu()->id;

	irq_unlock(key);
	if (argc > 1 && parse_cpu(sh, argv[1], &cpu) != 0) {
		return -EINVAL;
	}
	if (arch_num_cpus() > 1U) {
		shell_print(sh, "CPU%d:", cpu);
	}
	return run_on_cpu(cpu, cpu_info, sh, 0U);
}

static int cmd_cpu_irq(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t ms = argc > 1 ? strtoul(argv[1], NULL, 0) : 100U;
	int first = 0, last = (int)arch_num_cpus() - 1;

	if (argc > 2) {
		if (parse_cpu(sh, argv[2], &first) != 0) {
			return -EINVAL;
		}
		last = first;
	}
	for (int cpu = first; cpu <= last; cpu++) {
		if (arch_num_cpus() > 1U) {
			shell_print(sh, "CPU%d:", cpu);
		}
		(void)run_on_cpu(cpu, cpu_irq, sh, ms);
	}
	shell_print(sh, "SPI n = INTID - 32 (devicetree GIC_SPI n)");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cpu,
	SHELL_CMD_ARG(irq, NULL,
		      "[ms] [cpu]: pending interrupt with IRQs masked, exception rate (all CPUs)",
		      cmd_cpu_irq, 1, 2),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(cpu, &sub_cpu,
		       "[cpu]: CPU registers, memory attributes, clock and memory speed", cmd_cpu,
		       1, 1);
