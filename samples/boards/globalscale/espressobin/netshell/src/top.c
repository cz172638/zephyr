/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * "top": CPU use per thread and per CPU over an interval, from the
 * scheduler's runtime statistics (THREAD_RUNTIME_STATS).
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define TOP_MAX_THREADS 48

struct top_entry {
	k_tid_t tid;
	char name[CONFIG_THREAD_MAX_NAME_LEN];
	uint64_t cycles;
	uint64_t delta;
	int prio;
};

struct top_snap {
	struct top_entry t[TOP_MAX_THREADS];
	int n;
};

static struct top_snap snap[2];

static void collect_cb(const struct k_thread *thread, void *user_data)
{
	struct top_snap *s = user_data;
	k_thread_runtime_stats_t st;
	struct top_entry *e;
	const char *name;

	if (s->n >= TOP_MAX_THREADS) {
		return;
	}
	e = &s->t[s->n++];
	e->tid = (k_tid_t)thread;
	e->prio = k_thread_priority_get(e->tid);
	name = k_thread_name_get(e->tid);
	strncpy(e->name, name != NULL ? name : "?", sizeof(e->name) - 1);
	e->name[sizeof(e->name) - 1] = '\0';
	e->cycles = (k_thread_runtime_stats_get(e->tid, &st) == 0) ? st.execution_cycles : 0;
}

static void collect(struct top_snap *s)
{
	s->n = 0;
	k_thread_foreach_unlocked(collect_cb, s);
}

static int cmp_delta(const void *a, const void *b)
{
	const struct top_entry *x = a, *y = b;

	return (x->delta < y->delta) - (x->delta > y->delta);
}

/* xx.x % */
static void pct(char *buf, size_t len, uint64_t part, uint64_t whole)
{
	uint32_t p = whole != 0 ? (uint32_t)((part * 1000U) / whole) : 0;

	snprintk(buf, len, "%3u.%u", p / 10, p % 10);
}

static int cmd_top(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t interval = argc > 1 ? strtoul(argv[1], NULL, 0) : 1000;
	uint32_t count = argc > 2 ? strtoul(argv[2], NULL, 0) : 1;
	k_thread_runtime_stats_t c0[CONFIG_MP_MAX_NUM_CPUS], c1;
	char b[16];

	interval = CLAMP(interval, 100, 60000);

	for (uint32_t it = 0; it < MAX(count, 1); it++) {
		uint64_t all = 0;

		for (int cpu = 0; cpu < arch_num_cpus(); cpu++) {
			(void)k_thread_runtime_stats_cpu_get(cpu, &c0[cpu]);
		}
		collect(&snap[0]);
		k_msleep(interval);
		collect(&snap[1]);

		shell_print(sh, "--- %u ms ---", interval);
		for (int cpu = 0; cpu < arch_num_cpus(); cpu++) {
			uint64_t tot, idle;

			(void)k_thread_runtime_stats_cpu_get(cpu, &c1);
			tot = c1.execution_cycles - c0[cpu].execution_cycles;
			idle = c1.idle_cycles - c0[cpu].idle_cycles;
			all += tot;
			pct(b, sizeof(b), tot - MIN(idle, tot), tot);
			shell_print(sh, "CPU%d busy %s %%", cpu, b);
		}

		for (int i = 0; i < snap[1].n; i++) {
			struct top_entry *e = &snap[1].t[i];

			e->delta = e->cycles;
			for (int j = 0; j < snap[0].n; j++) {
				if (snap[0].t[j].tid == e->tid) {
					e->delta = e->cycles - MIN(snap[0].t[j].cycles, e->cycles);
					break;
				}
			}
		}
		qsort(snap[1].t, snap[1].n, sizeof(snap[1].t[0]), cmp_delta);

		shell_print(sh, "  %%CPU  prio  thread");
		for (int i = 0; i < snap[1].n; i++) {
			struct top_entry *e = &snap[1].t[i];

			pct(b, sizeof(b), e->delta, all);
			shell_print(sh, "%s %5d  %s", b, e->prio, e->name);
		}
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(top, NULL, "[interval ms] [count] CPU use per thread", cmd_top, 1, 2);
