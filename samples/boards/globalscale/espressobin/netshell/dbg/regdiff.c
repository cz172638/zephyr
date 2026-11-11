/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * dbg regdiff: snapshot a register range, diff it later, or watch it
 * while something happens (here or on the lab peer). For the Kirkwood
 * hints (functional spec A.6) on the Aardvark: where the EP keeps the
 * bus/device number the host gave it (Kirkwood 0x41A04, updated by every
 * CfgWr), the slave indications (hot reset, loopback), which registers a
 * hot reset wipes.
 *
 * Every read goes through a3700_probe_read32, so holes and gated blocks
 * show up as "abort" instead of taking the board down. Words that change
 * on their own (counters, status) are found at snap time ("settle") and
 * left out of the diff.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/lab/lab.h>
#include <zephyr/lab/mod.h>
#include <zephyr/shell/shell.h>

#include <soc.h>

#define SNAPS    4
#define MAX_LEN  0x10000
#define ABORTED  0xabadabadU

struct snap {
	char name[12];
	uintptr_t phys;
	size_t len;
	uint8_t *va;
	uint32_t *val;
	uint32_t *vol; /* bits that moved by themselves */
	uint8_t *bad;  /* aborted words */
};

static struct snap snaps[SNAPS];

static int num(const struct shell *sh, const char *s, unsigned long long *v)
{
	char *end;

	*v = strtoull(s, &end, 0);
	if (*end != '\0') {
		shell_error(sh, "bad number '%s'", s);
		return -EINVAL;
	}
	return 0;
}

static struct snap *find(const char *name)
{
	for (int i = 0; i < SNAPS; i++) {
		if (snaps[i].val != NULL && strcmp(snaps[i].name, name) == 0) {
			return &snaps[i];
		}
	}
	return NULL;
}

static void drop(struct snap *s)
{
	if (s->va != NULL) {
		k_mem_unmap_phys_bare(s->va, ROUND_UP(s->len + (s->phys & 0xfff), 0x1000));
	}
	k_free(s->val);
	k_free(s->vol);
	k_free(s->bad);
	memset(s, 0, sizeof(*s));
}

static int rd(struct snap *s, size_t i, uint32_t *v)
{
	return a3700_probe_read32((uintptr_t)s->va + (s->phys & 0xfff) + 4 * i, v);
}

/* snap <name> <phys> <len> [settle ms]: take (or retake) a snapshot */
static int cmd_snap(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long long phys, len, settle = 100;
	struct snap *s = find(argv[1]);
	size_t n, bad = 0, vol = 0;
	int64_t end;

	if (num(sh, argv[2], &phys) != 0 || num(sh, argv[3], &len) != 0 ||
	    (argc > 4 && num(sh, argv[4], &settle) != 0) || (phys & 3) || len == 0 ||
	    len > MAX_LEN || (len & 3) || strlen(argv[1]) >= sizeof(s->name)) {
		shell_error(sh, "phys 4-aligned, len 4..0x%x, name < 12 chars", MAX_LEN);
		return -EINVAL;
	}
	if (s != NULL) {
		drop(s);
	}
	for (int i = 0; i < SNAPS && s == NULL; i++) {
		if (snaps[i].val == NULL) {
			s = &snaps[i];
		}
	}
	if (s == NULL) {
		shell_error(sh, "%d snapshots in use (drop one)", SNAPS);
		return -ENOMEM;
	}
	n = len / 4;
	strcpy(s->name, argv[1]);
	s->phys = phys;
	s->len = len;
	s->val = k_malloc(n * 4);
	s->vol = k_calloc(n, 4);
	s->bad = k_calloc(n, 1);
	k_mem_map_phys_bare(&s->va, phys & ~0xfffULL, ROUND_UP(len + (phys & 0xfff), 0x1000),
			    K_MEM_PERM_RW | K_MEM_CACHE_NONE);
	if (s->val == NULL || s->vol == NULL || s->bad == NULL || s->va == NULL) {
		drop(s);
		return -ENOMEM;
	}
	for (size_t i = 0; i < n; i++) {
		if (rd(s, i, &s->val[i]) != 0) {
			s->bad[i] = 1;
			bad++;
		}
	}
	/* re-read for settle ms: what moves now is not our event */
	end = k_uptime_get() + settle;
	do {
		for (size_t i = 0; i < n; i++) {
			uint32_t v;

			if (!s->bad[i] && rd(s, i, &v) == 0) {
				s->vol[i] |= v ^ s->val[i];
			}
		}
	} while (k_uptime_get() < end);
	for (size_t i = 0; i < n; i++) {
		vol += s->vol[i] != 0;
	}
	shell_print(sh, "%s: 0x%llx + 0x%llx, %zu words, %zu abort, %zu volatile", s->name, phys,
		    len, n, bad, vol);
	return 0;
}

static int show_diff(const struct shell *sh, struct snap *s, bool update, bool all)
{
	int n = 0;

	for (size_t i = 0; i < s->len / 4; i++) {
		uint32_t v, ch;

		if (s->bad[i]) {
			continue;
		}
		if (rd(s, i, &v) != 0) {
			shell_print(sh, "%08lx: %08x -> abort", s->phys + 4 * i, s->val[i]);
			n++;
			continue;
		}
		ch = (v ^ s->val[i]) & (all ? ~0U : ~s->vol[i]);
		if (ch != 0) {
			shell_print(sh, "%08lx: %08x -> %08x  changed %08x%s", s->phys + 4 * i,
				    s->val[i], v, ch, s->vol[i] ? "  (+volatile bits)" : "");
			n++;
		}
		if (update) {
			s->val[i] = v;
		}
	}
	shell_print(sh, "%s: %d words changed%s", s->name, n, all ? " (volatile included)" : "");
	return n;
}

/* diff <name> [-u] [-a]: -u take the new values as the snapshot, -a include volatile */
static int cmd_diff(const struct shell *sh, size_t argc, char **argv)
{
	struct snap *s = find(argv[1]);
	bool update = false, all = false;

	if (s == NULL) {
		shell_error(sh, "%s: no such snapshot", argv[1]);
		return -ENOENT;
	}
	for (size_t i = 2; i < argc; i++) {
		update |= strcmp(argv[i], "-u") == 0;
		all |= strcmp(argv[i], "-a") == 0;
	}
	show_diff(sh, s, update, all);
	return 0;
}

/* event <name> <peer command...>: diff -u, run the command on the peer, diff */
static int cmd_event(const struct shell *sh, size_t argc, char **argv)
{
	struct snap *s = find(argv[1]);
	char cmd[160] = "", out[512];
	size_t o = 0;
	int ret;

	if (s == NULL) {
		shell_error(sh, "%s: no such snapshot", argv[1]);
		return -ENOENT;
	}
	for (size_t i = 2; i < argc; i++) {
		size_t l = strlen(argv[i]);

		if (o + l + 2 > sizeof(cmd)) {
			return -E2BIG;
		}
		if (o) {
			cmd[o++] = ' ';
		}
		memcpy(&cmd[o], argv[i], l + 1);
		o += l;
	}
	/* fresh baseline right before the event */
	for (size_t i = 0; i < s->len / 4; i++) {
		if (!s->bad[i]) {
			(void)rd(s, i, &s->val[i]);
		}
	}
	ret = lab_peer_exec(cmd, out, sizeof(out), K_SECONDS(10));
	shell_print(sh, "peer '%s': rc %d", cmd, ret);
	if (out[0] != '\0') {
		shell_fprintf(sh, SHELL_NORMAL, "%s", out);
	}
	k_msleep(10);
	show_diff(sh, s, false, false);
	return 0;
}

/* watch <name> <ms>: what changes, how often, during ms (do the event meanwhile) */
static int cmd_watch(const struct shell *sh, size_t argc, char **argv)
{
	struct snap *s = find(argv[1]);
	unsigned long long ms;
	uint32_t *moved, *last;
	uint16_t *count;
	size_t n;
	int64_t end;
	int shown = 0;

	if (s == NULL || num(sh, argv[2], &ms) != 0) {
		shell_error(sh, "watch <snapshot> <ms>");
		return -EINVAL;
	}
	n = s->len / 4;
	moved = k_calloc(n, 4);
	last = k_malloc(n * 4);
	count = k_calloc(n, 2);
	if (moved == NULL || last == NULL || count == NULL) {
		k_free(moved);
		k_free(last);
		k_free(count);
		return -ENOMEM;
	}
	for (size_t i = 0; i < n; i++) {
		last[i] = s->val[i];
		if (!s->bad[i]) {
			(void)rd(s, i, &last[i]);
			s->val[i] = last[i];
		}
	}
	end = k_uptime_get() + ms;
	while (k_uptime_get() < end) {
		for (size_t i = 0; i < n; i++) {
			uint32_t v;

			if (s->bad[i] || rd(s, i, &v) != 0 || v == last[i]) {
				continue;
			}
			moved[i] |= (v ^ last[i]) & ~s->vol[i];
			if (count[i] < UINT16_MAX) {
				count[i]++;
			}
			last[i] = v;
		}
		k_yield();
	}
	for (size_t i = 0; i < n; i++) {
		if (moved[i] != 0) {
			shell_print(sh, "%08lx: %08x .. %08x  bits %08x  %u changes", s->phys + 4 * i,
				    s->val[i], last[i], moved[i], count[i]);
			shown++;
		}
	}
	shell_print(sh, "%s: %d words moved in %llu ms (volatile bits excluded)", s->name, shown,
		    ms);
	k_free(moved);
	k_free(last);
	k_free(count);
	return 0;
}

/* find <name> <value> [mask]: words (current values) with (v & mask) == value */
static int cmd_find(const struct shell *sh, size_t argc, char **argv)
{
	struct snap *s = find(argv[1]);
	unsigned long long val, mask = 0xffffffff;
	int n = 0;

	if (s == NULL || num(sh, argv[2], &val) != 0 ||
	    (argc > 3 && num(sh, argv[3], &mask) != 0)) {
		shell_error(sh, "find <snapshot> <value> [mask]");
		return -EINVAL;
	}
	for (size_t i = 0; i < s->len / 4; i++) {
		uint32_t v;

		if (!s->bad[i] && rd(s, i, &v) == 0 && (v & mask) == val) {
			shell_print(sh, "%08lx: %08x%s", s->phys + 4 * i, v,
				    s->vol[i] ? "  (volatile)" : "");
			n++;
		}
	}
	shell_print(sh, "%d words", n);
	return 0;
}

static int cmd_ls(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	for (int i = 0; i < SNAPS; i++) {
		if (snaps[i].val != NULL) {
			size_t vol = 0, bad = 0;

			for (size_t j = 0; j < snaps[i].len / 4; j++) {
				vol += snaps[i].vol[j] != 0;
				bad += snaps[i].bad[j];
			}
			shell_print(sh, "%-11s 0x%08lx + 0x%05zx  %zu abort, %zu volatile",
				    snaps[i].name, snaps[i].phys, snaps[i].len, bad, vol);
		}
	}
	return 0;
}

static int cmd_drop(const struct shell *sh, size_t argc, char **argv)
{
	struct snap *s = find(argv[1]);

	if (s == NULL) {
		shell_error(sh, "%s: no such snapshot", argv[1]);
		return -ENOENT;
	}
	drop(s);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(regdiff_cmds,
	SHELL_CMD_ARG(snap, NULL, "<name> <phys> <len> [settle ms]  Snapshot (volatile words "
		      "found during settle)", cmd_snap, 4, 1),
	SHELL_CMD_ARG(diff, NULL, "<name> [-u] [-a]  Changed words (-u: update, -a: with "
		      "volatile)", cmd_diff, 2, 2),
	SHELL_CMD_ARG(event, NULL, "<name> <peer command...>  Fresh baseline, command on the lab "
		      "peer, diff", cmd_event, 3, 16),
	SHELL_CMD_ARG(watch, NULL, "<name> <ms>  Words that move during ms, how often",
		      cmd_watch, 3, 0),
	SHELL_CMD_ARG(find, NULL, "<name> <value> [mask]  Words matching now (e.g. own bus "
		      "number)", cmd_find, 3, 1),
	SHELL_CMD_ARG(drop, NULL, "<name>", cmd_drop, 2, 0),
	SHELL_SUBCMD_SET_END);

static void regdiff_fini(void)
{
	for (int i = 0; i < SNAPS; i++) {
		if (snaps[i].val != NULL) {
			drop(&snaps[i]);
		}
	}
}

LAB_DBG_CMD((dbg), "dbg", regdiff, &regdiff_cmds,
	    "Register snapshots: snap, diff, event (peer command), watch", cmd_ls, 1, 0, NULL,
	    regdiff_fini);
