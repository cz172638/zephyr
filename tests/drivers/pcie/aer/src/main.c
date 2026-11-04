/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * pcie_aer helpers against an emulated tree:
 *
 *   00:00.0 root port (AER) -> 01:00.0 switch upstream (AER)
 *     02:00.0 downstream (AER) -> 03:00.0 endpoint (AER)
 *     02:01.0 downstream (no AER) -> 04:00.0 endpoint (no AER), multifunction
 *                                    04:00.1 endpoint (AER)
 *
 * RW1C: Status, Secondary Status, Device Status, AER UE/CE status, Root
 * Error Status. A read of anything absent gives ~0 and counts as a UR.
 */

#include <string.h>

#include <zephyr/drivers/pcie/aer.h>
#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/ztest.h>

#define NFN 7
#define EXP 0x40
#define AER 0x100

struct fn {
	pcie_bdf_t bdf;
	uint32_t cfg[1024];
};

static struct fn fns[NFN];
static int absent_reads;

static struct fn *lookup(pcie_bdf_t bdf)
{
	for (int i = 0; i < NFN; i++) {
		if (fns[i].bdf == bdf) {
			return &fns[i];
		}
	}
	return NULL;
}

static uint32_t f_read(const struct device *dev, pcie_bdf_t bdf, unsigned int reg)
{
	struct fn *f = lookup(bdf);

	ARG_UNUSED(dev);
	if (f == NULL) {
		absent_reads++;
		return 0xffffffff;
	}
	return f->cfg[reg];
}

static bool w1c_reg(unsigned int off, uint32_t *mask, bool bridge)
{
	switch (off) {
	case 0x04:
		*mask = 0xffff0000;
		return true;
	case 0x1c:
		*mask = bridge ? 0xffff0000 : 0;
		return bridge;
	case EXP + 0x08:
		*mask = 0xffff0000;
		return true;
	case AER + 0x04:
	case AER + 0x10:
	case AER + 0x30:
		*mask = 0xffffffff;
		return true;
	default:
		return false;
	}
}

static void f_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg, uint32_t v)
{
	struct fn *f = lookup(bdf);
	uint32_t m;

	ARG_UNUSED(dev);
	if (f == NULL) {
		return;
	}
	bool bridge = ((f->cfg[3] >> 16) & 0x7f) == 1;

	if (w1c_reg(reg * 4, &m, bridge)) {
		f->cfg[reg] = (f->cfg[reg] & m & ~v) | (v & ~m);
	} else {
		f->cfg[reg] = v;
	}
}

static DEVICE_API(pcie_ctrl, api) = {
	.conf_read = f_read,
	.conf_write = f_write,
};

static const struct device ctrl = {
	.name = "fake-pcie",
	.api = &api,
};

static void mk(int i, pcie_bdf_t bdf, int type, bool bridge, int sec, int sub, bool aer,
	       bool mf)
{
	struct fn *f = &fns[i];

	memset(f, 0, sizeof(*f));
	f->bdf = bdf;
	f->cfg[0] = 0x00011b4b;
	f->cfg[1] = BIT(20); /* capabilities list */
	f->cfg[3] = ((bridge ? 1U : 0U) | (mf ? 0x80U : 0U)) << 16;
	if (bridge) {
		f->cfg[0x18 / 4] = (sub << 16) | (sec << 8) | PCIE_BDF_TO_BUS(bdf);
	}
	f->cfg[0x34 / 4] = EXP;
	f->cfg[EXP / 4] = 0x10 | (2U << 16) | ((uint32_t)type << 20);
	if (aer) {
		f->cfg[AER / 4] = 0x0001 | (1U << 16);
		/* ECRC gen and check capable */
		f->cfg[(AER + 0x18) / 4] = BIT(5) | BIT(7);
		/* default CE mask: Advisory Non-Fatal */
		f->cfg[(AER + 0x14) / 4] = BIT(13);
	}
}

static void *setup(void)
{
	return NULL;
}

static void before(void *f)
{
	ARG_UNUSED(f);
	mk(0, PCIE_BDF(0, 0, 0), PCIE_AER_TYPE_RP, true, 1, 4, true, false);
	mk(1, PCIE_BDF(1, 0, 0), PCIE_AER_TYPE_UP, true, 2, 4, true, false);
	mk(2, PCIE_BDF(2, 0, 0), PCIE_AER_TYPE_DP, true, 3, 3, true, false);
	mk(3, PCIE_BDF(2, 1, 0), PCIE_AER_TYPE_DP, true, 4, 4, false, false);
	mk(4, PCIE_BDF(3, 0, 0), PCIE_AER_TYPE_EP, false, 0, 0, true, false);
	mk(5, PCIE_BDF(4, 0, 0), PCIE_AER_TYPE_EP, false, 0, 0, false, true);
	mk(6, PCIE_BDF(4, 0, 1), PCIE_AER_TYPE_EP, false, 0, 0, true, false);
	absent_reads = 0;
	pcie_aer_stat_clear();
}

struct seen {
	pcie_bdf_t bdf[16];
	int n;
};

static void visit(const struct device *dev, pcie_bdf_t bdf, void *user)
{
	struct seen *s = user;

	ARG_UNUSED(dev);
	s->bdf[s->n++] = bdf;
}

ZTEST(pcie_aer, test_walk)
{
	struct seen s = {0};

	zassert_equal(pcie_aer_walk(&ctrl, PCIE_BDF(0, 0, 0), visit, &s), NFN);
	zassert_equal(s.n, NFN);
	zassert_equal(s.bdf[0], PCIE_BDF(0, 0, 0));
	zassert_equal(s.bdf[1], PCIE_BDF(1, 0, 0));
	zassert_equal(s.bdf[2], PCIE_BDF(2, 0, 0));
	zassert_equal(s.bdf[3], PCIE_BDF(3, 0, 0));
	zassert_equal(s.bdf[4], PCIE_BDF(2, 1, 0));
	zassert_equal(s.bdf[5], PCIE_BDF(4, 0, 0));
	zassert_equal(s.bdf[6], PCIE_BDF(4, 0, 1));
	/*
	 * Absent probes: the switch's internal bus 2 all devices (2..31),
	 * functions 2..7 of the multifunction 04:00; none of 01:01..1f,
	 * 03:01..1f, 04:01..1f (links below root / downstream ports)
	 */
	zassert_equal(absent_reads, 30 + 6, "absent reads %d", absent_reads);
}

ZTEST(pcie_aer, test_setup)
{
	struct pcie_aer_info in;

	/* enumeration leftovers */
	fns[2].cfg[(AER + 0x04) / 4] = BIT(20);
	fns[2].cfg[(EXP + 0x08) / 4] = BIT(19);
	fns[0].cfg[(AER + 0x30) / 4] = BIT(0);

	zassert_equal(pcie_aer_setup(&ctrl, PCIE_BDF(0, 0, 0), PCIE_AER_UR_EP), NFN);

	for (int i = 0; i < NFN; i++) {
		zassert_ok(pcie_aer_info(&ctrl, fns[i].bdf, &in));
		zassert_true(in.cmd & BIT(8), "SERR# on fn %d", i);
		zassert_equal(in.devctl & 0x7, 0x7, "reporting on fn %d", i);
		zassert_equal(in.devsta, 0, "devsta fn %d", i);
		zassert_equal(in.ue, 0);
		zassert_equal(in.capctl & (BIT(6) | BIT(8)), 0, "no ECRC by default");
		if (in.bridge) {
			zassert_true(in.bctl & BIT(1), "forwarding on fn %d", i);
			zassert_false(in.devctl & BIT(3), "no UR on ports");
		} else {
			zassert_true(in.devctl & BIT(3), "UR on endpoints");
		}
	}
	zassert_ok(pcie_aer_info(&ctrl, PCIE_BDF(0, 0, 0), &in));
	zassert_equal(in.root_cmd & 7, 7);
	zassert_equal(in.root_sta, 0);

	zassert_equal(pcie_aer_setup(&ctrl, PCIE_BDF(0, 0, 0), PCIE_AER_ECRC), NFN);
	zassert_ok(pcie_aer_info(&ctrl, PCIE_BDF(3, 0, 0), &in));
	zassert_equal(in.capctl & (BIT(6) | BIT(8)), BIT(6) | BIT(8));
	zassert_false(in.devctl & BIT(3));
}

static struct pcie_aer_event evs[8];
static int nev;

static void report(const struct device *dev, const struct pcie_aer_event *ev)
{
	ARG_UNUSED(dev);
	if (nev < ARRAY_SIZE(evs)) {
		evs[nev++] = *ev;
	}
}

ZTEST(pcie_aer, test_service_latched)
{
	const struct pcie_aer_stat *st;
	struct pcie_aer_info in;

	(void)pcie_aer_setup(&ctrl, PCIE_BDF(0, 0, 0), 0);
	nev = 0;
	/* 03:00.0 sent ERR_COR (RxErr), 04:00.1 ERR_FATAL (MalfTLP) */
	fns[4].cfg[(AER + 0x10) / 4] = BIT(0);
	fns[4].cfg[(EXP + 0x08) / 4] |= BIT(16);
	fns[6].cfg[(AER + 0x04) / 4] = BIT(18);
	fns[6].cfg[(AER + 0x0c) / 4] = BIT(18);
	fns[6].cfg[(AER + 0x1c) / 4] = 0x40000001;
	fns[0].cfg[(AER + 0x30) / 4] = BIT(0) | BIT(2) | BIT(4) | BIT(6);
	fns[0].cfg[(AER + 0x34) / 4] = (0x0401U << 16) | 0x0300;

	zassert_equal(pcie_aer_root_service(&ctrl, PCIE_BDF(0, 0, 0), report), 2);
	zassert_equal(nev, 2);
	zassert_equal(evs[0].bdf, PCIE_BDF(3, 0, 0));
	zassert_equal(evs[0].sev, PCIE_AER_COR);
	zassert_equal(evs[0].ce, BIT(0));
	zassert_equal(evs[1].bdf, PCIE_BDF(4, 0, 1));
	zassert_equal(evs[1].sev, PCIE_AER_FATAL);
	zassert_equal(evs[1].ue, BIT(18));
	zassert_true(evs[1].hdr_valid);
	zassert_equal(evs[1].hdr[0], 0x40000001);

	zassert_ok(pcie_aer_info(&ctrl, PCIE_BDF(3, 0, 0), &in));
	zassert_equal(in.ce, 0);
	zassert_equal(in.devsta, 0);
	zassert_ok(pcie_aer_info(&ctrl, PCIE_BDF(4, 0, 1), &in));
	zassert_equal(in.ue, 0);
	zassert_ok(pcie_aer_info(&ctrl, PCIE_BDF(0, 0, 0), &in));
	zassert_equal(in.root_sta, 0);

	st = pcie_aer_stat_get(0);
	zassert_not_null(st);
	zassert_equal(st->bdf, PCIE_BDF(3, 0, 0));
	zassert_equal(st->cnt[PCIE_AER_COR], 1);
	st = pcie_aer_stat_get(1);
	zassert_not_null(st);
	zassert_equal(st->cnt[PCIE_AER_FATAL], 1);
	zassert_is_null(pcie_aer_stat_get(2));
}

ZTEST(pcie_aer, test_service_sweep)
{
	(void)pcie_aer_setup(&ctrl, PCIE_BDF(0, 0, 0), 0);
	nev = 0;
	/* root port latched nothing; 04:00.0 (no AER) has NonFatal in DevSta */
	fns[5].cfg[(EXP + 0x08) / 4] |= BIT(17);

	zassert_equal(pcie_aer_root_service(&ctrl, PCIE_BDF(0, 0, 0), report), 1);
	zassert_equal(evs[0].bdf, PCIE_BDF(4, 0, 0));
	zassert_equal(evs[0].sev, PCIE_AER_NONFATAL);
	zassert_equal(fns[5].cfg[(EXP + 0x08) / 4] >> 16, 0);
	/* nothing left */
	zassert_equal(pcie_aer_root_service(&ctrl, PCIE_BDF(0, 0, 0), report), 0);
}

ZTEST(pcie_aer, test_bits_str)
{
	char b[64];

	pcie_aer_bits_str(b, sizeof(b), BIT(0) | BIT(6) | BIT(1), pcie_aer_ce_name);
	zassert_str_equal(b, "RxErr ? BadTLP");
	zassert_str_equal(pcie_aer_type_name(PCIE_AER_TYPE_DP), "Downstream");
}

ZTEST_SUITE(pcie_aer, NULL, setup, before, NULL, NULL);
