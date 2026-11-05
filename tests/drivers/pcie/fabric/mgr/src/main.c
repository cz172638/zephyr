/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fabric manager on an emulated tree:
 *
 *   00:00.0 root port -> 01:00.0 switch upstream
 *     02:00.0 downstream (ACS) -> 03:00.0 fabric endpoint, DSN 0x200
 *     02:01.0 downstream (ACS) -> 04:00.0 fabric endpoint, DSN 0x100
 *     02:02.0 downstream       -> 05:00.0 some other device
 *
 * The endpoints' BAR0 hold the address of a RAM region (64-bit BAR);
 * their "firmware" is a thread that takes the cards as the endpoint
 * function does.
 */

#include <string.h>

#include <zephyr/drivers/pcie/controller.h>
#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/ztest.h>

#define EXP  0x40
#define ACS  0x100
#define RSZ  0x4000
#define NFN  8

struct fn {
	pcie_bdf_t bdf;
	uint32_t cfg[1024];
};

static struct fn fns[NFN];
static uint8_t ep_mem[3][RSZ] __aligned(4096);
static struct pfab_node ep[2];

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

	return f != NULL ? f->cfg[reg] : 0xffffffff;
}

static void f_write(const struct device *dev, pcie_bdf_t bdf, unsigned int reg, uint32_t v)
{
	struct fn *f = lookup(bdf);

	if (f != NULL) {
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

static void mk(int i, pcie_bdf_t bdf, int type, int sec, bool acs, void *bar)
{
	struct fn *f = &fns[i];

	memset(f, 0, sizeof(*f));
	f->bdf = bdf;
	f->cfg[0] = 0xfab111ab;
	f->cfg[1] = BIT(20);
	f->cfg[3] = (bar == NULL ? 1U : 0U) << 16;
	if (bar == NULL) {
		f->cfg[0x18 / 4] = (sec << 16) | (sec << 8) | PCIE_BDF_TO_BUS(bdf);
		if (type == 4) {
			f->cfg[0x18 / 4] = (5 << 16) | (1 << 8);
		} else if (type == 5) {
			f->cfg[0x18 / 4] = (5 << 16) | (2 << 8) | 1;
		}
	} else {
		uint64_t a = (uintptr_t)bar;

		f->cfg[0x10 / 4] = (uint32_t)a | 0x4; /* 64-bit memory */
		f->cfg[0x14 / 4] = (uint32_t)(a >> 32);
	}
	f->cfg[0x34 / 4] = EXP;
	f->cfg[EXP / 4] = 0x10 | ((uint32_t)type << 20);
	if (acs) {
		f->cfg[ACS / 4] = 0x000d | (1U << 16);
		/* capability: SV TB RR CR UF; control: all on */
		f->cfg[(ACS + 4) / 4] = 0x1f | (0x1fU << 16);
	}
}

static int map(void *ctx, uint64_t bus, size_t size, volatile uint8_t **cpu, void **h)
{
	*cpu = (volatile uint8_t *)(uintptr_t)bus;
	*h = NULL;
	return 0;
}

static const struct pfab_map_ops ops = {.map = map};

static volatile bool fw_run;
static K_THREAD_STACK_DEFINE(fw_stack, 4096);
static struct k_thread fw;

/* endpoint firmware: what pcie_epf_fabric's thread does */
static void fw_fn(void *a, void *b, void *c)
{
	while (fw_run) {
		for (int i = 0; i < 2; i++) {
			int p = pfab_card_pending(&ep[i]);

			if (p == -ECANCELED) {
				pfab_node_stop(&ep[i], &ops);
			} else if (p > 0) {
				(void)pfab_card_accept(&ep[i], &ops);
			}
			if (ep[i].running) {
				(void)pfab_service(&ep[i]);
			}
		}
		/* never idle (a ping spins on the same priority); time goes on */
		k_yield();
		k_busy_wait(50);
	}
}

static void *setup(void)
{
	struct pfab_geom g;

	zassert_ok(pfab_geom_default(&g, RSZ));
	mk(0, PCIE_BDF(0, 0, 0), 4, 1, false, NULL);
	mk(1, PCIE_BDF(1, 0, 0), 5, 2, false, NULL);
	mk(2, PCIE_BDF(2, 0, 0), 6, 3, true, NULL);
	mk(3, PCIE_BDF(2, 1, 0), 6, 4, true, NULL);
	mk(4, PCIE_BDF(2, 2, 0), 6, 5, false, NULL);
	mk(5, PCIE_BDF(3, 0, 0), 0, 0, false, ep_mem[0]);
	mk(6, PCIE_BDF(4, 0, 0), 0, 0, false, ep_mem[1]);
	mk(7, PCIE_BDF(5, 0, 0), 0, 0, false, ep_mem[2]); /* zeros: not a node */
	pfab_region_init(ep_mem[0], &g, 0x200, PFAB_CAP_MSG, BIT(PFAB_ROLE_COMPUTE), "fw-a", "a");
	pfab_region_init(ep_mem[1], &g, 0x100, PFAB_CAP_MSG,
			 BIT(PFAB_ROLE_STORAGE) | BIT(PFAB_ROLE_IO), "fw-b", "b");
	pfab_node_init(&ep[0], "a", ep_mem[0], &g);
	pfab_node_init(&ep[1], "b", ep_mem[1], &g);

	fw_run = true;
	k_thread_create(&fw, fw_stack, K_THREAD_STACK_SIZEOF(fw_stack), fw_fn, NULL, NULL, NULL,
			k_thread_priority_get(k_current_get()), 0, K_NO_WAIT);
	return NULL;
}

ZTEST(pfab_mgr, test_1_scan)
{
	const struct pfab_mgr_ent *e;

	zassert_equal(pfab_mgr_scan(&ctrl, PCIE_BDF(0, 0, 0)), 2);
	e = pfab_mgr_get(0);
	zassert_equal(e->bdf, PCIE_BDF(3, 0, 0));
	zassert_equal(e->dsn, 0x200);
	zassert_str_equal(e->fw, "fw-a");
	zassert_equal(e->state, PFAB_STATE_READY);
	zassert_is_null(pfab_mgr_get(2));
	/* the BAR has to decode before the manager looks */
	zassert_true(fns[5].cfg[1] & BIT(1));
}

ZTEST(pfab_mgr, test_2_up)
{
	const struct pfab_mgr_ent *e;

	zassert_equal(pfab_mgr_up(&ctrl, PCIE_BDF(0, 0, 0), K_MSEC(1000)), 2);
	/* IDs by DSN: 0x100 first */
	e = pfab_mgr_get(0);
	zassert_equal(e->dsn, 0x100);
	zassert_equal(e->id, 1);
	zassert_equal(e->role, PFAB_ROLE_STORAGE, "lowest role offered");
	zassert_equal(e->state, PFAB_STATE_RUNNING);
	e = pfab_mgr_get(1);
	zassert_equal(e->id, 2);
	zassert_equal(e->role, PFAB_ROLE_COMPUTE);
	zassert_equal(ep[1].id, 1);
	zassert_equal(ep[0].id, 2);
	/* bus mastering, ACS redirects off (SV, TB stay) */
	zassert_equal(fns[5].cfg[1] & 0x6, 0x6);
	zassert_equal(fns[2].cfg[(ACS + 4) / 4] >> 16, 0x3);
	zassert_equal(fns[3].cfg[(ACS + 4) / 4] >> 16, 0x3);
	zassert_not_null(pfab_mgr_node());
	zassert_true(pfab_mgr_node()->running);
	zassert_equal(pfab_mgr_node()->epoch, pfab_mgr_epoch());
}

ZTEST(pfab_mgr, test_3_traffic)
{
	struct pfab_node *rc = pfab_mgr_node();
	struct pfab_ping_result r;
	struct pfab_msg m;

	int prio = k_thread_priority_get(&fw);

	/* the ping spins and yields: the firmware must be of the same priority */
	k_thread_priority_set(&fw, k_thread_priority_get(k_current_get()));

	/* root complex node <-> endpoints */
	zassert_ok(pfab_ping(rc, 1, 10, 8, K_MSEC(500), &r));
	zassert_equal(r.received, 10);
	zassert_ok(pfab_ping(rc, 2, 10, 8, K_MSEC(500), &r));
	zassert_equal(r.received, 10);
	/* back: a cooperative spinner would starve ztest between the tests */
	k_thread_priority_set(&fw, prio);

	/* endpoint <-> endpoint, without the root complex */
	zassert_ok(pfab_send(&ep[0], 1, PFAB_TYPE_USER, "p2p!", 4));
	zassert_ok(pfab_recv(&ep[1], &m, K_MSEC(500)));
	zassert_equal(m.src, 2);
	zassert_mem_equal(m.data, "p2p!", 4);
	zassert_ok(pfab_put(&ep[1], 2, 0, "rma.", 4));
	zassert_mem_equal(&ep_mem[0][ep[0].g.rma_off], "rma.", 4);
	/* and up to the root complex's node */
	zassert_ok(pfab_send(&ep[1], 0, PFAB_TYPE_USER, "mgr", 3));
	/* the node's own thread is of a lower priority than the test here */
	zassert_equal(pfab_poll(rc, &m), 1);
	zassert_equal(m.src, 1);
}

ZTEST(pfab_mgr, test_4_role_and_new_epoch)
{
	uint32_t old = pfab_mgr_epoch();

	zassert_ok(pfab_mgr_role_set(0x200, PFAB_ROLE_IO));
	zassert_equal(pfab_mgr_up(&ctrl, PCIE_BDF(0, 0, 0), K_MSEC(1000)), 2);
	zassert_not_equal(pfab_mgr_epoch(), old);
	zassert_equal(ep[0].role, PFAB_ROLE_IO);
	zassert_equal(ep[0].epoch, pfab_mgr_epoch());
	/* counters start again */
	zassert_ok(pfab_send(&ep[0], 1, PFAB_TYPE_USER, "x", 1));
	zassert_equal(ep[0].peer[1].sent, 1);

	pfab_mgr_down();
	zassert_false(ep[0].running);
	zassert_false(ep[1].running);
	zassert_false(pfab_mgr_node()->running);
	fw_run = false;
	k_thread_join(&fw, K_FOREVER);
}

ZTEST_SUITE(pfab_mgr, NULL, setup, NULL, NULL, NULL);
