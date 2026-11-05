/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fabric nodes in RAM: regions are arrays, a peer's "bus address" is
 * the array's address. Messages, credits, RMA, cards, epochs.
 */

#include <string.h>

#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/ztest.h>

#define RSZ 0x4000

static uint8_t reg[3][RSZ] __aligned(4096);
static struct pfab_node node[3];
static struct pfab_geom g;
static int maps, unmaps;

static int map(void *ctx, uint64_t bus, size_t size, volatile uint8_t **cpu, void **h)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(size);
	*cpu = (volatile uint8_t *)(uintptr_t)bus;
	*h = (void *)(uintptr_t)bus;
	maps++;
	return 0;
}

static void unmap(void *ctx, void *h)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(h);
	unmaps++;
}

static const struct pfab_map_ops ops = {.map = map, .unmap = unmap};

static void peer_of(struct pfab_peer *p, int i)
{
	*p = (struct pfab_peer){
		.valid = true,
		.id = i,
		.role = PFAB_ROLE_COMPUTE,
		.bus = (uintptr_t)reg[i],
		.g = g,
		.base = reg[i],
	};
}

/* nodes 0 and 1 started directly, node 2 only initialised */
static void before(void *f)
{
	struct pfab_peer p[2];

	ARG_UNUSED(f);
	zassert_ok(pfab_geom_default(&g, RSZ));
	for (int i = 0; i < 3; i++) {
		pfab_region_init(reg[i], &g, 0x1000 + i, PFAB_CAP_MSG | PFAB_CAP_RMA, BIT(2),
				 "test", "native_sim");
		pfab_node_init(&node[i], "n", reg[i], &g);
	}
	peer_of(&p[0], 1);
	pfab_node_start(&node[0], 0, PFAB_ROLE_MANAGER, 0x55, p, 1);
	peer_of(&p[0], 0);
	pfab_node_start(&node[1], 1, PFAB_ROLE_COMPUTE, 0x55, p, 1);
	maps = unmaps = 0;
}

ZTEST(pfab_core, test_geom)
{
	struct pfab_geom r;

	zassert_ok(pfab_region_geom(reg[0], &r));
	zassert_mem_equal(&r, &g, sizeof(g));
	zassert_equal(g.rx_off, PFAB_RINGS);
	zassert_true(g.rma_size > 0);
	zassert_equal(g.rma_off + g.rma_size, RSZ);
	zassert_equal(pfab_geom_default(&r, 0x1000), -ENOMEM);
	reg[2][0] ^= 1;
	zassert_equal(pfab_region_geom(reg[2], &r), -EINVAL);
}

ZTEST(pfab_core, test_send_poll)
{
	struct pfab_msg m;
	const char hello[] = "hello, fabric";

	zassert_equal(pfab_poll(&node[1], &m), 0);
	zassert_ok(pfab_send(&node[0], 1, PFAB_TYPE_USER, hello, sizeof(hello)));
	zassert_equal(pfab_poll(&node[1], &m), 1);
	zassert_equal(m.src, 0);
	zassert_equal(m.type, PFAB_TYPE_USER);
	zassert_equal(m.len, sizeof(hello));
	zassert_mem_equal(m.data, hello, sizeof(hello));
	zassert_equal(pfab_poll(&node[1], &m), 0);
	/* odd lengths: tail word */
	zassert_ok(pfab_send(&node[1], 0, 7, "abc", 3));
	zassert_equal(pfab_poll(&node[0], &m), 1);
	zassert_equal(m.len, 3);
	zassert_mem_equal(m.data, "abc", 3);
	zassert_equal(pfab_send(&node[0], 1, 1, hello, PFAB_PAYLOAD_MAX + 1), -EMSGSIZE);
	zassert_equal(pfab_send(&node[0], 2, 1, hello, 1), -ENOTCONN);
	zassert_equal(pfab_send(&node[0], 99, 1, hello, 1), -ENOTCONN);
}

ZTEST(pfab_core, test_credits)
{
	struct pfab_msg m;
	uint32_t v;

	for (v = 0; v < g.nslots; v++) {
		zassert_ok(pfab_send(&node[0], 1, PFAB_TYPE_USER, &v, sizeof(v)));
	}
	zassert_equal(pfab_send(&node[0], 1, PFAB_TYPE_USER, &v, sizeof(v)), -EAGAIN);
	zassert_equal(node[0].st.tx_full, 1);
	/* in order, and every take frees a slot */
	for (uint32_t i = 0; i < 3 * g.nslots; i++) {
		uint32_t got;

		zassert_equal(pfab_poll(&node[1], &m), 1, "msg %u", i);
		memcpy(&got, m.data, 4);
		zassert_equal(got, i);
		v = i + g.nslots;
		zassert_ok(pfab_send(&node[0], 1, PFAB_TYPE_USER, &v, sizeof(v)));
	}
	zassert_equal(node[1].st.rx, 3 * g.nslots);
}

ZTEST(pfab_core, test_put)
{
	size_t sz;
	volatile uint8_t *w = pfab_rma_local(&node[1], &sz);
	uint8_t data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

	zassert_equal(sz, g.rma_size);
	zassert_ok(pfab_put(&node[0], 1, 64, data, sizeof(data)));
	for (int i = 0; i < 10; i++) {
		zassert_equal(w[64 + i], data[i]);
	}
	zassert_equal(pfab_put(&node[0], 1, g.rma_size - 4, data, 8), -ERANGE);
	zassert_equal(pfab_put(&node[0], 1, 2, data, 4), -EINVAL);
}

ZTEST(pfab_core, test_stale_epoch)
{
	struct pfab_msg m;

	zassert_ok(pfab_send(&node[0], 1, 1, "x", 1));
	/* node 1 got a new epoch, node 0 still sends with the old one */
	node[1].epoch = 0x66;
	zassert_equal(pfab_poll(&node[1], &m), 0);
	zassert_equal(node[1].st.rx_stale, 1);
}

static void write_card(int who, uint16_t id, uint32_t epoch, const int *peers, int np)
{
	volatile uint8_t *r = reg[who];

	pfab_wr(r, PFAB_CARD_COMMIT, 0);
	pfab_wr(r, PFAB_CARD_MAGIC_OFF, PFAB_CARD_MAGIC);
	pfab_wr(r, PFAB_CARD_EPOCH, epoch);
	pfab_wr(r, PFAB_CARD_NODE, id | (PFAB_ROLE_STORAGE << 16));
	pfab_wr(r, PFAB_CARD_NPEERS, np);
	for (int i = 0; i < np; i++) {
		volatile uint8_t *e = r + PFAB_CARD_PEERS + i * PFAB_PEER_SIZE;
		int p = peers[i];

		pfab_wr(e, PFAB_PEER_NODE, p | (PFAB_ROLE_COMPUTE << 16));
		pfab_wr(e, PFAB_PEER_FLAGS, PFAB_PEER_VALID);
		pfab_wr64(e, PFAB_PEER_DSN, 0x1000 + p);
		pfab_wr64(e, PFAB_PEER_BUS, (uintptr_t)reg[p]);
		pfab_wr(e, PFAB_PEER_SIZE_OFF, g.size);
		pfab_wr(e, PFAB_PEER_GEOM, g.max_nodes | (g.nslots << 16));
		pfab_wr(e, PFAB_PEER_SLOT, g.slot_size);
		pfab_wr(e, PFAB_PEER_RX_OFF, g.rx_off);
		pfab_wr(e, PFAB_PEER_CRED_OFF, g.cred_off);
		pfab_wr(e, PFAB_PEER_RMA_OFF, g.rma_off);
		pfab_wr(e, PFAB_PEER_RMA_SIZE, g.rma_size);
	}
	pfab_wr(r, PFAB_CARD_COMMIT, epoch);
}

ZTEST(pfab_core, test_card)
{
	struct pfab_peer p[3];
	struct pfab_msg m;
	const int peers[] = {0, 1};

	zassert_equal(pfab_card_pending(&node[2]), 0);
	write_card(2, 2, 0x55, peers, 2);
	zassert_equal(pfab_card_pending(&node[2]), 1);
	zassert_ok(pfab_card_accept(&node[2], &ops));
	zassert_equal(maps, 2);
	zassert_equal(pfab_rd(reg[2], PFAB_INFO_STATE), PFAB_STATE_RUNNING);
	zassert_equal(pfab_rd(reg[2], PFAB_INFO_EPOCH), 0x55);
	zassert_equal(pfab_rd(reg[2], PFAB_INFO_NODE), 2 | (PFAB_ROLE_STORAGE << 16));
	zassert_equal(pfab_card_pending(&node[2]), 0);
	zassert_equal(node[2].role, PFAB_ROLE_STORAGE);

	/* node 0 learns of node 2 (as the manager's node would) */
	peer_of(&p[0], 1);
	peer_of(&p[1], 2);
	pfab_node_start(&node[0], 0, PFAB_ROLE_MANAGER, 0x55, p, 2);
	zassert_ok(pfab_send(&node[2], 0, PFAB_TYPE_USER, "p2p", 3));
	zassert_equal(pfab_poll(&node[0], &m), 1);
	zassert_equal(m.src, 2);

	/* withdrawn */
	pfab_wr(reg[2], PFAB_CARD_COMMIT, 0);
	zassert_equal(pfab_card_pending(&node[2]), -ECANCELED);
	pfab_node_stop(&node[2], &ops);
	zassert_equal(unmaps, 2);
	zassert_equal(pfab_rd(reg[2], PFAB_INFO_STATE), PFAB_STATE_READY);
	zassert_equal(pfab_send(&node[2], 0, 1, "x", 1), -ENOTCONN);
}

ZTEST(pfab_core, test_card_bad)
{
	const int peers[] = {2}; /* itself */

	write_card(2, 2, 0x77, peers, 1);
	pfab_wr(reg[2], PFAB_CARD_EPOCH, 0x78); /* torn card */
	zassert_equal(pfab_card_accept(&node[2], &ops), -EINVAL);
	zassert_equal(pfab_rd(reg[2], PFAB_INFO_STATE), PFAB_STATE_ERROR);
	/* refused once, not again and again */
	zassert_equal(pfab_card_pending(&node[2]), 0);
	write_card(2, 2, 0x79, peers, 1);
	zassert_equal(pfab_card_pending(&node[2]), 1);
	zassert_ok(pfab_card_accept(&node[2], &ops));
	/* the self entry is skipped */
	zassert_equal(maps, 0);
	write_card(2, 9, 0x7a, peers, 1);
	zassert_equal(pfab_card_accept(&node[2], &ops), -ERANGE);
}

static K_THREAD_STACK_DEFINE(svc_stack, 2048);
static struct k_thread svc;
static volatile bool svc_run;

static void svc_fn(void *a, void *b, void *c)
{
	while (svc_run) {
		(void)pfab_service(&node[1]);
		k_yield();
	}
}

ZTEST(pfab_core, test_echo_ping)
{
	struct pfab_ping_result r;
	struct pfab_msg m;

	svc_run = true;
	k_thread_create(&svc, svc_stack, K_THREAD_STACK_SIZEOF(svc_stack), svc_fn, NULL, NULL,
			NULL, k_thread_priority_get(k_current_get()), 0, K_NO_WAIT);
	zassert_ok(pfab_ping(&node[0], 1, 3 * g.nslots, 20, K_MSEC(500), &r));
	zassert_equal(r.sent, 3 * g.nslots);
	zassert_equal(r.received, r.sent);
	zassert_true(r.min_ns <= r.max_ns);
	/* user messages go to the queue */
	zassert_ok(pfab_send(&node[0], 1, PFAB_TYPE_USER + 1, "q", 1));
	for (int i = 0; i < 100 && pfab_recv(&node[1], &m, K_NO_WAIT) != 0; i++) {
		k_yield();
	}
	zassert_equal(m.type, PFAB_TYPE_USER + 1);
	svc_run = false;
	k_thread_join(&svc, K_FOREVER);
	zassert_equal(node[1].st.echo, 3 * g.nslots);
}

ZTEST_SUITE(pfab_core, NULL, NULL, before, NULL, NULL);
