/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fabric endpoint function on the fake controller. The test plays the
 * root complex: it writes the card into BAR0 and runs node 0, whose
 * region is in (simulated) host memory; the function reaches it through
 * its outbound window (fake_ob_flush() carries the writes over).
 */

#include <string.h>

#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/ztest.h>

#include "fake_epc.h"

#define RC_OFF  0x0000
#define RC_SIZE 0x4000

static struct pfab_node rc;
static struct pfab_geom rc_g;
static volatile uint8_t *bar;

static void write_card(uint32_t epoch, uint16_t id)
{
	volatile uint8_t *e = bar + PFAB_CARD_PEERS;

	pfab_wr(bar, PFAB_CARD_COMMIT, 0);
	pfab_wr(bar, PFAB_CARD_MAGIC_OFF, PFAB_CARD_MAGIC);
	pfab_wr(bar, PFAB_CARD_EPOCH, epoch);
	pfab_wr(bar, PFAB_CARD_NODE, id | (PFAB_ROLE_STORAGE << 16));
	pfab_wr(bar, PFAB_CARD_NPEERS, 1);
	pfab_wr(e, PFAB_PEER_NODE, 0 | (PFAB_ROLE_MANAGER << 16));
	pfab_wr64(e, PFAB_PEER_BUS, HOST_BASE + RC_OFF);
	pfab_wr(e, PFAB_PEER_SIZE_OFF, rc_g.size);
	pfab_wr(e, PFAB_PEER_GEOM, rc_g.max_nodes | (rc_g.nslots << 16));
	pfab_wr(e, PFAB_PEER_SLOT, rc_g.slot_size);
	pfab_wr(e, PFAB_PEER_RX_OFF, rc_g.rx_off);
	pfab_wr(e, PFAB_PEER_CRED_OFF, rc_g.cred_off);
	pfab_wr(e, PFAB_PEER_RMA_OFF, rc_g.rma_off);
	pfab_wr(e, PFAB_PEER_RMA_SIZE, rc_g.rma_size);
	pfab_wr(e, PFAB_PEER_FLAGS, PFAB_PEER_VALID);
	pfab_wr(bar, PFAB_CARD_COMMIT, epoch);
}

static bool wait_state(uint32_t state, uint32_t epoch)
{
	for (int i = 0; i < 200; i++) {
		if (pfab_rd(bar, PFAB_INFO_STATE) == state &&
		    (epoch == 0 || pfab_rd(bar, PFAB_INFO_EPOCH) == epoch)) {
			return true;
		}
		k_msleep(5);
	}
	return false;
}

/* a message for node 0, carried over from the outbound window */
static bool rc_wait(struct pfab_msg *m)
{
	for (int i = 0; i < 200; i++) {
		fake_ob_flush();
		if (pfab_poll(&rc, m) == 1) {
			return true;
		}
		k_msleep(2);
	}
	return false;
}

static void start_rc(uint32_t epoch, uint16_t ep_id)
{
	struct pfab_peer p = {
		.valid = true,
		.id = ep_id,
		.role = PFAB_ROLE_STORAGE,
		.bus = fake_bar[0].phys,
		.base = bar,
	};

	zassert_ok(pfab_region_geom(bar, &p.g));
	pfab_node_start(&rc, 0, PFAB_ROLE_MANAGER, epoch, &p, 1);
}

static void *setup(void)
{
	volatile uint8_t *r = host_ptr(HOST_BASE + RC_OFF);

	bar = fake_bar[0].virt;
	zassert_not_null(bar, "BAR0 not set up");
	zassert_ok(pfab_geom_default(&rc_g, RC_SIZE));
	pfab_region_init(r, &rc_g, 0, PFAB_CAP_MSG, BIT(PFAB_ROLE_MANAGER), "rc", "host");
	pfab_node_init(&rc, "rc", r, &rc_g);
	return NULL;
}

ZTEST(pfab_epf, test_1_info)
{
	struct pfab_geom g;

	zassert_equal(fake_bar[0].size, 0x4000);
	zassert_true(fake_bar[0].flags & PCIE_EPC_BAR_MEM64);
	zassert_ok(pfab_region_geom(bar, &g));
	zassert_equal(g.size, 0x4000);
	zassert_equal(pfab_rd(bar, PFAB_INFO_ROLES), 0x0c);
	zassert_equal(pfab_rd(bar, PFAB_INFO_STATE), PFAB_STATE_READY);
	zassert_equal(fake_hdr.vendor_id, 0x11ab);
	zassert_equal(fake_hdr.baseclass, 0x0b);
	/* unused BARs off, the reserved ones (upper halves here) left alone */
	zassert_equal(fake_bars_cleared, BIT(2) | BIT(4));
}

ZTEST(pfab_epf, test_2_card_and_messages)
{
	struct pfab_msg m;
	struct pfab_node *ep = pfab_node_get(0);
	uint8_t data[24];

	zassert_not_null(ep);
	write_card(0x1234, 2);
	zassert_true(wait_state(PFAB_STATE_RUNNING, 0x1234), "card not taken");
	zassert_equal(pfab_rd(bar, PFAB_INFO_NODE), 2 | (PFAB_ROLE_STORAGE << 16));
	zassert_equal(fake_maps, 1, "node 0 mapped through the outbound window");
	start_rc(0x1234, 2);

	/* node 0 -> endpoint: echo comes back through the outbound window */
	for (int i = 0; i < sizeof(data); i++) {
		data[i] = i ^ 0x5a;
	}
	for (int k = 0; k < 3 * rc_g.nslots; k++) {
		data[0] = k;
		zassert_ok(pfab_send(&rc, 2, PFAB_TYPE_ECHO_REQ, data, sizeof(data)), "k %d", k);
		zassert_true(rc_wait(&m), "no echo %d", k);
		zassert_equal(m.type, PFAB_TYPE_ECHO_REP);
		zassert_equal(m.src, 2);
		zassert_mem_equal(m.data, data, sizeof(data));
	}
	zassert_equal(ep->st.echo, 3 * rc_g.nslots);

	/* user message: queued at the endpoint */
	zassert_ok(pfab_send(&rc, 2, PFAB_TYPE_USER, "hi", 2));
	zassert_ok(pfab_recv(ep, &m, K_MSEC(500)));
	zassert_equal(m.len, 2);

	/* endpoint -> node 0, and an RMA write */
	zassert_ok(pfab_send(ep, 0, PFAB_TYPE_USER, "up", 2));
	zassert_true(rc_wait(&m));
	zassert_mem_equal(m.data, "up", 2);
	zassert_ok(pfab_put(ep, 0, 8, "rmaw", 4));
	fake_ob_flush();
	zassert_mem_equal((const uint8_t *)host_ptr(HOST_BASE + RC_OFF + rc_g.rma_off + 8),
			  "rmaw", 4);
}

ZTEST(pfab_epf, test_3_withdraw_and_bme)
{
	struct pfab_node *ep = pfab_node_get(0);

	/* withdrawn: the mapping goes */
	pfab_wr(bar, PFAB_CARD_COMMIT, 0);
	zassert_true(wait_state(PFAB_STATE_READY, 0));
	zassert_equal(fake_maps, 0);
	zassert_false(ep->running);

	/* a new card waits for bus mastering */
	fake_bme = false;
	write_card(0x1235, 3);
	k_msleep(100);
	zassert_equal(pfab_rd(bar, PFAB_INFO_STATE), PFAB_STATE_READY);
	fake_bme = true;
	zassert_true(wait_state(PFAB_STATE_RUNNING, 0x1235));
	zassert_equal(ep->id, 3);

	/* a bad card is refused, the reason in the info block */
	pfab_wr(bar, PFAB_CARD_COMMIT, 0);
	zassert_true(wait_state(PFAB_STATE_READY, 0));
	write_card(0x1236, 9);
	zassert_true(wait_state(PFAB_STATE_ERROR, 0x1236));
	zassert_equal((int32_t)pfab_rd(bar, PFAB_INFO_ERROR), -ERANGE);
}

ZTEST_SUITE(pfab_epf, NULL, setup, NULL, NULL, NULL);
