/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe fabric node: regions, cards, messages, RMA writes. Transport
 * neutral: the owner of a node maps the peers' regions (an endpoint
 * through its outbound window, the root complex through its BAR
 * windows) and the node only writes there; what it reads is its own
 * region.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pcie_fabric, CONFIG_PCIE_FABRIC_LOG_LEVEL);

static sys_slist_t nodes = SYS_SLIST_STATIC_INIT(&nodes);

static inline uint64_t cycles(void)
{
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
	return k_cycle_get_64();
#else
	return k_cycle_get_32();
#endif
}

static inline uint64_t cycles_ns(uint64_t t0)
{
#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
	return k_cyc_to_ns_floor64(k_cycle_get_64() - t0);
#else
	return k_cyc_to_ns_floor64((uint32_t)(k_cycle_get_32() - (uint32_t)t0));
#endif
}

const char *pfab_role_name(uint16_t role)
{
	static const char *const n[] = {"none", "manager", "compute", "storage", "io"};

	return role < ARRAY_SIZE(n) ? n[role] : "custom";
}

const char *pfab_state_name(uint32_t state)
{
	static const char *const n[] = {"init", "ready", "running", "error"};

	return state < ARRAY_SIZE(n) ? n[state] : "?";
}

/* ------------------------------------------------------------------------ */
/* Region                                                                    */

bool pfab_geom_valid(const struct pfab_geom *g)
{
	uint64_t rings = (uint64_t)g->max_nodes * g->nslots * g->slot_size;

	return g->max_nodes >= 2 && g->max_nodes <= PFAB_MAX_PEERS && g->nslots >= 2 &&
	       g->slot_size >= 32 && (g->slot_size & 3) == 0 && (g->rx_off & 3) == 0 &&
	       (g->cred_off & 3) == 0 && g->cred_off >= PFAB_CARD_PEERS +
	       PFAB_MAX_PEERS * PFAB_PEER_SIZE && g->cred_off + 4U * g->max_nodes <= g->rx_off &&
	       g->rx_off + rings <= g->size && g->rma_off >= g->rx_off + rings &&
	       (uint64_t)g->rma_off + g->rma_size <= g->size;
}

int pfab_geom_default(struct pfab_geom *g, size_t size)
{
	uint32_t rings = CONFIG_PCIE_FABRIC_MAX_NODES * CONFIG_PCIE_FABRIC_SLOTS *
			 CONFIG_PCIE_FABRIC_SLOT_SIZE;

	*g = (struct pfab_geom){
		.size = size,
		.max_nodes = CONFIG_PCIE_FABRIC_MAX_NODES,
		.nslots = CONFIG_PCIE_FABRIC_SLOTS,
		.slot_size = CONFIG_PCIE_FABRIC_SLOT_SIZE,
		.rx_off = PFAB_RINGS,
		.cred_off = PFAB_CREDITS,
		.rma_off = ROUND_UP(PFAB_RINGS + rings, 4096),
	};
	if (g->rma_off > size) {
		return -ENOMEM;
	}
	g->rma_size = size - g->rma_off;
	return pfab_geom_valid(g) ? 0 : -EINVAL;
}

static void wr_str(volatile uint8_t *r, uint32_t off, const char *s)
{
	uint8_t b[PFAB_STR_LEN] = {0};

	if (s != NULL) {
		strncpy((char *)b, s, sizeof(b) - 1);
	}
	for (int i = 0; i < PFAB_STR_LEN; i += 4) {
		pfab_wr(r, off + i, b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) |
				     ((uint32_t)b[i + 3] << 24));
	}
}

void pfab_region_init(volatile uint8_t *r, const struct pfab_geom *g, uint64_t dsn,
		      uint32_t caps, uint32_t roles, const char *fw, const char *board)
{
	for (uint32_t o = 0; o < g->size; o += 4) {
		pfab_wr(r, o, 0);
	}
	pfab_wr(r, PFAB_INFO_VERSION, PFAB_VERSION);
	pfab_wr(r, PFAB_INFO_SIZE, g->size);
	pfab_wr64(r, PFAB_INFO_DSN, dsn);
	pfab_wr(r, PFAB_INFO_CAPS, caps);
	pfab_wr(r, PFAB_INFO_ROLES, roles);
	pfab_wr(r, PFAB_INFO_GEOM, g->max_nodes | ((uint32_t)g->nslots << 16));
	pfab_wr(r, PFAB_INFO_SLOT, g->slot_size);
	pfab_wr(r, PFAB_INFO_RX_OFF, g->rx_off);
	pfab_wr(r, PFAB_INFO_CRED_OFF, g->cred_off);
	pfab_wr(r, PFAB_INFO_RMA_OFF, g->rma_off);
	pfab_wr(r, PFAB_INFO_RMA_SIZE, g->rma_size);
	wr_str(r, PFAB_INFO_FW, fw);
	wr_str(r, PFAB_INFO_BOARD, board);
	pfab_wr(r, PFAB_INFO_STATE, PFAB_STATE_READY);
	barrier_dmem_fence_full();
	/* magic last: a reader that sees it sees the rest */
	pfab_wr(r, PFAB_INFO_MAGIC, PFAB_MAGIC);
}

int pfab_region_geom(volatile const uint8_t *r, struct pfab_geom *g)
{
	uint32_t geom;

	if (pfab_rd(r, PFAB_INFO_MAGIC) != PFAB_MAGIC ||
	    (pfab_rd(r, PFAB_INFO_VERSION) >> 16) != (PFAB_VERSION >> 16)) {
		return -EINVAL;
	}
	geom = pfab_rd(r, PFAB_INFO_GEOM);
	*g = (struct pfab_geom){
		.size = pfab_rd(r, PFAB_INFO_SIZE),
		.max_nodes = geom & 0xffff,
		.nslots = geom >> 16,
		.slot_size = pfab_rd(r, PFAB_INFO_SLOT),
		.rx_off = pfab_rd(r, PFAB_INFO_RX_OFF),
		.cred_off = pfab_rd(r, PFAB_INFO_CRED_OFF),
		.rma_off = pfab_rd(r, PFAB_INFO_RMA_OFF),
		.rma_size = pfab_rd(r, PFAB_INFO_RMA_SIZE),
	};
	return pfab_geom_valid(g) ? 0 : -EINVAL;
}

/* ------------------------------------------------------------------------ */
/* Node                                                                      */

void pfab_node_init(struct pfab_node *n, const char *name, volatile uint8_t *local,
		    const struct pfab_geom *g)
{
	memset(n, 0, sizeof(*n));
	n->name = name;
	n->local = local;
	n->g = *g;
	n->poll_us = CONFIG_PCIE_FABRIC_POLL_US;
	k_mutex_init(&n->lock);
	k_msgq_init(&n->rxq, n->rxq_buf, sizeof(struct pfab_msg), CONFIG_PCIE_FABRIC_RXQ_DEPTH);
}

void pfab_node_register(struct pfab_node *n)
{
	sys_slist_append(&nodes, &n->reg);
}

struct pfab_node *pfab_node_get(int i)
{
	struct pfab_node *n;

	SYS_SLIST_FOR_EACH_CONTAINER(&nodes, n, reg) {
		if (i-- == 0) {
			return n;
		}
	}
	return NULL;
}

static void clear_rx(struct pfab_node *n)
{
	uint32_t rings = (uint32_t)n->g.max_nodes * n->g.nslots * n->g.slot_size;

	for (uint32_t o = 0; o < rings; o += 4) {
		pfab_wr(n->local, n->g.rx_off + o, 0);
	}
	for (uint32_t i = 0; i < n->g.max_nodes; i++) {
		pfab_wr(n->local, n->g.cred_off + 4 * i, 0);
	}
	memset(n->rx_count, 0, sizeof(n->rx_count));
	k_msgq_purge(&n->rxq);
}

static void drop_peers(struct pfab_node *n, const struct pfab_map_ops *ops)
{
	for (int i = 0; i < ARRAY_SIZE(n->peer); i++) {
		if (n->peer[i].valid && ops != NULL && ops->unmap != NULL) {
			ops->unmap(ops->ctx, n->peer[i].map);
		}
		memset(&n->peer[i], 0, sizeof(n->peer[i]));
	}
}

void pfab_node_stop(struct pfab_node *n, const struct pfab_map_ops *ops)
{
	k_mutex_lock(&n->lock, K_FOREVER);
	n->running = false;
	drop_peers(n, ops);
	pfab_wr(n->local, PFAB_INFO_STATE, PFAB_STATE_READY);
	k_mutex_unlock(&n->lock);
}

int pfab_card_pending(struct pfab_node *n)
{
	uint32_t commit = pfab_rd(n->local, PFAB_CARD_COMMIT);

	if (commit == 0) {
		return n->running ? -ECANCELED : 0;
	}
	if (n->running && commit == n->epoch) {
		return 0;
	}
	if (!n->running && pfab_rd(n->local, PFAB_INFO_STATE) == PFAB_STATE_ERROR &&
	    pfab_rd(n->local, PFAB_INFO_EPOCH) == commit) {
		return 0; /* refused already */
	}
	return 1;
}

static int card_peer(struct pfab_node *n, volatile const uint8_t *e, struct pfab_peer *p)
{
	uint32_t node = pfab_rd(e, PFAB_PEER_NODE);
	uint32_t geom = pfab_rd(e, PFAB_PEER_GEOM);

	*p = (struct pfab_peer){
		.id = node & 0xffff,
		.role = node >> 16,
		.dsn = pfab_rd64(e, PFAB_PEER_DSN),
		.bus = pfab_rd64(e, PFAB_PEER_BUS),
		.g = {
			.size = pfab_rd(e, PFAB_PEER_SIZE_OFF),
			.max_nodes = geom & 0xffff,
			.nslots = geom >> 16,
			.slot_size = pfab_rd(e, PFAB_PEER_SLOT),
			.rx_off = pfab_rd(e, PFAB_PEER_RX_OFF),
			.cred_off = pfab_rd(e, PFAB_PEER_CRED_OFF),
			.rma_off = pfab_rd(e, PFAB_PEER_RMA_OFF),
			.rma_size = pfab_rd(e, PFAB_PEER_RMA_SIZE),
		},
	};
	if ((pfab_rd(e, PFAB_PEER_FLAGS) & PFAB_PEER_VALID) == 0) {
		return -ENOENT;
	}
	if (p->id >= n->g.max_nodes || p->id == n->id || !pfab_geom_valid(&p->g) ||
	    n->id >= p->g.max_nodes || p->g.slot_size > CONFIG_PCIE_FABRIC_SLOT_SIZE ||
	    p->bus == 0) {
		return -EINVAL;
	}
	return 0;
}

int pfab_card_accept(struct pfab_node *n, const struct pfab_map_ops *ops)
{
	volatile uint8_t *r = n->local;
	uint32_t commit, node, npeers;
	int ret = 0;

	k_mutex_lock(&n->lock, K_FOREVER);
	commit = pfab_rd(r, PFAB_CARD_COMMIT);
	barrier_dmem_fence_full();
	n->running = false;
	drop_peers(n, ops);

	node = pfab_rd(r, PFAB_CARD_NODE);
	npeers = pfab_rd(r, PFAB_CARD_NPEERS);
	if (commit == 0 || pfab_rd(r, PFAB_CARD_MAGIC_OFF) != PFAB_CARD_MAGIC ||
	    pfab_rd(r, PFAB_CARD_EPOCH) != commit) {
		ret = -EINVAL;
		goto out;
	}
	if ((node & 0xffff) >= n->g.max_nodes || npeers > PFAB_MAX_PEERS) {
		ret = -ERANGE;
		goto out;
	}
	n->id = node & 0xffff;
	n->role = node >> 16;
	n->epoch = commit;

	for (uint32_t i = 0; i < npeers; i++) {
		struct pfab_peer p;
		int r2 = card_peer(n, r + PFAB_CARD_PEERS + i * PFAB_PEER_SIZE, &p);

		if (r2 == -ENOENT) {
			continue;
		}
		if (r2 != 0 || n->peer[p.id].valid) {
			LOG_WRN("%s: card peer %u refused (%d)", n->name, i, r2 ? r2 : -EEXIST);
			continue;
		}
		if (ops != NULL && ops->map != NULL) {
			r2 = ops->map(ops->ctx, p.bus, p.g.size, &p.base, &p.map);
			if (r2 != 0) {
				LOG_WRN("%s: peer %u at 0x%llx not mapped (%d)", n->name, p.id,
					(unsigned long long)p.bus, r2);
				continue;
			}
		}
		p.valid = p.base != NULL;
		n->peer[p.id] = p;
	}
	clear_rx(n);
	n->running = true;
	n->st.cards++;
out:
	pfab_wr(r, PFAB_INFO_ERROR, (uint32_t)ret);
	pfab_wr(r, PFAB_INFO_NODE, ret == 0 ? node : 0);
	pfab_wr(r, PFAB_INFO_EPOCH, commit);
	barrier_dmem_fence_full();
	pfab_wr(r, PFAB_INFO_STATE, ret == 0 ? PFAB_STATE_RUNNING : PFAB_STATE_ERROR);
	k_mutex_unlock(&n->lock);
	if (ret == 0) {
		LOG_INF("%s: node %u (%s), epoch %08x", n->name, n->id, pfab_role_name(n->role),
			n->epoch);
	} else {
		LOG_WRN("%s: card refused (%d)", n->name, ret);
	}
	return ret;
}

void pfab_node_start(struct pfab_node *n, uint16_t id, uint16_t role, uint32_t epoch,
		     const struct pfab_peer *peers, int npeers)
{
	k_mutex_lock(&n->lock, K_FOREVER);
	n->running = false;
	drop_peers(n, NULL);
	n->id = id;
	n->role = role;
	n->epoch = epoch;
	for (int i = 0; i < npeers; i++) {
		if (peers[i].valid && peers[i].id < ARRAY_SIZE(n->peer) && peers[i].id != id) {
			n->peer[peers[i].id] = peers[i];
			n->peer[peers[i].id].sent = 0;
		}
	}
	clear_rx(n);
	pfab_wr(n->local, PFAB_INFO_NODE, id | ((uint32_t)role << 16));
	pfab_wr(n->local, PFAB_INFO_EPOCH, epoch);
	pfab_wr(n->local, PFAB_INFO_STATE, PFAB_STATE_RUNNING);
	n->running = true;
	k_mutex_unlock(&n->lock);
}

/* ------------------------------------------------------------------------ */
/* Messages                                                                  */

/* 32-bit stores only, the last word assembled from the tail bytes */
static void copy_out(volatile uint8_t *dst, const uint8_t *src, size_t len)
{
	size_t i;

	for (i = 0; i + 4 <= len; i += 4) {
		uint32_t w;

		memcpy(&w, src + i, 4);
		*(volatile uint32_t *)(dst + i) = w;
	}
	if (i < len) {
		uint32_t w = 0;

		memcpy(&w, src + i, len - i);
		*(volatile uint32_t *)(dst + i) = w;
	}
}

static void copy_in(uint8_t *dst, volatile const uint8_t *src, size_t len)
{
	for (size_t i = 0; i < len; i += 4) {
		uint32_t w = *(volatile const uint32_t *)(src + i);

		memcpy(dst + i, &w, MIN(4U, len - i));
	}
}

int pfab_send(struct pfab_node *n, uint16_t dst, uint16_t type, const void *data, size_t len)
{
	struct pfab_peer *p;
	volatile uint8_t *slot;
	uint32_t credit, seq;
	int ret = 0;

	if (dst >= ARRAY_SIZE(n->peer)) {
		return -ENOTCONN;
	}
	k_mutex_lock(&n->lock, K_FOREVER);
	p = &n->peer[dst];
	if (!n->running || !p->valid) {
		ret = -ENOTCONN;
		goto out;
	}
	if (len > p->g.slot_size - PFAB_SLOT_HDR) {
		ret = -EMSGSIZE;
		goto out;
	}
	/* the receiver tells us in our region how many it took */
	credit = pfab_rd(n->local, n->g.cred_off + 4U * dst);
	if (p->sent - credit >= p->g.nslots) {
		n->st.tx_full++;
		ret = -EAGAIN;
		goto out;
	}
	seq = p->sent + 1;
	slot = p->base + p->g.rx_off +
	       ((uint32_t)n->id * p->g.nslots + (seq - 1) % p->g.nslots) * p->g.slot_size;
	pfab_wr(slot, PFAB_SLOT_LEN, (uint32_t)len | ((uint32_t)type << 16));
	pfab_wr(slot, PFAB_SLOT_SRC, n->id | ((n->epoch & 0xffff) << 16));
	copy_out(slot + PFAB_SLOT_HDR, data, len);
	/* posted writes on one path stay in order: the sequence word last */
	barrier_dmem_fence_full();
	pfab_wr(slot, PFAB_SLOT_SEQ, seq);
	p->sent = seq;
	n->st.tx++;
out:
	k_mutex_unlock(&n->lock);
	return ret;
}

int pfab_poll(struct pfab_node *n, struct pfab_msg *m)
{
	int ret = 0;

	k_mutex_lock(&n->lock, K_FOREVER);
	if (!n->running) {
		goto out;
	}
	for (int k = 0; k < n->g.max_nodes; k++) {
		uint16_t s = (n->rr + k) % n->g.max_nodes;
		struct pfab_peer *p = &n->peer[s];
		uint32_t want = n->rx_count[s] + 1;
		volatile uint8_t *slot;
		uint32_t lt, src;

		if (!p->valid) {
			continue;
		}
		slot = n->local + n->g.rx_off +
		       ((uint32_t)s * n->g.nslots + (want - 1) % n->g.nslots) * n->g.slot_size;
		if (pfab_rd(slot, PFAB_SLOT_SEQ) != want) {
			continue;
		}
		barrier_dmem_fence_full();
		lt = pfab_rd(slot, PFAB_SLOT_LEN);
		src = pfab_rd(slot, PFAB_SLOT_SRC);
		if ((src & 0xffff) != s || (src >> 16) != (n->epoch & 0xffff) ||
		    (lt & 0xffff) > n->g.slot_size - PFAB_SLOT_HDR) {
			/* a late write of an older epoch, or garbage: not ours */
			n->st.rx_stale++;
			continue;
		}
		m->src = s;
		m->type = lt >> 16;
		m->len = lt & 0xffff;
		copy_in(m->data, slot + PFAB_SLOT_HDR, m->len);
		n->rx_count[s] = want;
		/* credit back into the sender's region */
		barrier_dmem_fence_full();
		pfab_wr(p->base, p->g.cred_off + 4U * n->id, want);
		n->rr = (s + 1) % n->g.max_nodes;
		n->st.rx++;
		ret = 1;
		break;
	}
out:
	k_mutex_unlock(&n->lock);
	return ret;
}

int pfab_put(struct pfab_node *n, uint16_t dst, uint32_t off, const void *data, size_t len)
{
	struct pfab_peer *p;
	int ret = 0;

	if (dst >= ARRAY_SIZE(n->peer) || (off & 3) != 0) {
		return dst >= ARRAY_SIZE(n->peer) ? -ENOTCONN : -EINVAL;
	}
	k_mutex_lock(&n->lock, K_FOREVER);
	p = &n->peer[dst];
	if (!n->running || !p->valid) {
		ret = -ENOTCONN;
	} else if ((uint64_t)off + len > p->g.rma_size) {
		ret = -ERANGE;
	} else {
		copy_out(p->base + p->g.rma_off + off, data, len);
		n->st.puts++;
	}
	k_mutex_unlock(&n->lock);
	return ret;
}

volatile uint8_t *pfab_rma_local(struct pfab_node *n, size_t *size)
{
	if (size != NULL) {
		*size = n->g.rma_size;
	}
	return n->local + n->g.rma_off;
}

int pfab_service(struct pfab_node *n)
{
	struct pfab_msg m;
	int handled = 0;

	if (atomic_get(&n->exclusive) != 0) {
		return 0;
	}
	while (handled < n->g.max_nodes && pfab_poll(n, &m) == 1) {
		handled++;
		if (m.type == PFAB_TYPE_ECHO_REQ) {
			n->st.echo++;
			/* a full ring at the requester: it waits for us anyway */
			for (int i = 0; i < 100 && pfab_send(n, m.src, PFAB_TYPE_ECHO_REP, m.data,
							     m.len) == -EAGAIN; i++) {
				k_yield();
			}
		} else if (k_msgq_put(&n->rxq, &m, K_NO_WAIT) != 0) {
			n->st.rx_dropped++;
		}
	}
	return handled;
}

void pfab_service_wait(struct pfab_node *n)
{
	k_tid_t self = k_current_get();
	int want = n->poll_us == 0 ? K_LOWEST_APPLICATION_THREAD_PRIO
				   : CONFIG_PCIE_FABRIC_THREAD_PRIORITY;

	/* busy polling only takes what nobody else wants */
	if (k_thread_priority_get(self) != want) {
		k_thread_priority_set(self, want);
	}
	if (n->poll_us == 0) {
		k_yield();
	} else {
		k_usleep(n->poll_us);
	}
}

int pfab_recv(struct pfab_node *n, struct pfab_msg *m, k_timeout_t timeout)
{
	return k_msgq_get(&n->rxq, m, timeout);
}

int pfab_ping(struct pfab_node *n, uint16_t dst, uint32_t count, size_t len,
	      k_timeout_t timeout, struct pfab_ping_result *r)
{
	uint8_t buf[PFAB_PAYLOAD_MAX] __aligned(4);
	struct pfab_msg m;
	int ret = 0;

	memset(r, 0, sizeof(*r));
	r->min_ns = UINT64_MAX;
	len = MAX(len, sizeof(uint32_t));
	if (len > sizeof(buf)) {
		return -EMSGSIZE;
	}
	for (size_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)(i * 7 + 1);
	}
	atomic_set(&n->exclusive, 1);
	for (uint32_t i = 0; i < count; i++) {
		k_timepoint_t end = sys_timepoint_calc(timeout);
		uint64_t t0, dt;
		bool got = false;

		memcpy(buf, &i, sizeof(i));
		t0 = cycles();
		ret = pfab_send(n, dst, PFAB_TYPE_ECHO_REQ, buf, len);
		if (ret != 0) {
			break;
		}
		r->sent++;
		while (!got) {
			if (pfab_poll(n, &m) == 1) {
				uint32_t seq;

				memcpy(&seq, m.data, sizeof(seq));
				if (m.type == PFAB_TYPE_ECHO_REP && m.src == dst && seq == i &&
				    m.len == len) {
					got = true;
				} else if (m.type == PFAB_TYPE_ECHO_REQ) {
					(void)pfab_send(n, m.src, PFAB_TYPE_ECHO_REP, m.data, m.len);
				} else if (m.type != PFAB_TYPE_ECHO_REP &&
					   k_msgq_put(&n->rxq, &m, K_NO_WAIT) != 0) {
					n->st.rx_dropped++;
				}
				continue;
			}
			if (sys_timepoint_expired(end)) {
				break;
			}
			/* a node's service of the same priority (one CPU) */
			k_yield();
		}
		if (!got) {
			continue;
		}
		dt = cycles_ns(t0);
		r->received++;
		r->sum_ns += dt;
		r->min_ns = MIN(r->min_ns, dt);
		r->max_ns = MAX(r->max_ns, dt);
	}
	atomic_set(&n->exclusive, 0);
	return ret;
}
