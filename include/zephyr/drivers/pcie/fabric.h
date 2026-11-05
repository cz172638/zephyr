/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PCIe fabric: nodes behind a root complex talking to each other
 *
 * Every node owns a region that the others can write into: an endpoint
 * its BAR0, the root complex a piece of its RAM. Only writes cross the
 * fabric (posted, ordered along one path); everything a node reads is
 * local memory.
 *
 * Region layout (offsets in bytes, all fields little endian, 32-bit
 * accesses only):
 *
 *   0x0000 info     written by the owner: magic, geometry, DSN, state
 *   0x0100 card     written by the root complex: node ID, role, epoch,
 *                   the peer table (endpoints only)
 *   0x0800 credits  u32 per peer: messages of ours that peer consumed
 *   0x1000 rx rings one ring per sender (index = sender node ID),
 *                   nslots slots of slot_size bytes
 *   rma_off         one-sided write window, up to the end
 *
 * Slot: seq (u32, written last), len | type << 16, src | epoch << 16,
 * reserved, payload. A sender's n-th message to a receiver (1-based)
 * goes to slot (n - 1) % nslots with seq = n; the receiver takes it when
 * seq equals its count + 1 and returns a credit.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_FABRIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_FABRIC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Wire format                                                               */

#define PFAB_MAGIC      0x42414650U /* "PFAB" */
#define PFAB_CARD_MAGIC 0x44524143U /* "CARD" */
#define PFAB_VERSION    0x00010000U /* 1.0 */

/* info */
#define PFAB_INFO_MAGIC    0x00
#define PFAB_INFO_VERSION  0x04
#define PFAB_INFO_SIZE     0x08 /* region size */
#define PFAB_INFO_STATE    0x0c
#define PFAB_INFO_DSN      0x10 /* lo, hi */
#define PFAB_INFO_CAPS     0x18
#define PFAB_INFO_ROLES    0x1c /* roles offered, bit n = role n */
#define PFAB_INFO_GEOM     0x20 /* max_nodes | nslots << 16 */
#define PFAB_INFO_SLOT     0x24
#define PFAB_INFO_RX_OFF   0x28
#define PFAB_INFO_CRED_OFF 0x2c
#define PFAB_INFO_RMA_OFF  0x30
#define PFAB_INFO_RMA_SIZE 0x34
#define PFAB_INFO_EPOCH    0x38 /* card accepted */
#define PFAB_INFO_NODE     0x3c /* node ID | role << 16 */
#define PFAB_INFO_FW       0x40 /* 32 bytes, NUL padded */
#define PFAB_INFO_BOARD    0x60 /* 32 bytes, NUL padded */
#define PFAB_INFO_ERROR    0x80 /* why a card was refused (-errno) */
#define PFAB_STR_LEN       32

#define PFAB_STATE_INIT    0
#define PFAB_STATE_READY   1 /* waiting for a card */
#define PFAB_STATE_RUNNING 2
#define PFAB_STATE_ERROR   3

#define PFAB_CAP_MSG BIT(0)
#define PFAB_CAP_RMA BIT(1)
#define PFAB_CAP_DMA BIT(2)
#define PFAB_CAP_MSI BIT(3)
#define PFAB_CAP_MSIX BIT(4)

/* card */
#define PFAB_CARD_MAGIC_OFF 0x100
#define PFAB_CARD_EPOCH     0x104
#define PFAB_CARD_COMMIT    0x108 /* = epoch, written last; 0 = withdrawn */
#define PFAB_CARD_NODE      0x10c /* node ID | role << 16 */
#define PFAB_CARD_NPEERS    0x110
#define PFAB_CARD_PEERS     0x200
#define PFAB_PEER_SIZE      0x40
#define PFAB_MAX_PEERS      16

/* peer entry, relative to the entry */
#define PFAB_PEER_NODE     0x00 /* node ID | role << 16 */
#define PFAB_PEER_FLAGS    0x04
#define PFAB_PEER_DSN      0x08
#define PFAB_PEER_BUS      0x10 /* bus address of the peer's region */
#define PFAB_PEER_SIZE_OFF 0x18
#define PFAB_PEER_GEOM     0x1c
#define PFAB_PEER_SLOT     0x20
#define PFAB_PEER_RX_OFF   0x24
#define PFAB_PEER_CRED_OFF 0x28
#define PFAB_PEER_RMA_OFF  0x2c
#define PFAB_PEER_RMA_SIZE 0x30
#define PFAB_PEER_VALID    BIT(0)

#define PFAB_CREDITS 0x800
#define PFAB_RINGS   0x1000

/* slot */
#define PFAB_SLOT_SEQ   0x00
#define PFAB_SLOT_LEN   0x04 /* len | type << 16 */
#define PFAB_SLOT_SRC   0x08 /* src | (epoch & 0xffff) << 16 */
#define PFAB_SLOT_HDR   0x10

/* message types below 0x100 belong to the fabric itself */
#define PFAB_TYPE_ECHO_REQ 0x0001
#define PFAB_TYPE_ECHO_REP 0x0002
#define PFAB_TYPE_USER     0x0100

/* roles: free numbers, these have names in the shell */
#define PFAB_ROLE_NONE    0
#define PFAB_ROLE_MANAGER 1
#define PFAB_ROLE_COMPUTE 2
#define PFAB_ROLE_STORAGE 3
#define PFAB_ROLE_IO      4

/* ------------------------------------------------------------------------ */
/* Node                                                                      */

#ifndef CONFIG_PCIE_FABRIC_MAX_NODES
#define CONFIG_PCIE_FABRIC_MAX_NODES 8
#endif
#ifndef CONFIG_PCIE_FABRIC_SLOT_SIZE
#define CONFIG_PCIE_FABRIC_SLOT_SIZE 256
#endif
#ifndef CONFIG_PCIE_FABRIC_RXQ_DEPTH
#define CONFIG_PCIE_FABRIC_RXQ_DEPTH 8
#endif

#define PFAB_PAYLOAD_MAX (CONFIG_PCIE_FABRIC_SLOT_SIZE - PFAB_SLOT_HDR)

/** Geometry of a region */
struct pfab_geom {
	uint32_t size;
	uint16_t max_nodes;
	uint16_t nslots;
	uint32_t slot_size;
	uint32_t rx_off;
	uint32_t cred_off;
	uint32_t rma_off;
	uint32_t rma_size;
};

/** Another node as this one sees it */
struct pfab_peer {
	bool valid;
	uint16_t id;
	uint16_t role;
	uint64_t dsn;
	uint64_t bus;
	struct pfab_geom g;
	volatile uint8_t *base; /**< CPU view of the peer's region */
	uint32_t sent;		/**< messages sent to it this epoch */
	void *map;		/**< owner's mapping handle */
};

/** A received message */
struct pfab_msg {
	uint16_t src;
	uint16_t type;
	uint16_t len;
	uint8_t data[PFAB_PAYLOAD_MAX] __aligned(4);
};

struct pfab_stats {
	uint32_t tx, rx, tx_full, rx_stale, rx_dropped, echo, puts, cards;
};

/** Mapping of a peer region for pfab_card_accept() */
struct pfab_map_ops {
	int (*map)(void *ctx, uint64_t bus, size_t size, volatile uint8_t **cpu, void **handle);
	void (*unmap)(void *ctx, void *handle);
	void *ctx;
};

struct pfab_node {
	const char *name;
	volatile uint8_t *local; /**< CPU view of our region */
	struct pfab_geom g;
	uint16_t id;
	uint16_t role;
	uint32_t epoch;
	bool running;
	/** a shell command (ping) polls the rings itself */
	atomic_t exclusive;
	uint32_t poll_us;
	struct pfab_peer peer[CONFIG_PCIE_FABRIC_MAX_NODES];
	uint32_t rx_count[CONFIG_PCIE_FABRIC_MAX_NODES];
	uint16_t rr;
	struct k_mutex lock;
	struct k_msgq rxq;
	char rxq_buf[CONFIG_PCIE_FABRIC_RXQ_DEPTH * sizeof(struct pfab_msg)] __aligned(4);
	struct pfab_stats st;
	sys_snode_t reg;
};

/** Layout for a region of @p size bytes (Kconfig geometry) */
int pfab_geom_default(struct pfab_geom *g, size_t size);

/** Geometry is self-consistent and fits the region */
bool pfab_geom_valid(const struct pfab_geom *g);

/**
 * Initialise a region: clear it, write the info block, state READY.
 * @p fw and @p board may be NULL.
 */
void pfab_region_init(volatile uint8_t *r, const struct pfab_geom *g, uint64_t dsn,
		      uint32_t caps, uint32_t roles, const char *fw, const char *board);

/** Read the geometry from a region's info block; -EINVAL on a bad magic or layout */
int pfab_region_geom(volatile const uint8_t *r, struct pfab_geom *g);

void pfab_node_init(struct pfab_node *n, const char *name, volatile uint8_t *local,
		    const struct pfab_geom *g);

/** Make the node known to the shell */
void pfab_node_register(struct pfab_node *n);
struct pfab_node *pfab_node_get(int i);

/**
 * Card state of an endpoint region: 1 when a new committed card waits,
 * 0 when there is nothing new, -ECANCELED when the card was withdrawn
 * (commit 0) while running.
 */
int pfab_card_pending(struct pfab_node *n);

/**
 * Take the committed card: peers mapped through @p ops, counters reset,
 * our rings and credits cleared, info updated (epoch, node, RUNNING).
 * On a bad card the info shows ERROR with the reason.
 */
int pfab_card_accept(struct pfab_node *n, const struct pfab_map_ops *ops);

/** Drop the peers (unmapped through @p ops), state READY */
void pfab_node_stop(struct pfab_node *n, const struct pfab_map_ops *ops);

/**
 * Start without a card (the root complex's own node): identity and peers
 * given directly; peer[i].base must be set for valid peers.
 */
void pfab_node_start(struct pfab_node *n, uint16_t id, uint16_t role, uint32_t epoch,
		     const struct pfab_peer *peers, int npeers);

/** Send a message: -EAGAIN no credit, -ENOTCONN no such peer, -EMSGSIZE */
int pfab_send(struct pfab_node *n, uint16_t dst, uint16_t type, const void *data, size_t len);

/** Take one message from any sender (round robin): 1 taken, 0 none */
int pfab_poll(struct pfab_node *n, struct pfab_msg *m);

/** Write into a peer's RMA window */
int pfab_put(struct pfab_node *n, uint16_t dst, uint32_t off, const void *data, size_t len);

/** Our own RMA window (what peers put into) */
volatile uint8_t *pfab_rma_local(struct pfab_node *n, size_t *size);

/**
 * One service step for the node's thread: answers echo requests, queues
 * everything else for pfab_recv(). Returns the messages handled.
 */
int pfab_service(struct pfab_node *n);

/**
 * Wait between two service steps: poll_us, or (poll_us 0) yield at the
 * lowest application priority, so busy polling never starves the rest.
 */
void pfab_service_wait(struct pfab_node *n);

/** A queued message (from pfab_service()) */
int pfab_recv(struct pfab_node *n, struct pfab_msg *m, k_timeout_t timeout);

/** Round trip of @p count echoes of @p len bytes to @p dst, in ns */
struct pfab_ping_result {
	uint32_t sent, received;
	uint64_t min_ns, max_ns, sum_ns;
};
int pfab_ping(struct pfab_node *n, uint16_t dst, uint32_t count, size_t len,
	      k_timeout_t timeout, struct pfab_ping_result *r);

const char *pfab_role_name(uint16_t role);
const char *pfab_state_name(uint32_t state);

/** Little helpers for region access (32-bit, as BARs need) */
static inline uint32_t pfab_rd(volatile const uint8_t *b, uint32_t off)
{
	return *(volatile const uint32_t *)(b + off);
}

static inline void pfab_wr(volatile uint8_t *b, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(b + off) = v;
}

static inline uint64_t pfab_rd64(volatile const uint8_t *b, uint32_t off)
{
	return ((uint64_t)pfab_rd(b, off + 4) << 32) | pfab_rd(b, off);
}

static inline void pfab_wr64(volatile uint8_t *b, uint32_t off, uint64_t v)
{
	pfab_wr(b, off, (uint32_t)v);
	pfab_wr(b, off + 4, (uint32_t)(v >> 32));
}

/* ------------------------------------------------------------------------ */
/* Root complex: fabric manager                                              */

/** An endpoint the manager found */
struct pfab_mgr_ent {
	uint32_t bdf;
	uint64_t bus;
	volatile uint8_t *cpu;
	uint64_t dsn;
	uint32_t roles;
	uint32_t caps;
	struct pfab_geom g;
	uint16_t id;
	uint16_t role;
	uint32_t state;
	char fw[PFAB_STR_LEN + 1];
	char board[PFAB_STR_LEN + 1];
};

/** Find the fabric endpoints below @p root (BAR0 with the PFAB magic) */
int pfab_mgr_scan(const struct device *ctrl, uint32_t root);

/**
 * Hand out the cards: node IDs (0 = the root complex's node when
 * enabled), roles, peers; switch ports set for direct P2P (ACS
 * redirects off), bus mastering on. Waits until every endpoint runs
 * the new epoch. Returns the number of endpoints running.
 */
int pfab_mgr_up(const struct device *ctrl, uint32_t root, k_timeout_t timeout);

/** Withdraw the cards (endpoints stop) */
void pfab_mgr_down(void);

const struct pfab_mgr_ent *pfab_mgr_get(int i);
uint32_t pfab_mgr_epoch(void);

/** Role for the endpoint with this DSN at the next pfab_mgr_up() */
int pfab_mgr_role_set(uint64_t dsn, uint16_t role);

/** The root complex's own node (NULL when not enabled) */
struct pfab_node *pfab_mgr_node(void);

/** A controller enumerated again: bring the fabric up (CONFIG_PCIE_FABRIC_MGR_AUTO) */
void pfab_mgr_rescanned(const struct device *ctrl);

/** Map a region found behind a BAR (weak: k_mem_map_phys_bare uncached) */
volatile uint8_t *pfab_mgr_map(uintptr_t phys, size_t size);
void pfab_mgr_unmap(volatile uint8_t *virt, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_FABRIC_H_ */
