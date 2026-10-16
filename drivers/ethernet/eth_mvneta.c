/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell NETA gigabit Ethernet controller (Armada 3700 variant).
 *
 * Up to 8 RX and 8 TX queues, software buffer management, legacy parser.
 * TX queues are picked by packet priority and served in strict priority
 * order, each with an optional token bucket rate limit (exposed as Qav).
 * RX queues are picked by the VLAN PCP. Descriptor rings and packet
 * buffers live in non-cacheable memory and frames are copied to/from
 * net_pkt, so no cache maintenance is needed.
 *
 * The Armada 3700 variant has no MBUS decoding windows to program: a single
 * 4 GiB window bypasses the GbE address decoder. Its interrupt is a single
 * shared (not per-CPU) line.
 *
 * Register usage follows Linux drivers/net/ethernet/marvell/mvneta.c and
 * U-Boot drivers/net/mvneta.c.
 */

#define DT_DRV_COMPAT marvell_armada_3700_neta

#include <zephyr/kernel.h>
#include <zephyr/kernel/internal/mm.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/phy.h>
#include <ethernet/eth_stats.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_mvneta, CONFIG_ETHERNET_LOG_LEVEL);

#include "eth.h"

/* Queue registers, q = hardware queue 0..7 */
#define RXQ_CONFIG(q)         (0x1400 + ((q) << 2))
#define RXQ_HW_BUF_ALLOC      BIT(0)
#define RXQ_PKT_OFFSET_MASK   GENMASK(11, 8)
#define RXQ_BASE_ADDR(q)      (0x1480 + ((q) << 2))
#define RXQ_SIZE(q)           (0x14a0 + ((q) << 2))
#define RXQ_BUF_SIZE_SHIFT    19
#define RXQ_THRESHOLD(q)      (0x14c0 + ((q) << 2))
#define RXQ_STATUS(q)         (0x14e0 + ((q) << 2))
#define RXQ_OCCUPIED_MASK     GENMASK(13, 0)
#define RXQ_STATUS_UPDATE(q)  (0x1500 + ((q) << 2))
#define RXQ_ADD_NON_OCC_SHIFT 16
#define PORT_RX_RESET         0x1cc0
#define MBUS_RETRY            0x2010
#define UNIT_INTR_CAUSE       0x2080
#define UNIT_CONTROL          0x20b0
#define PHY_POLLING_ENABLE    BIT(1)
#define WIN_SIZE(w)           (0x2204 + ((w) << 3))
#define BASE_ADDR_ENABLE      0x2290
#define ACCESS_PROTECT_ENABLE 0x2294
#define PORT_CONFIG           0x2400
#define TX_UNSET_ERR_SUM      BIT(12)
#define RX_CSUM_WITH_PSEUDO   BIT(25)
#define PORT_CONFIG_EXTEND    0x2404
#define MAC_ADDR_LOW          0x2414
#define MAC_ADDR_HIGH         0x2418
#define SDMA_CONFIG           0x241c
#define SDMA_BRST_SIZE_16     4
#define RX_BRST_SZ(b)         ((b) << 1)
#define RX_NO_DATA_SWAP       BIT(4)
#define TX_NO_DATA_SWAP       BIT(5)
#define TX_BRST_SZ(b)         ((b) << 22)
#define VLAN_PRIO_TO_RXQ      0x2440
#define VLAN_PRIO_RXQ(p, q)   ((q) << ((p) * 3))
#define TXQ_CMD               0x2448
#define TXQ_DISABLE_SHIFT     8
#define RX_MIN_FRAME_SIZE     0x247c
#define TXQ_CMD_1             0x24e4
#define ACC_MODE              0x2500
#define ACC_MODE_EXT1         1
#define CPU_MAP(cpu)          (0x2540 + ((cpu) << 2))
#define CPU_RXQ_ACCESS_ALL    0x000000ff
#define CPU_TXQ_ACCESS_ALL    0x0000ff00
#define RXQ_TIME_COAL(q)      (0x2580 + ((q) << 2))
#define INTR_NEW_CAUSE        0x25a0
#define INTR_NEW_MASK         0x25a4
#define INTR_RXQ_SHIFT        8
#define INTR_MISC             BIT(31)
#define INTR_OLD_CAUSE        0x25a8
#define INTR_OLD_MASK         0x25ac
#define INTR_MISC_CAUSE       0x25b0
#define INTR_MISC_MASK        0x25b4
#define INTR_ENABLE           0x25b8
#define TXQ_INTR_ENABLE_ALL   0x0000ff00
#define RXQ_INTR_ENABLE_ALL   0x000000ff
#define RXQ_CMD               0x2680
#define RXQ_DISABLE_SHIFT     8
#define TXQ_TOKEN_COUNT(q)    (0x2700 + ((q) << 4))
#define TXQ_TOKEN_CFG(q)      (0x2704 + ((q) << 4))
#define GMAC_CTRL_0           0x2c00
#define GMAC0_PORT_ENABLE     BIT(0)
#define GMAC0_1000BASE_X      BIT(1)
#define GMAC0_MAX_RX_SHIFT    2
#define GMAC0_MAX_RX_MASK     GENMASK(14, 2)
#define GMAC_CTRL_2           0x2c08
#define GMAC2_INBAND_AN       BIT(0)
#define GMAC2_PCS_ENABLE      BIT(3)
#define GMAC2_PORT_RGMII      BIT(4)
#define GMAC2_PORT_RESET      BIT(6)
#define GMAC_AUTONEG_CONFIG   0x2c0c
#define AN_FORCE_LINK_DOWN    BIT(0)
#define AN_FORCE_LINK_PASS    BIT(1)
#define AN_INBAND_AN_ENABLE   BIT(2)
#define AN_CONFIG_MII_SPEED   BIT(5)
#define AN_CONFIG_GMII_SPEED  BIT(6)
#define AN_SPEED_EN           BIT(7)
#define AN_CONFIG_FLOW_CTRL   BIT(8)
#define AN_FLOW_CTRL_EN       BIT(11)
#define AN_CONFIG_FULL_DUPLEX BIT(12)
#define AN_DUPLEX_EN          BIT(13)
#define LPI_CTRL_1            0x2cc4
#define LPI_REQUEST_ENABLE    BIT(0)
#define LPI_REQUEST_FORCE     BIT(1)
#define LPI_MANUAL_MODE       BIT(2)
#define GMAC_STATUS           0x2c10
#define GMAC_LINK_UP          BIT(0)
#define DA_FILT_SPEC_MCAST    0x3400
#define DA_FILT_OTH_MCAST     0x3500
#define DA_FILT_UCAST_BASE    0x3600
#define TXQ_BASE_ADDR(q)      (0x3c00 + ((q) << 2))
#define TXQ_SIZE(q)           (0x3c20 + ((q) << 2))
#define TXQ_STATUS(q)         (0x3c40 + ((q) << 2))
#define TXQ_SENT_DESC_MASK    GENMASK(29, 16)
#define TXQ_UPDATE(q)         (0x3c60 + ((q) << 2))
#define TXQ_DEC_SENT_SHIFT    16
#define PORT_TX_RESET         0x3cf0

/* TX rate limiting (token buckets), Linux mvneta_setup_queue_rates() */
#define TXQ_CMD1             0x3e00
#define TXQ_CMD1_BW_LIM_EN   BIT(0)
#define TXQ_CMD1_BW_LIM_V1   BIT(3)
#define REFILL_NUM_CLK       0x3e08
#define REFILL_MAX_NUM_CLK   0xffff
#define TX_MTU               0x3e0c
#define TX_MTU_MAX           0x3ffff
#define TX_TOKEN_SIZE        0x3e14
#define TX_TOKEN_SIZE_MAX    0xffffffff
#define TXQ_BUCKET_REFILL(q) (0x3e20 + ((q) << 2))
#define REFILL_PERIOD_SHIFT  20
#define REFILL_VALUE_MAX     0x0007ffff
#define TXQ_TOKEN_SIZE(q)    (0x3e40 + ((q) << 2))
#define TXQ_TOKEN_SIZE_MAX   0x7fffffff

/* Refill every 1000 base periods of 100 ns: 10 kb/s per refill unit */
#define REFILL_BASE_PERIOD_NS 100
#define REFILL_PERIOD         1000
#define RATE_RESOLUTION_BPS   (NSEC_PER_SEC / (REFILL_BASE_PERIOD_NS * REFILL_PERIOD))

/* Descriptors (little endian layout) */
#define TXD_L4_CSUM_NOT BIT(31)
#define TXD_F_DESC      BIT(21)
#define TXD_L_DESC      BIT(20)
#define TXD_Z_PAD       BIT(19)

#define RXD_FIRST_DESC  BIT(27)
#define RXD_LAST_DESC   BIT(26)
#define RXD_ERR_SUMMARY BIT(16)

struct mvneta_tx_desc {
	uint32_t command;
	uint16_t reserved1;
	uint16_t data_size;
	uint32_t buf_phys_addr;
	uint32_t reserved2;
	uint32_t reserved3[4];
};

struct mvneta_rx_desc {
	uint32_t status;
	uint16_t reserved1;
	uint16_t data_size;
	uint32_t buf_phys_addr;
	uint32_t reserved2;
	uint32_t buf_cookie;
	uint16_t reserved3;
	uint16_t reserved4;
	uint32_t reserved5;
	uint32_t reserved6;
};

BUILD_ASSERT(sizeof(struct mvneta_tx_desc) == 32);
BUILD_ASSERT(sizeof(struct mvneta_rx_desc) == 32);

/* Marvell header prepended by the MAC on RX, CRC kept at the end */
#define MVNETA_MH_SIZE     2
#define MVNETA_FCS_LEN     4
/* MTU + MH + VLAN + Ethernet header + FCS, multiple of 8 */
#define MVNETA_MAX_RX_SIZE ROUND_UP(NET_ETH_MTU + MVNETA_MH_SIZE + 4 + 14 + 4, 8)
#define MVNETA_BUF_SIZE    ROUND_UP(MVNETA_MAX_RX_SIZE, 64)

#define RX_DESCS CONFIG_ETH_MVNETA_RX_DESCS
#define TX_DESCS CONFIG_ETH_MVNETA_TX_DESCS
#define NUM_RXQ  CONFIG_ETH_MVNETA_RX_QUEUES
#define NUM_TXQ  CONFIG_ETH_MVNETA_TX_QUEUES

#define RXQ_MASK     BIT_MASK(NUM_RXQ)
#define INTR_RX_MASK (RXQ_MASK << INTR_RXQ_SHIFT)

/*
 * Logical queue numbers go from 0 (lowest priority) up. In strict priority
 * mode (legacy WRR disabled, TXQ_CMD_1 = 0) the arbiter serves the
 * highest-priority hardware queue first; which end that is can be selected
 * with CONFIG_ETH_MVNETA_TXQ0_HIGHEST.
 */
#if defined(CONFIG_ETH_MVNETA_TXQ0_HIGHEST)
#define TXQ_HW(q) (NUM_TXQ - 1 - (q))
#else
#define TXQ_HW(q) (q)
#endif

struct mvneta_rxq_dma {
	struct mvneta_rx_desc desc[RX_DESCS];
	uint8_t buf[RX_DESCS][MVNETA_BUF_SIZE];
};

struct mvneta_txq_dma {
	struct mvneta_tx_desc desc[TX_DESCS];
	uint8_t buf[TX_DESCS][MVNETA_BUF_SIZE];
};

struct mvneta_dma {
	struct mvneta_rxq_dma rxq[NUM_RXQ];
	struct mvneta_txq_dma txq[NUM_TXQ];
};

struct mvneta_txq {
	struct k_mutex lock;
	uint16_t idx;
	uint16_t inflight;
	/* Qav emulation with the per-queue token bucket */
	bool qav_enabled;
	uint32_t idle_slope; /* bit/s */
};

struct mvneta_clk {
	const struct device *dev;
	clock_control_subsys_t id;
};

struct mvneta_config {
	DEVICE_MMIO_ROM;
	const struct device *phy_dev;
	const struct pinctrl_dev_config *pcfg;
	const struct mvneta_clk *clks;
	uint8_t num_clks;
	bool rgmii;
	struct net_eth_mac_config mac_cfg;
	void (*irq_config)(void);
	unsigned int irq;
};

struct mvneta_data {
	DEVICE_MMIO_RAM;
	struct net_if *iface;
	struct mvneta_dma *dma;
	uint8_t mac_addr[NET_ETH_ADDR_LEN];
	uint16_t rx_idx[NUM_RXQ];
	struct mvneta_txq txq[NUM_TXQ];
	uint32_t core_hz; /* GbE core clock, 0 if unknown (no shaping) */
	uint32_t link_mbps;
	uint32_t txq_cmd1_reset; /* limiter state as left by reset */
	bool link_up;
	bool started;
	bool shaper_on;
	struct k_mutex shaper_lock;
	struct k_sem rx_sem;
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_MVNETA_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

static inline uint32_t neta_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_GET(dev) + reg);
}

static inline void neta_write(const struct device *dev, uint32_t reg, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_GET(dev) + reg);
}

static inline void neta_rmw(const struct device *dev, uint32_t reg, uint32_t clr, uint32_t set)
{
	neta_write(dev, reg, (neta_read(dev, reg) & ~clr) | set);
}

static inline uint32_t dma_addr(const void *p)
{
	uintptr_t pa = k_mem_phys_addr((void *)p);

	__ASSERT(pa <= UINT32_MAX, "DMA address above 4 GiB");
	return (uint32_t)pa;
}

/* ---------------------------------------------------------------------- */
/* Priority to queue mapping                                               */

/* 802.1Q order of the Zephyr priorities: BK (1) < BE (0) < EE (2) < ... */
static inline unsigned int prio_rank(enum net_priority prio)
{
	switch (prio) {
	case NET_PRIORITY_BK:
		return 0;
	case NET_PRIORITY_BE:
		return 1;
	default:
		return MIN((unsigned int)prio, NET_MAX_PRIORITIES - 1);
	}
}

static unsigned int mvneta_tx_queue(enum net_priority prio)
{
#if NUM_TXQ == 1
	ARG_UNUSED(prio);
	return 0;
#elif CONFIG_NET_TC_TX_COUNT == NUM_TXQ
	/* One traffic class per queue, same mapping as the stack */
	return net_tx_priority2tc(prio);
#else
	return prio_rank(prio) * NUM_TXQ / NET_MAX_PRIORITIES;
#endif
}

static unsigned int mvneta_rx_queue(enum net_priority prio)
{
#if NUM_RXQ == 1
	ARG_UNUSED(prio);
	return 0;
#elif CONFIG_NET_TC_RX_COUNT == NUM_RXQ
	return net_rx_priority2tc(prio);
#else
	return prio_rank(prio) * NUM_RXQ / NET_MAX_PRIORITIES;
#endif
}

/* ---------------------------------------------------------------------- */
/* Address filtering                                                       */

static void mvneta_set_mac_addr(const struct device *dev, const uint8_t *a)
{
	uint32_t ucast[4] = {0};
	uint8_t nibble = a[5] & 0xf;

	neta_write(dev, MAC_ADDR_LOW, (a[4] << 8) | a[5]);
	neta_write(dev, MAC_ADDR_HIGH, (a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3]);

	/* Unicast table is indexed by the last nibble: accept, queue 0 */
	ucast[nibble / 4] = BIT(0) << ((nibble % 4) * 8);
	for (int i = 0; i < 4; i++) {
		neta_write(dev, DA_FILT_UCAST_BASE + 4 * i, ucast[i]);
	}
}

static void mvneta_set_mcast_filter(const struct device *dev, bool accept)
{
	/* Each byte: bit0 accept, bits 3:1 queue. IPv6 ND needs multicast. */
	uint32_t val = accept ? 0x01010101 : 0;

	for (uint32_t off = 0; off <= 0xfc; off += 4) {
		neta_write(dev, DA_FILT_SPEC_MCAST + off, val);
		neta_write(dev, DA_FILT_OTH_MCAST + off, val);
	}
}

/* VLAN tagged frames go to the queue of their PCP, untagged ones to 0 */
static void mvneta_set_rx_prio_map(const struct device *dev)
{
	uint32_t map = 0;

	for (uint8_t pcp = 0; pcp < 8; pcp++) {
		map |= VLAN_PRIO_RXQ(pcp, mvneta_rx_queue(net_vlan2priority(pcp)));
	}
	neta_write(dev, VLAN_PRIO_TO_RXQ, map);
}

/* ---------------------------------------------------------------------- */
/* RX                                                                      */

static void mvneta_rxq_update(const struct device *dev, unsigned int q, uint32_t done,
			      uint32_t filled)
{
	while (done > 0 || filled > 0) {
		uint32_t d = MIN(done, 0xff);
		uint32_t f = MIN(filled, 0xff);

		neta_write(dev, RXQ_STATUS_UPDATE(q), d | (f << RXQ_ADD_NON_OCC_SHIFT));
		done -= d;
		filled -= f;
	}
}

static void mvneta_rx_desc_fill(struct mvneta_rxq_dma *rq, uint16_t idx)
{
	struct mvneta_rx_desc *d = &rq->desc[idx];

	d->status = 0;
	d->data_size = 0;
	d->buf_phys_addr = dma_addr(rq->buf[idx]);
	d->buf_cookie = idx;
}

static void mvneta_rxq_init(const struct device *dev, unsigned int q)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_rxq_dma *rq = &data->dma->rxq[q];

	for (uint16_t i = 0; i < RX_DESCS; i++) {
		mvneta_rx_desc_fill(rq, i);
	}
	data->rx_idx[q] = 0;
	barrier_dmem_fence_full();

	neta_write(dev, RXQ_BASE_ADDR(q), dma_addr(rq->desc));
	neta_write(dev, RXQ_SIZE(q), RX_DESCS | ((MVNETA_BUF_SIZE >> 3) << RXQ_BUF_SIZE_SHIFT));

	/* Interrupt on every received frame */
	neta_write(dev, RXQ_THRESHOLD(q), 1);
	neta_write(dev, RXQ_TIME_COAL(q), CONFIG_ETH_MVNETA_RX_COAL_CYCLES);

	/* No packet offset, software buffer management */
	neta_rmw(dev, RXQ_CONFIG(q), RXQ_PKT_OFFSET_MASK | RXQ_HW_BUF_ALLOC, 0);

	mvneta_rxq_update(dev, q, 0, RX_DESCS);
}

static void mvneta_rx_one(const struct device *dev, struct mvneta_rxq_dma *rq, uint16_t idx)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_rx_desc *d = &rq->desc[idx];
	uint32_t status = d->status;
	uint16_t size = d->data_size;
	const uint8_t *frame = rq->buf[idx] + MVNETA_MH_SIZE;
	struct net_pkt *pkt;
	int len;

	if ((status & (RXD_FIRST_DESC | RXD_LAST_DESC)) != (RXD_FIRST_DESC | RXD_LAST_DESC) ||
	    (status & RXD_ERR_SUMMARY) != 0) {
		LOG_DBG("rx drop status 0x%08x size %u", status, size);
		eth_stats_update_errors_rx(data->iface);
		return;
	}

	len = (int)size - MVNETA_MH_SIZE - MVNETA_FCS_LEN;
	if (len <= 0 || len > MVNETA_MAX_RX_SIZE) {
		eth_stats_update_errors_rx(data->iface);
		return;
	}

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, len, NET_AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		eth_stats_update_errors_rx(data->iface);
		return;
	}

	/* Pick the RX traffic class from the PCP before the stack queues it */
	if (len >= 16 && sys_get_be16(&frame[12]) == NET_ETH_PTYPE_VLAN) {
		net_pkt_set_priority(pkt, net_vlan2priority(frame[14] >> 5));
	}

	if (net_pkt_write(pkt, frame, len) < 0 || net_recv_data(data->iface, pkt) < 0) {
		net_pkt_unref(pkt);
		eth_stats_update_errors_rx(data->iface);
	}
}

static int mvneta_rx(const struct device *dev, unsigned int q, int budget)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_rxq_dma *rq = &data->dma->rxq[q];
	uint32_t occupied;
	uint32_t done = 0;

	occupied = neta_read(dev, RXQ_STATUS(q)) & RXQ_OCCUPIED_MASK;
	occupied = MIN(occupied, (uint32_t)budget);
	barrier_dmem_fence_full();

	while (done < occupied) {
		uint16_t idx = data->rx_idx[q];

		mvneta_rx_one(dev, rq, idx);
		mvneta_rx_desc_fill(rq, idx);

		data->rx_idx[q] = (idx + 1) % RX_DESCS;
		done++;
	}

	if (done > 0) {
		barrier_dmem_fence_full();
		mvneta_rxq_update(dev, q, done, done);
	}

	return done;
}

/*
 * Serve the queues from the highest one down. After a batch from a lower
 * queue, start again from the top, so a burst of best effort traffic
 * cannot hold back a frame waiting in a higher queue.
 */
static void mvneta_rx_all(const struct device *dev)
{
	bool again;

	do {
		again = false;
		for (int q = NUM_RXQ - 1; q >= 0; q--) {
			if (mvneta_rx(dev, q, CONFIG_ETH_MVNETA_RX_BUDGET) > 0) {
				again = true;
				break;
			}
		}
	} while (again);
}

static void mvneta_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct mvneta_data *data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		/* The timeout is a safety net against a lost interrupt */
		(void)k_sem_take(&data->rx_sem, K_MSEC(CONFIG_ETH_MVNETA_RX_POLL_MS));

		uint32_t cause = neta_read(dev, INTR_NEW_CAUSE);

		if (cause & INTR_MISC) {
			LOG_DBG("misc cause 0x%08x", neta_read(dev, INTR_MISC_CAUSE));
			neta_write(dev, INTR_MISC_CAUSE, 0);
		}

		if (data->iface != NULL && net_if_is_up(data->iface)) {
			mvneta_rx_all(dev);
		}

		neta_write(dev, INTR_NEW_MASK, INTR_RX_MASK | INTR_MISC);
	}
}

static void mvneta_isr(const struct device *dev)
{
	struct mvneta_data *data = dev->data;

	neta_write(dev, INTR_NEW_MASK, 0);
	k_sem_give(&data->rx_sem);
}

/* ---------------------------------------------------------------------- */
/* TX                                                                      */

static void mvneta_txq_init(const struct device *dev, unsigned int q)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_txq *tq = &data->txq[q];
	unsigned int hw = TXQ_HW(q);

	memset(data->dma->txq[q].desc, 0, sizeof(data->dma->txq[q].desc));
	k_mutex_init(&tq->lock);
	tq->idx = 0;
	tq->inflight = 0;

	neta_write(dev, TXQ_TOKEN_CFG(hw), 0x03ffffff);
	neta_write(dev, TXQ_TOKEN_COUNT(hw), 0x3fffffff);
	neta_write(dev, TXQ_BASE_ADDR(hw), dma_addr(data->dma->txq[q].desc));
	/* No TX done interrupts: completions are reclaimed in send() */
	neta_write(dev, TXQ_SIZE(hw), TX_DESCS);
}

static void mvneta_tx_reclaim(const struct device *dev, unsigned int q)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_txq *tq = &data->txq[q];
	unsigned int hw = TXQ_HW(q);
	uint32_t sent = FIELD_GET(TXQ_SENT_DESC_MASK, neta_read(dev, TXQ_STATUS(hw)));

	sent = MIN(sent, tq->inflight);
	tq->inflight -= sent;

	while (sent > 0) {
		uint32_t n = MIN(sent, 0xff);

		neta_write(dev, TXQ_UPDATE(hw), n << TXQ_DEC_SENT_SHIFT);
		sent -= n;
	}
}

static int mvneta_send(const struct device *dev, struct net_pkt *pkt)
{
	struct mvneta_data *data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	unsigned int q = mvneta_tx_queue(net_pkt_priority(pkt));
	struct mvneta_txq *tq = &data->txq[q];
	struct mvneta_txq_dma *tqd = &data->dma->txq[q];
	struct mvneta_tx_desc *d;
	k_timepoint_t deadline;
	uint16_t idx;
	int ret = 0;

	if (len > MVNETA_BUF_SIZE) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&tq->lock, K_FOREVER);

	/* A shaped queue may legitimately stay full for a while: sleep, don't spin */
	deadline = sys_timepoint_calc(K_MSEC(CONFIG_ETH_MVNETA_TX_TIMEOUT_MS));
	mvneta_tx_reclaim(dev, q);
	while (tq->inflight >= TX_DESCS) {
		if (sys_timepoint_expired(deadline)) {
			LOG_WRN("txq %u full", q);
			ret = -EBUSY;
			goto out;
		}
		k_usleep(20);
		mvneta_tx_reclaim(dev, q);
	}

	idx = tq->idx;
	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, tqd->buf[idx], len) < 0) {
		ret = -EIO;
		goto out;
	}

	d = &tqd->desc[idx];
	d->buf_phys_addr = dma_addr(tqd->buf[idx]);
	d->data_size = len;
	d->command = TXD_L4_CSUM_NOT | TXD_F_DESC | TXD_L_DESC | TXD_Z_PAD;
	barrier_dmem_fence_full();

	tq->idx = (idx + 1) % TX_DESCS;
	tq->inflight++;
	neta_write(dev, TXQ_UPDATE(TXQ_HW(q)), 1);

out:
	k_mutex_unlock(&tq->lock);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
	}
	return ret;
}

/* ---------------------------------------------------------------------- */
/* Per-queue shaping (Qav emulation)                                       */

/*
 * Each TX queue has a token bucket refilled every REFILL_PERIOD base periods
 * of 100 ns by "refill value" tokens, i.e. 10 kb/s per unit, up to about
 * 5 Gb/s. That is a rate limiter, not an 802.1Qav credit-based shaper
 * (credit does not go negative while sending, there is no send slope),
 * but with the bucket set to one maximum-size frame it bounds a stream to
 * its idle slope with at most one frame of burst, which is what Qav is used
 * for on an AVB talker.
 */

static uint32_t mvneta_tx_mtu_tokens(void)
{
	/* Linux mvneta_txq_max_tx_size_set(): TX_MTU = max frame * 8 */
	return MIN(MVNETA_MAX_RX_SIZE * 8, TX_MTU_MAX);
}

static void mvneta_shaper_apply(const struct device *dev)
{
	struct mvneta_data *data = dev->data;
	uint32_t mtu = mvneta_tx_mtu_tokens();
	bool any = false;

	for (unsigned int q = 0; q < NUM_TXQ; q++) {
		any |= data->txq[q].qav_enabled;
	}

	if (!any) {
		/*
		 * No reservation: give the limiter back its reset state, as
		 * Linux runs without mqprio. Setting BW_LIM_EN | BW_LIM_SEL_V1
		 * here stalled all TX queues on the hardware.
		 */
		if (data->shaper_on) {
			neta_write(dev, TXQ_CMD1, data->txq_cmd1_reset);
			data->shaper_on = false;
		}
		return;
	}

	for (unsigned int q = 0; q < NUM_TXQ; q++) {
		const struct mvneta_txq *tq = &data->txq[q];
		unsigned int hw = TXQ_HW(q);
		uint32_t refill = REFILL_VALUE_MAX;
		uint32_t bucket = TXQ_TOKEN_SIZE_MAX;

		if (tq->qav_enabled) {
			refill = DIV_ROUND_UP(tq->idle_slope, RATE_RESOLUTION_BPS);
			refill = CLAMP(refill, 1, REFILL_VALUE_MAX);
			bucket = mtu;
		}

		neta_write(dev, TXQ_BUCKET_REFILL(hw),
			   refill | (REFILL_PERIOD << REFILL_PERIOD_SHIFT));
		neta_write(dev, TXQ_TOKEN_SIZE(hw), bucket);
	}

	/* Linux mvneta_enable_per_queue_rate_limit(): both bits clear */
	neta_write(dev, TXQ_CMD1,
		   data->txq_cmd1_reset & ~(TXQ_CMD1_BW_LIM_EN | TXQ_CMD1_BW_LIM_V1));
	data->shaper_on = true;
}

static void mvneta_shaper_init(const struct device *dev)
{
	struct mvneta_data *data = dev->data;
	uint32_t mtu = mvneta_tx_mtu_tokens();
	uint32_t cycles;

	k_mutex_init(&data->shaper_lock);
	data->link_mbps = 1000;
	data->shaper_on = false;
	data->txq_cmd1_reset = neta_read(dev, TXQ_CMD1);

	/* As Linux mvneta_txq_max_tx_size_set(): MTU, token sizes >= MTU */
	neta_rmw(dev, TX_MTU, TX_MTU_MAX, mtu);
	if (neta_read(dev, TX_TOKEN_SIZE) < mtu) {
		neta_write(dev, TX_TOKEN_SIZE, mtu);
	}
	for (int q = 0; q < 8; q++) {
		if ((neta_read(dev, TXQ_TOKEN_SIZE(q)) & TXQ_TOKEN_SIZE_MAX) < mtu) {
			neta_write(dev, TXQ_TOKEN_SIZE(q), mtu);
		}
	}

	if (data->core_hz == 0) {
		return;
	}

	cycles = (uint32_t)((uint64_t)data->core_hz * REFILL_BASE_PERIOD_NS / NSEC_PER_SEC);
	if (cycles == 0 || cycles > REFILL_MAX_NUM_CLK) {
		LOG_WRN("core clock %u Hz unusable for shaping", data->core_hz);
		data->core_hz = 0;
		return;
	}
	neta_write(dev, REFILL_NUM_CLK, cycles);
}

static int mvneta_set_qav(const struct device *dev, const struct ethernet_qav_param *p)
{
	struct mvneta_data *data = dev->data;
	struct mvneta_txq *tq;
	int ret = 0;

	if (data->core_hz == 0) {
		return -ENOTSUP;
	}
	if (p->queue_id < 0 || p->queue_id >= NUM_TXQ) {
		return -EINVAL;
	}
	tq = &data->txq[p->queue_id];

	k_mutex_lock(&data->shaper_lock, K_FOREVER);

	switch (p->type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		tq->qav_enabled = p->enabled;
		break;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		if (p->idle_slope > (uint64_t)data->link_mbps * 1000000U) {
			ret = -EINVAL;
			break;
		}
		tq->idle_slope = p->idle_slope;
		break;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		if (p->delta_bandwidth > 100) {
			ret = -EINVAL;
			break;
		}
		tq->idle_slope = p->delta_bandwidth * data->link_mbps * 10000U;
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	if (ret == 0) {
		mvneta_shaper_apply(dev);
	}

	k_mutex_unlock(&data->shaper_lock);
	return ret;
}

static int mvneta_get_qav(const struct device *dev, struct ethernet_qav_param *p)
{
	struct mvneta_data *data = dev->data;
	const struct mvneta_txq *tq;

	if (data->core_hz == 0) {
		return -ENOTSUP;
	}
	if (p->queue_id < 0 || p->queue_id >= NUM_TXQ) {
		return -EINVAL;
	}
	tq = &data->txq[p->queue_id];

	switch (p->type) {
	case ETHERNET_QAV_PARAM_TYPE_STATUS:
		p->enabled = tq->qav_enabled;
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_IDLE_SLOPE:
		p->idle_slope = tq->idle_slope;
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_OPER_IDLE_SLOPE:
		/* What the bucket really delivers, in 10 kb/s steps */
		p->oper_idle_slope =
			tq->qav_enabled ? ROUND_UP(tq->idle_slope, RATE_RESOLUTION_BPS) : 0;
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_DELTA_BANDWIDTH:
		p->delta_bandwidth = DIV_ROUND_UP(tq->idle_slope, data->link_mbps * 10000U);
		return 0;
	case ETHERNET_QAV_PARAM_TYPE_TRAFFIC_CLASS:
#if CONFIG_NET_TC_TX_COUNT == NUM_TXQ
		p->traffic_class = p->queue_id;
		return 0;
#else
		return -ENOTSUP;
#endif
	default:
		return -ENOTSUP;
	}
}

/* ---------------------------------------------------------------------- */
/* Port control                                                            */

static void mvneta_bypass_mbus_windows(const struct device *dev)
{
	/* One 4 GiB window, decoding is left to the MBUS/AXI bridge */
	neta_write(dev, WIN_SIZE(0), 0xffff0000);
	neta_rmw(dev, BASE_ADDR_ENABLE, BIT(0), 0);
	neta_rmw(dev, ACCESS_PROTECT_ENABLE, 0, 0x3);
}

static void mvneta_defaults(const struct device *dev)
{
	neta_write(dev, INTR_NEW_CAUSE, 0);
	neta_write(dev, INTR_OLD_CAUSE, 0);
	neta_write(dev, INTR_MISC_CAUSE, 0);
	neta_write(dev, INTR_NEW_MASK, 0);
	neta_write(dev, INTR_OLD_MASK, 0);
	neta_write(dev, INTR_MISC_MASK, 0);
	neta_write(dev, INTR_ENABLE, 0);

	neta_write(dev, MBUS_RETRY, 0x20);

	/* Armada 3700: every CPU may access every queue */
	for (int cpu = 0; cpu < 2; cpu++) {
		neta_write(dev, CPU_MAP(cpu), CPU_RXQ_ACCESS_ALL | CPU_TXQ_ACCESS_ALL);
	}

	neta_write(dev, PORT_RX_RESET, BIT(0));
	neta_write(dev, PORT_TX_RESET, BIT(0));
	/* Legacy WRR off: strict priority between TX queues */
	neta_write(dev, TXQ_CMD_1, 0);
	for (int q = 0; q < 8; q++) {
		neta_write(dev, TXQ_TOKEN_COUNT(q), 0);
		neta_write(dev, TXQ_TOKEN_CFG(q), 0);
	}
	neta_write(dev, PORT_TX_RESET, 0);
	neta_write(dev, PORT_RX_RESET, 0);

	neta_write(dev, ACC_MODE, ACC_MODE_EXT1);

	/* Default RX queue 0 for untagged traffic of all types */
	neta_write(dev, PORT_CONFIG, TX_UNSET_ERR_SUM | RX_CSUM_WITH_PSEUDO);
	neta_write(dev, PORT_CONFIG_EXTEND, 0);
	neta_write(dev, RX_MIN_FRAME_SIZE, 64);

	neta_write(dev, SDMA_CONFIG,
		   TX_BRST_SZ(SDMA_BRST_SIZE_16) | RX_BRST_SZ(SDMA_BRST_SIZE_16) | RX_NO_DATA_SWAP |
			   TX_NO_DATA_SWAP);

	neta_rmw(dev, UNIT_CONTROL, PHY_POLLING_ENABLE, 0);

	/* No EEE: TX LPI requests off (Linux mvneta_mac_disable_tx_lpi()) */
	neta_rmw(dev, LPI_CTRL_1, LPI_REQUEST_ENABLE | LPI_REQUEST_FORCE | LPI_MANUAL_MODE, 0);

	mvneta_set_mcast_filter(dev, true);
	mvneta_set_rx_prio_map(dev);

	neta_write(dev, INTR_ENABLE, RXQ_INTR_ENABLE_ALL | TXQ_INTR_ENABLE_ALL);
}

static int mvneta_port_power_up(const struct device *dev)
{
	const struct mvneta_config *cfg = dev->config;
	uint32_t ctrl;

	neta_write(dev, UNIT_INTR_CAUSE, 0);

	ctrl = neta_read(dev, GMAC_CTRL_2);
	ctrl &= ~(GMAC2_INBAND_AN | GMAC2_PCS_ENABLE);
	if (cfg->rgmii) {
		ctrl |= GMAC2_PORT_RGMII;
	}
	ctrl &= ~GMAC2_PORT_RESET;
	neta_write(dev, GMAC_CTRL_2, ctrl);

	if (!WAIT_FOR((neta_read(dev, GMAC_CTRL_2) & GMAC2_PORT_RESET) == 0, 1000,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	/* Link is forced from the PHY callback, no in-band autoneg */
	neta_rmw(dev, GMAC_AUTONEG_CONFIG,
		 AN_INBAND_AN_ENABLE | AN_SPEED_EN | AN_DUPLEX_EN | AN_FLOW_CTRL_EN |
			 AN_FORCE_LINK_PASS,
		 AN_FORCE_LINK_DOWN);

	neta_rmw(dev, GMAC_CTRL_0, GMAC0_MAX_RX_MASK | GMAC0_1000BASE_X,
		 ((MVNETA_MAX_RX_SIZE - MVNETA_MH_SIZE) / 2) << GMAC0_MAX_RX_SHIFT);

	return 0;
}

static uint32_t mvneta_txq_hw_mask(void)
{
	uint32_t mask = 0;

	for (unsigned int q = 0; q < NUM_TXQ; q++) {
		mask |= BIT(TXQ_HW(q));
	}
	return mask;
}

/* Queues are enabled on link up and stopped on link down, as Linux
 * mvneta_port_up()/mvneta_port_down() do: TX queues enabled while the link
 * was forced down never started transmitting.
 */
static void mvneta_port_up(const struct device *dev)
{
	neta_write(dev, TXQ_CMD, mvneta_txq_hw_mask());
	neta_write(dev, RXQ_CMD, RXQ_MASK);
}

static void mvneta_port_down(const struct device *dev)
{
	neta_write(dev, RXQ_CMD, RXQ_MASK << RXQ_DISABLE_SHIFT);
	(void)WAIT_FOR((neta_read(dev, RXQ_CMD) & RXQ_MASK) == 0, 1000, k_busy_wait(10));
	neta_write(dev, TXQ_CMD, mvneta_txq_hw_mask() << TXQ_DISABLE_SHIFT);
	(void)WAIT_FOR((neta_read(dev, TXQ_CMD) & 0xff) == 0, 1000, k_busy_wait(10));
}

static void mvneta_link_cb(const struct device *phy, struct phy_link_state *state, void *user_data)
{
	const struct device *dev = user_data;
	struct mvneta_data *data = dev->data;
	uint32_t val;

	ARG_UNUSED(phy);

	val = neta_read(dev, GMAC_AUTONEG_CONFIG);
	val &= ~(AN_FORCE_LINK_DOWN | AN_FORCE_LINK_PASS | AN_CONFIG_MII_SPEED |
		 AN_CONFIG_GMII_SPEED | AN_CONFIG_FLOW_CTRL | AN_CONFIG_FULL_DUPLEX);

	if (state->is_up) {
		val |= AN_FORCE_LINK_PASS;
		if (PHY_LINK_IS_SPEED_1000M(state->speed)) {
			val |= AN_CONFIG_GMII_SPEED;
			data->link_mbps = 1000;
		} else if (PHY_LINK_IS_SPEED_100M(state->speed)) {
			val |= AN_CONFIG_MII_SPEED;
			data->link_mbps = 100;
		} else {
			data->link_mbps = 10;
		}
		if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
			val |= AN_CONFIG_FULL_DUPLEX;
		}
	} else {
		val |= AN_FORCE_LINK_DOWN;
	}
	if (!state->is_up) {
		mvneta_port_down(dev);
	}
	neta_write(dev, GMAC_AUTONEG_CONFIG, val);
	data->link_up = state->is_up;
	if (state->is_up && data->started) {
		mvneta_port_up(dev);
	}

	LOG_INF("%s: link %s", dev->name, state->is_up ? "up" : "down");

	if (data->iface != NULL) {
		if (state->is_up) {
			net_eth_carrier_on(data->iface);
		} else {
			net_eth_carrier_off(data->iface);
		}
	}
}

/* ---------------------------------------------------------------------- */
/* Ethernet API                                                            */

static int mvneta_start(const struct device *dev, struct net_if *iface)
{
	const struct mvneta_config *cfg = dev->config;

	ARG_UNUSED(iface);

	struct mvneta_data *data = dev->data;

	neta_rmw(dev, GMAC_CTRL_0, 0, GMAC0_PORT_ENABLE);
	data->started = true;
	if (data->link_up) {
		mvneta_port_up(dev);
	}
	neta_write(dev, INTR_NEW_MASK, INTR_RX_MASK | INTR_MISC);
	irq_enable(cfg->irq);

	return 0;
}

static int mvneta_stop(const struct device *dev, struct net_if *iface)
{
	const struct mvneta_config *cfg = dev->config;

	ARG_UNUSED(iface);

	struct mvneta_data *data = dev->data;

	irq_disable(cfg->irq);
	neta_write(dev, INTR_NEW_MASK, 0);
	data->started = false;
	mvneta_port_down(dev);
	neta_rmw(dev, GMAC_CTRL_0, GMAC0_PORT_ENABLE, 0);

	return 0;
}

static enum ethernet_hw_caps mvneta_get_caps(const struct device *dev, struct net_if *iface)
{
	struct mvneta_data *data = dev->data;
	enum ethernet_hw_caps caps =
		ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE;

	ARG_UNUSED(iface);

	if (IS_ENABLED(CONFIG_NET_VLAN)) {
		/* Tagged frames pass the MAC; PCP picks the RX queue */
		caps |= ETHERNET_HW_VLAN;
	}
	if (NUM_TXQ > 1) {
		caps |= ETHERNET_PRIORITY_QUEUES;
	}
	if (data->core_hz != 0) {
		caps |= ETHERNET_QAV;
	}

	return caps;
}

static int mvneta_set_config(const struct device *dev, struct net_if *iface,
			     enum ethernet_config_type type, const struct ethernet_config *config)
{
	struct mvneta_data *data = dev->data;

	ARG_UNUSED(iface);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, NET_ETH_ADDR_LEN);
		mvneta_set_mac_addr(dev, data->mac_addr);
		net_if_set_link_addr(data->iface, data->mac_addr, NET_ETH_ADDR_LEN,
				     NET_LINK_ETHERNET);
		return 0;
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return mvneta_set_qav(dev, &config->qav_param);
	default:
		return -ENOTSUP;
	}
}

static int mvneta_get_config(const struct device *dev, struct net_if *iface,
			     enum ethernet_config_type type, struct ethernet_config *config)
{
	ARG_UNUSED(iface);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PRIORITY_QUEUES_NUM:
		config->priority_queues_num = NUM_TXQ;
		return 0;
	case ETHERNET_CONFIG_TYPE_QAV_PARAM:
		return mvneta_get_qav(dev, &config->qav_param);
	default:
		return -ENOTSUP;
	}
}

static const struct device *mvneta_get_phy(const struct device *dev, struct net_if *iface)
{
	const struct mvneta_config *cfg = dev->config;

	ARG_UNUSED(iface);

	return cfg->phy_dev;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *mvneta_get_stats(const struct device *dev, struct net_if *iface)
{
	struct mvneta_data *data = dev->data;

	ARG_UNUSED(iface);

	return &data->stats;
}
#endif

static void mvneta_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct mvneta_config *cfg = dev->config;
	struct mvneta_data *data = dev->data;

	data->iface = iface;
	ethernet_init(iface);

	net_if_set_link_addr(iface, data->mac_addr, NET_ETH_ADDR_LEN, NET_LINK_ETHERNET);
	mvneta_set_mac_addr(dev, data->mac_addr);
	LOG_INF("%s: MAC %02x:%02x:%02x:%02x:%02x:%02x, %u RX / %u TX queues", dev->name,
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2], data->mac_addr[3],
		data->mac_addr[4], data->mac_addr[5], NUM_RXQ, NUM_TXQ);

	net_if_carrier_off(iface);

	if (cfg->phy_dev == NULL || !device_is_ready(cfg->phy_dev)) {
		LOG_ERR("%s: PHY not ready", dev->name);
		return;
	}

	phy_link_callback_set(cfg->phy_dev, mvneta_link_cb, (void *)dev);

	k_thread_create(&data->rx_thread, data->rx_stack, K_KERNEL_STACK_SIZEOF(data->rx_stack),
			mvneta_rx_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_MVNETA_RX_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, dev->name);
}

static const struct ethernet_api mvneta_api = {
	.iface_api.init = mvneta_iface_init,
	.get_capabilities = mvneta_get_caps,
	.get_phy = mvneta_get_phy,
	.set_config = mvneta_set_config,
	.get_config = mvneta_get_config,
	.start = mvneta_start,
	.stop = mvneta_stop,
	.send = mvneta_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = mvneta_get_stats,
#endif
};

static void mvneta_load_mac(const struct device *dev)
{
	const struct mvneta_config *cfg = dev->config;
	struct mvneta_data *data = dev->data;
	uint32_t hi, lo;

	if (net_eth_mac_load(&cfg->mac_cfg, data->mac_addr) == 0) {
		return;
	}

	/* Whatever an earlier boot stage left in the MAC */
	hi = neta_read(dev, MAC_ADDR_HIGH);
	lo = neta_read(dev, MAC_ADDR_LOW);
	data->mac_addr[0] = hi >> 24;
	data->mac_addr[1] = hi >> 16;
	data->mac_addr[2] = hi >> 8;
	data->mac_addr[3] = hi;
	data->mac_addr[4] = lo >> 8;
	data->mac_addr[5] = lo;

	if (!net_eth_is_addr_valid((struct net_eth_addr *)data->mac_addr)) {
		gen_random_mac(data->mac_addr, 0x00, 0x50, 0x43);
		LOG_WRN("%s: no MAC address configured, using a random one", dev->name);
	}
}

static int mvneta_init(const struct device *dev)
{
	const struct mvneta_config *cfg = dev->config;
	struct mvneta_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	k_sem_init(&data->rx_sem, 0, 1);

	for (int i = 0; i < cfg->num_clks; i++) {
		if (!device_is_ready(cfg->clks[i].dev)) {
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clks[i].dev, cfg->clks[i].id);
		if (ret < 0) {
			return ret;
		}
	}

	/* The first clock is the GbE core clock, the shaper time base */
	if (cfg->num_clks == 0 ||
	    clock_control_get_rate(cfg->clks[0].dev, cfg->clks[0].id, &data->core_hz) < 0) {
		data->core_hz = 0;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	neta_rmw(dev, GMAC_CTRL_0, GMAC0_PORT_ENABLE, 0);

	mvneta_bypass_mbus_windows(dev);
	mvneta_defaults(dev);

	ret = mvneta_port_power_up(dev);
	if (ret < 0) {
		LOG_ERR("%s: port reset timeout", dev->name);
		return ret;
	}

	for (unsigned int q = 0; q < NUM_RXQ; q++) {
		mvneta_rxq_init(dev, q);
	}
	for (unsigned int q = 0; q < NUM_TXQ; q++) {
		mvneta_txq_init(dev, q);
	}
	mvneta_shaper_init(dev);
	mvneta_load_mac(dev);

	LOG_DBG("%s: core clock %u Hz", dev->name, data->core_hz);

	cfg->irq_config();

	return 0;
}

#define MVNETA_CLK(node_id, prop, idx)                                                             \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_IDX(node_id, idx)),                         \
		.id = (clock_control_subsys_t)DT_CLOCKS_CELL_BY_IDX(node_id, idx, index),          \
	},

#define MVNETA_IS_RGMII(n)                                                                         \
	(DT_INST_ENUM_IDX_OR(n, phy_connection_type, 3) >= 3 &&                                    \
	 DT_INST_ENUM_IDX_OR(n, phy_connection_type, 3) <= 6)

#define MVNETA_INIT(n)                                                                             \
	BUILD_ASSERT(MVNETA_IS_RGMII(n), "only RGMII is supported");                               \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct mvneta_clk mvneta_clks_##n[] = {                                       \
		DT_INST_FOREACH_PROP_ELEM(n, clocks, MVNETA_CLK)};                                 \
	static struct mvneta_dma mvneta_dma_##n __nocache __aligned(64);                           \
	static void mvneta_irq_config_##n(void)                                                    \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), mvneta_isr,                 \
			    DEVICE_DT_INST_GET(n), DT_INST_IRQ(n, flags));                         \
	}                                                                                          \
	static const struct mvneta_config mvneta_config_##n = {                                    \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.phy_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(n, phy_handle)),                  \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clks = mvneta_clks_##n,                                                           \
		.num_clks = ARRAY_SIZE(mvneta_clks_##n),                                           \
		.rgmii = MVNETA_IS_RGMII(n),                                                       \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(n),                                     \
		.irq_config = mvneta_irq_config_##n,                                               \
		.irq = DT_INST_IRQN(n),                                                            \
	};                                                                                         \
	static struct mvneta_data mvneta_data_##n = {                                              \
		.dma = &mvneta_dma_##n,                                                            \
	};                                                                                         \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, mvneta_init, NULL, &mvneta_data_##n, &mvneta_config_##n,  \
				      CONFIG_ETH_INIT_PRIORITY, &mvneta_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(MVNETA_INIT)
