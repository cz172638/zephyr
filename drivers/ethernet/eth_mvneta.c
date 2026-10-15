/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell NETA gigabit Ethernet controller (Armada 3700 variant).
 *
 * One RX and one TX queue, software buffer management, legacy parser.
 * Descriptor rings and packet buffers live in non-cacheable memory and
 * frames are copied to/from net_pkt, so no cache maintenance is needed.
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

/* Queue registers (queue 0 only) */
#define RXQ_CONFIG            0x1400
#define RXQ_HW_BUF_ALLOC      BIT(0)
#define RXQ_PKT_OFFSET_MASK   GENMASK(11, 8)
#define RXQ_BASE_ADDR         0x1480
#define RXQ_SIZE              0x14a0
#define RXQ_BUF_SIZE_SHIFT    19
#define RXQ_THRESHOLD         0x14c0
#define RXQ_STATUS            0x14e0
#define RXQ_OCCUPIED_MASK     GENMASK(13, 0)
#define RXQ_STATUS_UPDATE     0x1500
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
#define TXQ_CMD               0x2448
#define TXQ_DISABLE_SHIFT     8
#define RX_MIN_FRAME_SIZE     0x247c
#define TXQ_CMD_1             0x24e4
#define ACC_MODE              0x2500
#define ACC_MODE_EXT1         1
#define CPU_MAP(cpu)          (0x2540 + ((cpu) << 2))
#define CPU_RXQ_ACCESS_ALL    0x000000ff
#define CPU_TXQ_ACCESS_ALL    0x0000ff00
#define RXQ_TIME_COAL         0x2580
#define INTR_NEW_CAUSE        0x25a0
#define INTR_NEW_MASK         0x25a4
#define INTR_RXQ0             BIT(8)
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
#define TXQ_TOKEN_COUNT       0x2700
#define TXQ_TOKEN_CFG         0x2704
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
#define GMAC_STATUS           0x2c10
#define GMAC_LINK_UP          BIT(0)
#define DA_FILT_SPEC_MCAST    0x3400
#define DA_FILT_OTH_MCAST     0x3500
#define DA_FILT_UCAST_BASE    0x3600
#define TXQ_BASE_ADDR         0x3c00
#define TXQ_SIZE              0x3c20
#define TXQ_STATUS            0x3c40
#define TXQ_SENT_DESC_MASK    GENMASK(29, 16)
#define TXQ_UPDATE            0x3c60
#define TXQ_DEC_SENT_SHIFT    16
#define PORT_TX_RESET         0x3cf0

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

struct mvneta_dma {
	struct mvneta_rx_desc rx_desc[RX_DESCS];
	struct mvneta_tx_desc tx_desc[TX_DESCS];
	uint8_t rx_buf[RX_DESCS][MVNETA_BUF_SIZE];
	uint8_t tx_buf[TX_DESCS][MVNETA_BUF_SIZE];
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
	uint16_t rx_idx;
	uint16_t tx_idx;
	uint16_t tx_inflight;
	struct k_mutex tx_lock;
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

/* ---------------------------------------------------------------------- */
/* RX                                                                      */

static void mvneta_rxq_update(const struct device *dev, uint32_t done, uint32_t filled)
{
	while (done > 0 || filled > 0) {
		uint32_t d = MIN(done, 0xff);
		uint32_t f = MIN(filled, 0xff);

		neta_write(dev, RXQ_STATUS_UPDATE, d | (f << RXQ_ADD_NON_OCC_SHIFT));
		done -= d;
		filled -= f;
	}
}

static void mvneta_rx_desc_fill(struct mvneta_data *data, uint16_t idx)
{
	struct mvneta_rx_desc *d = &data->dma->rx_desc[idx];

	d->status = 0;
	d->data_size = 0;
	d->buf_phys_addr = dma_addr(data->dma->rx_buf[idx]);
	d->buf_cookie = idx;
}

static void mvneta_rxq_init(const struct device *dev)
{
	struct mvneta_data *data = dev->data;

	for (uint16_t i = 0; i < RX_DESCS; i++) {
		mvneta_rx_desc_fill(data, i);
	}
	data->rx_idx = 0;
	barrier_dmem_fence_full();

	neta_write(dev, RXQ_BASE_ADDR, dma_addr(data->dma->rx_desc));
	neta_write(dev, RXQ_SIZE, RX_DESCS | ((MVNETA_BUF_SIZE >> 3) << RXQ_BUF_SIZE_SHIFT));

	/* Interrupt on every received frame */
	neta_write(dev, RXQ_THRESHOLD, 1);
	neta_write(dev, RXQ_TIME_COAL, CONFIG_ETH_MVNETA_RX_COAL_CYCLES);

	/* No packet offset, software buffer management */
	neta_rmw(dev, RXQ_CONFIG, RXQ_PKT_OFFSET_MASK | RXQ_HW_BUF_ALLOC, 0);

	mvneta_rxq_update(dev, 0, RX_DESCS);
}

static void mvneta_rx_one(const struct device *dev, struct mvneta_rx_desc *d, uint16_t idx)
{
	struct mvneta_data *data = dev->data;
	uint32_t status = d->status;
	uint16_t size = d->data_size;
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

	if (net_pkt_write(pkt, data->dma->rx_buf[idx] + MVNETA_MH_SIZE, len) < 0 ||
	    net_recv_data(data->iface, pkt) < 0) {
		net_pkt_unref(pkt);
		eth_stats_update_errors_rx(data->iface);
	}
}

static int mvneta_rx(const struct device *dev)
{
	struct mvneta_data *data = dev->data;
	uint32_t occupied;
	uint32_t done = 0;

	occupied = neta_read(dev, RXQ_STATUS) & RXQ_OCCUPIED_MASK;
	barrier_dmem_fence_full();

	while (done < occupied) {
		uint16_t idx = data->rx_idx;

		mvneta_rx_one(dev, &data->dma->rx_desc[idx], idx);
		mvneta_rx_desc_fill(data, idx);

		data->rx_idx = (idx + 1) % RX_DESCS;
		done++;
	}

	if (done > 0) {
		barrier_dmem_fence_full();
		mvneta_rxq_update(dev, done, done);
	}

	return done;
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
			while (mvneta_rx(dev) > 0) {
			}
		}

		neta_write(dev, INTR_NEW_MASK, INTR_RXQ0 | INTR_MISC);
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

static void mvneta_txq_init(const struct device *dev)
{
	struct mvneta_data *data = dev->data;

	memset(data->dma->tx_desc, 0, sizeof(data->dma->tx_desc));
	data->tx_idx = 0;
	data->tx_inflight = 0;

	neta_write(dev, TXQ_TOKEN_CFG, 0x03ffffff);
	neta_write(dev, TXQ_TOKEN_COUNT, 0x3fffffff);
	neta_write(dev, TXQ_BASE_ADDR, dma_addr(data->dma->tx_desc));
	/* No TX done interrupts: completions are reclaimed in send() */
	neta_write(dev, TXQ_SIZE, TX_DESCS);
}

static void mvneta_tx_reclaim(const struct device *dev)
{
	struct mvneta_data *data = dev->data;
	uint32_t sent = FIELD_GET(TXQ_SENT_DESC_MASK, neta_read(dev, TXQ_STATUS));

	sent = MIN(sent, data->tx_inflight);
	data->tx_inflight -= sent;

	while (sent > 0) {
		uint32_t n = MIN(sent, 0xff);

		neta_write(dev, TXQ_UPDATE, n << TXQ_DEC_SENT_SHIFT);
		sent -= n;
	}
}

static int mvneta_send(const struct device *dev, struct net_pkt *pkt)
{
	struct mvneta_data *data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	struct mvneta_tx_desc *d;
	uint16_t idx;
	int ret = 0;

	if (len > MVNETA_BUF_SIZE) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&data->tx_lock, K_FOREVER);

	mvneta_tx_reclaim(dev);
	if (data->tx_inflight >= TX_DESCS) {
		if (!WAIT_FOR((mvneta_tx_reclaim(dev), data->tx_inflight < TX_DESCS), 10000,
			      k_busy_wait(10))) {
			LOG_WRN("tx ring full");
			ret = -EBUSY;
			goto out;
		}
	}

	idx = data->tx_idx;
	net_pkt_cursor_init(pkt);
	if (net_pkt_read(pkt, data->dma->tx_buf[idx], len) < 0) {
		ret = -EIO;
		goto out;
	}

	d = &data->dma->tx_desc[idx];
	d->buf_phys_addr = dma_addr(data->dma->tx_buf[idx]);
	d->data_size = len;
	d->command = TXD_L4_CSUM_NOT | TXD_F_DESC | TXD_L_DESC | TXD_Z_PAD;
	barrier_dmem_fence_full();

	data->tx_idx = (idx + 1) % TX_DESCS;
	data->tx_inflight++;
	neta_write(dev, TXQ_UPDATE, 1);

out:
	k_mutex_unlock(&data->tx_lock);
	if (ret < 0) {
		eth_stats_update_errors_tx(data->iface);
	}
	return ret;
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
	neta_write(dev, TXQ_CMD_1, 0);
	neta_write(dev, TXQ_TOKEN_COUNT, 0);
	neta_write(dev, TXQ_TOKEN_CFG, 0);
	neta_write(dev, PORT_TX_RESET, 0);
	neta_write(dev, PORT_RX_RESET, 0);

	neta_write(dev, ACC_MODE, ACC_MODE_EXT1);

	/* Default RX queue 0 for all traffic classes */
	neta_write(dev, PORT_CONFIG, TX_UNSET_ERR_SUM | RX_CSUM_WITH_PSEUDO);
	neta_write(dev, PORT_CONFIG_EXTEND, 0);
	neta_write(dev, RX_MIN_FRAME_SIZE, 64);

	neta_write(dev, SDMA_CONFIG,
		   TX_BRST_SZ(SDMA_BRST_SIZE_16) | RX_BRST_SZ(SDMA_BRST_SIZE_16) | RX_NO_DATA_SWAP |
			   TX_NO_DATA_SWAP);

	neta_rmw(dev, UNIT_CONTROL, PHY_POLLING_ENABLE, 0);

	mvneta_set_mcast_filter(dev, true);

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
		} else if (PHY_LINK_IS_SPEED_100M(state->speed)) {
			val |= AN_CONFIG_MII_SPEED;
		}
		if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
			val |= AN_CONFIG_FULL_DUPLEX;
		}
	} else {
		val |= AN_FORCE_LINK_DOWN;
	}
	neta_write(dev, GMAC_AUTONEG_CONFIG, val);

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

	neta_write(dev, RXQ_CMD, BIT(0));
	neta_write(dev, TXQ_CMD, BIT(0));
	neta_rmw(dev, GMAC_CTRL_0, 0, GMAC0_PORT_ENABLE);
	neta_write(dev, INTR_NEW_MASK, INTR_RXQ0 | INTR_MISC);
	irq_enable(cfg->irq);

	return 0;
}

static int mvneta_stop(const struct device *dev, struct net_if *iface)
{
	const struct mvneta_config *cfg = dev->config;

	ARG_UNUSED(iface);

	irq_disable(cfg->irq);
	neta_write(dev, INTR_NEW_MASK, 0);
	neta_write(dev, RXQ_CMD, BIT(0) << RXQ_DISABLE_SHIFT);
	neta_write(dev, TXQ_CMD, BIT(0) << TXQ_DISABLE_SHIFT);
	neta_rmw(dev, GMAC_CTRL_0, GMAC0_PORT_ENABLE, 0);

	return 0;
}

static enum ethernet_hw_caps mvneta_get_caps(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE;
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
	LOG_INF("%s: MAC %02x:%02x:%02x:%02x:%02x:%02x", dev->name, data->mac_addr[0],
		data->mac_addr[1], data->mac_addr[2], data->mac_addr[3], data->mac_addr[4],
		data->mac_addr[5]);

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

	k_mutex_init(&data->tx_lock);
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

	mvneta_rxq_init(dev);
	mvneta_txq_init(dev);
	mvneta_load_mac(dev);

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
