/*
 * Driver for Synopsys DesignWare MAC
 *
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 glue: GMAC 3.70a (dwc_ether_1000 core), RGMII or RMII.
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwmac_plat, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT rockchip_rk3399_gmac

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/util.h>

#include "eth_dwmac_priv.h"

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1, "RK3399 has one GMAC");
BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rgmii) ||
		     DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rgmii_id) ||
		     DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii),
	     "RK3399 GMAC: RGMII or RMII only");

/* The DMA bus master interface is a 64-bit AXI interface on this IP */
#define DATA_BUS_WIDTH 64

DWMAC_ASSERT_BUFFER_ALIGNMENT(DATA_BUS_WIDTH);

#define IS_RMII DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii)

/* GRF: the upper half-word is the write mask of the lower one */
#define GRF_BIT(nr)          (BIT(nr) | BIT((nr) + 16))
#define GRF_CLR_BIT(nr)      BIT((nr) + 16)
#define GRF_FIELD(v, m, s)   ((((v) & (m)) << (s)) | ((m) << ((s) + 16)))

#define GRF_SOC_CON5 0xc214U
#define GRF_SOC_CON6 0xc218U

#define CON5_PHY_INTF_RGMII GRF_FIELD(1U, 7U, 9)
#define CON5_PHY_INTF_RMII  GRF_FIELD(4U, 7U, 9)
#define CON5_SPEED_100M     GRF_BIT(7)
#define CON5_SPEED_10M      GRF_CLR_BIT(7)
#define CON5_RMII_MODE      GRF_BIT(6)
#define CON5_RMII_MODE_CLR  GRF_CLR_BIT(6)
#define CON5_RGMII_CLK_125M GRF_FIELD(0U, 3U, 4)
#define CON5_RGMII_CLK_25M  GRF_FIELD(3U, 3U, 4)
#define CON5_RGMII_CLK_2M5  GRF_FIELD(2U, 3U, 4)
#define CON5_RMII_CLK_25M   GRF_BIT(3)
#define CON5_RMII_CLK_2M5   GRF_CLR_BIT(3)

#define CON6_RXCLK_DLY_ENA GRF_BIT(15)
#define CON6_RXCLK_DLY_DIS GRF_CLR_BIT(15)
#define CON6_RX_DL(v)      GRF_FIELD(v, 0x7fU, 8)
#define CON6_TXCLK_DLY_ENA GRF_BIT(7)
#define CON6_TXCLK_DLY_DIS GRF_CLR_BIT(7)
#define CON6_TX_DL(v)      GRF_FIELD(v, 0x7fU, 0)

/* CRU_CLKSEL_CON19[4]: clk_rmii_src from the MAC clock divider or the pin */
#define CRU_CLKSEL_CON19      0x14cU
#define CLKSEL19_RMII_SRC_IO  GRF_BIT(4)
#define CLKSEL19_RMII_SRC_PLL GRF_CLR_BIT(4)

struct rk3399_gmac_cfg {
	const struct device *grf;
	uintptr_t cru;
	const struct pinctrl_dev_config *pcfg;
	struct reset_dt_spec reset;
	const uint32_t *clocks;
	uint8_t num_clocks;
	uint8_t tx_delay;
	uint8_t rx_delay;
	bool clock_input;
};

PINCTRL_DT_INST_DEFINE(0);

#define GMAC_CLKID(node_id, prop, idx) DT_CLOCKS_CELL_BY_IDX(node_id, idx, clkid)

static const uint32_t rk3399_gmac_clocks[] = {
	DT_INST_FOREACH_PROP_ELEM_SEP(0, clocks, GMAC_CLKID, (,))
};

static const struct rk3399_gmac_cfg rk3399_cfg = {
	.grf = DEVICE_DT_GET(DT_INST_PHANDLE(0, rockchip_grf)),
	.cru = DT_REG_ADDR(DT_INST_PHANDLE(0, rockchip_cru)),
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.reset = RESET_DT_SPEC_INST_GET(0),
	.clocks = rk3399_gmac_clocks,
	.num_clocks = ARRAY_SIZE(rk3399_gmac_clocks),
	.tx_delay = DT_INST_PROP(0, tx_delay),
	.rx_delay = DT_INST_PROP(0, rx_delay),
	.clock_input = DT_INST_ENUM_IDX(0, clock_in_out) == 0,
};

static void rk3399_gmac_grf(uint32_t reg, uint32_t val)
{
	(void)syscon_write_reg(rk3399_cfg.grf, reg, val);
}

void dwmac_platform_link_speed_changed(const struct device *dev, enum phy_link_speed speed)
{
	ARG_UNUSED(dev);

	if (IS_RMII) {
		rk3399_gmac_grf(GRF_SOC_CON5, PHY_LINK_IS_SPEED_100M(speed)
					    ? (CON5_RMII_CLK_25M | CON5_SPEED_100M)
					    : (CON5_RMII_CLK_2M5 | CON5_SPEED_10M));
		return;
	}

	if (PHY_LINK_IS_SPEED_1000M(speed)) {
		rk3399_gmac_grf(GRF_SOC_CON5, CON5_RGMII_CLK_125M);
	} else if (PHY_LINK_IS_SPEED_100M(speed)) {
		rk3399_gmac_grf(GRF_SOC_CON5, CON5_RGMII_CLK_25M);
	} else {
		rk3399_gmac_grf(GRF_SOC_CON5, CON5_RGMII_CLK_2M5);
	}
}

int dwmac_bus_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	mm_reg_t cru;
	int ret;

	if (!device_is_ready(rk3399_cfg.grf) || !device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	/* The interface select is sampled while the MAC is in reset */
	ret = reset_line_assert_dt(&rk3399_cfg.reset);
	if (ret != 0) {
		LOG_ERR("could not assert the GMAC reset (%d)", ret);
		return ret;
	}

	ret = pinctrl_apply_state(rk3399_cfg.pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("could not configure the GMAC pins (%d)", ret);
		return ret;
	}

	for (uint8_t i = 0; i < rk3399_cfg.num_clocks; i++) {
		ret = clock_control_on(cfg->clock,
				       (clock_control_subsys_t)(uintptr_t)rk3399_cfg.clocks[i]);
		if (ret < 0 && ret != -EALREADY) {
			LOG_ERR("could not enable GMAC clock %u (%d)", rk3399_cfg.clocks[i], ret);
			return ret;
		}
	}

	device_map(&cru, rk3399_cfg.cru, CRU_CLKSEL_CON19 + 4U, K_MEM_CACHE_NONE);
	sys_write32(rk3399_cfg.clock_input ? CLKSEL19_RMII_SRC_IO : CLKSEL19_RMII_SRC_PLL,
		    cru + CRU_CLKSEL_CON19);

	if (IS_RMII) {
		rk3399_gmac_grf(GRF_SOC_CON5, CON5_PHY_INTF_RMII | CON5_RMII_MODE);
	} else {
		rk3399_gmac_grf(GRF_SOC_CON5, CON5_PHY_INTF_RGMII | CON5_RMII_MODE_CLR |
						      CON5_RGMII_CLK_125M);
		rk3399_gmac_grf(GRF_SOC_CON6,
				(rk3399_cfg.tx_delay != 0U ? CON6_TXCLK_DLY_ENA
							   : CON6_TXCLK_DLY_DIS) |
				(rk3399_cfg.rx_delay != 0U ? CON6_RXCLK_DLY_ENA
							   : CON6_RXCLK_DLY_DIS) |
				CON6_TX_DL(rk3399_cfg.tx_delay) | CON6_RX_DL(rk3399_cfg.rx_delay));
	}

	ret = reset_line_deassert_dt(&rk3399_cfg.reset);
	if (ret != 0) {
		LOG_ERR("could not deassert the GMAC reset (%d)", ret);
		return ret;
	}

	return 0;
}

#define DESCRIPTOR_ALIGNMENT ((DATA_BUS_WIDTH) / (BITS_PER_BYTE))

BUILD_ASSERT(IS_ENABLED(CONFIG_NOCACHE_MEMORY),
	     "RK3399 GMAC needs CONFIG_NOCACHE_MEMORY for its DMA descriptors");

static struct dwmac_dma_desc dwmac_tx_descs[NB_TX_DESCS]
	__nocache_noinit __aligned(DESCRIPTOR_ALIGNMENT);
static struct dwmac_dma_desc dwmac_rx_descs[NB_RX_DESCS]
	__nocache_noinit __aligned(DESCRIPTOR_ALIGNMENT);

int dwmac_platform_init(const struct device *dev)
{
	const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0);
	struct dwmac_priv *p = dev->data;
	int ret;

	DWMAC_REG_WRITE(DWMAC_DMABMR, DWMAC_DMABMR_AAL | DWMAC_DMABMR_FB);

	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr, DEVICE_DT_INST_GET(0),
		    DT_INST_IRQ(0, flags));
	irq_enable(DT_INST_IRQN(0));

	ret = net_eth_mac_load(&mac_cfg, p->mac_addr);
	if (ret < 0) {
		LOG_ERR("could not get a MAC address (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct dwmac_config dwmac_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle)),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.mac_clk = (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(0, clkid),
};

static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0, dwmac_probe, NULL, &dwmac_instance, &dwmac_config,
			      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);
