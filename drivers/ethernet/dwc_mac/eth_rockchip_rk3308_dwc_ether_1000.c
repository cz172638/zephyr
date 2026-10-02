/*
 * Driver for Synopsys DesignWare MAC
 *
 * SPDX-FileCopyrightText: Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3308 glue for the 10/100 RMII MAC (MAC2IO), a DWMAC v3.x
 * ("dwc_ether_1000") core.
 *
 * There is no CRU or pinctrl driver for the RK3308 yet, so the clock tree,
 * gates, reset and the RMII pin group (M0, GPIO1_B4..GPIO1_C5) are set up
 * here directly through the CRU and GRF syscons. Register layout from the
 * RK3308 TRM v1.1 Part1 (chapters 2, 23 and 30).
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwmac_plat, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT rockchip_rk3308_mac

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/util.h>

#include "eth_dwmac_priv.h"

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "the RK3308 has a single MAC");
BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii),
	     "RK3308 MAC supports phy-connection-type = \"rmii\" only");
BUILD_ASSERT(IS_ENABLED(CONFIG_NOCACHE_MEMORY),
	     "RK3308 MAC requires CONFIG_NOCACHE_MEMORY for DMA descriptors");

/* AXI master; 64-bit is the stricter of the possible widths */
#define DATA_BUS_WIDTH 64

DWMAC_ASSERT_BUFFER_ALIGNMENT(DATA_BUS_WIDTH);

/* Rockchip "hiword" registers: upper 16 bits are the write enable mask */
#define HIWORD(mask, val) (((uint32_t)(mask) << 16) | ((val) & (mask)))

/* CRU */
#define CRU_PLL_CON0(n)  (0x20U * (n))
#define CRU_PLL_CON1(n)  (0x20U * (n) + 0x04U)
#define CRU_PLL_CON2(n)  (0x20U * (n) + 0x08U)
#define CRU_MODE         0x00a0U
#define CRU_CLKSEL_CON43 0x01acU
#define CRU_CLKGATE_CON8 0x0320U
#define CRU_CLKGATE_CON9 0x0324U
#define CRU_SOFTRST_CON7 0x041cU

#define PLL_CON0_BYPASS   BIT(15)
#define PLL_CON0_POSTDIV1 GENMASK(14, 12)
#define PLL_CON0_FBDIV    GENMASK(11, 0)
#define PLL_CON1_DSMPD    BIT(12)
#define PLL_CON1_POSTDIV2 GENMASK(8, 6)
#define PLL_CON1_REFDIV   GENMASK(5, 0)
#define PLL_CON2_FRACDIV  GENMASK(23, 0)
#define CRU_MODE_PLL(n)   (GENMASK(1, 0) << (2U * (n)))
#define CRU_MODE_NORMAL   1U

/* PLL index as used by CRU_PLL_CONx() and CRU_MODE */
#define RK3308_DPLL  1U
#define RK3308_VPLL0 2U
#define RK3308_VPLL1 3U

#define CLKSEL43_RMII_100M    BIT(15)
#define CLKSEL43_RMII_EXTCLK  BIT(14)
#define CLKSEL43_MAC_PLL_SEL  GENMASK(7, 6)
#define CLKSEL43_MAC_DIV      GENMASK(4, 0)

#define CLKGATE8_CLK_MAC_PLL  BIT(14)
#define CLKGATE9_PCLK_MAC     BIT(15)
#define CLKGATE9_ACLK_MAC     BIT(4)
#define CLKGATE9_CLK_MAC_REF  BIT(1)
#define CLKGATE9_CLK_MAC_TXRX BIT(0)

#define SOFTRST7_MAC_A        BIT(13)

/* GRF */
#define GRF_GPIO1B_IOMUX_L 0x0028U
#define GRF_GPIO1B_IOMUX_H 0x002cU
#define GRF_GPIO1C_IOMUX_L 0x0030U
#define GRF_GPIO1C_IOMUX_H 0x0034U
#define GRF_GPIO1B_P       0x00b4U
#define GRF_GPIO1C_P       0x00b8U
#define GRF_GPIO1B_E       0x0114U
#define GRF_GPIO1C_E       0x0118U
#define GRF_MAC_CON0       0x04a0U

#define GRF_MAC_CON0_INTF_SEL      GENMASK(4, 2)
#define GRF_MAC_CON0_INTF_SEL_RMII (4U << 2)
#define GRF_MAC_CON0_SPEED_100M    BIT(0)

#define RK3308_OSC_HZ     24000000ULL
#define RK3308_RMII_REF_HZ 50000000ULL

struct rk3308_mac_config {
	struct dwmac_config dwmac;
	const struct device *grf;
	const struct device *cru;
	bool clock_input;
};

static struct dwmac_dma_desc __nocache_noinit __aligned(CONFIG_DCACHE_LINE_SIZE)
	dwmac_tx_descs[NB_TX_DESCS];
static struct dwmac_dma_desc __nocache_noinit __aligned(CONFIG_DCACHE_LINE_SIZE)
	dwmac_rx_descs[NB_RX_DESCS];

static uint32_t rk3308_read(const struct device *syscon, uint16_t reg)
{
	uint32_t val = 0;

	(void)syscon_read_reg(syscon, reg, &val);

	return val;
}

static void rk3308_write(const struct device *syscon, uint16_t reg, uint32_t val)
{
	(void)syscon_write_reg(syscon, reg, val);
}

/* Output rate of DPLL/VPLL0/VPLL1 as left by the boot loader */
static uint64_t rk3308_pll_rate(const struct device *cru, unsigned int pll)
{
	uint32_t mode = rk3308_read(cru, CRU_MODE);
	uint32_t con0 = rk3308_read(cru, CRU_PLL_CON0(pll));
	uint32_t con1 = rk3308_read(cru, CRU_PLL_CON1(pll));
	uint32_t con2 = rk3308_read(cru, CRU_PLL_CON2(pll));
	uint64_t fbdiv, div;

	if ((((mode & CRU_MODE_PLL(pll)) >> (2U * pll)) != CRU_MODE_NORMAL) ||
	    ((con0 & PLL_CON0_BYPASS) != 0U)) {
		return RK3308_OSC_HZ;
	}

	/* feedback divider in 24-bit fixed point */
	fbdiv = (uint64_t)FIELD_GET(PLL_CON0_FBDIV, con0) << 24;
	if ((con1 & PLL_CON1_DSMPD) == 0U) {
		fbdiv += FIELD_GET(PLL_CON2_FRACDIV, con2);
	}

	div = (uint64_t)FIELD_GET(PLL_CON1_REFDIV, con1) *
	      FIELD_GET(PLL_CON0_POSTDIV1, con0) * FIELD_GET(PLL_CON1_POSTDIV2, con1);
	if (div == 0U) {
		return 0;
	}

	return ((RK3308_OSC_HZ * fbdiv) / div) >> 24;
}

/*
 * Feed clk_mac with 50 MHz from whichever PLL divides down to it best; it
 * also drives the MAC_CLK pin when the SoC provides the RMII reference.
 */
static int rk3308_mac_set_clk(const struct device *cru)
{
	static const uint8_t plls[] = { RK3308_DPLL, RK3308_VPLL0, RK3308_VPLL1 };
	uint64_t best_err = UINT64_MAX;
	uint32_t best_sel = 0, best_div = 0;

	for (uint32_t sel = 0; sel < ARRAY_SIZE(plls); sel++) {
		uint64_t rate = rk3308_pll_rate(cru, plls[sel]);
		uint64_t div = DIV_ROUND_CLOSEST(rate, RK3308_RMII_REF_HZ);
		uint64_t err;

		if ((div == 0U) || (div > (CLKSEL43_MAC_DIV + 1U))) {
			continue;
		}

		err = (rate / div > RK3308_RMII_REF_HZ) ? rate / div - RK3308_RMII_REF_HZ
							 : RK3308_RMII_REF_HZ - rate / div;
		LOG_DBG("PLL sel %u: %llu Hz, /%llu -> error %llu Hz", sel, rate, div, err);
		if (err < best_err) {
			best_err = err;
			best_sel = sel;
			best_div = (uint32_t)div;
		}
	}

	if (best_div == 0U) {
		LOG_ERR("no PLL can provide the 50 MHz RMII reference");
		return -EIO;
	}

	/* RMII allows +-50 ppm, i.e. 2500 Hz at 50 MHz */
	if (best_err > 2500U) {
		LOG_WRN("RMII reference off by %llu Hz", best_err);
	}

	rk3308_write(cru, CRU_CLKSEL_CON43,
		     HIWORD(CLKSEL43_RMII_EXTCLK | CLKSEL43_MAC_PLL_SEL | CLKSEL43_MAC_DIV,
			    FIELD_PREP(CLKSEL43_MAC_PLL_SEL, best_sel) |
			    FIELD_PREP(CLKSEL43_MAC_DIV, best_div - 1U)));

	LOG_DBG("clk_mac: PLL sel %u /%u", best_sel, best_div);

	return 0;
}

/* RMII pins, M0 group, 12 mA on the outputs and the reference clock */
static void rk3308_mac_pinmux(const struct device *grf)
{
	/* func 3: B4 MAC_CLK, B5 MDC, B6 MDIO */
	rk3308_write(grf, GRF_GPIO1B_IOMUX_L, HIWORD(GENMASK(15, 8), 0x3f00U));
	/* func 3: B7 RXER */
	rk3308_write(grf, GRF_GPIO1B_IOMUX_H, HIWORD(GENMASK(1, 0), 0x0003U));
	/* func 3: C0 RXDV, C1 TXEN, C2 TXD0, C3 TXD1, C4 RXD0 */
	rk3308_write(grf, GRF_GPIO1C_IOMUX_L, HIWORD(GENMASK(15, 0), 0x333fU));
	/* func 3: C5 RXD1 */
	rk3308_write(grf, GRF_GPIO1C_IOMUX_H, HIWORD(GENMASK(3, 0), 0x0003U));

	/* no pull on B4..B7 and C0..C5 (reset default is pull-down) */
	rk3308_write(grf, GRF_GPIO1B_P, HIWORD(GENMASK(15, 8), 0U));
	rk3308_write(grf, GRF_GPIO1C_P, HIWORD(GENMASK(11, 0), 0U));

	/* 12 mA: B4 MAC_CLK, C1 TXEN, C2 TXD0, C3 TXD1 */
	rk3308_write(grf, GRF_GPIO1B_E, HIWORD(GENMASK(9, 8), GENMASK(9, 8)));
	rk3308_write(grf, GRF_GPIO1C_E, HIWORD(GENMASK(7, 2), GENMASK(7, 2)));
}

static void rk3308_mac_set_speed(const struct rk3308_mac_config *cfg, bool speed_100m)
{
	rk3308_write(cfg->grf, GRF_MAC_CON0,
		     HIWORD(GRF_MAC_CON0_SPEED_100M, speed_100m ? GRF_MAC_CON0_SPEED_100M : 0U));
	rk3308_write(cfg->cru, CRU_CLKSEL_CON43,
		     HIWORD(CLKSEL43_RMII_100M, speed_100m ? CLKSEL43_RMII_100M : 0U));
}

void dwmac_platform_link_speed_changed(const struct device *dev, enum phy_link_speed speed)
{
	const struct rk3308_mac_config *cfg = dev->config;

	rk3308_mac_set_speed(cfg, PHY_LINK_IS_SPEED_100M(speed));
}

int dwmac_bus_init(const struct device *dev)
{
	const struct rk3308_mac_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->grf) || !device_is_ready(cfg->cru)) {
		LOG_ERR("GRF/CRU syscon not ready");
		return -ENODEV;
	}

	rk3308_write(cfg->cru, CRU_SOFTRST_CON7, HIWORD(SOFTRST7_MAC_A, SOFTRST7_MAC_A));

	rk3308_write(cfg->cru, CRU_CLKGATE_CON8, HIWORD(CLKGATE8_CLK_MAC_PLL, 0U));
	rk3308_write(cfg->cru, CRU_CLKGATE_CON9,
		     HIWORD(CLKGATE9_PCLK_MAC | CLKGATE9_ACLK_MAC | CLKGATE9_CLK_MAC_REF |
			    CLKGATE9_CLK_MAC_TXRX, 0U));

	if (cfg->clock_input) {
		rk3308_write(cfg->cru, CRU_CLKSEL_CON43,
			     HIWORD(CLKSEL43_RMII_EXTCLK, CLKSEL43_RMII_EXTCLK));
	} else {
		ret = rk3308_mac_set_clk(cfg->cru);
		if (ret < 0) {
			return ret;
		}
	}

	rk3308_mac_pinmux(cfg->grf);

	rk3308_write(cfg->grf, GRF_MAC_CON0,
		     HIWORD(GRF_MAC_CON0_INTF_SEL, GRF_MAC_CON0_INTF_SEL_RMII));
	rk3308_mac_set_speed(cfg, true);

	rk3308_write(cfg->cru, CRU_SOFTRST_CON7, HIWORD(SOFTRST7_MAC_A, 0U));

	/* let the RMII reference settle before dwmac_probe resets the MAC */
	k_msleep(1);

	return 0;
}

int dwmac_platform_init(const struct device *dev)
{
	const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0);
	struct dwmac_priv *p = dev->data;
	int ret;

	DWMAC_REG_WRITE(DWMAC_DMABMR, DWMAC_DMABMR_AAL | DWMAC_DMABMR_FB);

	/* identity mapped and uncached: the core hands these addresses to the DMA */
	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;
	p->tx_descs_phys = (struct dwmac_dma_desc *)k_mem_phys_addr(dwmac_tx_descs);
	p->rx_descs_phys = (struct dwmac_dma_desc *)k_mem_phys_addr(dwmac_rx_descs);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	ret = net_eth_mac_load(&mac_cfg, p->mac_addr);
	if (ret == -ENODATA) {
		LOG_WRN("No MAC address configured");
		return 0;
	}
	if (ret < 0) {
		LOG_ERR("Failed to load MAC address (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct rk3308_mac_config rk3308_mac_config = {
	.dwmac = {
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle)),
	},
	.grf = DEVICE_DT_GET(DT_INST_PHANDLE(0, rockchip_grf)),
	.cru = DEVICE_DT_GET(DT_INST_PHANDLE(0, rockchip_cru)),
	.clock_input = DT_INST_ENUM_HAS_VALUE(0, clock_in_out, input),
};

static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0, dwmac_probe, NULL, &dwmac_instance, &rk3308_mac_config,
			      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);
