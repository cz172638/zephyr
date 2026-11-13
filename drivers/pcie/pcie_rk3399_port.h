/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe: what the root complex and the endpoint driver share. One
 * controller (Cadence core behind the Rockchip "client" wrapper, PHY in
 * the GRF), either role; the client registers, clocks, resets and the
 * PHY bring-up are the same, only CLIENT_MODE_RC differs.
 *
 * Register sequences follow Linux drivers/pci/controller/pcie-rockchip*.c
 * and drivers/phy/rockchip/phy-rockchip-pcie.c.
 */

#ifndef ZEPHYR_DRIVERS_PCIE_PCIE_RK3399_PORT_H_
#define ZEPHYR_DRIVERS_PCIE_PCIE_RK3399_PORT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* Write-masked 16-bit fields: upper half-word selects the bits to write */
#define RK_WM16(mask, val) ((((uint32_t)(val) << (__builtin_ctz(mask))) & (mask)) | ((mask) << 16))

/* Client registers, apb + 0 */
#define RK_CLIENT_CONFIG            0x00U
#define  RK_CLIENT_CONF_ENABLE      BIT(0)
#define  RK_CLIENT_LINK_TRAIN       BIT(1)
#define  RK_CLIENT_SR_IOV_ENABLE    BIT(2)
#define  RK_CLIENT_ARI_ENABLE       BIT(3)
#define  RK_CLIENT_LANE_NUM         GENMASK(5, 4)
#define  RK_CLIENT_MODE_RC          BIT(6)
#define  RK_CLIENT_GEN_SEL_2        BIT(7)
#define RK_CLIENT_LEGACY_INT_CTRL   0x0cU
#define  RK_CLIENT_INT_PEND_ST      BIT(0)
#define  RK_CLIENT_INT_IN           BIT(1)
#define RK_CLIENT_SIDE_BAND_STATUS  0x20U
#define  RK_CLIENT_PHY_ST           BIT(12)
#define RK_CLIENT_DEBUG_OUT_0       0x3cU
#define  RK_CLIENT_LTSSM            GENMASK(5, 0)
#define RK_CLIENT_BASIC_STATUS0     0x44U
#define  RK_CLIENT_NEG_LINK_WIDTH   GENMASK(7, 6)
#define  RK_CLIENT_NEG_LINK_SPEED   BIT(5)
#define RK_CLIENT_BASIC_STATUS1     0x48U
#define  RK_CLIENT_LINK_STATUS      GENMASK(21, 20)
#define RK_CLIENT_INT_MASK          0x4cU
#define RK_CLIENT_INT_STATUS        0x50U

/* Cadence core inside the apb region (see pcie_ep_cdns.h for its map) */
#define RK_CORE_OFFSET              0x800000U

enum rk3399_pcie_rst {
	RK_RST_CORE,
	RK_RST_MGMT,
	RK_RST_MGMT_STICKY,
	RK_RST_PIPE,
	RK_RST_PM,
	RK_RST_PCLK,
	RK_RST_ACLK,
	RK_RST_PHY,
	RK_RST_NUM,
};

enum rk3399_pcie_clk {
	RK_CLK_ACLK,
	RK_CLK_ACLK_PERF,
	RK_CLK_HCLK,
	RK_CLK_PM,
	RK_CLK_NUM,
};

struct rk3399_pcie_clk_spec {
	const struct device *dev;
	uint32_t id;
};

/* Static description, from the devicetree node of either driver */
struct rk3399_pcie_port_cfg {
	struct rk3399_pcie_clk_spec clks[RK_CLK_NUM];
	struct reset_dt_spec rsts[RK_RST_NUM];
	const struct device *grf;
	uint8_t lanes;
	uint8_t link_gen;
};

static inline uint32_t rk_client_read(mm_reg_t apb, uint32_t reg)
{
	return sys_read32(apb + reg);
}

static inline void rk_client_write(mm_reg_t apb, uint32_t reg, uint32_t val)
{
	sys_write32(val, apb + reg);
}

/* Clocks on, resets ready */
int rk3399_pcie_port_prepare(const struct rk3399_pcie_port_cfg *cfg);

/*
 * Reset sequence, client mode (rc or ep), lanes, Gen, PHY power on, core
 * out of reset. Link training and configuration access stay off.
 */
int rk3399_pcie_port_init(const struct rk3399_pcie_port_cfg *cfg, mm_reg_t apb, bool rc);

static inline bool rk3399_pcie_link_up(mm_reg_t apb)
{
	return (rk_client_read(apb, RK_CLIENT_BASIC_STATUS1) & RK_CLIENT_LINK_STATUS) ==
	       RK_CLIENT_LINK_STATUS;
}

static inline uint8_t rk3399_pcie_ltssm(mm_reg_t apb)
{
	return rk_client_read(apb, RK_CLIENT_DEBUG_OUT_0) & RK_CLIENT_LTSSM;
}

/* "L0", "polling-active", ... (Cadence LTSSM encoding) */
const char *rk3399_pcie_ltssm_name(uint8_t s);

/* Negotiated speed (1 = 2.5, 2 = 5 GT/s) and width */
void rk3399_pcie_link_info(mm_reg_t apb, unsigned int *gen, unsigned int *width);

#endif /* ZEPHYR_DRIVERS_PCIE_PCIE_RK3399_PORT_H_ */
