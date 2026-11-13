/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 *
 * RK3399 PCIe controller bring-up shared by the root complex and the
 * endpoint driver (see pcie_rk3399_port.h).
 */

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "pcie_rk3399_port.h"

LOG_MODULE_REGISTER(pcie_rk3399, LOG_LEVEL_INF);

/* PCIe PHY in the GRF */
#define GRF_PCIE_PHY_LANEOFF 0xe214U
#define  PHY_LANE_IDLE       BIT(3)
#define GRF_PCIE_PHY_CONF    0xe220U
#define  PHY_CFG_WR          BIT(0)
#define  PHY_CFG_ADDR        GENMASK(6, 1)
#define  PHY_CFG_DATA        GENMASK(10, 7)
#define GRF_PCIE_PHY_STATUS  0xe2a4U
#define  PHY_PLL_LOCKED      BIT(9)
#define  PHY_PLL_OUTPUT      BIT(10)
#define PHY_CFG_PLL_LOCK     0x10U
#define PHY_CFG_CLK_TEST     0x10U
#define PHY_CFG_CLK_SCC      0x12U
#define PHY_CFG_SEPE_RATE    BIT(3)
#define PHY_CFG_PLL_100M     BIT(3)

#define PHY_TIMEOUT_US 100000U

static int grf_poll(const struct device *grf, uint32_t reg, uint32_t mask, bool set)
{
	uint32_t val;

	for (uint32_t t = 0; t < PHY_TIMEOUT_US; t += 200U) {
		int ret = syscon_read_reg(grf, reg, &val);

		if (ret != 0) {
			return ret;
		}
		if (((val & mask) != 0U) == set) {
			return 0;
		}
		k_busy_wait(200);
	}
	return -ETIMEDOUT;
}

static void phy_wr_cfg(const struct device *grf, uint32_t addr, uint32_t data)
{
	(void)syscon_write_reg(grf, GRF_PCIE_PHY_CONF,
			       RK_WM16(PHY_CFG_DATA, data) | RK_WM16(PHY_CFG_ADDR, addr));
	k_busy_wait(1);
	(void)syscon_write_reg(grf, GRF_PCIE_PHY_CONF, RK_WM16(PHY_CFG_WR, 1U));
	k_busy_wait(1);
	(void)syscon_write_reg(grf, GRF_PCIE_PHY_CONF, RK_WM16(PHY_CFG_WR, 0U));
}

static int phy_power_on(const struct rk3399_pcie_port_cfg *cfg)
{
	const struct device *grf = cfg->grf;
	int ret;

	for (uint32_t i = 0U; i < 4U; i++) {
		bool idle = (i >= cfg->lanes);

		ret = syscon_write_reg(grf, GRF_PCIE_PHY_LANEOFF,
				       ((PHY_LANE_IDLE << 16) | (idle ? PHY_LANE_IDLE : 0U)) << i);
		if (ret != 0) {
			return ret;
		}
	}

	ret = reset_line_deassert_dt(&cfg->rsts[RK_RST_PHY]);
	if (ret != 0) {
		return ret;
	}

	(void)syscon_write_reg(grf, GRF_PCIE_PHY_CONF, RK_WM16(PHY_CFG_ADDR, PHY_CFG_PLL_LOCK));
	ret = grf_poll(grf, GRF_PCIE_PHY_STATUS, PHY_PLL_LOCKED, true);
	if (ret != 0) {
		LOG_ERR("PHY PLL lock timeout");
		return ret;
	}

	phy_wr_cfg(grf, PHY_CFG_CLK_TEST, PHY_CFG_SEPE_RATE);
	phy_wr_cfg(grf, PHY_CFG_CLK_SCC, PHY_CFG_PLL_100M);

	ret = grf_poll(grf, GRF_PCIE_PHY_STATUS, PHY_PLL_OUTPUT, false);
	if (ret != 0) {
		LOG_ERR("PHY PLL output enable timeout");
		return ret;
	}

	(void)syscon_write_reg(grf, GRF_PCIE_PHY_CONF, RK_WM16(PHY_CFG_ADDR, PHY_CFG_PLL_LOCK));
	ret = grf_poll(grf, GRF_PCIE_PHY_STATUS, PHY_PLL_LOCKED, true);
	if (ret != 0) {
		LOG_ERR("PHY PLL relock timeout");
	}
	return ret;
}

static int resets(const struct rk3399_pcie_port_cfg *cfg, const enum rk3399_pcie_rst *ids,
		  size_t n, bool assert)
{
	for (size_t i = 0U; i < n; i++) {
		int ret = assert ? reset_line_assert_dt(&cfg->rsts[ids[i]])
				 : reset_line_deassert_dt(&cfg->rsts[ids[i]]);

		if (ret != 0) {
			return ret;
		}
	}
	return 0;
}

int rk3399_pcie_port_prepare(const struct rk3399_pcie_port_cfg *cfg)
{
	if (!device_is_ready(cfg->grf)) {
		return -ENODEV;
	}
	for (size_t i = 0U; i < RK_CLK_NUM; i++) {
		int ret;

		if (!device_is_ready(cfg->clks[i].dev)) {
			return -ENODEV;
		}
		ret = clock_control_on(cfg->clks[i].dev,
				       (clock_control_subsys_t)(uintptr_t)cfg->clks[i].id);
		if (ret != 0) {
			return ret;
		}
	}
	for (size_t i = 0U; i < RK_RST_NUM; i++) {
		if (!device_is_ready(cfg->rsts[i].dev)) {
			return -ENODEV;
		}
	}
	return 0;
}

int rk3399_pcie_port_init(const struct rk3399_pcie_port_cfg *cfg, mm_reg_t apb, bool rc)
{
	static const enum rk3399_pcie_rst pm_rsts[] = {RK_RST_PM, RK_RST_PCLK, RK_RST_ACLK};
	/* Deassert order matters. */
	static const enum rk3399_pcie_rst core_rsts[] = {RK_RST_PIPE, RK_RST_MGMT, RK_RST_CORE,
							 RK_RST_MGMT_STICKY};
	uint32_t elapsed = 0U;
	int ret;

	ret = resets(cfg, pm_rsts, ARRAY_SIZE(pm_rsts), true);
	if (ret == 0) {
		ret = reset_line_assert_dt(&cfg->rsts[RK_RST_PHY]);
	}
	if (ret == 0) {
		ret = resets(cfg, core_rsts, ARRAY_SIZE(core_rsts), true);
	}
	if (ret != 0) {
		return ret;
	}
	k_busy_wait(10);
	ret = resets(cfg, pm_rsts, ARRAY_SIZE(pm_rsts), false);
	if (ret != 0) {
		return ret;
	}

	rk_client_write(apb, RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_GEN_SEL_2, (cfg->link_gen > 1U) ? 1U : 0U));
	rk_client_write(apb, RK_CLIENT_CONFIG,
			RK_WM16(RK_CLIENT_ARI_ENABLE, 1U) |
				RK_WM16(RK_CLIENT_SR_IOV_ENABLE,
					(!rc && IS_ENABLED(CONFIG_PCIE_EPC_RK3399_SRIOV)) ? 1U : 0U) |
				RK_WM16(RK_CLIENT_LANE_NUM, ((uint32_t)cfg->lanes >> 1) & 3U) |
				RK_WM16(RK_CLIENT_LINK_TRAIN, 0U) |
				RK_WM16(RK_CLIENT_CONF_ENABLE, 0U) |
				RK_WM16(RK_CLIENT_MODE_RC, rc ? 1U : 0U));

	ret = phy_power_on(cfg);
	if (ret != 0) {
		return ret;
	}

	while ((rk_client_read(apb, RK_CLIENT_SIDE_BAND_STATUS) & RK_CLIENT_PHY_ST) != 0U) {
		if (elapsed >= PHY_TIMEOUT_US) {
			LOG_ERR("PHY PLLs did not lock");
			return -ETIMEDOUT;
		}
		k_busy_wait(1000);
		elapsed += 1000U;
	}

	return resets(cfg, core_rsts, ARRAY_SIZE(core_rsts), false);
}

const char *rk3399_pcie_ltssm_name(uint8_t s)
{
	static const char *const names[] = {
		[0x00] = "detect-quiet",       [0x01] = "detect-active",
		[0x02] = "polling-active",     [0x03] = "polling-compliance",
		[0x04] = "polling-config",     [0x05] = "config-linkwidth-start",
		[0x06] = "config-linkwidth-accept", [0x07] = "config-lanenum-accept",
		[0x08] = "config-lanenum-wait", [0x09] = "config-complete",
		[0x0a] = "config-idle",        [0x0b] = "recovery-rcvr-lock",
		[0x0c] = "recovery-speed",     [0x0d] = "recovery-rcvr-cfg",
		[0x0e] = "recovery-idle",      [0x10] = "L0",
		[0x11] = "rx-L0s-entry",       [0x12] = "rx-L0s-idle",
		[0x13] = "rx-L0s-fts",         [0x14] = "tx-L0s-entry",
		[0x15] = "tx-L0s-idle",        [0x16] = "tx-L0s-fts",
		[0x17] = "L1-entry",           [0x18] = "L1-idle",
		[0x19] = "L2-idle",            [0x1a] = "L2-transmit-wake",
		[0x20] = "disabled",           [0x21] = "loopback-entry-master",
		[0x22] = "loopback-active-master", [0x23] = "loopback-exit-master",
		[0x24] = "loopback-entry-slave", [0x25] = "loopback-active-slave",
		[0x26] = "loopback-exit-slave", [0x27] = "hot-reset",
	};

	return (s < ARRAY_SIZE(names) && names[s] != NULL) ? names[s] : "?";
}

void rk3399_pcie_link_info(mm_reg_t apb, unsigned int *gen, unsigned int *width)
{
	uint32_t v = rk_client_read(apb, RK_CLIENT_BASIC_STATUS0);

	*gen = (v & RK_CLIENT_NEG_LINK_SPEED) ? 2U : 1U;
	*width = 1U << FIELD_GET(RK_CLIENT_NEG_LINK_WIDTH, v);
}
