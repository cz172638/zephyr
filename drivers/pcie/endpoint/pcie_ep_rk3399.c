/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3399 PCIe endpoint controller: Cadence PCIe core behind the
 * Rockchip client wrapper, PHY in the GRF.
 *
 * Register sequences follow Linux drivers/pci/controller/pcie-rockchip*.c
 * and drivers/phy/rockchip/phy-rockchip-pcie.c.
 */

#define DT_DRV_COMPAT rockchip_rk3399_pcie_ep

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep.h>
#include <zephyr/drivers/pcie/endpoint/pcie_ep_cdns.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "pcie_ep_cdns.h"

LOG_MODULE_REGISTER(pcie_ep_rk3399, CONFIG_PCIE_EP_LOG_LEVEL);

/* Write-masked 16-bit fields: upper half-word selects the bits to write */
#define WM16(mask, val)	((((uint32_t)(val) << (__builtin_ctz(mask))) & (mask)) | ((mask) << 16))

/* Client registers, apb + 0 */
#define CLIENT_CONFIG			0x00U
#define  CLIENT_CONF_ENABLE		BIT(0)
#define  CLIENT_LINK_TRAIN_ENABLE	BIT(1)
#define  CLIENT_ARI_ENABLE		BIT(3)
#define  CLIENT_LANE_NUM		GENMASK(5, 4)
#define  CLIENT_MODE_RC			BIT(6)
#define  CLIENT_GEN_SEL_2		BIT(7)
#define CLIENT_LEGACY_INT_CTRL		0x0cU
#define  CLIENT_INT_PEND_ST		BIT(0)
#define  CLIENT_INT_IN			BIT(1)
#define CLIENT_SIDE_BAND_STATUS		0x20U
#define  CLIENT_PHY_ST			BIT(12)
#define CLIENT_BASIC_STATUS0		0x44U
#define  CLIENT_NEG_LINK_WIDTH		GENMASK(7, 6)
#define  CLIENT_NEG_LINK_SPEED		BIT(5)
#define CLIENT_BASIC_STATUS1		0x48U
#define  CLIENT_LINK_STATUS_MASK	GENMASK(21, 20)
#define  CLIENT_LINK_STATUS_UP		GENMASK(21, 20)

/* Cadence core inside the apb region */
#define RK3399_CORE_OFFSET		0x800000U

/* PCIe PHY in the GRF */
#define GRF_PCIE_PHY_LANEOFF		0xe214U
#define  PHY_LANE_IDLE			BIT(3)
#define GRF_PCIE_PHY_CONF		0xe220U
#define  PHY_CFG_WR			BIT(0)
#define  PHY_CFG_ADDR			GENMASK(6, 1)
#define  PHY_CFG_DATA			GENMASK(10, 7)
#define GRF_PCIE_PHY_STATUS		0xe2a4U
#define  PHY_PLL_LOCKED			BIT(9)
#define  PHY_PLL_OUTPUT			BIT(10)
#define PHY_CFG_PLL_LOCK		0x10U
#define PHY_CFG_CLK_TEST		0x10U
#define PHY_CFG_CLK_SCC			0x12U
#define PHY_CFG_SEPE_RATE		BIT(3)
#define PHY_CFG_PLL_100M		BIT(3)

/* PCI configuration space offsets */
#define PCI_COMMAND_REG			0x04U
#define  PCI_COMMAND_INTX_DISABLE	BIT(10)
#define PCI_EXP_LNKCTL			0x10U
#define  PCI_EXP_LNKCTL_RL		BIT(5)

#define RK3399_OB_REGIONS		32U
#define RK3399_OB_REGION_SIZE		MB(1)
#define RK3399_OB_IRQ_REGION		0U
#define RK3399_LINK_POLL_MS		5
#define RK3399_LINK_TRAIN_US		500000U
#define RK3399_PHY_TIMEOUT_US		100000U

enum rk3399_rst {
	RST_CORE,
	RST_MGMT,
	RST_MGMT_STICKY,
	RST_PIPE,
	RST_PM,
	RST_PCLK,
	RST_ACLK,
	RST_PHY,
	RST_NUM,
};

enum rk3399_clk {
	CLK_ACLK,
	CLK_ACLK_PERF,
	CLK_HCLK,
	CLK_PM,
	CLK_NUM,
};

struct rk3399_clk_spec {
	const struct device *dev;
	uint32_t id;
};

struct pcie_ep_rk3399_config {
	DEVICE_MMIO_NAMED_ROM(apb);
	uintptr_t mem_base;
	size_t mem_size;
	struct rk3399_clk_spec clks[CLK_NUM];
	struct reset_dt_spec rsts[RST_NUM];
	const struct device *grf;
	struct gpio_dt_spec perst;
	struct pcie_ep_cdns_header hdr;
	bool has_ids;
	uint8_t lanes;
	uint8_t link_gen;
	uint8_t max_functions;
	uint8_t msi_vectors;
};

struct pcie_ep_rk3399_data {
	DEVICE_MMIO_NAMED_RAM(apb);
	struct cdns_ep_core core;
	const struct device *dev;
	struct k_spinlock lock;
	uint32_t ob_map;
	mm_reg_t irq_va;
	uint64_t irq_pci_addr;
	uint8_t irq_pci_fn;
	struct k_work_delayable link_work;
	struct gpio_callback perst_cb;
	bool initialized;
	bool started;
	bool perst_asserted;
	bool link_up;
	uint32_t train_elapsed_us;
	pcie_ep_reset_callback_t reset_cb[PCIE_RESET_MAX];
	void *reset_arg[PCIE_RESET_MAX];
};

/* Required by DEVICE_MMIO_NAMED_* macros */
#define DEV_CFG(_dev)  ((const struct pcie_ep_rk3399_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct pcie_ep_rk3399_data *)(_dev)->data)

static const struct cdns_ep_variant rk3399_cdns_variant = {
	.ob_region_offset = 0x20U,
	.ib_bar_offset = 0x828U,
	.ob_max_nbits = 20U,
	.ob_has_cpu_addr = false,
	.did_via_rp_bank = true,
};

static inline uint32_t client_read(const struct device *dev, uint32_t reg)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, apb) + reg);
}

static inline void client_write(const struct device *dev, uint32_t reg, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, apb) + reg);
}

static int grf_write(const struct device *dev, uint32_t reg, uint32_t val)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;

	return syscon_write_reg(cfg->grf, reg, val);
}

static int grf_poll(const struct device *dev, uint32_t reg, uint32_t mask, bool set)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	uint32_t elapsed = 0U;
	uint32_t val;
	int ret;

	do {
		ret = syscon_read_reg(cfg->grf, reg, &val);
		if (ret != 0) {
			return ret;
		}

		if (((val & mask) != 0U) == set) {
			return 0;
		}

		k_busy_wait(200);
		elapsed += 200U;
	} while (elapsed < RK3399_PHY_TIMEOUT_US);

	return -ETIMEDOUT;
}

static void phy_wr_cfg(const struct device *dev, uint32_t addr, uint32_t data)
{
	(void)grf_write(dev, GRF_PCIE_PHY_CONF,
			WM16(PHY_CFG_DATA, data) | WM16(PHY_CFG_ADDR, addr));
	k_busy_wait(1);
	(void)grf_write(dev, GRF_PCIE_PHY_CONF, WM16(PHY_CFG_WR, 1U));
	k_busy_wait(1);
	(void)grf_write(dev, GRF_PCIE_PHY_CONF, WM16(PHY_CFG_WR, 0U));
}

static int rk3399_phy_power_on(const struct device *dev)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	int ret;

	for (uint32_t i = 0U; i < 4U; i++) {
		bool idle = (i >= cfg->lanes);

		ret = grf_write(dev, GRF_PCIE_PHY_LANEOFF,
				((PHY_LANE_IDLE << 16) | (idle ? PHY_LANE_IDLE : 0U)) << i);
		if (ret != 0) {
			return ret;
		}
	}

	ret = reset_line_deassert_dt(&cfg->rsts[RST_PHY]);
	if (ret != 0) {
		return ret;
	}

	(void)grf_write(dev, GRF_PCIE_PHY_CONF, WM16(PHY_CFG_ADDR, PHY_CFG_PLL_LOCK));
	ret = grf_poll(dev, GRF_PCIE_PHY_STATUS, PHY_PLL_LOCKED, true);
	if (ret != 0) {
		LOG_ERR("PHY PLL lock timeout");
		return ret;
	}

	phy_wr_cfg(dev, PHY_CFG_CLK_TEST, PHY_CFG_SEPE_RATE);
	phy_wr_cfg(dev, PHY_CFG_CLK_SCC, PHY_CFG_PLL_100M);

	ret = grf_poll(dev, GRF_PCIE_PHY_STATUS, PHY_PLL_OUTPUT, false);
	if (ret != 0) {
		LOG_ERR("PHY PLL output enable timeout");
		return ret;
	}

	(void)grf_write(dev, GRF_PCIE_PHY_CONF, WM16(PHY_CFG_ADDR, PHY_CFG_PLL_LOCK));
	ret = grf_poll(dev, GRF_PCIE_PHY_STATUS, PHY_PLL_LOCKED, true);
	if (ret != 0) {
		LOG_ERR("PHY PLL relock timeout");
	}

	return ret;
}

static int rk3399_resets(const struct device *dev, const enum rk3399_rst *ids, size_t n,
			 bool assert)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	int ret;

	for (size_t i = 0U; i < n; i++) {
		ret = assert ? reset_line_assert_dt(&cfg->rsts[ids[i]])
			     : reset_line_deassert_dt(&cfg->rsts[ids[i]]);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int rk3399_init_port(const struct device *dev)
{
	static const enum rk3399_rst pm_rsts[] = {RST_PM, RST_PCLK, RST_ACLK};
	/* Deassert order matters. */
	static const enum rk3399_rst core_rsts[] = {RST_PIPE, RST_MGMT, RST_CORE,
						    RST_MGMT_STICKY};
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	uint32_t elapsed = 0U;
	uint32_t val;
	int ret;

	ret = rk3399_resets(dev, pm_rsts, ARRAY_SIZE(pm_rsts), true);
	if (ret != 0) {
		return ret;
	}

	ret = reset_line_assert_dt(&cfg->rsts[RST_PHY]);
	if (ret != 0) {
		return ret;
	}

	ret = rk3399_resets(dev, core_rsts, ARRAY_SIZE(core_rsts), true);
	if (ret != 0) {
		return ret;
	}

	k_busy_wait(10);

	ret = rk3399_resets(dev, pm_rsts, ARRAY_SIZE(pm_rsts), false);
	if (ret != 0) {
		return ret;
	}

	client_write(dev, CLIENT_CONFIG, WM16(CLIENT_GEN_SEL_2, (cfg->link_gen > 1U) ? 1U : 0U));
	client_write(dev, CLIENT_CONFIG,
		     WM16(CLIENT_ARI_ENABLE, 1U) |
			     WM16(CLIENT_LANE_NUM, ((uint32_t)cfg->lanes >> 1) & 3U) |
			     WM16(CLIENT_CONF_ENABLE, 0U) | WM16(CLIENT_MODE_RC, 0U));

	ret = rk3399_phy_power_on(dev);
	if (ret != 0) {
		return ret;
	}

	while ((client_read(dev, CLIENT_SIDE_BAND_STATUS) & CLIENT_PHY_ST) != 0U) {
		if (elapsed >= RK3399_PHY_TIMEOUT_US) {
			LOG_ERR("PHY PLLs did not lock");
			return -ETIMEDOUT;
		}
		k_busy_wait(1000);
		elapsed += 1000U;
	}

	ret = rk3399_resets(dev, core_rsts, ARRAY_SIZE(core_rsts), false);
	if (ret != 0) {
		return ret;
	}

	val = cdns_ep_read(&((struct pcie_ep_rk3399_data *)dev->data)->core, CDNS_LM_ID);
	LOG_INF("core up, reset VID/SSVID 0x%08x", val);

	return 0;
}

static bool rk3399_link_is_up(const struct device *dev)
{
	return (client_read(dev, CLIENT_BASIC_STATUS1) & CLIENT_LINK_STATUS_MASK) ==
	       CLIENT_LINK_STATUS_UP;
}

static void rk3399_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct pcie_ep_rk3399_data *data =
		CONTAINER_OF(dwork, struct pcie_ep_rk3399_data, link_work);
	const struct device *dev = data->dev;
	uint32_t val;

	if (data->perst_asserted) {
		return;
	}

	if (((cdns_ep_read(&data->core, CDNS_LM_PL_CONFIG0) & CDNS_LM_PL_CONFIG0_LS_READY) ==
	     0U) ||
	    !rk3399_link_is_up(dev)) {
		data->train_elapsed_us += RK3399_LINK_POLL_MS * 1000U;
		if (data->train_elapsed_us >= RK3399_LINK_TRAIN_US) {
			LOG_DBG("link training still in progress");
			data->train_elapsed_us = 0U;
		}
		(void)k_work_reschedule(&data->link_work, K_MSEC(RK3399_LINK_POLL_MS));
		return;
	}

	val = client_read(dev, CLIENT_BASIC_STATUS0);
	LOG_INF("link up, %s GT/s x%u", ((val & CLIENT_NEG_LINK_SPEED) != 0U) ? "5" : "2.5",
		(unsigned int)(FIELD_GET(CLIENT_NEG_LINK_WIDTH, val) << 1));
	data->link_up = true;
}

static void rk3399_retrain_link(struct pcie_ep_rk3399_data *data)
{
	uint32_t reg = CDNS_RP_BASE + CDNS_EP_FUNC_PCIE_CAP + PCI_EXP_LNKCTL;

	cdns_ep_write(&data->core, reg, cdns_ep_read(&data->core, reg) | PCI_EXP_LNKCTL_RL);
}

/* Rockchip GPIO v1 has no both-edge trigger: arm the opposite edge each time. */
static void rk3399_perst_arm(const struct pcie_ep_rk3399_config *cfg, bool asserted)
{
	(void)gpio_pin_interrupt_configure_dt(&cfg->perst, asserted ? GPIO_INT_EDGE_TO_INACTIVE
								  : GPIO_INT_EDGE_TO_ACTIVE);
}

static void rk3399_perst_handler(const struct device *port, struct gpio_callback *cb,
				 gpio_port_pins_t pins)
{
	struct pcie_ep_rk3399_data *data =
		CONTAINER_OF(cb, struct pcie_ep_rk3399_data, perst_cb);
	const struct pcie_ep_rk3399_config *cfg = data->dev->config;
	int asserted = gpio_pin_get_dt(&cfg->perst);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	if (asserted < 0) {
		return;
	}

	rk3399_perst_arm(cfg, asserted != 0);

	if (asserted != 0) {
		if (data->perst_asserted) {
			return;
		}
		data->perst_asserted = true;
		data->link_up = false;
		(void)k_work_cancel_delayable(&data->link_work);
		if (data->reset_cb[PCIE_PERST] != NULL) {
			data->reset_cb[PCIE_PERST](data->reset_arg[PCIE_PERST]);
		}
		return;
	}

	if (!data->perst_asserted) {
		return;
	}

	data->perst_asserted = false;
	if (data->started) {
		rk3399_retrain_link(data);
		data->train_elapsed_us = 0U;
		(void)k_work_reschedule(&data->link_work, K_NO_WAIT);
	}
}

/* Generic PCIe endpoint API */

static int pcie_ep_rk3399_conf_read(const struct device *dev, uint32_t offset, uint32_t *val)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U) {
		return -EINVAL;
	}

	*val = cdns_ep_read(&data->core, CDNS_EP_FUNC_BASE(0) + offset);

	return 0;
}

static void pcie_ep_rk3399_conf_write(const struct device *dev, uint32_t offset, uint32_t val)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	if ((offset & 3U) != 0U || offset >= 0x1000U) {
		return;
	}

	cdns_ep_write(&data->core, CDNS_EP_FUNC_BASE(0) + offset, val);
}

static int pcie_ep_rk3399_map_addr(const struct device *dev, uint64_t pcie_addr,
				   uint64_t *mapped_addr, uint32_t size,
				   enum pcie_ob_mem_type ob_mem_type)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = dev->data;
	k_spinlock_key_t key;
	uint64_t offset;
	uint64_t avail;
	uint32_t region;
	uint8_t nbits;

	/* The AXI window sits below 4 GiB; host addresses are not restricted. */
	if ((ob_mem_type == PCIE_OB_HIGHMEM) || (size == 0U)) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

	for (region = 1U; region < RK3399_OB_REGIONS; region++) {
		if ((data->ob_map & BIT(region)) == 0U) {
			break;
		}
	}

	if (region == RK3399_OB_REGIONS) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	nbits = cdns_ep_ob_nbits(&data->core, pcie_addr, size);
	offset = pcie_addr & (BIT64(nbits) - 1U);
	avail = BIT64(nbits) - offset;

	cdns_ep_ob_program(&data->core, 0, region, cfg->mem_base + (region * RK3399_OB_REGION_SIZE),
			   pcie_addr - offset, nbits);
	data->ob_map |= BIT(region);

	k_spin_unlock(&data->lock, key);

	*mapped_addr = cfg->mem_base + (region * RK3399_OB_REGION_SIZE) + offset;

	return (int)MIN((uint64_t)size, avail);
}

static void pcie_ep_rk3399_unmap_addr(const struct device *dev, uint64_t mapped_addr)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t region;

	if ((mapped_addr < cfg->mem_base) || (mapped_addr >= (cfg->mem_base + cfg->mem_size))) {
		return;
	}

	region = (uint32_t)((mapped_addr - cfg->mem_base) / RK3399_OB_REGION_SIZE);
	if ((region == RK3399_OB_IRQ_REGION) || (region >= RK3399_OB_REGIONS)) {
		return;
	}

	key = k_spin_lock(&data->lock);
	if ((data->ob_map & BIT(region)) != 0U) {
		cdns_ep_ob_clear(&data->core, region);
		data->ob_map &= ~BIT(region);
	}
	k_spin_unlock(&data->lock, key);
}

static int rk3399_raise_intx(const struct device *dev)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	if ((cdns_ep_read(&data->core, CDNS_EP_FUNC_BASE(0) + PCI_COMMAND_REG) &
	     PCI_COMMAND_INTX_DISABLE) != 0U) {
		return -EINVAL;
	}

	/* The TRM asks for some AHB clock cycles between edges; 1 ms is ample. */
	client_write(dev, CLIENT_LEGACY_INT_CTRL,
		     WM16(CLIENT_INT_IN, 1U) | WM16(CLIENT_INT_PEND_ST, 1U));
	k_busy_wait(1000);
	client_write(dev, CLIENT_LEGACY_INT_CTRL,
		     WM16(CLIENT_INT_IN, 0U) | WM16(CLIENT_INT_PEND_ST, 0U));

	return 0;
}

static int rk3399_raise_msi(const struct device *dev, uint8_t fn, uint32_t irq_num)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = dev->data;
	k_spinlock_key_t key;
	uint64_t pci_addr;
	uint64_t base;
	uint16_t msi_data;
	int ret;

	ret = cdns_ep_msi_target(&data->core, fn, irq_num, &pci_addr, &msi_data);
	if (ret != 0) {
		return ret;
	}

	base = pci_addr & ~(uint64_t)0xffU;

	key = k_spin_lock(&data->lock);
	if ((data->irq_pci_addr != base) || (data->irq_pci_fn != fn)) {
		cdns_ep_ob_program(&data->core, fn, RK3399_OB_IRQ_REGION, cfg->mem_base, base, 8U);
		data->irq_pci_addr = base;
		data->irq_pci_fn = fn;
	}
	sys_write16(msi_data, data->irq_va + (mm_reg_t)(pci_addr & 0xffU));
	k_spin_unlock(&data->lock, key);

	return 0;
}

static int pcie_ep_rk3399_raise_irq(const struct device *dev, enum pci_ep_irq_type irq_type,
				    uint32_t irq_num)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	if (!data->link_up) {
		return -ENOTCONN;
	}

	switch (irq_type) {
	case PCIE_EP_IRQ_LEGACY:
		return rk3399_raise_intx(dev);
	case PCIE_EP_IRQ_MSI:
		return rk3399_raise_msi(dev, 0, irq_num);
	default:
		/* The MSI-X capability is present but does not work. */
		return -ENOTSUP;
	}
}

static int pcie_ep_rk3399_register_reset_cb(const struct device *dev, enum pcie_reset reset,
					    pcie_ep_reset_callback_t cb, void *arg)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	if (reset != PCIE_PERST) {
		return -ENOTSUP;
	}

	data->reset_cb[reset] = cb;
	data->reset_arg[reset] = arg;

	return 0;
}

static DEVICE_API(pcie_ep, pcie_ep_rk3399_api) = {
	.conf_read = pcie_ep_rk3399_conf_read,
	.conf_write = pcie_ep_rk3399_conf_write,
	.map_addr = pcie_ep_rk3399_map_addr,
	.unmap_addr = pcie_ep_rk3399_unmap_addr,
	.raise_irq = pcie_ep_rk3399_raise_irq,
	.register_reset_cb = pcie_ep_rk3399_register_reset_cb,
};

/* Cadence extensions */

static struct pcie_ep_rk3399_data *rk3399_ready_data(const struct device *dev)
{
	struct pcie_ep_rk3399_data *data = dev->data;

	return data->initialized ? data : NULL;
}

int pcie_ep_cdns_write_header(const struct device *dev, uint8_t fn,
			      const struct pcie_ep_cdns_header *hdr)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	if ((data == NULL) || (fn >= cfg->max_functions)) {
		return -EINVAL;
	}

	return cdns_ep_write_header(&data->core, fn, hdr);
}

int pcie_ep_cdns_set_bar(const struct device *dev, uint8_t fn, uint8_t bar, uintptr_t phys_addr,
			 uint64_t size, uint32_t flags)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	if ((data == NULL) || (fn >= cfg->max_functions)) {
		return -EINVAL;
	}

	return cdns_ep_set_bar(&data->core, fn, bar, phys_addr, size, flags);
}

int pcie_ep_cdns_clear_bar(const struct device *dev, uint8_t fn, uint8_t bar)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	if ((data == NULL) || (fn >= cfg->max_functions)) {
		return -EINVAL;
	}

	return cdns_ep_clear_bar(&data->core, fn, bar);
}

int pcie_ep_cdns_set_msi(const struct device *dev, uint8_t fn, uint8_t nr_vectors)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	if ((data == NULL) || (fn >= cfg->max_functions)) {
		return -EINVAL;
	}

	return cdns_ep_set_msi(&data->core, fn, nr_vectors);
}

int pcie_ep_cdns_start(const struct device *dev, uint32_t fn_mask)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);
	int ret;

	if (data == NULL) {
		return -EIO;
	}

	cdns_ep_write(&data->core, CDNS_LM_EP_FUNC_CFG,
		      (fn_mask | BIT(0)) & BIT_MASK(cfg->max_functions));

	data->started = true;
	data->train_elapsed_us = 0U;

	client_write(dev, CLIENT_CONFIG,
		     WM16(CLIENT_LINK_TRAIN_ENABLE, 1U) | WM16(CLIENT_CONF_ENABLE, 1U));

	if (cfg->perst.port != NULL) {
		ret = gpio_pin_get_dt(&cfg->perst);
		if (ret < 0) {
			return ret;
		}

		data->perst_asserted = (ret != 0);
		rk3399_perst_arm(cfg, data->perst_asserted);
		if (data->perst_asserted) {
			LOG_INF("waiting for PERST# deassertion");
			return 0;
		}
	}

	(void)k_work_reschedule(&data->link_work, K_NO_WAIT);

	return 0;
}

void pcie_ep_cdns_stop(const struct device *dev)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	if (data == NULL) {
		return;
	}

	if (cfg->perst.port != NULL) {
		(void)gpio_pin_interrupt_configure_dt(&cfg->perst, GPIO_INT_DISABLE);
	}

	data->started = false;
	data->link_up = false;
	(void)k_work_cancel_delayable(&data->link_work);

	client_write(dev, CLIENT_CONFIG,
		     WM16(CLIENT_LINK_TRAIN_ENABLE, 0U) | WM16(CLIENT_CONF_ENABLE, 0U));
}

bool pcie_ep_cdns_link_up(const struct device *dev)
{
	struct pcie_ep_rk3399_data *data = rk3399_ready_data(dev);

	return (data != NULL) && data->link_up;
}

static int pcie_ep_rk3399_init(const struct device *dev)
{
	const struct pcie_ep_rk3399_config *cfg = dev->config;
	struct pcie_ep_rk3399_data *data = dev->data;
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, apb, K_MEM_CACHE_NONE);

	data->dev = dev;
	data->core.base = DEVICE_MMIO_NAMED_GET(dev, apb) + RK3399_CORE_OFFSET;
	data->core.variant = &rk3399_cdns_variant;
	data->irq_pci_addr = UINT64_MAX;
	data->ob_map = BIT(RK3399_OB_IRQ_REGION);
	k_work_init_delayable(&data->link_work, rk3399_link_work);

	if (!device_is_ready(cfg->grf)) {
		return -ENODEV;
	}

	for (size_t i = 0U; i < CLK_NUM; i++) {
		if (!device_is_ready(cfg->clks[i].dev)) {
			return -ENODEV;
		}

		ret = clock_control_on(cfg->clks[i].dev,
				       (clock_control_subsys_t)(uintptr_t)cfg->clks[i].id);
		if (ret != 0) {
			return ret;
		}
	}

	for (size_t i = 0U; i < RST_NUM; i++) {
		if (!device_is_ready(cfg->rsts[i].dev)) {
			return -ENODEV;
		}
	}

	ret = rk3399_init_port(dev);
	if (ret != 0) {
		LOG_ERR("port init failed: %d", ret);
		return ret;
	}

	/* MSI-X is advertised but not functional on this revision. */
	cdns_ep_hide_msix_cap(&data->core);
	cdns_ep_write(&data->core, CDNS_LM_EP_FUNC_CFG, BIT(0));

	if (cfg->has_ids) {
		ret = cdns_ep_write_header(&data->core, 0, &cfg->hdr);
		if (ret != 0) {
			return ret;
		}
	}

	ret = cdns_ep_set_msi(&data->core, 0, cfg->msi_vectors);
	if (ret != 0) {
		return ret;
	}

	device_map(&data->irq_va, cfg->mem_base + (RK3399_OB_IRQ_REGION * RK3399_OB_REGION_SIZE),
		   RK3399_OB_REGION_SIZE, K_MEM_CACHE_NONE);

	if (cfg->perst.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->perst)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->perst, GPIO_INPUT);
		if (ret != 0) {
			return ret;
		}

		gpio_init_callback(&data->perst_cb, rk3399_perst_handler, BIT(cfg->perst.pin));
		ret = gpio_add_callback_dt(&cfg->perst, &data->perst_cb);
		if (ret != 0) {
			return ret;
		}
	}

	data->initialized = true;

	return 0;
}

#define RK3399_CLK(n, name)                                                                        \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, name)),                        \
		.id = DT_INST_CLOCKS_CELL_BY_NAME(n, name, clkid),                                 \
	}

#define RK3399_RST(n, name)                                                                        \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(n, name)),                         \
		.id = DT_INST_RESET_CELL_BY_NAME(n, name, id),                                     \
	}

#define PCIE_EP_RK3399_INIT(n)                                                                     \
	BUILD_ASSERT(DT_INST_PROP(n, max_functions) >= 1 &&                                        \
		     DT_INST_PROP(n, max_functions) <= CDNS_EP_MAX_FUNCS);                         \
	static const struct pcie_ep_rk3399_config pcie_ep_rk3399_config_##n = {                   \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(apb, DT_DRV_INST(n)),                           \
		.mem_base = DT_INST_REG_ADDR_BY_NAME(n, mem),                                 \
		.mem_size = DT_INST_REG_SIZE_BY_NAME(n, mem),                                 \
		.clks = {                                                                          \
			[CLK_ACLK] = RK3399_CLK(n, aclk),                                          \
			[CLK_ACLK_PERF] = RK3399_CLK(n, aclk_perf),                                \
			[CLK_HCLK] = RK3399_CLK(n, hclk),                                          \
			[CLK_PM] = RK3399_CLK(n, pm),                                              \
		},                                                                                 \
		.rsts = {                                                                          \
			[RST_CORE] = RK3399_RST(n, core),                                          \
			[RST_MGMT] = RK3399_RST(n, mgmt),                                          \
			[RST_MGMT_STICKY] = RK3399_RST(n, mgmt_sticky),                            \
			[RST_PIPE] = RK3399_RST(n, pipe),                                          \
			[RST_PM] = RK3399_RST(n, pm),                                              \
			[RST_PCLK] = RK3399_RST(n, pclk),                                          \
			[RST_ACLK] = RK3399_RST(n, aclk),                                          \
			[RST_PHY] = RK3399_RST(n, phy),                                            \
		},                                                                                 \
		.grf = DEVICE_DT_GET(DT_INST_PHANDLE(n, rockchip_grf)),                            \
		.perst = GPIO_DT_SPEC_INST_GET_OR(n, perst_gpios, {0}),                            \
		.has_ids = DT_INST_NODE_HAS_PROP(n, vendor_id) &&                                  \
			   DT_INST_NODE_HAS_PROP(n, device_id),                                    \
		.hdr = {                                                                           \
			.vendor_id = DT_INST_PROP_OR(n, vendor_id, 0),                             \
			.device_id = DT_INST_PROP_OR(n, device_id, 0),                             \
			.class_code = DT_INST_PROP(n, class_code),                                 \
			.subsys_vendor_id = DT_INST_PROP(n, subsystem_vendor_id),                  \
			.subsys_id = DT_INST_PROP(n, subsystem_id),                                \
			.interrupt_pin = 1,                                                        \
		},                                                                                 \
		.lanes = DT_INST_PROP(n, num_lanes),                                               \
		.link_gen = DT_INST_PROP(n, max_link_speed),                                       \
		.max_functions = DT_INST_PROP(n, max_functions),                                   \
		.msi_vectors = DT_INST_PROP(n, msi_vectors),                                       \
	};                                                                                         \
	static struct pcie_ep_rk3399_data pcie_ep_rk3399_data_##n;                                 \
	DEVICE_DT_INST_DEFINE(n, pcie_ep_rk3399_init, NULL, &pcie_ep_rk3399_data_##n,              \
			      &pcie_ep_rk3399_config_##n, POST_KERNEL,                             \
			      CONFIG_PCIE_EP_RK3399_INIT_PRIORITY, &pcie_ep_rk3399_api);

DT_INST_FOREACH_STATUS_OKAY(PCIE_EP_RK3399_INIT)
