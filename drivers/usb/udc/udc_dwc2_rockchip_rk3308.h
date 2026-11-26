/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Rockchip RK3308 OTG (DWC2) in device mode with the INNO USB2 PHY in the
 * USB2PHY GRF: OTG port out of suspend (PHY driven by the controller's
 * UTMI signals) and the 480 MHz clock output on, as Linux
 * phy-rockchip-inno-usb2 does. The controller clocks are left as they are:
 * they are running after reset and the maskrom boot uses this controller.
 * The B-session is overridden valid: the board is powered from this port.
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_ROCKCHIP_RK3308_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_ROCKCHIP_RK3308_H

#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <usb_dwc2_hw.h>

/* USB2PHY GRF: the upper half-word is the write mask of the lower one */
#define RK3308_U2PHY_OTG_SUS      0x0100U
#define RK3308_U2PHY_SUS_MASK     GENMASK(8, 0)
#define RK3308_U2PHY_CLKOUT_CTL   0x0108U
#define RK3308_U2PHY_CLKOUT_DIS   BIT(4)
#define RK3308_U2PHY_STATUS       0x0120U
#define RK3308_U2PHY_UTMI_BVALID  BIT(9)

#define RK3308_GOTGCTL_VBVAL_OVR  (BIT(2) | BIT(3))
#define RK3308_GOTGCTL_BVAL_OVR   (BIT(6) | BIT(7))

struct rk3308_usb_cfg {
	uintptr_t u2phy_grf;
	uint32_t ghwcfg[3];
};

static inline int rk3308_usb_pre_enable(const struct device *dev,
					const struct rk3308_usb_cfg *cfg)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mm_reg_t grf;
	uint32_t hw[3];

	device_map(&grf, cfg->u2phy_grf, 0x1000, K_MEM_CACHE_NONE);
	/* port out of suspend: UTMI controlled by the DWC2 */
	sys_write32(RK3308_U2PHY_SUS_MASK << 16, grf + RK3308_U2PHY_OTG_SUS);
	/* 480 MHz clock output on */
	sys_write32(RK3308_U2PHY_CLKOUT_DIS << 16, grf + RK3308_U2PHY_CLKOUT_CTL);
	k_busy_wait(2000);

	hw[0] = sys_read32((mem_addr_t)&base->ghwcfg1);
	hw[1] = sys_read32((mem_addr_t)&base->ghwcfg2);
	hw[2] = sys_read32((mem_addr_t)&base->ghwcfg4);
	if (hw[0] != cfg->ghwcfg[0] || hw[1] != cfg->ghwcfg[1] || hw[2] != cfg->ghwcfg[2]) {
		printk("dwc2: GHWCFG1/2/4 %08x %08x %08x (devicetree %08x %08x %08x), "
		       "GHWCFG3 %08x, GSNPSID %08x, PHY status %08x\n",
		       hw[0], hw[1], hw[2], cfg->ghwcfg[0], cfg->ghwcfg[1], cfg->ghwcfg[2],
		       sys_read32((mem_addr_t)&base->ghwcfg3),
		       sys_read32((mem_addr_t)&base->gsnpsid),
		       sys_read32(grf + RK3308_U2PHY_STATUS));
	}
	device_unmap(grf, 0x1000);
	return 0;
}

static inline int rk3308_usb_post_enable(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t gotgctl = (mem_addr_t)&base->gotgctl;

	sys_write32(sys_read32(gotgctl) | RK3308_GOTGCTL_VBVAL_OVR | RK3308_GOTGCTL_BVAL_OVR,
		    gotgctl);
	return 0;
}

static inline int rk3308_usb_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.hs = true;
	return 0;
}

#define QUIRK_RK3308_USB_DEFINE(n)						\
	static const struct rk3308_usb_cfg rk3308_usb_cfg_##n = {		\
		.u2phy_grf = DT_REG_ADDR(DT_INST_PHANDLE(n, rockchip_usb2phy_grf)), \
		.ghwcfg = {DT_INST_PROP(n, ghwcfg1), DT_INST_PROP(n, ghwcfg2),	\
			   DT_INST_PROP(n, ghwcfg4)},				\
	};									\
										\
	static int rk3308_usb_pre_enable_##n(const struct device *dev)		\
	{									\
		return rk3308_usb_pre_enable(dev, &rk3308_usb_cfg_##n);		\
	}									\
										\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {			\
		.caps = rk3308_usb_caps,					\
		.pre_enable = rk3308_usb_pre_enable_##n,			\
		.post_enable = rk3308_usb_post_enable,				\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_RK3308_USB_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_ROCKCHIP_RK3308_H */
