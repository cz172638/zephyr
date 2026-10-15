/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PCIe Endpoint Function (EPF) helpers.
 *
 * An EPF is a Zephyr device described as a child node of a PCIe endpoint
 * controller node. The child's unit address is the physical function number:
 *
 * @code{.dts}
 * &pcie0_ep {
 *	status = "okay";
 *	#address-cells = <1>;
 *	#size-cells = <0>;
 *
 *	epf_test: function@0 {
 *		compatible = "zephyr,pcie-epf-test";
 *		reg = <0>;
 *		vendor-id = <0x104c>;
 *		device-id = <0xb00d>;
 *	};
 * };
 * @endcode
 *
 * The EPF driver initialises after its controller (CONFIG_PCIE_EPF_INIT_PRIORITY),
 * claims its function number, writes its header, sets its BARs, configures
 * interrupts and registers a pcie_epc_listener. The EPC core then starts the
 * link (unless the controller has `zephyr,no-autostart`).
 *
 * An EPF never touches controller registers; everything goes through
 * pcie_epc.h, which keeps the function portable across controllers.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPF_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPF_H_

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup pcie_epc_interface
 * @{
 */

/** @brief Controller device of the EPF devicetree node @p node_id. */
#define PCIE_EPF_DT_EPC(node_id) DEVICE_DT_GET(DT_PARENT(node_id))

/** @brief Physical function number of the EPF devicetree node @p node_id. */
#define PCIE_EPF_DT_FUNC(node_id) ((uint8_t)DT_REG_ADDR(node_id))

/** @brief Instance variants of the macros above. */
#define PCIE_EPF_DT_INST_EPC(inst)  PCIE_EPF_DT_EPC(DT_DRV_INST(inst))
#define PCIE_EPF_DT_INST_FUNC(inst) PCIE_EPF_DT_FUNC(DT_DRV_INST(inst))

/**
 * @brief Fill a pcie_epc_header from the standard EPF DT properties.
 *
 * Uses vendor-id, device-id, revision-id, class-code (24 bit: base, sub,
 * prog-if), subsystem-vendor-id, subsystem-id and interrupt-pin.
 */
#define PCIE_EPF_DT_HEADER(node_id)                                                                \
	{                                                                                          \
		.vendor_id = DT_PROP(node_id, vendor_id),                                          \
		.device_id = DT_PROP(node_id, device_id),                                          \
		.revision_id = DT_PROP_OR(node_id, revision_id, 0),                                \
		.baseclass = (DT_PROP_OR(node_id, class_code, 0xff0000) >> 16) & 0xff,             \
		.subclass = (DT_PROP_OR(node_id, class_code, 0) >> 8) & 0xff,                      \
		.prog_if = DT_PROP_OR(node_id, class_code, 0) & 0xff,                              \
		.subsys_vendor_id = DT_PROP_OR(node_id, subsystem_vendor_id, 0),                   \
		.subsys_id = DT_PROP_OR(node_id, subsystem_id, 0),                                 \
		.interrupt_pin = DT_PROP_OR(node_id, interrupt_pin, 1),                            \
	}

/**
 * @brief Pick a BAR size the controller can actually provide.
 *
 * Returns @p want rounded up to a power of two and to the fixed size if the
 * controller hard-wires that BAR, or 0 if the BAR is reserved.
 */
size_t pcie_epf_bar_size(const struct pcie_epc_features *feat, uint8_t barno, size_t want);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPF_H_ */
