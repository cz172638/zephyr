/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Cadence PCIe endpoint controller extensions
 *
 * Function header, BAR and link control for endpoint controllers built on
 * the Cadence PCIe IP (Rockchip RK3399, TI K3). These complement the generic
 * PCIe endpoint API, which has no notion of BARs.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util_macro.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief BAR is I/O space instead of memory space */
#define PCIE_EP_CDNS_BAR_IO       BIT(0)
/** @brief BAR is 64-bit and consumes the next BAR as well */
#define PCIE_EP_CDNS_BAR_64BIT    BIT(1)
/** @brief BAR is prefetchable */
#define PCIE_EP_CDNS_BAR_PREFETCH BIT(2)

/** @brief Endpoint function configuration header */
struct pcie_ep_cdns_header {
	/** Vendor ID, shared by all functions, taken from function 0 */
	uint16_t vendor_id;
	/** Device ID */
	uint16_t device_id;
	/** Revision ID */
	uint8_t revision_id;
	/** Class code (base class << 16 | subclass << 8 | prog-if) */
	uint32_t class_code;
	/** Subsystem vendor ID, shared by all functions */
	uint16_t subsys_vendor_id;
	/** Subsystem ID */
	uint16_t subsys_id;
	/** Interrupt pin (0 none, 1 INTA ... 4 INTD) */
	uint8_t interrupt_pin;
};

/**
 * @brief Program the configuration header of an endpoint function.
 *
 * @param dev Endpoint controller.
 * @param fn Physical function number.
 * @param hdr Header values.
 *
 * @retval 0 Header written.
 * @retval -EINVAL Invalid function number.
 */
int pcie_ep_cdns_write_header(const struct device *dev, uint8_t fn,
			      const struct pcie_ep_cdns_header *hdr);

/**
 * @brief Expose local memory through a BAR.
 *
 * Inbound accesses to the BAR are translated to @p phys_addr. The BAR size is
 * rounded up to a power of two, minimum 128 bytes.
 *
 * @param dev Endpoint controller.
 * @param fn Physical function number.
 * @param bar BAR index 0..5; a 64-bit BAR must be even.
 * @param phys_addr Local physical address backing the BAR, aligned to its size.
 * @param size BAR size in bytes.
 * @param flags PCIE_EP_CDNS_BAR_* flags.
 *
 * @retval 0 BAR configured.
 * @retval -EINVAL Invalid function, BAR, size or alignment.
 */
int pcie_ep_cdns_set_bar(const struct device *dev, uint8_t fn, uint8_t bar, uintptr_t phys_addr,
			 uint64_t size, uint32_t flags);

/**
 * @brief Disable a BAR.
 *
 * @param dev Endpoint controller.
 * @param fn Physical function number.
 * @param bar BAR index 0..5.
 *
 * @retval 0 BAR disabled.
 * @retval -EINVAL Invalid function or BAR.
 */
int pcie_ep_cdns_clear_bar(const struct device *dev, uint8_t fn, uint8_t bar);

/**
 * @brief Set the number of MSI vectors advertised by a function.
 *
 * @param dev Endpoint controller.
 * @param fn Physical function number.
 * @param nr_vectors 1, 2, 4, 8, 16 or 32.
 *
 * @retval 0 MSI capability updated.
 * @retval -EINVAL Invalid function or vector count.
 */
int pcie_ep_cdns_set_msi(const struct device *dev, uint8_t fn, uint8_t nr_vectors);

/**
 * @brief Enable the configured functions and start link training.
 *
 * With a PERST# GPIO, training starts on PERST# deassertion; otherwise it
 * starts immediately.
 *
 * @param dev Endpoint controller.
 * @param fn_mask Bit mask of physical functions to enable (bit 0 is forced).
 *
 * @retval 0 Link start requested.
 * @retval -EIO Controller not initialized.
 */
int pcie_ep_cdns_start(const struct device *dev, uint32_t fn_mask);

/**
 * @brief Stop link training and disable configuration access from the host.
 *
 * @param dev Endpoint controller.
 */
void pcie_ep_cdns_stop(const struct device *dev);

/**
 * @brief Report whether the link is up.
 *
 * @param dev Endpoint controller.
 *
 * @retval true Link up.
 * @retval false Link down.
 */
bool pcie_ep_cdns_link_up(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EP_CDNS_H_ */
