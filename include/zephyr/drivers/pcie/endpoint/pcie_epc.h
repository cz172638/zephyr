/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PCIe Endpoint Controller (EPC) API.
 *
 * The EPC API is the hardware facing half of the Zephyr PCIe endpoint stack.
 * It describes what a PCIe endpoint controller can do (configuration header,
 * BARs with inbound translation, outbound windows, MSI/MSI-X/INTx generation,
 * link control) without saying anything about *what* the endpoint does.
 *
 * The protocol facing half is an Endpoint Function (EPF), see pcie_epf.h. An
 * EPF only talks to the EPC API, so the same function (e.g. the Linux
 * pci_endpoint_test compatible test function, an NTB/rpmsg bridge, ...) runs
 * unmodified on any controller: Cadence (TI K3), Synopsys DWC, iProc PAXB...
 *
 * Model (deliberately close to Linux pci-epc/pci-epf so that host side
 * drivers and mental models carry over):
 *
 *   +-------------------+   +-------------------+
 *   | EPF: test         |   | EPF: vendor proto |   function drivers
 *   +---------+---------+   +---------+---------+   (DT children of EPC)
 *             |   pcie_epc_*() API    |
 *   +---------v-----------------------v---------+
 *   | EPC core: OB window allocator, listeners, |   pcie_epc_core.c
 *   |           autostart, legacy helpers       |
 *   +---------------------+---------------------+
 *                         | struct pcie_epc_driver_api
 *   +---------------------v---------------------+
 *   | IP core library: Cadence LGA / DWC / ...  |   reusable per IP
 *   +---------------------+---------------------+
 *                         | small glue ops (start/stop link, link_up)
 *   +---------------------v---------------------+
 *   | SoC glue: TI J721E/J722S, ...             |   power, straps, IRQs
 *   +-------------------------------------------+
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PCIe Endpoint Controller interface
 * @defgroup pcie_epc_interface PCIe Endpoint Controller
 * @ingroup io_interfaces
 * @{
 */

/** Number of standard BARs of a type 0 function. */
#define PCIE_EPC_NUM_BARS 6

/** @brief Standard configuration header fields an EPF may program. */
struct pcie_epc_header {
	uint16_t vendor_id;
	uint16_t device_id;
	uint8_t revision_id;
	uint8_t prog_if;
	uint8_t subclass;
	uint8_t baseclass;
	uint8_t cache_line_size;
	uint16_t subsys_vendor_id;
	uint16_t subsys_id;
	/** 0 = none, 1 = INTA ... 4 = INTD */
	uint8_t interrupt_pin;
};

/** @name BAR flags
 * @{
 */
#define PCIE_EPC_BAR_MEM32    0U
#define PCIE_EPC_BAR_IO       BIT(0)
#define PCIE_EPC_BAR_MEM64    BIT(2)
#define PCIE_EPC_BAR_PREFETCH BIT(3)
/** @} */

/**
 * @brief One BAR of one function.
 *
 * @p phys is the address *as seen by the PCIe controller's AXI master*, i.e.
 * where inbound TLPs hitting this BAR end up. On most SoCs this is equal to
 * the CPU physical address; the glue may translate (see
 * pcie_epc_driver_api::cpu_to_axi).
 */
struct pcie_epc_bar {
	uint8_t barno;
	uint32_t flags;
	uint64_t phys;
	size_t size;
	/** CPU view of the backing memory, for the EPF's own use. */
	void *virt;
};

/** @brief Static capabilities of a controller (per function). */
struct pcie_epc_features {
	bool linkup_notifier;
	bool msi_capable;
	bool msix_capable;
	bool intx_capable;
	/** Bit mask of BARs that must not be used by EPFs. */
	uint8_t reserved_bar;
	/** Bit mask of BARs that are hard-wired 64-bit. */
	uint8_t bar_fixed_64bit;
	/** Non-zero entries are BARs with a hard-wired size. */
	size_t bar_fixed_size[PCIE_EPC_NUM_BARS];
	/** Required alignment of BAR backing memory and of OB mappings. */
	size_t align;
	/** Number of physical functions the controller is configured for. */
	uint8_t max_functions;
};

/** @brief Interrupt types an EP can raise toward the root complex. */
enum pcie_epc_irq_type {
	PCIE_EPC_IRQ_INTX,
	PCIE_EPC_IRQ_MSI,
	PCIE_EPC_IRQ_MSIX,
};

/** @brief Asynchronous controller events delivered to listeners. */
enum pcie_epc_event {
	/** Data link layer is up. */
	PCIE_EPC_EVT_LINK_UP,
	/** Link went down (cable pulled, host reboot, PERST#). */
	PCIE_EPC_EVT_LINK_DOWN,
	/** Host set Bus Master Enable for the function -> OB access allowed. */
	PCIE_EPC_EVT_BME,
	/** Host cleared Bus Master Enable. */
	PCIE_EPC_EVT_BME_CLEARED,
	/** In-band hot reset received. */
	PCIE_EPC_EVT_HOT_RESET,
	/** Function level reset of @p func. */
	PCIE_EPC_EVT_FLR,
	/** PERST# asserted (only if the glue can see PERST#). */
	PCIE_EPC_EVT_PERST_ASSERT,
	/** PERST# deasserted. */
	PCIE_EPC_EVT_PERST_DEASSERT,
};

/**
 * @brief Outbound window descriptor.
 *
 * CPU addresses in [@p cpu_base, @p cpu_base + @p size) are forwarded by the
 * controller to PCIe memory space once mapped with pcie_epc_map_addr().
 */
struct pcie_epc_ob_window {
	uintptr_t cpu_base;
	uint64_t axi_base;
	size_t size;
	/** Granule of the window allocator, power of two. */
	size_t page_size;
};

/**
 * @brief Event listener, embedded in EPF private data.
 *
 * Callbacks are invoked from the EPC core's work queue (never from ISR
 * context) so they may sleep and call any EPC API.
 */
struct pcie_epc_listener {
	sys_snode_t node;
	/** Function this listener belongs to, 0xff = all functions. */
	uint8_t func;
	void (*event)(const struct device *epc, struct pcie_epc_listener *l,
		      enum pcie_epc_event evt, uint8_t func);
};

/** @cond INTERNAL_HIDDEN */

#ifndef CONFIG_PCIE_EPC_OB_PAGES
#define CONFIG_PCIE_EPC_OB_PAGES 64
#endif

#ifndef CONFIG_PCIE_EPC_EVENT_QUEUE_DEPTH
#define CONFIG_PCIE_EPC_EVENT_QUEUE_DEPTH 16
#endif

/**
 * Common per-controller state owned by the EPC core. Every EPC driver must
 * place this structure at the very beginning of its @c dev->data.
 */
struct pcie_epc_data {
	struct k_mutex lock;
	sys_slist_t listeners;
	sys_snode_t node;
	const struct device *dev;
	struct pcie_epc_ob_window ob;
	ATOMIC_DEFINE(ob_map, CONFIG_PCIE_EPC_OB_PAGES);
	uint32_t func_map;
	bool started;
	bool autostart;
	struct k_work evt_work;
	struct k_msgq evtq;
	uint16_t evtq_buf[CONFIG_PCIE_EPC_EVENT_QUEUE_DEPTH];
};

/* Controller operations. All function numbers are physical function numbers. */
__subsystem struct pcie_epc_driver_api {
	int (*start)(const struct device *dev);
	void (*stop)(const struct device *dev);
	int (*write_header)(const struct device *dev, uint8_t func,
			    const struct pcie_epc_header *hdr);
	int (*set_bar)(const struct device *dev, uint8_t func, const struct pcie_epc_bar *bar);
	void (*clear_bar)(const struct device *dev, uint8_t func, uint8_t barno);
	int (*map_addr)(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
			uint64_t pci_addr, size_t size);
	void (*unmap_addr)(const struct device *dev, uint8_t func, uintptr_t cpu_addr);
	int (*set_msi)(const struct device *dev, uint8_t func, uint8_t nr_irqs);
	int (*get_msi)(const struct device *dev, uint8_t func);
	int (*set_msix)(const struct device *dev, uint8_t func, uint16_t nr_irqs,
			uint8_t bir, uint32_t offset);
	int (*get_msix)(const struct device *dev, uint8_t func);
	int (*raise_irq)(const struct device *dev, uint8_t func, enum pcie_epc_irq_type type,
			 uint16_t irq_num);
	int (*cfg_read)(const struct device *dev, uint8_t func, uint32_t offset, uint32_t *val);
	int (*cfg_write)(const struct device *dev, uint8_t func, uint32_t offset, uint32_t val);
	const struct pcie_epc_features *(*get_features)(const struct device *dev, uint8_t func);
	bool (*link_up)(const struct device *dev);
	/** Optional: translate a CPU physical address to the controller's AXI view. */
	uint64_t (*cpu_to_axi)(const struct device *dev, uintptr_t cpu_addr);
	/** Optional: Device Serial Number capability (EUI-64, byte 0 in bits 63:56). */
	int (*set_dsn)(const struct device *dev, uint8_t func, uint64_t dsn);
	/** Optional: copy between local memory and host memory with a DMA engine. */
	int (*xfer_dma)(const struct device *dev, uint8_t func, uint64_t pci_addr, void *local,
			size_t size, bool to_host);
};

/** @endcond */

/**
 * @brief Start the controller: enable configured functions and link training.
 *
 * Usually called by the EPC core autostart once all EPFs have bound. Call it
 * manually when the controller node has `zephyr,no-autostart`.
 */
int pcie_epc_start(const struct device *dev);

/** @brief Stop link training and take the link down. */
void pcie_epc_stop(const struct device *dev);

/**
 * @brief Program the type 0 header of function @p func.
 *
 * Vendor ID changes only take effect through function 0 on most IPs.
 */
static inline int pcie_epc_write_header(const struct device *dev, uint8_t func,
					const struct pcie_epc_header *hdr)
{
	return DEVICE_API_GET(pcie_epc, dev)->write_header(dev, func, hdr);
}

/**
 * @brief Enable a BAR and point its inbound translation at @p bar->phys.
 *
 * The size is rounded up by the controller to a power of two and to the
 * controller minimum; @p bar->phys must be aligned to the resulting size and
 * to pcie_epc_features::align.
 */
static inline int pcie_epc_set_bar(const struct device *dev, uint8_t func,
				   const struct pcie_epc_bar *bar)
{
	if (bar->barno >= PCIE_EPC_NUM_BARS) {
		return -EINVAL;
	}
	return DEVICE_API_GET(pcie_epc, dev)->set_bar(dev, func, bar);
}

/** @brief Disable a BAR. */
static inline void pcie_epc_clear_bar(const struct device *dev, uint8_t func, uint8_t barno)
{
	DEVICE_API_GET(pcie_epc, dev)->clear_bar(dev, func, barno);
}

/**
 * @brief Map a CPU address inside the outbound window to a PCIe address.
 *
 * Low level; most EPFs want pcie_epc_map_host() which also allocates the
 * window space and handles alignment.
 */
static inline int pcie_epc_map_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr,
				    uint64_t pci_addr, size_t size)
{
	return DEVICE_API_GET(pcie_epc, dev)->map_addr(dev, func, cpu_addr, pci_addr, size);
}

/** @brief Undo pcie_epc_map_addr(). */
static inline void pcie_epc_unmap_addr(const struct device *dev, uint8_t func, uintptr_t cpu_addr)
{
	DEVICE_API_GET(pcie_epc, dev)->unmap_addr(dev, func, cpu_addr);
}

/** @brief Advertise @p nr_irqs (1..32, power of two) MSI vectors. */
static inline int pcie_epc_set_msi(const struct device *dev, uint8_t func, uint8_t nr_irqs)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->set_msi ? api->set_msi(dev, func, nr_irqs) : -ENOTSUP;
}

/** @return number of MSI vectors enabled by the host, or negative errno. */
static inline int pcie_epc_get_msi(const struct device *dev, uint8_t func)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->get_msi ? api->get_msi(dev, func) : -ENOTSUP;
}

/** @brief Advertise MSI-X with the table in BAR @p bir at @p offset. */
static inline int pcie_epc_set_msix(const struct device *dev, uint8_t func, uint16_t nr_irqs,
				    uint8_t bir, uint32_t offset)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->set_msix ? api->set_msix(dev, func, nr_irqs, bir, offset) : -ENOTSUP;
}

/** @return number of MSI-X vectors enabled by the host, or negative errno. */
static inline int pcie_epc_get_msix(const struct device *dev, uint8_t func)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->get_msix ? api->get_msix(dev, func) : -ENOTSUP;
}

/**
 * @brief Raise an interrupt toward the host.
 *
 * @param irq_num 1-based vector number for MSI/MSI-X, ignored for INTx.
 */
static inline int pcie_epc_raise_irq(const struct device *dev, uint8_t func,
				     enum pcie_epc_irq_type type, uint16_t irq_num)
{
	return DEVICE_API_GET(pcie_epc, dev)->raise_irq(dev, func, type, irq_num);
}

/** @brief Read a 32-bit word of function @p func's configuration space. */
static inline int pcie_epc_cfg_read(const struct device *dev, uint8_t func, uint32_t offset,
				    uint32_t *val)
{
	return DEVICE_API_GET(pcie_epc, dev)->cfg_read(dev, func, offset, val);
}

/** @brief Write a 32-bit word of function @p func's configuration space. */
static inline int pcie_epc_cfg_write(const struct device *dev, uint8_t func, uint32_t offset,
				     uint32_t val)
{
	return DEVICE_API_GET(pcie_epc, dev)->cfg_write(dev, func, offset, val);
}

/** @brief Controller capabilities for function @p func. */
static inline const struct pcie_epc_features *pcie_epc_get_features(const struct device *dev,
								    uint8_t func)
{
	return DEVICE_API_GET(pcie_epc, dev)->get_features(dev, func);
}

/** @brief Current data link layer state. */
static inline bool pcie_epc_link_up(const struct device *dev)
{
	return DEVICE_API_GET(pcie_epc, dev)->link_up(dev);
}

/** @brief Translate a CPU physical address to what the controller emits on AXI. */
static inline uint64_t pcie_epc_cpu_to_axi(const struct device *dev, uintptr_t cpu_addr)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->cpu_to_axi ? api->cpu_to_axi(dev, cpu_addr) : (uint64_t)cpu_addr;
}

/**
 * @brief Should the endpoint function @p name bind this boot?
 *
 * Several functions may be described for one function number; the one
 * selected (setting pcie/ep/func, see pcie_epf_select(), else
 * CONFIG_PCIE_EPF_DEFAULT_FUNCTION, else the first one asking) binds and
 * the others stay off. Call it first in the function's init; the name is
 * also recorded for pcie_epf_names().
 */
bool pcie_epf_selected(const char *name);

/** @brief The selection in force (saved or default), NULL for "first one". */
const char *pcie_epf_selection(void);

/** @brief The function that bound this boot, NULL if none. */
const char *pcie_epf_bound(void);

/** @brief Names of the functions in this build; returns their number. */
size_t pcie_epf_names(const char **names, size_t max);

/** @brief Save the function for the next boot (must be one of pcie_epf_names()). */
int pcie_epf_select(const char *name);

/** @brief Drop the saved selection (back to the default). */
int pcie_epf_select_reset(void);

/**
 * @brief Claim function number @p func for an EPF.
 *
 * @retval -EBUSY function already claimed
 * @retval -EINVAL beyond pcie_epc_features::max_functions
 */
int pcie_epc_claim_function(const struct device *dev, uint8_t func);

/** @brief Register an event listener. */
void pcie_epc_add_listener(const struct device *dev, struct pcie_epc_listener *l);

/** @brief Unregister an event listener. */
void pcie_epc_remove_listener(const struct device *dev, struct pcie_epc_listener *l);

/**
 * @brief Allocate space in the outbound window.
 *
 * @param size bytes needed, rounded up to the window page size
 * @param[out] cpu_addr CPU address of the allocated space
 */
int pcie_epc_ob_alloc(const struct device *dev, size_t size, uintptr_t *cpu_addr);

/** @brief Release space allocated with pcie_epc_ob_alloc(). */
void pcie_epc_ob_free(const struct device *dev, uintptr_t cpu_addr, size_t size);

/** @brief Handle for a host memory mapping created by pcie_epc_map_host(). */
struct pcie_epc_host_map {
	uintptr_t cpu_base;
	size_t map_size;
	/** CPU pointer to the first requested byte of host memory. */
	volatile void *ptr;
	uint8_t func;
};

/**
 * @brief Make host memory [@p pci_addr, @p pci_addr + @p size) CPU accessible.
 *
 * Allocates outbound window space, honours the controller alignment and
 * programs the translation. Only valid after the host enabled bus mastering
 * (PCIE_EPC_EVT_BME).
 */
int pcie_epc_map_host(const struct device *dev, uint8_t func, uint64_t pci_addr, size_t size,
		      struct pcie_epc_host_map *map);

/** @brief Tear down a pcie_epc_map_host() mapping. */
void pcie_epc_unmap_host(const struct device *dev, struct pcie_epc_host_map *map);

/**
 * @brief Copy between local memory and host memory through the OB window.
 *
 * CPU copy with a trailing non-posted read when writing toward the host so
 * the data is visible before a subsequent interrupt is raised. Equivalent of
 * the legacy pcie_ep_xfer_data_memcpy().
 *
 * @param to_host true: local -> host, false: host -> local
 */
int pcie_epc_xfer_memcpy(const struct device *dev, uint8_t func, uint64_t pci_addr,
			 void *local, size_t size, bool to_host);

/**
 * @brief Set the Device Serial Number of function @p func.
 *
 * @param dsn EUI-64, first byte in bits 63:56 (lspci prints it as
 *            "Device Serial Number xx-xx-...", most significant first)
 * @retval -ENOTSUP the controller has no DSN capability
 */
static inline int pcie_epc_set_dsn(const struct device *dev, uint8_t func, uint64_t dsn)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->set_dsn ? api->set_dsn(dev, func, dsn) : -ENOTSUP;
}

/**
 * @brief Copy between local memory and host memory with the controller's DMA.
 *
 * Same contract as pcie_epc_xfer_memcpy(): when it returns, the data is in
 * place (toward the host: visible before a subsequent interrupt).
 *
 * @param to_host true: local -> host, false: host -> local
 * @retval -ENOTSUP the controller has no DMA
 */
static inline int pcie_epc_xfer_dma(const struct device *dev, uint8_t func, uint64_t pci_addr,
				    void *local, size_t size, bool to_host)
{
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	return api->xfer_dma ? api->xfer_dma(dev, func, pci_addr, local, size, to_host)
			     : -ENOTSUP;
}

/** @cond INTERNAL_HIDDEN */

/*
 * Driver-side helpers. Called by EPC drivers only.
 */

/** Initialise core state; call from the driver init before anything else. */
int pcie_epc_init_common(const struct device *dev, const struct pcie_epc_ob_window *ob,
			 bool autostart);

/**
 * Report an event. Safe to call from ISR context; delivery to listeners is
 * deferred to the system work queue. Events are dropped (and logged) if the
 * queue overflows.
 */
void pcie_epc_notify(const struct device *dev, enum pcie_epc_event evt, uint8_t func);

/** @endcond */

/** @} */

#if defined(CONFIG_PCIE_EPC_DSN_SETTINGS) || defined(__DOXYGEN__)
/**
 * @brief Device Serial Number kept in settings: show or change it.
 *
 * @param dev Endpoint controller.
 * @param arg NULL: show; "uid", "new" (random), "reset" (forget the saved
 *            one), or a hex EUI-64.
 * @param v The DSN now.
 * @param src Where a generated one came from, or untouched.
 * @return 0, -EINVAL on a bad value, -ENODEV for a controller without DSN.
 */
int pcie_epc_dsn_cmd(const struct device *dev, const char *arg, uint64_t *v, const char **src);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCIE_ENDPOINT_PCIE_EPC_H_ */
