/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Non-Transparent Bridge (NTB) device API.
 *
 * An NTB connects two memory domains, this one and a peer, with the same
 * few constructs the Linux NTB API (include/linux/ntb.h) has, so that a
 * client written against it (a transport, a network interface, a test
 * tool) does not care what provides the bridge:
 *
 *  - link: usable once both sides enabled it,
 *  - scratchpads: small registers; the own ones the peer writes, the
 *    peer's ones this side writes,
 *  - doorbells: bits this side rings at the peer (an interrupt there),
 *    and bits the peer rings here (a db_event callback),
 *  - memory windows: a window of this side's memory the peer writes into
 *    (inbound, "mw"), and a window of the peer's memory this side writes
 *    into (outbound, "peer mw").
 *
 * Unlike Linux there is one peer only (the index arguments of the Linux
 * API are left out) and no doorbell masks: callbacks come from a thread,
 * never from an ISR.
 *
 * The first provider is the vNTB PCIe endpoint function
 * (zephyr,pcie-epf-vntb), the peer being a Linux root complex with the
 * ntb_hw_epf driver.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_NTB_H_
#define ZEPHYR_INCLUDE_DRIVERS_NTB_H_

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Non-Transparent Bridge interface
 * @defgroup ntb_interface NTB
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief Client callbacks.
 *
 * Called from the provider's thread, never from an ISR; may call the API
 * but should not block for long.
 */
struct ntb_callbacks {
	/** The link changed; ntb_link_is_up() tells the new state. */
	void (*link_event)(const struct device *dev, void *user_data);
	/** The peer rang doorbell @p vector (bit @p vector of ntb_db_read()). */
	void (*db_event)(const struct device *dev, int vector, void *user_data);
};

/** @brief Alignment rules of a memory window. */
struct ntb_mw_align {
	/** The window must start at a multiple of this. */
	uint64_t addr_align;
	/** Its size must be a multiple of this. */
	uint64_t size_align;
	/** Largest window. */
	uint64_t size_max;
};

/** @cond INTERNAL_HIDDEN */

__subsystem struct ntb_driver_api {
	int (*set_callbacks)(const struct device *dev, const struct ntb_callbacks *cb,
			     void *user_data);
	int (*link_enable)(const struct device *dev, bool enable);
	bool (*link_is_up)(const struct device *dev);

	int (*spad_count)(const struct device *dev);
	uint32_t (*spad_read)(const struct device *dev, int idx);
	int (*spad_write)(const struct device *dev, int idx, uint32_t val);
	uint32_t (*peer_spad_read)(const struct device *dev, int idx);
	int (*peer_spad_write)(const struct device *dev, int idx, uint32_t val);

	uint64_t (*db_valid_mask)(const struct device *dev);
	uint64_t (*db_read)(const struct device *dev);
	int (*db_clear)(const struct device *dev, uint64_t bits);
	int (*peer_db_set)(const struct device *dev, uint64_t bits);

	int (*mw_count)(const struct device *dev);
	int (*mw_get_align)(const struct device *dev, int idx, struct ntb_mw_align *align);
	int (*mw_set_trans)(const struct device *dev, int idx, uint64_t phys, size_t size);
	int (*mw_get_local)(const struct device *dev, int idx, void **virt, uint64_t *phys,
			    size_t *size);
	int (*peer_mw_count)(const struct device *dev);
	int (*peer_mw_get_addr)(const struct device *dev, int idx, volatile void **virt,
				size_t *size);
	int (*peer_mw_xfer)(const struct device *dev, int idx, size_t offset, void *local,
			    size_t len, bool to_peer);
};

/** @endcond */

/**
 * @brief Register the client's callbacks (one client per device).
 *
 * @param cb NULL to unregister.
 * @retval -EBUSY another client is registered
 */
static inline int ntb_set_callbacks(const struct device *dev, const struct ntb_callbacks *cb,
				    void *user_data)
{
	return DEVICE_API_GET(ntb, dev)->set_callbacks(dev, cb, user_data);
}

/** @brief Enable (or disable) this side of the link. */
static inline int ntb_link_enable(const struct device *dev, bool enable)
{
	return DEVICE_API_GET(ntb, dev)->link_enable(dev, enable);
}

/** @return true once both sides enabled the link. */
static inline bool ntb_link_is_up(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->link_is_up(dev);
}

/** @return number of scratchpads on each side. */
static inline int ntb_spad_count(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->spad_count(dev);
}

/** @brief Read an own scratchpad (written by this side or by the peer). */
static inline uint32_t ntb_spad_read(const struct device *dev, int idx)
{
	return DEVICE_API_GET(ntb, dev)->spad_read(dev, idx);
}

/** @brief Write an own scratchpad. */
static inline int ntb_spad_write(const struct device *dev, int idx, uint32_t val)
{
	return DEVICE_API_GET(ntb, dev)->spad_write(dev, idx, val);
}

/** @brief Read a scratchpad of the peer. */
static inline uint32_t ntb_peer_spad_read(const struct device *dev, int idx)
{
	return DEVICE_API_GET(ntb, dev)->peer_spad_read(dev, idx);
}

/** @brief Write a scratchpad of the peer (the peer reads it with ntb_spad_read()). */
static inline int ntb_peer_spad_write(const struct device *dev, int idx, uint32_t val)
{
	return DEVICE_API_GET(ntb, dev)->peer_spad_write(dev, idx, val);
}

/** @return the doorbell bits that exist, in both directions. */
static inline uint64_t ntb_db_valid_mask(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->db_valid_mask(dev);
}

/** @return doorbell bits the peer rang and nobody cleared yet. */
static inline uint64_t ntb_db_read(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->db_read(dev);
}

/** @brief Clear doorbell bits rung by the peer. */
static inline int ntb_db_clear(const struct device *dev, uint64_t bits)
{
	return DEVICE_API_GET(ntb, dev)->db_clear(dev, bits);
}

/**
 * @brief Ring doorbell bits at the peer.
 *
 * Not from an ISR: raising the interrupt may sleep.
 */
static inline int ntb_peer_db_set(const struct device *dev, uint64_t bits)
{
	return DEVICE_API_GET(ntb, dev)->peer_db_set(dev, bits);
}

/** @return number of inbound memory windows (the peer writes this side's memory). */
static inline int ntb_mw_count(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->mw_count(dev);
}

/** @brief Alignment rules of inbound window @p idx. */
static inline int ntb_mw_get_align(const struct device *dev, int idx, struct ntb_mw_align *align)
{
	return DEVICE_API_GET(ntb, dev)->mw_get_align(dev, idx, align);
}

/**
 * @brief Point inbound window @p idx at local memory.
 *
 * @param phys address of the memory as the bridge hardware sees it
 * @retval -ENOTSUP the provider has fixed windows (see ntb_mw_get_local())
 */
static inline int ntb_mw_set_trans(const struct device *dev, int idx, uint64_t phys, size_t size)
{
	const struct ntb_driver_api *api = DEVICE_API_GET(ntb, dev);

	return api->mw_set_trans ? api->mw_set_trans(dev, idx, phys, size) : -ENOTSUP;
}

/**
 * @brief The local memory behind inbound window @p idx.
 *
 * Zephyr addition: a provider with its own backing memory for the window
 * (a carveout the controller can reach) hands it out here, so the client
 * needs no ntb_mw_set_trans().
 *
 * @param[out] virt CPU pointer, may be NULL
 * @param[out] phys bridge view, may be NULL
 * @param[out] size window size, may be NULL
 */
static inline int ntb_mw_get_local(const struct device *dev, int idx, void **virt,
				   uint64_t *phys, size_t *size)
{
	const struct ntb_driver_api *api = DEVICE_API_GET(ntb, dev);

	return api->mw_get_local ? api->mw_get_local(dev, idx, virt, phys, size) : -ENOTSUP;
}

/** @return number of outbound memory windows (this side writes the peer's memory). */
static inline int ntb_peer_mw_count(const struct device *dev)
{
	return DEVICE_API_GET(ntb, dev)->peer_mw_count(dev);
}

/**
 * @brief CPU view of outbound window @p idx.
 *
 * @retval -ENODATA the peer has not set up the window yet
 */
static inline int ntb_peer_mw_get_addr(const struct device *dev, int idx, volatile void **virt,
				       size_t *size)
{
	return DEVICE_API_GET(ntb, dev)->peer_mw_get_addr(dev, idx, virt, size);
}

/**
 * @brief Copy between local memory and outbound window @p idx.
 *
 * With the provider's DMA where it has one, a CPU copy otherwise. When it
 * returns, data toward the peer is visible to it before a following
 * ntb_peer_db_set().
 *
 * @param to_peer true: @p local -> peer window at @p offset; false: back
 */
static inline int ntb_peer_mw_xfer(const struct device *dev, int idx, size_t offset, void *local,
				   size_t len, bool to_peer)
{
	const struct ntb_driver_api *api = DEVICE_API_GET(ntb, dev);

	return api->peer_mw_xfer ? api->peer_mw_xfer(dev, idx, offset, local, len, to_peer)
				 : -ENOTSUP;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_NTB_H_ */
