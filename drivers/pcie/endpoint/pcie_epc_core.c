/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware independent part of the PCIe endpoint controller stack:
 * function bookkeeping, outbound window allocation, event fan-out and
 * autostart. Nothing in here knows about a particular IP.
 */

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>

LOG_MODULE_REGISTER(pcie_epc, CONFIG_PCIE_EP_LOG_LEVEL);

static sys_slist_t epc_registry = SYS_SLIST_STATIC_INIT(&epc_registry);
static K_MUTEX_DEFINE(epc_registry_lock);

static inline struct pcie_epc_data *epc_data(const struct device *dev)
{
	return (struct pcie_epc_data *)dev->data;
}

static size_t round_up_pow2(size_t v)
{
	size_t r = 1;

	while (r < v) {
		r <<= 1;
	}
	return r;
}

static void epc_evt_work(struct k_work *work)
{
	struct pcie_epc_data *d = CONTAINER_OF(work, struct pcie_epc_data, evt_work);
	struct pcie_epc_listener *l, *tmp;
	uint16_t msg;

	while (k_msgq_get(&d->evtq, &msg, K_NO_WAIT) == 0) {
		enum pcie_epc_event evt = (enum pcie_epc_event)(msg & 0xff);
		uint8_t func = msg >> 8;

		LOG_DBG("%s: event %d func %u", d->dev->name, evt, func);

		k_mutex_lock(&d->lock, K_FOREVER);
		SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&d->listeners, l, tmp, node) {
			if (l->func != 0xff && l->func != func && func != 0xff) {
				continue;
			}
			l->event(d->dev, l, evt, func);
		}
		k_mutex_unlock(&d->lock);
	}
}

int pcie_epc_init_common(const struct device *dev, const struct pcie_epc_ob_window *ob,
			 bool autostart)
{
	struct pcie_epc_data *d = epc_data(dev);

	k_mutex_init(&d->lock);
	sys_slist_init(&d->listeners);
	k_msgq_init(&d->evtq, (char *)d->evtq_buf, sizeof(d->evtq_buf[0]),
		    ARRAY_SIZE(d->evtq_buf));
	k_work_init(&d->evt_work, epc_evt_work);
	d->dev = dev;
	d->func_map = 0;
	d->started = false;
	d->autostart = autostart;

	memset(&d->ob, 0, sizeof(d->ob));
	if (ob != NULL && ob->size != 0) {
		d->ob = *ob;
		if (d->ob.page_size == 0) {
			d->ob.page_size = 4096;
		}
		d->ob.page_size = round_up_pow2(d->ob.page_size);
		/* Grow the granule until the bitmap covers the whole window. */
		while (d->ob.size / d->ob.page_size > CONFIG_PCIE_EPC_OB_PAGES) {
			d->ob.page_size <<= 1;
		}
		LOG_DBG("%s: OB window cpu 0x%lx axi 0x%llx size 0x%zx page 0x%zx", dev->name,
			(unsigned long)d->ob.cpu_base, (unsigned long long)d->ob.axi_base,
			d->ob.size, d->ob.page_size);
	}
	for (size_t i = 0; i < CONFIG_PCIE_EPC_OB_PAGES; i++) {
		atomic_clear_bit(d->ob_map, i);
	}

	k_mutex_lock(&epc_registry_lock, K_FOREVER);
	sys_slist_append(&epc_registry, &d->node);
	k_mutex_unlock(&epc_registry_lock);

	return 0;
}

void pcie_epc_notify(const struct device *dev, enum pcie_epc_event evt, uint8_t func)
{
	struct pcie_epc_data *d = epc_data(dev);
	uint16_t msg = (uint16_t)evt | ((uint16_t)func << 8);

	/*
	 * On LINK_DOWN / HOT_RESET any host mapping held by an EPF is stale;
	 * EPFs are expected to drop them in their listener.
	 */
	if (k_msgq_put(&d->evtq, &msg, K_NO_WAIT) != 0) {
		LOG_WRN("%s: event queue full, dropped event %d", dev->name, evt);
		return;
	}
	k_work_submit(&d->evt_work);
}

int pcie_epc_claim_function(const struct device *dev, uint8_t func)
{
	struct pcie_epc_data *d = epc_data(dev);
	const struct pcie_epc_features *feat = pcie_epc_get_features(dev, func);
	int ret = 0;

	if (func >= 32 ||
	    (feat != NULL && func >= MAX(feat->max_functions, 1) + feat->max_vfs)) {
		return -EINVAL;
	}

	k_mutex_lock(&d->lock, K_FOREVER);
	if (d->func_map & BIT(func)) {
		ret = -EBUSY;
	} else {
		d->func_map |= BIT(func);
	}
	k_mutex_unlock(&d->lock);

	return ret;
}

void pcie_epc_add_listener(const struct device *dev, struct pcie_epc_listener *l)
{
	struct pcie_epc_data *d = epc_data(dev);

	k_mutex_lock(&d->lock, K_FOREVER);
	sys_slist_append(&d->listeners, &l->node);
	k_mutex_unlock(&d->lock);
}

void pcie_epc_remove_listener(const struct device *dev, struct pcie_epc_listener *l)
{
	struct pcie_epc_data *d = epc_data(dev);

	k_mutex_lock(&d->lock, K_FOREVER);
	sys_slist_find_and_remove(&d->listeners, &l->node);
	k_mutex_unlock(&d->lock);
}

int pcie_epc_start(const struct device *dev)
{
	struct pcie_epc_data *d = epc_data(dev);
	int ret = 0;

	k_mutex_lock(&d->lock, K_FOREVER);
	if (!d->started) {
		ret = DEVICE_API_GET(pcie_epc, dev)->start(dev);
		d->started = (ret == 0);
	}
	k_mutex_unlock(&d->lock);

	if (ret != 0) {
		LOG_ERR("%s: start failed (%d)", dev->name, ret);
	}
	return ret;
}

void pcie_epc_stop(const struct device *dev)
{
	struct pcie_epc_data *d = epc_data(dev);
	const struct pcie_epc_driver_api *api = DEVICE_API_GET(pcie_epc, dev);

	k_mutex_lock(&d->lock, K_FOREVER);
	if (d->started && api->stop != NULL) {
		api->stop(dev);
	}
	d->started = false;
	k_mutex_unlock(&d->lock);
}

int pcie_epc_ob_alloc(const struct device *dev, size_t size, uintptr_t *cpu_addr)
{
	struct pcie_epc_data *d = epc_data(dev);
	size_t pages, total;
	int ret = -ENOMEM;

	if (d->ob.size == 0) {
		return -ENOTSUP;
	}

	total = d->ob.size / d->ob.page_size;
	/* Naturally aligned power-of-two blocks keep every IP's ATU happy. */
	pages = round_up_pow2(DIV_ROUND_UP(size, d->ob.page_size));
	if (pages > total) {
		return -ENOMEM;
	}

	k_mutex_lock(&d->lock, K_FOREVER);
	for (size_t first = 0; first + pages <= total; first += pages) {
		bool free = true;

		for (size_t i = 0; i < pages; i++) {
			if (atomic_test_bit(d->ob_map, first + i)) {
				free = false;
				break;
			}
		}
		if (!free) {
			continue;
		}
		for (size_t i = 0; i < pages; i++) {
			atomic_set_bit(d->ob_map, first + i);
		}
		*cpu_addr = d->ob.cpu_base + first * d->ob.page_size;
		ret = 0;
		break;
	}
	k_mutex_unlock(&d->lock);

	return ret;
}

void pcie_epc_ob_free(const struct device *dev, uintptr_t cpu_addr, size_t size)
{
	struct pcie_epc_data *d = epc_data(dev);
	size_t first, pages;

	if (d->ob.size == 0 || cpu_addr < d->ob.cpu_base) {
		return;
	}

	first = (cpu_addr - d->ob.cpu_base) / d->ob.page_size;
	pages = round_up_pow2(DIV_ROUND_UP(size, d->ob.page_size));

	k_mutex_lock(&d->lock, K_FOREVER);
	for (size_t i = 0; i < pages && first + i < CONFIG_PCIE_EPC_OB_PAGES; i++) {
		atomic_clear_bit(d->ob_map, first + i);
	}
	k_mutex_unlock(&d->lock);
}

int pcie_epc_map_host(const struct device *dev, uint8_t func, uint64_t pci_addr, size_t size,
		      struct pcie_epc_host_map *map)
{
	const struct pcie_epc_features *feat = pcie_epc_get_features(dev, func);
	size_t align = MAX(feat != NULL ? feat->align : 0, 256);
	size_t region = round_up_pow2(MAX(size, align));
	uint64_t pci_base;
	size_t offset;
	int ret;

	if (size == 0) {
		return -EINVAL;
	}

	/*
	 * Translation replaces the low log2(region) address bits, so the PCI
	 * base must be aligned to the region size. Grow the region until the
	 * aligned base still covers the whole request.
	 */
	for (;;) {
		pci_base = pci_addr & ~((uint64_t)region - 1);
		offset = (size_t)(pci_addr - pci_base);
		if (offset + size <= region) {
			break;
		}
		region <<= 1;
	}

	ret = pcie_epc_ob_alloc(dev, region, &map->cpu_base);
	if (ret != 0) {
		return ret;
	}

	ret = pcie_epc_map_addr(dev, func, map->cpu_base, pci_base, region);
	if (ret != 0) {
		pcie_epc_ob_free(dev, map->cpu_base, region);
		return ret;
	}

	map->map_size = region;
	map->func = func;
	map->ptr = (volatile void *)(map->cpu_base + offset);

	return 0;
}

void pcie_epc_unmap_host(const struct device *dev, struct pcie_epc_host_map *map)
{
	pcie_epc_unmap_addr(dev, map->func, map->cpu_base);
	pcie_epc_ob_free(dev, map->cpu_base, map->map_size);
	map->ptr = NULL;
}

int pcie_epc_xfer_memcpy(const struct device *dev, uint8_t func, uint64_t pci_addr,
			 void *local, size_t size, bool to_host)
{
	struct pcie_epc_host_map map;
	int ret;

	ret = pcie_epc_map_host(dev, func, pci_addr, size, &map);
	if (ret != 0) {
		return ret;
	}

	if (to_host) {
		memcpy((void *)map.ptr, local, size);
		barrier_dmem_fence_full();
		/* Non-posted read flushes the posted writes before an IRQ is sent. */
		(void)sys_read8((mem_addr_t)map.ptr);
	} else {
		memcpy(local, (const void *)map.ptr, size);
	}

	pcie_epc_unmap_host(dev, &map);

	return 0;
}

size_t pcie_epf_bar_size(const struct pcie_epc_features *feat, uint8_t barno, size_t want)
{
	if (barno >= PCIE_EPC_NUM_BARS) {
		return 0;
	}
	if (feat != NULL) {
		if (feat->reserved_bar & BIT(barno)) {
			return 0;
		}
		if (feat->bar_fixed_size[barno] != 0) {
			return feat->bar_fixed_size[barno];
		}
	}
	return round_up_pow2(MAX(want, 128));
}

static int pcie_epc_autostart(void)
{
	struct pcie_epc_data *d;

	k_mutex_lock(&epc_registry_lock, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&epc_registry, d, node) {
		if (!d->autostart) {
			continue;
		}
		if (d->func_map == 0) {
			LOG_INF("%s: no function bound, not starting link", d->dev->name);
			continue;
		}
		(void)pcie_epc_start(d->dev);
	}
	k_mutex_unlock(&epc_registry_lock);

	return 0;
}

SYS_INIT(pcie_epc_autostart, POST_KERNEL, CONFIG_PCIE_EPC_AUTOSTART_PRIORITY);
