/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PCIe fabric endpoint function: BAR0 is this node's region
 * (zephyr/drivers/pcie/fabric.h). The root complex finds it by the
 * magic, writes the card (node ID, role, peers with their bus
 * addresses) and commits it; this function maps every peer's region
 * through the outbound window and from then on the node writes there
 * directly. Everything it reads is its own BAR memory.
 *
 * One thread: card changes, bus master state, the node's service step
 * (echo answers, the receive queue). Outbound requests need bus
 * mastering, so a card waits until the host enabled it.
 *
 * Only pcie_epc.h is used: the function knows nothing about the
 * controller.
 */

#define DT_DRV_COMPAT zephyr_pcie_epf_fabric

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/version.h>
#include <zephyr/drivers/pcie/fabric.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>
#include <zephyr/drivers/pcie/role.h>
#ifdef CONFIG_MMU
#include <zephyr/kernel/internal/mm.h>
#endif

LOG_MODULE_REGISTER(pcie_epf_fabric, CONFIG_PCIE_EP_LOG_LEVEL);

#define CFG_COMMAND     0x04
#define CFG_COMMAND_BME BIT(2)

#ifdef BUILD_VERSION
#define FW_STRING STRINGIFY(BUILD_VERSION)
#else
#define FW_STRING KERNEL_VERSION_STRING
#endif

struct epf_fabric_config {
	const struct device *epc;
	uint8_t func;
	struct pcie_epc_header hdr;
	uintptr_t mem_base;
	size_t mem_size;
	bool mem_map;
	uint32_t region_size;
	uint32_t roles;
	k_thread_entry_t thread_fn;
};

struct epf_fabric_data {
	const struct device *dev;
	struct pcie_epc_listener listener;
	struct k_sem kick;
	uintptr_t mem_virt;
	struct pcie_epc_bar bar;
	struct pfab_node node;
	struct pcie_epc_host_map maps[CONFIG_PCIE_FABRIC_MAX_NODES];
	bool map_used[CONFIG_PCIE_FABRIC_MAX_NODES];
	struct pfab_map_ops ops;
	bool link;
	bool bme;
	uint64_t dsn;
	atomic_t reset_req;
	uint32_t resets;
	uint32_t map_errors;
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_PCIE_FABRIC_STACK_SIZE);
};

/* peer regions through the outbound window */
static int peer_map(void *ctx, uint64_t bus, size_t size, volatile uint8_t **cpu, void **handle)
{
	const struct device *dev = ctx;
	const struct epf_fabric_config *cfg = dev->config;
	struct epf_fabric_data *d = dev->data;
	int ret;

	for (int i = 0; i < ARRAY_SIZE(d->maps); i++) {
		if (d->map_used[i]) {
			continue;
		}
		ret = pcie_epc_map_host(cfg->epc, cfg->func, bus, size, &d->maps[i]);
		if (ret != 0) {
			d->map_errors++;
			return ret;
		}
		d->map_used[i] = true;
		*cpu = (volatile uint8_t *)d->maps[i].ptr;
		*handle = &d->maps[i];
		return 0;
	}
	return -ENOMEM;
}

static void peer_unmap(void *ctx, void *handle)
{
	const struct device *dev = ctx;
	const struct epf_fabric_config *cfg = dev->config;
	struct epf_fabric_data *d = dev->data;
	struct pcie_epc_host_map *m = handle;
	int i = m - d->maps;

	if (i < 0 || i >= ARRAY_SIZE(d->maps) || !d->map_used[i]) {
		return;
	}
	pcie_epc_unmap_host(cfg->epc, m);
	d->map_used[i] = false;
}

static void bme_update(struct epf_fabric_data *d)
{
	const struct epf_fabric_config *cfg = d->dev->config;
	uint32_t v = 0;

	(void)pcie_epc_cfg_read(cfg->epc, cfg->func, CFG_COMMAND, &v);
	d->bme = (v & CFG_COMMAND_BME) != 0U;
}

static void dsn_update(struct epf_fabric_data *d)
{
#if defined(CONFIG_PCIE_EPC_DSN_SETTINGS)
	const struct epf_fabric_config *cfg = d->dev->config;
	const char *src = NULL;
	uint64_t v = 0;

	if (pcie_epc_dsn_cmd(cfg->epc, NULL, &v, &src) == 0 && v != d->dsn) {
		d->dsn = v;
		pfab_wr64(d->node.local, PFAB_INFO_DSN, v);
	}
#else
	ARG_UNUSED(d);
#endif
}

static void fabric_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct epf_fabric_data *d = dev->data;
	struct pfab_node *n = &d->node;
	int64_t bme_check = 0;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	dsn_update(d);
	for (;;) {
		if (atomic_cas(&d->reset_req, 1, 0)) {
			/* the host forgot us: the peers' addresses may be gone too */
			if (n->running) {
				pfab_node_stop(n, &d->ops);
				d->resets++;
			}
			bme_check = 0;
		}
		if (d->link && (!n->running || k_uptime_get() - bme_check > 100)) {
			bme_update(d);
			bme_check = k_uptime_get();
		}
		if (d->link && d->bme) {
			int c = pfab_card_pending(n);

			if (c == -ECANCELED) {
				pfab_node_stop(n, &d->ops);
			} else if (c > 0) {
				dsn_update(d);
				(void)pfab_card_accept(n, &d->ops);
			}
		} else if (n->running && !d->bme) {
			pfab_node_stop(n, &d->ops);
		}
		if (n->running) {
			(void)pfab_service(n);
			/* events (link down, BME) are seen at the next step */
			pfab_service_wait(n);
		} else {
			(void)k_sem_take(&d->kick, d->link ? K_MSEC(10) : K_MSEC(100));
		}
	}
}

static void fabric_event(const struct device *epc, struct pcie_epc_listener *l,
			 enum pcie_epc_event evt, uint8_t func)
{
	struct epf_fabric_data *d = CONTAINER_OF(l, struct epf_fabric_data, listener);

	ARG_UNUSED(epc);
	ARG_UNUSED(func);

	switch (evt) {
	case PCIE_EPC_EVT_LINK_UP:
		d->link = true;
		break;
	case PCIE_EPC_EVT_LINK_DOWN:
		d->link = false;
		atomic_set(&d->reset_req, 1);
		break;
	case PCIE_EPC_EVT_BME:
		d->bme = true;
		break;
	case PCIE_EPC_EVT_BME_CLEARED:
		d->bme = false;
		atomic_set(&d->reset_req, 1);
		break;
	case PCIE_EPC_EVT_HOT_RESET:
	case PCIE_EPC_EVT_FLR:
	case PCIE_EPC_EVT_PERST_ASSERT:
		atomic_set(&d->reset_req, 1);
		break;
	default:
		break;
	}
	k_sem_give(&d->kick);
}

static int setup_bar(const struct device *dev)
{
	const struct epf_fabric_config *cfg = dev->config;
	struct epf_fabric_data *d = dev->data;
	const struct pcie_epc_features *feat = pcie_epc_get_features(cfg->epc, cfg->func);
	size_t sz = pcie_epf_bar_size(feat, 0, cfg->region_size);
	size_t align = feat != NULL ? MAX(feat->align, 128U) : 128U;
	uintptr_t base = ROUND_UP(cfg->mem_base, MAX(sz, align));
	bool b64 = feat == NULL || (feat->bar_fixed_64bit & BIT(0)) != 0U;
	int ret;

	if (feat != NULL && (feat->reserved_bar & BIT(0)) != 0U) {
		LOG_ERR("BAR0 is not available on %s", cfg->epc->name);
		return -EINVAL;
	}
	if (sz == 0 || sz < cfg->region_size) {
		LOG_ERR("BAR0: 0x%x bytes not possible", cfg->region_size);
		return -EINVAL;
	}
	if (base + sz > cfg->mem_base + cfg->mem_size) {
		LOG_ERR("memory-region too small for 0x%zx bytes", sz);
		return -ENOMEM;
	}
	d->bar = (struct pcie_epc_bar){
		.barno = 0,
		.flags = b64 ? PCIE_EPC_BAR_MEM64 : PCIE_EPC_BAR_MEM32,
		.phys = pcie_epc_cpu_to_axi(cfg->epc, base),
		.size = sz,
		.virt = (void *)(d->mem_virt + (base - cfg->mem_base)),
	};
	ret = pcie_epc_set_bar(cfg->epc, cfg->func, &d->bar);
	if (ret != 0) {
		LOG_ERR("set_bar failed (%d)", ret);
		return ret;
	}
	for (uint8_t b = 1; b < PCIE_EPC_NUM_BARS; b++) {
		if ((b == 1 && b64) || (feat != NULL && (feat->reserved_bar & BIT(b)) != 0U)) {
			continue;
		}
		pcie_epc_clear_bar(cfg->epc, cfg->func, b);
	}
	return 0;
}

static int epf_fabric_init(const struct device *dev)
{
	const struct epf_fabric_config *cfg = dev->config;
	struct epf_fabric_data *d = dev->data;
	struct pfab_geom g;
	uint32_t caps = PFAB_CAP_MSG | PFAB_CAP_RMA;
	int ret;

	d->dev = dev;
	k_sem_init(&d->kick, 0, 1);

	if (!pcie_epf_selected("fabric")) {
		return -ENODEV;
	}
	if (!device_is_ready(cfg->epc)) {
		if (pcie_ctrl_is_rc()) {
			LOG_INF("%s: root complex mode, function off", dev->name);
			return -ENODEV;
		}
		LOG_ERR("controller %s not ready", cfg->epc->name);
		return -ENODEV;
	}
	ret = pcie_epc_claim_function(cfg->epc, cfg->func);
	if (ret != 0) {
		LOG_ERR("function %u: claim failed (%d)", cfg->func, ret);
		return ret;
	}
	ret = pcie_epc_write_header(cfg->epc, cfg->func, &cfg->hdr);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_MMU
	if (cfg->mem_map) {
		k_mem_map_phys_bare((uint8_t **)&d->mem_virt, cfg->mem_base, cfg->mem_size,
				    K_MEM_PERM_RW |
#ifdef K_MEM_ARM_NORMAL_NC
				    K_MEM_ARM_NORMAL_NC
#else
				    K_MEM_CACHE_NONE
#endif
		);
	} else {
		d->mem_virt = cfg->mem_base;
	}
#else
	d->mem_virt = cfg->mem_base;
#endif

	ret = setup_bar(dev);
	if (ret != 0) {
		return ret;
	}
	ret = pfab_geom_default(&g, d->bar.size);
	if (ret != 0) {
		LOG_ERR("region 0x%zx: geometry does not fit (%d)", d->bar.size, ret);
		return ret;
	}
	pfab_region_init(d->bar.virt, &g, 0, caps, cfg->roles, FW_STRING, CONFIG_BOARD);
	pfab_node_init(&d->node, dev->name, d->bar.virt, &g);
	pfab_node_register(&d->node);
	d->ops = (struct pfab_map_ops){.map = peer_map, .unmap = peer_unmap, .ctx = (void *)dev};

	d->link = pcie_epc_link_up(cfg->epc);
	d->listener.func = cfg->func;
	d->listener.event = fabric_event;
	pcie_epc_add_listener(cfg->epc, &d->listener);

	k_thread_create(&d->thread, d->stack, K_KERNEL_STACK_SIZEOF(d->stack), cfg->thread_fn,
			(void *)dev, NULL, NULL, CONFIG_PCIE_FABRIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&d->thread, dev->name);

	LOG_INF("%s: fn%u %04x:%04x on %s, region 0x%zx at 0x%llx, %u nodes x %u slots x %u B, "
		"RMA 0x%x", dev->name, cfg->func, cfg->hdr.vendor_id, cfg->hdr.device_id,
		cfg->epc->name, d->bar.size, (unsigned long long)d->bar.phys, g.max_nodes,
		g.nslots, g.slot_size, g.rma_size);
	return 0;
}

static const int fabric_api;

#define EPF_FABRIC_MEM(n)                                                                          \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region),                                       \
		    (.mem_base = DT_REG_ADDR(DT_INST_PHANDLE(n, memory_region)),                   \
		     .mem_size = DT_REG_SIZE(DT_INST_PHANDLE(n, memory_region)),                   \
		     .mem_map = true,),                                                            \
		    (.mem_base = (uintptr_t)epf_fabric_mem_##n,                                    \
		     .mem_size = sizeof(epf_fabric_mem_##n),))

#define EPF_FABRIC_MEM_BUF(n)                                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, memory_region), (),                                   \
		    (static uint8_t epf_fabric_mem_##n[CONFIG_PCIE_EPF_FABRIC_STATIC_MEM]          \
		     __aligned(CONFIG_PCIE_EPF_FABRIC_STATIC_MEM);))

#define EPF_FABRIC_DEFINE(n)                                                                       \
	EPF_FABRIC_MEM_BUF(n)                                                                      \
	static void epf_fabric_thread_##n(void *p1, void *p2, void *p3)                            \
	{                                                                                          \
		fabric_thread(p1, p2, p3);                                                         \
	}                                                                                          \
	static const struct epf_fabric_config epf_fabric_cfg_##n = {                               \
		.epc = PCIE_EPF_DT_INST_EPC(n),                                                    \
		.func = PCIE_EPF_DT_INST_FUNC(n),                                                  \
		.hdr = PCIE_EPF_DT_HEADER(DT_DRV_INST(n)),                                         \
		EPF_FABRIC_MEM(n)                                                                  \
		.region_size = DT_INST_PROP(n, region_size),                                       \
		.roles = DT_INST_PROP(n, roles),                                                   \
		.thread_fn = epf_fabric_thread_##n,                                                \
	};                                                                                         \
	static struct epf_fabric_data epf_fabric_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, epf_fabric_init, NULL, &epf_fabric_data_##n,                      \
			      &epf_fabric_cfg_##n, POST_KERNEL, CONFIG_PCIE_EPF_INIT_PRIORITY,     \
			      &fabric_api);

DT_INST_FOREACH_STATUS_OKAY(EPF_FABRIC_DEFINE)
