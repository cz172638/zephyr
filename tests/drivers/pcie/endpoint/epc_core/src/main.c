/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware independent EPC core: outbound allocator, host mapping
 * alignment, function claiming and event delivery, against a fake
 * controller that records what it was asked to program.
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epf.h>

#define WIN_SIZE (1024 * 1024)

static uint8_t window[WIN_SIZE] __aligned(WIN_SIZE);

struct fake_epc_data {
	struct pcie_epc_data common; /* must be first */
	int started;
	uintptr_t last_cpu;
	uint64_t last_pci;
	size_t last_size;
	int maps;
};

static struct pcie_epc_features fake_feat = {
	.msi_capable = true,
	.align = 256,
	.max_functions = 2,
	.reserved_bar = BIT(1),
	.bar_fixed_size = {[4] = 4096},
};

static int fake_start(const struct device *dev)
{
	((struct fake_epc_data *)dev->data)->started++;
	return 0;
}

static int fake_map(const struct device *dev, uint8_t func, uintptr_t cpu, uint64_t pci,
		    size_t size)
{
	struct fake_epc_data *d = dev->data;

	d->last_cpu = cpu;
	d->last_pci = pci;
	d->last_size = size;
	d->maps++;
	return 0;
}

static void fake_unmap(const struct device *dev, uint8_t func, uintptr_t cpu)
{
	((struct fake_epc_data *)dev->data)->maps--;
}

static const struct pcie_epc_features *fake_features(const struct device *dev, uint8_t func)
{
	return &fake_feat;
}

static bool fake_link_up(const struct device *dev)
{
	return true;
}

static DEVICE_API(pcie_epc, fake_api) = {
	.start = fake_start,
	.map_addr = fake_map,
	.unmap_addr = fake_unmap,
	.get_features = fake_features,
	.link_up = fake_link_up,
};

static int fake_init(const struct device *dev)
{
	struct pcie_epc_ob_window ob = {
		.cpu_base = (uintptr_t)window,
		.axi_base = 0x10000000,
		.size = WIN_SIZE,
		.page_size = 4096,
	};

	return pcie_epc_init_common(dev, &ob, false);
}

static struct fake_epc_data fake_data;
DEVICE_DEFINE(fake_epc, "fake_epc", fake_init, NULL, &fake_data, NULL, POST_KERNEL, 50,
	      &fake_api);

static const struct device *const epc = DEVICE_GET(fake_epc);

ZTEST(epc_core, test_ob_alloc_natural_alignment)
{
	uintptr_t a, b, c;

	/* 64 pages of 16 KiB: the 1 MiB window is split to fit the bitmap */
	zassert_ok(pcie_epc_ob_alloc(epc, 100, &a));
	zassert_ok(pcie_epc_ob_alloc(epc, 40000, &b)); /* -> 64 KiB block */
	zassert_equal((b - (uintptr_t)window) % 65536, 0, "not naturally aligned");
	zassert_ok(pcie_epc_ob_alloc(epc, 100, &c));
	zassert_true(c != a && c != b);
	zassert_equal(pcie_epc_ob_alloc(epc, 2 * WIN_SIZE, &a), -ENOMEM);

	pcie_epc_ob_free(epc, b, 40000);
	pcie_epc_ob_free(epc, c, 100);
	pcie_epc_ob_free(epc, a, 100);
	/* everything free again: one allocation of the whole window works */
	zassert_ok(pcie_epc_ob_alloc(epc, WIN_SIZE, &a));
	zassert_equal(a, (uintptr_t)window);
	pcie_epc_ob_free(epc, a, WIN_SIZE);
}

ZTEST(epc_core, test_map_host_alignment)
{
	struct pcie_epc_host_map m;

	/* 0x1234_5678 + 0x300 crosses a 1 KiB boundary: needs a 2 KiB region */
	zassert_ok(pcie_epc_map_host(epc, 0, 0x12345678, 0x300, &m));
	zassert_equal(fake_data.last_pci & (fake_data.last_size - 1), 0, "PCI base misaligned");
	zassert_true(fake_data.last_pci <= 0x12345678);
	zassert_true(fake_data.last_pci + fake_data.last_size >= 0x12345678 + 0x300);
	zassert_equal((uintptr_t)m.ptr - m.cpu_base, 0x12345678 - fake_data.last_pci);
	zassert_equal(fake_data.maps, 1);
	pcie_epc_unmap_host(epc, &m);
	zassert_equal(fake_data.maps, 0);
	zassert_is_null((void *)m.ptr);
}

ZTEST(epc_core, test_claim_function)
{
	zassert_ok(pcie_epc_claim_function(epc, 0));
	zassert_equal(pcie_epc_claim_function(epc, 0), -EBUSY);
	zassert_ok(pcie_epc_claim_function(epc, 1));
	zassert_equal(pcie_epc_claim_function(epc, 2), -EINVAL);
}

ZTEST(epc_core, test_start_once)
{
	zassert_ok(pcie_epc_start(epc));
	zassert_ok(pcie_epc_start(epc));
	zassert_equal(fake_data.started, 1);
}

static K_SEM_DEFINE(evt_sem, 0, 4);
static enum pcie_epc_event last_evt;
static uint8_t last_func;

static void on_event(const struct device *dev, struct pcie_epc_listener *l,
		     enum pcie_epc_event evt, uint8_t func)
{
	last_evt = evt;
	last_func = func;
	k_sem_give(&evt_sem);
}

ZTEST(epc_core, test_events)
{
	struct pcie_epc_listener l = {.func = 1, .event = on_event};

	pcie_epc_add_listener(epc, &l);

	pcie_epc_notify(epc, PCIE_EPC_EVT_BME, 0); /* other function: filtered */
	pcie_epc_notify(epc, PCIE_EPC_EVT_BME, 1);
	zassert_ok(k_sem_take(&evt_sem, K_MSEC(100)));
	zassert_equal(last_evt, PCIE_EPC_EVT_BME);
	zassert_equal(last_func, 1);
	zassert_equal(k_sem_take(&evt_sem, K_MSEC(50)), -EAGAIN);

	pcie_epc_notify(epc, PCIE_EPC_EVT_LINK_DOWN, 0xff); /* broadcast */
	zassert_ok(k_sem_take(&evt_sem, K_MSEC(100)));
	zassert_equal(last_evt, PCIE_EPC_EVT_LINK_DOWN);

	pcie_epc_remove_listener(epc, &l);
}

ZTEST(epc_core, test_epf_bar_size)
{
	zassert_equal(pcie_epf_bar_size(&fake_feat, 0, 100), 128);
	zassert_equal(pcie_epf_bar_size(&fake_feat, 0, 5000), 8192);
	zassert_equal(pcie_epf_bar_size(&fake_feat, 1, 4096), 0, "reserved BAR");
	zassert_equal(pcie_epf_bar_size(&fake_feat, 4, 1), 4096, "fixed size BAR");
	zassert_equal(pcie_epf_bar_size(&fake_feat, 6, 1), 0);
}

ZTEST_SUITE(epc_core, NULL, NULL, NULL, NULL, NULL);
