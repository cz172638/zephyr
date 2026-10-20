/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Aardvark endpoint programming against a RAM-backed register file:
 * mode/refclk setup, header, outbound windows, MSI through a window and
 * the link poll events.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/sys_io.h>

#include "pcie_epc_aardvark.h"

#define OB_SIZE  (1024 * 1024)
#define OB_PHYS  0xe8000000ULL
#define MSI_CAP  0x50

static uint8_t regs[ADVK_REG_SPACE] __aligned(4096);
static uint8_t ob[OB_SIZE] __aligned(OB_SIZE);

static struct advk_ep_data dev_data;

/* registers right after init, before any test changed them */
static uint32_t init_ctrl0, init_refclk, init_mode, init_ob3;

static DEVICE_API(pcie_epc, api) = {
	.start = advk_ep_start,
	.stop = advk_ep_stop,
	.write_header = advk_ep_write_header,
	.set_bar = advk_ep_set_bar,
	.clear_bar = advk_ep_clear_bar,
	.map_addr = advk_ep_map_addr,
	.unmap_addr = advk_ep_unmap_addr,
	.set_msi = advk_ep_set_msi,
	.get_msi = advk_ep_get_msi,
	.raise_irq = advk_ep_raise_irq,
	.cfg_read = advk_ep_cfg_read,
	.cfg_write = advk_ep_cfg_write,
	.get_features = advk_ep_get_features,
	.link_up = advk_ep_link_up,
	.cpu_to_axi = advk_ep_cpu_to_axi,
};

static inline uint32_t R(uint32_t off)
{
	return sys_read32((mem_addr_t)&regs[off]);
}

static inline void W(uint32_t off, uint32_t v)
{
	sys_write32(v, (mem_addr_t)&regs[off]);
}

static void set_ltssm(uint8_t s)
{
	W(ADVK_LMI_CFG, (uint32_t)s << ADVK_LMI_LTSSM_SHIFT);
}

static int fake_init(const struct device *dev)
{
	struct advk_ep *ep = advk_ep(dev);

	/* reset-ish state left by a root complex user: RC, refclk out, cmd mode */
	W(ADVK_CTRL0, ADVK_CTRL0_IS_RC | ADVK_CTRL0_LINK_TRAIN | ADVK_CTRL0_INTA);
	W(ADVK_REF_CLK, 0x1344);
	W(ADVK_CTRL_CONFIG, ADVK_CTRL_MODE_CMD);
	W(ADVK_OB_MATCH_LS(3), 0x12340001);
	/* capability list: MSI at 0x50 -> PCIe at 0xc0 */
	W(ADVK_CFG_CMD_STATUS, ADVK_STS_CAPLIST);
	W(ADVK_CFG_CAP_PTR, MSI_CAP);
	W(MSI_CAP, 0x0000c005 | ADVK_MSI_CTRL_64BIT);
	W(ADVK_CFG_PCIE_CAP, 0x00000010);

	ep->reg = (mm_reg_t)regs;
	ep->ob_virt = (uintptr_t)ob;
	ep->ob_phys = OB_PHYS;
	ep->ob_size = OB_SIZE;
	ep->num_lanes = 1;
	ep->max_speed = 2;

	int ret = advk_ep_init(dev, false);

	init_ctrl0 = R(ADVK_CTRL0);
	init_refclk = R(ADVK_REF_CLK);
	init_mode = R(ADVK_CTRL_CONFIG);
	init_ob3 = R(ADVK_OB_MATCH_LS(3));
	return ret;
}

DEVICE_DEFINE(advk_fake, "advk_fake", fake_init, NULL, &dev_data, NULL, POST_KERNEL, 50, &api);

static const struct device *const epc = DEVICE_GET(advk_fake);

static struct {
	int count[PCIE_EPC_EVT_PERST_DEASSERT + 1];
	enum pcie_epc_event last;
} evts;

static void on_event(const struct device *dev, struct pcie_epc_listener *l,
		     enum pcie_epc_event evt, uint8_t func)
{
	evts.count[evt]++;
	evts.last = evt;
}

static struct pcie_epc_listener listener = {.func = 0xff, .event = on_event};

ZTEST(epc_aardvark, test_ob_encode)
{
	uint32_t r[7];

	zassert_ok(advk_ob_encode(0xe8040000ULL, 0x123400010000ULL, 0x10000, r));
	zassert_equal(r[0], 0xe8040001);
	zassert_equal(r[1], 0);
	zassert_equal(r[2], 0x00010000);
	zassert_equal(r[3], 0x1234);
	zassert_equal(r[4], 0xffff0000);
	zassert_equal(r[5], 0xffffffff);
	zassert_equal(r[6], ADVK_OB_TYPE_MEM);

	zassert_equal(advk_ob_encode(0xe8000000ULL, 0, 0x8000, r), -EINVAL, "below 64K");
	zassert_equal(advk_ob_encode(0xe8000000ULL, 0, 0x30000, r), -EINVAL, "not pow2");
	zassert_equal(advk_ob_encode(0xe8010000ULL, 0, 0x20000, r), -EINVAL, "axi align");
	zassert_equal(advk_ob_encode(0xe8000000ULL, 0x1000, 0x10000, r), -EINVAL, "pci align");
}

ZTEST(epc_aardvark, test_setup)
{
	const struct pcie_epc_features *f = pcie_epc_get_features(epc, 0);
	uint32_t c0 = init_ctrl0;

	zassert_equal(c0 & ADVK_CTRL0_IS_RC, 0, "endpoint mode");
	zassert_equal(c0 & ADVK_CTRL0_LINK_TRAIN, 0, "no training before start");
	zassert_equal(c0 & GENMASK(31, 28), 0, "INTx deasserted");
	zassert_equal(FIELD_GET(ADVK_CTRL0_GEN_MASK, c0), 1, "Gen2");
	zassert_equal(FIELD_GET(ADVK_CTRL0_LANES_MASK, c0), 0, "x1");
	zassert_equal(init_refclk, 0x1342, "RC refclk setting (TX on, RX off)");
	zassert_equal(init_mode & ADVK_CTRL_MODE_CMD, 0, "direct mode");
	zassert_true(R(ADVK_CTRL2) & ADVK_CTRL2_OB_WIN_EN);
	zassert_equal(init_ob3, 0, "stale window disabled");
	zassert_equal(R(ADVK_CFG_LNKCTL2) & 0xf, 2);
	zassert_equal(R(ADVK_HOST_INT_MASK), UINT32_MAX);

	zassert_equal(advk_ep(epc)->msi_cap, MSI_CAP);
	zassert_true(f->msi_capable);
	zassert_false(f->msix_capable);
	zassert_equal(f->align, ADVK_OB_MIN_SIZE);

	zassert_ok(pcie_epc_start(epc));
	zassert_true(R(ADVK_CTRL0) & ADVK_CTRL0_LINK_TRAIN);
	pcie_epc_stop(epc);
	zassert_false(R(ADVK_CTRL0) & ADVK_CTRL0_LINK_TRAIN);
}

static const struct pcie_epc_header hdr = {
	.vendor_id = 0x11ab,
	.device_id = 0x0100,
	.revision_id = 0x02,
	.baseclass = 0xff,
	.subsys_vendor_id = 0x1234,
	.subsys_id = 0x5678,
	.interrupt_pin = 1,
};

ZTEST(epc_aardvark, test_header)
{
	zassert_ok(pcie_epc_write_header(epc, 0, &hdr));
	zassert_equal(R(ADVK_LMI_VENDOR_ID), 0x123411ab);
	zassert_equal(R(ADVK_CFG_ID), 0x010011ab);
	zassert_equal(R(ADVK_CFG_REV_CLASS), 0xff000002);
	zassert_equal(R(ADVK_CFG_SSID), 0x56781234);
	zassert_equal((R(ADVK_CFG_INTR) >> 8) & 0xff, 1);
	zassert_equal(pcie_epc_write_header(epc, 1, &hdr), -EINVAL);
}

ZTEST(epc_aardvark, test_outbound)
{
	struct pcie_epc_host_map m;
	uint64_t axi;

	zassert_ok(pcie_epc_map_host(epc, 0, 0x12345678a000ULL, 0x2000, &m));
	/* 64 KiB minimum window around the request */
	zassert_equal(m.map_size, 0x10000);
	axi = OB_PHYS + (m.cpu_base - (uintptr_t)ob);
	zassert_equal(R(ADVK_OB_MATCH_LS(0)), (uint32_t)axi | ADVK_OB_WIN_ENABLE);
	zassert_equal(R(ADVK_OB_REMAP_LS(0)), 0x56780000);
	zassert_equal(R(ADVK_OB_REMAP_MS(0)), 0x1234);
	zassert_equal(R(ADVK_OB_MASK_LS(0)), 0xffff0000);
	zassert_equal((uintptr_t)m.ptr - m.cpu_base, 0xa000);
	zassert_equal(pcie_epc_cpu_to_axi(epc, m.cpu_base), axi);

	pcie_epc_unmap_host(epc, &m);
	zassert_equal(R(ADVK_OB_MATCH_LS(0)), 0);
	zassert_equal(advk_ep(epc)->ob_cpu[0], 0);
}

ZTEST(epc_aardvark, test_msi)
{
	uint32_t ctrl;

	zassert_ok(pcie_epc_set_msi(epc, 0, 8));
	zassert_equal(FIELD_GET(ADVK_MSI_CTRL_MMC, R(MSI_CAP)), 3);
	zassert_equal(pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_MSI, 1), -EINVAL, "not enabled");

	/* host: 4 vectors at 0x1_fee0_0040, data 0x4020 */
	ctrl = R(MSI_CAP) | ADVK_MSI_CTRL_EN | FIELD_PREP(ADVK_MSI_CTRL_MME, 2);
	W(MSI_CAP, ctrl);
	W(MSI_CAP + 4, 0xfee00040);
	W(MSI_CAP + 8, 0x1);
	W(MSI_CAP + 12, 0x4020);
	zassert_equal(pcie_epc_get_msi(epc, 0), 4);

	memset(ob, 0, sizeof(ob));
	zassert_ok(pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_MSI, 3));
	/* the window was freed again, the allocator handed out page 0 */
	zassert_equal(sys_read32((mem_addr_t)&ob[0x40]), 0x4022);
	zassert_equal(R(ADVK_OB_MATCH_LS(0)), 0, "window released");
	zassert_equal(pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_MSI, 5), -EINVAL, "beyond MME");
}

ZTEST(epc_aardvark, test_intx)
{
	zassert_ok(pcie_epc_write_header(epc, 0, &hdr));
	zassert_ok(pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_INTX, 0));
	/* pulse: asserted and released again */
	zassert_equal(R(ADVK_CTRL0) & ADVK_CTRL0_INTA, 0);
	zassert_equal(pcie_epc_raise_irq(epc, 0, PCIE_EPC_IRQ_MSIX, 1), -ENOTSUP);
}

static void settle(void)
{
	/* events go through the system work queue */
	k_msleep(5);
}

ZTEST(epc_aardvark, test_poll_events)
{
	struct advk_ep *ep = advk_ep(epc);

	pcie_epc_add_listener(epc, &listener);
	memset(&evts, 0, sizeof(evts));
	zassert_ok(pcie_epc_write_header(epc, 0, &hdr));

	set_ltssm(0x02);
	advk_ep_poll(epc, 10);
	settle();
	zassert_equal(evts.count[PCIE_EPC_EVT_LINK_UP], 0);
	zassert_false(pcie_epc_link_up(epc));

	set_ltssm(ADVK_LTSSM_L0);
	advk_ep_poll(epc, 20);
	settle();
	zassert_equal(evts.count[PCIE_EPC_EVT_LINK_UP], 1);
	zassert_true(pcie_epc_link_up(epc));

	W(ADVK_CFG_CMD_STATUS, R(ADVK_CFG_CMD_STATUS) | ADVK_CMD_MASTER);
	advk_ep_poll(epc, 30);
	advk_ep_poll(epc, 40);
	settle();
	zassert_equal(evts.count[PCIE_EPC_EVT_BME], 1);

	/* hot reset: link goes down and the core forgets the header */
	set_ltssm(ADVK_LTSSM_HOT_RESET);
	W(ADVK_CFG_REV_CLASS, 0x01040000);
	W(ADVK_CTRL0, ADVK_CTRL0_LINK_TRAIN);
	advk_ep_poll(epc, 50);
	settle();
	zassert_equal(evts.count[PCIE_EPC_EVT_HOT_RESET], 1);
	zassert_equal(evts.count[PCIE_EPC_EVT_LINK_DOWN], 1);
	zassert_equal(evts.count[PCIE_EPC_EVT_BME_CLEARED], 1);
	zassert_equal(R(ADVK_CFG_REV_CLASS), 0xff000002, "header reprogrammed");
	zassert_true(R(ADVK_CTRL0) & ADVK_CTRL0_LINK_TRAIN, "training kept on");
	zassert_equal(ep->reapplies, 1);

	/* history: polling-active, L0, hot-reset */
	zassert_equal(ep->hist[ep->hist_head].state, ADVK_LTSSM_HOT_RESET);
	zassert_equal(ep->hist[(ep->hist_head + ADVK_LTSSM_HIST - 1) % ADVK_LTSSM_HIST].state,
		      ADVK_LTSSM_L0);

	pcie_epc_remove_listener(epc, &listener);
}

ZTEST(epc_aardvark, test_ltssm_names)
{
	zassert_str_equal(advk_ltssm_name(0x10), "L0");
	zassert_str_equal(advk_ltssm_name(0x27), "hot-reset");
	zassert_str_equal(advk_ltssm_name(0x0f), "?");
	zassert_str_equal(advk_ltssm_name(0x3f), "?");
}

ZTEST_SUITE(epc_aardvark, NULL, NULL, NULL, NULL, NULL);
