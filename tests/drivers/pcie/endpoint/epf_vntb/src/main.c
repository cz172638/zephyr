/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * vNTB endpoint function against a host that does what the Linux
 * ntb_hw_epf driver does (i.MX8 BAR map: config BAR0, doorbells BAR2,
 * window BAR4), with Zephyr on the other side of the NTB API.
 */

#include <string.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/ntb.h>
#include <zephyr/drivers/pcie/endpoint/pcie_epc.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#ifdef CONFIG_SHELL_BACKEND_DUMMY
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#endif

#include "fake_epc.h"

/* ntb_hw_epf register map */
#define NTB_EPF_COMMAND        0x00
#define NTB_EPF_ARGUMENT       0x04
#define NTB_EPF_CMD_STATUS     0x08
#define NTB_EPF_LINK_STATUS    0x0a
#define NTB_EPF_LOWER_ADDR     0x10
#define NTB_EPF_UPPER_ADDR     0x14
#define NTB_EPF_LOWER_SIZE     0x18
#define NTB_EPF_UPPER_SIZE     0x1c
#define NTB_EPF_MW_COUNT       0x20
#define NTB_EPF_MW1_OFFSET     0x24
#define NTB_EPF_SPAD_OFFSET    0x28
#define NTB_EPF_SPAD_COUNT     0x2c
#define NTB_EPF_DB_ENTRY_SIZE  0x30
#define NTB_EPF_DB_DATA(n)     (0x34 + (n) * 4)
#define NTB_EPF_DB_OFFSET(n)   (0xb4 + (n) * 4)

#define CMD_CONFIGURE_DOORBELL 1
#define CMD_TEARDOWN_DOORBELL  2
#define CMD_CONFIGURE_MW       3
#define CMD_TEARDOWN_MW        4
#define CMD_LINK_UP            5
#define CMD_LINK_DOWN          6
#define MSIX_ENABLE            BIT(16)

#define STATUS_OK    1
#define STATUS_ERROR 2

#define SPADS   16
#define DB_SLOTS 8
#define MW_SIZE 0x10000

static const struct device *const ntb = DEVICE_DT_GET(DT_NODELABEL(vntb0));
static const struct device *const epc = DEVICE_DT_GET(DT_NODELABEL(fake_epc));

static K_SEM_DEFINE(link_sem, 0, 16);
static K_SEM_DEFINE(db_sem, 0, 64);
static uint32_t db_vectors;

static void on_link(const struct device *dev, void *user_data)
{
	k_sem_give(&link_sem);
}

static void on_db(const struct device *dev, int vector, void *user_data)
{
	db_vectors |= BIT(vector);
	k_sem_give(&db_sem);
}

static const struct ntb_callbacks cb = {
	.link_event = on_link,
	.db_event = on_db,
};

/* host view of the BARs */
static mem_addr_t ctrl(uint32_t off)
{
	return (mem_addr_t)fake_bar[0].virt + off;
}

static uint32_t hrd(uint32_t off)
{
	return sys_read32(ctrl(off));
}

static void hwr(uint32_t off, uint32_t v)
{
	sys_write32(v, ctrl(off));
}

/* ntb_epf_send_command(): argument, command, poll the status, clear it */
static int host_cmd(uint32_t cmd, uint32_t arg)
{
	uint16_t st = 0;

	hwr(NTB_EPF_ARGUMENT, arg);
	hwr(NTB_EPF_COMMAND, cmd);
	for (int i = 0; i < 1000 && st == 0; i++) {
		k_msleep(1);
		st = sys_read16(ctrl(NTB_EPF_CMD_STATUS));
	}
	sys_write16(0, ctrl(NTB_EPF_CMD_STATUS));
	return st;
}

/* ntb_epf_peer_db_set(): DB#n is interrupt slot n + 2 */
static void host_ring(int db)
{
	uint32_t slot = db + 2;
	uint32_t entry = hrd(NTB_EPF_DB_ENTRY_SIZE);
	uint32_t data = hrd(NTB_EPF_DB_DATA(slot));
	uint32_t off = hrd(NTB_EPF_DB_OFFSET(slot));

	sys_write32(data, (mem_addr_t)fake_bar[2].virt + entry * slot + off);
}

static uint32_t host_peer_spad(int idx)
{
	return hrd(hrd(NTB_EPF_SPAD_OFFSET) + 4 * SPADS + 4 * idx);
}

static void host_spad_write(int idx, uint32_t v)
{
	hwr(hrd(NTB_EPF_SPAD_OFFSET) + 4 * idx, v);
}

static void *setup(void)
{
	zassert_true(device_is_ready(ntb));
	zassert_ok(ntb_set_callbacks(ntb, &cb, NULL));
	return NULL;
}

static void before(void *f)
{
	memset(fake_irq_msix, 0, sizeof(fake_irq_msix));
	memset(fake_irq_msi, 0, sizeof(fake_irq_msi));
	k_sem_reset(&link_sem);
	k_sem_reset(&db_sem);
	k_sem_reset(&fake_irq_sem);
	db_vectors = 0;
	fake_bme = true;
}

/* host probe up to the link: doorbells with MSI-X, link up */
static void host_probe(void)
{
	zassert_equal(host_cmd(CMD_CONFIGURE_DOORBELL, MSIX_ENABLE | DB_SLOTS), STATUS_OK);
	zassert_equal(host_cmd(CMD_LINK_UP, 0), STATUS_OK);
}

static void host_remove(void)
{
	zassert_equal(host_cmd(CMD_LINK_DOWN, 0), STATUS_OK);
	zassert_equal(host_cmd(CMD_TEARDOWN_DOORBELL, DB_SLOTS), STATUS_OK);
}

ZTEST(epf_vntb, test_a_layout)
{
	uint32_t spad_off = hrd(NTB_EPF_SPAD_OFFSET);

	zassert_equal(fake_hdr.vendor_id, 0x1957);
	zassert_equal(fake_hdr.device_id, 0x0809);
	zassert_equal(fake_hdr.baseclass, 0x05, "ntb_hw_epf matches class 05:00");
	zassert_equal(fake_hdr.subclass, 0x00);

	/* BAR0 config, BAR2 doorbells, BAR4 window, all 64-bit */
	zassert_not_equal(fake_bar[0].size, 0);
	zassert_not_equal(fake_bar[2].size, 0);
	zassert_equal(fake_bar[4].size, MW_SIZE);
	zassert_equal(fake_bar[4].flags & PCIE_EPC_BAR_MEM64, PCIE_EPC_BAR_MEM64);
	zassert_true(fake_bar[2].size >= 32 * 4, "a slot for every possible vector");

	zassert_equal(hrd(NTB_EPF_MW_COUNT), 1);
	zassert_equal(hrd(NTB_EPF_MW1_OFFSET), 0);
	zassert_equal(hrd(NTB_EPF_SPAD_COUNT), SPADS);
	zassert_equal(hrd(NTB_EPF_DB_ENTRY_SIZE), 4);
	zassert_equal(hrd(NTB_EPF_DB_DATA(2)), 3);
	zassert_true(spad_off + 2 * 4 * SPADS <= fake_bar[0].size);

	/* MSI-X table in the config BAR after the control region */
	zassert_equal(fake_msix_nr, DB_SLOTS);
	zassert_equal(fake_msix_bir, 0);
	zassert_true(fake_msix_off >= 0x134 && (fake_msix_off & 7) == 0);
	zassert_true(fake_msix_off + DB_SLOTS * 16 <= spad_off);
	zassert_equal(fake_msi_nr, 8);

	zassert_equal(ntb_mw_count(ntb), 1);
	zassert_equal(ntb_peer_mw_count(ntb), 1);
	zassert_equal(ntb_spad_count(ntb), SPADS);
	/* db-count 8: link, unused, 6 doorbells */
	zassert_equal(ntb_db_valid_mask(ntb), 0x3f);
}

ZTEST(epf_vntb, test_link)
{
	zassert_false(ntb_link_is_up(ntb));
	host_probe();
	zassert_false(ntb_link_is_up(ntb), "host alone");
	zassert_equal(sys_read16(ctrl(NTB_EPF_LINK_STATUS)) & 1, 0);

	zassert_ok(ntb_link_enable(ntb, true));
	zassert_true(ntb_link_is_up(ntb));
	zassert_equal(sys_read16(ctrl(NTB_EPF_LINK_STATUS)) & 1, 1);
	zassert_ok(k_sem_take(&link_sem, K_MSEC(100)), "local link event");
	zassert_equal(fake_irq_msix[1], 1, "link event at the host: vector 0");

	/* host goes away: down at both sides */
	zassert_equal(host_cmd(CMD_LINK_DOWN, 0), STATUS_OK);
	zassert_false(ntb_link_is_up(ntb));
	zassert_ok(k_sem_take(&link_sem, K_MSEC(100)));
	zassert_equal(fake_irq_msix[1], 2);

	zassert_equal(host_cmd(CMD_LINK_UP, 0), STATUS_OK);
	zassert_true(ntb_link_is_up(ntb));
	zassert_ok(ntb_link_enable(ntb, false));
	zassert_false(ntb_link_is_up(ntb));
	host_remove();
}

ZTEST(epf_vntb, test_spads)
{
	/* host spad n is Zephyr's peer spad n and the other way around */
	host_spad_write(3, 0x12345678);
	zassert_equal(ntb_peer_spad_read(ntb, 3), 0x12345678);
	zassert_ok(ntb_spad_write(ntb, 5, 0xcafe0005));
	zassert_equal(host_peer_spad(5), 0xcafe0005);
	zassert_equal(ntb_spad_read(ntb, 5), 0xcafe0005);
	zassert_ok(ntb_peer_spad_write(ntb, 7, 0x77));
	zassert_equal(hrd(hrd(NTB_EPF_SPAD_OFFSET) + 4 * 7), 0x77);
	zassert_equal(ntb_spad_write(ntb, SPADS, 1), -EINVAL);
	zassert_equal(ntb_spad_read(ntb, -1), 0);
}

ZTEST(epf_vntb, test_doorbells)
{
	host_probe();
	zassert_ok(ntb_link_enable(ntb, true));

	/* host rings DB#0 and DB#4 */
	host_ring(0);
	host_ring(4);
	zassert_ok(k_sem_take(&db_sem, K_MSEC(100)));
	zassert_ok(k_sem_take(&db_sem, K_MSEC(100)));
	zassert_equal(db_vectors, BIT(0) | BIT(4));
	zassert_equal(ntb_db_read(ntb), BIT(0) | BIT(4));
	zassert_equal(sys_read32((mem_addr_t)fake_bar[2].virt + 4 * 2), 0, "slot consumed");
	zassert_ok(ntb_db_clear(ntb, BIT(0)));
	zassert_equal(ntb_db_read(ntb), BIT(4));
	zassert_ok(ntb_db_clear(ntb, BIT(4)));

	/* Zephyr rings DB#1 and DB#5: vectors 3 and 7 (1-based 4 and 8) */
	zassert_ok(ntb_peer_db_set(ntb, BIT(1) | BIT(5)));
	zassert_equal(fake_irq_msix[4], 1);
	zassert_equal(fake_irq_msix[8], 1);
	zassert_equal(ntb_peer_db_set(ntb, BIT(6)), -EINVAL, "beyond db-count");

	/* no bus mastering: nothing goes out */
	fake_bme = false;
	k_msleep(2);
	zassert_equal(ntb_peer_db_set(ntb, BIT(1)), -EACCES);
	fake_bme = true;
	k_msleep(2);

	zassert_ok(ntb_link_enable(ntb, false));
	host_remove();
	zassert_equal(ntb_peer_db_set(ntb, BIT(1)), -ENOTCONN, "doorbells torn down");
}

ZTEST(epf_vntb, test_msi_fallback)
{
	/* a host without MSI-X: 4 MSI vectors, 2 doorbells */
	zassert_equal(host_cmd(CMD_CONFIGURE_DOORBELL, 4), STATUS_OK);
	zassert_equal(ntb_db_valid_mask(ntb), 0x3);
	zassert_ok(ntb_peer_db_set(ntb, BIT(1)));
	zassert_equal(fake_irq_msi[4], 1);
	zassert_equal(fake_irq_msix[4], 0);
	zassert_equal(host_cmd(CMD_TEARDOWN_DOORBELL, 4), STATUS_OK);
	zassert_equal(ntb_db_valid_mask(ntb), 0x3f);
}

ZTEST(epf_vntb, test_windows)
{
	static uint8_t pattern[256];
	volatile uint8_t *peer;
	uint64_t phys;
	size_t size;
	void *va;

	host_probe();

	/* inbound: the host writes the BAR, Zephyr reads its memory */
	zassert_ok(ntb_mw_get_local(ntb, 0, &va, &phys, &size));
	zassert_equal(size, MW_SIZE);
	zassert_equal(va, fake_bar[4].virt);
	zassert_equal(phys, fake_bar[4].phys);
	memcpy((uint8_t *)fake_bar[4].virt + 0x100, "inbound", 8);
	zassert_mem_equal((uint8_t *)va + 0x100, "inbound", 8);

	/* outbound: no window before the host maps one */
	zassert_equal(ntb_peer_mw_get_addr(ntb, 0, (volatile void **)&peer, &size), -ENODATA);

	/* ntb_epf_mw_set_trans(): host buffer at HOST_BASE + 0x4000, 32 KiB */
	hwr(NTB_EPF_LOWER_ADDR, (uint32_t)(HOST_BASE + 0x4000));
	hwr(NTB_EPF_UPPER_ADDR, (uint32_t)((HOST_BASE + 0x4000) >> 32));
	hwr(NTB_EPF_LOWER_SIZE, 0x8000);
	hwr(NTB_EPF_UPPER_SIZE, 0);
	zassert_equal(host_cmd(CMD_CONFIGURE_MW, 0), STATUS_OK);
	zassert_equal(host_cmd(CMD_CONFIGURE_MW, 1), STATUS_ERROR, "one window only");

	zassert_ok(ntb_peer_mw_get_addr(ntb, 0, (volatile void **)&peer, &size));
	zassert_equal(size, 0x8000);
	peer[0x10] = 0xa5;
	fake_ob_flush();
	zassert_equal(fake_host_mem[0x4010], 0xa5, "CPU store through the window");

	for (int i = 0; i < sizeof(pattern); i++) {
		pattern[i] = (uint8_t)(i ^ 0x5a);
	}
	zassert_ok(ntb_peer_mw_xfer(ntb, 0, 0x7f00, pattern, sizeof(pattern), true));
	zassert_mem_equal(&fake_host_mem[0x4000 + 0x7f00], pattern, sizeof(pattern));
	memset(pattern, 0, sizeof(pattern));
	zassert_ok(ntb_peer_mw_xfer(ntb, 0, 0x7f00, pattern, sizeof(pattern), false));
	zassert_equal(pattern[1], 1 ^ 0x5a);
	zassert_equal(ntb_peer_mw_xfer(ntb, 0, 0x7f01, pattern, sizeof(pattern), true), -EINVAL,
		      "beyond the host window");

	fake_bme = false;
	k_msleep(2);
	zassert_equal(ntb_peer_mw_xfer(ntb, 0, 0, pattern, 4, true), -EACCES);
	fake_bme = true;
	k_msleep(2);

	zassert_equal(host_cmd(CMD_TEARDOWN_MW, 0), STATUS_OK);
	zassert_equal(ntb_peer_mw_get_addr(ntb, 0, (volatile void **)&peer, &size), -ENODATA);
	zassert_equal(fake_maps, 0);
	host_remove();
}

ZTEST(epf_vntb, test_x_unknown_command)
{
	zassert_equal(host_cmd(42, 0), STATUS_ERROR);
	zassert_equal(hrd(NTB_EPF_COMMAND), 0, "consumed");
}

static __maybe_unused uint32_t host_spad(int idx)
{
	return hrd(hrd(NTB_EPF_SPAD_OFFSET) + 4 * idx);
}

/* the host writes its peer scratchpads, which are Zephyr's own ones */
static __maybe_unused void host_peer_spad_write(int idx, uint32_t v)
{
	hwr(hrd(NTB_EPF_SPAD_OFFSET) + 4 * SPADS + 4 * idx, v);
}

ZTEST(epf_vntb, test_y_shell)
{
#ifdef CONFIG_SHELL_BACKEND_DUMMY
	const struct shell *sh = shell_backend_dummy_get_ptr();
	size_t len;

	/* the shell is the client now */
	zassert_ok(ntb_set_callbacks(ntb, NULL, NULL));
	host_probe();

	zassert_ok(shell_execute_cmd(sh, "ntb link up"));
	zassert_true(ntb_link_is_up(ntb));
	zassert_ok(shell_execute_cmd(sh, "ntb echo on"));

	/* echo: host puts 41 into Zephyr's spad 0, rings DB#1 */
	k_sem_reset(&fake_irq_sem);
	host_peer_spad_write(0, 41);
	host_ring(1);
	zassert_ok(k_sem_take(&fake_irq_sem, K_MSEC(200)));
	zassert_equal(host_spad(0), 42, "answer in the host's own spad 0");
	zassert_equal(fake_irq_msix[4], 1, "DB#1 back: vector 4");

	zassert_ok(shell_execute_cmd(sh, "ntb spad 9 0x99"));
	zassert_equal(host_peer_spad(9), 0x99);
	zassert_ok(shell_execute_cmd(sh, "ntb peer_spad 2 0x22"));
	zassert_equal(host_spad(2), 0x22);
	zassert_ok(shell_execute_cmd(sh, "ntb peer_db 0x4"));
	zassert_equal(fake_irq_msix[5], 1);

	/* inbound window: fill and crc agree, and the host sees the pattern */
	zassert_ok(shell_execute_cmd(sh, "ntb mw 0 fill 0x1000 8192"));
	zassert_equal(sys_read32((mem_addr_t)fake_bar[4].virt + 8), 0x1002);
	shell_backend_dummy_clear_output(sh);
	zassert_ok(shell_execute_cmd(sh, "ntb mw 0 crc 0 8192"));
	zassert_not_null(strstr(shell_backend_dummy_get_output(sh, &len), "8192 bytes"));

	/* host window */
	hwr(NTB_EPF_LOWER_ADDR, (uint32_t)(HOST_BASE + 0x1000));
	hwr(NTB_EPF_UPPER_ADDR, (uint32_t)((HOST_BASE + 0x1000) >> 32));
	hwr(NTB_EPF_LOWER_SIZE, 0x2000);
	hwr(NTB_EPF_UPPER_SIZE, 0);
	zassert_equal(host_cmd(CMD_CONFIGURE_MW, 0), STATUS_OK);
	zassert_ok(shell_execute_cmd(sh, "ntb peer_mw 0 wr 0x10 0x11223344 0x55667788"));
	zassert_equal(sys_get_le32(&fake_host_mem[0x1014]), 0x55667788);
	zassert_ok(shell_execute_cmd(sh, "ntb peer_mw 0 fill 0xabc0000"));
	zassert_equal(sys_get_le32(&fake_host_mem[0x1000 + 0x1ffc]), 0xabc0000 + 0x7ff);
	zassert_not_ok(shell_execute_cmd(sh, "ntb peer_mw 0 rd 0x1ff0 32"), "beyond");

	zassert_ok(shell_execute_cmd(sh, "ntb status"));
	zassert_ok(shell_execute_cmd(sh, "ntb spad"));
	zassert_ok(shell_execute_cmd(sh, "vntb"));
	shell_backend_dummy_clear_output(sh);
	zassert_ok(shell_execute_cmd(sh, "ntb dev"));
	zassert_not_null(strstr(shell_backend_dummy_get_output(sh, &len), "* function@0"));

	zassert_ok(shell_execute_cmd(sh, "ntb echo off"));
	zassert_ok(shell_execute_cmd(sh, "ntb link down"));
	zassert_equal(host_cmd(CMD_TEARDOWN_MW, 0), STATUS_OK);
	host_remove();
	zassert_ok(ntb_set_callbacks(ntb, &cb, NULL));
#else
	ztest_test_skip();
#endif
}

ZTEST(epf_vntb, test_z_link_down_resets)
{
	host_probe();
	zassert_ok(ntb_link_enable(ntb, true));
	hwr(NTB_EPF_LOWER_ADDR, (uint32_t)HOST_BASE);
	hwr(NTB_EPF_UPPER_ADDR, (uint32_t)(HOST_BASE >> 32));
	hwr(NTB_EPF_LOWER_SIZE, 0x1000);
	hwr(NTB_EPF_UPPER_SIZE, 0);
	zassert_equal(host_cmd(CMD_CONFIGURE_MW, 0), STATUS_OK);
	host_spad_write(0, 0xdead);
	k_sem_reset(&link_sem);

	/* host reboot */
	pcie_epc_notify(epc, PCIE_EPC_EVT_HOT_RESET, 0);
	zassert_ok(k_sem_take(&link_sem, K_MSEC(200)));
	zassert_false(ntb_link_is_up(ntb));
	zassert_equal(fake_maps, 0, "host window unmapped");
	zassert_equal(ntb_peer_spad_read(ntb, 0), 0, "host scratchpads cleared");
	zassert_equal(ntb_peer_db_set(ntb, BIT(0)), -ENOTCONN);

	/* the host comes back and finds Zephyr's side still enabled */
	host_probe();
	zassert_true(ntb_link_is_up(ntb));
	zassert_ok(ntb_link_enable(ntb, false));
	host_remove();
}

ZTEST_SUITE(epf_vntb, NULL, setup, before, NULL, NULL);
