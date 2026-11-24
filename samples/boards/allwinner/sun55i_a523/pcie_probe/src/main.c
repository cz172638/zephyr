/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Step-by-step bring-up of the A523 PCIe controller (DesignWare) from the
 * shell, to find out whether it runs as an endpoint. Sequence and values
 * follow the vendor BSP (pcie-sunxi-plat.c, sunxi-inno-combophy.c) and the
 * Linux PCK-600 power domain driver.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/* CCU */
#define CCU_PHYS          0x02001000
#define CCU_MBUS_GATE     0x804
#define CCU_MBUS_USB3     BIT(6)
#define CCU_USB3_REF_CLK  0xa84 /* [31] gate, [26:24] 0 HOSC 1 PERI0_200M, [4:0] M */
#define CCU_PCIE_AUX_CLK  0xaa0 /* [31] gate, [24] 0 HOSC, [4:0] M */
#define CCU_PCIE_BGR      0xaac /* [16] PCIE_USB3_SYS_RST, 1 = de-assert */

/* PIO, new layout: 0x30 per bank, CFG0..3 at 0x00, DAT at 0x10 */
#define PIO_PHYS        0x02000000
#define PIO_BANK(p)     (((p) - 'A') * 0x30)
#define PIO_CFG(p, pin) (PIO_BANK(p) + ((pin) / 8U) * 4U)
#define PIO_DAT(p)      (PIO_BANK(p) + 0x10)

/* R_CCU and PCK-600 (PPU per domain, 0x1000 each) */
#define R_CCU_PHYS     0x07010000
#define R_CCU_PPU_BGR  0x1ac /* [0] PPU0 gate, [16] PPU0 reset */
#define PPU_PHYS       0x07060000
#define PPU_PD_PCIE    7
#define PPU_PWPR       0x000
#define PPU_PWSR       0x008
#define PPU_DCDR0      0x170
#define PPU_DCDR1      0x174
#define PPU_LPS0_DELAY 0xc00
#define PPU_LPS1_DELAY 0xc04
#define PPU_OFF2ON     0xc10
#define PPU_MODE_ON    0x8U

/* PCIe/USB3 sub-system ("combo") application registers and INNO PHY */
#define COMBO_PHYS      0x04f00000
#define COMBO_PCIEBGR   0x004 /* [17] ACLK, [16] HCLK, [1] PERSTN, [0] POWER_UP_RSTN */
#define COMBO_PHYCTL    0x010 /* [31] 0 PCIe, [30] refclk, [8] pipe rst sel, [0] PHY_RSTN */
#define COMBO_PHYCTL_FPGA_SYS_RSTN BIT(1)
#define COMBO_PHYSTS0   0x080
#define COMBO_PHYSTS1   0x084
#define INNO_PHYS       0x04f80000

/* DesignWare controller */
#define DBI_PHYS        0x04800000
#define DBI2_PHYS       0x04900000
#define APP_PHYS        0x04c00000
#define APP_LTSSM_CTRL  0xc00 /* [7:4] device type (BSP): 0 EP, 4 RC; [0] LTSSM enable */
#define APP_DEV_TYPE    GENMASK(7, 4)
#define DBI_MISC_CTRL1  0x8bc /* [0] DBI_RO_WR_EN */
#define DBI_PL_DEBUG0   0x728 /* [5:0] LTSSM state */
#define DBI_PL_DEBUG1   0x72c

struct rgn {
	const char *name;
	uintptr_t phys;
	size_t size;
	mm_reg_t va;
};

static struct rgn rgns[] = {
	{"ccu", CCU_PHYS, 0x1000}, {"pio", PIO_PHYS, 0x1000},
	{"rccu", R_CCU_PHYS, 0x1000}, {"ppu", PPU_PHYS, 0x8000},
	{"combo", COMBO_PHYS, 0x1000}, {"inno", INNO_PHYS, 0x2000},
	{"dbi", DBI_PHYS, 0x1000}, {"dbi2", DBI2_PHYS, 0x1000},
	{"app", APP_PHYS, 0x2000},
};

enum { CCU, PIO, RCCU, PPU, COMBO, INNO, DBI, DBI2, APP };

static uint32_t rd(int r, uint32_t off)
{
	return sys_read32(rgns[r].va + off);
}

static void wr(int r, uint32_t off, uint32_t v)
{
	sys_write32(v, rgns[r].va + off);
}

static void rmw(int r, uint32_t off, uint32_t clr, uint32_t set)
{
	wr(r, off, (rd(r, off) & ~clr) | set);
}

static void map_all(void)
{
	static bool done;

	if (done) {
		return;
	}
	for (size_t i = 0; i < ARRAY_SIZE(rgns); i++) {
		device_map(&rgns[i].va, rgns[i].phys, rgns[i].size, K_MEM_CACHE_NONE);
	}
	done = true;
}

/* PCIe power domain of the PCK-600 (Linux sun55i-pck600.c) */
static int step_power(const struct shell *sh)
{
	uint32_t pd = PPU_PD_PCIE * 0x1000U;
	int64_t t0;

	rmw(RCCU, R_CCU_PPU_BGR, 0, BIT(0) | BIT(16));
	wr(PPU, pd + PPU_DCDR0, 0xffffff);
	wr(PPU, pd + PPU_DCDR1, 0xffff);
	wr(PPU, pd + PPU_LPS0_DELAY, 0x08080808);
	wr(PPU, pd + PPU_LPS1_DELAY, 0x0808);
	wr(PPU, pd + PPU_OFF2ON, 0x8);
	shell_print(sh, "power: PWSR before %08x", rd(PPU, pd + PPU_PWSR));
	rmw(PPU, pd + PPU_PWPR, 0xf, PPU_MODE_ON);
	(void)rd(PPU, pd + PPU_PWPR);
	t0 = k_uptime_get();
	while ((rd(PPU, pd + PPU_PWSR) & 0xf) != PPU_MODE_ON) {
		if (k_uptime_get() - t0 > 10) {
			shell_error(sh, "power: PCIE domain did not turn on, PWSR %08x",
				    rd(PPU, pd + PPU_PWSR));
			return -ETIMEDOUT;
		}
	}
	shell_print(sh, "power: PCIE domain on, PWSR %08x", rd(PPU, pd + PPU_PWSR));
	return 0;
}

/* reference clock 100 MHz (PERI0_200M / 2), AUX from HOSC, MBUS gate, sub-system reset */
static int step_clocks(const struct shell *sh)
{
	wr(CCU, CCU_USB3_REF_CLK, BIT(31) | (1U << 24) | 1U);
	rmw(CCU, CCU_PCIE_AUX_CLK, BIT(24) | GENMASK(4, 0), BIT(31));
	rmw(CCU, CCU_MBUS_GATE, 0, CCU_MBUS_USB3);
	rmw(CCU, CCU_PCIE_BGR, 0, BIT(16));
	shell_print(sh, "clocks: ref %08x aux %08x mbus %08x bgr %08x", rd(CCU, CCU_USB3_REF_CLK),
		    rd(CCU, CCU_PCIE_AUX_CLK), rd(CCU, CCU_MBUS_GATE), rd(CCU, CCU_PCIE_BGR));
	shell_print(sh, "combo: version %08x", rd(COMBO, 0));
	return 0;
}

/* Cubie A5E: GMA340 SEL = PB6 (1 = M.2 PCIe, 0 = USB-A USB3), OE# = PB7 */
static int step_gma(const struct shell *sh, bool pcie)
{
	rmw(PIO, PIO_CFG('B', 6), GENMASK(31, 24), (1U << 24) | (1U << 28));
	rmw(PIO, PIO_DAT('B'), BIT(6) | BIT(7), pcie ? BIT(6) : 0);
	shell_print(sh, "gma340: %s, PB_CFG0 %08x PB_DAT %08x", pcie ? "M.2 PCIe" : "USB3",
		    rd(PIO, PIO_CFG('B', 0)), rd(PIO, PIO_DAT('B')));
	return 0;
}

/* sub-system clocks; resets of the controller stay asserted when hold is set */
static int step_combo(const struct shell *sh, bool hold)
{
	rmw(COMBO, COMBO_PCIEBGR, 0, BIT(17) | BIT(16));
	if (!hold) {
		rmw(COMBO, COMBO_PCIEBGR, 0, BIT(1) | BIT(0));
	}
	shell_print(sh, "combo: PCIEBGR %08x", rd(COMBO, COMBO_PCIEBGR));
	return 0;
}

static void step_release(const struct shell *sh)
{
	rmw(COMBO, COMBO_PCIEBGR, 0, BIT(1) | BIT(0));
	shell_print(sh, "release: PCIEBGR %08x", rd(COMBO, COMBO_PCIEBGR));
}

/* INNO combo PHY: 100 MHz internal reference (BSP sunxi_combphy_pcie_phy_100M, v1) */
static int step_phy(const struct shell *sh)
{
	rmw(INNO, 0x1004, (0x3U << 3) | BIT(0), BIT(0) | BIT(2) | BIT(4));
	rmw(INNO, 0x1018, 0x3U << 4, 0x3U << 4);
	rmw(INNO, 0x101c, 0x0fffffffU, 0);
	rmw(INNO, 0x107c, 0x3ffffU, (0x2U << 12) | 0x32U);
	rmw(INNO, 0x1030, 0x3U << 20, 0);
	rmw(INNO, 0x1050, 0x7U << 5, 0x1U << 5);
	rmw(INNO, 0x1054, 0x7U << 5, 0x1U << 5);
	rmw(INNO, 0x0804, 0xfU << 4, 0xcU << 4);
	rmw(INNO, 0x109c, 0x3U << 8, BIT(1));
	wr(INNO, 0x1418, 0x80540a0a);

	/* PHY used by PCIe, internal reference, assert then release */
	rmw(COMBO, COMBO_PHYCTL, BIT(31) | BIT(30) | (0x3U << 8) | COMBO_PHYCTL_FPGA_SYS_RSTN | BIT(0),
	    0);
	rmw(COMBO, COMBO_PHYCTL, 0, BIT(0));
	rmw(COMBO, COMBO_PHYCTL, 0, COMBO_PHYCTL_FPGA_SYS_RSTN);
	k_busy_wait(100);
	shell_print(sh, "phy: PHYCTL %08x PHYSTS0 %08x PHYSTS1 %08x", rd(COMBO, COMBO_PHYCTL),
		    rd(COMBO, COMBO_PHYSTS0), rd(COMBO, COMBO_PHYSTS1));
	return 0;
}

/* pulse the sub-system power-up reset in the CCU; it also clears PCIEBGR */
static void step_ccurst(const struct shell *sh)
{
	rmw(CCU, CCU_PCIE_BGR, BIT(16), 0);
	k_busy_wait(10);
	rmw(CCU, CCU_PCIE_BGR, 0, BIT(16));
	shell_print(sh, "ccurst: bgr %08x, PCIEBGR %08x", rd(CCU, CCU_PCIE_BGR),
		    rd(COMBO, COMBO_PCIEBGR));
}

/* PCIe domain of the PCK-600 off and on again */
static int step_pdcycle(const struct shell *sh)
{
	uint32_t pd = PPU_PD_PCIE * 0x1000U;
	int64_t t0 = k_uptime_get();

	rmw(PPU, pd + PPU_PWPR, 0xf, 0);
	while ((rd(PPU, pd + PPU_PWSR) & 0xf) != 0) {
		if (k_uptime_get() - t0 > 10) {
			shell_error(sh, "pdcycle: domain did not turn off, PWSR %08x",
				    rd(PPU, pd + PPU_PWSR));
			return -ETIMEDOUT;
		}
	}
	shell_print(sh, "pdcycle: off, PWSR %08x", rd(PPU, pd + PPU_PWSR));
	k_busy_wait(100);
	return step_power(sh);
}

static int step_mode(const struct shell *sh, bool ep)
{
	rmw(APP, APP_LTSSM_CTRL, APP_DEV_TYPE, ep ? 0 : BIT(6));
	shell_print(sh, "mode: %s, LTSSM_CTRL %08x", ep ? "EP" : "RC", rd(APP, APP_LTSSM_CTRL));
	return 0;
}

/* DBI2 shadow of BAR0 (mask) and class code writability */
static void probe_writable(const struct shell *sh)
{
	uint32_t bar0 = rd(DBI, 0x10), mask0 = rd(DBI2, 0x10), cls = rd(DBI, 0x08);
	uint32_t m, c;

	rmw(DBI, DBI_MISC_CTRL1, 0, BIT(0));
	wr(DBI2, 0x10, 0x000fffffU);
	m = rd(DBI2, 0x10);
	wr(DBI2, 0x10, mask0);
	wr(DBI, 0x08, (cls & 0xff) | 0x05800000U);
	c = rd(DBI, 0x08);
	wr(DBI, 0x08, cls);
	rmw(DBI, DBI_MISC_CTRL1, BIT(0), 0);
	shell_print(sh, "  BAR0 %08x, DBI2 BAR0 mask %08x -> wrote 000fffff, read %08x", bar0,
		    mask0, m);
	shell_print(sh, "  class %08x -> wrote %08x, read %08x", cls, (cls & 0xff) | 0x05800000U,
		    c);
}

static int step_dump(const struct shell *sh)
{
	uint32_t id = rd(DBI, 0x00), hdr = rd(DBI, 0x0c);
	uint8_t cap = rd(DBI, 0x34) & 0xff;

	shell_print(sh, "app: LTSSM_CTRL %08x CLK_REQ %08x INIT_RST %08x SII_ELEC %08x",
		    rd(APP, APP_LTSSM_CTRL), rd(APP, 0x400), rd(APP, 0x500), rd(APP, 0xa00));
	shell_print(sh, "app: SII_INT0 %08x SII_INT1 %08x", rd(APP, 0xe08), rd(APP, 0xe0c));
	shell_print(sh, "dbi: id %08x cmd/sts %08x class %08x hdr type %02x (%s)", id,
		    rd(DBI, 0x04), rd(DBI, 0x08), (hdr >> 16) & 0x7f,
		    ((hdr >> 16) & 0x7f) == 0 ? "EP" : "RC/bridge");
	/* walk to the PCIe capability */
	for (int n = 0; cap != 0 && n < 16; n++) {
		uint32_t h = rd(DBI, cap & ~3U);

		if ((h & 0xff) == 0x10) {
			uint32_t cp = rd(DBI, cap & ~3U);
			uint32_t ls = rd(DBI, (cap & ~3U) + 0x10);

			shell_print(sh, "dbi: PCIe cap @%02x type %u, LnkCap %08x LnkSta %04x", cap,
				    (cp >> 20) & 0xf, rd(DBI, (cap & ~3U) + 0x0c), ls >> 16);
			break;
		}
		cap = (h >> 8) & 0xff;
	}
	shell_print(sh, "dbi: PL_DEBUG0 %08x (LTSSM %02x) PL_DEBUG1 %08x", rd(DBI, DBI_PL_DEBUG0),
		    rd(DBI, DBI_PL_DEBUG0) & 0x3f, rd(DBI, DBI_PL_DEBUG1));
	probe_writable(sh);
	return 0;
}

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_power(sh);
}

static int cmd_clocks(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_clocks(sh);
}

static int cmd_gma(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_gma(sh, argc < 2 || strcmp(argv[1], "usb") != 0);
}

static int cmd_combo(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_combo(sh, argc >= 2 && strcmp(argv[1], "hold") == 0);
}

static int cmd_release(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	step_release(sh);
	return 0;
}

static int cmd_phy(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_phy(sh);
}

static int cmd_ccurst(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	step_ccurst(sh);
	return 0;
}

static int cmd_pdcycle(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_pdcycle(sh);
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_mode(sh, strcmp(argv[1], "ep") == 0);
}

static int cmd_ltssm(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	rmw(APP, APP_LTSSM_CTRL, BIT(0), strcmp(argv[1], "on") == 0 ? BIT(0) : 0);
	shell_print(sh, "LTSSM_CTRL %08x", rd(APP, APP_LTSSM_CTRL));
	return 0;
}

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	map_all();
	return step_dump(sh);
}

/*
 * seq ep|rc: power, clocks, M.2, sub-system clocks with the controller held
 * in reset, role, release, PHY, dump. seq bsp-ep: the BSP order, which sets
 * the role only after everything runs.
 */
static int cmd_seq(const struct shell *sh, size_t argc, char **argv)
{
	bool ep = strcmp(argv[1], "rc") != 0;
	bool bsp = strcmp(argv[1], "bsp-ep") == 0;
	bool ccu = strcmp(argv[1], "ep-ccu") == 0;
	bool pdc = strcmp(argv[1], "ep-pd") == 0;
	int ret;

	map_all();
	ret = pdc ? step_pdcycle(sh) : step_power(sh);
	if (ret != 0) {
		return ret;
	}
	step_clocks(sh);
	step_gma(sh, true);
	if (ccu) {
		/* role written, then the CCU power-up reset with the role in place */
		step_combo(sh, true);
		step_mode(sh, true);
		step_ccurst(sh);
		step_combo(sh, true);
		step_mode(sh, true);
		step_release(sh);
		step_phy(sh);
	} else if (bsp) {
		step_combo(sh, false);
		step_phy(sh);
		step_mode(sh, true);
	} else {
		step_combo(sh, true);
		step_mode(sh, ep);
		step_release(sh);
		step_phy(sh);
	}
	return step_dump(sh);
}

static int cmd_rd(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa = strtoul(argv[1], NULL, 0);
	size_t n = argc > 2 ? strtoul(argv[2], NULL, 0) : 1;
	mm_reg_t va;

	/* never unmapped: the mapping is 1:1 and may be shared with the regions above */
	device_map(&va, ROUND_DOWN(pa, 0x1000), 0x2000, K_MEM_CACHE_NONE);
	for (size_t i = 0; i < n && i < 64; i++) {
		shell_print(sh, "%08lx: %08x", (unsigned long)(pa + i * 4U),
			    sys_read32(va + (pa & 0xfff) + i * 4U));
	}
	return 0;
}

static int cmd_wr(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t pa = strtoul(argv[1], NULL, 0);
	uint32_t v = strtoul(argv[2], NULL, 0);
	mm_reg_t va;

	device_map(&va, ROUND_DOWN(pa, 0x1000), 0x1000, K_MEM_CACHE_NONE);
	sys_write32(v, va + (pa & 0xfff));
	shell_print(sh, "%08lx: %08x", (unsigned long)pa, sys_read32(va + (pa & 0xfff)));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_a523pcie,
	SHELL_CMD(power, NULL, "Turn on the PCIe power domain (PCK-600)", cmd_power),
	SHELL_CMD(clocks, NULL, "Reference/AUX clocks, MBUS gate, sub-system reset", cmd_clocks),
	SHELL_CMD_ARG(gma, NULL, "gma [pcie|usb]: GMA340 lane switch (Cubie A5E)", cmd_gma, 1, 1),
	SHELL_CMD_ARG(combo, NULL, "combo [hold]: sub-system clocks (+ resets)", cmd_combo, 1, 1),
	SHELL_CMD(release, NULL, "Release PERSTN and POWER_UP_RSTN", cmd_release),
	SHELL_CMD(ccurst, NULL, "Pulse the sub-system power-up reset in the CCU", cmd_ccurst),
	SHELL_CMD(pdcycle, NULL, "PCIe domain of the PCK-600 off and on", cmd_pdcycle),
	SHELL_CMD(phy, NULL, "Set up the INNO combo PHY for PCIe", cmd_phy),
	SHELL_CMD_ARG(mode, NULL, "mode ep|rc: device type in LTSSM_CTRL[7:4]", cmd_mode, 2, 0),
	SHELL_CMD_ARG(ltssm, NULL, "ltssm on|off", cmd_ltssm, 2, 0),
	SHELL_CMD(dump, NULL, "Controller state, header type, writable BAR mask/class",
		  cmd_dump),
	SHELL_CMD_ARG(seq, NULL, "seq ep|rc|bsp-ep|ep-ccu|ep-pd: whole sequence + dump", cmd_seq, 2, 0),
	SHELL_CMD_ARG(rd, NULL, "rd <phys> [n]", cmd_rd, 2, 1),
	SHELL_CMD_ARG(wr, NULL, "wr <phys> <value>", cmd_wr, 3, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(a523pcie, &sub_a523pcie, "A523 PCIe controller probe", NULL);

int main(void)
{
	printk("a523pcie: ready\n");
	return 0;
}
