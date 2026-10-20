/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell access to the CCI-400 of the Armada 3700 (remapped by TF-A to
 * 0xfe000000). Register layout per the ARM CCI-400 TRM / TF-A cci.h:
 * control override 0x0, speculation 0x4, secure access 0x8, status 0xc,
 * slave interface n at 0x1000 * (n + 1): snoop control +0x0, shareable
 * override +0x4. The CPU cluster is on slave interface 3, interfaces
 * 0..2 are the ACE-Lite I/O ports.
 *
 * Without TF-A setting bit 0 of the secure access register, non-secure
 * writes are ignored.
 */

#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define CCI_BASE      0xfe000000UL
#define CCI_SIZE      0x10000
#define CCI_CTRL_OVR  0x0
#define CCI_SPEC_CTRL 0x4
#define CCI_SECURE    0x8
#define CCI_STATUS    0xc
#define CCI_IMP_ERR   0x10
#define CCI_PMU_CTRL  0x100
#define CCI_SL(n)     (0x1000 * ((n) + 1))
#define CCI_SL_SNOOP  0x0
#define CCI_SL_SHARE  0x4
#define CCI_SL_RQOS   0x100
#define CCI_SL_WQOS   0x104
#define CCI_SL_QOSCTL 0x10c
#define CCI_SL_MAXOT  0x110
#define CCI_NUM_SL    5

static mm_reg_t cci;

static mm_reg_t cci_base(void)
{
	if (cci == 0) {
		device_map(&cci, CCI_BASE, CCI_SIZE, K_MEM_CACHE_NONE);
	}
	return cci;
}

static uint32_t rd(uint32_t off)
{
	return sys_read32(cci_base() + off);
}

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t pid = (rd(0xfe0) & 0xff) | ((rd(0xfe4) & 0xff) << 8) | ((rd(0xfe8) & 0xff) << 16) |
		       ((rd(0xfec) & 0xff) << 24);

	shell_print(sh, "CCI @ 0x%lx: periph id %08x (part 0x%03x rev %u), comp id %02x %02x %02x %02x",
		    CCI_BASE, pid, pid & 0xfff, (rd(0xfe8) >> 4) & 0xf, rd(0xff0) & 0xff,
		    rd(0xff4) & 0xff, rd(0xff8) & 0xff, rd(0xffc) & 0xff);
	shell_print(sh, "ctrl override %08x  speculation %08x  secure access %08x", rd(CCI_CTRL_OVR),
		    rd(CCI_SPEC_CTRL), rd(CCI_SECURE));
	shell_print(sh, "status %08x  imprecise err %08x  pmu ctrl %08x", rd(CCI_STATUS),
		    rd(CCI_IMP_ERR), rd(CCI_PMU_CTRL));
	for (int i = 0; i < CCI_NUM_SL; i++) {
		uint32_t sn = rd(CCI_SL(i) + CCI_SL_SNOOP);
		uint32_t shr = rd(CCI_SL(i) + CCI_SL_SHARE);

		shell_print(sh,
			    "S%d: snoop %08x (%s%s%s%s) share-ovr %08x (%s) rqos %08x wqos %08x "
			    "qosctl %08x maxot %08x",
			    i, sn, (sn & BIT(30)) ? "snoop-capable " : "",
			    (sn & BIT(31)) ? "dvm-capable " : "", (sn & BIT(0)) ? "SNOOP " : "",
			    (sn & BIT(1)) ? "DVM" : "", shr,
			    (shr & 0x3) == 0x3 ? "shareable" : ((shr & 0x3) == 0x2 ? "non-shareable" : "none"),
			    rd(CCI_SL(i) + CCI_SL_RQOS), rd(CCI_SL(i) + CCI_SL_WQOS),
			    rd(CCI_SL(i) + CCI_SL_QOSCTL), rd(CCI_SL(i) + CCI_SL_MAXOT));
	}
	return 0;
}

/* cci share <slave> none|ns|sh: shareable override of an ACE-Lite port */
static int cmd_share(const struct shell *sh, size_t argc, char **argv)
{
	int i = atoi(argv[1]);
	uint32_t v;

	if (i < 0 || i >= CCI_NUM_SL) {
		return -EINVAL;
	}
	if (strcmp(argv[2], "none") == 0) {
		v = 0;
	} else if (strcmp(argv[2], "ns") == 0) {
		v = 2;
	} else if (strcmp(argv[2], "sh") == 0) {
		v = 3;
	} else {
		shell_error(sh, "none | ns | sh");
		return -EINVAL;
	}
	sys_write32(v, cci_base() + CCI_SL(i) + CCI_SL_SHARE);
	shell_print(sh, "S%d share-ovr %08x (reads back; 0 when TF-A keeps CCI secure)", i,
		    rd(CCI_SL(i) + CCI_SL_SHARE));
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cci, SHELL_CMD(dump, NULL, "CCI-400 registers", cmd_dump),
			       SHELL_CMD_ARG(share, NULL, "<slave> none|ns|sh Shareable override",
					     cmd_share, 3, 0),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(cci, &sub_cci, "Armada 3700 CCI-400", NULL);
