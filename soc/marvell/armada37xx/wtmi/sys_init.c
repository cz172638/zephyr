/*
* ***************************************************************************
* Copyright (C) 2017 Marvell International Ltd.
* ***************************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of Marvell nor the names of its contributors may be used
* to endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************
*/

/*
 * Adapted for Zephyr on the Armada 3720 Cortex-M3 by Jiri Kastner (2026)
 * from A3700-utils-marvell wtmi/sys_init/main.c and apctl.c: clock preset,
 * AVS, DDR tuning and the release of the A53, run from
 * soc_early_init_hook() before the kernel starts. Output goes straight to
 * UART0 (the console driver is not up yet).
 */

#include <stdarg.h>

#include <zephyr/sys/cbprintf.h>

#include "sys.h"
#include "clock.h"
#include "avs.h"
#include "ddr/ddrcore.h"
#include "a3700_wtmi.h"

#define _MB(sz) ((sz) << 20)

/* Armada 37xx always works with a DDR bus of 16 bits */
#define MC_BUS_WIDTH 16
#define DDR_CS_CAP(dev_sz, dev_bw) ((dev_sz) * (MC_BUS_WIDTH / (dev_bw)))

/* Two CM3 DRAM decode windows, up to 1.5 GiB */
#define CM3_DRAM_WIN0_ID     0
#define CM3_DRAM_WIN0_BASE   0x60000000
#define CM3_DRAM_WIN0_SZ_MAX 0x40000000
#define CM3_DRAM_WIN1_ID     1
#define CM3_DRAM_WIN1_BASE   0xA0000000
#define CM3_DRAM_WIN1_SZ_MAX 0x20000000

/* Mailbox to the A53 BootROM */
#define HOST_INT_CMD_COMPLETE_BIT BIT0
#define MBOX_STATUS_START_AP      0x1003

#define UART0_TX    (MVEBU_UART0_REG_BASE + 0x04)
#define UART0_STAT  (MVEBU_UART0_REG_BASE + 0x0c)
#define UART_TX_FUL BIT11
#define UART_TX_EMP BIT6

static int uart_out(int c, void *ctx)
{
	ARG_UNUSED(ctx);

	if (c == '\n') {
		(void)uart_out('\r', NULL);
	}
	while (readl(UART0_STAT) & UART_TX_FUL) {
	}
	writel(c, UART0_TX);
	return c;
}

int a3700_wtmi_printf(const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = cbvprintf(uart_out, NULL, fmt, ap);
	va_end(ap);
	return n;
}

static void set_cm3_win_remap(u32 win, u32 remap_addr)
{
	u32 reg = readl(CM3_WIN_CONROL(win));

	writel(reg & ~BIT0, CM3_WIN_CONROL(win));
	writel(remap_addr & 0xFFFF0000, CM3_WIN_REMAP_LOW(win));
	writel(reg | BIT0, CM3_WIN_CONROL(win));
}

static u32 do_checksum32(u32 *start, u32 len)
{
	u32 sum = 0;

	while (len >= 4) {
		sum += *start++;
		len -= 4;
	}
	return sum;
}

/* warm boot flag in BIT0 of 0xC001404C */
static int sys_check_warm_boot(void)
{
	return (readl(0xC001404C) & BIT0) ? 1 : 0;
}

void a3700_wtmi_start_ap(uint32_t entry)
{
	writel(entry, MAILBOX_OUT_ARG(0));
	writel(MBOX_STATUS_START_AP, MAILBOX_OUT_STATUS);
	writel(readl(HOST_INT_SET) | HOST_INT_CMD_COMPLETE_BIT, HOST_INT_SET);
}

int a3700_wtmi_sys_init(void)
{
	struct ddr_topology map;
	struct ddr_init_para ddr_para = {0};
	struct ddr_init_result *result_in_dram, result_in_sram;
	u32 chksum_in_dram = 0;

	/* busy-wait calibration for wait_ns() */
	clock_init();

	printf("Zephyr CM3: system early-init\n");

	result_in_dram = (struct ddr_init_result *)(DDR_TUNE_RESULT_MEM_BASE);

	ddr_para.warm_boot = sys_check_warm_boot();
	if (ddr_para.warm_boot) {
		chksum_in_dram = *((u32 *)(DDR_TUNE_RESULT_MEM_BASE +
					   sizeof(struct ddr_init_result)));
		if (chksum_in_dram !=
		    do_checksum32((u32 *)result_in_dram, sizeof(struct ddr_init_result))) {
			printf("DDR tuning result checksum ERROR!\n");
		}
	}

	map.bus_width = CONFIG_BUS_WIDTH;
	map.cs_num = CONFIG_CS_NUM;
	map.cs[0].group_num = 0;
	map.cs[0].bank_num = 8;
	map.cs[0].capacity = DDR_CS_CAP(CONFIG_DEV_CAP, CONFIG_BUS_WIDTH);
	if (map.cs_num > 1) {
		map.cs[1].group_num = 0;
		map.cs[1].bank_num = 8;
		map.cs[1].capacity = DDR_CS_CAP(CONFIG_DEV_CAP, CONFIG_BUS_WIDTH);
	}

	set_clock_preset(WTMI_CLOCK);
	init_avs(get_cpu_clock());

	set_ddr_type(CONFIG_DDR_TYPE);
	set_ddr_topology_parameters(map);

	ddr_para.log_level = LOG_LEVEL_ERROR;
	ddr_para.flags = FLAG_REGS_DUMP_ALL;
	ddr_para.clock_init = setup_clock_tree;
	ddr_para.speed = get_ddr_clock();

	ddr_para.cs_wins[0].base = CM3_DRAM_WIN0_BASE;
	ddr_para.cs_wins[0].size = (_MB(map.cs[0].capacity) > CM3_DRAM_WIN0_SZ_MAX)
					   ? CM3_DRAM_WIN0_SZ_MAX
					   : _MB(map.cs[0].capacity);

	if (map.cs_num > 1) {
		if (_MB(map.cs[0].capacity) < CM3_DRAM_WIN0_SZ_MAX) {
			ddr_para.cs_wins[1].base = CM3_DRAM_WIN0_BASE + _MB(map.cs[0].capacity);
			ddr_para.cs_wins[1].size =
				MIN(_MB(map.cs[1].capacity),
				    (CM3_DRAM_WIN0_SZ_MAX + CM3_DRAM_WIN1_SZ_MAX -
				     _MB(map.cs[0].capacity)));
			set_cm3_win_remap(CM3_DRAM_WIN1_ID, CM3_DRAM_WIN0_SZ_MAX);
		} else {
			ddr_para.cs_wins[1].base = CM3_DRAM_WIN1_BASE;
			ddr_para.cs_wins[1].size =
				MIN(_MB(map.cs[1].capacity), CM3_DRAM_WIN1_SZ_MAX);
			set_cm3_win_remap(CM3_DRAM_WIN1_ID, _MB(map.cs[0].capacity));
		}
	}

	/*
	 * On a warm boot the stored tuning is reused; otherwise tune into
	 * SRAM first (the training uses DRAM) and store the result after.
	 */
	if (ddr_para.warm_boot) {
		init_ddr(ddr_para, result_in_dram);
	} else {
		init_ddr(ddr_para, &result_in_sram);
		*result_in_dram = result_in_sram;
		*((u32 *)(DDR_TUNE_RESULT_MEM_BASE + sizeof(struct ddr_init_result))) =
			do_checksum32((u32 *)&result_in_sram, sizeof(struct ddr_init_result));
	}

	printf("Zephyr CM3: CPU %d MHz, DDR %d MHz\n", get_cpu_clock(), get_ddr_clock());
	return 0;
}
