/*
 * Copyright (c) 2026 Jiri Kastner
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Marvell Armada 3700 rWTM mailbox: protocol shared by both ends.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_MVEBU_RWTM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_MVEBU_RWTM_H_

#include <stdint.h>

#define MVEBU_RWTM_NARGS 16

/* Reply status: command 9:0, value 29:10, error 31:30 (WTMI format) */
#define MVEBU_RWTM_STS_SUCCESS (0U << 30)
#define MVEBU_RWTM_STS_FAIL    (1U << 30)
#define MVEBU_RWTM_STS_BADCMD  (2U << 30)
#define MVEBU_RWTM_STS_LATER   (3U << 30)
#define MVEBU_RWTM_STS(cmd, val, err)                                                              \
	(((cmd) & 0x3ffU) | (((val) & 0xfffffU) << 10) | MVEBU_RWTM_STS_##err)
#define MVEBU_RWTM_STS_CMD(s)   ((s) & 0x3ffU)
#define MVEBU_RWTM_STS_VALUE(s) (((s) >> 10) & 0xfffffU)
#define MVEBU_RWTM_STS_ERROR(s) ((s) & (3U << 30))

/* WTMI firmware commands */
#define MVEBU_RWTM_CMD_GET_RANDOM 1
#define MVEBU_RWTM_CMD_BOARD_INFO 2
#define MVEBU_RWTM_CMD_REBOOT     9
#define MVEBU_RWTM_REBOOT_MAGIC   0xdeadbeefU

/*
 * Zephyr mbox channels: command MVEBU_RWTM_CMD_CHAN + channel, argument
 * 0 the length, arguments 1..15 the data.
 */
#define MVEBU_RWTM_CMD_CHAN 0x200
#define MVEBU_RWTM_CHANNELS 16
#define MVEBU_RWTM_MTU      ((MVEBU_RWTM_NARGS - 1) * 4)

/*
 * Secure processor side: a command below MVEBU_RWTM_CMD_CHAN. Called in
 * the interrupt; fills out[] and returns the reply status, which the
 * driver sends unless it is MVEBU_RWTM_STS_LATER. The default answers
 * BADCMD.
 */
uint32_t mvebu_rwtm_sp_legacy_cmd(uint16_t cmd, const uint32_t args[MVEBU_RWTM_NARGS],
				  uint32_t out[MVEBU_RWTM_NARGS]);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MBOX_MBOX_MVEBU_RWTM_H_ */
