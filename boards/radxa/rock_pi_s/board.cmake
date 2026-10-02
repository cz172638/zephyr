# Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
# SPDX-License-Identifier: Apache-2.0

# Debug/load through SWD (sdmmc0 pins muxed as JTAG by the boot loader)
board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd.cfg")
board_runner_args(openocd "--use-elf")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
