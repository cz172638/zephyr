/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FLASH_SPI_NOR_EX_H_
#define ZEPHYR_INCLUDE_DRIVERS_FLASH_SPI_NOR_EX_H_

#include <zephyr/drivers/flash.h>

/**
 * @brief Read the factory unique ID of a SPI NOR (command 0x4B, 4 dummy
 * bytes, Winbond and compatible). out: uint8_t[8], first byte read first.
 */
#define FLASH_EX_OP_SPI_NOR_READ_UID (FLASH_EX_OP_VENDOR_BASE + 0x4b)

#define SPI_NOR_UID_LEN 8

#endif
