/*
 * Copyright (c) 2026 Jiri Kastner
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PCIE_PCIE_SHELL_PRIV_H_
#define ZEPHYR_DRIVERS_PCIE_PCIE_SHELL_PRIV_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/pcie/pcie_shell.h>
#include <zephyr/shell/shell.h>

/* backend of the controller in use this boot; prints an error when none */
const struct pcie_shell_ctrl *pcie_shell_active(const struct shell *sh);

/* "aaaa:bbbb" (hex) */
int pcie_shell_parse_pair(const char *s, uint16_t *a, uint16_t *b);

int pcie_shell_cmd_boot(const struct shell *sh, size_t argc, char **argv);
int pcie_shell_cmd_epftest(const struct shell *sh, size_t argc, char **argv);

#endif /* ZEPHYR_DRIVERS_PCIE_PCIE_SHELL_PRIV_H_ */
