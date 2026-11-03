# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(ahci.c)
zephyr_library_sources_ifdef(CONFIG_AHCI_SHELL ahci_shell.c)
