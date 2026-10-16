# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_FWUPDATE_YMODEM ymodem.c fwupdate_shell.c)
