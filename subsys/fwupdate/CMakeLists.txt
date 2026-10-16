# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_FWUPDATE_YMODEM ymodem.c)
if(CONFIG_FWUPDATE_YMODEM OR CONFIG_FWUPDATE_TCP)
  zephyr_library_sources(fwupdate_shell.c)
endif()
