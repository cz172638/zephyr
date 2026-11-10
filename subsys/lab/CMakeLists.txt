# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(lab_agent.c lab_client.c lab_shell.c)
zephyr_library_sources_ifdef(CONFIG_LAB_EXT lab_llext.c)
