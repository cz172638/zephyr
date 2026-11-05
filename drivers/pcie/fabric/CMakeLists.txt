# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(pfab_core.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_FABRIC_MGR pfab_mgr.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_FABRIC_SHELL pfab_shell.c)
