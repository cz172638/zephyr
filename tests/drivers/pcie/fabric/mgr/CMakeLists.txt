# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pcie_fabric_mgr)

# manager and tree walk against an emulated controller (no controller in DT)
target_sources(app PRIVATE src/main.c ${ZEPHYR_BASE}/drivers/pcie/fabric/pfab_mgr.c
	       ${ZEPHYR_BASE}/drivers/pcie/pcie_aer.c)
target_compile_definitions(app PRIVATE CONFIG_PCIE_AER_SOURCES=8 CONFIG_PCIE_FABRIC_RC_NODE=1
			   CONFIG_PCIE_FABRIC_RC_REGION_SIZE=0x4000)
