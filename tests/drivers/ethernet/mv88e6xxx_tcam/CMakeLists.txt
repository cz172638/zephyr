# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mv88e6xxx_tcam)

target_sources(app PRIVATE src/main.c
	${ZEPHYR_BASE}/drivers/ethernet/phy/phy_marvell_mv88e6xxx_tcam.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/ethernet/phy)
