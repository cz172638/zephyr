# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rk3399_pcie_shell)
target_sources_ifdef(CONFIG_SMP app PRIVATE src/ep_affinity.c)

target_sources(app PRIVATE src/main.c src/cpuinfo.c)
