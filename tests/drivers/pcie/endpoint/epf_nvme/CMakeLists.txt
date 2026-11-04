# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pcie_epf_nvme)

target_sources(app PRIVATE src/main.c src/fake_epc.c)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/drivers/pcie/endpoint/functions)
