# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
# the fake endpoint controller of the vNTB test
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../endpoint/epf_vntb)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pcie_fabric_epf)

target_include_directories(app PRIVATE ../../endpoint/epf_vntb/src)
target_sources(app PRIVATE src/main.c ../../endpoint/epf_vntb/src/fake_epc.c)
