# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_PCIE_EPF_TEST pcie_epf_test.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_EPF_NVME pcie_epf_nvme.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_EPF_VNTB pcie_epf_vntb.c)
