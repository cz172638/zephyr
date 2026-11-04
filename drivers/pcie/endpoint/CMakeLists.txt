# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(pcie_ep_common.c)

zephyr_library_sources_ifdef(
  CONFIG_PCIE_EP_IPROC
  pcie_ep_iproc.c
  pcie_ep_iproc_msi.c
  )

zephyr_library_sources_ifdef(CONFIG_PCIE_EPC pcie_epc_core.c pcie_epf_select.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_EPC_DSN_SETTINGS pcie_epc_dsn.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_EPC_AARDVARK
  pcie_epc_aardvark.c
  pcie_epc_aardvark_a3700.c
  pcie_epc_aardvark_a3700_xor.c
  )
add_subdirectory_ifdef(CONFIG_PCIE_EPC functions)
