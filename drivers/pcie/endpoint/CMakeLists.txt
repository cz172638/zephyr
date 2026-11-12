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
zephyr_library_sources_ifdef(CONFIG_PCIE_EPC_AARDVARK_IRQTEST
  pcie_epc_aardvark_a3700_irqtest.c
  )
zephyr_library_sources_ifdef(CONFIG_PCIE_EPC_AARDVARK_XORTEST
  pcie_epc_aardvark_a3700_xortest.c
  )
add_subdirectory_ifdef(CONFIG_PCIE_EPC functions)
zephyr_library_sources_ifdef(CONFIG_PCIE_EP_CDNS pcie_ep_cdns.c)
zephyr_library_sources_ifdef(CONFIG_PCIE_EPC_RK3399 pcie_epc_rk3399.c)
