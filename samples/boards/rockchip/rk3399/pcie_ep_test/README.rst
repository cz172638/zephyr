.. zephyr:code-sample:: rk3399-pcie-ep-test
   :name: RK3399 PCIe endpoint test function

   Exercise the RK3399 PCIe endpoint with the Linux pci_endpoint_test driver.

Overview
********

The sample configures the RK3399 PCIe controller as an endpoint and serves
the register protocol of the Linux ``pci-epf-test`` function on BAR0, so a
Linux host can run the ``pci_endpoint_test`` driver and the ``pcitest`` tool
against it. BAR0-BAR4 are 128 KiB and BAR5 is 1 MiB, all backed by the
``pcie_ep_mem`` DRAM region. INTx and MSI are supported; DMA and MSI-X are
not.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/rockchip/rk3399/pcie_ep_test
   :board: rock_pi_4/rk3399/a53
   :goals: build

On the host, the endpoint enumerates as 1d87:3399:

.. code-block:: console

   echo 1d87 3399 > /sys/bus/pci/drivers/pci-endpoint-test/new_id
   pcitest -B
   pcitest -i 1 -m 1
   pcitest -r -s 65536
   pcitest -w -s 65536
