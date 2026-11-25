.. zephyr:code-sample:: sun55i-a523-pcie-shell
   :name: A523 PCIe root complex shell

   Bring up the A523 PCIe root complex and inspect the devices behind it.

Overview
********

The A523 PCIe controller (Synopsys DesignWare, root complex only, PCIe 2.1
x1) is brought up at boot: power domain, clocks, combo PHY, link training
and enumeration. On the Radxa Cubie A5E the link goes to the M.2 Key-M
slot. The ``pcie`` shell commands show the link state and the configuration
space of the devices found.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/allwinner/sun55i_a523/pcie_shell
   :board: cubie_a5e/sun55i_a523/smp
   :goals: build

Commands:

* ``pcie controller info``: link, LTSSM state, root port, windows
* ``pcie controller link retrain``: PERST#, link training and a new scan
* ``pcie controller link trace 2000``: LTSSM state changes for 2 s
* ``pcie ls``: enumerated devices
