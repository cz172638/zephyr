.. zephyr:code-sample:: a523-pcie-probe
   :name: A523 PCIe controller probe

   Bring up the PCIe controller of the Allwinner A523 step by step and
   check whether it can run as an endpoint.

Overview
********

The A523 user manual documents the PCIe controller as root complex only,
while the vendor BSP switches it to endpoint mode through bits [7:4] of
``PCIE_LTSSM_CTRL`` (``0x04c00c00``). The ``a523pcie`` shell commands power
up the PCIe domain (PCK-600), enable clocks and resets, set up the combo
PHY, select the role and dump what the controller reports, so that the
role switch can be checked without a driver.

Requirements
************

A board with the A523/A527/T527, such as the Radxa Cubie A5E (the M.2
slot is reached through the GMA340 switch, ``a523pcie gma pcie``).

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/allwinner/sun55i_a523/pcie_probe
   :board: cubie_a5e/sun55i_a523/smp
   :goals: build

.. code-block:: console

   uart:~$ a523pcie seq ep
   uart:~$ a523pcie dump
