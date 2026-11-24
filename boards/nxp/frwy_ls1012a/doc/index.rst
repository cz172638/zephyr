.. zephyr:board:: frwy_ls1012a

Overview
********

The FRWY-LS1012A (Freeway) board carries the NXP Layerscape LS1012A: one
Arm Cortex-A53 core, 1 GiB DDR3L, a W25M161AW QSPI device (16 Mbit NOR and
1 Gbit NAND), two SGMII Ethernet ports, one M.2 Type-E slot with PCIe Gen2 x1
on SerDes lane D and a Kinetis K20 that provides CMSIS-DAP JTAG and a USB
serial port for the console (DUART0).

Hardware
********

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Zephyr runs from DDR at ``0x82000000``, the BL33 address of the NXP TF-A
(``nxp-qoriq/atf``, ``PLAT=ls1012afrwy``), so it can be the BL33 image in the
FIP that BL2 loads from QSPI. With U-Boot in QSPI instead, start it from the
prompt, for example after a YMODEM transfer over the console:

.. code-block:: console

   => loady 0x82000000
   => dcache flush; dcache off; icache flush; icache off; go 0x82000000

Build the :zephyr:code-sample:`hello_world` sample:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: frwy_ls1012a
   :goals: build

OpenOCD reaches the core over the on-board CMSIS-DAP with
``openocd -f board/nxp_frdm-ls1012a.cfg``.

References
**********

- `FRWY-LS1012A Board Reference Manual <https://www.nxp.com/docs/en/reference-manual/FRWY-LS1012ARM.pdf>`_
