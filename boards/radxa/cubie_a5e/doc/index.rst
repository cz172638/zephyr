.. zephyr:board:: cubie_a5e

Overview
********

The Radxa Cubie A5E is built around the Allwinner A527: two clusters of
four Cortex-A55 behind a DSU, a GIC-600, 1, 2 or 4 GiB of LPDDR4.

Zephyr runs as BL33 of TF-A (``PLAT=sun55i_a523``), which U-Boot SPL loads
together with Zephyr from a FIT image. BL31 enters Zephyr at ``0x4a000000``
in EL2. U-Boot SPL sets up DRAM, clocks and the pin muxing of the console.

Supported features
==================

.. zephyr:board-supported-hw::

The ``cubie_a5e/sun55i_a523/smp`` variant brings up all eight cores
through PSCI.

Programming and Debugging
*************************

Console
=======

UART0 on pins 8 (TX) and 10 (RX) of the 40-pin header, 115200 8N1.

Flashing
========

Build U-Boot (``radxa-cubie-a5e_defconfig``) with ``BL31`` pointing to the
TF-A image, replace the ``uboot`` image of its FIT with ``zephyr.bin``
(load address ``0x4a000000``) and write SPL and FIT to the SD card at
8 KiB:

.. code-block:: console

   dd if=image.bin of=/dev/sdX bs=1k seek=8 conv=fsync

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: cubie_a5e/sun55i_a523/smp
   :goals: build
