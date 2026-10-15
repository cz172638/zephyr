.. zephyr:board:: espressobin

Overview
********

The Globalscale ESPRESSObin is a networking board built around the Marvell
Armada 3720: two Cortex-A53 cores, a Cortex-M3 (secure processor, used by
the boot firmware), a NETA gigabit Ethernet controller connected to a
Marvell 88E6341 switch with three front ports, USB 3.0, SATA, PCIe and
SDIO.

Zephyr runs on the A53 cores as the BL33 image of TF-A (``PLAT=a3700``):

=================================  ==================================
Target                             Description
=================================  ==================================
``espressobin``                    one A53 core
``espressobin/armada3720/smp``     both A53 cores, started through PSCI
=================================  ==================================

The board files were written for the v5 (DDR3, 1 GiB, two chip selects);
the v7 only differs in the DDR setup, which is done by the boot firmware.

Hardware
********

Supported features
==================

.. zephyr:board-supported-hw::

* UART0 (``uart-sum`` interrupt), 115200 baud on the micro-USB connector
* North/south bridge pin multiplexing and peripheral clock gates
* GICv3, ARM generic timer (25 MHz), PSCI
* Ethernet: NETA0 (RGMII-ID, fixed 1000/full) and the 88E6341 switch on the
  Orion SMI bus. The switch is configured as a port expander: ``wan``,
  ``lan0`` and ``lan1`` can only exchange frames with the CPU, there is no
  DSA tagging and Zephyr sees a single interface whose carrier is up while
  any of the three ports has link.

Memory map
==========

======================  ========  ===================================
Address                 Size      Use
======================  ========  ===================================
``0x00000000``          64 MiB    Zephyr (BL33 entry at the base)
``0x04000000``          16 MiB    TF-A BL31 and secure memory
``0xd0000000``          32 MiB    internal registers
======================  ========  ===================================

Only the first 64 MiB of DRAM are given to Zephyr; enlarge ``ddr0`` in the
board devicetree if needed, skipping the TF-A region.

Programming and Debugging
*************************

Boot chain
==========

The boot ROM loads a TIM image from SPI NOR. It contains the WTMI firmware
for the Cortex-M3 (DDR training via mv-ddr-marvell, clock setup) and TF-A
BL1. BL1 loads the FIP with BL2, BL31 and Zephyr as BL33. U-Boot is not
needed.

Sources:

* TF-A: https://git.trustedfirmware.org/TF-A/trusted-firmware-a.git
  (``contrib/mv-ddr`` submodule or https://github.com/MarvellEmbeddedProcessors/mv-ddr-marvell)
* A3700-utils-marvell: https://github.com/MarvellEmbeddedProcessors/A3700-utils-marvell
* Crypto++ (used by the TIM tools): https://github.com/weidai11/cryptopp
* Toolchains: ``aarch64-linux-gnu-`` for TF-A, ``arm-linux-gnueabi-`` for
  the WTMI firmware.

Building
========

.. zephyr-app-commands::
   :zephyr-app: samples/net/dhcpv4_client
   :board: espressobin/armada3720/smp
   :goals: build

Then build the flash and UART images with Zephyr as BL33:

.. code-block:: console

   TFA=~/tfa WTP=~/A3700-utils-marvell CRYPTOPP=~/cryptopp \
   CLOCKS=CPU_800_DDR_800 DDR_TOPOLOGY=2 \
       boards/globalscale/espressobin/support/mkflash.sh build/zephyr/zephyr.bin

``CLOCKS`` must match the marking on the SoC (``C080``: 800 MHz,
``C100``: 1 GHz), ``DDR_TOPOLOGY`` the memory population (``2`` for the v5
1 GiB board; check the ``mv_ddr`` banner of the stock firmware, e.g.
``DDR3 16b 1GB 2CS``).

Flashing
========

From an existing U-Boot (keep a copy of the original flash first):

.. code-block:: console

   => sf probe
   => sf read 0x1000000 0 0x400000      # backup, save over TFTP/USB
   => tftpboot 0x1000000 flash-image.bin
   => sf update 0x1000000 0 $filesize

Booting or recovering over UART needs no jumpers: ``mox-imager``
(https://gitlab.nic.cz/turris/mox-imager) sends the escape sequence that
puts the boot ROM into UART mode (``-E``) and then the image, and stays in
a terminal (``-t``, quit with Ctrl-\\ C). It takes the flash image as is:

.. code-block:: console

   mox-imager -D /dev/ttyUSB0 -E -b 6000000 -t flash-image.bin

or the separate files from ``uart-images.tgz.bin``:

.. code-block:: console

   tar xf uart-images.tgz.bin && cd uart-images
   mox-imager -D /dev/ttyUSB0 -E -b 6000000 -t TIM_ATF.bin wtmi_h.bin boot-image_h.bin

This is also the way to iterate on Zephyr without touching the SPI NOR.

Console
=======

The micro-USB connector carries UART0 through an FTDI bridge,
115200 8N1:

.. code-block:: console

   picocom -b 115200 /dev/ttyUSB0

Limitations
***********

* No DSA tagging yet: the three switch ports are one network.
* The board MAC address lives in the stock U-Boot environment only, so a
  random address is used (``zephyr,random-mac-address``); set
  ``local-mac-address`` in an overlay to use the one on the sticker.
* eth1 (SGMII) needs the COMPHY and is not used on this board.
* No drivers yet for SPI, I2C, GPIO, SDHCI, USB, SATA or PCIe.
