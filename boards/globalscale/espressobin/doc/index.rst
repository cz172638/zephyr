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
* North/south bridge pin multiplexing, peripheral clock gates and rates
  (TBG, muxes and dividers are read back as set by the boot firmware)
* GICv3, ARM generic timer (25 MHz), PSCI
* Ethernet: NETA0 (RGMII-ID, fixed 1000/full) and the 88E6341 switch on the
  Orion SMI bus. By default the switch is a port expander: ``wan``,
  ``lan0`` and ``lan1`` can only exchange frames with the CPU, there is no
  tagging and Zephyr sees a single interface whose carrier is up while any
  of the three ports has link. With the ``espressobin-dsa`` snippet the CPU
  port uses the Marvell DSA tag and each port is its own interface, see
  `DSA`_.
* Ethernet queues: up to 8 TX queues in strict priority order, selected by
  packet priority, each with an optional rate limit exposed as 802.1Qav
  (``ETHERNET_QAV``); up to 8 RX queues selected by the VLAN PCP. See
  `Priority queues and shaping`_.

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

Updating from Zephyr
====================

With ``CONFIG_FLASH=y``, ``CONFIG_FLASH_MAP=y`` and ``CONFIG_FWUPDATE=y`` the
shell can replace the image in the SPI NOR ``firmware`` partition, without
U-Boot. The image is the ``flash-image.bin`` from ``mkflash.sh``; it is
received completely into RAM, must carry the TIM header, and is then erased,
written (header sector last) and read back:

.. code-block:: console

   uart:~$ fwupdate ymodem
   (tio: ctrl-t y, flash-image.bin)
   received "flash-image.bin", 1001448 bytes, crc32 ...
   firmware updated, reboot to start it

``fwupdate ymodem dry`` only receives and checks. At 115200 baud a 1 MiB
image takes about a minute and a half. If power is lost while writing, the
boot ROM finds no header and falls back to UART boot (``mox-imager``).

Booting from U-Boot
===================

Zephyr carries an AArch64 ``Image`` header (``text_offset`` 0), so an
existing U-Boot (stock Marvell or mainline as BL33 of the same TF-A) can
start ``zephyr.bin`` with ``booti``; it moves the image to the start of
DRAM. The devicetree argument is ignored.

.. code-block:: console

   => load mmc 0:1 $kernel_addr_r boot/Image
   => booti $kernel_addr_r - $fdtcontroladdr

or over the network:

.. code-block:: console

   => dhcp $kernel_addr_r zephyr.bin && booti $kernel_addr_r - $fdtcontroladdr

The SPI NOR ``zephyr-storage`` partition (``0x3e0000``, 64 KiB) holds the
Zephyr settings, apart from the U-Boot environment at ``0x3f0000``.

Console
=======

The micro-USB connector carries UART0 through an FTDI bridge,
115200 8N1:

.. code-block:: console

   picocom -b 115200 /dev/ttyUSB0

Priority queues and shaping
===========================

The number of queues follows the traffic classes of the stack:

.. code-block:: cfg

   CONFIG_NET_TC_TX_COUNT=8     # -> CONFIG_ETH_MVNETA_TX_QUEUES=8
   CONFIG_NET_TC_RX_COUNT=4     # -> CONFIG_ETH_MVNETA_RX_QUEUES=4
   CONFIG_NET_VLAN=y
   CONFIG_NET_L2_ETHERNET_MGMT=y
   CONFIG_ETH_MVNETA_SHELL=y    # "mvneta" shell commands

Each TX queue has a token bucket with 10 kb/s resolution. Setting the Qav
idle slope (``NET_REQUEST_ETHERNET_SET_QAV_PARAM``) limits that queue to
the reserved rate with one frame of burst; this is a rate limiter, not a
credit-based shaper. With 8 traffic classes, queue N carries class N.
With the default ``CONFIG_NET_TC_MAPPING_STRICT`` best effort is queue 1
and priority CA (PCP 3) queue 3; for AVB use
``CONFIG_NET_TC_MAPPING_SR_CLASS_A_AND_B``, which puts SR class A (PCP 3)
into the highest queue 7 and class B (PCP 2) into queue 6. From the shell,
a 20 Mb/s class A reservation on interface 1 with the SR mapping:

.. code-block:: console

   uart:~$ mvneta qav 1 7 20000000
   uart:~$ mvneta queues 1
   uart:~$ net iface 1

Which end of the hardware queues the arbiter serves first is not
documented publicly. The driver assumes queue 7; if a test with two
saturating streams shows the opposite, set
``CONFIG_ETH_MVNETA_TXQ0_HIGHEST=y``.

DSA
===

.. code-block:: console

   west build -b espressobin/armada3720/smp -S espressobin-dsa samples/net/dhcpv4_client

``eth0`` becomes the DSA conduit (promiscuous, no IP), the CPU port and the
front ports get their own interfaces, ``swp0`` (CPU port, no IP) and
``swp1`` to ``swp3`` for switch ports 1 to 3. The front labels differ
between board versions: v5 port 1 ``wan``, 2 ``lan0``, 3 ``lan1``; v7 port 1
``lan1``, 2 ``lan0``, 3 ``wan``.

The user port PHYs are handled by the generic PHY driver on the switch's
internal MDIO bus (``marvell,mv88e6xxx-mdio``), the tags by
``dsa_tag_mv88e6xxx.c``. The snippet uses the 4 byte DSA tag; for EDSA set
``dsa-tag-protocol = <DSA_TAG_PROTO_EDSA>`` on the CPU port and
``CONFIG_DSA_TAG_SIZE=8``.

PTP
===

With the ``espressobin-dsa`` snippet and a PTP clock enabled (any gPTP
configuration), the 88E6341 time stamps PTP event messages on the front
ports. ``marvell,mv88e6xxx-ptp`` is the clock: a software time counter on
the free running 32 bit TAI, as in Linux, so stepping and rate changes do
not touch the switch. Arrival time stamps are read from the port when the
tagger sees an event message, departure time stamps are polled after
sending (one frame in flight per port). On the 88E6341 time stamps are
taken at the MAC; Linux does the same, the PHY time stamps are unreliable.

.. code-block:: console

   west build -b espressobin/armada3720/smp -S espressobin-dsa \
      samples/boards/globalscale/espressobin/netshell -- -DEXTRA_CONF_FILE=gptp.conf

``net gptp`` shows the port states, ``ptp_clock`` the clock.
``samples/net/ethernet/tsn-switch`` also builds with the snippet (software
bridge over the ports, gPTP relay; its Qbv setup is not supported).

Qav on the switch ports
=======================

The ``swpN`` interfaces have four egress queues with the switch's credit
based shapers (802.1Qav), set per queue through the Ethernet management
Qav requests, e.g. with the netshell's ``mvneta qav``, which works on any
interface:

.. code-block:: console

   uart:~$ net iface                    # interface index of swpN
   uart:~$ mvneta qav 4 3 20000000      # interface 4, queue 3, 20 Mbit/s
   uart:~$ mvneta queues 4
   uart:~$ mv88e6xxx qav                # shaper registers of all ports

Queue N is traffic class N: the tagger puts the packet priority into the
DSA tag and the switch's IEEE-PRI table maps it like the Zephyr priority to
traffic class mapping for four classes. With the default strict mapping SR
class A and B (priority 3 and 2) share queue 1; for one queue per class set
``CONFIG_NET_TC_MAPPING_SR_CLASS_A_AND_B=y`` (needs ``CONFIG_NET_TC_TX_COUNT``
and ``CONFIG_NET_TC_RX_COUNT`` of at least 2): class A goes to queue 3,
class B to queue 2. A shaped port switches to strict priority between its
queues. The hi limit comes from the idle slope and the link speed (one
maximum frame at port rate), the rate register is in 64 kbit/s steps.

The register use follows the Linux CBS offload patches (Cedric Jehasse,
Luke Howard), there is no public documentation. ``mv88e6xxx avb`` reads
and writes any register of the PTP, AVB, Qav and Qvb blocks.

Limitations
***********

* DSA ports are standalone (no bridging in the switch); bridging is done
  in software (``CONFIG_NET_ETHERNET_BRIDGE``).
* The switch PTP clock is a software time counter: no PPS output or
  event capture (TAI GPIOs) yet.
* Qav on the switch ports shapes per queue only: no AVB port modes or
  stream reservation (ATU AVB entries), no Qbv.
* The Armada 3700 NETA has a single, shared interrupt: no RSS, all RX is
  handled by one thread.
* The board MAC address lives in the U-Boot environment only, so a
  random address is used (``zephyr,random-mac-address``); set
  ``local-mac-address`` in an overlay to use the one on the sticker.
* eth1 (SGMII) needs the COMPHY and is not used on this board.
* No drivers yet for SPI, I2C, GPIO, SDHCI, USB, SATA or PCIe.
