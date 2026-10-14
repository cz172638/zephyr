.. zephyr:board:: rock_pi_s

Overview
********

The Radxa ROCK Pi S is a small board built around the Rockchip RK3308
(4x Cortex-A35, GIC-400) with 256 or 512 MiB of DDR3.

Zephyr runs in the non-secure world as BL33, started straight from TF-A BL31
with nothing else in the chain::

  BootROM -> U-Boot TPL (DRAM init) -> U-Boot SPL -> TF-A BL31 -> Zephyr (BL33)

There is no OP-TEE and no U-Boot proper. It is entered at EL2 (or EL1) and runs
at EL1. Secondary cores are started with PSCI ``CPU_ON`` through TF-A.

Supported Features
==================

.. zephyr:board-supported-hw::

Only the generic timer, GIC-400 and DW APB UARTs (ns16550) are supported.
The UART clock is assumed to be 24 MHz as left by the boot loader; there is
no clock/pinctrl driver yet.

Memory
======

The image is linked at ``0x02000000`` (16 MiB), clear of BL31's secure
carve-out at ``0x00000000``-``0x000fffff``.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Build::

  west build -b rock_pi_s samples/hello_world
  west build -b rock_pi_s/rk3308/smp samples/synchronization

Booting straight from BL31
==========================

Zephyr is the BL33 payload: the build wraps ``zephyr.bin`` together with BL31
into ``zephyr.itb``, a FIT image laid out like U-Boot's ``u-boot.itb``, so the
U-Boot SPL loads it unchanged and TF-A jumps straight to Zephyr, with the MMU
and caches off. No OP-TEE and no U-Boot proper are built in.

Inputs, as environment or CMake variables:

``BL31``
  TF-A ``bl31.elf`` (required, split into one image per loadable segment)
``MKIMAGE``
  ``mkimage`` binary (default: ``UBOOT_DIR/tools/mkimage``, then ``PATH``)
``UBOOT_DIR``
  U-Boot build directory, only used to locate ``mkimage`` (optional)
``BL31_FDT``
  DTB image for BL33, only if your SPL build requires an fdt image (optional)

Without ``BL31`` no ``zephyr.itb`` is made. ::

  export BL31=.../trusted-firmware-a/build/rk3308/release/bl31/bl31.elf
  west build -b rock_pi_s samples/hello_world

SPL finds the BL33 entry through the image with ``os = "u-boot"``, which is why
the Zephyr image is marked that way. No BL33 DTB is emitted: the Rockchip BL31
does not read its console from one and Zephyr carries its own devicetree. If a
particular SPL build insists on an fdt image in the config, pass ``BL31_FDT``
(any valid dtb, e.g. the built ``zephyr.dtb`` -- Zephyr ignores the runtime
dtb).

Write it where SPL looks for ``u-boot.itb`` (sector 16384 = 8 MiB, after
``idbloader.img`` at sector 64)::

  dd if=build/zephyr/zephyr.itb of=/dev/sdX seek=16384 conv=fsync

For a maskrom USB boot, use the U-Boot USB image with its ``u-boot.itb``
replaced by ``zephyr.itb``.

The SPL keeps its stack, heap and BSS at ``0x03c00000``-``0x03ffffff`` and
reads the FIT header below ``0x00800000``, clear of Zephyr at ``0x02000000``.

Quick load for development
==========================

Any loader halted with caches off can drop the raw binary in and jump to it,
e.g. from a U-Boot prompt kept only for bring-up::

  => load mmc 0:1 0x02000000 zephyr.bin
  => dcache flush; dcache off
  => go 0x02000000

or ``bootelf`` with ``zephyr.elf``. Do not use ``booti``: the image is not
relocatable and U-Boot would move it to the start of its first DRAM bank.

Debugging
=========

Over SWD (the SD card pins must be muxed to JTAG; without U-Boot proper that
is up to the debugger scripts, and caches must be off while loading)::

  OPENOCD_INTERFACE=cmsis-dap west debug

Serial console: UART0, 1500000 8N1.
