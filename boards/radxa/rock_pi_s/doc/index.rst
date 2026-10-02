.. zephyr:board:: rock_pi_s

Overview
********

The Radxa ROCK Pi S is a small board built around the Rockchip RK3308
(4x Cortex-A35, GIC-400) with 256 or 512 MiB of DDR3.

Zephyr runs in the non-secure world as BL33 of the standard Rockchip boot
chain::

  BootROM -> U-Boot TPL (DRAM init) -> U-Boot SPL -> TF-A BL31
          [-> OP-TEE BL32] -> Zephyr (BL33) or U-Boot proper -> Zephyr

It is entered at EL2 (or EL1) and runs at EL1. Secondary cores are started
with PSCI ``CPU_ON`` through TF-A.

Supported Features
==================

.. zephyr:board-supported-hw::

Only the generic timer, GIC-400 and DW APB UARTs (ns16550) are supported.
The UART clock is assumed to be 24 MHz as left by the boot loader; there is
no clock/pinctrl driver yet.

Memory
======

The image is linked at ``0x02000000`` (16 MiB). Keep it clear of BL31, U-Boot
and the OP-TEE TZDRAM/SHMEM carve-outs.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Build::

  west build -b rock_pi_s samples/hello_world
  west build -b rock_pi_s/rk3308/smp samples/synchronization

From U-Boot proper::

  => load mmc 0:1 0x02000000 zephyr.bin
  => dcache flush; dcache off
  => go 0x02000000

or ``bootelf`` with ``zephyr.elf``. Do not use ``booti``: the image is not
relocatable and U-Boot would move it to the start of its first DRAM bank.

Over SWD (U-Boot must have muxed the SD card pins to JTAG, D-cache off)::

  OPENOCD_INTERFACE=cmsis-dap west debug

Serial console: UART0, 1500000 8N1.
