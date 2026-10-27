.. zephyr:board:: rock_pi_4

Overview
********

The Radxa ROCK Pi 4 is a single board computer built around the Rockchip
RK3399: two Cortex-A72 cores (big cluster), four Cortex-A53 cores (little
cluster) and two Cortex-M0 cores, one in the PMU power domain and one in the
PERILP power domain.

Every core type is a separate Zephyr target:

======================================= =======================================
Target                                  Cores
======================================= =======================================
``rock_pi_4/rk3399/a53``                CPU0 (Cortex-A53)
``rock_pi_4/rk3399/a53/smp``            CPU0-CPU3 (Cortex-A53)
``rock_pi_4/rk3399/a72``                CPU4 (Cortex-A72)
``rock_pi_4/rk3399/a72/smp``            CPU4-CPU5 (Cortex-A72)
``rock_pi_4/rk3399/m0_pmu``             PMU Cortex-M0
``rock_pi_4/rk3399/m0_perilp``          PERILP Cortex-M0
======================================= =======================================

Hardware
********

See https://docs.radxa.com/en/rock4/rock4b

Supported Features
==================

.. zephyr:board-supported-hw::

Memory map
==========

The images of an AMP setup use separate DRAM areas:

=========================== =====================================================
Address range               Use
=========================== =====================================================
0x00000000 - 0x001fffff     TF-A
0x00200000 - 0x03ffffff     little cluster image (BL33 entry point)
0x04000000 - 0x05ffffff     big cluster image
0x06000000 - 0x060fffff     PMU M0 image (M0 address 0x0)
0x06100000 - 0x061fffff     PERILP M0 image (M0 address 0x0)
0x06200000 - 0x063fffff     shared memory, non-cacheable
0x08000000 - 0x0bffffff     memory exposed through PCIe endpoint BARs
=========================== =====================================================

Booting
*******

The boot ROM, the SPL, TF-A and U-Boot run on CPU0. A little cluster image
is linked at 0x00200000 and can replace U-Boot as BL33 in the SPL FIT image,
so TF-A enters Zephyr directly.

A big cluster image entered on an A53 asks PSCI to start CPU4 at its entry
point and powers the A53 off (:kconfig:option:`CONFIG_RK3399_A72_SELF_MIGRATE`),
so it can be started the same way as a little cluster image.

The Cortex-M0 execute address is remapped through the SGRF, which is only
writable from the secure world. Starting an M0 image therefore needs the
``RK_SIP_M0_CTRL`` SiP call in TF-A; see :zephyr:code-sample:`rk3399-amp`.
While the non-secure world owns the PMU M0, TF-A cannot run DDR frequency
scaling or system suspend on it.

Serial console
==============

UART2 at 1500000 8N1 on the 40-pin header (pins 8 and 10) is the console of
all targets. The M0 targets use it in polled mode.

PCIe endpoint
=============

The RK3399 PCIe controller can run as an endpoint on the M.2 M-key slot. The
host drives PERST# on GPIO4_D3, the slot 3.3 V supply must stay off and, as
the RK3399 PHY has no reference clock input, the host must accept a separate
reference clock. See :zephyr:code-sample:`rk3399-pcie-ep-test`.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Load ``zephyr.bin`` at the start of the target's DRAM area and jump to it, for
example from U-Boot:

.. code-block:: console

   tftp 0x00200000 zephyr.bin; go 0x00200000
