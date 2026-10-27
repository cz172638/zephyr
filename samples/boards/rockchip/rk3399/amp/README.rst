.. zephyr:code-sample:: rk3399-amp
   :name: RK3399 AMP launcher

   Start the big cluster and both Cortex-M0 cores from the little cluster.

Overview
********

The sample runs on CPU0 and starts the images a previous boot stage placed
in DRAM: the big cluster image through PSCI ``CPU_ON``, the PMU and PERILP
Cortex-M0 images through the TF-A ``RK_SIP_M0_CTRL`` SiP call. Addresses are
set by ``CONFIG_AMP_*`` and default to the ROCK Pi 4 memory map.

Requirements
************

TF-A with ``RK_SIP_M0_CTRL`` (plat/rockchip/rk3399, ``drivers/pmu/m0_ns.c``).
Without it the M0 starts fail with ``-ENOTSUP``.

Building and Running
********************

.. code-block:: console

   west build -b rock_pi_4/rk3399/a53 samples/boards/rockchip/rk3399/amp -d build/amp
   west build -b rock_pi_4/rk3399/a72 samples/hello_world -d build/a72
   west build -b rock_pi_4/rk3399/m0_pmu samples/hello_world -d build/m0_pmu
   west build -b rock_pi_4/rk3399/m0_perilp samples/hello_world -d build/m0_perilp

Load and run from U-Boot:

.. code-block:: console

   tftp 0x04000000 a72.bin
   tftp 0x06000000 m0_pmu.bin
   tftp 0x06100000 m0_perilp.bin
   tftp 0x00200000 amp.bin
   go 0x00200000
