Clock preset, AVS and DDR tuning of the Marvell WTMI firmware for the
Armada 3720 Cortex-M3, from A3700-utils-marvell
(https://github.com/MarvellEmbeddedProcessors/A3700-utils-marvell,
commit a3e1c67bb378e1d8a938e1b826cb602af83628d2, wtmi/sys_init and
wtmi/common), BSD 3-clause license (see the file headers).

Unchanged: clock.[ch], avs.[ch], ddr/*, bits.h, regs.h, io.h, delay.[ch].
Replaced for Zephyr: sys.h, types.h (bool from stdbool.h), autoconf.h
(values from Kconfig). sys_init.c is adapted from main.c and apctl.c.
