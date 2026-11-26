# Copyright (c) 2026 Jiri Kastner <cz172638@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Load and run Zephyr on the ROCK Pi S (RK3308) over OpenOCD + GDB.
#
#   # terminal 1
#   OPENOCD_INTERFACE=cmsis-dap openocd \
#       -f boards/radxa/rock_pi_s/support/openocd.cfg
#   # terminal 2 (DRAM up: boot chain at BL31/U-Boot, "dcache off" if in U-Boot)
#   gdb-multiarch build/zephyr/zephyr.elf \
#       -x boards/radxa/rock_pi_s/support/rock_pi_s.gdb
#   (gdb) zephyr_connect
#   (gdb) zephyr_run
#
# Zephyr is linked at CONFIG_SRAM_BASE_ADDRESS (0x02000000); __start (reset.S)
# sits at that base, which is also the image entry.

define zephyr_connect
	target extended-remote :3333
	monitor targets rk3308.a35.0
	monitor halt
end
document zephyr_connect
Attach to OpenOCD (gdb port 3333), select core 0 and halt it.
end

define zephyr_load
	monitor halt
	load
	set $pc = &__start
	echo zephyr: loaded, pc at __start\n
end
document zephyr_load
Write the ELF LOAD segments to DRAM and point the PC at the reset vector.
Caches must be off (APB-AP writes bypass them); run from a halted boot chain.
end

define zephyr_run
	zephyr_load
	continue
end
document zephyr_run
zephyr_load, then resume. Console is UART0 at 1500000 8N1.
end
