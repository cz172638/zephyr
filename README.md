# FEL artefacts for the Radxa Cubie A5E (Allwinner A527)

Naming convention: `cubie-NNNN-<description>.bin` — `NNNN` is a monotonically
increasing build number.

All images are used the same way (board in FEL mode, UART0 = 40-pin pins 8/10
@ 115200):

```
sunxi-fel -v uboot fel/<file>.bin
```

Each file is a single FEL loadable: an "eGON.BT0"-wrapped U-Boot SPL (built
from upstream U-Boot `radxa-cubie-a5e_defconfig`, does DRAM init and returns
to FEL) followed by a FIT image; `sunxi-fel` writes the FIT components to
their load addresses and starts the entry point via an AArch64 RMR warm
reset.

## cubie-0001-zephyr-spi-flash-fel-atf.bin

**The SPI-flash smoke test, recommended image.**

SPL + FIT with `atf` (TF-A BL31, `PLAT=sun55i_a523`, from
cz172638/arm-trusted-firmware branch `sun55i-a523`, entry 0x54000) as the
@firmware and Zephyr (printk + shell build from the `claude/a527-zephyr`
line, load 0x4a000000) as the @loadable. Same chain as U-Boot uses
(@SPL → @BL31 → @BL33).

Expected UART0 output:

```
*** Booting Zephyr OS build ... ***
cubie_a5e SPI flash smoke test
flash size: 16777216 bytes (rc=0)
read(0,3): rc=0 jedec id = ef 40 18
read(0,16): rc=0
data: <16 bytes of the flash start>
smoke test done
uart:~$
```

The Zephyr shell prompt (`uart:~$`) is interactive.

## cubie-0002-uboot-shell-fel-selftest.bin

Differential boot-chain self-test: same SPL + FIT/RMR flow, but upstream
U-Boot proper (no TF-A, upstream handles EL3 entry itself) as the payload.
Verified working on hardware — proves the image layout, DRAM init and warm
reset entry are correct.

## cubie-0004-zephyr-spi-flash-fel-atf.bin

**Current smoke test build (recommended).** Same chain as cubie-0001, with
the SPI driver fixed to map its controller, CCU and PIO MMIO windows with
`device_map()` before use (boot via BL31 runs Zephyr with the MMU enabled;
the previous build took a data abort on the first CCU register access,
`FAR 0x0200196c`).

## cubie-0003-zephyr-spi-flash-fel-noatf.bin

Diagnostic variant: SPL + FIT with Zephyr only as the payload (no BL31,
Zephyr entered directly at EL3 via RMR). Silent on hardware — Zephyr on this
board expects the TF-A BL31 setup; use cubie-0001.

## cubie-0005-uboot-spi-flash-fel.bin

U-Boot proper (same chain as cubie-0002) rebuilt with the on-board SPI NOR
reachable from the U-Boot command line: `CONFIG_SPI_SUNXI` (`spi-sunxi` DM
driver with the `allwinner,sun55i-a523-spi` variant) and `&spi0` enabled in
the board DT (pins PC2/PC3/PC4/PC12, `flash@0 jedec,spi-nor`, 24 MHz from
OSC24M).

```
sf probe            # expect: SF: Detected w25q128 ...
sf erase <off> +<size>
sf write <dram_addr> <off> <size>
```

Extra payload for writing can be pushed through FEL along with the boot:

```
sunxi-fel -v uboot fel/cubie-0005-uboot-spi-flash-fel.bin \
    write 0x40080000 <some-file.bin>
```
