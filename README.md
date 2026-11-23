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

## cubie-0006-uboot-spi-flash-nor.bin

**Image for programming the on-board SPI NOR (bytes, from offset 0).**
Same content as cubie-0005 (SPL + U-Boot FIT; sha256 `cbf7a43a...a9d5a`,
834049 bytes). Layout matches what U-Boot SPL expects when booting from NOR:
SPL at offset 0 (eGON.BT0, header length 0xC000), the FIT at 0xC000
(= `max(sunxi_get_spl_size(), CONFIG_SYS_SPI_U_BOOT_OFFS)`).

**VERIFIED ON HARDWARE — the board cold-boots from SPI NOR with this image.**
After programming, a full power cycle (no USB) gives on UART0:

```
U-Boot SPL ... Trying to boot from sunxi SPI
U-Boot ... SF: Detected w25q128fw with page size 256 Bytes,
           erase size 4 KiB, total 16 MiB
```

Recommended write procedure (ymodem via `loady` — no FEL DRAM addresses
involved at all):

```
sunxi-fel -v uboot fel/cubie-0005-uboot-spi-flash-fel.bin
# (stop autoboot)
loady 0x40080000        # send cubie-0006-uboot-spi-flash-nor.bin via ymodem
md 0x40080000 4         # must show "eGON" (4e4f4765 ...)
sf erase 0 +0x100000
sf write 0x40080000 0 0xcba01
sf read 0x40100000 0 0x40 && md 0x40100000 4   # eGON at offset 0
sf read 0x40101000 0xc000 0x20 && md 0x40101000 2   # d00dfeed at 0xC000
# full power cycle -> cold boot from NOR
```

**Known pitfall (bit us on the first attempt):** the `sf write` source
address is only valid if the very same `sunxi-fel` invocation also carries
the `write 0x40080000 <file>` payload; `sunxi-fel` never places the full
SPL+FIT file contiguously into DRAM on its own (SPL goes to SRAM, the FIT
components go to their load addresses). Writing NOR from an address that
was never loaded silently programs blank content (read back as all 0xFF),
and the BROM then falls through every boot source into FEL without ever
starting anything from SPI. Always verify with `sf read` + `md` (eGON at 0,
`d00dfeed` at 0xC000) before doing a cold-boot power cycle — this matches
what the BROM checks: it boots a plain eGON image from SPI NOR offset 0
(Andre Przywara's A523 SPI series, Armbian #11461 both confirm this).

Then do a full power cycle: if the A523 BROM picks SPI NOR as the boot
source, U-Boot starts off the flash on its own. If the BROM does not offer
SPI NOR boot on this SoC, the fallback chain stays SPL-via-FEL and the NOR
is used only as payload storage.

### Alternative: write the NOR directly via FEL (no U-Boot needed)

`sunxi-fel spiflash-read/write` programs the SPI NOR straight from the FEL
session, which removes the DRAM-address pitfall entirely. Upstream
sunxi-tools does not know the A523 yet; apply `fel/sunxi-tools-a523-spiflash.patch`
(A523: SPI0 @ 0x04025000, PIO @ 0x02000000, CCM @ 0x02001000, SPI0 BGR/CLK
= +0x96C/+0x940, pins PC2/PC3/PC4/PC12 = fn4, H6-style gate/reset bits) and
rebuild, then:

```
sunxi-fel ver                 # confirm genuine FEL mode first
sunxi-fel spiflash-info       # expect: Winbond W25Qxx (id ef40)
sunxi-fel spiflash-write 0 fel/cubie-0006-uboot-spi-flash-nor.bin
sunxi-fel spiflash-read 0 0x40 ver.bin && xxd ver.bin | head
```

## cubie-0007-zephyr-spi-flash-debug-fel.bin

**Zephyr marker-debug build (SPL + FIT with BL31 + Zephyr, like cubie-0004).**
SPI driver prints `MK` progress markers on UART0 (115200) from its
`POST_KERNEL` init, via `printk`:

```
MK0: spi0 init enter
MK1: map base phys=0x04025000 / base VA=...
MK2: map ccu       / ccu VA=...
MK3: map pio       / pio VA=...
MK4: ccu enable
MK5: pinmux / pinmux done
MK6: srst poll enter / (srst poll, timeout=... reg=...)
MK7: srst poll exit timeout=... reg=...
MK8: cs configure
MK9: spi0 init done
*** Booting Zephyr OS ... ***
cubie_a5e SPI flash smoke test
...
```

The last printed `MK` number localises the hang precisely:
- stops between MK0..MK3 → `device_map()` (`k_mem_map_phys_bare`) never
  returns — suspected deadlock in the fail path holding `z_mm_lock`
  (kernel/mmu.c fail path does not release the lock before `k_panic()`),
- stops at MK4 → CCU write (gate/clock) hangs the bus,
- never reaches MK7 → GCR soft-reset poll loop never sees SRST clear
  (controller not clocked / bus gate problem),
- reaches MK9 → init is fine and the fault is elsewhere (app or flash bus).

Run: `sunxi-fel -v uboot fel/cubie-0007-zephyr-spi-flash-debug-fel.bin`
(178583 bytes, sha256 `a0610b882c79831c...`).

## fel/sunxi-tools-a523-spiflash.patch

Patch for sunxi-tools adding A523 (soc_id 0x1890) support to the
`spiflash-info`/`spiflash-read`/`spiflash-write` FEL commands. Compile with
`make` in a sunxi-tools tree where the patch was applied with
`git apply`. Unverified on hardware as of this writing.
