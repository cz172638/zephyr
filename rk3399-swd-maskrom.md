# RK3399 (ROCK Pi 4): USB maskrom boot do RAM, SWD přes SD slot, debug soubory

Stav k 16. 11. 2026, vše ověřené na desce. Nic se nezapisuje do SPI, na eMMC ani na SD a karty není potřeba přendavat.

## Maskrom: boot do RAM přes USB

```
rkdeveloptool db <img>-rock_pi_4-loader.bin
# nebo
xrock maskrom <img>-rock_pi_4-usb471.bin <img>-rock_pi_4-usb472.bin --rc4 on
```

| soubor | obsah |
|---|---|
| `-usb471.bin` | U-Boot TPL (DDR init) |
| `-usb472.bin` | SPL z buildu s `ROCKCHIP_MASKROM_IMAGE` a `SPL_LOAD_FIT_ADDRESS=0x10000000` a za ním FIT (BL31 + Zephyr + DTB). `rk3399-image.pl --maskrom` přepíše binman symbol `_binman_payload_prop_size` |
| `-loader.bin` | obojí v jednom souboru ve formátu boot_merger (tag „BOOT“, čip `0x33333043` = RK330C): položka 0x471 a 0x472, data kódovaná RC4 s klíčem BootROM po celé délce a doplněná na 2 KiB, na konci CRC32 s polynomem `0x04c10db7` |

- `rkdeveloptool pack` tenhle loader neumí vyrobit: RC4 volá s 16bitovou délkou a 472 je větší než 64 KiB.
- V loaderu nejsou flash položky, takže funguje jen `db`, ne `ul`/`wl`. Na ty by byla potřeba 472, která mluví protokolem rockusb (U-Boot proper s `rockusb 0 mmc 0`).
- SPL pozná USB boot (`BROM_BOOTSOURCE_ID` 0x81 na `0xff8c0010`) a vypíše `Trying to boot from RAM`. Hláška `spl_perform_arch_fixups: could not map BootROM boot device to ofpath` je neškodná.
- Jak do maskromu: deska bez bootovatelného média (SPI, eMMC i SD prázdné) a vypnout/zapnout. OTG je horní USB 3.0 port, kabel A–A.

## Napájení

ROCK Pi 4 **nenapájet z USB portu laptopu.** Po BL31 (DDR na 800 MHz, start BL33) odběr stoupne a SoC se resetuje (brownout). Vypadá to jako smyčka TPL → SPL → BL31 → TPL, ať jako BL33 běží Zephyr nebo U-Boot proper.

## SWD přes micro SD slot

„JTAG“ v SD slotu je u RK3399 dvouvodičové SWD. AP (Cortex-A) i MCU (Cortex-M0) mají každý svou dvojici. Všechny ROCK Pi 4 targety (a53, a72, smp, m0_perilp, m0_pmu) mají v DT zapnutý `sd_jtag` (`rockchip,rk3399-sd-jtag`, `CONFIG_RK3399_SD_JTAG`). Ten při startu (PRE_KERNEL_1) přepne iomux GPIO4_B2–B5 na funkci 2 a nastaví `GRF_SOC_CON7.force_jtag`. SD host je vypnutý.

Raspberry Pi Debug Probe (CMSIS-DAP), port D:

| probe | barva | RK3399 | SD signál | pin microSD |
|---|---|---|---|---|
| SD (SWDIO) | žlutý (kraj) | APJTAG_TMS (GPIO4_B3) | DAT3/CD | 2 |
| GND | černý | VSS | VSS | 6 |
| SC (SWCLK) | oranžový (kraj) | APJTAG_TCK (GPIO4_B2) | DAT2 | 1 |

- MCU dvojice: CLK (pin 5) = MCUJTAG_TCK, CMD (pin 3) = MCUJTAG_TMS.
- VDD (pin 4) nepřipojovat, úrovně jsou 3,3 V.
- UART2 jde přes port U téhož probe.
- JST-SH konektory pořádně domáčknout.

```
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed 1000" \
        -f target/rk3399.cfg
aarch64-zephyr-elf-gdb zephyr.elf -ex "target extended-remote :3333"
(gdb) add-symbol-file bl31.elf
```

Ke každému buildu dává CI archiv `<img>-<deska>-debug.tar.xz`: `zephyr.elf`, `.map`, `.dts`, `.config`, TF-A ELF (ROCK Pi 4 `bl31.elf`, ESPRESSObin `bl1/bl2/bl31.elf`) a na ROCK Pi 4 i U-Boot TPL, SPL a proper.

## První nález přes SWD

Zephyr stál po POST_KERNEL 75 a SWD ukázal CPU0 v `arch_cpu_idle` za `wfi`. Hlavní vlákno spalo v `k_msleep` při inicializaci eMMC a tick časovače nepřišel, protože build neobsahoval opravu GIC (`IGRPMODR` = 0 v NS buildech, viz [rk3399-pcie.md](rk3399-pcie.md) sekce 10). Od 16. 11. je v `epc`.

## Vydání (CI)

V release pro ROCK Pi 4 jsou `-spi.bin`, `-emmc.bin`, `-boot.bin`, `-usb471.bin`, `-usb472.bin`, `-loader.bin` a `-debug.tar.xz`. Popis souborů je v README u každého release.
