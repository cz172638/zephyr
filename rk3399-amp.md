# Zephyr AMP na RK3399: A53 + A72 + PERILP M0 (Radxa ROCK Pi 4A)

Stav k 2026-10-15. Zephyr `main` @ `124121fbb`, větev `rk3399-amp`. U-Boot `master` @ `65a7532`, větev `rk3399-perilp-m0`.

## TL;DR

- Na každém typu jádra běží samostatný Zephyr image. A53 je BL33 a startuje A72 a M0. A72 běží SMP na obou jádrech. Na PERILP M0 jsou pomalé periferie a power management.
- PMU M0 nechávám TF-A, protože ho používá pro DDR DFS a suspend.
- Boot: TPL/SPL → BL31 → Zephyr A53 (BL33) → PSCI `CPU_ON` A72 a uvolnění M0 resetem přes CRU.
- Ověřeno bez HW:
  - build všech tří targetů, sysbuild sample a Edge-V regrese (twister build-only, 5/5),
  - checkpatch bez chyb,
  - U-Boot SPL s remapem M0 se přeloží a disassembly sedí na offsety SGRF,
  - FIT generátor otestovaný na syntetickém 3-segmentovém BL31 (`mkimage -l` vypíše správnou konfiguraci).
- Na HW netestováno. Nejvíc nejistot je kolem M0: offsety remapu jsou odvozené, velikost remap okna neznám a routing přerušení není veřejný.

## 1. Rozdělení jader a obrazů

| target | jádra | role | konzole | paměť (fyz.) |
|---|---|---|---|---|
| `rock_pi_4a/rk3399/a53` | 4× A53, MPIDR 0x0–0x3 | BL33, launcher, shell | UART2 (piny 8/10, 1 500 000) | 0x00200000 / 30 MiB |
| `rock_pi_4a/rk3399/a72` | 2× A72, MPIDR 0x100–0x101 | výpočetní / RT | UART4 (piny 19/21) | 0x02000000 / 32 MiB |
| `rock_pi_4a/rk3399/m0` | PERILP Cortex-M0 | pomalé periferie, PM | RAM konzole | 0x04000000 / 1 MiB |
| sdílená | – | AMP hlavička + M0 log | – | 0x04100000 / 1 MiB |

Ve výchozím stavu je každý image non-SMP. `CONFIG_SMP=y` funguje v rámci clusteru: A53 má v DT čtyři CPU, A72 dvě. A72 demo běží SMP.

Proč všechno v DDR: TF-A označí celý 192 KiB INTMEM jako secure (`SGRF_INTSRAM_S`). Prvních 16 KiB si navíc bere pro vlastní kód a M0 binárky. NS A-jádra ani M0 (všichni masteři jsou NS) tam tedy přístup nemají.

## 2. Zephyr: co se změnilo

1. **SoC `rk3399` s HWMv2 clustery** `a53`, `a72` a `m0`.
   - DT je rozdělený na periferie viditelné ze všech jader (`dts/vendor/rockchip/rk3399_periph.dtsi`), společnou A část a jeden include na cluster.
   - PSCI je popsané jako `arm,psci-1.1`. Dřívější `1.0` žádný driver nebindoval.
   - Generic timer 24 MHz.
   - Na A72 je zapnuté `GIC_SAFE_CONFIG`, takže distributor neresetuje.
   - Khadas Edge-V se přesunul na `khadas_edgev/rk3399/a53`.
2. **pinctrl RK3399** (GRF a PMUGRF, iomux a pull). Offsety jsou převzaté z popisu banků v linuxovém `pinctrl-rockchip.c`.
3. **AMP launcher na A53** (`soc/rockchip/rk3399/amp_launcher.c`), který popisuje uzel `rockchip,rk3399-amp`:
   - inicializuje hlavičku v `zephyr,ipc_shm` (mapovanou non-cacheable, protože M0 není koherentní),
   - M0: nastaví `fclk_cm0s` z GPLL/8 (`CLKSEL_CON24`), odgatuje (`CLKGATE_CON7/24/25`) a uvolní resety v `SOFTRST_CON11` (NOC a jádro, 5 µs, PO) ve stejném pořadí, jaké TF-A používá pro PMU M0,
   - A72: `pm_cpu_on(0x100, 0x02000000)`. Na bázi obrazu leží `b __reset` (arm64 image header), takže jako vstup stačí báze.
   - shell: `amp status`, `amp m0 start|stop|log`, `amp a72 start`.
4. **Heartbeat na A72 a M0** (`amp_remote.c`): image se zapíše jako `running` a každých 100 ms zvýší čítač.
5. **Deska ROCK Pi 4A** se třemi targety. `board_a72.c` přepne `clk_uart4_pmu` na xin24m a odgatuje `pclk_uart4_pmu`. Do té doby UART4 nikdo neinicializuje.
6. **Sample** `samples/boards/radxa/rock_pi_4a/amp` přes sysbuild postaví všechny tři obrazy najednou.

## 3. Boot

### 3.1 U-Boot SPL

Patch `rockchip: rk3399: optionally remap the PERILP Cortex-M0 in SPL` (Kconfig `ROCKCHIP_RK3399_PERILP_M0_REMAP`, `..._BASE=0x04000000`) dělá v `spl_board_init()` (EL3) toto:

```
SGRF + 0x8100 (PERILP_CON0) <- mask bit7, 0
SGRF + 0x810c (PERILP_CON3) <- base[27:12]
SGRF + 0x811c (PERILP_CON7) <- base[31:28]
```

Offset 0x8100 vychází z `struct rk3399_pmusgrf_regs::perilp_con[]` v U-Boot a sedí s poznámkou „PERILPM0 @ 0x8100“ v rešerši SGRF. Sekvence zrcadlí `m0_init()` a `m0_configure_execute_addr()` z TF-A pro PMU M0 (`SGRF_PMU_CON0/3/7` na 0xc100). TRM tyto registry neuvádí ([rockchip-linux/kernel#170](https://github.com/rockchip-linux/kernel/issues/170)).

### 3.2 FIT

`boards/radxa/rock_pi_4a/support/mkfit_amp.py`:

- `firmware = atf-1` (BL31, vstup 0x40000),
- `loadables = zephyr-a53 (os="u-boot"), atf-2 (PMUSRAM), atf-3 (INTMEM), zephyr-a72, zephyr-m0`.

SPL bere jako BL33 obraz s `os = "u-boot"`. Fallback `CONFIG_TEXT_BASE` je na rk3399 také 0x00200000, takže by stejně trefil A53. Výstup se generuje s `mkimage -E -B 0x8`, jak to Rockchip SPL očekává.

```sh
west build --sysbuild -b rock_pi_4a/rk3399/a53 samples/boards/radxa/rock_pi_4a/amp
boards/radxa/rock_pi_4a/support/mkfit_amp.py --bl31 bl31.elf \
    --a53 build/amp/zephyr/zephyr.elf --a72 build/remote_a72/zephyr/zephyr.elf \
    --m0 build/remote_m0/zephyr/zephyr.elf --mkimage u-boot/tools/mkimage -o amp.itb
dd if=idbloader.img of=/dev/sdX seek=64 conv=notrunc
dd if=amp.itb of=/dev/sdX seek=16384 conv=notrunc
```

Poznámka k TF-A: s Zephyr SDK toolchainem (gcc 14, aarch64-zephyr-elf) BL31 pro rk3399 nesestavím. Linker hlásí `relocation truncated (R_AARCH64_ADR_PREL_LO21)` v A72 errata vektorech. Použij `aarch64-linux-gnu`/`aarch64-none-elf` jako v dokumentaci U-Boot. M0 blob TF-A s binutils 2.4x potřebuje `-Wl,-e,0`, jinak skončí na `--fatal-warnings`.

## 4. Nejistoty a otevřené body

1. **PERILP M0 remap.**
   - Offsety odvozuji z U-Boot struktury a analogie s PMU M0.
   - Velikost remap okna neznám. Předpokládám lineární okno aspoň 2 MiB, takže sdílená paměť je pro M0 vidět na 0x00100000. Pokud je okno menší, M0 RAM konzole a heartbeat nebudou vidět (A53 a A72 to neovlivní).
   - Ověření na HW: po startu M0 zkontrolovat `amp status` (heartbeat M0) a `amp m0 log`.
2. **Hodiny M0.** GPLL = 800 MHz předpokládám podle U-Boot SPL a `fclk_cm0s` = 100 MHz odpovídá `SYS_CLOCK_HW_CYCLES_PER_SEC` M0 obrazu. Když nesedí, časy na M0 budou škálované.
3. **Přerušení M0.** Arbiter (142 vstupů na 18 výstupů do NVIC) není zdokumentovaný. Mailbox0 je podle TRM pro PERILP M0, takže je to první kandidát pro IPC A53 ↔ M0.
4. **IPC.** Zatím jen sdílená hlavička. Další krok: A53 ↔ A72 přes SGI (GICv3 `ICC_SGI1R` s Aff1 = 1) a mbox driver, A ↔ M0 přes mailbox0.
5. **Hodiny a resety obecně.** Clock ani reset driver pro RK3399 v Zephyru není. Používám pevné hodnoty a přímé zápisy do CRU.
6. **UART4 a SPI1.** Desky v1.4+ mají na stejných pinech SPI flash.
7. **TF-A a A72.** BL31 rk3399 umí `CPU_ON` druhého clusteru (`PLATFORM_CLUSTER_COUNT 2`, CCI-500). Že TF-A pustí A72 s NS EL1 vstupem z jiného OS, ověří až běh na HW.

## 5. Artefakty

- `zephyr-rk3399-amp.bundle`: Zephyr `rk3399-amp` (5 commitů nad `124121fbb`) a `gh-pages`.
- `uboot-rk3399-perilp-m0.bundle`: U-Boot `rk3399-perilp-m0` (1 commit nad `65a7532`).

```sh
git -C zephyr fetch zephyr-rk3399-amp.bundle rk3399-amp:rk3399-amp gh-pages:gh-pages
git -C u-boot fetch uboot-rk3399-perilp-m0.bundle rk3399-perilp-m0:rk3399-perilp-m0
```

## Zdroje

- TF-A `plat/rockchip/rk3399/drivers/pmu/m0_ctl.c`, `drivers/secure/secure.c`, `include/plat.ld.S`, `include/shared/bl31_param.h`
- U-Boot `arch/arm/include/asm/arch-rockchip/grf_rk3399.h`, `common/spl/spl_atf.c`, `common/spl/spl_fit.c`
- Linux `drivers/clk/rockchip/clk-rk3399.c`, `include/dt-bindings/clock/rk3399-cru.h`, `drivers/pinctrl/pinctrl-rockchip.c`, `arch/arm64/boot/dts/rockchip/rk3399-base.dtsi`
- Radxa: [ROCK 4 GPIO](https://docs.radxa.com/en/rock4/hardware/rock4-gpio)
- RK3399 TRM Part 1, kap. 6 (Cortex-M0, interrupt arbiter): [mouser.com/pdfDocs/Rockchip_RK3399TRM.pdf](https://www.mouser.com/pdfDocs/Rockchip_RK3399TRM.pdf)
- SGRF rešerše: [gist CrystalGamma](https://gist.github.com/CrystalGamma/a68333fa4c9fda7eb6c09d30ad4937fe)
