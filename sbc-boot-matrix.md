# Zephyr (a ThreadX) na všech jádrech: boot řetězce a plán po deskách

Stav k 2026-10-15. Zkontrolované stromy:

- TF-A `master` @ `0892ad4`
- U-Boot `master` @ `65a7532`
- Zephyr `main` @ `124121fbb`
- ThreadX `master` @ `56c8695`

## 1. Princip: co dělá který stupeň

| úroveň | Arm (aarch64) | RISC-V | úkol |
|---|---|---|---|
| ROM | BootROM | BootROM | načte první stupeň do SRAM |
| BL1/BL2 | TF-A BL1+BL2, **nebo** U-Boot TPL/SPL, **nebo** TI R5 SPL | U-Boot SPL / FSBL | DRAM init, načtení dalších obrazů |
| BL31 | TF-A BL31 (EL3: PSCI, SMC, GIC secure) | OpenSBI `fw_dynamic` (M-mode: SBI, HSM, timer) | runtime firmware |
| BL33 | **Zephyr / ThreadX** (EL2 → EL1 NS) | **Zephyr / ThreadX** (S-mode) | OS |

Zephyr jako BL33 nebo SBI payload potřebuje:

- **aarch64:** link na BL33 adresu platformy, `CONFIG_ARMV8_A_NS=y`, PSCI (`arm,psci-1.1`/`0.2`). Jádra se zapínají přes PSCI `CPU_ON`, jak pro SMP, tak pro AMP druhého clusteru.
- **RISC-V:** `CONFIG_RISCV_S_MODE_EXTERNAL_SBI=y` a `PM_CPU_OPS_SBI` (HSM `hart_start`). Obojí je v Zephyru, viz `qemu_riscv64/.../opensbi_smp`.
- **MCU jádra** (M0, M3, R5F, E906, LP core) žádné TF-A nemají. Startuje je hostitel (reset a hodiny) nebo bootloader.

Kde TF-A nemá BL2, zastává jeho roli U-Boot SPL. Na Rockchipu, Allwinneru a SpacemiTu je to standardní cesta i v upstreamu. Plnohodnotný TF-A BL2 + FIP je upstream jen u NXP Layerscape a Marvellu.

### Více obrazů „správně“ v TF-A

FIP umí kromě pevných slotů (`--nt-fw` = BL33) nést i obecné bloby: `fiptool create --blob uuid=<UUID>,file=a72.bin`. BL2 je nahraje, když je platforma má v `bl_mem_params_desc` (vlastní image ID + UUID). Tohle je otevřená a čistá cesta pro AMP obrazy na NXP a Marvellu. Na Rockchipu a Allwinneru to samé řeší FIT `loadables` v SPL.

## 2. Matice desek

Legenda: ✅ upstream a otevřené · 🟡 otevřené, ale ve forku nebo WIP · 🔒 binární blob · ❌ chybí

| deska | SoC | jádra (všechna) | DRAM init | BL1/BL2 | BL31 / SBI | Zephyr dnes |
|---|---|---|---|---|---|---|
| ESPRESSObin | Marvell A3720 | 2× A53, 1× Cortex-M3 (secure CPU, WTMI) | ✅ mv-ddr + WTMI (CZ.NIC) | ✅ TF-A BL1+BL2 (`PLAT=a3700`) | ✅ TF-A | ❌ |
| ROCK Pi S | RK3308 | 4× A35 | 🔒 rkbin `ddr.bin` (U-Boot `sdram_rk3308.c` = jen detekce velikosti) | ✅ U-Boot TPL/SPL (s blobem) | 🔒 rkbin `rk3308_bl31` (v TF-A rk3308 není) | ❌ (chybí i `CPU_CORTEX_A35` v arch/arm64) |
| FRWY-LS1012A, FRDM-LS1012A | NXP LS1012A | 1× A53 (PFE PE jádra jen s firmwarem) | 🟡 NXP DDR driver ve forku TF-A | 🟡 TF-A BL2 přes PBL/RCW, jen NXP fork (qoriq atf, LSDK) | 🟡 tamtéž | ❌ (je jen `ls1046a`) |
| Radxa Zero 3E | RK3566 | 4× A55 (RISC-V MCU: ověřit v TRM) | 🔒 rkbin `ddr.bin` (`sdram_rk3568.c` = jen detekce) | ✅ U-Boot SPL | ✅ TF-A `rk3568` | 🟡 `rk3568` SoC (stačí varianta rk3566) |
| ROCK Pi 4 | RK3399 | 2× A72, 4× A53, 2× M0 | ✅ U-Boot TPL `sdram_rk3399.c` | ✅ U-Boot SPL | ✅ TF-A `rk3399` | 🟡 větev `rk3399-amp` (A53, A72, PERILP M0) |
| Cubie A5E | Allwinner A527 (sun55iw3) | 8× A55 (2 clustery) + XuanTie E906 (RISC-V MCU) | ✅ U-Boot `dram_sun55i_a523.c` | ✅ U-Boot SPL | 🟡 TF-A WIP fork (v upstream tabulce Allwinneru není) | ❌ |
| Cubie A7A | Allwinner A733 | 2× A76 + 6× A55 (další jádra neověřená) | 🔒 vendor boot0 | 🔒 vendor | ❌ | ❌ |
| BeagleY-AI | TI J722S / AM67A | 4× A53, 3× R5F (wkup, mcu, main), 2× C7x | ✅ U-Boot R5 SPL `k3-ddrss` | ✅ U-Boot R5 SPL (+ 🔒 TIFS) | ✅ TF-A `k3` | 🟡 jen R5F `main`/`mcu` |
| BeagleBone AI-64 | TI J721E | 2× A72, 6× R5F, 2× C66x, 1× C7x | ✅ U-Boot R5 SPL `k3-ddrss` | ✅ U-Boot R5 SPL (+ 🔒 TIFS) | ✅ TF-A `k3` | 🟡 jen R5F |
| ESP32-P4 | Espressif | 2× HP RV32IMAFC, 1× LP RV32 | ROM (PSRAM) | MCUboot / Zephyr simple boot | – (bez SBI, M-mode) | ✅ `hpcore`, `lpcore` |
| „xy k1“ = SpacemiT K1 (předpoklad) | SpacemiT K1 | 8× X60 RV64GCV (2×4) + RCPU subsystém (ISA nezveřejněná) | 🔒 `ddr_fw.bin` (SpacemiT) | 🟡 U-Boot SPL poslaný do ML 2026-03, nezamergovaný | ✅ OpenSBI upstream (v1.9) | ❌ |

Poznámky k TI K3:

- Security firmware TIFS běží na security M4 a je **povinný blob**.
- DM firmware obsadí jedno R5F: na J722S `wkup_r5f0`, na J721E `mcu_r5f0_0`. To R5F proto nemůže být Zephyr, pokud Zephyr sám neimplementuje DM.

C66x, C7x a HiFi DSP Zephyr ani ThreadX nepodporují. Ty bych vynechal.

### Odpověď na otázku k NXP

Ano, TF-A má pro Layerscape plný **BL2** včetně otevřeného DDR driveru (`drivers/nxp/ddr`, `DDR_DRIVER_NEEDED,BL2`). Upstream ho mají jen `ls1028a`, `ls1043a`, `ls1046a`, `ls1088a` a `lx2160a`. **LS1012A je jen v NXP forku** (qoriq `atf`, větve LSDK). OpenWrt `tfa-layerscape` staví `PLAT=ls1012afrdm|ls1012afrwy|ls1012ardb` z `LSDK-20.04`. U-Boot upstream na to má `ls1012afrdm_tfa_defconfig` a `ls1012afrwy_tfa_defconfig` (`CONFIG_TFABOOT=y`), tedy U-Boot jako BL33.

Otevřená cesta tedy existuje, ale přes fork. Port `soc-ls1012a` do upstream TF-A by byl dobrý příspěvek a přímo se nabízí jako výukový projekt: PBL → BL2 → DDR → FIP.

## 3. Plán (pořadí podle „naučit se správně a otevřeně“)

1. **ROCK Pi 4 (RK3399): dokončit.**
   - Remap PERILP M0 přesunout z U-Boot SPL do TF-A `bl31_plat_setup`, kde už je `m0_init()` pro PMU M0. U-Boot tak zůstane vanilla.
   - BL31 stavět s `aarch64-none-elf`/`aarch64-linux-gnu`. Se Zephyr SDK gcc 14 linker hlásí ADR_PREL_LO21.
2. **ESPRESSObin (A3720): referenční TF-A BL1+BL2+BL31+FIP, celé otevřené.**
   - Zephyr: nový SoC `a3700` (2× A53, GIC-500) a driver UART `marvell,armada-3700-uart` (Zephyr ho nemá, je to jednoduchý driver).
   - Do FIP: `--nt-fw zephyr.bin`.
   - Cortex-M3 běží WTMI, které obsluhuje reset a DVFS pro TF-A. Zephyr na CM3 = reimplementace WTMI mailboxu, až později.
3. **FRDM/FRWY-LS1012A: TF-A BL2 přes PBL.**
   - Zephyr: SoC `ls1012a` podle `ls1046a` (A53, GIC-400 = GICv2, DUART ns16550). Jen 1 jádro, takže UP.
   - TF-A: buď NXP fork, nebo upstream port `soc-ls1012a`.
4. **SpacemiT K1: referenční RISC-V cesta SPL → OpenSBI → S-mode.**
   - Zephyr: SoC `k1` se 2 clustery (`x60c0`, `x60c1`) a AMP přes SBI HSM. Základ je `qemu_riscv64` opensbi. Potřeba PLIC, UART pxa/ns16550 a timer přes SBI.
   - DDR blob nejde obejít.
5. **BeagleY-AI, BeagleBone AI-64: A-jádra do Zephyru.**
   - J722S A53 vychází z existujícího `am62x/a53` (`soc/ti/k3/common/cortex_a`). J721E A72 je nový cluster.
   - BL31 je TI upstream (`PRELOADED_BL33_BASE=0x80080000`). R5F targety už existují.
6. **Radxa Zero 3E (RK3566):** varianta `rk3566` v `soc/rockchip/rk35`. DDR blob zůstává.
7. **Cubie A5E (A527):** až TF-A sun55i dorazí upstream. Mezitím Zephyr na E906 (M-mode, bez MMU), startovaný z A55.
8. **ROCK Pi S (RK3308):** přidat `CPU_CORTEX_A35` do arch/arm64. DDR i BL31 jsou zatím blob. Otevřená alternativa je port TF-A `rk3308` (blízký `rk3328`), což je větší úkol.
9. **ESP32-P4:** už funguje. Zbývá AMP HP ↔ LP a ověřit SMP na obou HP jádrech.
10. **Cubie A7A (A733):** odložit, mainline nic není.

## 4. ThreadX

- Porty existují pro `cortex_a35/a53/a55/a72/a76` včetně `_smp`, dále `cortex_m0/m3`, `cortex_r5`, `risc-v32` a `risc-v64`.
- **ARMv8-A příklady startují v EL3** (`startup.S` nastavuje `SCR_EL3`, `ICC_SRE_EL3` a sestupuje do EL1). Jako BL33 pod TF-A je potřeba nový low-level start: EL2 → EL1 NS, sekundární jádra přes PSCI místo spin/WFE, GIC jen NS část. Stačí jeden takový „BL33 port“ pro všechny aarch64 desky.
- **RISC-V64 má příklad `bananapi-f3`** (SpacemiT K1). Ten už běží v S-mode pod OpenSBI (SBI `set_timer` přes `ecall`), takže na K1 jde ThreadX hned.

## 5. Zdroje

- TF-A: `plat/nxp/soc-*`, `plat/marvell/armada/a3k`, `docs/plat/marvell/armada/build.rst`, `docs/plat/allwinner.rst`, `docs/plat/rockchip.rst`, `plat/ti/k3/platform.mk`
- U-Boot: `doc/board/rockchip/rockchip.rst` (rkbin TPL/BL31 pro rk3308/rk3568), `doc/board/spacemit/k1-spl.rst`, `drivers/ram/rockchip/`, `arch/arm/mach-sunxi/dram_sun55i_a523.c`
- ThreadX: `ports_arch/ARMv8-A/threadx_smp/ports/gnu/example_build/sample_threadx/startup.S`, `ports/risc-v64/gnu/example_build/bananapi-f3/`
- [OpenWrt tfa-layerscape Makefile](https://git-03.infra.openwrt.org/openwrt/staging/luka/tree/package/boot/tfa-layerscape/Makefile)
- [NXP community: TFA on LS1012A](https://community.nxp.com/t5/Layerscape/TFA-on-LS1012A-noSEC/m-p/985202)
- [linux-sunxi: A523](https://linux-sunxi.org/A523), [linux-sunxi: Radxa Cubie A7A](https://linux-sunxi.org/Radxa_Cubie_A7A)
- [Aurélien Jarno: upstream OpenSBI on SpacemiT K1](https://blog.aurel32.net/upstream-opensbi-spacemit-k1.html), [BPI-F3: K1 datasheet](https://docs.banana-pi.org/en/BPI-F3/SpacemiT_K1_datasheet)
