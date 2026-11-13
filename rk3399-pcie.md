# RK3399 (ROCK Pi 4): PCIe RC/EP jako u ESPRESSObinu, Zephyr z TF-A, boot a úložiště na eMMC

Větev `epc` (sloučené `espressobin` a `rk3399`), commity `c9eb648da` (PCIe ovladače), `43c64204b` (image skript), `28817904e` (pinctrl, CRU), `fc4cfc915` (SPI, SD) `4a3afa951` (role v settings, FAT na SD), `0c8c25e9d` (JTAG ve slotu SD), `a0e6614b9` (eMMC, `emmc update`) `ef03ba389` (image 153). Další image vydává CI jako `lab-build-<n>` (od 154) se společným číslem pro všechny desky. TF-A je ve větvi `rk3399-m0-ns`, commit `e09b846ea`. U-Boot je ve větvi `rk3399-perilp-m0`, beze změn v defconfigu.

## 1. Co je hotové

RK3399 má jediný PCIe řadič a ten umí obě role. Je to Cadence jádro za Rockchip „client" wrapperem, PHY se nastavuje přes GRF. Ovladače jsou udělané stejně jako u Aardvarku:

- V image jsou oba ovladače.
- Role se určí jednou při bootu, ještě před inicializací řadiče (`zephyr/drivers/pcie/role.h`).
- Ovladač, který nebyl vybrán, na hardware nesahá.
- Shell je společný: `pcie controller | ls | cfg | cap`.

| Soubor | Účel |
|---|---|
| `drivers/pcie/pcie_role.c`, `include/zephyr/drivers/pcie/role.h` | Role pro tento boot: `pcie_role_get/save/saved`; `pcie_ctrl_is_rc()` funguje pro Aardvark i RK3399 |
| `drivers/pcie/pcie_rk3399_port.{c,h}` | Společný bring-up: hodiny, resety, client registry, PHY přes GRF, LTSSM a stav linku |
| `drivers/pcie/endpoint/pcie_epc_rk3399.c` | EP na EPC API, nahrazuje starý `pcie_ep_rk3399.c` (staré pcie_ep API) |
| `drivers/pcie/controller/pcie_rk3399.c` | RC (`rockchip,rk3399-pcie`), generická enumerace |
| `dts/arm64/rockchip/rk3399.dtsi` | `pcie0` (RC) a `pcie0_ep` (EP) na stejném `apb` |
| `boards/radxa/rock_pi_4/rock_pi_4_a.dtsi` | PERST# na GPIO4_D3, 3,3 V slotu na GPIO2_D2 |
| `samples/boards/rockchip/rk3399/pcie_shell` | Shell s oběma rolemi a funkcí `pcie_epf_test` |
| `pinctrl_rk3399.c`, `sdhc_rk3399_emmc.c`, `sdhc_dw_mshc.c`, `spi_rockchip.c` | Piny, eMMC, micro SD/JTAG, SPI (sekce 7) |
| `scripts/rockchip/rk3399-image.pl` | SPI a SD image (Perl), SD s MBR a FAT |

EPF funkce (`test`, `fabric`, `nvme`, `vntb`) a `lab_role()` se dřív ptaly přímo na `advk_mode_get()`. Teď volají `pcie_ctrl_is_rc()`, takže fungují i na RK3399.

## 2. Volba role

```
uart:~$ pcie controller mode rc      # nebo ep
uart:~$ kernel reboot warm
```

Kam se role ukládá:

- **Settings** (`pcie/mode`), pokud je `CONFIG_PCIE_ROLE_SETTINGS` a backend naběhne. Sample `pcie_shell` je má v souboru `/EMMC:/settings` na FAT eMMC.
- **`PMUGRF_OS_REG1` (`0xff320304`)** se zapisuje při každém uložení a čte se, když settings roli nemají nebo nejsou (např. eMMC bez FAT). Hodnota přežije warm reset, ne vypnutí napájení. Ukládá se jako magic `0x50430000 | role`. `OS_REG0` je boot mode, `OS_REG2/3` drží informace o DRAM, na ty se nesahá.
- **Bez uložené role** rozhoduje `CONFIG_PCIE_ROLE_DEFAULT_RC`, výchozí je EP.

Pořadí inicializace (POST_KERNEL):

| Priorita | Co |
|---|---|
| 81 | role |
| 82 | EPC |
| 83 | EPF |
| 84 | RC |

## 3. Bring-up RC a EP vedle sebe

| Krok | EP (`pcie_epc_rk3399.c`) | RC (`pcie_rk3399.c`) |
|---|---|---|
| 1. GPIO | 3,3 V slotu (GPIO2_D2) **vypnout**, napájí host; PERST# (GPIO4_D3) jako vstup | PERST# výstup, aktivní (reset); 3,3 V slotu zapnout |
| 2. Hodiny, resety | `rk3399_pcie_port_prepare()` | totéž |
| 3. Reset sekvence, client | `port_init(rc=false)`: lanes, Gen2 sel, `MODE_RC=0`, `CONF_ENABLE=0` | `port_init(rc=true)`, potom `CONF_ENABLE=1` |
| 4. PHY | GRF `0xe214/0xe220/0xe2a4`: PLL lock, 100 MHz, relock | totéž |
| 5. Identita | `LM_ID` (VID/SSVID), hlavička f0 z DT, MSI-X capability skrytá | VID `0x1d87` do `LM+0x44`, class `0x060400` do `RP+0x08`, L1SS skrytý (next pointer za TPH), MPS 256, FTS, common clock, RCB |
| 6. Okna | OB: 32 regionů × 1 MiB v `0xfa000000`, region 0 pro MSI, zbytek pro alokátor EPC; IB: BARy přes `cdns_ep_set_bar()` | OB region 0 = config (`0xf8000000`, 32 MiB, 25 bitů); regiony 1..32 podle `ranges`: mem `0xfa000000` 30 MiB, io `0xfbe00000` 1 MiB; IB region 2, 32 bitů → 0 |
| 7. Link | `LINK_TRAIN=1` až po uvolnění PERST#; PERST# střídá hrany (DW GPIO neumí obě) | `LINK_TRAIN=1`, 100 ms, uvolnit PERST#, 100 ms, čekat na link max. 500 ms |
| 8. Gen2 | podle hosta | retrain (`LnkCtl.RL`), čekat na `LM_CTRL[4:3]=5G`, jinak zůstane Gen1 |
| 9. Za běhu | work každých 20 ms: link, BME, hot reset → `pcie_epc_notify()` | `pcie_generic_ctrl_enumerate()` a `pcie_dev_scan()` |
| 10. IRQ | INTx přes `CLIENT_LEGACY_INT_CTRL`, MSI přes region 0; MSI-X `-ENOTSUP` | zatím žádné, zařízení jen pollovat |

Config requesty RC jdou přes OB region 0, adresa `bus<<20 | dev<<15 | fn<<12 | reg`. Typ 0/1 se přepíná v `DESC0` při každém přístupu (pod spinlockem). Jsou dostupné jen sběrnice 0–31 a pod root portem jen zařízení 0. Root port je skutečný hardware: normální pohled na `core+0`, privilegovaný na `core+0x200000`. Na rozdíl od Aardvarku tu není softwarový root port.

## 4. Mapa registrů (apb `0xfd000000`)

| Oblast | Offset od apb | Obsah |
|---|---|---|
| client | `0x000000` | CONFIG, LEGACY_INT_CTRL `0x0c`, SIDE_BAND `0x20`, DEBUG_OUT_0 (LTSSM) `0x3c`, BASIC_STATUS0/1 `0x44/0x48`, INT_MASK/STATUS `0x4c/0x50` |
| core: config f0 / root port | `0x800000` | normální pohled (EP: funkce `fn<<12`) |
| core: LM | `0x900000` | `PL_CONFIG0`/`CTRL`, PLC1 `+4`, ID `+0x44`, EP BAR cfg `+0x240`, EP_FUNC_CFG `+0x2c0`, RC_BAR_CONF `+0x300` |
| core: RP bank | `0xa00000` | privilegovaný zápis: class, LCS `+0xd0`, DCSR `+0xc8`, THP `+0x274` |
| core: AT | `0xc00000` | OB region r na `r*0x20` (ADDR0/1, DESC0/1), IB root portu `+0x800`, IB EP BARů `+0x828` |

## 5. Boot chain bez rkbin

```
BootROM → U-Boot TPL (LPDDR4 init) → U-Boot SPL → FIT {
    firmware  atf-1    BL31 0x40000
    loadables zephyr   0x200000, os "u-boot" → BL33
              atf-2..4 0x62000, PMUSRAM 0xff3b0000, SRAM 0xff8c0000
    fdt       fdt-1    0x07f00000 (SPL do něj zapíše /fit-images → BL31 dostane vstup Zephyru)
}
```

Bez `fdt` by SPL předal BL31 jako BL33 `CONFIG_TEXT_BASE` (0x800000), ne Zephyr.

Sestavení:

```
# TF-A (M0 firmware překládá arm-zephyr-eabi ze SDK)
make CROSS_COMPILE=aarch64-linux-gnu- \
     M0_CROSS_COMPILE=$SDK/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi- PLAT=rk3399 bl31
# U-Boot jen kvůli TPL/SPL
make O=../uboot-build-rp4 rock-pi-4-rk3399_defconfig
make O=../uboot-build-rp4 CROSS_COMPILE=aarch64-linux-gnu- BL31=.../bl31.elf
# Zephyr
west build -b rock_pi_4/rk3399/a53 samples/boards/rockchip/rk3399/pcie_shell
# image
scripts/rockchip/rk3399-image.pl --bl31 .../bl31.elf --zephyr build/zephyr/zephyr.elf \
    --uboot ../uboot-build-rp4 --installer -o 153-rock_pi_4
```

Oprava v TF-A (`e09b846ea`): M0 Makefile předával linker script přes `-Wl,-T`. Gcc ze Zephyr SDK má výchozí specs s picolibc a přidal k němu i svůj script. Výsledek: `.text` na `0x10000000`, `rk3399m0pmu.bin` o velikosti 256 MiB a přetečení PMUSRAM. Oprava je `-T`.

## 6. Image od 153: jen eMMC (152 byla SPI + SD)

Jména souborů jsou `<n>-rock_pi_4-<co>` (153–155 se ještě jmenovaly `<n>-rp4-…`). `<n>` je číslo buildu `lab-build-<n>`.

Obě ROCK Pi 4 mají eMMC, takže je jediné boot médium i úložiště. Slot micro SD slouží jen k instalaci a pro JTAG breakout.

| Soubor | Obsah | Použití |
|---|---|---|
| `<n>-rock_pi_4-emmc.bin` (48 MiB) | MBR, `idbloader.img` na sektoru 64, FIT na sektoru 16384, FAT16 32 MiB od sektoru 32768 | eMMC od sektoru 0 |
| `<n>-rock_pi_4-sd-install.bin` (80 MiB) | stejný boot, na FAT (64 MiB) leží `<n>-rock_pi_4-emmc.bin` | instalační karta |

BootROM i U-Boot SPL čtou eMMC a SD stejně (sektor 64 / 16384). Proto `-emmc.bin` nabootuje i z karty a instalační karta je jen tentýž image s větší FAT.

**Instalace na eMMC bez Linuxu i bez U-Boot:**

```
$ dd if=<n>-rock_pi_4-sd-install.bin of=/dev/sdX bs=1M conv=fsync
# karta do slotu; eMMC prázdná nebo s naším image (BootROM: SPI → eMMC → SD)
uart:~$ fs ls /SD:
uart:~$ emmc update /SD:/<n>-rock_pi_4-emmc.bin
................................................
done: 50331648 bytes written and verified; remove the card (or put the JTAG breakout back) and reset
```

`emmc update` píše raw od sektoru 0 po 32 KiB, každý blok zpětně čte a porovnává. FAT na eMMC před zápisem odpojí a po něm zase připojí.

**Pozor na pořadí BootROM (SPI → eMMC → SD):**
- **Starý image v SPI flash vyhraje** nad eMMC (desky 4A+/4B+/4C, pokud tam je image 151/152). Smaž ho z Zephyru: `flash erase flash@0 0 0x1000`, případně maskrom (SPI CLK na GND při zapnutí).
- **Na eMMC, která už bootuje, se k instalační kartě nedostaneš.** BootROM vezme eMMC dřív. Nový image proto pusť `emmc update` z karty, kterou nabootuješ, až když eMMC nebootuje (prázdná nebo s poškozeným idbloaderem). Nebo pozdější verzi zapiš z běžícího Zephyru: soubor na `/EMMC:` (nebo `/SD:`) a `emmc update` přepíše eMMC, ze které běží. Zápis jde přes disk API, běžící kód je v RAM.

Další soubory na FAT (LLEXT moduly, data) přidá `--fat-add soubor`. Velikost FAT na eMMC mění `--fat-mb`, nebo ji po instalaci zvětšíš na Linuxu (`parted resizepart 1 100%`, `fatresize`). SPI image skript udělá už jen na požádání (`--spi`).

- **Role:** settings v souboru `/EMMC:/settings` (`CONFIG_SETTINGS_FILE`). FAT eMMC se připojí v `SYS_INIT` POST_KERNEL 76, tedy po SDHC (70) a před rolí (81). `OS_REG1` zůstává jako záloha.
- **Device tree pro SPL:** všechny desky používají `rk3399-rock-pi-4a`. SPL umí eMMC (`MMC_SDHCI_ROCKCHIP`) i SD a pořadí bere „same-as-spl“.
- **LLEXT moduly s prefixem 153** zatím nejsou: chybí síť (lab agent). Úložiště je (`/EMMC:`), `lab mod load` z eMMC je další krok.

## 7. eMMC, SD/JTAG a SPI v Zephyru

| Ovladač | Soubor | Co umí |
|---|---|---|
| pinctrl | `drivers/pinctrl/pinctrl_rk3399.c` | iomux a pull v PMUGRF (banky 0–1) a GRF (2–4), `RK_PINMUX(bank, pin, func, pull)`; 1,8V pull typ banky 0 A/B a banky 2 C/D se přepočítá |
| CRU | `clock_control_rk3399_cru.c` | `get_rate`/`set_rate` pro `SCLK_SPI0/1/2/4/5`, `SCLK_SDMMC`, `SCLK_EMMC`, `ACLK_EMMC` z PLL registrů (GPLL, pro nízké frekvence 24 MHz) |
| eMMC | `drivers/sdhc/sdhc_rk3399_emmc.c` (`rockchip,rk3399-sdhci-5.1`) | Arasan SDHCI 5.1 + eMMC PHY v GRF, polling, PIO, 1/4/8 bit, legacy a high speed SDR do 52 MHz |
| SD | `drivers/sdhc/sdhc_dw_mshc.c` (`rockchip,rk3399-dw-mshc`) | DesignWare MSHC na SDHC API, polling, FIFO PIO, auto CMD12, 3,3 V, do 50 MHz, 1/4 bit; JTAG ve slotu (`rockchip,sd-jtag`) |
| SPI | `drivers/spi/spi_rockchip.c` (`rockchip,rk3066-spi`) | master, 8bit, polling přes FIFO, CS0/1 nebo `cs-gpios`; na ROCK Pi 4 jen kvůli smazání starého image v SPI flash |

**eMMC** (`&sdhci`, disk „EMMC“, 8 bit):
- **Hodiny:** základní hodiny `SCLK_EMMC` z GPLL (594/3 = 198 MHz), zapsané do `corecfg_baseclkfreq` (`GRF_EMMCCORE_CON0`). Programovatelné hodiny jsou vypnuté (`CON11` = 0). Dělič N = 0 (plné základní hodiny) na tomhle jádře nefunguje (v Linuxu `CLOCK_DIV_ZERO_BROKEN`), takže 52 MHz → 49,5 MHz.
- **PHY** (GRF `0xf780`–`0xf7a0`): vypne se při změně hodin a nad 400 kHz znovu zapne. Pořadí: impedance 50 Ω, výstupní tap delay, power, čekání na `caldone`, rozsah DLL podle hodin karty (50/100/150/200 MHz) a čekání na `dllrdy`. Postup podle U-Boot `rockchip_sdhci.c`.
- **Co nastavil U-Boot SPL** (při bootu z eMMC), ovladač zahodí: PHY off, `SW_RESET` all.
- **Chybí:** HS200/HS400 (tuning, 1,8V signalizace), DDR52, ADMA a přerušení.

**SD slot jako JTAG** (`rockchip,sd-jtag`):

| Stav slotu | Co se stane |
|---|---|
| prázdný (card detect = žádná karta) | `GRF_SOC_CON7.force_jtag` zůstane nastavený a SoC sám přepne piny na JTAG: GPIO4_B2/B3 AP TCK/TMS, B4/B5 MCU TCK/TMS |
| karta | normální SD (`/SD:`), instalace |
| breakout, který card detect vidí jako kartu | po power-on neodpoví žádný příkaz (4 timeouty odpovědi) → ovladač přepne iomux GPIO4_B2–B5 na funkci 2 (JTAG), vypne napájení a hodiny slotu a do dalšího bootu hlásí „žádná karta“ |

Při detekci se na pinech krátce objeví SD hodiny a příkazy (CMD0/8/55/1). Debugger připojený během bootu to může vidět, po přepnutí je JTAG čistý.

**SPI1 + flash** (4A+/4B+/4C): už nebootuje ani neukládá. Je v device tree jen proto, aby šel smazat starý image (`flash erase`). SPI1 sdílí GPIO1_A7/B0 s UART4, konzole je na UART2.

## 8. Co chybí

- **Na hardwaru zatím nic neověřeno.** Ověřené je jen sestavení (Zephyr, TF-A, U-Boot; lokálně i v GitHub Actions) a rozložení image (FIT magic na správných offsetech, idbloader sedí, MBR a FAT čitelné přes mtools). Ovladače eMMC, SD/JTAG a SPI neběžely na železe.
- **RC bez IRQ.** INTx by šlo přes client INT. MSI potřebuje GIC ITS, ten Zephyr na RK3399 zatím nemá. Chybí AER.
- **Žádná perzistentní identita** (`pcie id set` platí do resetu) ani DSN; settings už jsou (`/EMMC:/settings`), napojení id/DSN na ně chybí.
- **REFCLK:** RK3399 PHY nemá vstup referenčních hodin, partner musí mít vlastní (SRIS/SSC podle hosta). **Na crossover kabelu nikdy nepropojovat REFCLK ani napájení.**
- **EP a RC zatím jen na A53 obrazu.** A72 overlay pro `pcie_shell` není.

## 9. TODO: plná kontrola L0 / LTSSM (nezapomenout)

Otázka byla, jestli jde u PCIe řadiče plně řídit L0. Krátká odpověď: standard dává jen hrubé páky, jemné řízení je vendor-specifické.

**Standard (každý řadič):**
- `LnkCtl.Retrain Link` (jen downstream/root port): Recovery → L0.
- `LnkCtl.Link Disable`: Disabled, po zrušení znovu Detect.
- `LnkCtl.ASPM Control` (L0s/L1), `LnkCtl2.Target Link Speed` a `Enter Compliance`.
- `LnkCtl3` (Perform Equalization), Hot Reset přes Secondary Bus Reset v bridge control.
- Stav: `LnkSta` (speed, width, Training, DLLLA).

**Vendor:**

| Řadič | Co je k dispozici |
|---|---|
| DesignWare | Port Logic `0x708` Port Force Link (vynutit stav LTSSM), `0x70c` ACK freq, LTSSM debug `0x728/0x72c` |
| Cadence (RK3399, J721E) | LM: `PL_CONFIG0` (link status, LTSSM ve vyšších revizích); RK3399 client `DEBUG_OUT_0` je jen čtení; `LINK_TRAIN` v client CONFIG zapíná a vypíná trénink |
| Aardvark | `CTRL0` link train, LTSSM ve `0x4c0x`. Force-state registr neznám, kandidát k hledání pomocí `regdiff`/`wmap` |

**Plán:** `pcie link <ctrl> {retrain|disable|enable|speed N|aspm l0s|l1|off|compliance}` jako vendor-neutral příkaz nad standardními registry. Vendor-specifické vynucení stavů (DW Force Link, Cadence LM) přijde do `pcie debug ltssm` jako LLEXT modul.
