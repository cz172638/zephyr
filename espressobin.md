# Zephyr na ESPRESSObin (Marvell Armada 3720): TF-A BL33 + síť přes 88E6341

Stav k 2026-10-15. Zephyr `main` @ `124121fbb`, větev `espressobin` (8 commitů). TF-A `master` @ `0892ad4`, mv-ddr @ `7bcb9dc`, A3700-utils @ `a3e1c67`.

## TL;DR

- Boot je celý otevřený a obejde se bez U-Boot: boot ROM → TIM → WTMI na CM3 (mv-ddr trénuje DDR) → TF-A BL1 → BL2 → BL31 → **Zephyr jako BL33** na EL2 → EL1 NS.
- Targety jsou `espressobin` (1× A53) a `espressobin/armada3720/smp` (2× A53, druhé jádro startuje přes PSCI `CPU_ON`).
- Nové drivery:
  - pinctrl NB/SB,
  - hradla periferních hodin,
  - UART (`marvell,armada-3700-uart`),
  - MDIO `marvell,orion-mdio`,
  - minimální switch 88E6xxx (přes PHY API),
  - Ethernet NETA (`marvell,armada-3700-neta`).
- Síť: eth0 je připojený přes RGMII-ID s pevným 1000/full na port 0 switche 88E6341 (adresa SMI 1, multi-chip). Porty `wan`, `lan0` a `lan1` posílají rámce jen na CPU a v Zephyru se jeví jako **jedno rozhraní**, zatím bez DSA tagů.
- Ověřeno bez HW:
  - build obou targetů (hello_world, synchronization, dhcpv4_client, echo_server, tests/drivers/ethernet/api),
  - twister build-only 7/7,
  - checkpatch bez chyb,
  - TF-A `mrvl_flash` + `mrvl_uart` s Zephyrem jako BL33 (`fiptool info` ukazuje BL33 = 0x6B898 B).
- Na HW netestováno.

## 1. Boot chain

```
BootROM (SPI NOR)
  └─ TIM header ─ WTMI (Cortex-M3): clocks, mv-ddr DDR3 training, reset/DVFS service
               └─ TF-A BL1 (A53, EL3) ─ BL2 ─ FIP: BL31 + BL33 (Zephyr @ 0x0)
```

| komponenta | zdroj | poznámka |
|---|---|---|
| WTMI | A3700-utils-marvell `wtmi/` | `CROSS_CM3=arm-linux-gnueabi-` |
| DDR init | mv-ddr-marvell (`tfa/contrib/mv-ddr`) | `DDR_TOPOLOGY=2` = DDR3 16b 1 GiB 2CS (v5), banner `mv_ddr-devel-g541616b DDR3 16b 1GB 2CS` |
| BL1/BL2/BL31 | TF-A `PLAT=a3700` | BL31 + secure paměť na 0x04000000–0x05000000 |
| BL33 | Zephyr `zephyr.bin` | linkovaný na 0x0, 64 MiB |

Build (skript `boards/globalscale/espressobin/support/mkflash.sh`):

```sh
west build -b espressobin/armada3720/smp samples/net/dhcpv4_client
TFA=~/tfa WTP=~/A3700-utils-marvell CRYPTOPP=~/cryptopp \
CLOCKS=CPU_800_DDR_800 DDR_TOPOLOGY=2 \
    boards/globalscale/espressobin/support/mkflash.sh build/zephyr/zephyr.bin
# -> tfa/build/a3700/release/flash-image.bin, uart-images.tgz.bin
```

Na co dát pozor:

- `CLOCKSPRESET` musí odpovídat značení čipu: `C080` → `CPU_800_DDR_800`, `C100` → `CPU_1000_DDR_800`.
- TF-A potřebuje `MV_DDR_PATH` explicitně. Dokumentace tvrdí, že má in-tree default, ale bez něj build skončí chybou „requires MV_DDR_PATH“. Nejdřív `git submodule update --init contrib/mv-ddr`.

### Nahrání

Nejbezpečnější iterace je přes UART bez zápisu do flash. Boot ROM přepne do UART režimu escape sekvence, takže nejsou potřeba žádné jumpery:

```sh
mox-imager -D /dev/ttyUSB0 -E -b 6000000 -t flash-image.bin
```

Trvale ze stávajícího U-Boot (nejdřív si zazálohuj původní flash):

```
=> sf probe
=> sf read 0x1000000 0 0x400000          # záloha
=> tftpboot 0x1000000 flash-image.bin
=> sf update 0x1000000 0 $filesize
```

Zpět na původní firmware se dostaneš stejným `mox-imager` se zálohou.

## 2. Zephyr: co přibylo

| commit | obsah |
|---|---|
| `drivers: pinctrl: add Marvell Armada 37xx pin controller` | NB `0xd0013800` / SB `0xd0018800`. Výběrový registr +0x30: skupina na bit, 0 = funkce. Stav pinů tvoří skupiny `(bridge, mask, value)`. |
| `drivers: clock_control: add Armada 37xx peripheral clock gates` | Jen hradla (`CLK_DIS` +0x14). Index ve specifikátoru odpovídá tabulce v Linuxu, takže DT reference sedí s Linuxem. |
| `drivers: serial: add Marvell Armada 3700 UART driver` | Polling i IRQ přes `uart-sum` (SPI 11). Dělič nastaví jen při `clock-frequency` + `current-speed`, jinak nechá nastavení od WTMI/TF-A. |
| `drivers: mdio: add Marvell Orion SMI driver` | Jeden registr: data, PHY [20:16], reg [25:21], read b26, valid b27, busy b28. Volitelně hodiny a piny, protože na A3720 leží SMI uvnitř bloku GbE0 (`0xd0032004`). |
| `drivers: ethernet: phy: add minimal Marvell 88E6xxx switch support` | Switch v multi-chip režimu se vůči MAC tváří jako PHY (viz 3.2). |
| `drivers: ethernet: add Marvell NETA driver for Armada 3700` | Viz 3.1. |
| `soc: marvell: add Armada 3720` | `soc/marvell/armada37xx`, `dts/arm64/marvell/armada3720.dtsi`: GICv3 (GICD `0xd1d00000`, GICR `0xd1d40000`), timer 25 MHz, PSCI 0.2 smc. |
| `boards: globalscale: add ESPRESSObin` | Dva targety, DT se sítí, `mkflash.sh`, `doc/index.rst`. |

## 3. Síť

```
NETA0 (0xd0030000, SPI 42) ══ RGMII-ID 1000/full ══ 88E6341 port 0 (CPU)
                                                     ├─ port 1 wan  (PHY 0x11)
SMI (0xd0032004) ── multi-chip, adresa 1 ──────────┤ port 2 lan0 (PHY 0x12)
                                                     └─ port 3 lan1 (PHY 0x13)
```

### 3.1 NETA (`eth_mvneta.c`)

- Jedna RX a jedna TX fronta, softwarová správa bufferů (`ACC_MODE` = EXT1) a legacy parser.
- **MBUS okna:** varianta 3700 dekódovací okna neprogramuje. Stačí jedno 4GiB okno (`WIN_SIZE(0)=0xffff0000`, clear `BASE_ADDR_ENABLE` bit0, `ACCESS_PROTECT` |= 3) stejně jako `mvneta_bypass_mbus_windows()` v U-Boot.
- **DMA:**
  - Descriptory (32 B) a buffery leží v `__nocache` (`CONFIG_NOCACHE_MEMORY`, MMU je mapuje jako Normal NC).
  - Rámce se kopírují do a z `net_pkt`, takže není potřeba řešit cache ani koherenci.
  - DRAM začíná na 0, takže 32bitové DMA adresy stačí.
- **RX:**
  - HW na začátek rámce přidá 2B Marvell header a na konec nechá 4B FCS. Délka paketu je tedy `data_size - 6` a data začínají na offsetu 2.
  - Rámec rozdělený do víc descriptorů nebo s `ERR_SUMMARY` se zahodí.
  - Doplnění bufferů: `RXQ_STATUS_UPDATE` = `done | filled << 16`, po nejvýš 255 najednou.
- **TX:**
  - Jeden descriptor na rámec: `F_DESC | L_DESC | Z_PAD | L4_CSUM_NOT`.
  - `TXQ_UPDATE` += 1. Odeslané descriptory se uvolňují v `send()` z `TXQ_STATUS[29:16]`, takže TX interrupt není potřeba.
- **IRQ:**
  - Linka je jedna a sdílená, ne per-CPU. ISR zamaskuje `INTR_NEW_MASK` a probudí RX vlákno. Vlákno vyprázdní frontu a znovu odmaskuje RXQ0 a MISC.
  - RX práh je 1 rámec.
  - Vlákno navíc polluje každých `CONFIG_ETH_MVNETA_RX_POLL_MS` (100 ms) jako pojistku pro případ, že je přerušení špatně routované.
- **Link:**
  - Port běží v RGMII bez in-band AN (`GMAC_CTRL_2`: `PORT_RGMII`, reset uvolněný).
  - Při link up z PHY callbacku se nastaví `FORCE_LINK_PASS` a rychlost a duplex v `GMAC_AUTONEG_CONFIG` (jako `mvneta_mac_link_up()` v Linuxu).
  - Při link down se nastaví `FORCE_LINK_DOWN`.
- **Filtry:**
  - Unicast: 1 záznam podle posledního nibble MAC.
  - Multicast: přijímá se všechen, aby fungovalo IPv6 ND.
- **MAC:**
  - Pořadí zdrojů: DT (`local-mac-address`), pak `zephyr,random-mac-address`, pak to, co nechal v registrech předchozí boot stage, a nakonec náhodná adresa.
  - Na desce je nastavené `zephyr,random-mac-address`. Skutečná MAC je jen v prostředí U-Boot, takže ji ze štítku dej do overlaye.

### 3.2 88E6341 (`phy_marvell_mv88e6xxx.c`, compatible `marvell,mv88e6085`)

Multi-chip SMI: na adrese switche (1) jsou jen dva registry, CMD (0) a DATA (1). Příkaz má tvar `BUSY | MODE_22 | OP | dev<<5 | reg`, kde OP = write 0x0400 / read 0x0800. Interní adresy jsou port p = 0x10+p a Global 2 = 0x1c. PHY se čtou nepřímo přes registry Global 2 SMI PHY CMD/DATA (0x18/0x19).

Init (podle U-Boot `espressobin_last_stage_init()`):

1. Načte `PORT_SWITCH_ID` (port 0, reg 3). Pro 6341 je produkt 0x341.
2. Uživatelské porty: `BASE_VLAN` (reg 6) = BIT(CPU), takže každý port vidí jen CPU.
3. Porty 0–3: `PORT_CTL0` (reg 4) = 0x7f (forwarding a floods).
4. CPU port: `PORT_MAC_CTL` (reg 1) = 0xe002 (RGMII RX/TX delay ve switchi, 1000). Hodnota je v DT jako `marvell,cpu-port-mac-ctl`.
5. PHY 0x11–0x13: v BMCR shodí power-down, zapne AN, FD a 1000 a pošle reset.
6. Monitor (`CONFIG_PHY_MONITOR_PERIOD`) čte `PORT_STS` bit 11 uživatelských portů. Carrier MAC je „up“, když má link aspoň jeden port. Rychlost hlásí vždy 1000/full, protože to je CPU link.

`phy_read`/`phy_write` sahají na interní registry switche (`reg_addr[12:8]` = zařízení, `[4:0]` = registr), např. `phy_read(switch, 0x1000)` je `PORT_STS` portu 0. MDIO shell (`CONFIG_MDIO_SHELL`) vidí jen registry CMD/DATA na adrese 1.

### 3.3 Pořadí inicializace

- `phy-handle` má v `ethernet-controller.yaml` `dependency-mode: ignore`, takže pořadí určují DT ordinály: eth0 (14), pak mdio (20), pak switch (21). Eth0 v `init()` na PHY nesahá.
- MDIO si samo zapne hodiny GbE0 a SMI piny.
- PHY callback a RX vlákno se registrují až v `iface_init`, kdy už jsou switch i MDIO připravené.

## 4. Nejistoty a otevřené body

1. **CPU port switche.**
   - Hodnota 0xe002 je převzatá z U-Boot. Link na portu 0 se v ní nevynucuje (ForcedLink = 0), to Linux DSA dělá. Pokud bude RGMII link mrtvý, zkus `marvell,cpu-port-mac-ctl = <0xe03e>` (force link, FD, 1000).
   - Ověření: dump `PORT_STS` portu 0 přes `phy_read(switch, 0x1000)`.
2. **RGMII delaye.** Delay dělá switch a NETA žádné nemá. Kdyby se rámce na RX ztrácely s CRC chybou (`INTR_MISC_CAUSE` bit 9), zkus u switche jen jeden směr (bit 15 nebo 14).
3. **Časová koalescence RX.** `RXQ_TIME_COAL` = 0. Při prahu 1 rámec by na tom nemělo záležet. Kdyby RX IRQ nechodily, pojistkou je 100ms polling (viditelné jako latence ~100 ms u pingu).
4. **SMP a per-CPU registry.**
   - `CPU_MAP` je zapsaný pro obě CPU (všechny fronty). Na A3700 jsou cause registry viditelné ze všech CPU (Linux `neta_armada3700`).
   - IRQ jde přes GIC SPI 42 na CPU, které má Zephyr nastavené.
5. **MAC adresa.** Zatím jen náhodná nebo z DT. Čtení z prostředí stock U-Boot v SPI NOR (u `mvebu_espressobin` defconfigu je `CONFIG_ENV_OFFSET` na konci 4MiB flash, ověřit) by šlo doplnit přes flash driver, ten ale ještě není.
6. **Do budoucna:**
   - DSA s Marvell tagy (rozhraní na každý port),
   - eth1 přes SGMII (COMPHY), SPI NOR, I2C, GPIO, SDHCI, USB, PCIe,
   - Zephyr na CM3 by znamenal reimplementovat WTMI mailbox pro TF-A (reset, DVFS).

## 5. Artefakty

- `zephyr-espressobin.bundle`: Zephyr `espressobin` (8 commitů nad `124121fbb`) + `gh-pages`
- `flash-image.bin`, `uart-images.tgz.bin`: TF-A + WTMI + Zephyr `dhcpv4_client` (SMP), `CPU_800_DDR_800`, `DDR_TOPOLOGY=2`, SPI NOR

```sh
git -C zephyr fetch zephyr-espressobin.bundle espressobin:espressobin gh-pages:gh-pages
```

Očekávaný výstup na konzoli (115200 8N1) po bootu:

```
*** Booting Zephyr OS ... ***
<inf> phy_mv88e6xxx: switch product 0x341 rev 0 at SMI address 1
<inf> eth_mvneta: ethernet@d0030000: MAC 02:...
<inf> eth_mvneta: ethernet@d0030000: link up
<inf> net_dhcpv4_client_sample: Address[1]: 192.168.x.y
```

## Zdroje

- Linux `drivers/net/ethernet/marvell/mvneta.c`, `drivers/net/dsa/mv88e6xxx/`, `arch/arm64/boot/dts/marvell/armada-37xx.dtsi`, `armada-3720-espressobin.dtsi`, `drivers/pinctrl/mvebu/pinctrl-armada-37xx.c`, `drivers/clk/mvebu/armada-37xx-periph.c`, `drivers/tty/serial/mvebu-uart.c`
- U-Boot `drivers/net/mvneta.c`, `drivers/net/mvmdio.c`, `board/Marvell/mvebu_armada-37xx/board.c`
- TF-A `docs/plat/marvell/armada/build.rst`, `uart-booting.rst`
- [A3700-utils-marvell](https://github.com/MarvellEmbeddedProcessors/A3700-utils-marvell), [mv-ddr-marvell](https://github.com/MarvellEmbeddedProcessors/mv-ddr-marvell), [mox-imager](https://gitlab.nic.cz/turris/mox-imager)
