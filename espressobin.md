# Zephyr na ESPRESSObin (Marvell Armada 3720): TF-A BL33 + síť přes 88E6341

Stav k 2026-10-20. Zephyr `main` @ `124121fbb`, větev `espressobin` (53 commitů). TF-A `master` @ `0892ad4`, mv-ddr @ `7bcb9dc`, A3700-utils @ `a3e1c67`.

## TL;DR

- Boot je celý otevřený a obejde se bez U-Boot: boot ROM → TIM → WTMI na CM3 (mv-ddr trénuje DDR) → TF-A BL1 → BL2 → BL31 → **Zephyr jako BL33** na EL2 → EL1 NS.
- Targety jsou `espressobin` (1× A53) a `espressobin/armada3720/smp` (2× A53, druhé jádro startuje přes PSCI `CPU_ON`).
- Nové drivery:
  - pinctrl NB/SB,
  - periferní hodiny NB/SB a TBG (hradla, čtení frekvencí),
  - UART (`marvell,armada-3700-uart`),
  - MDIO `marvell,orion-mdio`,
  - minimální switch 88E6xxx (přes PHY API),
  - Ethernet NETA (`marvell,armada-3700-neta`).
- NETA má **8 RX a 8 TX front**: TX podle priority paketu se strict priority, každá fronta má token bucket vystavený jako Qav (`ETHERNET_QAV`), RX fronty se vybírají podle VLAN PCP. Viz 3.4.
- Síť: eth0 je připojený přes RGMII-ID s pevným 1000/full na port 0 switche 88E6341 (adresa SMI 1, multi-chip). Porty `wan`, `lan0` a `lan1` posílají rámce jen na CPU a v Zephyru se jeví jako **jedno rozhraní**, zatím bez DSA tagů.
- Ověřeno bez HW:
  - build obou targetů (hello_world, synchronization, dhcpv4_client, echo_server, tests/drivers/ethernet/api), multi-queue konfigurace (8 TX / 4 RX, VLAN, shell) s dhcpv4_client a ethernet/vlan,
  - twister build-only 7/7,
  - checkpatch bez chyb,
  - TF-A `mrvl_flash` + `mrvl_uart` s Zephyrem jako BL33 (`fiptool info` ukazuje BL33 = 0x6B898 B).
- **Ověřeno na HW (2026-10-16, ESPRESSObin v5, 88F3720 1 GHz):**
  - boot ROM → WTMI (mv-ddr, DDR3 1 GiB 2CS) → TF-A v2.15 BL1/BL2/BL31 → Zephyr SMP, druhé jádro naběhne přes PSCI,
  - UART, shell, switch 88E6341 nalezen (produkt 0x341), MAC z DT, 4 RX / 8 TX fronty, link up přes port switche,
  - SPI NOR, `fwupdate ymodem` přes tio (1 MB za ~100 s při 115200, tedy ~10 KB/s), zápis a verifikace, `kernel reboot cold` přes PSCI do nového image.
  - **síť: DHCP přes `lan1` (port 3 switche) → 192.168.200.167**, RX i TX přes NETA a 88E6341.
- Opravy z prvního testu na HW:
  - TX fronty zapnuté ve `start()` při vynuceném link down nikdy nevysílaly (descriptory pending, TX DMA nečinné). Fronty se teď zapínají v link callbacku, jako Linux `mvneta_port_up()`.
  - CPU port switche s `0xe002` z U-Bootu naběhl 1000/half. Výchozí hodnota je teď `0xe03e` (vynucený link, full, 1000).
  - `TXQ_CMD1` má po resetu hodnotu `0x09` a mimo Qav ho necháváme být.
  - ping na bránu 0,08–0,10 ms (RX přes přerušení), UDP TX ~820 Mb/s po drátu (MIB `good_octets_tx` odpovídá zperf),
  - **Qav/limiter po kalibraci:** nastaveno 5/10/20/50 Mb/s, naměřeno 5,01/10,07/20,11/50,22 Mb/s (L2 rámce včetně FCS, bez preambule a IFG), chyba ≤ 0,7 %.
- Zbývá ověřit: pořadí priorit HW TX front (test dvou streamů), RX propustnost (`zperf udp download`).

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

### 3.4 Fronty, priority a Qav

Počet front se řídí traffic classes stacku (`CONFIG_ETH_MVNETA_TX_QUEUES` má default `NET_TC_TX_COUNT`, RX obdobně):

```
CONFIG_NET_TC_TX_COUNT=8
CONFIG_NET_TC_RX_COUNT=4
CONFIG_NET_VLAN=y
CONFIG_NET_L2_ETHERNET_MGMT=y
CONFIG_ETH_MVNETA_SHELL=y
```

- **TX výběr fronty:** při stejném počtu front a TX tříd je fronta N = traffic class N (`net_tx_priority2tc()`), jinak lineárně podle 802.1Q pořadí priorit (BK < BE < EE < … < NC).
  - Výchozí `CONFIG_NET_TC_MAPPING_STRICT`: BE → fronta 1, CA (PCP 3) → fronta 3.
  - Pro AVB je lepší `CONFIG_NET_TC_MAPPING_SR_CLASS_A_AND_B`: SR třída A (PCP 3) jde do nejvyšší fronty 7, třída B (PCP 2) do 6. Rezervace třídy A 20 Mb/s pak vypadá jako `mvneta qav 1 7 20000000`.
- **TX arbitráž:** `TXQ_CMD_1` = 0 vypíná legacy WRR, fronty jdou ve strict priority. Každá fronta má vlastní ring a mutex. Plná fronta v `send()` spí po 20 µs (max `CONFIG_ETH_MVNETA_TX_TIMEOUT_MS`), protože omezená fronta může být plná legitimně.
- **RX:** `VLAN_PRIO_TO_RXQ` (0x2440) mapuje PCP na frontu (přes `net_vlan2priority()` a `net_rx_priority2tc()`), netagované rámce jdou do fronty 0. Vlákno bere vyšší fronty dřív, po dávkách `CONFIG_ETH_MVNETA_RX_BUDGET` (16) a po každé dávce z nižší fronty začne znovu odshora. Prioritu paketu nastaví z PCP ještě před `net_recv_data()`, takže paket skončí ve správné RX třídě stacku.
- **Qav přes token bucket** (registry podle Linux `mvneta_setup_queue_rates()`):
  - `REFILL_NUM_CLK` (0x3e08) = takty jádra GbE na 100 ns. Frekvenci jádra dává nový `get_rate()` v clock driveru: TBG → mux → děliče → `gbe0_core`.
  - `TXQ_BUCKET_REFILL(q)` (0x3e20) = `ceil(idle_slope / 10 kb/s) | 1000 << 20`. Rozlišení je 10 kb/s, rozsah do ~5,2 Gb/s.
  - `TXQ_TOKEN_SIZE(q)` (0x3e40) = jeden maximální rámec (stejné jednotky jako `TX_MTU` = velikost × 8), takže burst je jeden rámec. Neomezené fronty mají refill a bucket na maximu.
  - `TXQ_CMD1` (0x3e00): obě bity `BW_LIM_EN | BW_LIM_SEL_V1` = 0 zapne per-queue limiter (Linux „version 3“). Pokud Qav nemá zapnutá žádná fronta, bity se zase nastaví.
  - Podporuje `STATUS`, `IDLE_SLOPE`, `DELTA_BANDWIDTH` (procento aktuální rychlosti linku), `OPER_IDLE_SLOPE` (zaokrouhleno na 10 kb/s) a `TRAFFIC_CLASS` (jen při 1:1 frontách a třídách). Dále `ETHERNET_CONFIG_TYPE_PRIORITY_QUEUES_NUM`.
  - Je to omezovač rychlosti, ne credit-based shaper (kredit nejde do mínusu, chybí send slope). Pro AVB talkera to ale dělá podstatné: drží stream na rezervaci s burstem jednoho rámce a strict priority ho pouští před best effort.
- **Ladění:** `mvneta regs <iface>` (MAC, fronty, limiter, LPI, TX descriptory, MIB) a `mvneta sw <iface> [addr reg]` (porty switche).
- **Shell:** `mvneta queues <iface>` vypíše stav Qav po frontách, `mvneta qav <iface> <queue> <b/s|off>` ho nastaví přes `NET_REQUEST_ETHERNET_SET_QAV_PARAM`. Jde tedy stejnou cestou jako aplikace. `net iface` ukáže schopnosti „Priority queues“ a „IEEE 802.1Qav“.

Paměť: DMA ringy a buffery pro 4 RX × 32 a 8 TX × 16 descriptorů zabírají 392 KiB nocache.

### 3.5 SPI NOR a `fwupdate ymodem`

- `spi_mvebu_a3700.c`: řadič v legacy režimu (bez FIFO, 1 bajt na přenos, polling), nativní CS i `cs-gpios`. Prescaler počítá z hodin `sqf` (NB index 7).
- Deska: W25Q32DW jako `jedec,spi-nor` (`ef 60 16`, 4 MiB, 50 MHz). Oddíly: `firmware` 0–0x3F0000 (`boot_partition`), `u-boot-env` 0x3F0000–0x400000.
- `subsys/fwupdate`: `fwupdate ymodem [dry]` přepne shell UART do bypass a obraz přijme YMODEMem (CRC-16, bloky 128/1024 B) do RAM.
  - Kontroly: ohlášená velikost a TIM header (`TIMH` na offsetu 4).
  - Zápis: nejdřív smaže první sektor, pak zbytek, zapíše data, header až nakonec a vše přečte zpátky.
  - Při výpadku napájení uprostřed zápisu boot ROM nenajde header a přejde do UART recovery.
  - Na hostiteli ověřeno proti lrzsz `sb` (1K i 128B bloky, s vloženými chybami na lince) a tio 2.7 přes pty.
- `fwupdate tcp [port] [dry]`: totéž přes TCP (`nc -N <ip> 6969 < flash-image.bin`), odpověď s velikostí a crc32 přijde zpátky po spojení.
- Layout stránky SPI NOR je na desce 4 KiB (`SPI_NOR_FLASH_LAYOUT_PAGE_SIZE`). S výchozími 64 KiB vyjde NVS sektor na 65536 B, nevejde se do 16 bitů a `settings_backend_init()` vrátí `-EDOM` (-33). Mazání velkých rozsahů dál používá 64KiB bloky.
- `samples/drivers/spi_flash` na ESPRESSObinu nespouštět: testuje na offsetu 0xFF000, tedy uvnitř firmware oddílu.

### 3.6 Shell přes SSH (`samples/boards/globalscale/espressobin/netshell`)

Pro desku bez konzole: DHCP, mDNS hostname `espressobin<mac>`, Zephyr SSH server s autorizací klíčem (RSA, PEM vložený při buildu z `-DNETSHELL_AUTHORIZED_KEY=...`) a `fwupdate tcp`.

- Settings v NVS v oddílu `u-boot-env` (`zephyr,settings-partition`). Když je v oddílu nečitelný obsah (starý U-Boot env), sample ho jednou smaže.
- Host klíč (RSA 2048) se při prvním bootu vygeneruje a uloží (`ssh/keys/id_rsa`), náhodná locally-administered MAC se uloží jako `netshell/mac`. DHCP pak dává pořád stejnou IP.
- Ověřeno na desce: SSH přihlášení klíčem, MAC, IP i host klíč přežijí reboot.
- Entropie je zatím z `TEST_RANDOM_GENERATOR` (není kryptograficky bezpečná). Chybí driver TRNG Armada 3700.

```sh
ssh -i ~/.ssh/id_rsa root@espressobin<mac>.local
```

### 3.7 DSA (snippet `espressobin-dsa`)

```sh
west build -b espressobin/armada3720/smp -S espressobin-dsa samples/boards/globalscale/espressobin/netshell
```

- `eth0` je conduit: promiskuitní, bez IP. `swp0` je CPU port (bez IP), `swp1`–`swp3` jsou porty switche 1–3. Popisky se liší podle verze desky: v5 má port 1 `wan`, 2 `lan0`, 3 `lan1`; v7 má port 1 `lan1`, 2 `lan0`, 3 `wan` (stejně jako v Linuxu).
- Driver `marvell,mv88e6085` zůstává PHY conduitu. V DSA režimu přepne CPU port do frame mode DSA nebo EDSA, nastaví u EDSA EtherType `0xdada`, rámce do 2048 B na všech portech a číslo zařízení 0. Carrier conduitu pak sleduje link CPU portu.
- PHY uživatelských portů obsluhuje generický `phy_mii` na interní MDIO switche (`marvell,mv88e6xxx-mdio`, Global 2 SMI PHY, adresy 0x11–0x13).
- Tagger `dsa_tag_mv88e6xxx.c` vychází z Linuxu `tag_dsa.c`. Na TX posílá FROM_CPU na port rozhraní, na RX bere TO_CPU a FORWARD (FRAME2REG a trunky zahazuje). 802.1Q hlavičku skládá do tagu a zpět. Test je v `tests/net/dsa_tag_mv88e6xxx` (native_sim/native/64, 12 případů).
- Snippet používá 4bajtový DSA tag. Pro EDSA nastav `dsa-tag-protocol = <DSA_TAG_PROTO_EDSA>` a `CONFIG_DSA_TAG_SIZE=8`.
- mvneta: přidán promiskuitní režim (`UNI_PROMISC_MODE` + celá unicast tabulka) a do maximální délky RX rámce se počítá `CONFIG_DSA_TAG_SIZE`.
- netshell spouští DHCP na každém uživatelském portu a každé rozhraní má vlastní stabilní MAC v settings (`netshell/mac<idx>`). Conduit si drží MAC kvůli unikátnímu hostname.
- Ověřeno na desce: rozhraní na každém portu, DHCP i SSH fungují.

### 3.8 PTP (gPTP přes switch)

```sh
west build -b espressobin/armada3720/smp -S espressobin-dsa samples/boards/globalscale/espressobin/netshell -- -DEXTRA_CONF_FILE=gptp.conf
```

- 88E6341 (podle Linuxu `mv88e6341_ops`: `ptp_ops = mv88e6352`, `avb_ops = mv88e6390`): TAI je volně běžící 32bitový čítač, perioda v `TAI_CLOCK_PERIOD` (8 ns → přetečení za 34 s). Časové značky se ukládají per port: příjem do ARR0/ARR1, odeslání do DEP, vždy se sequence ID. Přístup jde přes Global 2 AVB 0x16/0x17 v layoutu 6390 (port 0x1e = TAI, 0x1f = PTP global).
- `marvell,mv88e6xxx-ptp` (`drivers/ptp_clock/ptp_clock_mv88e6xxx.c`) je softwarový timecounter nad TAI jako v Linuxu: násobitel 32.32, update každé 4 s, `rate_adjust` = absolutní poměr vůči nominálu (tak ho volá adaptér `precision_clock` z gPTP). Hardware se při nastavování nemění.
- Globální PTP nastavení: EtherType 0x88F7, časové značky pro všechny event zprávy, Pdelay_Resp do ARR1. Na 6341 jsou značky z MAC místo PHY (`PTP_CFG` 0x07 = 0x8080, Linux uvádí, že značky z PHY jsou nespolehlivé).
- Porty s `ptp-clock`: `PORT_PTP_CFG0` = bez kontroly transportSpecific. RX: tagger u event zpráv (typy 0–3) zavolá hook switche, ten přečte ARR0/ARR1, ověří seq a nastaví `pkt->timestamp`. TX: `port_txtstamp` si paket podrží a DEP se polluje po 1 ms až 20 ms, pak `net_if_call_timestamp_cb()`. Na každém portu může čekat na značku jen jeden rámec.
- Testy: `tests/drivers/ptp_clock/mv88e6xxx_tc` (timecounter: přetečení, poměr, kroky) a hook v `tests/net/dsa_tag_mv88e6xxx`.
- Protistrana: Linux s `ptp4l -f gPTP.cfg -i <iface> -m` (linuxptp, profil 802.1AS) a NIC s HW časovými značkami.
- Na hardwaru zatím neověřeno.

### 3.9 Qav na portech switche

- Registry nejsou veřejně zdokumentované. Postup převzat z linuxových patchů pro CBS offload (Cedric Jehasse, v7, 2026-06; Luke Howard, MQPRIO/CBS, 2026-05, strom `PADL/linux` větev `rpi-6.18.y-xebros-rmu`). V upstream Linuxu zatím nejsou.
- Přístup přes Global 2 AVB 0x16/0x17, blok v bitech 7:5 (0 PTP, 1 AVB, 2 Qav, 3 Qvb). Port registry Qav: `RATE(q)` = 2q, `HILIMIT(q)` = 2q+1. Globální registry bloku jsou na portu 0x1f (layout 6390) nebo 0xf (6352).
- 88E6341 (`mv88e6341_qav_info`): rate po 64 kbit/s (16 bitů), hi limit 14 bitů, 4 fronty. Plánování front je stejné jako u 6352: port reg 0x0a bity 13:12, 3 = všechny fronty strict, 0 = WRR 8:4:2:1. 88E6352: rate po 32 kbit/s (15 bitů) a navíc nezdokumentovaný enable bit 15 v Qav port reg 0x08 (jen u Luka Howarda).
- Zephyr: `swpN` hlásí 4 prioritní fronty a `ETHERNET_QAV`. Qav požadavky (status, idle slope, delta bandwidth) se přes `set_config` převádějí na rate a hi limit. Hi limit = 1542 B × idle slope / rychlost linky (jeden maximální rušivý rámec), přepočítá se při změně linky. Shaped port přejde na strict, po vypnutí se vrátí původní plánování.
- Priorita → fronta: tagger zapisuje `net_pkt_priority` do PRI v tagu (netagované FROM_CPU rámce). Global 1 IEEE-PRI (0x18) se nastaví podle mapování Zephyru pro 4 třídy, takže fronta N = traffic class N. Při výchozím strict mapování sdílí SR třídy A i B frontu 1. `CONFIG_NET_TC_MAPPING_SR_CLASS_A_AND_B` (vyžaduje TX/RX TC ≥ 2) pošle třídu A do fronty 3 a B do fronty 2.
- Shell: `mvneta qav <iface> <q> <b/s|off>` (obecný, přes net_mgmt), `mv88e6xxx qav [port]` (dump rate/hilimit, AVB port cfg, plánování, IEEE-PRI), `mv88e6xxx avb <blok> <port|g|tai> <addr> [val]` (libovolný registr bloků, 4 slova).
- Bez AVB režimů portu (`PORT_AVB_CFG` 15:14), bez rezervací v ATU (MAC_AVB) a bez izochronních pointerů. Podle Cedrikova patche to pro shaping nestačí. Na hardwaru neověřeno; test: `zperf` UDP s prioritou do shaped fronty, rychlost se má zastavit na idle slope.
- Poznámka k `PADL/linux`: `mv88e6xxx_avb_enable()` nemá `return 0` před chybovými labely, takže po úspěchu všechno zase vrátí.

### 3.10 Router a HW bridge (netshell `router.conf`)

```sh
west build -b espressobin/armada3720/smp -S espressobin-dsa -S espressobin-dsa-v7 \
  samples/boards/globalscale/espressobin/netshell -- -DEXTRA_CONF_FILE=router.conf
```

- Názvy portů: snippety `espressobin-dsa-v5` a `espressobin-dsa-v7` nastaví DT `label` portů (jako Linux `armada-3720-espressobin.dtsi` a `-v7.dts`). DSA core pak pojmenuje rozhraní podle labelu, jinak zůstává `swpN`.
- `wan` dostane DHCP. `lan0` a `lan1` jdou do `bridge0` s 192.168.10.1/24, na kterém běží DHCP server (pool od .100, 32 adres, router option = bridge). IPv4 forwarding je bez NAT, upstream proto potřebuje routu na 192.168.10.0/24 přes WAN adresu desky. MAC bridge je stabilní v settings (`netshell/brmac`).
- Offload: do DSA API přibyly `port_bridge_join`/`port_bridge_leave`. Zephyr bridge mezi porty, mezi kterými už forwarduje switch, nic nepřeposílá (flood ani FDB) a nevznikají duplikáty.
- 88E6341: přemostěné porty mají společný port based VLAN map (0x06 bity 10:0) a FID 1, standalone porty FID 0, jako Linux bez VLAN filteringu. MAC bridge je statický záznam v ATU na CPU portu, protože rámce FROM_CPU se neučí (ATU FID je v G1 reg 0x01, Luke Howard 2026-07). Při změně členství se smažou naučené adresy.
- Rezervovaný multicast 01:80:C2:00:00:0x a 2x (STP, LLDP, 802.1AS) se v DSA režimu trapuje na CPU (G1 0x1a pointery 0x00–0x03 = 0xff, CPU_DEST a PTP_CPU_DEST = port 0 s mgmt prioritou). Mezi přemostěnými porty se neflooduje.
- Test `test_bridge_offload` v `tests/net/dsa_tag_mv88e6xxx`, upstream `tests/net/bridge` dál prochází.
- DNS: Zephyr nemá forwarder. DHCP server rozdává DNS server z leasu na `wan` (option callback na option 6, nové API `net_dhcpv4_server_set_dns_address()` s testem v `tests/net/dhcpv4/server`).
- Forwarding přes výchozí router: DHCP klient v Zephyru nastaví jen bránu rozhraní a výchozí router nepřidá, takže forwarding z LAN ven nenašel cestu. Netshell router přidává router z option 3 leasu na `wan`. Navíc `ipv4_route_packet()` při routování přes výchozí router nechával paketu vstupní rozhraní. Opraveno, test `test_route_ipv4_forward_via_default_router` v `tests/net/route/ipv4` bez opravy padá.
- Limity: mDNS si na každém rozhraní registruje UDP connection handler. Výchozí `CONFIG_NET_MAX_CONN=4` tak s DSA a DHCP klientem dojde a DHCP server pak selže při `bind` s `-ENOENT` (SSH stejně). Netshell má 16, router 24 (+ 32 kontextů a fd).
- Ověřeno na desce (v5): rozhraní a názvy portů, bridge0, DHCP server, DNS z leasu na `wan` a forwarding z LAN ven (ping na 8.8.8.8, `apt update` ze Zero 3E za deskou).

### 3.11 TCAM (88E6341)

- Podle product briefu má Topaz TCAM pro ACL a PCL. Registry nejsou veřejně popsané.
- Čtení na desce potvrdilo layout jako v Linuxu `tcam.c` pro 6390/6393: SMI 0x1f, op registr (busy 15, 1 flush all, 2 flush, 3 load, 4 get next, 5 read; page v bitech 11:10, entry 7:0). Flushnutá entry má keys1 = `0x00ff` (data bez masky, nikdy nematchne). Port reg 0x0d bity 1:0 jsou po resetu 0 (TCAM vypnutý).
- Klíč má 48 B (page 0: keys1/keys2 se SPV a frame type, bajty 0–21 v reg 6–27; page 1: bajty 22–47 v reg 2–27). Registry jsou ve tvaru maska << 8 | data. Akce je na page 2: ACT3 (reg 4, bit 11 DPV valid, DPV 10:0) a ACT5 (reg 6, režim DPV 15:14: AND/OR/replace).
- Klíč má vždy 4B slot pro tag: EtherType na 16, IPv4 od 18 (protokol 27, src 30, dst 34, L4 porty 38/40 bez IP options).
- Implementace: `phy_marvell_mv88e6xxx_tcam.c`. Tabulka pravidel se řadí podle priority (nejnižší index vyhrává) a přepisuje se celá. Na portech s pravidly se zapne 48B režim. Akce: permit (výjimka), drop, trap na CPU, redirect a mirror. Kódování registrů má test v `tests/drivers/ethernet/mv88e6xxx_tcam`.
- Shell: `mv88e6xxx tcam add <id> <prio> <porty> <akce> [match…]`, `del`, `list`, `flush`, `dump <entry>`.
- Zatím neznámé: akce pro QoS (QPri/FPri override) a policing na page 2. TCAM čítače jsou v MIB bank 1 (0x08–0x0b).
- Ověřeno na desce: drop ICMP na lan0+lan1 v 48B režimu (port 0x0d = 1) zastaví ping ze Zero 3E za deskou. Výjimka `permit proto icmp dst 192.168.10.1` s nižší prioritou funguje (ping na desku projde, na 8.8.8.8 ne), takže pořadí podle priority i match na IPv4 dst (offset 34) sedí.

## 4. Nejistoty a otevřené body

1. **CPU port switche.** Vyřešeno: `0xe03e` dává 1000/full, ověřeno přes `mvneta sw`.
2. **RGMII delaye.** Delay dělá switch a NETA žádné nemá. Kdyby se rámce na RX ztrácely s CRC chybou (`INTR_MISC_CAUSE` bit 9), zkus u switche jen jeden směr (bit 15 nebo 14).
3. **Časová koalescence RX.** `RXQ_TIME_COAL` = 0. Při prahu 1 rámec by na tom nemělo záležet. Kdyby RX IRQ nechodily, pojistkou je 100ms polling (viditelné jako latence ~100 ms u pingu).
4. **SMP a per-CPU registry.**
   - `CPU_MAP` je zapsaný pro obě CPU (všechny fronty). Na A3700 jsou cause registry viditelné ze všech CPU (Linux `neta_armada3700`).
   - IRQ jde přes GIC SPI 42 na CPU, které má Zephyr nastavené.
5. **MAC adresa.** Zatím jen náhodná nebo z DT. Čtení z prostředí stock U-Boot v SPI NOR (u `mvebu_espressobin` defconfigu je `CONFIG_ENV_OFFSET` na konci 4MiB flash, ověřit) by šlo doplnit přes flash driver, ten ale ještě není.
6. **SPI NOR.** Vyřešeno: čtení, zápis (`fwupdate`) i NVS settings ověřeny na desce.
7. **Pořadí priorit HW TX front.** Veřejně není zdokumentované, který konec arbiter obsluhuje první. Driver předpokládá frontu 7. Test: dva saturující UDP streamy s různou prioritou. Když vyhrává ten nižší, zapni `CONFIG_ETH_MVNETA_TXQ0_HIGHEST=y`.
8. **Jednotky token bucketu.** Vyřešeno měřením: limiter počítá bajty L2 rámce včetně FCS (bez preambule a IFG), 10 kb/s na jednotku refillu.
9. **Frekvence jádra GbE a časovač refillu.** `DIV_SEL1` bit 14 je na desce 0, `gbe0_core` = 250 MHz. Časovač refillu ale běží 2× rychleji: podle Linuxova vzorce limiter pouští dvojnásobek (5 Mb/s → 10,06 Mb/s). Driver proto programuje `REFILL_NUM_CLK` pro 2× takt (`MVNETA_REFILL_CLK_MULT`). Bucket je jeden maximální rámec plus jeden refill, s bucketem na jeden rámec limiter při 40 Mb/s dosáhl jen 85 % rychlosti (ztráta tokenů v plném bucketu, potvrzeno simulací).
10. **Do budoucna:**
   - DSA: TCAM, VLAN-aware bridge, forward-tagged TX do bridge, NAT, PPS/event capture (TAI GPIO), AVB režimy portů a SRP rezervace, Rsvd2CPU pro 01:80:C2:00:00:0x (gPTP, LLDP),
   - eth1 přes SGMII (COMPHY), I2C, GPIO, Xenon SDHCI (microSD/eMMC), USB, PCIe,
   - Zephyr na CM3 by znamenal reimplementovat WTMI mailbox pro TF-A (reset, DVFS).

## 5. Artefakty

- `zephyr-espressobin.bundle`: Zephyr `espressobin` (53 commitů nad `124121fbb`) + `gh-pages`
- `flash-image-router-v7.bin`, `flash-image-router-v5.bin`, `flash-image-router-v5-gptp.bin` (+ gPTP): netshell router (wan DHCP, lan0+lan1 HW bridge, DHCP server, forwarding), `CPU_1000_DDR_800`
- `flash-image-netshell-qav.bin`: netshell + DSA + gPTP + Qav na `swpN`, `CPU_1000_DDR_800`
- `flash-image-netshell.bin`: netshell (SSH + `fwupdate tcp`), `CPU_1000_DDR_800`; `netshell-sd.tar.gz` pro `booti` ze stock U-Bootu
- `flash-image.bin`, `uart-images.tgz.bin`: TF-A + WTMI + Zephyr `dhcpv4_client` (SMP, 8 TX / 4 RX fronty, VLAN, shell `mvneta`), `CPU_800_DDR_800`, `DDR_TOPOLOGY=2`, SPI NOR

```sh
git -C zephyr fetch zephyr-espressobin.bundle espressobin:espressobin gh-pages:gh-pages
```

Očekávaný výstup na konzoli (115200 8N1) po bootu:

```
*** Booting Zephyr OS ... ***
<inf> phy_mv88e6xxx: switch product 0x341 rev 0 at SMI address 1
<inf> eth_mvneta: ethernet@d0030000: MAC 02:..., 4 RX / 8 TX queues
<inf> eth_mvneta: ethernet@d0030000: link up
<inf> net_dhcpv4_client_sample: Address[1]: 192.168.x.y
```

## Zdroje

- Linux `drivers/net/ethernet/marvell/mvneta.c`, `drivers/net/dsa/mv88e6xxx/`, `arch/arm64/boot/dts/marvell/armada-37xx.dtsi`, `armada-3720-espressobin.dtsi`, `drivers/pinctrl/mvebu/pinctrl-armada-37xx.c`, `drivers/clk/mvebu/armada-37xx-periph.c`, `drivers/tty/serial/mvebu-uart.c`
- U-Boot `drivers/net/mvneta.c`, `drivers/net/mvmdio.c`, `board/Marvell/mvebu_armada-37xx/board.c`
- TF-A `docs/plat/marvell/armada/build.rst`, `uart-booting.rst`
- [A3700-utils-marvell](https://github.com/MarvellEmbeddedProcessors/A3700-utils-marvell), [mv-ddr-marvell](https://github.com/MarvellEmbeddedProcessors/mv-ddr-marvell), [mox-imager](https://gitlab.nic.cz/turris/mox-imager)
