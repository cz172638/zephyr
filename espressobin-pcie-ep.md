# Armada 3720 (ESPRESSObin) – PCIe řadič Aardvark v EP módu

Shrnutí k 2026-10-30. Vychází z testů na ESPRESSObinu v5 (Zephyr, driver
`pcie_epc_aardvark`, funkce `epf_test`, shell `advk_ep`) proti hostu
BeagleY-AI (J722S, Linux 7.3.0-arm64-k3-r10, za PCIe switchem ASM1184e,
od 30. 10. ASM1806) a ze zdrojů TF-A, U-Boot a Linuxu. Marvell EP mód
nikde nedokumentuje. Co není označené jako ověřené na HW, je odvozené ze
zdrojů nebo neověřené.

Zephyr: větev `espressobin` (od upstream `124121fbb`), poslední stav
`ffbb4731f`, obrazy `dbg-ep/NN-*.bin` (číslování níže).

## 1. Zdroje

| Zdroj | Co obsahuje | Pro EP |
|---|---|---|
| TF-A `drivers/marvell/comphy/phy-comphy-3700.c` | PCIe power on, lane 1 | PHY je stejná pro RC i EP. Krok 6 zapíná výstup 100/125 MHz na REFCLK (`MISC_CTRL0` bit 4), EP ho musí vypnout. PCIe power off TF-A nemá. |
| TF-A `io_addr_dec.c`, `marvell_plat_config.c` | IO decode okna masterů | PCIe `0xc500`, DMA (XOR) `0xc200`, attr `0x3d`, target DRAM. **Tudy jdou příchozí TLP do DDR i XOR do PCIe.** |
| TF-A `dram_win.c` | CPU decode okna | PCIe `0xe8000000` + 128 MiB (target 2, attr 0). Targety: 0 DRAM, 1 interní registry, 2 PCIe, 3 PCIe přes MCI, 4 BootROM, 5 MCI external, 7 rWTM RAM, 8 CCI-400. |
| U-Boot / Linux `pci-aardvark.c` | jen RC | Mapa registrů, OB okna `0x4c00`, LMI (LTSSM `0x6000`, vendor ID erratum `0x6044`), RC MSI. EP mód ani DMA nikde. |
| ESPRESSObin wiki „PCIe Endpoint“ | recept | Kabel bez REFCLK a napájení, výchozí ID `1b4b:0100`. |

## 2. Inicializace EP (driver, ověřeno)

1. Pinctrl: `PCIE1_RESETn` jako GPIO (výstup řadiče, v EP nesmí budit linku). Clock gate SB bit 14.
2. COMPHY lane 1 v PCIe módu přes TF-A SiP, potom vypnout `MISC_CTRL0` bit 4 (REFCLK výstup).
3. Direct mód (`0x18000` bit 0 = 0), `REF_CLK` `0x4814`, `CTRL0` `IS_RC` = 0, šířka, generace, training vypnutý.
4. Target link speed v `LNKCTL2`. **Gen1 je teď výchozí** (`max-link-speed = <1>` v overlay), viz kap. 9.
5. `CTRL2`, default action MEM, OB okna smazat, přerušení maskovat.
6. BAR config v LMI (kap. 3): BAR2 a ROM vypnout, BAR0/BAR4 nastaví `set_bar`.
7. Okno PCIe IO decode 0 jen na carveout BARů (`memory-region` na `&pcie_ep`), okno DMA jednotky 1 na PCIe (kap. 6).
8. Hlavička (LMI `0x6044` + `0x00`, class `0x08`), MSI MMC, MSI-X (kap. 5), DSN (kap. 7).
9. Start: `CTRL0` bit 6. Autostart až po DSN a uložené hlavičce (priorita 90).

Reference: PHY z krystalu, host vlastních 100 MHz (oddělené reference bez SSC).

## 3. Konfigurační prostor a BARy

- **Config space = registry řadiče `0x000–0xfff`.** Od `0x1000` díra. Capability: PM `0x80` → MSI `0x90` → MSI-X `0xb0` → PCIe `0xc0`. Rozšířené: AER `0x100` → DSN `0x150` → L1SS `0x240` → LTR `0x2a8` → Secondary PCIe `0x300`. Resizable BAR ani VSEC není.
- **Z hostu jsou DSN i ostatní RO registry opravdu jen pro čtení** (`setpci ECAP_DSN` nic nezmění).

### BAR config v LMI (nalezeno `advk_ep barhunt`, 2026-10-30)

`0x6240`, 7 bitů na BAR: BAR0 od bitu 0, BAR2 od bitu 7, BAR4 od bitu 19.

| bity pole | význam |
|---|---|
| [4:0] | aperture, velikost = 2^(aperture + 7) |
| [5] | enable |
| [6] | prefetchable |

| `0x6240` | význam |
|---|---|
| [18:16] | neznámé (u BAR0/2/4 obří velikosti, nejspíš horní bit aperture) |
| [30:26] | ROM aperture |
| [31] | ROM enable |

| `0x6244` | význam |
|---|---|
| bity 16/17/18 | 64bit BAR0/2/4 |
| bity 26/28/30 | I/O BAR0/2/4 |
| bit 12 | BAR3 (32bit, 128K) |

- Reset: `0x6240 = a958152a` (BAR0 64bit 128K, BAR2 I/O 128K, BAR4 64bit 256K, ROM 128K), `0x6244 = 105d050a`.
- **Driver od 6e7c35084:** BAR0/BAR4 64bit paměťové, velikost z `set_bar` (128 B – 2 GiB), BAR2 i ROM vypnuté. epf_test: BAR0 8K, BAR4 16 MiB v carveoutu 32 MiB na `0x10000000`. Ověřeno: `pcitest -b 0`, `-b 4` OK.
- Host nastavuje okna mostů při bootu: po zvětšení BARu je potřeba restart hosta (nebo `pci=realloc`), `rescan` nestačí.
- **Expansion ROM:** BAR na `0x30` (128K, enable bit 0). Inbound cíl neznámý: přes slot `0x5808` dostal host při čtení `/sys/…/rom` synchronous external abort v `pci_get_rom_size()` (oops). Proto je vypnutá.

### Inbound

- Překlad na BAR: `0x5800/0x5804` BAR0, `0x5808/0x580c` BAR2, `0x5810/0x5814` BAR4 (lo `ffffff86`, hi `ffffff80`). `0x5818` a výš skončí abortem. Bity pod velikostí BARu bere řadič z TLP: základ musí být zarovnaný na velikost BARu.
- Za překladem okno **PCIe IO decode 0** (`0xd000c500`: `ctrl = (size/64K − 1) << 16 | attr << 8 | target << 4 | en`, base na +4). Mimo okno skončí přístup hosta chybou. Driver ho omezuje na carveout.
- Bloky `0x5a00/0x5c00/0x5e00` a položky `+0x20…+0xa0`: inbound okna (PCI adresa, AXI, maska) jako v RC. V EP módu netestováno (kandidát na podrozsahy uvnitř BARu).

## 4. Outbound a Bus Master Enable

- OB okna `0x4c00` (sady `0x4d00/0x4e00/0x4f00`), CPU okno `0xe8000000`. EP čte i zapisuje paměť hosta.
- **J722S nemá před PCIe IOMMU ani firewall:** EP přečetl i BL31 hosta na `0x80000000`.
- **Bez BME v Command registru EP řadič outbound odmítá:** CPU zápis přes OB okno (MSI) = SError a Zephyr stojí, XOR = MBUS chyba. BME vynuluje restart EP pod navázaným driverem hosta. Od obrazu 88 MSI/MSI-X/DMA bez BME vrací `-EACCES` a epf_test nechá příkaz čekat, až host BME zapne (`remove` + `rescan` nebo `setpci COMMAND=0407`).
- Linux `pci_endpoint_test` čeká na IRQ bez timeoutu (proces v D stavu).

## 5. Přerušení

- **MSI:** 16 vektorů, adresa doorbellu ITS `0x01001400` (bus), bez zpětného čtení. Ověřeno.
- **MSI-X (od obrazu 85):** capability `0xb0` je lokálně celá zapisovatelná (ID, next, Table Size, Enable, FMask, offset/BIR, PBA). Tabulka a PBA leží v BARu, tedy v naší DDR. Reset BAR0+0 a PBA +0x10000 (přes registry epf_test). Driver je přesouvá na BAR0+0x40 a PBA +0x140. MSI-X je softwarové: EP přečte záznam (adresa, data, maska) a zapíše přes OB okno, při masce nastaví PBA. Ověřeno: `pcitest -x 1..16` OK.
- INTx: `CTRL0` bity 28–31, neověřeno.

## 6. DMA

- **Aardvark vlastní DMA engine nemá.** `wmap` přes `0x10000–0x1ffff`: zapisovatelný je jen blok `0x19000` (dvě 64bit adresy s výchozí hodnotou na vlastní registry `0xd0089010/14`, masky `0x1900c`, `0x19034`, ID `0x19304–0c`). Je to nejspíš příjem MSI v RC módu. `0x10000–0x17fff` a `0x1a000–0x1ffff` jsou jen nuly.
- **DMA = XOR engine SoC** (`0xd0060900/0b00`, jako Linux `mv_xor`), API `pcie_epc_xfer_dma()`, epf_test s `pcitest -d`.
  - Okno DMA jednotky 1 (`0xd000c210/214`) na PCIe: base `0xe8000000`, 128 MiB, target 2. Bez něj ERR_MBUS (cause `0x202`).
  - Po EOC je stav deskriptoru zapsaný zpět až později (u zápisů k hostu na Gen1 vždy). Driver čeká na write-back.
  - **Otevřené: pořadí MSI za daty.** XOR potvrdí zápisy k hostu dřív, než projdou linkou. Kontrolní čtení CPU (1 µs) ani XOR (5 µs) je nezachytí, protože AXI kanály čtení a zápisu jsou nezávislé. Kandidáti: attr okna `0xd000c210` (bufferable), MSI posílaný přes XOR, nebo čekání podle délky přenosu.

| Gen1 x1, `pcitest`, 1–4 MiB | DMA | CPU memcpy |
|---|---|---|
| host → EP (`-w`) | 147 MB/s | ~1 MB/s (non-posted ready) |
| EP → host (`-r`) | ~231 MB/s celkem (XOR „366“ kvůli posted) | 30 MB/s |
| copy (`-c`) | 211 MB/s | — |

Na Gen2: host → EP 206 MB/s, EP → host ~330 MB/s. CRC32 instrukcemi Armv8 trvá ~350 µs/MiB.

## 7. DSN a hlavička

- **DSN** (`CONFIG_PCIE_EPC_DSN_SETTINGS`): při prvním startu z 64bit Unique ID SPI NOR (příkaz 0x4B, `FLASH_EX_OP_SPI_NOR_READ_UID`) s bitem locally administered, jinak náhodné. Uloží se do NVS (`pcie/dsn0`) a zapíše se před startem linky. Shell `pcie_dsn show|set|uid|new`.
- Z hostu se DSN nastaví přes **schránku v BAR0**: `0x1004` lo, `0x1008` hi a nakonec `0x1000` = `0x44534e21` („DSN!“), například `pcitest -W resource0 0x1004 lo hi 0x44534e21`. Poll uloží změnu do NVS.
- **Hlavička:** `epf_hdr set vid:pid [svid:ssid [class]] | reset`, uložená v NVS (`epf_test/hdr0`). **`104c:b500` (TI DRA74x) je v tabulce `pci_endpoint_test`** (BAR0, zarovnání 4K), driver se naváže bez `new_id`. Ověřeno.
- A3720 nemá dokumentované unikátní ID (OTP čte jen CM3, na MOX ho programuje CZ.NIC). Switch Topaz vlastní unikátní MAC nemá. Tovární MAC desky je `ethaddr` v prostředí U-Bootu (SPI NOR `0x3f0000`).

## 8. Koherence

- `advk_ep coh xor rd|wr` a `coh bar4 rd|wr|show|flush|inval`: vzor jen v cache proti vzoru v DDR, pro XOR i pro přístup hosta přes BAR4. Na HW zatím nespuštěno. Totéž po `cci share N sh`.

## 9. Kvalita linky a AER

- Na Gen2 přes provizorní crossover: RxErr + BadTLP v páru (host → EP), Replay Timer Timeout na obou stranách, občas `link down (recovery-rcvr-lock)`. Odhad **BER ~6·10⁻⁹** (spec ≤ 10⁻¹²). Data díky LCRC a replay vždy v pořádku.
- **Na Gen1 nula chyb**, link padá jen při restartu hosta. Gen1 je tedy výchozí, Gen2 až s lepším propojením (případně REFCLK z hosta).
- **AER na J722S:** služba je zapnutá (IRQ 566), ale zprávy ERR_COR nikdo nezpracuje: Root Error Status zůstává `ERR_COR MultiERR_COR` a `dmesg` nic nehlásí. Zprávy přes switch přitom dorazí (zdroj 05:00.0).
- UR na portech switche = enumerace prázdných slotů (CfgRd na neexistující zařízení), neškodné.
- ECRC: EP umí gen/check (`CapCtl a0`), vypnuté.
- Nástroje: `advk_ep aer [clear|count|stop]` na EP, `aerdump.pl [-a] [-c] [-w s] | -f lspci.txt` na hostu.

## 10. Switch a host

- **ASM1184e:** upstream port bez BARu, VSEC `0x300` délky `0xc00`. Uvnitř stínová hlavička (`0x340`), tabulka portů 1…7 (ASM1184e používá liché), čtyři bloky po `0x80` (ladění PHY a nastavení podle portů). Řadič SPI/EEPROM není vidět.
- **ASM1806** (od 30. 10.): ACS a multicast. EP je `05:00.0` za portem `02:06.0`.
- **J722S:** bus adresa = CPU − `0x600000000`. Okna mostů se nastaví při bootu. Čtení `/sys/…/rom` u nefunkční ROM shodí proces oopsem (bezpečněji `pcitest -M phys` přes `/dev/mem`). Restart EP vyžaduje `remove` + `rescan`.
- EP konfigurační prostor hosta ani switche nevidí (config requesty jdou jen dolů). Topologii musí dodat host.

## 11. Nástroje

- **EP shell `advk_ep`:**
  - stav: `status`, `hist`, `trace`, `aer`, `dmastat`, `barcfg`, `cpuwin`, `iodec`, `bar`,
  - registry: `dump`, `rd`, `wr`, `probe`, `nz`, `map`, `wmap`, `barhunt`,
  - experimenty: `xor`, `copy`, `coh`, `rom`, `ibwin`, `ibscan`,
  - řízení: `up`, `down` (link nedrží dole, MAC PHY znovu zapne), `msi`, `host`, `comphy`, `refclk`.
- Ostatní shell: `pcie_dsn`, `epf_hdr`, `cci`, `devmem`, `fwupdate`.
- **Host:** `pcitest` (+ `-P resource [len]`, `-W resource off val…`, `-M phys [len]`), `aerdump.pl`, `advk_lab.pl` (konzole EP a SSH na hostu, testy, flash přes XMODEM/YMODEM), `crct` (kontrola CRC32 Armv8).

## 12. TODO

- Pořadí MSI za DMA zápisy (kap. 6).
- Koherence XOR a PCIe (kap. 8), podle výsledku cacheable BAR paměť a vypuštění údržby cache.
- Inbound okna `0x5a00…` v EP módu: podrozsahy BARu (vNTB, více memory windows v jednom BARu).
- Expansion ROM: najít inbound cíl (tagy v DDR, čtení přes `-M`).
- `0x6240` bity 16–18 a 0x6244 zbytek, BAR2 jako 32bit paměťový (slot `0x5808`).
- Registr s bus/device number EP (Requester ID).
- Pád `barhunt` v `0x4800–0x48ff`: barhunt bez ochrany proti abortu.
- P2P a multicast přes ASM1806.
- Boot přes PCIe (loader v SPI NOR, posuvné okno BAR4 nebo schránka podle TI `spl_dfu` přes PCIe).
- `advk_ep down` tak, aby link opravdu shodil (reset řadiče).
- `iodec reset` (1 GiB) se nezapíše.

## 13. Obrazy (výběr)

| # | obsah |
|---|---|
| 69–70 | inbound na BAR, carveout `0x10000000`, první úplný `pcitest` |
| 76–77 | `wmap`, `down` s PHY |
| 78 | DMA přes XOR (`pcitest -d`) |
| 80 | `advk_ep aer` |
| 82–84 | DSN (NVS, UID SPI NOR, schránka) |
| 85 | MSI-X |
| 86–88 | časy po fázích, CRC Armv8, BME ochrana a čekání |
| 89–91 | Gen1, `dmastat`, write-back deskriptoru XOR |
| 92–93 | `barhunt`, velikosti BARů přes LMI, BAR2 vypnutý |
| 94–96 | ROM experimenty, ROM vypnutá |
| 97 | `epf_hdr` (vid:pid, sub, class v NVS) |
