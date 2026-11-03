# Aardvark (A3720): config space a LMI, RC vedle EP

Zdroj: příloha `ffee657d-rc.txt`, image 115, stejná deska, jednou `pcie controller mode rc`, jednou `ep`.

- **RC:** link L0 na ASM1184, NVMe za ním.
- **EP:** bez linku, LTSSM visí v Polling.
- **Výpisy:** `pcie debug dump 0x0 0x400`, `pcie debug dump 0x6000 0x400`, `pcie debug dump 0x4800 0x60`, `pcie cfg 0:0.0 0 0x100` (v RC softwarový root port, v EP lokální cfg), `pcie debug wmap 0x0 0x400`.

## 1. Shrnutí

Hardwarový config space je v obou rolích **tatáž type-0 hlavička se stejným řetězcem capabilities** (0x80 PM → 0x90 MSI → 0xb0 MSI-X → 0xc0 PCIe; extended 0x100 AER → 0x150 DSN → 0x240 L1SS → 0x2a8 LTR → 0x300 SecPCIe). Žádná capability v jedné roli nepřibude ani nezmizí. Mění se jen obsah.

Co řídí rozdíly:

| Zdroj | Co v cfg mění |
|---|---|
| `CTRL0 0x4800` bit 2 `IS_RC` | typ portu v PCIe cap (`0xc0`), verzi PCIe cap (v1 → v2), BARy (LMI `0x6240`/`0x6244` versus pevné 128M), zapisovatelnost polí specifických pro root/downstream port (Link Disable/Retrain, AER Root Error Command, Link Control 3, Lane EQ) a pro endpoint (Clock PM) |
| `CTRL0 [1:0]` gen select | Link Cap max speed `0xcc`, Link Cap 2 `0xec`, výchozí Target Link Speed `0xf0` |
| LMI `0x6044` | Vendor ID `0x00`, Subsystem Vendor ID `0x2c` |
| LMI `0x6240`/`0x6244` | jen v EP: velikost, typ a enable BARů 0–5; v obou rolích ROM BAR `0x30` |
| LMI `0x6300` (hypotéza) | BARy v RC módu |
| ovladač přímo v cfg | class, Command, DevCtl, MSI MMC, MSI-X tabulka/PBA, DSN |

Statická část LMI je v obou rolích **stejná**, kromě `0x6044` a `0x6240`. Ostatní rozdíly v LMI jsou živý stav linku.

## 2. Config space 0x000–0x0ff, hardware

`wm` je maska zapisovatelných bitů z `wmap`, `—` znamená, že slovo nejde zapsat.

| off | RC | wm RC | EP | wm EP | rozdíl a příčina |
|---|---|---|---|---|---|
| `0x00` | `010011ab` | `ffff0000` | `010011ab` | `ffff0000` | stejné; Device ID v cfg, Vendor jen z LMI `0x6044[15:0]` |
| `0x04` | `00100004` | `00000547` | `00100000` | `00000547` | v RC BME=1, ovladač ho přeposílá ze softwarového root portu (bez něj blokuje DMA zařízení) |
| `0x08` | `06040000` | `ffffffff` | `ff000000` | `ffffffff` | class píše ovladač: RC bridge `0604`, EP z epf hlavičky |
| `0x0c` | `00000000` | `ff0000ff` | `00000000` | `ff0000ff` | **Header Type 0 i v RC.** Type-1 hlavička (bus numbers, okna) v hardwaru není, proto softwarový root port. |
| `0x10` | `00000000` | `f8000000` | `00000004` | `ffffe000` | BAR0: RC pevně 128M, typ 0; EP podle LMI `0x6240` (8K, 64-bit) |
| `0x14` | `00000000` | `ffffffff` | `00000000` | `ffffffff` | RC: BAR1 celý zapisovatelný (horní půlka 64-bit páru); EP: horní půlka BAR0 |
| `0x18` | `00000000` | `f8000000` | `00000000` | — | BAR2: RC 128M; EP vypnutý (`0x6240` bit 12 = 0) |
| `0x1c` | `00000000` | `ffffffff` | `00000000` | — | RC: BAR3 = horní půlka |
| `0x20` | `00000000` | `f8000000` | `00000004` | `ff000000` | BAR4: RC 128M; EP 16M 64-bit |
| `0x24` | `00000000` | `ffffffff` | `00000000` | `ffffffff` | horní půlka BAR4 v obou |
| `0x2c` | `010011ab` | `ffff0000` | `00000000` | `ffff0000` | SSVID z LMI `0x6044[31:16]` (RC `11ab`, EP `0`); SSID: v RC hardwarový reset `0x0100` (RC ovladač ho nepíše), v EP zapsala epf hlavička `0` |
| `0x30` | `00000000` | `fffe0001` | `00000000` | — | ROM BAR: **LMI platí i v RC** (`0x6240` [30:26]=10 → 128K, bit 31 enable); v EP ovladač ROM vypnul |
| `0x34` | `00000080` | `000000ff` | `00000080` | `000000ff` | cap pointer, stejné |
| `0x3c` | `000001ff` | `000007ff` | `000001ff` | `000007ff` | stejné |
| `0x80` | `5a039001` | `5a07ffff` | `5a039001` | `5a07ffff` | PM cap, stejné |
| `0x84` | `00000008` | `00008103` | `00000008` | `00008103` | PMCSR, stejné |
| `0x90` | `008ab005` | `007fff00` | `0088b005` | `007fff00` | MSI MMC: RC reset 32 vektorů, EP 16 (zapsal ovladač) |
| `0xb0` | `000fc011` | `c7ffffff` | `000fc011` | `c7ffffff` | MSI-X, 16 položek v obou |
| `0xb4` | `00000000` | `ffffffff` | `00000040` | `ffffffff` | MSI-X tabulka: EP BAR0+0x40; RC nenastaveno |
| `0xb8` | `00010000` | `ffffffff` | `00000140` | `ffffffff` | PBA: EP BAR0+0x140; RC hardwarový reset |
| `0xc0` | `00420010` | — | `00010010` | — | **PCIe cap: RC = v2, Root Port; EP = v1, Endpoint.** Nezapisovatelné, řídí to `IS_RC`. |
| `0xc4` | `00008242` | `1ffc8fe7` | `00008242` | `1ffc8fe7` | DevCap stejné (MPS 512) |
| `0xc8` | `0000200f` | `000079ff` | `00002810` | `000079ff` | DevCtl: RC ovladač zapnul hlášení chyb a vypnul RO/NS; EP má výchozí hodnoty (RO, NS, MRRS 512) |
| `0xcc` | `00409c12` | `0043fc00` | `00409c11` | `0047fc00` | max speed z gen select (RC Gen2, EP Gen1); **bit 18 Clock PM zapisovatelný jen v EP** |
| `0xd0` | `00110000` | `100000db` | `00110000` | `100001cb` | LnkCtl: RC zapíše bit 4 Link Disable (a Retrain bit 5, ten se sám nuluje), EP bit 8 Clock PM Enable. LnkSta v RC: **2.5 GT/s x1, i když obě strany umí Gen2** (viz kap. 5) |
| `0xe4` | `00000812` | `0000081f` | `00000812` | `0000081f` | DevCap2 stejné |
| `0xe8` | `00000000` | `0000041f` | `00000000` | `0000041f` | DevCtl2 stejné |
| `0xec` | `00000006` | — | `00000002` | — | LinkCap2: RC 2.5+5, EP 2.5 (gen select) |
| `0xf0` | `00010002` | `0000ffff` | `00010001` | `0020ffff` | Target Link Speed: RC zapisuje ovladač (2), EP podle gen select (1); EP navíc bit 21 (RW1C v LnkSta2) |

## 3. Extended capabilities 0x100–0x3ff

Hodnoty jsou v obou rolích stejné, kromě `0x30c`. Rozdíly jsou v zapisovatelnosti:

| off | RC | wm RC | EP | wm EP | poznámka |
|---|---|---|---|---|---|
| `0x100` AER hdr | `15020001` | `ffff0000` | stejné | stejné | next pointer zapisovatelný |
| `0x108`/`0x10c` | `0`/`00462030` | `005ff010` | stejné | stejné | Uncorr Mask/Severity |
| `0x114`/`0x118` | `0000e000`/`000000a0` | `0000f1c1`/`000001e0` | stejné | stejné | Corr Mask, AER Cap/Ctl |
| `0x12c` | `00000000` | `00000007` | `00000000` | — | **AER Root Error Command jen v RC.** Hardware má plný AER root portu: Root Error Status `0x130` a Source ID `0x134` jdou použít pro zachytávání chyb z downstreamu. |
| `0x150`–`0x158` DSN | `24010003 0754392c de64ccf7` | `ffff0000 ffffffff ffffffff` | stejné | stejné | DSN zapisovatelné v obou; hodnota stejná (image 115 měl společný klíč) |
| `0x240`–`0x24c` L1SS | stejné | stejné | stejné | stejné | |
| `0x2a8`/`0x2ac` LTR | stejné | stejné | stejné | stejné | |
| `0x304` | `00000000` | `00000001` | `00000000` | — | Link Control 3 (Perform Equalization) jen v RC |
| `0x30c` | `00000000` | `00007f7f` | `00007f7f` | — | Lane 0 EQ Control: v RC nastavitelné presety a hinty (downstream port je posílá), v EP jen ke čtení `7f7f` |
| `0x32c` | `00000000` | — | `00000000` | `00001fff` | jen v EP, za koncem SecPCIe struktury pro x1 (lane EQ by končilo na `0x30e`). Neznámý registr. |

## 4. LMI 0x6000–0x63ff a řídicí blok 0x4800

### LMI

| LMI | RC | EP | význam |
|---|---|---|---|
| `0x6000` | `1000ff21` | `03000000` | CFG_REG, živý stav. LTSSM `[29:24]`: RC `0x10` = L0, EP `0x03` = Polling. Dřívější EP s linkem: `1007ff09`. Bity `[23:0]` jsou nejspíš stav linku, význam neznámý. |
| `0x6014`, `0x6018` | `05e2607e`, `0f0071de` | `0`, `0` | živé čítače, při linku nenulové |
| `0x6028`–`0x6034` | `5dc..`, `5ed`, `b8`, `621`, `8b9` | `0` | živé čítače |
| `0x6044` | `11ab11ab` | `000011ab` | Vendor + SSVID. RC ovladač zapisuje obě půlky (Linux erratum), EP ovladač horní půlku nechává 0. |
| `0x6240` | `a958152a` | `29880526` | BAR cfg 0. RC: hardwarový reset, ovladač na něj nesahá; v RC módu se pro BARy stejně nepoužívá, jen pro ROM. EP: BAR0 8K, BAR2 off, BAR4 16M, ROM off. |
| `0x6244` | `105d050a` | `105d050a` | BAR cfg 1, stejné (BAR0/4 64-bit, BAR2 I/O) |
| `0x6300` | `00151515` | `00151515` | stejné, viz níže |
| ostatní | stejné | stejné | `0x6004`–`0x6010`, `0x601c`–`0x6024`, `0x6038`–`0x603c`, `0x604c`–`0x6054`, `0x6208`, `0x6210`, `0x6224`, `0x62ac` |

Rozpad `0x6240` (pole podle barhunt/cfghunt; velikost = 2^(aperture+7)):

| pole | bity | RC (reset) | EP |
|---|---|---|---|
| BAR0 aperture / enable / pref | `[4:0]` / 5 / 6 | 10 → 128K / 1 / 0 | 6 → 8K / 1 / 0 |
| BAR2 aperture / enable | `[11:7]` / 12 | 10 → 128K / 1 | 10 / 0 |
| BAR4 aperture / enable / pref | `[23:19]` / 24 / 25 | 11 → 256K / 1 / 0 | 17 → 16M / 1 / 0 |
| ROM aperture / enable | `[30:26]` / 31 | 10 → 128K / 1 | 10 / 0 |

**Hypotéza k `0x6300` = RC BAR config.** Tři bajty `0x15` odpovídají třem 64-bit párům BAR0/2/4. Aperture 21 dá s offsetem +6 2^27 = 128M, přesně maska `f8000000`, kterou ukázal wmap v RC. Odpovídá to i tomu, že cfghunt v EP módu u `0x6300` žádný vliv na cfg nenašel (v EP se nepoužívá), a Cadence má RC BAR config na stejném místě (LM + `0x300`). **Ověření:** v RC zapsat `0x6300` bajt 0 = `0x14` a pustit `pcie debug wmap 0x10 0x14`. Pokud hypotéza platí, `0x10` ukáže `fc000000`.

K čemu jsou BARy root portu: ve stromu je ovladač neprogramuje a inbound jde přes IO decode okno TF-A. Pokud `0x6300` funguje, dá se BARy úplně vypnout (Linux je nevystavuje) nebo použít jako inbound filtr.

### Řídicí blok 0x4800

| reg | RC | EP | |
|---|---|---|---|
| `0x4800` CTRL0 | `000183c5` | `000183c0` | `[1:0]` gen: RC 1 = Gen2, EP 0 = Gen1; bit 2 `IS_RC`; bit 6 link training v obou |
| `0x4808` CTRL2 | `00000457` | `00000057` | bit 10 MSI receiver jen v RC |
| `0x4844` ISR0 mask | `06f007ff` | `07ff3fff` | RC odmaskováno `f90ff800` (chybové zprávy 11–13, INTx 16–19, MSI 24); EP `f800c000` |
| `0x4850` MSI addr | `000e0200` | `0` | MSI cílová adresa v RC |
| `0x485c` MSI mask | `fffffffc` | `ffffffff` | v RC odmaskované vektory 0 a 1 |
| ostatní | stejné | stejné | `0x4804`, `0x480c`, `0x4810`, `0x4814` REF_CLK `1342`, `0x4828`, `0x484c` |

## 5. Co z toho plyne

1. **EP hlásí PCIe cap verze 1.** Linux u v1 nečte DevCap2, DevCtl2, LinkCap2 ani LinkCtl2 (`pcie_cap_version() < 2`, lspci je také vynechá). Hostitel tak nevidí completion timeout, LTR ani 10-bit tags, přestože jsou v hardwaru vyplněné. Verze v cfg zapisovatelná není (`0xc0` je bez wmap). Je to buď vlastnost hardwaru při `IS_RC=0`, nebo ji řídí některé LMI slovo, které cfghunt v EP neprošel. Stojí za projití `0x6000`–`0x6054` cfghuntem cíleně na `0xc0`.
2. **Hardwarová hlavička je type 0 i v RC.** Type-1 pole (bus numbers, okna) v hardwaru nejsou a softwarový root port je nutnost, ne volba. Typ portu (Root Port) hlásí hardware sám, softwarový patch typu na `0xc0` je zbytečný; nutný je jen bit Slot Implemented.
3. **Link běží na 2.5 GT/s, i když obě strany umí 5 GT/s.** Target Link Speed = 2 je zapsané před tréninkem, takže po L0 je potřeba retrain (Linux to tak dělá). Zkusit `pcie controller link retrain` a pak `pcie cap get 0:0.0 exp`. Pokud to pomůže, retrain přidám do init sekvence.
4. **AER root portu je v hardwaru celý** (`0x12c`–`0x134` jen v RC). Bude jednoduché zapnout Root Error Command a obsloužit ISR0 bity 11–13 (ERR_COR/NONFATAL/FATAL), včetně Source ID.
5. **LMI se mezi rolemi liší jen ve dvou statických slovech** (`0x6044`, `0x6240`). Přepnutí role je tedy hlavně `IS_RC` a gen select v CTRL0. Pro EP link k BeagleY: EP visí v LTSSM `0x03` (Polling), RC na stejné desce, ve stejném slotu a se stejným COMPHY projde do L0. Rozdíl je jen v roli a v gen (EP má Gen1). Stálo by za to zkusit EP s gen select 1 (Gen2), jako má RC.
6. **Otevřené:**
   - SSID `0x0100` v RC hardwarové hlavičce (reset, nikdo ho nezapisuje).
   - Neznámý zapisovatelný `0x32c` v EP.
   - `0x6000 [23:0]` (`ff21` vs `ff09`).
