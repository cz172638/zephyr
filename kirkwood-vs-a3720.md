# Kirkwood (88F6281) vs. Armada 3720: srovnání a hledání PCIe DMA

Zdroj: Marvell 88F6180/88F619x/88F6281 Functional Specifications (MV-S104860-U0 Rev. C, 2008), Linux `pci-aardvark.c` a `mv_xor`, TF-A `marvell_plat_config.c`, naše měření na ESPRESSObinu. Kirkwood je o dvě generace starší, ale řada bloků A3720 z této rodiny vychází.

## 1. Bloky: co je stejné a co ne

| Blok | Kirkwood 88F6281 | Armada 3720 | Vztah |
|---|---|---|---|
| PCIe | „PEX“ (Linux `pci-mvebu`), PCIe 1.1 x1, RC/EP, registry `0x40000–0x4FFFF` | **Aardvark** (Linux `pci-aardvark`), Gen2 x1, RC/EP, `0xd0070000` + 128 KiB | **jiné IP.** Myšlenka je podobná: config hlavička namapovaná do registrů a lokálně RW i u RO polí (Kirkwood `CfgMapToMemEn`, Aardvark `0x000–0xfff`). |
| PCIe DMA | **žádný.** Data hýbou jiní Mbus mastery (XOR, IDMA, jednotky se svým DMA) | podle Linuxu žádný, ale Aardvark má **command mode s frontami a DMA přerušeními** (kap. 3) | Kirkwood potvrzuje vzor Marvellu „DMA = XOR“. Aardvark je ale jiný. |
| XOR | 2 enginy × 2 kanály: **port 0** `0x60800`/`0x60A00`, **port 1** `0x60900`/`0x60B00` | Linux DT zná jen `0x60900`/`0x60B00` (1 engine, 2 kanály) | **stejné IP.** A3720 používá adresy Kirkwood portu 1, takže stojí za ověření, jestli na `0x60800`/`0x60A00` není druhý engine. |
| XOR režimy | XOR (8 zdrojů), **CRC32C** (iSCSI), DMA, **MemInit** (memset) | Linux: memcpy, XOR, interrupt; režim v deskriptoru (config mode 7) | deskriptor je stejný (32 B DMA/CRC, 64 B XOR, Own bit 31, Success bit 30, `crc32` slovo). Kirkwood má režim v config registru, A3720 (novější revize) navíc v deskriptoru. |
| XOR přerušení | cause `+0x30`, mask `+0x40`: bity 0–9 (EOD, EOC, Stopped, Paused, AddrDecode, AccProt, WrProt, Own, IntParity, Xbar), kanál 1 v 16–25, **RW0C** | Linux `mv_xor` stejně | **potvrzuje náš ovladač** (image 147). Pozn.: Kirkwood doc uvádí bity masky kanálu 1 jako RO. Pravděpodobně chyba v dokumentu (Linux je zapisuje), ale kdyby kanál 1 přerušení nedával, zachrání to náš fallback na polling. |
| SATA | SATA II se **EDMA** (Linux `sata_mv`): request/response fronty (CRQB 32 B, CRPB 8 B), ePRD tabulky | **AHCI** (Linux `ahci_mvebu`) | jiný programovací model. U A3720 je z Marvell světa jen vendor indirect reg `0xa0/0xa4` a COMPHY. |
| Ethernet | GbE (Linux `mv643xx_eth`), SDMA se **spojovým seznamem** deskriptorů, 8 RX/8 TX front, Marvell header, DSA tagy, MIB | **NETA** (Linux `mvneta`), **kruhy** s čítači occupied/processed, volitelně BM | NETA je potomek mv643xx: stejná rodina hlaviček, DSA a MIB, ale deskriptory jsou kruhy místo spojového seznamu. |
| Dekódování adres | **každá jednotka má svá okna** (XOR `XEBARx`, GbE BARy, SATA okna, PCIe okna) | **centrální IO decode pro každého mastera** `0xd000c000 + p*0x100`, které nastavuje TF-A | stejná myšlenka (master → okna → target), jen přesunutá do jednoho bloku |

**IO decode mastery A3720** (TF-A `io_dec_win_conf`):

| Blok | Master |
|---|---|
| `c000` | USB |
| `c100` | USB3 |
| `c200` | **DMA (XOR)** |
| `c300` | NETA0 |
| `c400` | NETA1 |
| `c500` | **PCIe** |
| `c800` | SATA |
| `ca00` | SD |
| `cb00` | eMMC |
| `ce00` | EIP97 |

TF-A nenastavuje bloky **`c600` (abort), `c700`, `c900`, `cc00`, `cd00`**: jsou to skrytí mastery.

## 2. Co z Kirkwood specky použít hned

- **XOR MemInit** (memset): registry high bloku `+0xB0` destination, `+0xC0` block size, `+0xE0/+0xE4` initial value (Linux `mv_xor.h` má stejné offsety). Použití: NVMe Write Zeroes, nulování bufferů, test paměti. Smí ho mít jen jeden kanál naráz.
- **XOR CRC32C**: řetězec deskriptorů, výsledek v posledním deskriptoru. Použití: iSCSI a NVMe-oF data digest, kontrola přenosů fabricu, ext4 a btrfs metadata. Na A3720 neověřené. Linux ho v `mv_xor` nepoužívá.
- **Hot insertion deskriptorů** do běžícího řetězce: pro síťový ovladač (`pfab-net`) bez restartu kanálu.
- **Druhý XOR engine**: `regs scan 0xd0060800 0x100`, `regs scan 0xd0060a00 0x100`. Pokud dekóduje a výchozí hodnoty sedí s portem 1 (config, okna), máme 4 kanály.

## 3. PCIe DMA v Aardvarku: stopy

Linux `pci-aardvark.c` sice používá jen **direct mode**, ale definuje víc, než používá:

- `CTRL_CONFIG_REG` `0x18000` bit 0: **`PCIE_CORE_MODE_DIRECT` (0) / `PCIE_CORE_MODE_COMMAND` (1)**.
- Centrální přerušení `0x1b000` (`HOST_CTRL_INT_STATUS`, `+4` mask). Bity:

| Bit | Název | Bit | Název |
|---|---|---|---|
| 0 | `CMDQ_INT` | 13 | `DIR_RD_DDR_DET` |
| 1 | `MSI_STATUS_INT` | 14 | `DIR_WR_DDR_DET` |
| 3 | `CMD_SENT_DONE` | 16 | `CORE_INT` |
| 4 | **`DMA_INT`** | 17 | `CORE_INT_PIO` |
| 5 | **`IB_DXFERDONE`** | 18 | `DPMU_INT` |
| 6 | **`OB_DXFERDONE`** | 19 | `PCIE_MIS_INT` |
| 7 | **`OB_RXFERDONE`** | 20, 21 | `MSI_INT1/2_DET` |
| 12 | `COMPQ_INT` | 22 | **`RC_DBELL_DET`** |
| | | 23 | `EP_STATUS` |

To vypadá na **command mode**: fronta příkazů (CMDQ) a fronta dokončení (COMPQ) v paměti a řadič sám přenáší data inbound i outbound („data transfer done“). Linux, U-Boot ani TF-A ho nepoužívají a Marvell ho veřejně nedokumentuje. Odkazy na funkční errata jsou jen pod NDA.

Navazuje na to naše měření: `wmap 0x10000–0x1ffff` našlo zapisovatelný jen blok **`0x19000`**: dvě 64bit adresy s výchozí hodnotou ukazující do vlastních registrů (`0xd0089010/14`), masky `0x1900c`, `0x19034`, ID `0x19304–0c`. Pravděpodobně **báze CMDQ/COMPQ** nebo MSI příjmu.

`RC_DBELL_DET` je doorbell: zápis od hostitele vyvolá přerušení na EP. To je přesně to, co v `pcie-net-brainstorm.md` chybí pro EP bez pollingu.

## 4. Postup hledání

Používáme jen abort-safe čtení (`regs`, `a3700_probe_read32`), zápisy jen přes `wmap` (zapíše a obnoví), a nic se nespouští bez pasti na adresy.

1. **Mapa prostoru Aardvarku** `0xd0070000–0xd008ffff`:
   ```
   regs scan 0xd0070000 0x20000          # dekódující a abortující úseky
   regs nz   0xd0088000 0x4000           # 0x18000–0x1bfff: ctrl core, 0x19000, central int
   ```
   Porovnat s Linuxem (`0x0000` cfg, `0x4000` PIO, `0x4800` control, `0x4c00` OB, `0x5800` a `0x5a00…` inbound, `0x6000` LMI, `0x18000` ctrl, `0x1b000` int). Co dekóduje a Linux to nezná, je kandidát.
2. **Implementované bity přerušení**: zapsat `0xffffffff` do masky `0x1b004` a přečíst. Totéž pro ISR0/ISR1 masky (`0x4844`, `0x484c`) a PIO ISR masku. Implementované bity, které Linux nepoužívá, ukážou, co blok umí. Pak obnovit.
3. **Command mode „naprázdno“** (EP s linkou dole, nebo RC bez zařízení): přepnout `0x18000` bit 0 na 1 a hned potom:
   - `regs snap 0xd0088000 0x4000` → `regs diff`: co se po přepnutí změnilo nebo začalo dekódovat;
   - `pcie debug wmap` na `0x18000–0x1bfff`: v command mode mohou být RW registry, které jsou v direct mode RO (front base/size/head/tail);
   - vrátit direct mode.

   DMA signatura, kterou hledáme:
   - páry adresních registrů se spodními bity RO (zarovnání),
   - registr počtu nebo velikosti fronty,
   - head/tail indexy,
   - start bit, který se sám nuluje,
   - status s W1C bity, které odpovídají bitům 0–7 a 12 v `0x1b000`.
4. **Past na adresy (IO decode jako senzor DMA):** okna PCIe mastera (`c500`) a neznámých masterů (`c700`, `c900`, `cc00`, `cd00`) nastavit tak, aby pokrývala jen malý scratch buffer. Error status `+0xf8` každého bloku ukáže, **který master** se mimo okno pokusil číst nebo psát. Pokus s nalezenými registry fronty (báze na scratch buffer s jedním příkazem a nulovou délkou) pak bezpečně ukáže, jestli a odkud command mode čte.
5. **Snap/diff kolem známých událostí** (bez command mode):
   - enumerace hostitelem: kde se uloží bus/device number EP (Kirkwood ho má v `PCI Express Status Register 0x41A04` `PexBusNum/PexDevNum`, u Aardvarku je to otevřené TODO);
   - zápis hostitele do BARu: změní se `0x1b000` bit 13/14 `DIR_*_DDR_DET`?;
   - MSI z EP, XOR přenos k hostiteli.
6. **Doorbell:** v EP módu odmaskovat `RC_DBELL_DET` a nechat hostitele zapisovat do různých offsetů BAR0/BAR4 a konfiguračního prostoru. Hledáme, který zápis bit nastaví (možná zvláštní offset v `0x19000` nebo v LMI). Nástroj: [`pcie debug irqtest`](pcie-irqtest.md).
7. Kdyby command mode existoval a fungoval: EP ovladač dostane **vlastní PCIe DMA** (bez okna DMA jednotky, bez XOR write-back problému s pořadím) a doorbell. Jinak zůstává XOR (případně 4 kanály) a polling.

## 5. Ethernet vs. SATA: podobnost

| | Kirkwood GbE (SDMA) | Kirkwood SATA (EDMA) | A3720 NETA | A3720 AHCI |
|---|---|---|---|---|
| Fronta | **spojový seznam** deskriptorů (next pointer), 8 RX + 8 TX | **kruhy**: request (CRQB 32 B) a response (CRPB 8 B), in/out pointer registry | **kruhy**, čítače occupied/processed | command list (32 slotů) + received FIS |
| Vlastnictví | Own bit 31 v cmd/status | indexy in/out | čítače | `PxCI`/`PxSACT` bity |
| Scatter-gather | řetěz bufferů s F/L bity | **ePRD tabulka** (adresa, počet, EOT) | buffery po deskriptorech | **PRDT** (adresa, počet, I) |
| Dokončení | zavření deskriptoru, EI bit | **CRPB** s ID, flagy a **timestampem** | RX deskriptor se statusem | D2H FIS, PxIS |
| Moderace přerušení | coalescing (čas, počet) | **Command Delay Threshold** (počet nebo čas) | coalescing | – |
| Offloady | checksum IP/L4, Marvell header, DSA | – | totéž + BM | NCQ |
| Okna | vlastní BARy jednotky | vlastní okna | IO decode `c300/c400` | IO decode `c800` |

Společný vzor Marvellu: jednotka je samostatný Mbus/AXI master s vlastním DMA, okny, cause/mask registry (RW0C) a moderací přerušení. Kirkwood SATA EDMA a NETA mají stejný model, **fronty s indexy producenta a konzumenta** (stejně jako NVMe). GbE SDMA a XOR používají spojový seznam. ePRD a PRDT jsou prakticky totéž (adresa, délka, konec).

Pro náš fabric a `pfab-net`:

- **Kruhy s indexy jsou pro PCIe lepší než spojový seznam.** Spojový seznam znamená čtení pointerů přes linku. Kruh v paměti konzumenta a index zapsaný producentem znamenají jen zápisy, což je náš model.
- **Completion s ID a timestampem** (jako CRPB) se hodí pro PTP a měření latence v `pfab-net`.
- **Moderace přerušení** podle počtu a času (GbE coalescing, EDMA threshold) odpovídá `ethtool -C`.

## 6. DFT (loopbacky, PRBS): co Kirkwood dokumentuje a co hledat na A3720

Kirkwood spec, kap. 6.19 a registry:

| Blok | Kirkwood | A3720: stav / kde hledat |
|---|---|---|
| PCIe master loopback | `ConfMstrLb` (PCIe Control `0x41A00` bit 26), jen RC | Aardvark má stavy LTSSM `0x21–0x26`, ovládací bit neznámý → `pcie debug loopback scan` |
| PCIe slave loopback | řídí protějšek, `PexSlvLb` | EP by měl ukázat LTSSM `0x25`, když RC loopback zapne |
| PCIe interní loopback | PHY `0x0` bit 0 (shallow) / bit 1 (deep), `0x44[9:8]=3`, `0x81[11:10]=2` | COMPHY lane 1; Linux zná `0x23` jako `DIG_LOOPBACK_EN` (bez popisu bitů) → `dbg comphy` |
| PRBS | PHY `0x40` bit 2, chyby `0x50/0x51`, reset `0x40` bit 3 | hypotéza na COMPHY → `dbg comphy kw <lane> prbs` |
| PCIe Debug Control `0x41A60` | maskování resetů, soft reset jednotky | v Aardvarku nenalezeno |
| SATA | near-end PHY loopback, vzor BIST-DW1/2, `LBPass`, BIST FIS | AHCI: BIST Activate FIS (typ `0x58`) z command listu; PHY loopback přes COMPHY lane 2 (`0x223` = U-Boot `vphy_loopback_reg0`) |
| GbE | `PcsLB` (PSC1 bit 1) | NETA GMAC `0x2c04` bit 5 GMII / bit 6 PCS (podle `mvpp2`) → `dbg netlb` |
| UART, audio, TS | MCR loopback, playback → record, TS0↔TS1 | nezkoumáno |

Nástroje jsou LLEXT moduly obrazu 150, postup v [lab-debug-modules.md](lab-debug-modules.md).

## 7. PCIe remapování adres

| Směr | Kirkwood | A3720 |
|---|---|---|
| CPU → PCIe | CPU okna `0x20000…` s Remap Low/High (spec 2.1.1, horní bity adresy nahradí remap podle velikosti okna) | CPU decode okno PCIe `0xe8000000` (TF-A `dram_win.c`, `CPU_DEC_REMAP_LOW`), hlavně **OB okna Aardvarku `0x4c00`** (match, remap, maska, akce). Ověřeno |
| PCIe → paměť | BAR1/BAR2 (velikost a enable v `0x41804/08`), pod nimi **6 podoken** `0x41820 + 0x10·n` (control: en, BarMap, target, attr, velikost; base = PCI adresa; remap + en; okna 4–5 remap high), default window `0x418B0` pro netrefené, ROM okno `0x418C0/C4` (spec 2.3.1, 2.3.2) | velikost BARů v LMI `0x6240/44`, **AXI báze celého BARu `0x5800–0x5814`** (ověřeno), za tím MBus IO decode `0xd000c500` (TF-A: bez remapu). Podokna nenalezena; kandidáti `0x5820…`, `0x5a00/0x5c00/0x5e00` → `pcie debug ibmap kwhunt` |

Na Kirkwoodu jde nesedící přístup do default window, jinak je to UR plus přerušení. Na A3720 jsme zatím viděli jen UR/abort za IO decode oknem.
