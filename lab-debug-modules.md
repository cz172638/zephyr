# Debug příkazy jako LLEXT moduly (ESPRESSObin)

Debug příkazy už nemusí být ve firmware. Modul se nahraje za běhu (`lab mod load`) a jeho příkaz se objeví pod `pcie debug <jméno>` nebo `dbg <jméno>`. Nový obraz SPI/SATA je potřeba jen při změně mimo debug kód. Ladění pak znamená přeložit a poslat `.llext`, bez flashování.

Obrazy: **`150-lab-mods-spi.bin`** a **`150-lab-mods-sata.bin`**, stejný obraz pro RC i EP. Moduly k nim: `150-*.llext`. Commit `e012aabe3`.

## Pravidla

- **Číslo obrazu v prefixu.** Modul `NNN-jméno.llext` patří k obrazu `NNN` (`CONFIG_LAB_IMAGE`). Číslo nese i uvnitř a `lab mod load` / `lab ext` modul pro jiný obraz odmítne: `built for image 149, this firmware is image 150; rebuild it, or -f`. S `-f` se nahraje i tak (na vlastní riziko).
- **Kdy nový obraz.** Jen když se mění kód mimo moduly, nebo když modul potřebuje symbol, který firmware neexportuje. To druhé hlídá `west build -d build-eb-all -t lab_mods`: přeloží jen moduly a zkontroluje je proti existujícímu `zephyr.elf` (`scripts/lab/llext-symcheck.pl`). Výsledek je `ok`, nebo seznam chybějících symbolů. Firmware se při tom nepřekládá.
- **Číslo se zvedá** s každou změnou firmware (`CONFIG_LAB_IMAGE` ve snippetu `espressobin-lab`).
- `lab status` i `lab hello` ukazují `image`/`img=`, takže je vidět, co na které desce běží.

## Příkazy

| Příkaz | Co dělá |
|---|---|
| `lab mod load [-f] <slot\|/cesta>...` | nahraje moduly (ze slotu po `lab put`, nebo z FAT); stejné jméno příkazu = náhrada |
| `lab mod unload <příkaz\|all>...` | uvolní modul; nejdřív jeho `fini` (zastaví časovače, vrátí IRQ, odmapuje) |
| `lab mod ls` | nahrané moduly: příkaz, mount, soubor, velikost, stáří |
| `lab ext [-f] <slot\|/cesta> [args]` | jednorázový test (`lab_ext_main`), výstup jde do volajícího shellu |

Z PC jedním příkazem (put + load):

```sh
export LAB_TOKEN=tajne123
lab.pl 192.168.10.2 mod 150-irqtest.llext 150-loopback.llext 150-comphy.llext
```

Z RC na EP (EP = `192.168.10.2`):

```
lab put /SD:/150-loopback.llext          # nebo slot, který přišel z PC
lab exec lab mod load 150-loopback.llext
```

## Moduly v obrazu 150

| Soubor | Příkaz | Účel |
|---|---|---|
| `150-irqtest.llext` | `pcie debug irqtest …` | centrální přerušení `0x1b000` ([pcie-irqtest.md](pcie-irqtest.md)); dřív ve firmware |
| `150-xortest.llext` | `pcie debug xortest [size] [loops]` | XOR engine; dřív ve firmware |
| `150-loopback.llext` | `pcie debug loopback …` | LTSSM trace, hledání ovládání loopbacku Aardvarku |
| `150-comphy.llext` | `dbg comphy …` | registry lanů COMPHY: dump, snímek, rozdíl, pohyblivé bity, zápis |
| `150-netlb.llext` | `dbg netlb …` | MAC loopback NETA (GMII/PCS), MIB čítače |
| `150-regdiff.llext` | `dbg regdiff …` | snímky registrů, diff, událost na protější desce, hlídání (kap. 6) |
| `150-mbusx.llext` | `dbg mbusx …` | MBus okna (IO decode, CPU, OB), SATA DMA přímo do paměti hostu (kap. 7) |
| `150-ibmap.llext` | `pcie debug ibmap …` | inbound blok `0x5800–0x5fff`: „iATU“ Aardvarku (jen EP), kap. 5 |
| `150-lab_hello.llext` | `lab ext 150-lab_hello.llext` | self-test loaderu (je i zabudovaný: `lab selftest`) |

Moduly jsou na ARM64 přeložené bez optimalizace (`-Os` LLEXT build pro ARM64 odebírá). Pro tyhle účely to nevadí.

## 1. Mlčící deska v EP módu: porovnání s funkční

Obě desky jsou `espressobin_v5_0_1`. Jestli se liší, ukáže rozdíl registrů při stejném firmware a stejném nastavení. Funkční EP označuju „A“, mlčící „B“. Obě jsou nastavené jako EP, každá postupně proti stejnému hostu a stejným kabelem.

| Krok | A (funkční EP) | B (mlčící EP) |
|---|---|---|
| 1 | `lab mod load 150-comphy.llext 150-loopback.llext` | totéž |
| 2 | `pcie debug loopback ltssm 2000` (během startu linku) | totéž: kde skončí? |
| 3 | `dbg comphy` (shrnutí lanů) | `dbg comphy` |
| 4 | `dbg comphy dump 1` > uložit | `dbg comphy dump 1` > uložit |
| 5 | `dbg comphy watch 1 2000 10` | `dbg comphy watch 1 2000 10` |

Výstupy ze 4 a 5 stačí poslat (nebo `diff` na PC). Lane 1 je PCIe.

Co hledat:

- `0x01` (PLL lock, bit 8);
- `0x23` bit 4 `PLL_READY_TX`;
- reference v `0x01[4:0]`;
- `0x4f` bit 4 (výstup REFCLK v EP vypnutý?);
- bity, které se na B nehýbou a na A ano (CDR, signal detect).

Krok 2 rozliší podle tabulky z předchozí zprávy:

| B skončí v LTSSM | Co to znamená |
|---|---|
| Detect | nevidí terminaci protějšku |
| Polling | nepřijímá TS |
| Config / Recovery | data tečou, nesedí lane nebo rychlost |

## 2. Hledání loopbacku PCIe (Kirkwood DFT na Aardvarku)

LTSSM Aardvarku má stavy `0x21–0x26` (loopback master/slave), ale nevíme, čím se zapínají. RC hledá, EP hlásí, jestli dostal příkaz k loopbacku (`LOOPBACK-*-slave`):

| Krok | RC | EP |
|---|---|---|
| 1 | `lab mod load 150-loopback.llext` | `lab mod load 150-loopback.llext` |
| 2 | link nahoře: `pcie debug loopback ltssm` → `0x10 L0` | `pcie debug loopback ltssm` → `0x10 L0` |
| 3 | `pcie debug loopback scan 0x4800 0x0fffff80 100 peer` | (odpovídá RC přes agenta) |
| 4 | dtto `0x4804`, `0x4808` (CTRL1/2) | |
| 5 | nalezený bit: `pcie debug loopback try <off> <bit> 500 peer` (podrobný trace) | |

- `scan` bit po bitu: překlopí ho, 100 ms sleduje LTSSM, zeptá se EP na jeho stav, vrátí bit a počká na L0. Řádek s `<== PEER LOOPS BACK` je hledaný bit.
- Když se link do 1 s nevrátí, `scan` skončí a vypíše, kde LTSSM visí. Pak pomůže `pcie debug hotreset` nebo restart.
- Maska `0x0fffff80` u CTRL0 vynechává známé bity:
  - [1:0] generace,
  - 2 `IS_RC`,
  - [5:3] šířka,
  - 6 link training,
  - [31:28] INTx.
- `hold <off> <bit> 0|1` nastaví bit natrvalo, pro ruční pokusy.

## 3. COMPHY: interní loopback a PRBS

Postup podle Kirkwoodu (PHY reg `0x0` bit 0/1, `0x44`, `0x81`, PRBS `0x40` bit 2, čítače `0x50`/`0x51`) je **jen hypotéza**. Číslování registrů A3720 se od Kirkwoodu liší. Bezpečně se to dá zkoušet na desce bez kabelu, s PCIe lanem v klidu:

```
dbg comphy snap 1
dbg comphy kw 1 lb shallow
pcie debug loopback ltssm 1000     # projde Polling → Config → L0 sám na sebe?
dbg comphy kw 1 prbs on ; dbg comphy kw 1 prbs count
dbg comphy diff 1                  # co se změnilo
dbg comphy kw 1 lb off ; dbg comphy kw 1 prbs off
```

Užitečnější je asi začít registrem `0x23`. Linux mu říká `DIG_LOOPBACK_EN`, ale používá z něj jen bity [11:10] a 4. Kandidáty jsou spodní bity: `dbg comphy wr 1 0x23 0x1 0x1` a sledovat LTSSM. Zápisy do COMPHY můžou shodit SATA, USB3 nebo PCIe; pak pomůže reset.

Lane 2 (SATA/USB3) je za blokem AHCI. Když má vypnutý takt, modul to pozná zkušebním čtením a lane odmítne (`no access`), místo aby deska spadla na abort.

## 4. NETA: MAC loopback

GMAC v NETA vypadá jako GMAC v `mvpp2`, kde je v `GMAC_CTRL_1` bit 5 GMII loopback a bit 6 PCS loopback (na A3720 by to byl `0x2c04`). ESPRESSObin má jen NETA0 a během loopbacku je deska mimo síť. Proto je tu `pulse`, který loopback po zadané době zase vypne:

```
dbg netlb                      # ctrl0..4, stav loopbacku
dbg netlb pulse 0 gmii 2000    # 2 s loopback, čítače tx/rx za tu dobu
dbg netlb on 0 pcs 10000       # zapnout, samo se vypne za 10 s
```

Na UARTu je během pulsu vidět, jestli chodí rámce zpátky. `rx ≥ tx` během pulsu je příznak, že bit opravdu dělá loopback.

## 5. Inbound okna Aardvarku („iATU“): `pcie debug ibmap`

Známé je jen `0x5800/0x5804` BAR0, `0x5808/0x580c` BAR2 a `0x5810/0x5814` BAR4: 64bitová AXI báze každého BARu. Položky na `+0x20`, `+0x40`, `+0x80`, `+0xa0` a bloky `0x5a00/0x5c00/0x5e00` vypadají stejně. Můžou to být další okna uvnitř BARu (jako iATU v address match módu), jiné funkce nebo ROM.

Postup:

1. EP označí DDR značkami: každých 4 KiB slovo `0x1ba0xxxx`, kde `xxxx` je pořadí.
2. Kandidátní registr se nasměruje na označenou oblast.
3. RC přes lab agenta (`devmem` na protější desce) čte BAR EP na několika offsetech.
4. Modul přečtené hodnoty dekóduje: kam v DDR čtení dopadlo.

Modul je jen pro EP (v RC módu se nenahraje). Hostem je ESPRESSObin v RC módu (CPU adresa = bus adresa, okno `0xe8000000`).

| Krok | EP (`ibmap`) | RC |
|---|---|---|
| 1 | `lab mod load 150-ibmap.llext` | link nahoře, EP vyjmenovaný (`pcie ls`) |
| 2 | `pcie debug ibmap` → nenulové registry bloku, IO decode okno, bus adresy BARů | |
| 3 | `pcie debug ibmap tag 0x11000000 0x100000` (uvnitř carveoutu 0x10000000+32M a IO decode okna!) | |
| 4 | `pcie debug ibmap host 4 0 0x1000 0x10000` → co RC teď čte z BAR4 | (devmem přes agenta) |
| 5 | `pcie debug ibmap hunt 0x5818 0x5ffc 4 0 0x1000 0x10000 0x100000` | (devmem přes agenta) |
| 6 | registr s výsledkem: ruční pokusy `set <reg> <axi>` + `host 4 <off>…` | |

- `hunt` zkusí každý pár lo/hi v rozsahu (krok 8, vlastní pár BARu přeskočí). Nastaví ho na značky, nechá RC přečíst BAR na zadaných offsetech, vrátí původní hodnotu. Vypisuje jen změny: `05a00 -> BAR4+0x10000: 1ba00010 = tag, AXI 0x11010000`, nebo `no effect`.
- Z toho plyne:
  - které registry překládají,
  - od jakého offsetu BARu platí (porovnáním offsetů 0/4K/64K/1M),
  - jestli mají masku nebo velikost (sousední slovo, další pokus se `set`).
- **Riziko:** registr, který pošle čtení mimo IO decode okno, skončí na RC completer abortem. Na Zephyr RC to může být SError a pád RC. Proto značky jen uvnitř okna (`tag` varuje) a RC s `lab` připravený na restart.
- S Linuxem jako hostem: `set` na EP, ruční čtení na hostu (`pcitest -P`, `devmem`) a `pcie debug ibmap decode <hodnota>…` na EP. `hostoff` nastaví posun CPU adresy hostu proti bus adrese (J722S: `0x600000000`).
- **Formát podle Kirkwoodu (spec 2.3, registry `0x41820…`):**
  - Kirkwood dekóduje inbound ve dvou krocích. Zásah do BAR1/BAR2 se dál rozdělí šesti okny po `0x10`:
    - `+0` control: en, BarMap (bit 1), target [7:4], attr [15:8], velikost [31:16] jako jedničky po 64 KiB;
    - `+4` báze: PCI adresa [31:16];
    - `+0xc` remap: en (bit 0), AXI [31:16];
    - okna 4 a 5 mají navíc remap high na `+0x10`.
  - Co do žádného okna nepadne, jde do „default window“ (`0x418B0`). ROM má vlastní okno s remapem (`0x418C0/C4`).
  - `kwwin <reg> <bar> <off> <size> <axi>` zapíše jedno takové okno do kandidátního bloku (target DRAM, attr `0x3d` jako IO decode A3720; BarMap = 1 pro BAR4 je odhad).
  - `kwhunt <from> <to> 0x10|0x20 <bar> <off>` to zkusí blok po bloku: 64K okno BAR+off → značky, RC čte BAR+off a +0x1000, původní hodnoty vrátí.
  - Pořadí pokusů: `kwhunt 0x5a00 0x5ff0 0x10 4 0x10000`, pak se stride `0x20`, pak bloky `0x5820…0x58bc`.
  - Známý pár `0x5810/0x5814` (AXI báze BAR4) odpovídá spíš kirkwoodímu remapu celého BARu nebo default window.
- Image 150 neexportuje `snprintf` (jen `sprintf`), modul s tím počítá. Přidat export = nový obraz.

## 6. Snímky registrů: `dbg regdiff`

K nápovědám z Kirkwoodu (příloha A.6): kde má EP bus/device číslo, které mu dal host (Kirkwood `0x41A04`, mění se s každým CfgWr), indikace z protějšku (hot reset, loopback) a co smaže hot reset. Všechna čtení jdou přes probe, takže díry a vypnuté bloky se ukážou jako `abort` a deska nespadne. Slova, která se mění sama (čítače, stav), se najdou při snímku (`settle`) a z diffu se vynechají.

| Krok | EP | RC |
|---|---|---|
| 1 | `lab mod load 150-regdiff.llext` | |
| 2 | `dbg regdiff snap ctrl 0xd0074000 0x4000` (Aardvark `0x4000–0x7fff`: ctrl, OB, inbound, LMI) | |
| 3 | `dbg regdiff event ctrl pcie cfg set 1:0.0 0x3c 0x1ff` | (příkaz běží na RC přes agenta: CfgWr na EP) |
| 4 | `dbg regdiff find ctrl 0x100 0xff00` (bus 1 v [15:8]) a `find ctrl 0x1 0xff` | |
| 5 | `dbg regdiff event ctrl pcie debug hotreset` | hot reset z RC |

- `event` vezme těsně před událostí čerstvý stav a po ní vypíše jen změny.
- V kroku 5 jde hlavně o registry, které se vynulují (to by vysvětlilo nutnost driver přeprogramovat) a o indikace „slave hot reset“.
- Bus/device číslo se pozná nejlépe tak, že EP leží za switchem (bus ≠ 1) a `find` hledá to číslo.
- `watch ctrl 5000` po 5 s vypíše, která slova se hýbala a kolikrát; mezitím se dělá událost ručně (restart hostu, `rescan`).
- Snímky: max 4, každý do 64 KiB. `diff -u` vezme nové hodnoty jako základ, `-a` ukáže i volatilní bity.

## 7. MBus okna a DMA periferií přímo do hostu: `dbg mbusx`

Kirkwood spec 6.20: jednotka PCIe není transparentní most, ale funguje jako NTB (okna s remapem). Kap. 2.4: SATA DMA smí cílit do SDRAM *i* do PCIe. Na A3720 to znamená, že periferie (SATA, NETA, USB3) může přes své okno IO decode s targetem 2 (PCIe) a přes OB okno Aardvarku číst a zapisovat přímo paměť hostu. Pro NVMe target a pfab-net by to byl zero-copy.

- `dbg mbusx` vypíše okna IO decode všech masterů podle TF-A (usb, usb3, dma, neta0/1, pcie, sata, sd, emmc, eip97), CPU decode okna `0xd000cf00` a OB okna Aardvarku.
- `set <master> <win> <target> <attr> <base> <size>`, `off`, `ob <win|free> <axi> <host> <size>` mění okna.
- `restore` vrátí všechno původní, při `lab mod unload` se to stane samo.

**`sata2host`: AHCI čte sektory rovnou do paměti hostu** (EP; host = ESPRESSObin v RC módu, bus adresa = fyzická adresa RC):

| Krok | EP | RC |
|---|---|---|
| 1 | `lab mod load 150-mbusx.llext`, disk běží (`ata` / FAT připojený) | link nahoře, volná oblast DDR, např. `0x1f000000` |
| 2 | `dbg mbusx sata` (volné okno SATA?) a `dbg mbusx` (volné OB okno?) | |
| 3 | `dbg mbusx sata2host 0x1f000000 0 8` | (devmem přes agenta) |

- Modul nejdřív přečte sektor do lokální DDR (pro srovnání). Pak dočasně nastaví volné okno SATA: AXI `0xef000000` + 1 MiB → target 2 (PCIe, attr 0 jako okno XOR) a volné OB okno `0xef000000` → bus adresa hostu.
- Příkaz jde do slotu 1 AHCI, vedle slotu 0 driveru, a jen když je driver v klidu (`CI`/`SACT` = 0, jinak `-EBUSY`). Pak modul okna vrátí a přes agenta přečte na RC první 4 slova: `= local` znamená, že SATA zapsala přímo do paměti hostu.
- Disk za port multiplierem: 4. argument je PMP. Jiné AXI okno: 5. argument.
- **Riziko:** špatná bus adresa přepíše paměť hostu. Proto oblast, kterou RC nepoužívá.
- Pokus s okny IO decode na jiný target, například okno PCIe masteru (`pcie`) na target 1 (interní registry, jako BAR0 na Kirkwoodu): `set pcie 0 1 <attr> …` a čtení BARu z hostu. Attr pro registry zatím neznáme.

## 8. Vlastní modul

Konec souboru místo `SHELL_SUBCMD_ADD`:

```c
#include <zephyr/lab/mod.h>

LAB_DBG_CMD((pcie_debug), "pcie", mujtest, &mujtest_cmds, "nápověda",
	    cmd_show, 1, 0, mujtest_init, mujtest_fini);
```

- Přeložený do firmware je to obyčejný `pcie debug mujtest`.
- Jako modul přidat do `samples/.../netshell/CMakeLists.txt` řádek `lab_dbg_module(mujtest <zdroj.c>)`.
- `fini` musí vrátit všechno, co modul nastavil: časovače, práce v workqueue, IRQ (na ARM64 není `irq_disconnect_dynamic`; linka se vrací na `z_irq_spurious`), mapování.
- Modul může volat syscally, libc, `shell_*`, `k_timer_*`, `k_work_*`, `k_mem_map_phys_bare`, dynamická IRQ, cache a ESPRESSObin funkce z `src/dbg_exports.c`. Ostatní nahlásí `-t lab_mods`.
- Zařízení z DT jdou přes `DEVICE_DT_GET` (`CONFIG_LLEXT_EXPORT_DEVICES`).

## 9. Testy

- `tests/subsys/lab/ext` (qemu_cortex_a53, MMU): 7/7. Pokrývá nahrání, mount pod `dbg`, podpříkaz, náhradu stejného jména s čistými daty, `fini` při uvolnění, odmítnutí jiného obrazu a `-f`, opakované nahrávání bez vyčerpání slotů.
- `tests/subsys/lab/agent` (native_sim/64): 8/8.
- Všechny moduly obrazu 150 projdou kontrolou symbolů.
- Relokace modulů: jen `ABS64`, `ADR_PREL_PG_HI21`, `ADD_ABS_LO12_NC`, `CALL26`; ARM64 loader je všechny umí.
- Na hardwaru zatím neověřeno.

Opravy po cestě:

- **Upstream `add_llext_target`:** kopie objektu do `.llext` měla jen pořadovou závislost, takže změněný zdroják `.llext` nepřeložil. Opraveno v `cmake/modules/extensions.cmake`.
- **Dummy shell (testy):** dokud ho jeho vlákno neaktivuje, zahazuje výstup handlerů. V testech to znamená krátce počkat po startu.
