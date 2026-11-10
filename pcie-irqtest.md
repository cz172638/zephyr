# ESPRESSObin (Armada 3720): `pcie debug irqtest`, centrální přerušení Aardvarku v EP módu

Nástroj pro krok 6 v [Kirkwood vs. Armada 3720](kirkwood-vs-a3720.md#4-postup-hledání). Zjišťuje, která akce hostitele nastaví který bit `HOST_CTRL_INT_STATUS` (`0x1b000`) a jestli se bit dostane až do GIC (SPI 29). Nejvíc nás zajímají:

| Bit | Název | Co čekáme |
|---|---|---|
| 22 | `RC_DBELL_DET` | doorbell: zápis hostitele vyvolá přerušení na EP (chybí pro `pfab-net` bez pollingu) |
| 14 | `DIR_WR_DDR_DET` | inbound zápis do DDR (direct mode), možná každý zápis do BARu |

Linux oba bity definuje, ale nepoužívá. Marvell je nedokumentuje.

## Režimy

```
pcie debug irqtest arm [irq|open|poll] [bity] [ms]
```

| Režim | Maska `0x1b004` | GIC linka | Kdo čte status | Odpovídá na otázku |
|---|---|---|---|---|
| `irq` (výchozí) | bity odmaskované | zapnutá | ISR | dá se na to pověsit ovladač? |
| `open` | bity odmaskované | vypnutá | časovač (`ms`, výchozí 1) + GIC pending bit | aktivuje bit linku SPI 29? |
| `poll` | nezměněná (maskovaná) | vypnutá | časovač | latchuje se status i při masce? |

- **Bity:** jména podle Linuxu (`RC_DBELL_DET`) nebo krátká (`dbell`, `wrddr`, `rdddr`, `msi`, `dma`, `ibdone`, `obdone`, `cmdq`, `compq`, `core`, …), čárkou oddělené, `all` nebo maska. Výchozí je `dbell,wrddr`.
- **Událost** uloží: čas od `arm` v µs, raw `0x1b000`, ISR0, ISR1, GIC pending a až 8 slov z rozsahu `irqtest regs`. Nastavené bity, které patří k testu, se smažou (W1C), ostatní zůstanou.
- **`arm`** nejdřív vypíše, co bylo nalatchované už před startem, pak to smaže.
- **Ochrana proti bouři:** víc než 2000 vstupů do ISR za sekundu linku vypne, vrátí masku a pokračuje režimem `poll` (`show` hlásí „storm“). `DIR_WR_DDR_DET` při blokovém zápisu hostitele bouři klidně udělá.

Další příkazy:

| Příkaz | Co dělá |
|---|---|
| `irqtest` / `irqtest show` | stav, `0x1b000`/`0x1b004` dekódované, ISR0/1 s maskami, čítače po bitech |
| `irqtest log [clear]` | posledních 32 událostí |
| `irqtest wait [ms]` | čeká na další událost a vypíše ji (výchozí 5 s), pro testy řízené z hostitele |
| `irqtest regs <off> [n]` / `regs off` | slova registrů zachycená s každou událostí (ověřuje se, že čtení neabortuje) |
| `irqtest kick` | nastaví SPI 29 jako pending v GIC: ověří cestu do ISR bez hostitele |
| `irqtest off` | konec, vrátí původní masku testovaných bitů |

`pcie debug up`/`setup` celý řadič přenastaví a `0x1b004` zase zamaskuje. `show` to pozná („armed bits masked again“), pak stačí znovu `arm`.

## GIC linka a RC ovladač

SPI 29 má staticky připojený RC ovladač (`pcie_aardvark.c`), protože image obsahuje oba ovladače a roli vybírá `advk/mode`.

| Build | Linka pro `irq` |
|---|---|
| bez RC ovladače | `irqtest` ji připojí staticky |
| s RC ovladačem + `CONFIG_DYNAMIC_INTERRUPTS=y` | v EP módu ji `irqtest` za běhu převezme (`irq_connect_dynamic`), RC ovladač v EP módu nic nedělá |
| s RC ovladačem bez dynamických přerušení (nebo se `SHARED_INTERRUPTS`) | jen `open` a `poll` |

Snippet `espressobin-pcie-ep` zapíná `CONFIG_DYNAMIC_INTERRUPTS`. Kconfig `CONFIG_PCIE_EPC_AARDVARK_IRQTEST` je s shellem defaultně zapnutý.

## Postup testu

Na EP (ESPRESSObin, `-S espressobin-pcie-ep`):

```
pcie debug irqtest arm irq                   # 0. cesta do ISR bez hostitele:
pcie debug irqtest kick                      #    "isr entries 0 -> 1"
pcie debug irqtest arm poll all              # 1. co se vůbec latchuje (maska zavřená)
pcie debug irqtest arm open dbell,wrddr      # 2. aktivuje to linku?
pcie debug irqtest arm irq dbell,wrddr       # 3. skutečné přerušení
pcie debug irqtest regs 0x19000 8            # kandidát na data doorbellu (blok 0x19000)
pcie debug irqtest wait 30000                # a teď hostitel
```

Na hostiteli (Linux, adresy BARů z `lspci -vv -d 1b4b:`):

```sh
B0=0x...; B4=0x...                         # BAR0, BAR4 z lspci
busybox devmem $((B0 + 0x0))  32 0x1       # BAR0 začátek
busybox devmem $((B0 + 0x1000)) 32 0x1     # další offsety, krok 4 KiB
busybox devmem $((B4 + 0x0))  32 0x1       # BAR4
busybox devmem $B0 32                      # čtení: DIR_RD_DDR_DET?
setpci -s <bdf> 0x44.l=0x1                 # zápis do konfiguračního prostoru
```

Co zapsat do výsledků (pro každou akci hostitele):

- které bity se nastavily (`log`) a jestli v režimu `poll` (latch při masce), `open` (`gic 1`) a `irq` (ISR),
- jestli se něco změnilo ve slovech `regs` (adresa nebo data zápisu → doorbell registr),
- jestli `DIR_WR_DDR_DET` přichází na **každý** zápis, nebo jen na některé BARy nebo offsety (pak by sám mohl sloužit jako doorbell: zápis do vyhrazeného okna BARu).

Když `RC_DBELL_DET` nenastaví žádný zápis do BARu ani do konfiguračního prostoru, další kandidáti jsou zápisy do offsetů bloku `0x19000` přes BAR namapovaný na registry řadiče (inbound translace BARu na `0xd0070000`, `pcie debug bar`) a MSI zápisy hostitele na adresu, kterou EP sám nabízí.

## Stav

Přeloženo pro `espressobin/armada3720` (`epf_test`) s RC ovladačem (dynamicky) i bez něj (staticky). Na hardwaru zatím neověřeno.
