# pcivsec.pl: vendor-specific config space (VSEC) PCIe switchů

Linuxový nástroj pro host (Perl, jen standardní moduly, root): najde VSEC a DVSEC capability, vypíše, uloží, porovná a hlídá libovolnou část 4 KiB config space funkce. Dekóduje i to, co víme o ASMedia ASM1184e. `lspci` u VSEC vypíše jen hlavičku (`<?>`).

Ve stromu: `scripts/pcie/pcivsec.pl`.

```sh
pcivsec.pl ls                              # funkce s VSEC/DVSEC
pcivsec.pl dump  01:00.0 300 c00           # hex dump VSEC upstream portu ASM1184e
pcivsec.pl snap  01:00.0 up-none.bin       # uložit config space
pcivsec.pl diff  up-none.bin 01:00.0       # co se změnilo (soubor nebo funkce)
pcivsec.pl watch 01:00.0 10 300 c00        # změny dwordů za běhu (10 ms)
pcivsec.pl ports 01:00.0                   # stav portů switche (ASM1184e)
pcivsec.pl ports 01:00.0 watch 20          # ... průběžně, s časem
```

`watch` nejdřív sekundu čte rozsah a slova, která se mění sama, z výpisu vynechá.

## ASM1184e (`1b21:1184`): co jsme vyčetli

Upstream port má VSEC na `0x300`, `ID=0000 Rev=0 Len=c00`. Následující je **odhad z pěti snímků** (2026-11-12): prázdný switch, jedna deska na různých portech, deska v L0, deska, která nenaběhne.

| VSEC | Význam (odhad) |
|---|---|
| `0x400 + 0x80·n`, n = 0…3 | blok portu 1 / 3 / 5 / 7 |
| blok `+0x4c` bajt 3 | příznaky trénování: `0x21` prázdný port, `0x6d` přijímač detekovaný, bez zámku, `0x7f` L0, `0x13` link byl a spadl |
| blok `+0x4c` bajt 2 | `0x08` bez linku, `0x0b` po L0 |
| blok `+0x50` | `00000b00` bez linku, `07000700` L0, `06000f00` po pádu linku |
| `0x3d8` bajt 3, `0x3dc` | globální stav (`0x03`, `0x2f`), nejasné |

Rozdíl `0x6d` proti `0x7f`: chybí bity 1 a 4. Switch desku detekuje, ale nezachytí její TS. To odpovídá ESPRESSObinu, který jako EP visí v `polling-active`, protože PHY nezachytí data v žádném směru (hodiny/PLL, viz `lab-debug-modules.md`).

Ověřit a zpřesnit: `ports … watch` při startu desky ukáže pořadí, ve kterém příznaky naskakují.

## Další switche

- **PLX/Broadcom PEX87xx (PEX8747):** registry portů jsou v config space každého portu (a přes BAR0 upstream portu). `dump`/`diff`/`watch` na bdf portu fungují hned. Dekodér přibude podle data booku nebo snímků.
- **ASM1806:** jiný VID:PID, VSEC zatím neprozkoumaný; stejný postup snímků.

Až budou pole ověřená, je plán patch do pciutils (`ls-ecaps.c`, `cap_evendor()` s rozlišením podle výrobce, jen `-vv`, označené jako reverse-engineered).
