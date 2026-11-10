# Lab agent: desky si testy řídí samy, LLEXT na A3720

Dvě ESPRESSObiny (RC a EP) jsou na stejném ethernetovém segmentu. Místo přepínání mezi dvěma konzolemi a opisování kroků posílá jedna deska příkazy druhé. Patří k tomu:

- **`lab exec`**: shell příkaz na protější desce, výstup a návratový kód se vrátí sem;
- **`lab time`**: posun hodin mezi deskami (pro časování událostí na obou stranách);
- **`lab put`**: soubor do RAM slotu nebo na disk protější desky;
- **`lab ext`** / **`lab selftest`**: testy jako LLEXT moduly. Modul se přeloží jednou, pošle se a spustí bez přeflashování.

Stejný protokol umí i `scripts/lab/lab.pl` z linuxového stroje.

Obrazy: **`149-lab-llext-spi.bin`** a **`149-lab-llext-sata.bin`**. Je to stejný obraz pro RC i EP (`advk/mode`), sestavení `build-eb-all` + `-S espressobin-lab`. Commit `ac4cce0a0`.

> Jen pro laboratorní segment. Ochranu tvoří sdílený token a adresa protější desky, nic víc: kdo token zná, spustí na desce libovolný shell příkaz.

V příkladech je RC `192.168.10.1` a EP `192.168.10.2`. Dosaď svoje adresy (`net iface`).

## 1. První spuštění

| Krok | EP (192.168.10.2) | RC (192.168.10.1) |
|---|---|---|
| 1 | flash `149-lab-llext-*.bin`, boot | flash `149-lab-llext-*.bin`, boot |
| 2 | `lab token tajne123` | `lab token tajne123` |
| 3 | `lab peer 192.168.10.1` | `lab peer 192.168.10.2` |
| 4 | | `lab hello` → `role=ep id=… fw=… board=espressobin t=…` |
| 5 | | `lab time` → `offset … ns, best rtt … ns` |
| 6 | `lab selftest 20` → `selftest: PASS (20 runs …)` | `lab selftest 20` |
| 7 | | `lab exec lab selftest` (selftest EP, spuštěný z RC) |

Token a peer se ukládají do settings (NVS) a po rebootu zůstanou. Bez tokenu agent všechno zahazuje. Bez nastaveného peeru přijme požadavek odkudkoli, kdo zná token (to se hodí pro `lab.pl` z PC). `lab token -` / `lab peer -` hodnotu smaže.

Od teď se všechno dá dělat z jednoho shellu na RC.

## 2. Příklad: irqtest z jedné konzole

Posloupnost z [pcie-irqtest.md](pcie-irqtest.md), celá z RC:

| RC shell | co se stane na EP |
|---|---|
| `lab exec pcie debug irqtest arm irq dbell,wrddr` | zapne zachytávání |
| `pcie ls -v` | (zjistíš BAR0 EP, např. `0xe8000000`) |
| `devmem 0xe8000000 32 0x1` | zápis do BAR0 → EP možná zachytí `DIR_WR_DDR_DET` |
| `devmem 0xe8001000 32 0x1` | další offset |
| `lab exec pcie debug irqtest log` | výpis zachycených událostí z EP, přímo v konzoli RC |
| `lab exec pcie debug irqtest off` | konec |

Z PC (Perl, bez dalších modulů) to samé:

```sh
export LAB_TOKEN=tajne123
lab.pl 192.168.10.2 exec pcie debug irqtest arm irq dbell,wrddr
lab.pl 192.168.10.1 exec devmem 0xe8000000 32 0x1
lab.pl 192.168.10.2 exec pcie debug irqtest log
```

(Pro `lab.pl` musí mít deska `lab peer` nastavený na PC, nebo smazaný.) `exec` končí návratovým kódem vzdáleného příkazu, takže jde rovnou do shell skriptu nebo `make`.

## 3. Příkazy

| Příkaz | Co dělá |
|---|---|
| `lab status` | role, id (MAC), token/peer, počty obsloužených / zahozených požadavků, posun hodin |
| `lab token [<tajné>\|-]` | sdílený token (max 64 znaků, bez mezer), uloží se |
| `lab peer [<ipv4>\|-]` | protější deska, uloží se |
| `lab hello [<ipv4>]` | kdo tam je (role, MAC, verze firmware, board, čas od bootu) |
| `lab exec <příkaz…>` | spustí příkaz na protější desce; výstup do 16 KiB, timeout 30 s (`CONFIG_LAB_EXEC_TIMEOUT`) |
| `lab time [n]` | n výměn (výchozí 16), použije tu s nejkratším RTT; `peer = local + offset` |
| `lab put <slot\|/cesta> [<cíl>]` | pošle RAM slot nebo soubor (`/SD:/…`) protější desce; cíl bez `/` je RAM slot, s `/` soubor |
| `lab files` / `lab rm` | přijaté RAM sloty (8 slotů, 1 MiB) / smazání všech |
| `lab ext <slot\|/cesta> [args…]` | nahraje LLEXT modul, spustí `lab_ext_main`, uvolní ho; `PASS`/`FAIL n` |
| `lab selftest [n]` | totéž s modulem zabudovaným do firmware, n× za sebou |

Běží to na stejném „dummy“ shellu jako `ssh deska 'příkaz'`. Oba ho teď sdílejí přes zámek, takže se výstupy nepomíchají.

## 4. LLEXT na A3720: co ověřit

V QEMU (`qemu_cortex_a53`, MMU) prošlo:

- upstream `tests/subsys/llext`: 53/53;
- náš loader `tests/subsys/lab/ext`: 4/4 (běh, 5× reload s čistými `.data`/`.bss`, chybějící symbol, poškozený ELF).

Na hardwaru zatím neověřeno. Postup:

| Krok | Příkaz (kterákoli deska) | Očekávání |
|---|---|---|
| 1 | `lab selftest` | `lab hello ext: argc 1, argv[0] lab_hello`, `… 0 failures`, `selftest: PASS` |
| 2 | `lab selftest 100` | 100× load/run/unload bez úniku haldy a bez pádu (MMU práva se nastavují a vrací pokaždé) |
| 3 | `llext list` | prázdný seznam: modul se po každém běhu uvolnil |
| 4 | z PC: `lab.pl 192.168.10.2 run lab_hello.llext` | modul poslaný po síti, `PASS`, exit 0 |
| 5 | `lab put lab_hello.llext /SD:/lab_hello.llext` (z RC), pak `lab exec lab ext /SD:/lab_hello.llext` | načtení z FAT |

Při chybě oprávnění MMU (text bez X, nebo data s X) spadne krok 1 na synchronní výjimku. Do výsledku pak zapiš ESR/FAR z výpisu.

`lab_hello.llext` (4 KiB) posílám spolu s obrazy. Vzniká v `build-eb-all/llext/`.

Co modul testuje:

- relokace: volání do firmware (`R_AARCH64_CALL26` na `printk`), `ADR_PREL_PG_HI21`/`ADD_ABS_LO12` na rodata a data, `ABS64` v tabulce ukazatelů na funkce;
- nulovou `.bss` a inicializovaná `.data` při každém načtení;
- CRC32 (kontrolní hodnota `0xcbf43926`);
- `busy_wait` 1 ms proti `now_ns`;
- alokaci se zarovnáním a operace s cache.

## 5. Vlastní testovací modul

Kernel exportuje modulům skoro jen `printk`. Všechno ostatní dostane modul tabulkou `struct lab_ext_api` (`include/zephyr/lab/lab.h`). Díky tomu modul přeložený jednou funguje s dalšími buildy firmware, dokud sedí `LAB_EXT_API_VERSION`.

```c
#include <zephyr/lab/lab.h>
#include <zephyr/llext/symbol.h>

int lab_ext_main(const struct lab_ext_api *api, int argc, char **argv)
{
	volatile uint32_t *advk = api->map_io(0xd0070000, 0x20000);
	char out[256];

	api->printf("role %s, LTSSM 0x%02x\n", api->role(),
		    (advk[0x6000 / 4] >> 24) & 0x3f);	/* LMI CFG_REG */
	/* na protější desce */
	api->peer_exec("pcie debug irqtest arm irq dbell", out, sizeof(out), 2000);
	/* ... */
	return 0;	/* 0 = PASS */
}
EXPORT_SYMBOL(lab_ext_main);
```

Tabulka:

| Člen | Co dělá |
|---|---|
| `printf` | výpis |
| `now_ns` | čas |
| `busy_wait_us`, `sleep_ms` | čekání |
| `local_exec` | shell příkaz na této desce |
| `peer_exec` | shell příkaz na protější desce |
| `peer_offset_ns` | posun hodin z `lab time` |
| `role` | `rc` / `ep` |
| `map_io` | registry, nekešované |
| `cache_flush`, `cache_invd` | údržba cache |
| `alloc`, `free` | paměť |

Překlad: v CMakeLists aplikace `add_llext_target(...)` jako u `lab_hello` (viz `samples/.../netshell/CMakeLists.txt`). Samostatně bez stromu aplikace to půjde přes LLEXT EDK (`west build -t llext-edk`), to je další krok.

Omezení:

- jméno modulu max 15 znaků (`LLEXT_MAX_NAME_LEN`);
- RAM slot max 23 znaků;
- typ ELF je relocatable object (výchozí pro ARM64);
- halda LLEXT 1 MiB (`CONFIG_LLEXT_HEAP_SIZE=1024`).

`local_exec` z modulu spuštěného přes `lab exec` přepíše výstup, který se zrovna vrací protější desce: používej ho jen v modulech spouštěných lokálně.

## 6. Protokol

UDP 7777, jeden požadavek = jeden datagram: textová hlavička, `\n`, volitelně binární data.

```
požadavek  LAB1 <token> <op> <id> [args]\n[data]
odpověď    LAB1 <op> <id> [args]\n[data]

HELLO                         -> HELLO role=.. id=.. fw=.. board=.. t=<ns>
EXEC <příkaz…>                -> OUT <seq>\n<1200 B textu> …  END <rc> <počet OUT> <délka>
TIME <t1>                     -> TIME <t1> <t2> <t3>
PUT <jméno> <off> <celkem>\n<≤1024 B>   -> ACK <off> <chyba>
```

- Klient opakuje požadavek se stejným `id` (3×). Server odpoví z cache, takže se příkaz nikdy nespustí dvakrát.
- Když chybí nějaký `OUT`, klient pošle požadavek znovu a dostane celý výstup ještě jednou.
- Server je jedno vlákno: příkazy se vykonávají po jednom.
- Požadavky se špatným tokenem nebo z jiné adresy než `lab peer` se tiše zahodí (`lab status`: `dropped`).

Testy:

- `tests/subsys/lab/agent` (native_sim/64, offloaded sockets, agent mluví sám se sebou přes 127.0.0.1): hello, malý a velký výstup (9 datagramů), návratové kódy, čas, PUT 5000 B a výměna slotu, zahození z cizí adresy, chybějící token. 8/8.
- `lab.pl` ověřen proti stejnému procesu (`--rt`): hello 3,4 ms, `exec spam 300`, rc, time, put 3000 B za 10 ms, špatný token.

## 7. Další kroky

- `irqtest.llext`: celý průzkum doorbellu jako jeden modul spuštěný na RC. Ten řídí EP přes `peer_exec` a sám zapisuje do BARů.
- LLEXT EDK pro překlad modulů mimo strom.
- `lab run` přímo na desce (put + ext na protější straně jedním příkazem). Z PC už to umí `lab.pl … run`.
