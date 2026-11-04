# ESPRESSObin (Armada 3720): vNTB endpoint, NTB mezi hostem a Zephyrem

Stav k 2026-11-04. Zephyr větev `espressobin` (nad `e39b714b2`), commity:

| commit | obsah |
|---|---|
| `3ca84197f` | aardvark: BAR2 jako 64bit paměťový BAR |
| `389ef5574` | `include/zephyr/drivers/ntb.h` (NTB API) + shell `ntb` |
| `097f13c40` | funkce `zephyr,pcie-epf-vntb` (`drivers/pcie/endpoint/functions/pcie_epf_vntb.c`) |
| `66e7ccdf5` | test `tests/drivers/pcie/endpoint/epf_vntb` |
| `8649deab5` | snippet `espressobin-vntb` |

Vzor: Linux `drivers/pci/endpoint/functions/pci-epf-vntb.c` (Frank Li, NXP) a na hostu `drivers/ntb/hw/epf/ntb_hw_epf.c`, oboje z master @ `05441d6` (7.4-rc1).

**Ověřeno:** test na `native_sim` i `native_sim/native/64` (9 případů, scénář se shellem včetně echo), všechny endpoint testy 75/75, build netshell router + DSA + PCIe EP + vNTB pro `espressobin/armada3720/smp` bez varování, checkpatch 0 chyb (kromě chybějícího Signed-off-by). **Na HW netestováno, BAR2 na A3720 taky ne.**

## 1. Co to je

```
  host (Linux, BeagleY-AI)                 ESPRESSObin (Zephyr)
  ntb_tool / ntb_transport + ntb_netdev    ntb shell / klient NTB API
  ntb_hw_epf  ──── PCIe ────  Aardvark EP ─ zephyr,pcie-epf-vntb
```

Linuxový vNTB dělá na EP straně virtuální PCI sběrnici s „pci-vntb“ zařízením, aby na EP běžel stejný `ntb_transport`. Zephyr NTB vrstvu nemá, takže funkce rovnou implementuje nové **NTB API** (`zephyr/drivers/ntb.h`): link, scratchpady (vlastní / peer), doorbelly, inbound okna (host píše do naší paměti) a outbound okna (my píšeme do paměti hosta). API je ořezané Linuxové `ntb_dev_ops`: jeden peer, bez masek doorbellů, callbacky z threadu. Navíc `ntb_mw_get_local()` (paměť za inbound oknem dává provider, carveout) a `ntb_peer_mw_xfer()` (kopie do okna hosta přes DMA).

## 2. BARy a registry

`ntb_hw_epf` bere rozložení BARů z tabulky podle vendor/device ID (class musí být `05:00`):

| ID | config | doorbell | okna | pro A3720 |
|---|---|---|---|---|
| `1957:0809` (i.MX8) | BAR0 (+ peer spad) | BAR2 | BAR4, BAR5 | **ano** (BAR0/2/4 64bit) |
| `1912:0030` (R-Car) | BAR0 (+ peer spad) | BAR4 | BAR2 | ano |
| `104c:b00d` (J721E) | BAR0, peer spad BAR1 | BAR2 | BAR2..5 | ne (doorbell a okno v jednom BARu) |

Obě použitelné varianty potřebují tři BARy, proto aardvark nově nabízí **BAR2 jako 64bit paměťový** (LMI `0x6244` bit 17 = 64bit, bit 28 = I/O, inbound slot `0x5808`). Do teď byl vypnutý. Na desce neověřené; když nepůjde, zbývá jen jiná tabulka v `ntb_hw_epf` (patch hosta).

Snippet `espressobin-vntb` (i.MX8), vše v carveoutu `0x10000000` (32 MiB), největší první, zarovnané na velikost a na 64 KiB:

| BAR | obsah | velikost |
|---|---|---|
| BAR4 | okno 0 (host píše do DDR desky) | 4 MiB @ `0x10000000` |
| BAR0 | control (0x134 B), MSI-X tabulka (8 × 16 B + PBA), scratchpady hosta (64), naše (64) | 1 KiB @ `0x10400000` |
| BAR2 | 32 doorbell slotů po 4 B | 128 B @ `0x10410000` |

Control oblast = Linuxová `struct epf_ntb_ctrl`: `command 0x00`, `argument 0x04`, `command_status 0x08` (u16), `link_status 0x0a`, `addr 0x10`, `size 0x18`, `num_mws 0x20`, `mw1_offset 0x24` (0), `spad_offset 0x28`, `spad_count 0x2c`, `db_entry_size 0x30` (4), `db_data[32] 0x34` (n + 1), `db_offset[32] 0xb4` (0).

Příkazy hosta (polling po `CONFIG_PCIE_EPF_VNTB_POLL_US`, 1 ms; Linux 5 ms; host čeká 1 s): `CONFIGURE_DOORBELL` (bit 16 MSI-X, dolních 16 bitů počet vektorů), `TEARDOWN_DOORBELL`, `CONFIGURE_MW` (addr/size → `pcie_epc_map_host()`), `TEARDOWN_MW`, `LINK_UP`, `LINK_DOWN`. Neznámý příkaz vrátí ERROR (Linux nevrátí nic a host čeká na timeout).

Přerušení (1-based vektory k hostu): 1 = link, 2 = nepoužitý (historicky), 3.. = doorbell 0... Doorbell n od hosta = slot n + 2 v BAR2. `db-count = 8` → 6 doorbellů v každém směru, MSI 8, MSI-X 8. Host chce aspoň 4 vektory.

Link je „up“, až když ho zapnou obě strany (`LINK_UP` od hosta a `ntb_link_enable()` / `ntb link up`); změna jde k hostu vektorem 1 a lokálně callbackem.

Reset (link down, hot reset, FLR, PERST#): odmapují se okna hosta, zapomene se počet vektorů, host link = off, smažou se příkazy, sloty a scratchpady hosta. BAR paměť restart hosta přežije, takže by tam jinak zůstal starý stav.

Bez BME: `ntb_peer_db_set()` i `ntb_peer_mw_xfer()` vrátí `-EACCES` (outbound bez BME = SError na A3720).

## 3. Build

```sh
west build -b espressobin/armada3720/smp -S espressobin-dsa -S espressobin-pcie-ep -S espressobin-vntb \
  samples/boards/globalscale/espressobin/netshell -- -DEXTRA_CONF_FILE="router.conf;pcie-ep-uartlog.conf"
boards/globalscale/espressobin/support/mkflash.sh build/zephyr/zephyr.bin   # CLOCKS=CPU_1000_DDR_800
```

`espressobin-vntb` musí jít za `espressobin-pcie-ep` (smaže `epf_test0`, používá `pcie_bar_mem`).

## 4. Test na desce

Host (kernel s `CONFIG_NTB=m CONFIG_NTB_EPF=m CONFIG_NTB_TOOL=m`, případně `CONFIG_NTB_TRANSPORT CONFIG_NTB_NETDEV`). Po změně BARů restart hosta (okna mostů za ASM1806 se počítají při bootu).

```sh
lspci -nn -d 1957:0809 -vv          # BAR0 1K, BAR2 128, BAR4 4M (64bit), MSI-X 8
modprobe ntb_hw_epf                  # dmesg: ntb_hw_epf ... ; /sys/bus/ntb/devices/
modprobe ntb_tool
D=/sys/kernel/debug/ntb_tool/0000:05:00.0
echo Y > $D/link                     # host LINK_UP
```

EP konzole:

```
vntb                                 # BARy, control oblast, vektory, statistiky
ntb link up
ntb status                           # link up, okno 0 4096 KiB
ntb echo on                          # doorbell n -> peer spad 0 = spad 0 + 1, doorbell n zpět
```

Na hostu:

```sh
cat $D/peer0/link                    # Y
echo 41 > $D/peer0/spad0            # do našeho spad 0
echo 's 1' > $D/peer_db             # doorbell 0 -> slot 2
cat $D/spad0                        # 0x2a (echo)
cat $D/db                           # 0x1
# okno hosta (náš peer mw): host alokuje buffer a pošle CONFIGURE_MW
echo 65536 > $D/peer0/mw_trans0
# EP: ntb peer_mw 0 fill 0x1000 ; ntb peer_mw 0 crc
head -c 32 $D/peer0/mw0 | xxd
# naše okno (BAR4): host píše, EP čte
echo Hello > $D/peer0/peer_mw0
# EP: ntb mw 0 rd 0 16
```

## 5. Otevřené

- BAR2 na A3720 v paměťovém režimu (velikost z LMI, inbound `0x5808`) na HW.
- `ntb_transport` + síťové rozhraní kompatibilní s `ntb_netdev` nad NTB API (QP v okně, handshake ve scratchpadech).
- Doorbelly pollingem: latence ≤ 1 ms. Interrupt by šel jen s inbound zápisem do registru, který budí GIC (Aardvark v EP takový nemá).
- Okno hosta jde přes OB okno (16 MiB celkem, granule 64 KiB). MSI doorbell drží jednu stránku.
- `ntb_perf` a `ntb_pingpong` (pingpong potřebuje port number, `NTB_TOPO_NONE` ho nemá).
