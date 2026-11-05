# Networking přes PCIe fabric – brainstorming

Cíl: síťové rozhraní nad PCIe fabricem (RC + switch + EP), se kterým jdou použít běžné nástroje (`ip`, `tc`, `ethtool`, `iperf`/`zperf`, `ptp4l`). Data mají jít přes DMA, ideálně bez kopírování. Uzly jsou Linux, Zephyr a ThreadX (NetX Duo). Pozn.: ThreadX dnes spadá pod Eclipse Foundation, ne pod LF. Všechno se řídí tím, co umí switch, EP a RC. Navazuje na `pcie-fabric.md` (karty, regiony, epochy) a `pcie-aer.md`.

## 1. Proč to na PCIe sedí

- PCIe je paketová: TLP (Memory Write), tedy bezztrátová linková vrstva s LCRC, retry a credit flow control, volitelně ECRC end-to-end. Proti Ethernetu odpadá ztrátovost a „best effort“.
- Posted zápis nepotřebuje odpověď a posted zápisy po stejné cestě drží pořadí. Model „data, pak příznak“ funguje bez čtení přes linku (stejně jako dnešní fabric).
- Kontrolní součet jde vypustit: PCIe má LCRC na každém hopu, s ECRC i end-to-end. Netdev tak může hlásit `CHECKSUM_UNNECESSARY` (jako veth nebo virtio s `GUEST_CSUM`).
- MTU nemá ethernetový strop: rámec je jen pole bajtů v cizí paměti. 64 KiB rámec znamená málo deskriptorů, málo přerušení a vysoké `iperf` čísla i na x1 Gen2.

## 2. Co už existuje (a proč to nestačí)

| Cesta | Linux strana | Pro | Proti |
|---|---|---|---|
| **ntb_transport + ntb_netdev** přes naše vNTB | stock `ntb_netdev` na RC | máme vNTB, `ip`/`iperf` hned | jen RC↔EP; kopie do memory window; formát je „Linux interní“ |
| **virtio-net device na EP** | stock `virtio_net` na RC | žádný nový ovladač na RC; ethtool, multiqueue, GSO, `virtio_net_hdr` | asymetrické (RC = driver, EP = device), jen RC↔EP; EP čte kruhy hosta přes linku (non-posted, latence); v Linuxu EP strana jen jako RFC (pci-epf-vnet), v Zephyru device strana chybí |
| **MHI (`mhi_net`)** | stock, Qualcomm modemy | hotový přenos přes EP | specifické pro modemy, asymetrické |
| **Vlastní `pfab-net`** nad fabricem | nový ovladač | mesh (EP↔EP přímo), jen zápisy, zero-copy podle schopností, multicast, PTM | všechno napsat: Linux, Zephyr, ThreadX |

Návrh: **cíl je `pfab-net`**. Rychlé demo uděláme přes ntb_netdev (vNTB už je hotové). virtio-net na EP je volitelná „kompatibilní“ cesta pro hostitele, kteří o fabricu nic nevědí.

## 3. Schopnosti, ze kterých se skládá datová cesta

Každý uzel v info bloku nabídne síťové schopnosti. Manager k nim přidá schopnosti switchů na cestě a pro **každou dvojici uzlů** zvolí režim a zapíše ho do karty.

### Endpoint

| Schopnost | Proč | Příklady |
|---|---|---|
| DMA engine (směry, kanály, max. velikost, scatter-gather) | zero-copy a výkon; kdo ho má, ten tlačí (push) nebo tahá (pull) | A3720: XOR engine (bez SG po paketech, write-back pozdě); DWC eDMA (LS1012A?, K1?); J7: UDMA |
| Inbound mapování: jen BAR / podrozsahy BARu / dynamické | kam smí peer zapsat → jestli lze přijímat přímo do síťových bufferů | A3720: BAR přes LMI + inbound okna `0x5a00` (neověřeno); Linux 7.3: `dynamic_inbound_mapping`, `subrange_mapping` |
| Velikost outbound okna | kolik peerů nebo bufferů jde naráz namapovat | A3720 EP: 16 MiB / 64 KiB granule |
| Doorbell (inbound zápis → IRQ na EP) | bez pollingu | Linux 7.3: `pci_epf_doorbell` (MSI přes ITS nebo „embedded“); A3720: GICv3 ITS – ověřit, jestli zápis do BARu namapovaného na `GITS_TRANSLATER` vyvolá LPI |
| MSI/MSI-X k RC | notifikace RC | všude |
| Vystavení DMA engine peerovi (aux resource) | RC nebo peer řídí DMA engine EP (remote eDMA) | Linux: `PCI_EPC_AUX_*` |
| PTM requester | čas pro PTP | A3720 ne; ověřit J722S, K1, LS1012A |
| Gen/šířka, MPS, MRRS | rychlost linky pro `ethtool`, velikost TLP | A3720: Gen2 x1, MPS 128 B nastavené, DevCap 512 B |

### Switch

| Schopnost | Využití |
|---|---|
| ACS (P2P Request/Completion Redirect) | vypnout → P2P se otočí ve switchi (bez IOMMU izolace stejně nic nechrání) |
| **Multicast capability** (ASM1806) | broadcast a multicast rámce: jeden zápis do MC okna se zreplikuje na členské porty (ARP, ND, mDNS, PTP announce) |
| **VC / TC** (ASM1184 má VC cap) | `tc mqprio` → PCIe Traffic Class → arbitráž VC ve switchi = skutečné QoS přes fabric (TSN směr) |
| DPC, AER, hot plug | carrier down a izolace uzlu, `ip link` stav, health reporter |
| Cut-through, MPS cesty | odhad latence; MPS = minimum po cestě |

### Root complex

| Schopnost | Využití |
|---|---|
| Inbound do celé DRAM | RC přijímá zero-copy do čehokoli (A3720 ano, 1:1) |
| MSI controller | EP → RC notifikace |
| IOMMU (SMMU) | izolace a mapování bufferů (A3720 nemá; K1, J722S, RK3399 mají – P2P přes RC pak jen s ATS nebo identitním mapováním) |
| P2P přes root port | A3720 ne → P2P jen přes switch |
| PTM root | časová doména pro PHC |
| DMA engine na RC | většinou žádný pro PCIe → RC tlačí CPU zápisy (WC), nebo za něj pracuje DMA EP |

## 4. Transport `pfab-net` (jen zápisy, každý čte jen svou paměť)

Pro každou dvojici a směr A→B dvě fronty, obě v paměti toho, kdo je čte:

```
v paměti B (přijímač):   TX completion ring   ← A zapisuje: {buf_id, len, flags, gso, ts}
v paměti A (odesílatel): RX free-buffer ring  ← B zapisuje: {buf_id, bus_addr, len}
                         credits / doorbell index
```

1. B předem publikuje volné buffery (bus adresy svých síťových bufferů) do A.
2. A vezme buffer, **DMA zapíše rámec přímo z vlastního skb nebo net_bufu do bufferu v B** (DMA čte lokálně a zapisuje přes linku = posted).
3. Po dokončení DMA A zapíše completion do B. Ten musí jít po datech: stejný DMA kanál, fence, nebo u A3720 XOR čekat na write-back (známý problém s pořadím MSI z epf_test).
4. Notifikace: MSI (B = RC), doorbell (B = EP s doorbellem), nebo polling s moderací (`ethtool -C`).
5. B předá rámec do stacku bez kopie a doplní free ring.

Zero-copy podle schopností dvojice (rozhoduje manager):

| B (přijímač) | Inbound | Režim |
|---|---|---|
| RC | celá DRAM | **zero-copy** do page_pool / net_buf |
| EP s dynamickým nebo podrozsahovým inbound | stránky page_pool namapované do BARu | **zero-copy** (Linux 7.3 EP) |
| EP jen s pevným BARem (A3720) | buffer pool *v* BAR carveoutu | **zero-copy v Zephyru a ThreadX** (pool alokuje přímo v carveoutu); v Linux EP 1 kopie (bounce) |

Kdo DMA nemá (RC): pro směr RC→EP lze použít **pull**. EP publikuje „TX request ring“ v RC paměti… což ale znamená čtení přes linku. Lepší je **remote DMA**: RC dostane od EP okno do jeho DMA engine (aux resource) a spouští ho sám. Fallback je CPU zápis přes WC mapování BARu.

## 5. Netdev model

- **Jeden netdev na uzel** (`pfab0`) jako virtuální L2 segment: tabulka MAC → node ID z karty (MAC = locally administered z DSN).
  - Unicast jde přímo na peera.
  - Broadcast a multicast jdou přes PCIe multicast, pokud ho switch má, jinak se replikují N−1 zápisy.
  - Bridge, routing, VLAN fungují nad tím normálně.
- Multiqueue: fronta na každou dvojici, případně i na CPU. `ethtool -l/-L` kanály, `-g` velikost ringů, `-S` statistiky po peerech a frontách, `-C` moderace doorbellů, `ethtool` link = Gen × šířka (× počet peerů?).
- Offloady:
  - checksum vypnout (`CHECKSUM_UNNECESSARY`);
  - **GSO/GRO jako virtio**: v completion jde `gso_type`/`gso_size`, takže se po fabricu posílají 64 KiB „super-rámce“ a segmentuje se až na hraně k fyzickému Ethernetu;
  - RX timestamp v completion.
- `tc`: `mqprio` → PCIe TC → VC ve switchi (pokud je). Jinak aspoň prioritní fronty a pořadí obsluhy.
- `devlink` (Linux): fabric na úrovni uzlu (role, epocha, schopnosti, health reporter z AER/DPC).
- Carrier: karta RUNNING a link up → carrier on. Stažení karty, AER fatal nebo DPC → carrier off.

## 6. Čas (PTP)

- **PTM** (pokud RC i EP umí) dává synchronní čas RC↔EP po linkách. Netdev pak vystaví PHC (`ethtool -T`, `ptp4l`, `phc2sys`) a hardware timestamp rámce bere čas DMA dokončení přepočtený do PTM domény. Zephyr už má PTM na host straně (`drivers/pcie/host/ptm.c`).
- Bez PTM: dvoucestný přenos času po fabric zprávách (jako PTP, ale s časem zápisu slotu), přesnost omezená pollingem nebo IRQ. Pro AVB/TSN směr je to zajímavé spojení s VC/TC výše.

## 7. Implementace po OS

| | Linux | Zephyr | ThreadX / NetX Duo |
|---|---|---|---|
| Uzel fabricu | `pci-epf-fabric` (EP), manager (RC, userspace nebo modul) | `pcie_epf_fabric`, manager | port `pfab_core` (C, bez závislostí) |
| Síťový ovladač | netdev + page_pool, NAPI, dmaengine | ethernet L2 driver, `net_pkt` s net_bufy v BAR carveoutu, DMA přes `pcie_epc_xfer_dma` | `nx_driver`, NX_PACKET pool v BAR carveoutu |
| Nástroje | ip, tc, ethtool, iperf3, ptp4l | `net iface`, `net stats`, **zperf** (iperf2 kompatibilní), gPTP | NetX Duo iperf, ping |
| Zero-copy RX | RC: ano; EP: s dynamickým inbound | ano (pool v carveoutu) | ano (pool v carveoutu) |

Společné jádro protokolu (ringy, kredity, karty) by mělo být jedno C (`pfab_core` bez OS závislostí + tenké OS vrstvy), aby se Linux, Zephyr a ThreadX nerozešly. Formát na lince se drží v jedné hlavičce (jako dnes `pfab_wire.h`).

## 8. Úskalí

- **Pořadí DMA a completion:** A3720 XOR potvrdí dřív, než data projdou linkou. Pro completion je potřeba stejný kanál, read-back, nebo zápis completion samotným DMA za daty.
- **Relaxed Ordering:** zrychlí DMA, ale completion pak smí předběhnout data. Buď RO jen pro data a completion bez RO, nebo RO vypnout.
- **Koherence:** EP carveout je nekešovaný (pomalý pro stack). Lepší je kešovaný pool s invalidací po DMA, případně CCI, pokud je IO koherentní (A3720 test koherence je otevřený).
- **Polling vs. IRQ:** bez doorbellu na EP busy polling (jedno jádro) nebo moderace. Doorbell přes ITS na A3720 stojí za ověření.
- **O(N²) fronty:** při 16 uzlech a 2 frontách na směr je to v pořádku. Při větším počtu sdílené RX fronty s ID odesílatele.
- **MTU:** Zephyr a ThreadX mají menší limity (Zephyr NET_ETH_MTU). Manager vyjedná minimum po dvojicích, GSO metadata to obejdou.
- **Bezpečnost:** bez IOMMU může každý uzel psát do bufferů jiných. Pro lab OK, jinak IOMMU na RC a P2P přes RC s ATS.

## 9. Postup

1. **Demo:** Linux RC `ntb_netdev` ↔ Zephyr vNTB s ntb_transport-kompatibilní L2 vrstvou → `ip`, `iperf` hned (kopie, jen RC↔EP).
2. **pfab-net v1** (Zephyr EP + Zephyr RC): jeden netdev, push s DMA (XOR), pool v carveoutu, polling → `zperf`.
3. **Linux netdev** nad `pci-epf-fabric` (EP) a nad managerem (RC): page_pool, NAPI, ethtool, GSO metadata → `iperf3`, `tc`.
4. Schopnosti v info bloku a výběr režimu po dvojicích v manageru (DMA, inbound, doorbell, MTU, MC, TC).
5. PCIe multicast (ASM1806) pro broadcast, VC/TC mapování pro `tc mqprio`.
6. PTM → PHC, PTP.
7. ThreadX/NetX Duo driver.
8. Volitelně virtio-net device na EP pro hostitele bez našeho ovladače.

## Otevřené otázky

- Má DWC eDMA na LS1012A/K1? (určí, jestli DWC EP bude push i pull).
- Inbound okna A3720 `0x5a00…` v EP módu: podrozsahy BARu → dynamické mapování RX bufferů i na A3720?
- ITS doorbell na A3720 EP: jaké DeviceID vidí ITS u inbound zápisu z PCIe?
- PTM a VC na J722S, K1, LS1012A, ASM1806 (`pcie cap ls`).
