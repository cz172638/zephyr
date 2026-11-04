# ESPRESSObin (Armada 3720): NVMe endpoint, SATA disky jako NVMe namespacy

Stav k 2026-11-04. Zephyr větev `espressobin`, commity `97f6771db` (funkce), `bc71c3b9a` (test), `a27ce9c6c` (snippet), nad `eb6f061cf`. Vzor: Linux `drivers/nvme/target/pci-epf.c` (nvmet-pci-epf, Damien Le Moal / Rick Wertenbroek, od 6.14).

**Ověřeno:** test `tests/drivers/pcie/endpoint/epf_nvme` na `native_sim/native/64` (10/10), build `netshell` (router + DSA + PCIe EP + SATA + NVMe target) pro `espressobin/armada3720/smp` bez varování, checkpatch 0 chyb (kromě chybějícího Signed-off-by). **Na HW netestováno.**

## 1. Co to je

Řadič Aardvark v EP módu se hostu ukáže jako NVMe řadič (class `01:08:02`, NVMe 1.4). Na hostu se naváže standardní `nvme` driver podle class code, bez `new_id`. Každý disk z `disk-names` je jeden namespace:

| NSID | disk | kde |
|---|---|---|
| 1 | `sata0` | SATA port (nebo PM port 0) |
| 2–5 | `sata1`–`sata4` | porty port multiplieru |

Chybějící disk je neaktivní namespace (Identify NS vrací nuly, v Active NS listu není). Při dalším Identify se hledá znovu, takže disk, který naběhne později, se objeví po `nvme ns-rescan` na hostu.

## 2. Srovnání s Linuxem

| | Linux nvmet-pci-epf | Zephyr `zephyr,pcie-epf-nvme` |
|---|---|---|
| BAR0 | registry, doorbelly (stride 4 B), MSI-X tabulka | totéž; tabulka na `0x2000`, BAR0 16 KiB, 64bit |
| CC | polling 10 ms | polling `PCIE_EPF_NVME_CC_POLL_MS` (10 ms) |
| SQ | polling, round robin, burst 8, busy poll 5 s | totéž, burst z Arbitration feature, busy poll 100 ms, pak 1 ms |
| příkazy | nvmet core (bdev/file backend) | vlastní: admin + NVM read/write/flush/write zeroes nad `disk_access` |
| paralelismus | workqueue na SQ | jeden thread, příkaz po příkazu |
| data | PRP + SGL, dmaengine nebo memcpy přes OB | jen PRP (SGLS = 0), `pcie_epc_xfer_dma()` (XOR) nebo memcpy |
| CQ | mapovaná trvale, `memcpy_toio` | mapuje se pro každou CQE, zápis po dwordech, phase poslední, pak non-posted read |
| IRQ | coalescing s prahem 8 + na konci dávky | jedno IRQ na dávku z SQ, admin hned |
| po shutdownu | `CC.EN` v BARu se smaže | totéž (a i po chybné konfiguraci, CFS) |

Shutdown i chybná konfigurace: host po `CC.EN=0` hned vidí `RDY=0` a může `EN` nastavit znovu dřív, než to funkce zachytí. Bez smazání `EN` v BARu by to nevypadalo jako přechod 0 → 1. Test na tohle narazil.

## 3. Implementované příkazy

**Admin:** Identify (CNS 0 NS, 1 controller, 2 active list, 3 descriptors: NGUID + CSI, 5/6 pro CSI NVM = nuly), Get Log Page (error = nuly, SMART, firmware slot), Get/Set Features (arbitration, power mgmt, temp threshold, error recovery, VWC, number of queues, IRQ coalescing, IRQ config, write atomicity, async event config; SEL 0–3, SV → not saveable), Create/Delete I/O SQ/CQ, Abort (vždy „not aborted“), AER (drží se, nikdy se nedokončí, zahodí se při resetu), Keep Alive.

**NVM:** Read, Write, Flush (NSID `0xffffffff` = všechny), Write Zeroes (`ONCS` bit 3). DSM/discard ne.

**Identify Controller:** VID/SSVID z hlavičky, SN = DSN funkce v hexu (nebo `serial-number`), MN z DT, FR = verze Zephyru, MDTS z `PCIE_EPF_NVME_MDTS_KB`, CNTLID 1, CNTRLTYPE I/O, ACL 3, AERL 3, NN = počet disků, VWC = `PCIE_EPF_NVME_VWC`, SUBNQN `nqn.2014-08.org.zephyrproject:nvme:<SN>`, jeden power state 5 W.

**CAP:** MQES z `max-queue-entries`, CQR, TO z `PCIE_EPF_NVME_READY_TIMEOUT_MS`, CSS NVM, MPSMIN = MPSMAX = 4 KiB, DSTRD 0, bez NSSR/BPS/CMB/PMR.

**NGUID** = DSN (EUI-64) + 4 B nul + NSID. EUI-64 namespacu není, musel by se vymyslet.

## 4. Bezpečnost vůči Aardvarku

- **BME:** bez Bus Master Enable vede outbound na A3720 k SError. Thread proto před každým průchodem čte Command registr a bez BME na frontu nesáhne, polluje jen CC.
- **Reset:** `LINK_DOWN`, `HOT_RESET`, `FLR`, `PERST_ASSERT` ⇒ vše zpět do stavu po zapnutí. CC, CSTS a fronty se vynulují, CAP a VS se zapíšou znovu. BAR v DDR přežije restart hosta, starý `CC.EN` nesmí zůstat.
- **Class code:** funkce ho vynutí na `010802` bez ohledu na DT. Uložený override `pcie controller id` s class ho ale přepíše a funkce na to varuje v logu. Pak `pcie controller id reset`.
- **BAR4** (po resetu zapnutý) funkce vypne, host vidí jen BAR0.
- **Pořadí DMA → CQE → MSI:** data k hostu jdou přes XOR s read-back tail flush (`a3700_ep_xfer_dma`). CQE zapisuje CPU s non-posted readem a MSI jde až po něm.

## 5. Build

```sh
west build -b espressobin/armada3720/smp samples/boards/globalscale/espressobin/netshell \
  -S espressobin-dsa -S espressobin-dsa-v5 \
  -S espressobin-pcie-ep -S espressobin-sata -S espressobin-nvme-target \
  -- -DEXTRA_CONF_FILE="router.conf;pcie-ep-uartlog.conf" -DNETSHELL_AUTHORIZED_KEY=~/.ssh/id_rsa.pub.pem
boards/globalscale/espressobin/support/mkflash.sh build/zephyr/zephyr.bin
```

Snippet `espressobin-nvme-target` musí přijít **za** `espressobin-pcie-ep` (maže `&epf_test0`) a `espressobin-sata`.

- `PCIE_EPF_NVME_MDTS_KB=128` a `AHCI_BOUNCE_SECTORS=256`: jeden NVMe příkaz = jeden ATA příkaz.
- `FS_FATFS_MOUNT_MKFS=n`: disky patří hostu. **Nemountovat `/sata0:` v Zephyru, dokud je host používá** (dva zapisovatelé = rozbitý FS).

## 6. Na hostu (BeagleY-AI, Linux)

```sh
echo 1 > /sys/bus/pci/rescan          # nebo restart hosta (okna mostů, viz espressobin-pcie-ep.md)
lspci -nn -d 11ab:0100 -vv            # Class 0108, BAR0 16K 64bit, MSI-X 16
dmesg | grep nvme                     # nvme nvme0: 4/0/0 default/read/poll queues
nvme list
nvme id-ctrl /dev/nvme0 | grep -E 'sn|mn|mdts|nn|oncs|vwc'
nvme id-ns /dev/nvme0n1
nvme smart-log /dev/nvme0
dd if=/dev/nvme0n1 of=/dev/null bs=128k count=800 iflag=direct status=progress
fio --name=r --filename=/dev/nvme0n1 --rw=randread --bs=4k --iodepth=16 --ioengine=io_uring --direct=1 --runtime=20
```

Na EP: `nvme_epf status` (CC/CSTS, BME, namespacy, fronty s head/tail/doorbell, čítače, IRQ), `nvme_epf clear`.

## 7. Očekávání a limity

- **Propustnost:** strop dává XOR DMA (Gen1: host → EP 147 MB/s, EP → host ~231 MB/s, viz `espressobin-pcie-ep.md`) a SATA (AHCI bounce memcpy). Příkazy jdou sériově, takže iodepth na hostu pomůže jen tím, že SQ nebude prázdná.
- **Latence:** pro každou CQE se programuje OB okno (map + unmap), u PRP listu ještě jedno čtení.
- **2 TiB:** `disk_access` má 32bit čísla sektorů. Disk nad 2³² sektorů se ořízne a varuje v logu. Řešení: 64bit `disk_access` nebo přímý hook na `ahci_rw()` (ta už 64bit LBA má).
- **4Kn disky** fungují (LBADS 12), 512e se hlásí jako 512.
- Bez SGL, bez DSM/TRIM, bez namespace managementu, bez CMB.

## 8. Další kroky

1. HW test: enumerace, `nvme list`, `dd`/`fio`, `nvme smart-log`, restart hosta za běhu (reset přes link down).
2. Trvalé mapování CQ (a SQ) jako v Linuxu, zmenší režii OB oken. Aardvark má 8 oken, takže na víc front je nevystačí.
3. DSM deallocate → ATA DATA SET MANAGEMENT (TRIM) pro SSD.
4. 64bit LBA backend (AHCI přímo).
5. ZNS (zoned namespace, CSI 2) nad SMR/CMR diskem: plán z `espressobin-rc-sata-usb`.
6. Paralelismus: NCQ v AHCI + víc příkazů v letu (dnes jeden).
