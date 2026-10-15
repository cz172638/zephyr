# PCIe endpoint v Zephyru: oddělení abstrakce od HW, J721E/J722S na R5F

Stav k 2026-10-15. Zephyr `main` @ `124121fbb`, Linux `master` @ `0fcb05b`.

## TL;DR

- Současné `pcie_ep.h` je fakticky API ovladače iProc: neumí hlavičku, BARy, funkce, link události ani správu outbound okna. Druhý řadič by nemohl nic znovu použít.
- Navrženo a implementováno dvouvrstvé API po vzoru Linuxu: **EPC** (`pcie_epc.h`, co umí řadič) a **EPF** (`pcie_epf.h`, co dělá endpoint). Mezi nimi HW-nezávislé jádro (alokátor OB okna, mapování host paměti, události, autostart).
- HW strana je rozdělena na **knihovnu IP jádra** (Cadence, sdílená všemi SoC s tímto IP) a **SoC glue** (TI K3: straps v CTRL_MMR, TISCI power domain, user_cfg/intd_cfg).
- První funkce: `zephyr,pcie-epf-test`, registrově kompatibilní s linuxovým `pci_endpoint_test` → test ze standardního Linux hostu přes `pcitest`/kselftest.
- DT pro J721E (BeagleBone AI-64, PCIe1) a J722S (BeagleY-AI, PCIe0) na MAIN R5F; overlaye pro Linux, které PCIe, SerDes a BAR carveout přenechají R5F.
- Ověřeno: build obou desek bez warningů, checkpatch čistý, 12 unit testů na `native_sim` (jádro + programování Cadence registrů), Linux overlaye přeložené a aplikované `fdtoverlay` na 4 base DTB. **Na HW netestováno.**
- Největší díra: **SerDes** (Sierra na J721E, Torrent na J722S) Zephyr zatím neumí; driver předpokládá, že je nastavený předem.

## 1. Výchozí stav v Zephyru

`drivers/pcie/endpoint/` obsahuje jen Broadcom iProc PAXB (`pcie_ep_iproc*.c`) a `pcie_ep_common.c`. API `include/zephyr/drivers/pcie/endpoint/pcie_ep.h`:

| op | poznámka |
|---|---|
| `conf_read/conf_write` | nepřímý přístup do config space |
| `map_addr/unmap_addr` | OB mapování, ale s pevným modelem "lowmem/highmem okno" z iProcu |
| `raise_irq` | MSI/MSI-X/legacy |
| `register_reset_cb` | PERST/hot reset/FLR callback v ISR |
| `dma_xfer` | PAX DMA |

Co chybí, aby šlo napsat přenositelný endpoint:

- hlavička (vendor/device/class) – u iProcu ji nastaví firmware/straps,
- BARy a jejich inbound translace,
- pojem funkce (PF číslo) a vazba "funkce ↔ řadič" v DT,
- link události (link up/down, Bus Master Enable),
- obecná správa outbound okna (alokace, zarovnání, víc regionů),
- schopnosti řadiče (`features`: rezervované/pevné BARy, zarovnání, počet funkcí).

## 2. Návrh

```
  +-------------------+   +-------------------+
  | EPF: test         |   | EPF: vlastní prot.|   funkční drivery, DT děti řadiče
  +---------+---------+   +---------+---------+   (reg = číslo PF)
            |    pcie_epc_*() API   |
  +---------v-----------------------v---------+
  | EPC core: OB alokátor, map_host, události,|   pcie_epc_core.c (HW-nezávislé)
  |           claim funkcí, autostart, memcpy |
  +---------------------+---------------------+
                        | struct pcie_epc_driver_api
  +---------------------v---------------------+
  | IP knihovna: Cadence LGA (/ DWC / ...)    |   pcie_epc_cdns.c
  +---------------------+---------------------+
                        | glue ops: start_link, stop_link, link_up
  +---------------------v---------------------+
  | SoC glue: TI K3 (J721E/J7200/AM64/J722S)  |   pcie_epc_ti_k3.c: power, straps, IRQ
  +-------------------------------------------+
```

### 2.1 EPC API (`pcie_epc.h`)

| funkce | Linux ekvivalent | poznámka |
|---|---|---|
| `pcie_epc_start/stop` | `pci_epc_start/stop` | povolí funkce + link training |
| `pcie_epc_write_header` | `pci_epc_write_header` | vendor ID jen přes PF0 |
| `pcie_epc_set_bar/clear_bar` | `pci_epc_set_bar` | `phys` = adresa z pohledu AXI masteru řadiče |
| `pcie_epc_map_addr/unmap_addr` | `pci_epc_map_addr` | low-level, CPU adresa uvnitř OB okna |
| `pcie_epc_set_msi/get_msi`, `set_msix/get_msix` | stejné | |
| `pcie_epc_raise_irq` | `pci_epc_raise_irq` | INTx/MSI/MSI-X, číslo vektoru od 1 |
| `pcie_epc_cfg_read/cfg_write` | – | 32bit přístup do config space funkce |
| `pcie_epc_get_features` | `pci_epc_get_features` | |
| `pcie_epc_link_up` | – | |
| `pcie_epc_cpu_to_axi` | – | překlad CPU→AXI (RAT, TCM aliasy…) |

Jádro (`pcie_epc_core.c`) přidává bez znalosti HW:

- `pcie_epc_claim_function()` – každá EPF si zabere číslo PF, kolize = `-EBUSY`.
- `pcie_epc_ob_alloc/free()` – bitmapa nad OB oknem; bloky jsou **přirozeně zarovnané mocniny dvou**, což sedí na ATU Cadence i DWC.
- `pcie_epc_map_host()` – alokuje okno, zarovná PCI adresu na velikost regionu (translace nahrazuje dolní log2(size) bitů) a zvětšuje region, dokud pokrývá požadavek; vrací ukazatel na první bajt.
- `pcie_epc_xfer_memcpy()` – kopie přes OB okno, k zápisu na host přidá bariéru + non-posted read (data viditelná dřív než IRQ). Náhrada `pcie_ep_xfer_data_memcpy()`.
- `pcie_epc_notify()` – ISR-safe; události jdou přes `k_msgq` do system workqueue a odtud listenerům EPF (mohou spát a volat API).
- autostart: `SYS_INIT` po inicializaci EPF spustí link na řadičích s aspoň jednou funkcí (vypnout: `zephyr,no-autostart`).

Pořadí inicializace (POST_KERNEL): EPC 70 → EPF 75 → autostart 80.

### 2.2 EPF (`pcie_epf.h`)

EPF je normální Zephyr device, DT dítě řadiče:

```dts
&pcie0_ep {
	status = "okay";
	epf_test0: function@0 {
		compatible = "zephyr,pcie-epf-test";
		reg = <0>;               /* PF číslo */
		vendor-id = <0x104c>;
		device-id = <0xb010>;
		class-code = <0xff0000>;
		memory-region = <&pcie_bar_mem>;
	};
};
```

Makra `PCIE_EPF_DT_EPC()`, `PCIE_EPF_DT_FUNC()`, `PCIE_EPF_DT_HEADER()`, helper `pcie_epf_bar_size()` (respektuje rezervované/pevné BARy).

### 2.3 Legacy API a iProc – migrace

V tomto kroku se `pcie_ep.h` ani iProc nemění (regresní build `bcm958402m2/bcm58402/m7` s `CONFIG_PCIE_ENDPOINT=y` prochází). Navržený postup:

1. **teď**: nové řadiče jen přes `pcie_epc`.
2. **port iProc** na `pcie_epc_driver_api`: `conf_read/write` → `cfg_read/write`; `raise_irq` 1:1; `write_header/set_bar` → `-ENOTSUP` nebo zápis do PAXB IARR/IMAP; `register_reset_cb` → `pcie_epc_notify(PERST_*/HOT_RESET/FLR)`.
   iProc má dvě pevná OB okna (low/high) s jedinou translací – `struct pcie_epc_ob_window` je potřeba rozšířit na **pole oken** s příznakem `>4G`; alokátor pak odpovídá `PCIE_OB_LOWMEM/HIGHMEM`.
3. `pcie_ep.h` → tenké inline wrappery nad EPC (pro `dma_iproc_pax*`), pak deprecate.

## 3. TI K3 implementace

### 3.1 Cadence knihovna (`pcie_epc_cdns.c`)

Psáno z popisu registrů (offsety jsou v `pcie_epc_cdns_regs.h`), chování porovnané s linuxovým `pcie-cadence-ep.c`; kód odtamtud převzat není (GPL vs Apache-2.0).

- Config space funkce na `reg + (fn << 12)`; **všechny sub-word zápisy jako 32bit RMW** (J721E jiné šířky netoleruje).
- BAR: `LM_EP_FUNC_BAR_CFG0/1`, 8 bitů na BAR (aperture = log2(size)−7, ctrl 4/5/6/7), inbound `AT_IB_EP_FUNC_BAR_ADDR0/1`.
- OB region: `AT_OB_REGION_*`, nbits = log2(velikost), v DESC0 jen číslo funkce (bus/dev zachytí IP).
- Region 0 je vyhrazený pro IRQ: INTx jako normal message (Assert/Deassert_INTA, routing local), MSI/MSI-X jako zápis do 256B okna. **IRQ region je na konci OB okna**, aby začátek (zarovnaný na 128 MiB) zůstal pro přirozeně zarovnané alokace.
- `cdns_pcie_ep_poll()`: změny link stavu a BME každé funkce → události.

### 3.2 TI glue (`pcie_epc_ti_k3.c`)

- `ti,j721e-pcie-ep` + specifický compatible (`ti,j722s-pcie-ep`, `ti,am64-pcie-ep`, `ti,j7200-pcie-ep`, `ti,j784s4-pcie-ep`) vybírá bit link-down a šířku pole lanes.
- **Straps** v `PCIEn_CTRL` (bit7 = RC → 0 pro EP, gen v [1:0], lanes v [8] / [9:8]) se zapíšou *před* požadavkem na TISCI power domain – řadič je vzorkuje při zapnutí.
- Power domain `ti,sci-pm-domain` musí mít `zephyr,pm-device-runtime-auto`: bez toho ji `pm_device_driver_init()` zapíná v PRE_KERNEL_1 (prio 10), kdy TISCI/secproxy (prio 40) ještě neběží.
- Link training: `user_cfg+0x04` bit0; stav `user_cfg+0x14` [1:0] == 3.
- Link-down IRQ z `intd_cfg` volitelně, **default polling** (50 ms), protože routing PCIe wrapper událostí do R5F VIM jsem v TISCI dokumentaci nedohledal.

### 3.3 DT (R5F pohled, 32bit adresy)

| | J721E PCIe0 | J721E PCIe1 | J722S PCIe0 |
|---|---|---|---|
| intd_cfg | 0x02900000 | 0x02910000 | 0x0f102000 |
| user_cfg | 0x02907000 | 0x02917000 | 0x0f100000 |
| reg | 0x0d000000 /8M | 0x0d800000 /8M | 0x0d000000 /8M |
| mem (OB) | 0x10000000 /128M | 0x18000000 /128M | 0x68000000 /128M (DAT0) |
| PCIEn_CTRL | main 0x104070 | main 0x104074 | wkup 0x43004070 |
| TISCI dev | 239 | 240 | 259 |

J721E PCIe2/3 mají okna jen nad 4 GiB → bez RAT mapování nedostupné z R5F, proto nepopsané.

TISCI klient z MAIN R5F (z TI TISCI dokumentace):

| SoC | host | secure proxy | rx thread / IRQ | tx thread |
|---|---|---|---|---|
| J721E | MAIN_0_R5_0 = 35 | NAVSS0_SEC_PROXY_0 @0x32c00000 | 61 / INTR_225 | 63 |
| J722S | MAIN_0_R5_1 = 38 (non-secure) | DMASS0_SEC_PROXY_0 @0x4d000000 | 6 / INTR_65 | 7 |

J721E potřebuje thready > 31 → nový `CONFIG_MBOX_TI_SECURE_PROXY_MAX_CHANNELS`.

### 3.4 R5F specifika

- J7 R5F desky běží s `CONFIG_ARM_MPU=n`: datové přístupy jdou podle default mapy R5 jako non-cacheable, takže BAR paměť a OB okno nepotřebují cache maintenance. S MPU je nutné přidat IO region pro `reg`/wrapper a non-cacheable region pro OB okno (atribut je už na BAR carveoutu).
- BAR paměť musí být v DDR (carveout), **ne v TCM** – TCM je jen lokální pro R5.
- OB okno jako Normal non-cacheable, ne Device: `memcpy` s libovolným zarovnáním na Device paměti R5 faultuje; pořadí zajišťuje bariéra + read-back v jádře.

## 4. Linux strana

Commit `5fa2865` (Linux, `arch/arm64/boot/dts/ti/`):

| overlay | base | co dělá |
|---|---|---|
| `k3-am67a-beagley-ai-pcie0-ep-r5f.dtso` | BeagleY-AI | `pcie0_rc`, `serdes_wiz1` disabled; carveout 0xa7000000/16M no-map; lane mux SerDes1→PCIe0 |
| `k3-j722s-evm-pcie0-ep-r5f.dtso` | J722S EVM | totéž, SerDes0 nechá na USB |
| `k3-j721e-beagleboneai64-pcie1-ep-r5f.dtso` | BBAI-64 | `pcie1_rc`, `serdes_wiz1` disabled; carveout 0xac000000/16M |
| `k3-j721e-evm-pcie0-ep-r5f.dtso` | J721E EVM | `pcie0_rc`, `serdes_wiz0` disabled; carveout 0xac000000/16M |

Proč to stačí: Linux sahá na TISCI device/clock jen přes DT uzly, které používá; s vypnutým `pcieN_rc` a `serdes_wizN` se k PCIe ani SerDes nedostane. `serdes_ln_ctrl` (reg-mux) při probe zapíše `idle-states`, proto je PCIe lane explicitně zafixovaná, aby ji Linux nepřepsal.

Použití (U-Boot):

```
fdt apply ${loadaddr_dtbo}   # nebo name_overlays=k3-am67a-beagley-ai-pcie0-ep-r5f.dtbo
```

R5F firmware pak buď U-Boot early boot (`rproc`), nebo Linux remoteproc (`/lib/firmware/j722s-main-r5f0_0-fw`). Pozor: `echo stop > .../remoteproc*/state` R5F resetuje, link spadne; TISCI device zůstává držený hostem 38/35, nový start ho znovu dostane exclusive.

## 5. Test

Na root complexu (jiný stroj, nebo druhá K3 deska v RC módu):

```sh
modprobe pci_endpoint_test
lspci -nn | grep 104c          # 104c:b010 (J722S) / 104c:b00d (J721E)
pcitest -b 0                   # BAR0
pcitest -i 1; pcitest -m 1     # MSI
pcitest -w -s 65536            # EP zapíše do host paměti
pcitest -r -s 65536            # EP čte z host paměti
pcitest -c -s 65536            # EP kopíruje host->host
```

Na R5F shell: `pcie_ep status`, `pcie_ep msi 1`.

Ověřeno bez HW:

```
west build -b beagley_ai/j722s/main_r5f0_0   samples/drivers/pcie/endpoint/epf_test
west build -b beaglebone_ai64/j721e/main_r5f0_0 samples/drivers/pcie/endpoint/epf_test
west build -b native_sim/native/64 tests/drivers/pcie/endpoint/epc_core   # 6/6 PASS
west build -b native_sim/native/64 tests/drivers/pcie/endpoint/epc_cdns   # 6/6 PASS
```

`epc_cdns` kontroluje přesné hodnoty registrů (BAR0 512 B → `0x82`, BAR4 1 MiB pref64 → `0xed`, OB nbits/adresy, MSI data `(data & ~mask) | (vec-1)`, ARI next-function).

## 6. Otevřené body

1. **SerDes** – nejdřív. J722S/AM64 používají Torrent 10G; TI `mcupsdk-core` má BSD-3 `csl_serdes3` + tabulky `csl_wiz16m_ct2_refclk100MHz_32b_PCIe` a sekvenci v `source/drivers/pcie/v0/soc/am64x_am243x/pcie_serdes.c` – port do `hal_ti` + malý PHY driver. J721E PCIe běží přes **Sierra** 16G (PDK, ne mcupsdk-core). Do té doby: `CONFIG_PCIE_EPC_TI_K3_SERDES_PRECONFIGURED=y` = SerDes musí nastavit někdo před Zephyrem.
2. **HW konektory**: M.2 E-key na BBAI-64 i FFC na BeagleY-AI jsou host sloty (deska budí REFCLK a PERST#). EP vyžaduje adaptér/kabel, který vezme REFCLK a PERST# z externího RC a překříží TX/RX (nebo SRIS). TI referenční EP setup je J721E EVM s kabelem.
3. **Ověřit host ID** na HW: J721E MAIN_0_R5_0 (35) vs. kontext podle security stavu R5; při NAK zkusit 36 (thready 66/68, IRQ 227).
4. **Interrupt routing** PCIe wrapper → R5F VIM (link-down, případně PERST), pak IRQ místo pollingu.
5. **PERST#** jako vstup (GPIO) → `PCIE_EPC_EVT_PERST_*` a korektní re-init po resetu hostu.
6. **DMA**: EPF zatím kopíruje CPU přes OB okno; pro výkon BCDMA/PKTDMA nebo DMA v PCIe IP přes volitelné `dma_xfer` v EPC.
7. **Více funkcí / SR-IOV** (`CONFIG_PCIE_EPC_CDNS_MAX_FUNCS`, VF zatím ne).
8. **iProc port** dle 2.3 + rozšíření OB okna na pole.

## 7. Artefakty

- `zephyr-pcie-epc-k3.bundle` – Zephyr větev `pcie-epc-k3` (6 commitů nad `124121fbb`) + `gh-pages` s tímto dokumentem.
- `linux-r5f-pcie-ep.bundle` – Linux větev `r5f-pcie-ep` (1 commit nad `0fcb05b`).

```sh
git -C zephyr fetch /path/zephyr-pcie-epc-k3.bundle pcie-epc-k3:pcie-epc-k3 gh-pages:gh-pages
git -C linux  fetch /path/linux-r5f-pcie-ep.bundle r5f-pcie-ep:r5f-pcie-ep
```

## Zdroje

- TI TISCI J722S hosts / secure proxy: <https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/j722s/hosts.html>, <https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/j722s/sec_proxy.html>
- TI TISCI J721E hosts / secure proxy: <https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/j721e/hosts.html>, <https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/j721e/sec_proxy.html>
- Linux: `drivers/pci/controller/cadence/{pcie-cadence-ep.c,pci-j721e.c,pcie-cadence-lga-regs.h}`, `drivers/misc/pci_endpoint_test.c`, `arch/arm64/boot/dts/ti/k3-j72{1e,2s}*.dts*`
- TI MCU+ SDK: <https://github.com/TexasInstruments/mcupsdk-core> (`source/drivers/pcie`, `source/drivers/hw_include/serdes_cd`)
