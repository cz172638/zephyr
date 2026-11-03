# A3720 / ESPRESSObin: SATA, USB 2.0, USB 3.0, OTG a „SATA device“

Zdroje:

- **HW specifikace:** Marvell 88F3710/88F3720 Hardware Specifications, MV-S110707-U0 Rev. B (příloha `6647a477`).
- **Adresy, IRQ a hodiny:** Linux `armada-37xx.dtsi`.
- **COMPHY a USB32 control:** TF-A `phy-comphy-3700.c` a U-Boot `comphy_a3700.c`.
- **Co je otevřené:** Functional Specifications (MV-S110895), na které HW specifikace odkazuje, k dispozici nejsou. Registry device-módu je proto potřeba namapovat (kap. 5).

## 1. Jednotky

| jednotka | adresa | IRQ (GIC SPI) | hodiny (Linux index) | PHY |
|---|---|---|---|---|
| SATA AHCI host | `0xd00e0000`, `0x178` | 27 | nb 1 `sata_host` | COMPHY lane 2 |
| ↳ nepřímý přístup k PHY lane 2 | `0xd00e0178` addr / `0xd00e017c` data | | | SATA blok +`0x200`, USB3 na lane 2 má jiný offset |
| USB 3.0 řadič (xHCI host / device) | `0xd0058000`, `0x4000` | 3 | sb 12 `usb32_ss_sys`, sb 11 `usb32_usb2_sys` | USB3: COMPHY lane 0 (nebo 2); USB2: **USB0** = UTMI OTG PHY |
| ↳ USB3 PHY lane 0 | `0xd005c000`, `0x400` | | | |
| ↳ UTMI OTG PHY (USB0) | `0xd005d000`, `0x800` | | | VBUS, ID, detekce nabíječky |
| ↳ USB32 control / misc | `0xd005d800`, `0x800` | | | **volba host/device** (kap. 3) |
| USB 2.0 řadič (EHCI, jen host) | `0xd005e000`, `0x1000` | 17 | – | **USB1** = UTMI host PHY |
| ↳ UTMI host PHY (USB1) | `0xd005f000`, `0x800` | | | |
| ↳ USB2 host misc | `0xd005f800`, `0x800` | | | |
| COMPHY společné (selektor lane) | `0xd0018300`, `0x300` | | | |

**SERDES** (HW specifikace, tab. 33):

| | lane 0 | lane 1 | lane 2 |
|---|---|---|---|
| PCIe | | RC/EP | |
| SATA | | | SATA |
| SGMII | SGMII1 | SGMII0 | |
| USB3 | host / device | | host / device |

USB3 může běžet jen na jedné z lane 0 a 2. Podle HW specifikace (kap. 5.3) musí USB3 a USB2 vést do stejného konektoru a platná je jen kombinace USB3 + USB2 **Port0**.

**ESPRESSObin** (komentáře v Linux dts):

- **J7 = USB3 port:** USB3 řadič, lane 0 a USB0 (OTG PHY).
- **J8 = USB2 port:** EHCI a USB1 (host PHY).
- **SATA konektor:** lane 2.

## 2. Co hardware umí (HW specifikace)

**USB 3.0 řadič je host *nebo* device:**

- **Host:** xHCI 1.0, 4 sloty, 16 endpointů.
- **Device:** EP0 a 2 IN + 2 OUT endpointy, SS/HS/FS/LS, dimenzované pro **UASP** (USB Attached SCSI).
- **USB2 PHY tohoto řadiče** (USB0) má VBUS sense, ID pin a detekci nabíječky.

**USB 2.0 řadič (EHCI) je jen host.** Device mód u něj specifikace neuvádí.

**SATA je jen AHCI host** („SATA 3.0 AHCI Host Controller“): 1 port, Gen1–3, NCQ 32, partial/slumber. Target ani device mód v AHCI neexistuje a specifikace žádný nezmiňuje.

**„Skrytý“ režim, který existuje, je most USB → SATA.** Kapitola Applications popisuje mobilní DAS: „when directly attached to a USB host, the device can operate on the USB source power and function as an external mass-storage device“. V seznamu správy napájení je navíc „Support for A53 CPU power-off during USB-SATA Bridge mode“.

- Deska se tedy hostiteli ukáže jako USB disk: USB3 řadič v device módu přes UASP, SATA řadič jako host k disku.
- Formulace o vypnutí A53 během bridge módu naznačuje, že přenosy obsluhuje něco jiného než A53, nejspíš Cortex-M3 (WTMI) nebo hardware. Popis je ale jen ve Functional Spec.
- Prakticky: Zephyr na A53 (nebo později na CM3) s USB device stackem (MSC, případně UASP) a backendem `sata0`.

**Na mini-PCIe jde jen PCIe.** SoC má jen dva USB2 PHY (USB0, USB1) a oba vedou do J7 a J8. USB piny mini-PCIe slotu (36 D−, 38 D+) proto ze SoC napájet nejde. Buď nejsou zapojené, nebo jsou paralelně s některým portem. Ověř prozvoněním piny 36/38 proti D−/D+ na J7 a J8.

**OTG/device tedy jde jen na J7** (USB3 port, USB0 PHY). Na J8 ani na mini-PCIe ne.

## 3. Přepínání host/device: USB32 control `0xd005d800`

Z U-Boot `comphy_a3700.c`:

| bit | název | význam |
|---|---|---|
| 0 | `ID_MODE` | 1 = Soft ID (rozhoduje bit 1), 0 = Hard ID (rozhoduje pin USB_ID) |
| 1 | `SOFT_ID` | U-Boot ho pro host **nuluje**; 1 bude device |
| 4 | `INT_MODE` | přerušení podle ID (jen host nebo jen device), ne host a device ORované dohromady |

U-Boot pro host zapíše `ID_MODE=1, SOFT_ID=0, INT_MODE=1` a poznamenává: „Device mode works with Hard ID detection“. Pin USB_ID má vnitřní pull-up na 1.8 V. Plovoucí ID tedy znamená B-device a hardware sám od sebe skončí v device módu. Linux i U-Boot proto host vynucují softwarově.

**COMPHY lane 0 pro device:** TF-A `MV_SIP_COMPHY_POWER_ON` zná režim `COMPHY_USB3D_MODE` (5) jen v selektoru lane. Inicializaci PHY dělá pouze pro `USB3H/USB3` (4/10). Device mód tedy znamená PHY jako pro host a pak `SOFT_ID=1`.

## 4. Elektrika device módu na J7

- **J7 je Type-A a deska na něj dává VBUS.** Kabel A–A by spojil dva 5V zdroje proti sobě. Pro device mód je nutný kabel s **přerušeným VBUS** (červený vodič), případně vypnutý zdroj VBUS na desce (`USB2_DRVVBUS` na MPP, pokud ho ESPRESSObin používá; zjistit z dts nebo měřením).
- **`USB_VBUS` sense pin** (vnitřní pull-down, 0–5.5 V): pokud je na desce spojený s VBUS konektoru, device řadič uvidí session trvale. To je pro test v pořádku.
- **USB3 SuperSpeed páry** se v A–A kabelu kříží správně (TX↔RX), USB2 D+/D− jsou přímo.

## 5. Mapování registrů (image 118, shell `regs`)

`regs scan|nz|dump <pa> <len>` čte přes probe-read, takže abort se ukáže jako `????????` a nespadne. `regs clk nb|sb <i> on|off` přepíná hodinová hradla. Jednotka s vypnutými hodinami nemusí odpovídat vůbec.

```
regs clk sb 11 on
regs clk sb 12 on
regs scan 0xd0058000 0x8000      # 0x58000-0x5ffff: xHCI, PHY, misc, EHCI
regs nz   0xd0058000 0x40        # xHCI capability: CAPLENGTH/HCIVERSION, HCSPARAMS1-3, HCCPARAMS
regs nz   0xd005d000 0x1000      # OTG PHY + USB32 control
regs nz   0xd005f000 0x1000      # host PHY + USB2 misc
regs scan 0xd00e0000 0x2000      # AHCI a co následuje za 0x178
regs nz   0xd0018300 0x300       # COMPHY: selektor lane
ata debug regs
ata debug phy dump               # SATA PHY lane 2
```

Pro hledání device řadiče to samé dvakrát:

1. Jak je: host nebo hard-ID stav po bootu.
2. `devmem 0xd005d800 32` (přečíst), pak zapsat přečtenou hodnotu s nastavenými bity 0 a 1 (`devmem 0xd005d800 32 <v|0x3>`, tedy `ID_MODE=1, SOFT_ID=1`) a znovu `regs scan 0xd0058000 0x8000` a `regs nz 0xd0058000 0x100`.

Rozdíl ukáže, jestli se device registry objeví v okně xHCI (jiné capability nebo jiný blok), nebo jinde v `0x58000–0x5ffff`. Přepnutí SOFT_ID za běhu je bezpečné, protože host řadič zatím v Zephyru nic nedělá.

## 6. Co z toho plyne pro Zephyr

| cíl | potřeba | stav |
|---|---|---|
| SATA disk | AHCI ovladač + COMPHY lane 2 přes SiP | **image 117/118**: `ata`, `/sata0:` |
| USB3 host (J7) | Zephyr nemá xHCI host ovladač (UHC: DWC2, MAX3421E, MCUX EHCI/OHCI, Renesas) | větší práce |
| USB2 host (J8) | EHCI jen jako NXP MCUX obálka (`uhc_mcux_ehci`), obecný EHCI ovladač chybí | větší práce |
| **USB device (J7)** | Zephyr UDC ovladač pro tento řadič + `SOFT_ID=1` | blokuje neznámá registrová mapa device řadiče (kap. 5) |
| USB → SATA most | USB device (MSC, později UASP) + `sata0` | po USB device |

Device mód je pro Zephyr zajímavější než host: device stack (UDC, MSC, CDC-ACM, NCM) je v Zephyru vyspělý. Chybí jen ovladač řadiče a k němu mapa registrů. Pokud se ukáže, že device část je standardní IP (například jiný známý USB3 device řadič), dá se navázat na existující ovladač.
