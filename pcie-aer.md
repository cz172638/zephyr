# PCIe AER below the root port (Zephyr, Aardvark RC)

Advanced Error Reporting for the whole tree under a root port, switches
included. The helpers in `drivers/pcie/pcie_aer.c` are controller neutral:
they go through `pcie_ctrl_conf_read/write`, so another RC backend
(RK3399) can use them as they are.

## What has to be set where

An error message (ERR_COR, ERR_NONFATAL, ERR_FATAL) reaches the root port
only when every hop allows it:

| Where | Register | Bits | Effect |
|---|---|---|---|
| every function | Device Control (PCIe cap + 0x08) | 0 COR, 1 NFE, 2 FE, 3 UR | the function sends messages for its own errors |
| every function | Command (0x04) | 8 SERR# | non-fatal and fatal also without the DevCtl bits |
| every bridge (switch ports, root port) | Bridge Control (0x3E) | 1 SERR# Enable | **forwards** ERR_NONFATAL/FATAL from below |
| functions with AER | AER + 0x04/0x10 | | UE/CE status, RW1C |
| | AER + 0x08/0x14/0x0C | | masks, severity (hardware defaults kept) |
| | AER + 0x18 | 6 gen, 8 check (if 5, 7) | ECRC, optional |
| root port | AER + 0x2C Root Error Command | 0-2 | interrupt for received COR/NFE/FE |
| | AER + 0x30 / 0x34 | | Root Error Status, Error Source ID (first source) |
| Aardvark | ISR0 (0x4840) | 11-13 | controller interrupt for the messages |

UR is enabled on endpoints only by default. On ports, reads of absent
functions during a rescan end with UR, so every rescan would report.

A function without AER (cheap switches sometimes leave it out) still
reports through Device Status (PCIe cap + 0x0A: C/N/F/U), and the
service sweep picks those up.

## Tree walk

Through the bridges' secondary/subordinate bus numbers. Below a root port
or a downstream port (a link) only device 0 is probed. The internal bus of
a switch (below its upstream port) is probed fully, and functions 1-7 only
when the header says multifunction. So the walk itself causes no UR on the
links.

## Servicing

ISR0 bits 11-13 → `k_work` on the system work queue. Config requests go
through the PIO block with polling and a mutex, so they cannot run in the
ISR.

1. Root Error Status. For COR, the source is in Source ID [15:0]; for
   UNC, it is in [31:16] (fatal per bit 6).
2. The source's AER UE/CE (without the masked ones) and the header log
   for UNC are read and cleared (RW1C), along with Device Status.
3. If the root port latched nothing, or "multiple" is set, the walk looks
   for every function with status set.
4. Counters are kept per source. The log shows `CONFIG_PCIE_AER_LOG_LIMIT`
   messages per source; after that the errors are only counted.

## Shell

```
pcie aer                      # tree: type, AER?, reporting, forwarding, DevSta, UE/CE, message counts
pcie aer 02:01.0              # one function decoded (+ root cmd/status/source ID on the root port)
pcie aer setup [ur] [urbridge] [ecrc]   # set up again with other options
pcie aer clear                # status of the whole tree and counters
pcie aer poll                 # service by hand (no interrupt needed)
pcie controller irq           # ISR0 error counters + serviced events
```

Example line:

```
02:01.0 Downstream AER rep CNF  fwd sta -    UE - CE RxErr | msgs c3 n0 f0
```

`rep` = DevCtl reporting C/N/F/U, `fwd`/`NOFWD` = Bridge Control SERR#,
`sta` = Device Status.

## Kconfig

- `CONFIG_PCIE_AARDVARK_AER` (y): setup after every enumeration, interrupt servicing
- `CONFIG_PCIE_AER_UR_EP` (y), `CONFIG_PCIE_AER_UR_BRIDGE` (n), `CONFIG_PCIE_AER_ECRC` (n)
- `CONFIG_PCIE_AER_SOURCES` (16), `CONFIG_PCIE_AER_LOG_LIMIT` (16)

## Test

`tests/drivers/pcie/aer` on `native_sim/native/64` uses an emulated tree:
root port → switch (upstream, 2× downstream, one without AER) → endpoints,
one of them multifunction. The test checks:

- walk order and the number of absent probes,
- setup bits and status clearing,
- servicing from the Source ID,
- the sweep without a latched source.

## Ověřeno na HW (2026-11-04, image 145)

RC ESPRESSObin → ASM1184 (všechny porty s AER) → 02:07.0 → EP ESPRESSObin (06:00.0, NVMe funkce).

- `devmem 0xe8080000 32 …` (v okně 02:07.0 `0xe8000000–0xe80fffff`, mimo BAR0 EP): EP odmítne posted zápis jako UR, pošle ERR_NONFATAL, porty ho přepošlou, ISR0 bit 12 → obsluha.
- **Root port Aardvarku Source ID v RC zapisuje** (`unc 06:00.0`), obsluha šla přímou cestou. Header log `40008001 0000180f e8080000` = MWr na tu adresu.
- **ASM1184 neloguje UR na konfigurační požadavek** na zařízení ≠ 0 za downstream portem (`pcie cfg 06:05.0`): vrátí `ffffffff`, ale nenastaví DevSta UR ani AER. Advisory chyby se na něm netestují. Na test je lepší posted zápis.
- Zdvojené řádky logu na UART konzoli: dva backendy (`LOG_BACKEND_UART` + shell), ne dvě události.

## Open

- Bez zotavení (retrain linky, secondary bus reset při fatální chybě), jen hlášení.
- ASM1806 (ACS) za stejným testem.
