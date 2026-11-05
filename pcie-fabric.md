# PCIe fabric: nodes behind one root complex

The aim is a low-latency fabric. A root complex (RC), a switch and several endpoints (EP) form the nodes; PCIe is the transport. The RC hands out "ID cards" (node ID, role, peer table). From then on the nodes write to each other directly, peer to peer through the switch.

Code (branch `espressobin`):

- `include/zephyr/drivers/pcie/fabric.h`: wire format and API.
- `drivers/pcie/fabric/`:
  - `pfab_core.c`: node, messages, RMA.
  - `pfab_mgr.c`: manager plus RC node 0.
  - `pfab_shell.c`: shell commands.
- `drivers/pcie/endpoint/functions/pcie_epf_fabric.c`: EP function `zephyr,pcie-epf-fabric`.
- `snippets/espressobin-fabric`.
- Tests in `tests/drivers/pcie/fabric/{core,epf,mgr}` (native_sim, 15 tests).

## Principle

- **Only writes cross the fabric.** A posted write needs no completion, and posted writes on one path stay in order. Everything a node reads is its own memory, so it never needs a PCIe read.
- **Each node owns a region the others write into:** an EP owns its BAR0, the RC a piece of RAM (the EPs reach it by inbound DMA).
- **A message** is data followed by the `seq` word, written last. The receiver takes the slot when `seq` equals the number it has taken so far plus 1, then returns a credit (a write into the sender's region).
- **RMA:** a one-sided write into the peer's window. Typically followed by a message on the same path; ordering guarantees the data arrive first.
- **The RC is the manager:** it finds the nodes, assigns IDs and roles, sets the switch for P2P and writes the cards. In the data path it is just one node among the others (node 0).

## Region (all fields LE, 32-bit accesses only)

| Offset | Who writes | Contents |
|---|---|---|
| `0x0000` info | owner | magic `PFAB`, version 1.0, region size, state (init/ready/running/error), DSN, caps (MSG, RMA, DMA…), roles offered (bit n = role n), geometry (max_nodes, nslots, slot_size, offsets), accepted epoch, node ID \| role, fw (git hash), board, error of a refused card |
| `0x0100` card | RC | magic `CARD`, epoch, **commit** (= epoch, written last; 0 = withdrawn), node ID \| role, npeers |
| `0x0200` peer table | RC | up to 16 × 64 B: ID \| role, valid, DSN, **bus address of the region**, size, geometry |
| `0x0800` credits | peers | u32 per peer: how many of our messages it has taken |
| `0x1000` rx rings | peers | ring per sender (index = sender's ID) × nslots × slot_size |
| `rma_off` | peers | RMA window up to the end of the region |

Slot: `+0` seq (last), `+4` len \| type<<16, `+8` src \| (epoch & 0xffff)<<16, `+16` payload. A late write from an older epoch is ignored (counted as stale).

Defaults (Kconfig): 8 nodes, 16 slots × 256 B (240 B payload), region 64 KiB, so rings take 32 KiB and the RMA window ~28 KiB.

Types below `0x100` are reserved for the fabric: `1` echo request, `2` echo reply. Roles: 0 none, 1 manager, 2 compute, 3 storage, 4 io, the rest free.

## Bring-up (manager)

1. **Scan:** walk the tree through the bridges (like AER). For a type 0 function: turn on Memory Space, read BAR0, map 4 KiB and check the magic, then map the whole region.
2. **Withdraw** the old cards (commit = 0). The nodes drop their peers and switch to READY.
3. **Node IDs by DSN** (1..n, so the same boards get the same IDs). Role from the table (`pcie fabric role`), otherwise the lowest role the node offers.
4. **Switch ports:** ACS P2P Request Redirect, Completion Redirect and Upstream Forwarding off. Peer traffic then turns at the switch and does not go through the RC. ASM1806 has ACS; ASM1184 has none and routes P2P directly anyway.
5. **Memory and Bus Master Enable** on every node. The A3720 EP refuses outbound traffic without BME.
6. **Cards:** every node gets all the others (including node 0 = the RC) with bus addresses and geometry. A fence, then commit = epoch.
7. **Wait** until every node reports RUNNING with this epoch.

The EP function maps each peer's region through its outbound window when it takes the card (`pcie_epc_map_host`). The RC node writes into the EP BARs through its CPU windows. Bus address of the RC node = physical + `CONFIG_PCIE_FABRIC_RC_INBOUND_OFFSET` (0 on A3720: TF-A maps all DRAM 1:1).

Automatically (`CONFIG_PCIE_FABRIC_MGR_AUTO`): after every enumeration on the RC, in its own work queue, with retries while the EPs are still booting.

## Shell (`pcie fabric`)

| Command | Where | What |
|---|---|---|
| `node` | anywhere | local node: ID, role, epoch, peers (bus, sent/credit/taken), counters |
| `ping <node> [count] [len]` | anywhere | round trip via echo, min/avg/max µs |
| `send <node> <text>` / `recv [ms]` | anywhere | messages |
| `put <node> <off> <word>…` / `rma [off] [len]` | anywhere | RMA writes, our own window |
| `poll [us]` | anywhere | service thread interval; `0` = busy at the lowest priority |
| `ls`, `scan`, `up`, `down` | RC | manager |
| `role <dsn> <role>` | RC | role for the next `up` |

Example (RC ESPRESSObin, EP ESPRESSObin with `pcie controller ep function fabric`):

```
# EP (once, then reboot)
pcie controller ep function fabric
# RC
pcie fabric ls
pcie fabric ping 1 1000 16
pcie fabric send 1 hello
# EP
pcie fabric recv
pcie fabric poll 0          # busy polling: lower latency of the echo answers
```

## Notes

- **Latency:** with `poll 50` (default) the echo waits up to 50 µs on each side, so measure latency with `pcie fabric poll 0` on both sides. A CPU write through the A3720 outbound window is one TLP per 32-bit word: fine for small messages, use DMA (XOR) for large ones (not yet).
- **Coherence:** the EP region is in the uncached carveout and the RC node's region is mapped uncached (the cached alias is flushed first).
- **Hot reset, link down, BME off** on an EP mean it drops its peers. The manager then has to call `up` again (a new epoch).
- **Security:** none. Every node may write into any other region (no IOMMU on A3720, and ACS redirect is deliberately off).

## Linux endpoint (out-of-tree module)

`pci-epf-fabric` is a separate repository (bundle `pcie-fabric-linux.bundle`, DKMS, Linux 6.13+). It uses the same wire format and does what the Zephyr EP function does:

- a kernel thread takes the card and maps the peers with `pci_epc_mem_map()`, then handles the echo service and the receive queue;
- configfs attributes: `region_size`, `roles`, `dsn` (random by default; set it so the node ID stays the same), `poll_us`, `need_bme`;
- debugfs files: `node`, `send`, `recv`, `ping`, `put`, `rma`.

It is meant for the BeagleY-AI (J722S) as an EP. It builds against 7.3-rc7 (arm64, W=1 clean) but has not been tested on hardware yet.

## Resizable BAR

The A3720 EP has no Resizable BAR capability. Its extended chain is AER → DSN → L1SS → LTR → SecPCIe. Emulating one is impossible: host writes to config space never reach the EP's CPU, and there are no free writable words to hold a capability.

- **What it has instead:** the BAR size is set locally in LMI `0x6240` before link-up (`region-size`). That is the same model as `BAR_RESIZABLE` in the Linux EP framework, where the size is chosen once by the EP.
- **Possible later (non-standard):**
  - resize through a BAR0 mailbox followed by `remove` + `rescan`;
  - a separate large RMA BAR (BAR4, 64-bit, up to 2 GiB).
- **As a root complex** the A3720 has a PCIe CPU window of only 128 MiB, so large BARs do not fit anyway.

## Next steps

1. Hardware: RC ESPRESSObin plus EP ESPRESSObin: `ping` RC↔EP (node 0 ↔ 1).
2. A second Zephyr EP (LS1012A with a DWC EP driver) for EP↔EP P2P through the ASM1806.
3. A Linux side:
   - EP function in the kernel (BeagleY-AI, `pci-epf-fabric`),
   - on a Linux RC, the manager in userspace through sysfs `resource0`,
   - then a libfabric provider `pcief` (`FI_EP_DGRAM` + `rxd` → RDM, AV = cards, `FI_RMA` write only) and `fi_pingpong` from fabtests.
4. DMA for large messages and RMA, notification through MSI or a doorbell (if one exists) instead of polling.
