# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
#
# OpenOCD helpers for RK3399: decoded dumps of the PCIe controller and of the
# interconnect error loggers. All reads go through the AHB-AP, so the cores
# keep running (no halt needed).
#
#   openocd -f interface/cmsis-dap.cfg -c "transport select swd" \
#           -f target/rk3399.cfg -f scripts/rockchip/rk3399-debug.tcl
#   > rk_pcie            ;# client, link, function 0 header, MSI, BARs, OB/IB
#   > rk_pcie_ob 32      ;# all outbound regions, also the unused ones
#   > rk_noc             ;# NoC error loggers (rk_noc clear: also clear them)
#   > rk_wdt off         ;# stop the watchdog before looking at a hung board
#
# Set rk_ap to the memory AP target if it is not called rk3399.ahb.

if {![info exists rk_ap]} {
	set rk_ap rk3399.ahb
}

set rk_pcie_apb  0xfd000000
set rk_pcie_core 0xfd800000
set rk_pcie_lm   0xfd900000
set rk_pcie_at   0xfdc00000
# RK3399: hardware region 0 is the 32 MiB config window at 0xf8000000, the
# 1 MiB regions of 0xfa000000 are hardware regions 1..32 (TRM table 17-4)
set rk_pcie_ob_base 0xfa000000

proc rk_rd {addr {n 1}} {
	global rk_ap
	return [$rk_ap read_memory $addr 32 $n]
}

proc rk_hex {v} {
	return [format "0x%08x" $v]
}

proc rk_bits {v hi lo} {
	return [expr {($v >> $lo) & ((1 << ($hi - $lo + 1)) - 1)}]
}

set rk_ltssm [dict create \
	0x00 detect-quiet 0x01 detect-active 0x02 polling-active 0x03 polling-compliance \
	0x04 polling-config 0x05 config-linkwidth-start 0x06 config-linkwidth-accept \
	0x07 config-lanenum-accept 0x08 config-lanenum-wait 0x09 config-complete \
	0x0a config-idle 0x0b recovery-rcvr-lock 0x0c recovery-speed 0x0d recovery-rcvr-cfg \
	0x0e recovery-idle 0x10 L0 0x11 rx-L0s-entry 0x12 rx-L0s-idle 0x13 rx-L0s-fts \
	0x14 tx-L0s-entry 0x15 tx-L0s-idle 0x16 tx-L0s-fts 0x17 L1-entry 0x18 L1-idle \
	0x19 L2-idle 0x1a L2-transmit-wake 0x20 disabled 0x27 hot-reset]

proc rk_pcie_client {} {
	global rk_pcie_apb rk_ltssm rk_pcie_lm
	lassign [rk_rd $rk_pcie_apb] cfg
	lassign [rk_rd [expr {$rk_pcie_apb + 0x3c}]] dbg
	lassign [rk_rd [expr {$rk_pcie_apb + 0x44}] 4] st0 st1 imask istat
	lassign [rk_rd $rk_pcie_lm] pl0
	set l [rk_bits $dbg 5 0]
	set k [format "0x%02x" $l]
	set lname [expr {[dict exists $rk_ltssm $k] ? [dict get $rk_ltssm $k] : "?"}]
	echo [format "client  config %s: conf_en %d link_train %d sriov %d ari %d %s gen2_sel %d" \
		[rk_hex $cfg] [rk_bits $cfg 0 0] [rk_bits $cfg 1 1] [rk_bits $cfg 2 2] \
		[rk_bits $cfg 3 3] [expr {[rk_bits $cfg 6 6] ? "RC" : "EP"}] [rk_bits $cfg 7 7]]
	echo [format "        ltssm 0x%02x %s, status0 %s (Gen%d x%d), status1 %s (link %d)" \
		$l $lname [rk_hex $st0] [expr {[rk_bits $st0 5 5] + 1}] \
		[expr {1 << [rk_bits $st0 7 6]}] [rk_hex $st1] [rk_bits $st1 21 20]]
	echo [format "        int mask %s status %s" [rk_hex $imask] [rk_hex $istat]]
	echo [format "LM      pl_config0 %s (link %s)" [rk_hex $pl0] \
		[expr {[rk_bits $pl0 0 0] ? "up" : "down"}]]
}

proc rk_pcie_hdr {{fn 0}} {
	global rk_pcie_core
	set b [expr {$rk_pcie_core + ($fn << 12)}]
	set h [rk_rd $b 16]
	lassign $h id cmdst classrev hdr
	set cmd [expr {$cmdst & 0xffff}]
	set st [expr {$cmdst >> 16}]
	echo [format "fn%d     id %04x:%04x class %06x, command %04x (mem %d bme %d intx_dis %d), status %04x" \
		$fn [expr {$id & 0xffff}] [expr {$id >> 16}] [expr {$classrev >> 8}] $cmd \
		[rk_bits $cmd 1 1] [rk_bits $cmd 2 2] [rk_bits $cmd 10 10] $st]
	set bars {}
	for {set i 0} {$i < 6} {incr i} {
		lappend bars [format "BAR%d %s" $i [rk_hex [lindex $h [expr {4 + $i}]]]]
	}
	echo "        [join $bars {, }]"
	lassign [rk_rd [expr {$b + 0x90}] 4] m0 m1 m2 m3
	set mc [expr {$m0 >> 16}]
	set m64 [rk_bits $mc 7 7]
	set addr [expr {$m64 ? ($m2 << 32) | $m1 : $m1}]
	set data [expr {$m64 ? $m3 & 0xffff : $m2 & 0xffff}]
	echo [format "        MSI %s, %d/%d vectors, 64bit %d, addr 0x%x data 0x%04x" \
		[expr {[rk_bits $mc 0 0] ? "on" : "off"}] [expr {1 << [rk_bits $mc 6 4]}] \
		[expr {1 << [rk_bits $mc 3 1]}] $m64 $addr $data]
	lassign [rk_rd [expr {$b + 0xc8}] 3] devctl lnkcap lnkctl
	echo [format "        devctl %04x devsta %04x, lnkctl %04x lnksta %04x" \
		[expr {$devctl & 0xffff}] [expr {$devctl >> 16}] \
		[expr {$lnkctl & 0xffff}] [expr {$lnkctl >> 16}]]
}

proc rk_pcie_bars {{fn 0}} {
	global rk_pcie_lm rk_pcie_at
	set ctl {disabled io32 ? ? mem32 pf-mem32 mem64 pf-mem64}
	lassign [rk_rd [expr {$rk_pcie_lm + 0x240 + $fn * 8}] 2] c0 c1
	set ib [rk_rd [expr {$rk_pcie_at + 0x828 + $fn * 0x40}] 12]
	for {set i 0} {$i < 6} {incr i} {
		set c [expr {$i < 4 ? $c0 : $c1}]
		set f [expr {($c >> (($i % 4) * 8)) & 0xff}]
		set ap [expr {$f & 0x1f}]
		set lo [lindex $ib [expr {$i * 2}]]
		set hi [lindex $ib [expr {$i * 2 + 1}]]
		set k [lindex $ctl [expr {$f >> 5}]]
		if {$k eq "disabled"} {
			echo [format "BAR%d    disabled" $i]
			continue
		}
		echo [format "BAR%d    %-8s %7d KiB -> AXI 0x%x" $i $k \
			[expr {(1 << ($ap + 7)) / 1024}] [expr {($hi << 32) | $lo}]]
	}
}

proc rk_pcie_ob {{n 4}} {
	global rk_pcie_at rk_pcie_ob_base
	for {set r 0} {$r <= $n} {incr r} {
		lassign [rk_rd [expr {$rk_pcie_at + $r * 0x20}] 4] a0 a1 d0 d1
		if {$d0 == 0 && $a0 == 0 && $r > 0} {
			continue
		}
		if {$r == 0} {
			set win "cfg 0xf8000000"
		} else {
			set win [format "0x%08x" [expr {$rk_pcie_ob_base + ($r - 1) * 0x100000}]]
		}
		set type [expr {$d0 & 0xf}]
		set tn [expr {$type == 2 ? "mem" : $type == 10 ? "cfg0" : $type == 11 ? "cfg1" : \
			$type == 12 ? "msg" : $type == 6 ? "io" : [format "type%d" $type]}]
		echo [format "OB r%-2d  %s -> PCI 0x%x, %d bits, %s fn %d (desc0 %s desc1 %s)" \
			$r $win [expr {($a1 << 32) | ($a0 & 0xffffff00)}] [expr {($a0 & 0x3f) + 1}] \
			$tn [rk_bits $d0 31 24] [rk_hex $d0] [rk_hex $d1]]
	}
}

proc rk_pcie {{fn 0}} {
	rk_pcie_client
	rk_pcie_hdr $fn
	rk_pcie_bars $fn
	rk_pcie_ob
}

proc rk_noc {{clear ""}} {
	set loggers {
		slv0 0xffa64000 "masters -> slaves"
		slv1 0xffa68080 "PMU M0 -> PMU domain"
		msch0 0xffa87c80 "-> DDR channel 0"
		msch1 0xffa8fc80 "-> DDR channel 1"
	}
	set code {SLV DEC UNS DISC SEC HIDE TMO ?}
	set opc {RD RDW RDL RDX WR WRW WRC ?7 PRE URG ?a ?b ?c ?d ?e ?f}
	foreach {name base desc} $loggers {
		set v [rk_rd $base 13]
		set vld [expr {[lindex $v 3] & 1}]
		set l0 [lindex $v 5]
		set l1 [lindex $v 6]
		set l3 [lindex $v 8]
		set l5 [lindex $v 10]
		set l7 [lindex $v 12]
		echo [format "%-6s %-22s %s %s %s addr 0x%08x len1 %d master %d axi-id 0x%x user 0x%04x route 0x%05x sec %d" \
			$name $desc [expr {$vld ? "VALID" : "stale"}] \
			[lindex $code [rk_bits $l0 10 8]] [lindex $opc [rk_bits $l0 4 1]] $l3 \
			[rk_bits $l0 27 16] [rk_bits $l5 15 10] [rk_bits $l5 9 0] \
			[expr {$l5 >> 16}] [expr {$l1 & 0x7ffff}] [expr {$l7 & 1}]]
		if {$clear eq "clear" && $vld} {
			global rk_ap
			$rk_ap write_memory [expr {$base + 0x10}] 32 1
		}
	}
}

# DesignWare watchdog at 0xff848000 (Zephyr pcie_shell feeds it, 10 s):
#   rk_wdt        state: enabled, timeout range, counter
#   rk_wdt off    stop it: pulse its reset in the CRU (SRST_P_WDT0,
#                 SOFTRST_CON20[5]); a running DW watchdog cannot be disabled
#                 any other way. Use before looking at a hung board.
#   rk_wdt kick   restart the counter once
set rk_wdt_base 0xff848000
set rk_cru_softrst20 0xff760450

proc rk_wdt {{op ""}} {
	global rk_ap rk_wdt_base rk_cru_softrst20
	if {$op eq "off"} {
		$rk_ap write_memory $rk_cru_softrst20 32 [expr {(1 << 21) | (1 << 5)}]
		$rk_ap write_memory $rk_cru_softrst20 32 [expr {1 << 21}]
	} elseif {$op eq "kick"} {
		$rk_ap write_memory [expr {$rk_wdt_base + 0x0c}] 32 0x76
	} elseif {$op ne ""} {
		echo "rk_wdt \[off|kick\]"
		return
	}
	lassign [rk_rd $rk_wdt_base 5] cr torr ccvr crr stat
	echo [format "wdt     %s, mode %s, timeout range %d (2^%d cycles), counter %s, irq %d" \
		[expr {$cr & 1 ? "ENABLED" : "off"}] [expr {$cr & 2 ? "irq first" : "reset"}] \
		[expr {$torr & 0xf}] [expr {16 + ($torr & 0xf)}] [rk_hex $ccvr] [expr {$stat & 1}]]
}
