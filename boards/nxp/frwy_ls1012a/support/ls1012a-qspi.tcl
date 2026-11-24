# Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
#
# SPI NOR on the LS1012A QuadSPI controller from OpenOCD, through the
# AXI-AP (AP0) without the CPU: works with whatever runs on the core,
# including nothing bootable in the flash. Same operation sequence as the
# Zephyr flash_fsl_qspi driver (Linux spi-fsl-qspi): one LUT sequence per
# command, IP commands through the FIFOs, reads through the AHB buffer.
#
#   openocd -f board/nxp_frdm-ls1012a.cfg -f ls1012a-qspi.tcl
#   > halt
#   > qspi_init                     ;# reset and set up the controller, prints JEDEC ID
#   > qspi_read 0 32                ;# hex dump
#   > qspi_flash nor-zephyr-shell.tcl     ;# erase, program, verify
#
# The image comes as a Tcl file made on the host by qspi-image.pl
# (OpenOCD's Jim Tcl has no binary file access): start offset, length and
# the 64 byte chunks that are not erased (all 0xff), as big-endian words.

if {[catch {axi cget -type}]} {
	target create axi mem_ap -dap ls1012a.dap -ap-num 0
	targets ls1012a.cpu
}
# created at run time it is not examined yet; the CPU need not be reachable
axi arp_examine

set QSPI   0x01550000
set QMEM   0x40000000
set SEQ    15
set q_lastlut {}

proc now {} {
	if {[catch {clock seconds} t]} {
		return 0
	}
	return $t
}

proc bswap {v} {
	return [expr {(($v & 0xff) << 24) | (($v & 0xff00) << 8) |
		      (($v >> 8) & 0xff00) | (($v >> 24) & 0xff)}]
}

# big-endian controller registers
proc q_wr {off v} {
	global QSPI
	axi mww [expr {$QSPI + $off}] [bswap $v]
}

proc q_rd {off} {
	global QSPI
	return [bswap [lindex [axi read_memory [expr {$QSPI + $off}] 32 1] 0]]
}

proc q_wait_clear {off mask} {
	for {set i 0} {$i < 1000} {incr i} {
		if {([q_rd $off] & $mask) == 0} {
			return
		}
	}
	error [format "qspi: reg 0x%x mask 0x%x stuck" $off $mask]
}

proc q_invalidate {} {
	set mcr [q_rd 0x000]
	q_wr 0x000 [expr {$mcr | 0x3}]
	q_wr 0x000 [expr {$mcr & ~0x3}]
}

proc lut_def {idx ins opr} {
	return [expr {((($ins << 10) | $opr) << (($idx % 2) * 16)) & 0xffffffff}]
}

# cmd, address bytes, address, dummy bytes, data direction (in/out/none), length
proc q_lut {cmd alen addr dummy dir len} {
	global SEQ
	set lut {0 0 0 0}
	lset lut 0 [lut_def 0 1 $cmd]
	set idx 1
	for {set i 0} {$i < $alen} {incr i} {
		set b [expr {($addr >> (8 * ($alen - $i - 1))) & 0xff}]
		lset lut [expr {$idx / 2}] [expr {[lindex $lut [expr {$idx / 2}]] | [lut_def $idx 4 $b]}]
		incr idx
	}
	if {$dummy} {
		lset lut [expr {$idx / 2}] [expr {[lindex $lut [expr {$idx / 2}]] | [lut_def $idx 3 [expr {$dummy * 8}]]}]
		incr idx
	}
	if {$len} {
		set ins [expr {$dir eq "out" ? 8 : 7}]
		lset lut [expr {$idx / 2}] [expr {[lindex $lut [expr {$idx / 2}]] | [lut_def $idx $ins 0]}]
	}
	global q_lastlut
	if {$lut eq $q_lastlut} {
		return
	}
	set q_lastlut $lut
	q_wr 0x300 0x5af05af0
	q_wr 0x304 0x2
	for {set i 0} {$i < 4} {incr i} {
		q_wr [expr {0x310 + 4 * ($SEQ * 4 + $i)}] [lindex $lut $i]
	}
	q_wr 0x300 0x5af05af0
	q_wr 0x304 0x1
}

# one operation; out: list of bytes, returns the bytes read
proc q_op {cmd {alen 0} {addr 0} {dummy 0} {dir none} {len 0} {out {}}} {
	global QMEM SEQ
	q_wait_clear 0x15c 0x6
	if {$len} {
		q_wr 0x000 [expr {0xf000c | 0xc00}]
		q_wr 0x16c 0x101
	}
	q_lut $cmd $alen $addr $dummy $dir $len
	set in {}
	if {$dir eq "in" && $len > 124} {
		set in [axi read_memory $QMEM 8 $len]
	} else {
		q_wr 0x110 0x11f
		if {$dir eq "out"} {
			for {set i 0} {$i < $len} {incr i 4} {
				# bytes in memory order, little-endian into the word (as Linux)
				set w 0
				for {set j 3} {$j >= 0} {incr j -1} {
					set b [lindex $out [expr {$i + $j}]]
					if {$b eq ""} {
						set b 0
					}
					set w [expr {($w << 8) | $b}]
				}
				q_wr 0x154 $w
			}
		}
		q_wr 0x160 0xffffffff
		q_wr 0x008 [expr {($SEQ << 24) | $len}]
		for {set i 0} {$i < 1000} {incr i} {
			if {[q_rd 0x160] & 1} {
				break
			}
		}
		if {$dir eq "in"} {
			for {set i 0} {$i < $len} {incr i 4} {
				set w [q_rd [expr {0x200 + $i}]]
				for {set j 0} {$j < 4 && $i + $j < $len} {incr j} {
					lappend in [expr {($w >> (8 * $j)) & 0xff}]
				}
			}
		}
	}
	if {$dir ne "in"} {
		# the flash changed (or may have): drop the AHB buffer
		q_invalidate
	}
	return $in
}

proc q_busy_wait {} {
	for {set i 0} {$i < 100000} {incr i} {
		if {([lindex [q_op 0x05 0 0 0 in 1] 0] & 1) == 0} {
			return
		}
	}
	error "qspi: flash stays busy"
}

proc qspi_init {} {
	global QMEM q_lastlut
	q_wr 0x000 0x3
	q_wr 0x000 [expr {0x4000 | 0xf0000}]
	q_wr 0x108 [expr {[q_rd 0x108] & ~0x70061}]
	foreach r {0x030 0x034 0x038} {
		q_wr $r 0
	}
	q_wr 0x020 [expr {15 << 12}]
	q_wr 0x110 0x1f
	q_wr 0x01c [expr {0x80000000 | ((1024 / 8) << 8)}]
	q_wr 0x180 [expr {$QMEM + 1024}]
	q_wr 0x184 [expr {$QMEM + 2048}]
	q_wr 0x188 [expr {$QMEM + 3072}]
	q_wr 0x18c [expr {$QMEM + 4096}]
	q_wr 0x000 [expr {0xf0000 | 0xc}]
	q_wr 0x160 0xffffffff
	q_wr 0x164 0
	# fixed for every operation: chip select A1, buffers 0-2 unused
	q_wr 0x100 $QMEM
	foreach r {0x010 0x014 0x018} {
		q_wr $r 0xe
	}
	set q_lastlut {}
	set id [q_op 0x9f 0 0 0 in 3]
	echo [format "qspi: JEDEC %02x %02x %02x" {*}$id]
}

# read <len> bytes from <off>, fast read in 1 KiB AHB chunks
proc q_readbytes {off len} {
	set data {}
	while {$len > 0} {
		set n [expr {$len > 1024 ? 1024 : $len}]
		if {$n > 124} {
			set n [expr {$n & ~7}]
		}
		lappend data {*}[q_op 0x0b 3 $off 1 in $n]
		incr off $n
		incr len -$n
	}
	return $data
}

proc qspi_read {off len} {
	set data [q_readbytes $off $len]
	for {set i 0} {$i < [llength $data]} {incr i 16} {
		set line [format "%06x:" [expr {$off + $i}]]
		foreach b [lrange $data $i [expr {$i + 15}]] {
			append line [format " %02x" $b]
		}
		echo $line
	}
}

proc qspi_erase {off len} {
	for {set a $off} {$a < $off + $len} {incr a 0x10000} {
		q_op 0x06
		q_op 0xd8 3 $a
		q_busy_wait
	}
}

# Fast path for programming: fixed LUT sequences, the address in SFAR
# (LUT ADDR, 24 bits) instead of single bytes, so a page program costs a
# couple of register writes besides the data; AHB reads cover the whole
# flash (verify by memory reads).
set FLASH_SIZE 0x200000

proc q_fast_init {} {
	global QMEM FLASH_SIZE q_lastlut
	# per sequence 4 words, two instructions each (CMD, ADDR 24, DUMMY 8, READ, WRITE)
	set lut {
		1 {0x00000406 0 0 0}
		2 {0x1c000405 0 0 0}
		3 {0x08180402 0x00002000 0 0}
		4 {0x0818040b 0x1c000c08 0 0}
		5 {0x081804d8 0 0 0}
	}
	q_wr 0x300 0x5af05af0
	q_wr 0x304 0x2
	foreach {seq words} $lut {
		for {set i 0} {$i < 4} {incr i} {
			q_wr [expr {0x310 + 4 * ($seq * 4 + $i)}] [lindex $words $i]
		}
	}
	q_wr 0x300 0x5af05af0
	q_wr 0x304 0x1
	set q_lastlut {}
	q_wr 0x000 [expr {0x4000 | 0xf0000}]
	q_wr 0x020 [expr {4 << 12}]
	foreach r {0x180 0x184 0x188 0x18c} {
		q_wr $r [expr {$QMEM + $FLASH_SIZE}]
	}
	q_wr 0x110 0x11f
	q_wr 0x000 [expr {0xf0000 | 0xc}]
	q_invalidate
}

proc q_ip {seq len} {
	q_wr 0x160 0xffffffff
	q_wr 0x008 [expr {($seq << 24) | $len}]
	for {set i 0} {$i < 1000} {incr i} {
		if {[q_rd 0x160] & 1} {
			return
		}
	}
	error "qspi: IP command timeout"
}

proc q_fast_wait {} {
	for {set i 0} {$i < 100000} {incr i} {
		q_wr 0x000 [expr {0xf000c | 0x400}]
		q_ip 2 1
		if {([q_rd 0x200] & 1) == 0} {
			return
		}
	}
	error "qspi: flash stays busy"
}

# qspi_flash <image.tcl> [first chunk index]: from a chunk > 0 nothing is
# erased (resume after an interrupted run)
proc qspi_flash {file {first 0}} {
	global QMEM
	source $file
	set elen [expr {($img_len + 0xffff) & ~0xffff}]
	echo [format "qspi: %s, %d bytes at 0x%x, %d chunks" $file $img_len $img_off \
		[llength $img_chunks]]
	q_fast_init
	if {$first == 0} {
		echo [format "qspi: erasing 0x%x" $elen]
		for {set a $img_off} {$a < $img_off + $elen} {incr a 0x10000} {
			q_ip 1 0
			q_wr 0x100 [expr {$QMEM + $a}]
			q_ip 5 0
			q_fast_wait
		}
	}
	set t0 [now]
	set n 0
	foreach c [lrange $img_chunks $first end] {
		lassign $c a words
		q_ip 1 0
		q_wr 0x100 [expr {$QMEM + $img_off + $a}]
		foreach w $words {
			# bytes in memory order, little-endian into the register word
			q_wr 0x154 [bswap $w]
		}
		q_ip 3 [expr {4 * [llength $words]}]
		q_fast_wait
		if {([incr n] % 256) == 0} {
			echo [format "qspi: %d/%d chunks (%d s)" [expr {$first + $n}] \
				[llength $img_chunks] [expr {[now] - $t0}]]
		}
	}
	echo [format "qspi: %d chunks written in %d s, verifying" $n [expr {[now] - $t0}]]
	q_invalidate
	foreach c $img_chunks {
		lassign $c a words
		set got [axi read_memory [expr {$QMEM + $img_off + $a}] 32 [llength $words]]
		set i 0
		foreach w $words g $got {
			if {[bswap $w] != $g} {
				error [format "qspi: verify failed at 0x%06x" [expr {$img_off + $a + 4 * $i}]]
			}
			incr i
		}
	}
	echo [format "qspi: done, %d s" [expr {[now] - $t0}]]
}
