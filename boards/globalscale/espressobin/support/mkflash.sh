#!/bin/sh
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Build the ESPRESSObin SPI NOR image (TF-A BL1/BL2/BL31 + WTMI + Zephyr
# as BL33) and the UART recovery images.
#
#   mkflash.sh <zephyr.bin> [make variables...]
#
# Environment (defaults in brackets):
#   TFA       TF-A tree                           [$HOME/work/tfa]
#   WTP       A3700-utils-marvell tree            [$HOME/work/a3700-utils]
#   MV_DDR    mv-ddr-marvell tree                 [$TFA/contrib/mv-ddr]
#   CRYPTOPP  Crypto++ tree                       [$HOME/work/cryptopp]
#   CLOCKS    CLOCKSPRESET, match the chip marking: C080 -> CPU_800_DDR_800,
#             C100 -> CPU_1000_DDR_800            [CPU_800_DDR_800]
#   DDR_TOPOLOGY  0 = 512 MiB DDR3 1CS, 2 = 1 GiB DDR3 2CS (v5),
#                 see A3700-utils tim/ddr/DDR_TOPOLOGY_*.txt   [2]
#   BOOTDEV   SPINOR or EMMCNORM                  [SPINOR]
#   CM3       Zephyr image for the Cortex-M3 (espressobin/armada3720_cm3)
#             in place of the whole WTMI (sys_init + fuse app); needs the
#             A3700-utils WTMI_EXEC_ADDR support                  [unset]
#
# PCIe endpoint images keep their BAR backing store outside Zephyr (inbound
# translation per BAR), A3700_NS_IMAGE_OFFSET is no longer needed.
#
# Outputs in $TFA/build/a3700/release: flash-image.bin, uart-images.tgz.bin

set -eu

BL33=${1:?usage: $0 <zephyr.bin> [make vars]}
shift
BL33=$(realpath "$BL33")
EXTRA_VARS="$*"

TFA=${TFA:-$HOME/work/tfa}
WTP=${WTP:-$HOME/work/a3700-utils}
MV_DDR=${MV_DDR:-$TFA/contrib/mv-ddr}
CRYPTOPP=${CRYPTOPP:-$HOME/work/cryptopp}
CLOCKS=${CLOCKS:-CPU_800_DDR_800}
DDR_TOPOLOGY=${DDR_TOPOLOGY:-2}
BOOTDEV=${BOOTDEV:-SPINOR}
CM3=${CM3:-}

set --
if [ -n "$CM3" ]; then
	CM3=$(realpath "$CM3")
	# EXEC_ADDR: the a3700_cm3_entry stub (soc/marvell/armada37xx/
	# cm3_entry.S), from the zephyr.elf next to the image
	CM3_ELF=$(dirname "$CM3")/zephyr.elf
	WTMI_EXEC_ADDR=0x$(nm "$CM3_ELF" | awk '/ a3700_cm3_entry$/ {print $1}')
	if [ "$WTMI_EXEC_ADDR" = "0x" ]; then
		echo "no a3700_cm3_entry in $CM3_ELF" >&2
		exit 1
	fi
	export WTMI_EXEC_ADDR
	echo "CM3 entry $WTMI_EXEC_ADDR"
	set -- WTMI_MULTI_IMG="$CM3"
fi

# make only compares timestamps: a different but older BL33 or WTMI
# (e.g. the Marvell one after a CM3= build) would be ignored
rm -f "$TFA"/build/a3700/release/fip.bin "$TFA"/build/a3700/release/wtmi.bin

# one target at a time: both regenerate the same TIM/DDR files
for target in mrvl_flash mrvl_uart; do
	make -C "$TFA" -j"$(nproc)" \
		CROSS_COMPILE=aarch64-linux-gnu- CROSS_CM3=arm-linux-gnueabi- \
		USE_COHERENT_MEM=0 PLAT=a3700 \
		CLOCKSPRESET="$CLOCKS" DDR_TOPOLOGY="$DDR_TOPOLOGY" \
		BOOTDEV="$BOOTDEV" PARTNUM=0 MARVELL_SECURE_BOOT=0 \
		WTP="$WTP" MV_DDR_PATH="$MV_DDR" CRYPTOPP_PATH="$CRYPTOPP" \
		BL33="$BL33" "$@" $EXTRA_VARS "$target"
done

ls -l "$TFA"/build/a3700/release/flash-image.bin \
	"$TFA"/build/a3700/release/uart-images.tgz.bin
