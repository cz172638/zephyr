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
#
# Outputs in $TFA/build/a3700/release: flash-image.bin, uart-images.tgz.bin

set -eu

BL33=${1:?usage: $0 <zephyr.bin> [make vars]}
shift
BL33=$(realpath "$BL33")

TFA=${TFA:-$HOME/work/tfa}
WTP=${WTP:-$HOME/work/a3700-utils}
MV_DDR=${MV_DDR:-$TFA/contrib/mv-ddr}
CRYPTOPP=${CRYPTOPP:-$HOME/work/cryptopp}
CLOCKS=${CLOCKS:-CPU_800_DDR_800}
DDR_TOPOLOGY=${DDR_TOPOLOGY:-2}
BOOTDEV=${BOOTDEV:-SPINOR}

# make only compares timestamps: a different but older BL33 would be ignored
rm -f "$TFA"/build/a3700/release/fip.bin

# one target at a time: both regenerate the same TIM/DDR files
for target in mrvl_flash mrvl_uart; do
	make -C "$TFA" -j"$(nproc)" \
		CROSS_COMPILE=aarch64-linux-gnu- CROSS_CM3=arm-linux-gnueabi- \
		USE_COHERENT_MEM=0 PLAT=a3700 \
		CLOCKSPRESET="$CLOCKS" DDR_TOPOLOGY="$DDR_TOPOLOGY" \
		BOOTDEV="$BOOTDEV" PARTNUM=0 MARVELL_SECURE_BOOT=0 \
		WTP="$WTP" MV_DDR_PATH="$MV_DDR" CRYPTOPP_PATH="$CRYPTOPP" \
		BL33="$BL33" "$@" "$target"
done

ls -l "$TFA"/build/a3700/release/flash-image.bin \
	"$TFA"/build/a3700/release/uart-images.tgz.bin
