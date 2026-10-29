#!/bin/sh
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Put an ESPRESSObin image built with BOOTDEV=SATA on a disk the A3720
# BootROM boots from: a GPT partition of the Marvell boot type
# (6828311A-BA55-42A4-BCDE-A89BB5EDECAE). The TIM flash offsets (TIM 0,
# WTMI 0x4000, OBMI 0x15000) are relative to the start of that partition.
#
#   sata-install.sh [-n] <disk> <flash-image.bin>
#
# -n keeps the existing partition table and only rewrites the boot
# partition (found by its type). Without -n the disk is repartitioned:
# 1: 4 MiB Marvell boot, 2: rest of the disk, Linux filesystem.

set -eu

MRVL=6828311A-BA55-42A4-BCDE-A89BB5EDECAE
KEEP=0
[ "${1:-}" = -n ] && { KEEP=1; shift; }
[ $# -eq 2 ] || { echo "usage: $0 [-n] <disk> <flash-image.bin>" >&2; exit 2; }
DISK=$1 IMG=$2

[ -b "$DISK" ] || { echo "$DISK is not a block device" >&2; exit 1; }
if ! dd if="$IMG" bs=4 count=1 skip=1 2>/dev/null | grep -q 2TAS; then
	echo "$IMG has no SATA boot TIM (build with BOOTDEV=SATA)" >&2
	exit 1
fi

if [ $KEEP -eq 0 ]; then
	lsblk "$DISK"
	printf 'Repartition %s? Everything on it is lost [y/N] ' "$DISK"
	read -r a
	[ "$a" = y ] || exit 1
	sfdisk --wipe always "$DISK" <<-EOT
	label: gpt
	size=4MiB, type=$MRVL, name=a3700-boot
	type=0FC63DAF-8483-4772-8E79-3D69D8477DE4, name=data
	EOT
fi

N=$(sfdisk --dump "$DISK" | grep -i "type=$MRVL" | head -1 | cut -d' ' -f1)
[ -n "$N" ] || { echo "no Marvell boot partition on $DISK" >&2; exit 1; }
partprobe "$DISK" 2>/dev/null || blockdev --rereadpt "$DISK" || true
udevadm settle 2>/dev/null || true
dd if="$IMG" of="$N" bs=64k conv=fsync status=none
echo "$IMG -> $N"
