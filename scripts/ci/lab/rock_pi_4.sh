#!/usr/bin/env bash
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# ROCK Pi 4 (RK3399): PCIe RC/EP shell, eMMC as boot medium and storage.
# U-Boot TPL/SPL (no Rockchip blobs) + TF-A BL31 + Zephyr BL33. Outputs:
#
#   <img>-<name>-emmc.bin        the image: eMMC from sector 0 (boots from SD too)
#   <img>-<name>-sd-install.bin  micro SD that boots the same and carries
#                                -emmc.bin: "emmc update /SD:/<img>-<name>-emmc.bin"
#
# Variables, beyond common.sh:
#   IMG, NAME                  [153, rp4]
#   TFA_URL/TFA_SHA, UBOOT_URL/UBOOT_SHA

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

TFA_URL=${TFA_URL:-https://github.com/cz172638/arm-trusted-firmware}
TFA_SHA=${TFA_SHA:-e09b846ea342644ec25b51accec8375249d7fc48}       # rk3399-m0-ns
UBOOT_URL=${UBOOT_URL:-https://github.com/cz172638/u-boot}
UBOOT_SHA=${UBOOT_SHA:-ae095c516eea36b7b9b2c279557f1ca43d0b6f88}   # rk3399-perilp-m0
IMG=${IMG:-153}
NAME=${NAME:-rp4}
APP=$ZEPHYR_BASE/samples/boards/rockchip/rk3399/pcie_shell

host_deps gcc-aarch64-linux-gnu make git bc bison flex swig python3-dev \
	libssl-dev libgnutls28-dev uuid-dev dosfstools mtools
sdk_setup
west_setup
fw_fetch tfa-rk3399 "$TFA_URL" "$TFA_SHA"
fw_fetch u-boot "$UBOOT_URL" "$UBOOT_SHA"

log "TF-A BL31 (M0 firmware with arm-zephyr-eabi)"
make -C "$WS/fw/tfa-rk3399" -j"$JOBS" PLAT=rk3399 bl31 \
	CROSS_COMPILE=aarch64-linux-gnu- \
	M0_CROSS_COMPILE="$SDK_GNU/arm-zephyr-eabi/bin/arm-zephyr-eabi-"
BL31=$WS/fw/tfa-rk3399/build/rk3399/release/bl31/bl31.elf

log "U-Boot TPL/SPL"
UB=$WS/build/u-boot-rock-pi-4
make -C "$WS/fw/u-boot" O="$UB" rock-pi-4-rk3399_defconfig
make -C "$WS/fw/u-boot" O="$UB" -j"$JOBS" CROSS_COMPILE=aarch64-linux-gnu- BL31="$BL31"

log "zephyr $IMG-$NAME"
B=$WS/build/rock_pi_4-$NAME
west build -p -d "$B" -b rock_pi_4/rk3399/a53 "$APP"

mkdir -p "$OUT"
perl "$ZEPHYR_BASE/scripts/rockchip/rk3399-image.pl" --bl31 "$BL31" \
	--zephyr "$B/zephyr/zephyr.elf" --uboot "$UB" --installer -o "$WS/build/$IMG-$NAME"
cp "$WS/build/$IMG-$NAME-emmc.bin" "$WS/build/$IMG-$NAME-sd-install.bin" "$OUT/"

readme "ROCK Pi 4 image $IMG-$NAME" <<EOT
Board: Radxa ROCK Pi 4 (RK3399), Zephyr on the A53 cluster as BL33.

| file | use |
|---|---|
| \`$IMG-$NAME-emmc.bin\` | eMMC from sector 0 (also boots from a micro SD) |
| \`$IMG-$NAME-sd-install.bin\` | \`dd if=$IMG-$NAME-sd-install.bin of=/dev/sdX bs=1M conv=fsync\`, boot from it, then \`emmc update /SD:/$IMG-$NAME-emmc.bin\`, remove the card, reset |

BootROM order is SPI, eMMC, SD: an old image in the SPI flash wins, clear it
with \`flash erase\`. The micro SD slot doubles as JTAG (breakout).
EOT
checksums
ls -l "$OUT"
