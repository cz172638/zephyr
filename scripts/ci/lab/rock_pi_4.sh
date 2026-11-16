#!/usr/bin/env bash
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# ROCK Pi 4 (RK3399): PCIe RC/EP shell; boots from the SPI NOR or the eMMC,
# the eMMC FAT is the storage.
# U-Boot TPL/SPL (no Rockchip blobs) + TF-A BL31 + Zephyr BL33. Outputs:
#
#   <img>-<name>-spi.bin         SPI NOR image (idbloader at 0, FIT at 0xe0000),
#                                for "fwupdate ymodem" (boot_partition)
#   <img>-<name>-emmc.bin        the image: eMMC from sector 0 (boots from SD too)
#   <img>-<name>-boot.bin        its start up to the end of the FIT (MBR,
#                                idbloader, FIT at 1 MiB), for
#                                "emmc update <file>"
#
# Variables, beyond common.sh:
#   IMG, NAME                  [153, rock_pi_4]
#   TFA_URL/TFA_SHA, UBOOT_URL/UBOOT_SHA

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

TFA_URL=${TFA_URL:-https://github.com/cz172638/arm-trusted-firmware}
TFA_SHA=${TFA_SHA:-e09b846ea342644ec25b51accec8375249d7fc48}       # rk3399-m0-ns
UBOOT_URL=${UBOOT_URL:-https://github.com/cz172638/u-boot}
UBOOT_SHA=${UBOOT_SHA:-ae095c516eea36b7b9b2c279557f1ca43d0b6f88}   # rk3399-perilp-m0
IMG=${IMG:-153}
NAME=${NAME:-$BOARD}
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
# FIT at 1 MiB instead of 8 MiB: the boot area image (-boot.bin) stays small
"$WS/fw/u-boot/scripts/config" --file "$UB/.config" --set-val SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 0x800
make -C "$WS/fw/u-boot" O="$UB" olddefconfig
make -C "$WS/fw/u-boot" O="$UB" -j"$JOBS" CROSS_COMPILE=aarch64-linux-gnu- BL31="$BL31"

log "zephyr $IMG-$NAME"
B=$WS/build/rock_pi_4-$NAME
west build -p -d "$B" -b rock_pi_4/rk3399/a53 "$APP"

mkdir -p "$OUT"
perl "$ZEPHYR_BASE/scripts/rockchip/rk3399-image.pl" --bl31 "$BL31" \
	--zephyr "$B/zephyr/zephyr.elf" --uboot "$UB" --spi -o "$WS/build/$IMG-$NAME"
cp "$WS/build/$IMG-$NAME-spi.bin" "$OUT/"
cp "$WS/build/$IMG-$NAME-emmc.bin" "$OUT/"
cp "$WS/build/$IMG-$NAME-boot.bin" "$OUT/"

readme "ROCK Pi 4 image $IMG-$NAME" <<EOT
Board: Radxa ROCK Pi 4 (RK3399), Zephyr on the A53 cluster as BL33.

| file | use |
|---|---|
| \`$IMG-$NAME-spi.bin\` | SPI NOR (boot_partition): \`fwupdate ymodem\`, send it (tio: ctrl-t y), then \`kernel reboot cold\` |
| \`$IMG-$NAME-emmc.bin\` | eMMC from sector 0 (also boots from a micro SD) |
| \`$IMG-$NAME-boot.bin\` | eMMC boot area only (FAT with settings kept): \`emmc update /EMMC:/$IMG-$NAME-boot.bin\`, then \`kernel reboot cold\` |

BootROM order is SPI, eMMC, SD: a valid image in the SPI NOR wins; to boot the
eMMC again erase its first sector: \`flash erase flash@0 0 0x1000\`.
The micro SD slot is for the JTAG breakout only.
EOT
checksums
ls -l "$OUT"
