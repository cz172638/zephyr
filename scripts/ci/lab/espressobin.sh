#!/usr/bin/env bash
# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# ESPRESSObin v5 lab image: netshell with every PCIe EP function, SATA
# and the lab agent; LLEXT debug modules. Outputs in $OUT:
#
#   <img>-<name>-spi.bin    SPI NOR (TF-A + WTMI + Zephyr BL33)
#   <img>-<name>-sata.bin   SATA boot (sata-install.sh)
#   <img>-<name>-uart.tgz   UART recovery images (mox-imager / WtpDownload)
#   <img>-*.llext           debug modules (lab mod load)
#
# Variables, beyond common.sh:
#   IMG       image number, also the LLEXT prefix  [CONFIG_LAB_IMAGE of the snippet]
#   NAME      image name                            [lab-mods]
#   CLOCKS    CPU_800_DDR_800 or CPU_1000_DDR_800    [CPU_800_DDR_800]
#   DDR_TOPOLOGY                                    [2 (1 GiB, v5)]
#   NETSHELL_AUTHORIZED_KEY   PEM public key text (ssh-keygen -e -m PKCS8)
#                             or a path to it; the image is not built without
#   TFA_URL/TFA_SHA, WTP_URL/WTP_SHA, MV_DDR_URL/MV_DDR_SHA,
#   CRYPTOPP_URL/CRYPTOPP_SHA  firmware trees

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

TFA_URL=${TFA_URL:-https://github.com/cz172638/arm-trusted-firmware}
TFA_SHA=${TFA_SHA:-d2064baa83d5fceea085efc5cc5848f1708028fa}          # espressobin-cci
WTP_URL=${WTP_URL:-https://github.com/cz172638/A3700-utils-marvell}
WTP_SHA=${WTP_SHA:-fc61a2101532d62ab46387c11176ff5fa60cbc59}          # espressobin-cm3
MV_DDR_URL=${MV_DDR_URL:-https://github.com/MarvellEmbeddedProcessors/mv-ddr-marvell}
MV_DDR_SHA=${MV_DDR_SHA:-7bcb9dc7ea7fa233bf96bd0350a4ec7c205e342e}    # master
CRYPTOPP_URL=${CRYPTOPP_URL:-https://github.com/weidai11/cryptopp}
CRYPTOPP_SHA=${CRYPTOPP_SHA:-5873a73a231c95771f5bbf3466a484952fb22780} # master

SNIPPETS="espressobin-pcie-ep espressobin-pcie-nvme espressobin-sata espressobin-vntb
	  espressobin-nvme-target espressobin-fabric espressobin-lab"
APP=$ZEPHYR_BASE/samples/boards/globalscale/espressobin/netshell
IMG=${IMG:-$(sed -n 's/^CONFIG_LAB_IMAGE=//p' "$ZEPHYR_BASE/snippets/espressobin-lab/espressobin-lab.conf")}
NAME=${NAME:-lab-mods}
CLOCKS=${CLOCKS:-CPU_800_DDR_800}
DDR_TOPOLOGY=${DDR_TOPOLOGY:-2}

if [ -z "${NETSHELL_AUTHORIZED_KEY:-}" ]; then
	echo "NETSHELL_AUTHORIZED_KEY not set (PEM text or file)" >&2
	exit 1
fi
mkdir -p "$WS/build"
KEY=$WS/build/netshell-authorized-key.pem
if [ -f "$NETSHELL_AUTHORIZED_KEY" ]; then
	cp "$NETSHELL_AUTHORIZED_KEY" "$KEY"
else
	printf '%s\n' "$NETSHELL_AUTHORIZED_KEY" > "$KEY"
fi

host_deps gcc-aarch64-linux-gnu gcc-arm-linux-gnueabi g++ make git
sdk_setup
west_setup
fw_fetch tfa "$TFA_URL" "$TFA_SHA"
fw_fetch a3700-utils "$WTP_URL" "$WTP_SHA"
fw_fetch mv-ddr "$MV_DDR_URL" "$MV_DDR_SHA"
fw_fetch cryptopp "$CRYPTOPP_URL" "$CRYPTOPP_SHA"

B=$WS/build/espressobin-$NAME
snip=()
for s in $SNIPPETS; do snip+=(-S "$s"); done
log "zephyr $IMG-$NAME"
west build -p -d "$B" -b espressobin/armada3720 "$APP" "${snip[@]}" -- \
	-DEXTRA_CONF_FILE=pcie-ep-uartlog.conf \
	-DNETSHELL_AUTHORIZED_KEY="$KEY" \
	-DCONFIG_LAB_IMAGE="$IMG"
# symbol check of the modules against the firmware's exports
west build -d "$B" -t lab_mods

mkdir -p "$OUT"
export TFA=$WS/fw/tfa WTP=$WS/fw/a3700-utils MV_DDR=$WS/fw/mv-ddr CRYPTOPP=$WS/fw/cryptopp
export CLOCKS DDR_TOPOLOGY
REL=$TFA/build/a3700/release
for dev in SPINOR SATA; do
	log "TF-A + WTMI, BOOTDEV=$dev"
	BOOTDEV=$dev sh "$ZEPHYR_BASE/boards/globalscale/espressobin/support/mkflash.sh" \
		"$B/zephyr/zephyr.bin"
	case $dev in
	SPINOR)
		cp "$REL/flash-image.bin" "$OUT/$IMG-$NAME-spi.bin"
		cp "$REL/uart-images.tgz.bin" "$OUT/$IMG-$NAME-uart.tgz"
		;;
	SATA) cp "$REL/flash-image.bin" "$OUT/$IMG-$NAME-sata.bin" ;;
	esac
done
cp "$B"/llext/"$IMG"-*.llext "$OUT/"
cp "$ZEPHYR_BASE/boards/globalscale/espressobin/support/sata-install.sh" "$OUT/"
cp "$ZEPHYR_BASE/scripts/lab/lab.pl" "$OUT/"

readme "ESPRESSObin image $IMG-$NAME" <<EOT
Board: ESPRESSObin v5, $CLOCKS, DDR_TOPOLOGY=$DDR_TOPOLOGY.
Snippets: $(echo $SNIPPETS), EXTRA_CONF_FILE=pcie-ep-uartlog.conf.

| file | use |
|---|---|
| \`$IMG-$NAME-spi.bin\` | SPI NOR: U-Boot \`sf probe; sf update\` or \`fwupdate\` from netshell |
| \`$IMG-$NAME-sata.bin\` | SATA boot: \`sata-install.sh\` |
| \`$IMG-$NAME-uart.tgz\` | UART recovery (mox-imager) |
| \`$IMG-*.llext\` | debug modules: \`lab mod load\` (prefix must match the image) |
| \`lab.pl\` | lab agent host client (UDP 7777) |
EOT
checksums
ls -l "$OUT"
