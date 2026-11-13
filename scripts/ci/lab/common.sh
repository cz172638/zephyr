# Copyright (c) 2026 Jiri Kastner
# SPDX-License-Identifier: Apache-2.0
#
# Shared by the board scripts in this directory. Sourced, not run.
#
# Works the same in the Zephyr CI container (ghcr.io/zephyrproject-rtos/ci,
# Zephyr SDK in /opt/toolchains, Python venv in /opt/python/venv, root) and
# on an Ubuntu 24.04 host with the SDK and a venv of your own.
#
# Layout (west T2: this Zephyr tree is the manifest repository):
#
#   $WS/.west                 west init -l zephyr
#   $WS/zephyr                this checkout (ZEPHYR_BASE)
#   $WS/modules/...           only the modules in WEST_MODULES
#   $WS/fw/<name>             TF-A, U-Boot, ... pinned to a SHA
#   $WS/build/<name>          build directories
#   $OUT                      images, SHA256SUMS, README.md
#
# Variables (environment, defaults):
#   WS          workspace        [parent of this zephyr tree]
#   OUT         output directory [$WS/out]
#   JOBS        make -j          [nproc]
#   WEST_MODULES  west project filter (west config manifest.project-filter)

set -eu

CI_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ZEPHYR_BASE=$(cd "$CI_DIR/../../.." && pwd)
export ZEPHYR_BASE
WS=${WS:-$(dirname "$ZEPHYR_BASE")}
OUT=${OUT:-$WS/out}
JOBS=${JOBS:-$(nproc)}
WEST_MODULES=${WEST_MODULES:-cmsis,cmsis_6,fatfs,mbedtls,tf-psa-crypto}

log() { printf '\n==== %s\n' "$*"; }

# Host packages the CI image does not carry; installed when running as
# root (CI container), otherwise only reported.
host_deps() {
	local missing=() p
	for p in "$@"; do
		dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
	done
	[ ${#missing[@]} -eq 0 ] && return 0
	if [ "$(id -u)" = 0 ]; then
		log "apt-get install ${missing[*]}"
		apt-get update -qq
		DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends \
			"${missing[@]}"
	else
		echo "missing host packages: sudo apt-get install ${missing[*]}" >&2
		return 1
	fi
}

# Zephyr SDK: the CI image has it in /opt/toolchains; elsewhere set
# ZEPHYR_SDK_INSTALL_DIR or run zephyr-sdk-*/setup.sh -c once.
sdk_setup() {
	if [ -z "${ZEPHYR_SDK_INSTALL_DIR:-}" ]; then
		local d
		for d in /opt/toolchains/zephyr-sdk-* "$HOME"/zephyr-sdk-* "$HOME"/sdk/zephyr-sdk-*; do
			[ -x "$d/setup.sh" ] && ZEPHYR_SDK_INSTALL_DIR=$d && break
		done
	fi
	[ -n "${ZEPHYR_SDK_INSTALL_DIR:-}" ] || { echo "no Zephyr SDK found" >&2; return 1; }
	export ZEPHYR_SDK_INSTALL_DIR ZEPHYR_TOOLCHAIN_VARIANT=zephyr
	SDK_GNU=$ZEPHYR_SDK_INSTALL_DIR/gnu
	echo "SDK $ZEPHYR_SDK_INSTALL_DIR ($(cat "$ZEPHYR_SDK_INSTALL_DIR/sdk_version"))"
}

# west T2 workspace around this tree, only the listed modules
west_setup() {
	# container checkouts belong to another uid than root
	[ -n "${CI:-}" ] && git config --global --add safe.directory '*'
	if [ ! -d "$WS/.west" ]; then
		log "west init -l $ZEPHYR_BASE"
		(cd "$WS" && west init -l "$(basename "$ZEPHYR_BASE")")
	fi
	west config -l >/dev/null
	(cd "$WS" && west config manifest.project-filter -- \
		"-.*,$(echo "+$WEST_MODULES" | sed 's/,/,+/g')")
	log "west update ($WEST_MODULES)"
	(cd "$WS" && west update --narrow -o=--depth=1)
}

# fw_fetch <dir> <url> <sha>: shallow checkout of one commit
fw_fetch() {
	local dir=$WS/fw/$1 url=$2 sha=$3
	if [ "$(git -C "$dir" rev-parse HEAD 2>/dev/null)" = "$sha" ]; then
		return 0
	fi
	log "fw $1: $url $sha"
	mkdir -p "$dir"
	git -C "$dir" init -q
	git -C "$dir" remote remove origin 2>/dev/null || true
	git -C "$dir" remote add origin "$url"
	# a SHA from a local path or bundle may need a full fetch
	git -C "$dir" fetch -q --depth=1 origin "$sha" 2>/dev/null ||
		git -C "$dir" fetch -q origin
	git -C "$dir" checkout -q --detach "$sha"
	git -C "$dir" clean -qfdx
}

# readme <title>: $OUT/README.md from stdin plus the commits used
readme() {
	mkdir -p "$OUT"
	{
		echo "# $1"
		echo
		cat
		echo
		echo "## Commits"
		echo
		echo "| tree | commit |"
		echo "|---|---|"
		echo "| zephyr | $(git -C "$ZEPHYR_BASE" rev-parse HEAD) $(git -C "$ZEPHYR_BASE" log -1 --format=%s) |"
		local d
		for d in "$WS"/fw/*; do
			[ -d "$d/.git" ] && echo "| $(basename "$d") | $(git -C "$d" rev-parse HEAD) |"
		done
		echo "| SDK | $(cat "$ZEPHYR_SDK_INSTALL_DIR/sdk_version") |"
	} > "$OUT/README.md"
}

checksums() {
	(cd "$OUT" && find . -maxdepth 1 -type f ! -name SHA256SUMS -printf '%f\n' | sort |
		xargs -r sha256sum > SHA256SUMS)
}
