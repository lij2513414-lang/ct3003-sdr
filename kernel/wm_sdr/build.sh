#!/bin/bash
# Build wm_sdr.ko inside WSL against the local OpenWrt kernel tree,
# then copy the .ko back next to this script.
set -e

SRC_DIR=$(cd "$(dirname "$0")" && pwd)
B=/tmp/wm_sdr_build
KDIR=/home/l/mt7981-build/build_dir/target-aarch64_cortex-a53_musl/linux-mediatek_mt7981/linux-5.4.284
TC=/home/l/mt7981-toolchains/openwrt-toolchain-24.10.0-mediatek-filogic_gcc-13.3.0_musl.Linux-x86_64/toolchain-aarch64_cortex-a53_gcc-13.3.0_musl

rm -rf "$B"
mkdir -p "$B"
cp "$SRC_DIR"/*.c "$SRC_DIR"/*.h "$SRC_DIR"/Kbuild "$B/"

make -C "$KDIR" -j8 ARCH=arm64 \
	CROSS_COMPILE="$TC/bin/aarch64-openwrt-linux-musl-" \
	HOSTCC=gcc M="$B" modules

cp "$B/wm_sdr.ko" "$SRC_DIR/"
ls -l "$SRC_DIR/wm_sdr.ko"
