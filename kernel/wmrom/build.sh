#!/bin/bash
set -e
SRC_DIR=$(cd "$(dirname "$0")" && pwd)
B=/tmp/wmrom_build
KDIR=/home/l/mt7981-build/build_dir/target-aarch64_cortex-a53_musl/linux-mediatek_mt7981/linux-5.4.284
TC=/home/l/mt7981-toolchains/openwrt-toolchain-24.10.0-mediatek-filogic_gcc-13.3.0_musl.Linux-x86_64/toolchain-aarch64_cortex-a53_gcc-13.3.0_musl

rm -rf "$B"; mkdir -p "$B"
cp "$SRC_DIR"/*.c "$SRC_DIR"/Kbuild "$B/"
make -C "$KDIR" -j8 ARCH=arm64 CROSS_COMPILE="$TC/bin/aarch64-openwrt-linux-musl-" HOSTCC=gcc M="$B" modules
cp "$B/wmrom.ko" "$SRC_DIR/"
ls -l "$SRC_DIR/wmrom.ko"
