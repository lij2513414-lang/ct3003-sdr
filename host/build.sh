#!/bin/bash
# Build the wmsdr CLI for CT3003 (aarch64 musl), copy it back next to this script.
set -e

SRC_DIR=$(cd "$(dirname "$0")" && pwd)
TC=/home/l/mt7981-toolchains/openwrt-toolchain-24.10.0-mediatek-filogic_gcc-13.3.0_musl.Linux-x86_64/toolchain-aarch64_cortex-a53_gcc-13.3.0_musl

"$TC"/bin/aarch64-openwrt-linux-musl-gcc -O2 -static -Wall -Wextra \
	-o "$SRC_DIR/wmsdr" "$SRC_DIR/wmsdr.c" "$SRC_DIR/wmsdr_io.c" "$SRC_DIR/wmsdr_httpd.c" -lpthread
"$TC"/bin/aarch64-openwrt-linux-musl-gcc -O2 -static -Wall -Wextra \
	-o "$SRC_DIR/wmrom" "$SRC_DIR/wmrom.c"
ls -l "$SRC_DIR/wmsdr" "$SRC_DIR/wmrom"
