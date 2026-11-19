#!/usr/bin/env python3
"""Build the flat "WMSD" image consumed by wmrom.ko.

  python mkflat.py -p firmware/build/mt7981_patch_iq.bin \\
      -w ct3003/analysis/current-20261111/WIFI_RAM_CODE_MT7981.bin \\
      -a ct3003/analysis/current-20261111/7981_WACPU_RAM_CODE_release.bin \\
      -o wmrom.flat

The ROM patch is supplied as its ALPS section only (from offset 0xa0, length
from the big-endian section descriptor).  Both WM and WA containers are
FW_FORMAT_V3: region payloads are concatenated from offset 0 in order, followed
by one 40-byte region tailer per region (last region first) and a 36-byte
common tailer.
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

MAGIC = 0x44534D57
ENTRY = 0x0220A800
PATCH_ADDR = 0x900000
FLAG_PATCH = 0x01
FLAG_WA = 0x02


def patch_section(blob: bytes) -> bytes:
    if blob[0x10:0x14] != b"ALPS":
        raise ValueError("not an ALPS patch")
    _, off, size, addr, dlsize = struct.unpack_from(">IIIII", blob, 0x60)
    if addr != PATCH_ADDR or size != dlsize or off + size > len(blob):
        raise ValueError("unsupported patch descriptor")
    return blob[off:off + size]


def v3_regions(blob: bytes):
    common = struct.unpack_from("<BBBBBBB", blob, len(blob) - 36)
    count = common[2]
    if not count or count > 64:
        raise ValueError("bad region count")
    data = 0
    regions = []
    for i in range(count):
        t = len(blob) - 36 - (count - i) * 40
        _crc, _dlen, _blk, _rsv, addr, dlen, feat = struct.unpack_from("<IIIIIIB", blob, t)
        if dlen > len(blob) or data + dlen > len(blob):
            raise ValueError("region length out of range")
        regions.append((addr, dlen, feat, blob[data:data + dlen]))
        data += dlen
    return regions


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--patch", type=Path, required=True)
    ap.add_argument("-w", "--wm", type=Path, required=True)
    ap.add_argument("-a", "--wa", type=Path, required=False)
    ap.add_argument("-o", "--output", type=Path, required=True)
    args = ap.parse_args()

    patch = patch_section(args.patch.read_bytes())
    regions = [(PATCH_ADDR, len(patch), FLAG_PATCH, patch)]
    for addr, dlen, feat, data in v3_regions(args.wm.read_bytes()):
        flags = (feat & 0xE0) | (feat << 8)
        regions.append((addr, dlen, flags, data))
    if args.wa:
        for addr, dlen, feat, data in v3_regions(args.wa.read_bytes()):
            flags = FLAG_WA | (feat & 0xE0) | (feat << 8)
            regions.append((addr, dlen, flags, data))

    header = struct.pack("<IIII", MAGIC, 1, ENTRY, len(regions))
    table = bytearray()
    payload = bytearray()
    base = 16 + 16 * len(regions)
    for addr, length, flags, data in regions:
        table += struct.pack("<IIII", addr, length, flags, base + len(payload))
        payload += data
    out = header + bytes(table) + bytes(payload)
    args.output.write_bytes(out)
    print(f"{args.output}: {len(out)} bytes, {len(regions)} regions")
    for addr, length, flags, _ in regions:
        print(f"  {addr:#010x} {length:8d} flags={flags:#06x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
