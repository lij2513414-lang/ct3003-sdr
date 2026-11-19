#!/usr/bin/env python3
"""Build a CT3003 ALPS ROM patch that installs the delay-hook IQ pump.

The stock patch section is 0x25c0 bytes and is completely full.  The patch
format does not cover the section with a CRC (the driver sends
MtCmdPatchFinishReq with ucCheckCrc=0), and its section descriptor carries an
explicit dl_size, so we extend the loaded section to 0x3900 bytes and put our
hook at 0x00903000 (past the stock 0x009025c0 end, still inside the writable
patch SRAM we probed on the device).  Only the 10-byte delay stub at
0x0090044e is rewritten; the other ten stock hooks are untouched.

Outputs: firmware/build/mt7981_patch_iq.bin
"""
from __future__ import annotations

import hashlib
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STOCK = HERE.parent / "stock" / "new_mt7981_patch_e1_hdr.bin"
STOCK_SHA256 = "10a70a53045aeac9cdbceb1ed2bc5f943cb9faaf005b7363c021c2e66c671f19"
HEADER = 0xA0
SECTION = 0x25C0
NEW_SECTION = 0x3900
STUB_OFF = 0x44E
HOOK_ADDR = 0x00903000
HOOK_OFF = HOOK_ADDR - 0x00900000
ALPS_OFF = 0x10
DESC_OFF = 0x60
SECTION_SIZE_OFF = 0x68
DL_SIZE_OFF = 0x70


def build(stock: bytes, hook: bytes, stub_patch: bytes) -> bytes:
    if hashlib.sha256(stock).hexdigest() != STOCK_SHA256:
        raise ValueError("stock patch hash mismatch")
    if stock[ALPS_OFF:ALPS_OFF + 4] != b"ALPS":
        raise ValueError("not an ALPS image")
    sec_type, sec_off, sec_size, dl_addr, dl_size = struct.unpack_from(">IIIII", stock, DESC_OFF)
    if (sec_type, sec_off, sec_size, dl_addr, dl_size) != (0x00030002, HEADER, SECTION, 0x00900000, SECTION):
        raise ValueError("unexpected stock section descriptor")
    if len(stub_patch) != 10:
        raise ValueError("stub patch must be 10 bytes")
    if len(hook) > NEW_SECTION - HOOK_OFF:
        raise ValueError("hook does not fit in the extended section")

    base = stock[HEADER:HEADER + SECTION]
    section = bytearray(NEW_SECTION)
    section[:SECTION] = base
    section[STUB_OFF:STUB_OFF + 10] = stub_patch
    section[HOOK_OFF:HOOK_OFF + len(hook)] = hook

    image = bytearray(stock[:HEADER] + bytes(section))
    struct.pack_into(">I", image, SECTION_SIZE_OFF, NEW_SECTION)
    struct.pack_into(">I", image, DL_SIZE_OFF, NEW_SECTION)
    return bytes(image)


def main() -> int:
    hook = (HERE.parent / "wm" / "iq" / "hook.bin").read_bytes()
    stub = (HERE.parent / "wm" / "iq" / "patch.bin").read_bytes()
    image = build(STOCK.read_bytes(), hook, stub)
    out = HERE.parent / "build" / "mt7981_patch_iq.bin"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(image)
    print(f"{out}: {len(image)} bytes sha256={hashlib.sha256(image).hexdigest()}")
    print(f"hook at 0x{HOOK_ADDR:08X} ({len(hook)} bytes), stub patch at 0x0090044E")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
