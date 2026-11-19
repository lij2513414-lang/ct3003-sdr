#!/usr/bin/env python3
"""Build the delay-hook payload and the 6-byte stub patch for CT3003 WM.

Outputs (next to this script):
  hook.bin   - code to write at 0x00902000
  patch.bin  - 6 bytes to write at 0x0090044E (j wm_hook ; nop)

The Andes Cygwin tools refuse to create files in some workspace directories,
so all compiler work happens in a temp dir under ct3003/firmware/build.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
FW = HERE.parent.parent.parent / "ct3003" / "firmware"  # ct3003-sdr/ct3003/firmware
sys.path.insert(0, str(FW))

import toolchain  # noqa: E402


def run(args, env, cwd):
    proc = subprocess.run(args, capture_output=True, text=True, env=env, cwd=cwd)
    if proc.returncode:
        raise RuntimeError(f"{args[0]} failed rc={proc.returncode}\n{proc.stdout}\n{proc.stderr}")
    return proc


def main() -> int:
    tools = toolchain.resolve()
    gcc = tools["gcc"]
    ld = tools["ld"]
    objcopy = tools["objcopy"]
    readelf = tools["readelf"]
    env = toolchain.command_env(gcc)

    build_root = FW / "build"
    build_root.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix="iq-hook-", dir=build_root))

    def asm(src: Path, out: Path, flags: list[str]) -> None:
        run([str(gcc), "-c", "-G0", "-ffreestanding", "-fno-builtin", "-fno-pic",
             "-fno-pie", *flags, str(src), "-o", str(out)], env, tmp)

    def link(obj: Path, script: str, out: Path, entry: str) -> None:
        lds = tmp / (out.stem + ".ld")
        lds.write_text(script, encoding="ascii")
        run([str(ld), "-T", str(lds), "--no-undefined", "--fatal-warnings",
             "-e", entry, "-o", str(out), str(obj)], env, tmp)

    def objdump_bin(elf: Path, out: Path) -> bytes:
        run([str(objcopy), "-O", "binary", str(elf), str(out)], env, tmp)
        return out.read_bytes()

    try:
        hook_obj = tmp / "hook.o"
        asm(HERE / "hook.S", hook_obj, [])

        hook_elf = tmp / "hook.elf"
        link(hook_obj, """
ENTRY(wm_hook)
SECTIONS {
  . = 0x00903000;
  .text : { *(.text) *(.text.*) }
  /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
""", hook_elf, "wm_hook")
        hook = objdump_bin(hook_elf, tmp / "hook.bin")
        if len(hook) > 0xF00:
            raise RuntimeError(f"hook payload {len(hook)} bytes exceeds 0xf00")

        patch_obj = tmp / "patch_ram.o"
        asm(HERE / "patch_ram.S", patch_obj, [])
        patch_elf = tmp / "patch_ram.elf"
        link(patch_obj, """
ENTRY(wm_patch)
SECTIONS {
  . = 0x0090044E;
  .text : { *(.text) *(.text.*) }
  /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
""", patch_elf, "wm_patch")
        patch = objdump_bin(patch_elf, tmp / "patch_ram.bin")
        if len(patch) != 10:
            raise RuntimeError(f"expected a 10-byte patch, got {len(patch)}")
        patch_bytes = patch

        print(run([str(readelf), "-h", str(hook_elf)], env, tmp).stdout.splitlines()[0])

        (HERE / "hook.bin").write_bytes(hook)
        (HERE / "patch.bin").write_bytes(patch_bytes)
        print(f"hook.bin  {len(hook)} bytes  {hook[:32].hex()}")
        print(f"patch.bin {len(patch_bytes)} bytes {patch_bytes.hex()}")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())

