"""Offline checks for the CT3003 IQ hook payload and extended ALPS patch."""
from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parent
sys.path.insert(0, str(FIRMWARE / "rom_patch"))

import build_iq_patch as iq  # noqa: E402


class IqPatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.hook = (FIRMWARE / "wm" / "iq" / "hook.bin").read_bytes()
        cls.stub = (FIRMWARE / "wm" / "iq" / "patch.bin").read_bytes()
        cls.stock = iq.STOCK.read_bytes()
        cls.image = iq.build(cls.stock, cls.hook, cls.stub)

    def test_hook_placement(self) -> None:
        self.assertEqual(len(self.stub), 10)
        self.assertEqual(len(self.image), iq.HEADER + iq.NEW_SECTION)
        section = self.image[iq.HEADER:]
        self.assertEqual(section[iq.HOOK_OFF:iq.HOOK_OFF + len(self.hook)], self.hook)

    def test_only_stub_changed_in_stock_section(self) -> None:
        base = self.stock[iq.HEADER:iq.HEADER + iq.SECTION]
        section = self.image[iq.HEADER:iq.HEADER + iq.SECTION]
        changed = [i for i, (a, b) in enumerate(zip(base, section)) if a != b]
        self.assertEqual(changed, list(range(iq.STUB_OFF, iq.STUB_OFF + 10)))
        self.assertEqual(section[iq.STUB_OFF:iq.STUB_OFF + 10], self.stub)

    def test_descriptor_extended(self) -> None:
        desc = struct.unpack_from(">IIIII", self.image, iq.DESC_OFF)
        self.assertEqual(desc, (0x00030002, iq.HEADER, iq.NEW_SECTION,
                                0x00900000, iq.NEW_SECTION))

    def test_prebuilt_image_matches(self) -> None:
        built = (FIRMWARE / "build" / "mt7981_patch_iq.bin").read_bytes()
        self.assertEqual(self.image, built)


if __name__ == "__main__":
    unittest.main()
