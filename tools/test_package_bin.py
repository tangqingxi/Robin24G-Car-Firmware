#!/usr/bin/env python3
"""打包尺寸门的回归：超限必须返回非零且不覆盖交付文件。"""
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import package_flight_control as pkg
from check_bin_size import DEFAULT_LIMIT, check_bin_size


class PackageSizeTest(unittest.TestCase):
    def test_under_limit_publishes(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src.bin"
            dst = Path(tmp) / "daily" / "dst.bin"
            src.write_bytes(b"\x00" * 100)
            argv = ["package_flight_control.py", str(src), str(dst)]
            old = sys.argv
            sys.argv = argv
            try:
                self.assertEqual(pkg.main(), 0)
            finally:
                sys.argv = old
            self.assertTrue(dst.is_file())
            self.assertEqual(dst.stat().st_size, 100)
            self.assertFalse(dst.with_name(dst.name + ".tmp").exists())

    def test_over_limit_returns_nonzero_and_keeps_old(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "src.bin"
            dst = Path(tmp) / "daily" / "dst.bin"
            dst.parent.mkdir(parents=True)
            dst.write_bytes(b"OLD")
            src.write_bytes(b"\x00" * (DEFAULT_LIMIT + 1))
            argv = ["package_flight_control.py", str(src), str(dst)]
            old = sys.argv
            sys.argv = argv
            try:
                self.assertEqual(pkg.main(), 1)
            finally:
                sys.argv = old
            self.assertEqual(dst.read_bytes(), b"OLD")
            self.assertFalse(dst.with_name(dst.name + ".tmp").exists())

    def test_check_direct(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / "x.bin"
            src.write_bytes(b"\x00" * DEFAULT_LIMIT)
            self.assertTrue(check_bin_size(src))
            src.write_bytes(b"\x00" * (DEFAULT_LIMIT + 1))
            self.assertFalse(check_bin_size(src))


if __name__ == "__main__":
    unittest.main(verbosity=2)
