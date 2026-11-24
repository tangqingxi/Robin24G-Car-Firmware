#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_hashes.py 的退出码口径自测。

覆盖：
  1. 本地烧录副本缺失：干净检出仍可验证交付镜像。
  2. 通用构建产物可随编译变化，具名烧录副本必须与交付件一致。
  3. 交付维护镜像被改动时返回失败。

用法（在仓库根目录）：
  python firmware_delivery/test_verify_hashes.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DELIVERY = REPO / "firmware_delivery"
BASELINE = REPO / "firmware_baseline"


class VerifyHashesTest(unittest.TestCase):
    def _make_tree(self, tmp):
        """复制交付目录和基线，构造一个独立的临时仓库根"""
        root = Path(tmp)
        shutil.copytree(DELIVERY, root / "firmware_delivery")
        shutil.copytree(BASELINE, root / "firmware_baseline")
        return root

    def _run(self, root):
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        return subprocess.run(
            [sys.executable, str(root / "firmware_delivery" / "verify_hashes.py")],
            cwd=str(root), timeout=30,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding="utf-8", errors="replace", env=env)

    def test_missing_burn_copies_does_not_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._make_tree(tmp)
            result = self._run(root)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn("参考-缺失", result.stdout)
            self.assertIn("交付镜像全部通过", result.stdout)

    def test_burn_copies_use_delivery_hashes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._make_tree(tmp)
            delivery = root / "firmware_delivery"
            manifest = json.loads((delivery / "manifest.json").read_text(encoding="utf-8"))
            burn = manifest["burn"]
            burn_dir = root / burn["dir"]
            burn_dir.mkdir(parents=True)
            for key in burn["copies"]:
                shutil.copy2(delivery / manifest["current"][key]["file"], burn_dir)
            (burn_dir / "Robin_24G_Car_Flash.bin").write_bytes(b"\x00")
            result = self._run(root)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn("交付镜像全部通过", result.stdout)

            name = Path(manifest["current"]["flight_baseline"]["file"]).name
            (burn_dir / name).write_bytes(b"\x00")
            result = self._run(root)
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn("[FAIL] 当前实飞基线", result.stdout)

    def test_changed_daily_image_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._make_tree(tmp)
            target = root / "firmware_delivery" / "daily" / "Robin_24G_Car_maint_269a.bin"
            data = bytearray(target.read_bytes())
            data[0] ^= 0xFF
            target.write_bytes(data)
            result = self._run(root)
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertIn("交付镜像存在不一致", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
