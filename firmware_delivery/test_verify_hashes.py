#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_hashes.py 的退出码口径自测。

覆盖：
  1. 构建目录通用 .bin 缺失：交付镜像仍通过（退出码 0）；
  2. 构建目录通用 .bin 哈希变化：交付镜像仍通过（退出码 0，只显示参考-变化）；
  3. 受版本管理的日常镜像被改动：退出码 1。

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
BASELINE = REPO / "firmware_baseline" / "Robin_24G_Car_Flash" / "Robin_24G_Car_Flash.bin"


class VerifyHashesTest(unittest.TestCase):
    def _make_tree(self, tmp):
        """复制交付目录和基线，构造一个独立的临时仓库根"""
        root = Path(tmp)
        shutil.copytree(DELIVERY, root / "firmware_delivery")
        baseline_dir = root / "firmware_baseline" / "Robin_24G_Car_Flash"
        baseline_dir.mkdir(parents=True)
        shutil.copy2(BASELINE, baseline_dir / BASELINE.name)
        return root

    def _build_bin_path(self, root):
        """按 manifest 的路径返回临时构建产物的位置"""
        manifest = json.loads((root / "firmware_delivery" / "manifest.json").read_text(
            encoding="utf-8"))
        entry = manifest["build_artifacts"]
        return root / entry["dir"] / (entry.get("file") or entry["generic_bin"])

    def _copy_build_bin(self, root, mutate=False):
        """复制真实构建产物；mutate=True 时翻转首字节制造哈希变化"""
        manifest = json.loads((REPO / "firmware_delivery" / "manifest.json").read_text(
            encoding="utf-8"))
        entry = manifest["build_artifacts"]
        src = REPO / entry["dir"] / (entry.get("file") or entry["generic_bin"])
        if not src.is_file():
            self.skipTest("本仓库没有构建产物，跳过“构建产物变化”用例")
        dst = self._build_bin_path(root)
        dst.parent.mkdir(parents=True)
        data = bytearray(src.read_bytes())
        if mutate:
            data[0] ^= 0xFF
        dst.write_bytes(data)

    def _run(self, root):
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        return subprocess.run(
            [sys.executable, str(root / "firmware_delivery" / "verify_hashes.py")],
            cwd=str(root), timeout=30,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding="utf-8", errors="replace", env=env)

    def test_missing_build_artifact_does_not_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._make_tree(tmp)
            # 故意不创建构建目录里的通用 .bin，模拟干净检出
            result = self._run(root)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn("参考-缺失", result.stdout)
            self.assertIn("交付镜像全部通过", result.stdout)

    def test_changed_build_artifact_does_not_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self._make_tree(tmp)
            self._copy_build_bin(root, mutate=True)
            result = self._run(root)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn("参考-变化", result.stdout)
            self.assertIn("交付镜像全部通过", result.stdout)

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
