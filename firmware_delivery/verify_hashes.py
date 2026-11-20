#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按 manifest.json 核对交付镜像的字节数和 SHA256。

用法（在仓库根目录）：
  python firmware_delivery/verify_hashes.py

退出码只由受版本管理的日常镜像、归档镜像和原始基线决定。
构建目录里的通用 Robin_24G_Car_Flash.bin 会被重编覆盖，干净检出时也可能
不存在，因此只打印参考结果，不参与退出码。

只读文件，不烧录、不修改任何镜像。
"""

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def sha256_of(path):
    """计算文件的 SHA256（分块读取，避免一次占满内存）"""
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def check_entry(entry, base):
    """核对一条清单：文件存在、字节数、SHA256"""
    path = ROOT / base / entry["file"]
    label = str(path.relative_to(ROOT))
    if not path.is_file():
        print(f"[缺失] {label}")
        return False
    size = path.stat().st_size
    digest = sha256_of(path)
    ok = (size == entry["bytes"]) and (digest == entry["sha256"].upper())
    status = "OK  " if ok else "FAIL"
    print(f"[{status}] {label}")
    if size != entry["bytes"]:
        print(f"       字节数: 期望 {entry['bytes']}，实际 {size}")
    if digest != entry["sha256"].upper():
        print(f"       SHA256: 期望 {entry['sha256']}")
        print(f"               实际 {digest}")
    return ok


def check_single(entry, required=True):
    """
    核对 build_artifacts / baseline 这种单文件条目。

    required=True：缺失或哈希不符都算失败（用于基线）；
    required=False：只打印参考结果，不参与退出码（用于构建目录通用 .bin）。
    """
    rel = entry.get("file") or entry.get("generic_bin")
    base = entry.get("dir", "")
    path = ROOT / base / rel
    label = str(path.relative_to(ROOT))
    if not path.is_file():
        if required:
            print(f"[缺失] {label}")
            return False
        print(f"[参考-缺失] {label}")
        print("       构建目录的通用 .bin 可能被重编覆盖，干净检出时也可能不存在；"
              "此项不影响交付核对")
        return True
    size = path.stat().st_size
    digest = sha256_of(path)
    expected_size = entry.get("bytes", entry.get("generic_bin_bytes"))
    expected_hash = entry.get("sha256", entry.get("generic_bin_sha256"))
    ok = (size == expected_size) and (digest == expected_hash.upper())
    if ok:
        status = "OK"
    else:
        status = "FAIL" if required else "参考-变化"
    print(f"[{status}] {label}")
    if size != expected_size:
        print(f"       字节数: 期望 {expected_size}，实际 {size}")
    if digest != expected_hash.upper():
        print(f"       SHA256: 期望 {expected_hash}")
        print(f"               实际 {digest}")
    return ok


def main():
    manifest_path = Path(__file__).resolve().parent / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    required_ok = True

    print("== 日常镜像 ==")
    for item in manifest["daily"]:
        required_ok &= check_entry(item, "firmware_delivery")

    print("== 归档镜像 ==")
    for item in manifest["archive"]:
        required_ok &= check_entry(item, "firmware_delivery")

    print("== 构建产物（通用名，仅供参考，不参与退出码） ==")
    print("（该文件会被重编覆盖，干净检出时可能不存在；缺失/变化都不影响交付核对）")
    check_single(manifest["build_artifacts"], required=False)

    print("== 原始基线 ==")
    required_ok &= check_single(manifest["baseline"], required=True)

    print("交付镜像全部通过（构建产物仅供参考）"
          if required_ok else "交付镜像存在不一致，请先核对再烧录")
    return 0 if required_ok else 1


if __name__ == "__main__":
    sys.exit(main())
