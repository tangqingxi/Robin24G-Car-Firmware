#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按 manifest.json 核对交付镜像的字节数和 SHA256。

用法（在仓库根目录）：
  python firmware_delivery/verify_hashes.py

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


def check_single(entry):
    """核对 build_artifacts / baseline 这种单文件条目"""
    rel = entry.get("file") or entry.get("generic_bin")
    base = entry.get("dir", "")
    path = ROOT / base / rel
    label = str(path.relative_to(ROOT))
    if not path.is_file():
        print(f"[缺失] {label}")
        return False
    size = path.stat().st_size
    digest = sha256_of(path)
    expected_size = entry.get("bytes", entry.get("generic_bin_bytes"))
    expected_hash = entry.get("sha256", entry.get("generic_bin_sha256"))
    ok = (size == expected_size) and (digest == expected_hash.upper())
    status = "OK  " if ok else "FAIL"
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
    all_ok = True

    print("== 日常镜像 ==")
    for item in manifest["daily"]:
        all_ok &= check_entry(item, "firmware_delivery")

    print("== 归档镜像 ==")
    for item in manifest["archive"]:
        all_ok &= check_entry(item, "firmware_delivery")

    print("== 构建产物（通用名，仅核对，不作为交付路径） ==")
    all_ok &= check_single(manifest["build_artifacts"])

    print("== 原始基线 ==")
    all_ok &= check_single(manifest["baseline"])

    print("全部通过" if all_ok else "存在不一致，请先核对再烧录")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
