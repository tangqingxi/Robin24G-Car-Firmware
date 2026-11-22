#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 manifest 核验当前交付、归档、基线、回退件和常用烧录目录副本。

只读文件，不烧录、不修改镜像。运行：
    python firmware_delivery/verify_hashes.py
"""
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MISSING = []


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def check(rel: str, size: int, sha: str, label: str) -> bool:
    path = ROOT / rel
    if not path.is_file():
        print(f"[缺失] {label}: {rel}")
        MISSING.append(rel)
        return False
    actual_size = path.stat().st_size
    actual_sha = sha256_of(path)
    ok = actual_size == size and actual_sha == sha.upper()
    print(("[OK]   " if ok else "[FAIL] ") + f"{label}: {rel}")
    if actual_size != size:
        print(f"       字节：期望 {size}，实际 {actual_size}")
    if actual_sha != sha.upper():
        print(f"       SHA256：期望 {sha}")
        print(f"               实际 {actual_sha}")
    return ok


def check_entry(entry: dict, base: str = "") -> bool:
    rel = entry["file"]
    if base:
        rel = f"{base}/{rel}"
    return check(rel, int(entry["bytes"]), entry["sha256"], entry.get("role", rel))


def main() -> int:
    manifest = json.loads((Path(__file__).resolve().parent / "manifest.json").read_text(encoding="utf-8"))
    ok = True

    print("== 当前无桨日志应用 / bootloader / 诊断 / 遥控 ==")
    t2 = manifest["task2"]
    for key in ("app", "bootloader", "debug", "remote"):
        ok &= check_entry(t2[key], "firmware_delivery")

    print("== 维护件 ==")
    for item in manifest["daily"]:
        ok &= check_entry(item, "firmware_delivery")

    print("== 已验收回退件 ==")
    for key in ("normal", "arm_debug"):
        ok &= check_entry(manifest["candidates"][key], "firmware_delivery")
    ok &= check_entry(manifest["archive"][0], "firmware_delivery")  # b3cb56f

    print("== 归档唯一镜像 ==")
    for item in manifest["archive"]:
        ok &= check_entry(item, "firmware_delivery")
    ok &= check_entry(manifest["arm_debug"], "firmware_delivery")

    print("== 原始基线 ==")
    b = manifest["baseline"]
    ok &= check(b["file"], int(b["bytes"]), b["sha256"], "原始基线")

    print("== 常用烧录目录具名副本 ==")
    burn = manifest["burn"]
    for item in burn["files"]:
        ok &= check(f"{burn['dir']}/{item['file']}",
                    int(item["bytes"]), item["sha256"], item["role"])

    if MISSING:
        print(f"\n缺失 {len(MISSING)} 个受管文件。")
    print("\n交付镜像全部通过" if ok else "\n交付镜像存在不一致，请先核对再烧录。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
