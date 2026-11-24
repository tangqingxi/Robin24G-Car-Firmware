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
DELIVERY = ROOT / "firmware_delivery"


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def check(path: Path, entry: dict) -> bool:
    rel = path.relative_to(ROOT).as_posix()
    label = entry.get("role", path.name)
    size = entry["bytes"]
    sha = entry["sha256"]
    if not path.is_file():
        print(f"[缺失] {label}: {rel}")
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


def main() -> int:
    manifest = json.loads((DELIVERY / "manifest.json").read_text(encoding="utf-8"))
    current = manifest["current"]
    ok = True

    print("== 当前交付镜像 ==")
    for entry in current.values():
        ok &= check(DELIVERY / entry["file"], entry)

    print("== 归档镜像 ==")
    for entry in manifest["archive"]:
        ok &= check(DELIVERY / entry["file"], entry)

    print("== 原始基线 ==")
    original_dir = ROOT / "firmware_baseline"
    original_manifest = json.loads((original_dir / "manifest.json").read_text(encoding="utf-8"))
    for entry in original_manifest["files"]:
        if entry["file"].endswith(".bin"):
            ok &= check(original_dir / entry["target"] / entry["file"], entry)

    print("== 常用烧录目录具名副本 ==")
    burn = manifest["burn"]
    copies = [(ROOT / burn["dir"] / Path(current[key]["file"]).name, current[key])
              for key in burn["copies"]]
    if any(path.exists() for path, _ in copies):
        for path, entry in copies:
            ok &= check(path, entry)
    else:
        print("[参考-缺失] 本地没有具名副本，可从当前交付目录取镜像。")

    print("\n交付镜像全部通过" if ok else "\n交付镜像存在不一致，请先核对再烧录。")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
