#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 data/MIGRATION_20261122.md 核对迁移后的路径、数量和 SHA256。"""
import hashlib
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
MAP = Path(__file__).resolve().parent / "DATA_MIGRATION.md"
ROW = re.compile(r"^\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|\s*(\d+)\s*\|\s*`([0-9A-F]{64})`\s*\|")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main() -> int:
    rows = []
    for line in MAP.read_text(encoding="utf-8").splitlines():
        m = ROW.match(line)
        if m:
            rows.append(m.groups())
    ok = True
    for old, new, size, digest in rows:
        path = ROOT / new
        if not path.is_file():
            print(f"[缺失] {new}")
            ok = False
            continue
        actual = sha(path)
        good = path.stat().st_size == int(size) and actual == digest
        print(("[OK]   " if good else "[FAIL] ") + new)
        if not good:
            print(f"       期望 {size} / {digest}")
            print(f"       实际 {path.stat().st_size} / {actual}")
            ok = False
        old_path = ROOT / old
        if old != new and old_path.exists():
            print(f"[FAIL] 旧路径仍存在: {old}")
            ok = False

    cal = sorted(p.name for p in (ROOT / "data").glob("*.json"))
    logs = sorted(p.name for p in (ROOT / "data" / "logs").glob("*"))
    print(f"校准 JSON: {len(cal)} {cal}")
    print(f"data/logs: {len(logs)}")
    if len(cal) != 3 or len(logs) != 4:
        ok = False
    print("RESULT OK" if ok else "RESULT FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
