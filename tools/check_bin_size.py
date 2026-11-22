#!/usr/bin/env python3
"""检查最终 .bin 的大小上限；超限返回非零，调用方不得继续发布。"""
import argparse
import sys
from pathlib import Path

APP_BASE = 0x4000
APP_END = 0xF000
DEFAULT_LIMIT = APP_END - APP_BASE


def check_bin_size(path: Path, limit: int = DEFAULT_LIMIT) -> bool:
    size = path.stat().st_size
    end = APP_BASE + size
    max_end = APP_BASE + limit
    if end > max_end:
        print(f"[超限] {path}: 0x{APP_BASE:04X}+{size}=0x{end:05X} > "
              f"0x{max_end:05X}")
        return False
    print(f"[尺寸通过] {path}: 0x{APP_BASE:04X}+{size}=0x{end:05X} <= "
          f"0x{max_end:05X}，剩余 {max_end - end} 字节")
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin", type=Path)
    ap.add_argument("--limit", type=int, default=DEFAULT_LIMIT)
    args = ap.parse_args()
    if not args.bin.is_file():
        print(f"[缺失] {args.bin}")
        return 2
    return 0 if check_bin_size(args.bin, args.limit) else 1


if __name__ == "__main__":
    raise SystemExit(main())
