#!/usr/bin/env python3
"""飞控最终镜像打包：先检查 45,056 字节上限，再复制到交付目录。

超限或输入缺失时返回非零，且不创建/覆盖交付文件。
用法：
  python tools/package_flight_control.py <build.bin> <delivery.bin>
"""
import argparse
import shutil
import sys
from pathlib import Path

from check_bin_size import DEFAULT_LIMIT, check_bin_size


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--limit", type=int, default=DEFAULT_LIMIT)
    args = ap.parse_args()

    if not args.src.is_file():
        print(f"[缺失] {args.src}")
        return 2
    if not check_bin_size(args.src, args.limit):
        print("[停止发布] 未复制交付镜像")
        return 1

    args.dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.dst.with_name(args.dst.name + ".tmp")
    shutil.copyfile(args.src, tmp)
    tmp.replace(args.dst)
    print(f"[已发布] {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
