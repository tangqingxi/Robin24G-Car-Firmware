#!/usr/bin/env python3
"""无 OTA bootloader：从 subdir.mk 的 C_SRCS/OBJS 列表移除 ota_boot。"""
import argparse
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("mk")
args = ap.parse_args()
p = Path(args.mk)
lines = p.read_text(encoding="utf-8").splitlines()


def filter_block(start_prefix):
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith(start_prefix):
            out.append(line)
            i += 1
            kept = []
            while i < len(lines):
                cur = lines[i]
                if cur.strip() == "":
                    break
                if "ota_boot" not in cur:
                    kept.append(cur)
                i += 1
            for j, item in enumerate(kept):
                if j == len(kept) - 1:
                    out.append(item.rstrip(" \\"))
                else:
                    out.append(item)
            continue
        out.append(line)
        i += 1
    return out


lines = filter_block("C_SRCS +=")
lines = filter_block("OBJS +=")
p.write_text("\n".join(lines) + "\n", encoding="utf-8")
print("patched", p)
