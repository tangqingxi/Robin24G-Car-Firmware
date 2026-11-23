#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""气压 BDT 记录导出回归：完整、截断、CRC 错误、CSV 行数。"""
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import baro_bdt_export as ex


def make_record(count=300, corrupt=False):
    raw = bytearray(ex.RAW_BYTES)
    bits = bytearray(ex.BIT_BYTES)
    for i in range(ex.SLOTS):
        struct.pack_into("<h", raw, i * 2, 600 - i) if i < count else struct.pack_into("<h", raw, i * 2, 0)
        if i < count:
            bits[i // 8] |= 1 << (i % 8)
    h = bytearray(32)
    h[0:4] = b"BARO"
    h[4] = 1
    struct.pack_into("<H I H H H H H", h, 6, count, 5000, 0, 0, 123, 17, 2)
    struct.pack_into("<H H H", h, 24, ex.RAW_BYTES, 1, ex.SLOTS)
    crc = ex.crc16(h[:22] + raw + bits)
    if corrupt:
        crc ^= 0xFFFF
    struct.pack_into("<H", h, 22, crc)
    return bytes(h + raw + bits)


def main():
    data = make_record()
    samples, stats = ex.parse_record(data)
    assert len(samples) == 300 and stats["count"] == 300
    assert samples[0] == (0, 600) and samples[-1] == (29900, 301)

    for bad in (data[:100], data[:650]):
        try:
            ex.parse_record(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("截断记录未报错")

    try:
        ex.parse_record(make_record(corrupt=True))
    except ValueError:
        pass
    else:
        raise AssertionError("CRC 错误未报错")

    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "Read.bin"
        out = Path(td) / "baro.csv"
        src.write_bytes(data)
        subprocess.run([sys.executable, str(TOOLS / "baro_bdt_export.py"),
                        "--bin", str(src), "--out", str(out)], check=True)
        lines = out.read_text(encoding="utf-8-sig").splitlines()
        assert len(lines) == 301, len(lines)
    print("PASS baro BDT complete/truncated/CRC/CSV")
    print("RESULT OK")


if __name__ == "__main__":
    main()
