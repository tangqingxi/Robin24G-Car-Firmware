#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""气压 BDT 端到端导出回归：固件写出的 Read.bin、32 位基准+16 位差值。"""
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import baro_bdt_export as ex


def make_record(count=300, base=40000, corrupt=False):
    raw = bytearray(ex.RAW_BYTES)
    bits = bytearray(ex.BIT_BYTES)
    for i in range(ex.SLOTS):
        if i < count:
            struct.pack_into("<h", raw, i * 2, i)      # base + i，均超过 32767
            bits[i // 8] |= 1 << (i % 8)
    h = bytearray(ex.HEADER_LEN)
    h[0:4] = b"BARO"
    h[4] = 1
    struct.pack_into("<H I H H H H H", h, 6, count, 5000, 0, 0, 123, 17, 2)
    struct.pack_into("<H H H", h, 24, ex.RAW_BYTES, 1, ex.SLOTS)
    struct.pack_into("<i", h, 30, base)
    crc = ex.crc16(h[:22] + b"\x00\x00" + h[24:] + raw + bits)
    if corrupt:
        crc ^= 0xFFFF
    struct.pack_into("<H", h, 22, crc)
    return bytes(h + raw + bits)


def main():
    firmware = Path("baro_read.bin")
    data = firmware.read_bytes() if firmware.is_file() else make_record()
    samples, stats = ex.parse_record(data)
    assert len(samples) == 300 and stats["count"] == 300, (len(samples), stats)
    base = stats["base"]
    assert base == 40000, base
    assert samples[0] == (0, 40000) and samples[-1] == (29900, 40299), samples[:2]

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

    drift = Path("baro_read_drift.bin")
    if drift.is_file():
        frames, stats = ex.parse_record(drift.read_bytes())
        assert len(frames) == 300 and stats["sample_ms"] == 500, stats
        assert frames[-1][0] == 149500, frames[-1]
        print("PASS drift: 300 samples, 500ms, last=149500ms")

    for name, expect, missing_slot in (
            ("baro_read_initial.bin", 299, 0),
            ("baro_read_miss.bin", 299, 1),
            ("baro_read_initial_drift.bin", 299, 0),
            ("baro_read_miss_drift.bin", 299, 1)):
        path = Path(name)
        if not path.is_file():
            continue
        frames, _ = ex.parse_record(path.read_bytes())
        assert len(frames) == expect, (name, len(frames))
        times = {t for t, _ in frames}
        assert missing_slot * 100 not in times, (name, missing_slot)
        print(f"PASS {name}: {len(frames)} samples, slot {missing_slot} empty")

    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "Read.bin"
        out = Path(td) / "baro.csv"
        src.write_bytes(data)
        subprocess.run([sys.executable, str(TOOLS / "baro_bdt_export.py"),
                        "--bin", str(src), "--out", str(out)], check=True)
        lines = out.read_text(encoding="utf-8-sig").splitlines()
        assert len(lines) == 301, len(lines)
    print("PASS baro BDT firmware Read.bin, 300 samples, 32-bit base + delta")
    print("RESULT OK")


if __name__ == "__main__":
    main()
