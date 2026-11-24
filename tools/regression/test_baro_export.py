#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""气压 BDT 端到端导出回归：固件写出的 Read.bin、32 位基准+16 位差值。"""
import csv
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import baro_bdt_export as ex


def make_record(count=300, base=40000, corrupt=False):
    raw = bytearray(600)
    bits = bytearray(ex.BIT_BYTES)
    for i in range(300):
        if i < count:
            struct.pack_into("<h", raw, i * 2, i)      # base + i，均超过 32767
            bits[i // 8] |= 1 << (i % 8)
    h = bytearray(40)
    h[0:4] = b"BARO"
    h[4] = 1
    struct.pack_into("<H I H H H H H", h, 6, count, 5000, 0, 0, 123, 17, 2)
    struct.pack_into("<H H H", h, 24, 600, 1, 300)
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
    assert samples[0] == (0, 40000, None) and samples[-1] == (29900, 40299, None), samples[:2]

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

    fixed = Path("baro_fixed_read.bin")
    if fixed.is_file():
        frames, stats = ex.parse_record(fixed.read_bytes())
        assert len(frames) == 90 and stats["sample_ms"] == 1000, stats
        assert stats["p_count"] == 90 and stats["f_count"] == 90, stats
        assert frames[0][1:] == (1390050, 92995, 1016812), frames[0]
        assert frames[-1][0] == 89000, frames[-1]
        print("PASS fixed: 90 samples, raw pressure/temperature/fixed 0.1Pa")

    temp = Path("baro_temp_read.bin")
    if temp.is_file():
        frames, stats = ex.parse_record(temp.read_bytes())
        assert len(frames) == 150 and stats["sample_ms"] == 1000, stats
        assert stats["p_count"] == 150 and stats["t_count"] == 150, stats
        assert frames[0][1:] == (40000, 93000), frames[0]
        assert frames[-1][0] == 149000, frames[-1]
        assert frames[-1][1:] == (40149, 93149), frames[-1]
        assert stats["version"] == 4
        calibration = stats["calibration"]
        assert calibration[1] == (100000, -12037, 2201, -311, 212, -187, -420, 512, -33)
        p, t = ex.compensated_readings(*frames[0][1:], calibration)
        # 按规格书以独立十进制计算得到的参考值。
        assert abs(p - 100937.84207073484) < 1e-8, p
        assert abs(t - 30.04695214665) < 1e-8, t
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "calibrated.csv"
            subprocess.run([sys.executable, str(TOOLS / "baro_bdt_export.py"),
                            "--bin", str(temp), "--out", str(out)], check=True)
            with out.open(encoding="utf-8-sig", newline="") as fp:
                first = next(csv.DictReader(fp))
            assert first["pressure_pa"] == "100937.842" and first["temperature_c"] == "30.047", first
        corrupt = bytearray(temp.read_bytes())
        corrupt[48] ^= 1
        try:
            ex.parse_record(corrupt)
        except ValueError:
            pass
        else:
            raise AssertionError("校准字节损坏未被 CRC 检出")
        print("PASS temp: 150 samples and factory-calibrated values")

    missing = Path("baro_temp_missing.bin")
    if missing.is_file():
        frames, stats = ex.parse_record(missing.read_bytes())
        assert len(frames) == 150 and stats["t_count"] == 149, stats
        assert sum(1 for row in frames if row[2] is None) == 1
        assert ex.compensated_readings(*frames[0][1:], stats["calibration"]) == (None, None)
        print("PASS temp missing: one temperature slot blank")

    no_calibration = Path("baro_temp_cal_missing.bin")
    if no_calibration.is_file():
        frames, stats = ex.parse_record(no_calibration.read_bytes())
        assert len(frames) == 150 and stats["calibration"] is None, stats
        assert ex.compensated_readings(*frames[0][1:], stats["calibration"]) == (None, None)
        print("PASS missing factory calibration: raw samples still readable")

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
        times = {row[0] for row in frames}
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
