#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LPC2623 地面 BDT 记录导出。

v1/v2：单通道气压；v3：气压+温度；v4：气压+温度+板端定点温补结果(0.1Pa)。
原始计数均按“32 位基准 + 16 位差值”还原。
"""
import argparse
import csv
import datetime
import struct
import sys
from pathlib import Path

MAGIC = b"BARO"
SLOTS_OLD = 300
SLOTS_TEMP = 150
SLOTS_FIXED = 90
BIT_BYTES = 38


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_record(data):
    """返回 (rows, stats)。"""
    if len(data) < 5 or data[:4] != MAGIC:
        raise ValueError("magic 不是 BARO")
    version = data[4]
    if version in (1, 2):
        return _parse_single(data, version)
    if version == 3:
        return _parse_temp(data)
    if version == 4:
        return _parse_fixed(data)
    raise ValueError(f"不支持的记录版本 {version}")


def _parse_single(data, version):
    header_len = 40
    need = header_len + 600 + 38
    if len(data) < need:
        raise ValueError(f"记录截断：{len(data)} 字节，至少需要 {need}")
    h = data[:header_len]
    sample_ms = struct.unpack_from("<H", h, 34)[0] if version >= 2 else 100
    count, start_ms, i2c_fail, dropped, max_read, max_gap, missed = struct.unpack_from(
        "<H I H H H H H", h, 6)
    crc = struct.unpack_from("<H", h, 22)[0]
    raw_bytes, flags, slots = struct.unpack_from("<H H H", h, 24)
    base = struct.unpack_from("<i", h, 30)[0]
    if raw_bytes != 600 or slots != SLOTS_OLD:
        raise ValueError(f"记录尺寸不符：raw={raw_bytes} slots={slots}")
    if not (flags & 1):
        raise ValueError("记录未标记为完整")
    raw = data[header_len:header_len + 600]
    bits = data[header_len + 600:header_len + 638]
    if crc != crc16(h[:22] + b"\x00\x00" + h[24:] + raw + bits):
        raise ValueError("CRC16 校验失败")
    valid = sum((bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_OLD))
    if valid != count:
        raise ValueError(f"有效位数 {valid} 与头部 count={count} 不符")
    rows = []
    for slot in range(SLOTS_OLD):
        if (bits[slot // 8] >> (slot % 8)) & 1:
            delta = struct.unpack_from("<h", raw, slot * 2)[0]
            rows.append((slot * sample_ms, base + delta, None))
    stats = {
        "version": version, "count": count, "start_ms": start_ms,
        "i2c_fail": i2c_fail, "dropped": dropped, "max_read_us": max_read,
        "max_gap_ms": max_gap, "missed_ticks": missed,
        "sample_ms": sample_ms, "base": base, "init_failed": bool(h[5] & 1),
    }
    return rows, stats


def _parse_temp(data):
    header_len = 48
    need = header_len + 300 + 300 + 19 + 19
    if len(data) < need:
        raise ValueError(f"记录截断：{len(data)} 字节，至少需要 {need}")
    h = data[:header_len]
    p_count = struct.unpack_from("<H", h, 6)[0]
    start_ms, i2c_fail, dropped, max_read, max_gap, missed = struct.unpack_from(
        "<I H H H H H", h, 8)
    crc = struct.unpack_from("<H", h, 22)[0]
    slots, sample_ms = struct.unpack_from("<H H", h, 24)
    p_base = struct.unpack_from("<i", h, 28)[0]
    t_base = struct.unpack_from("<i", h, 32)[0]
    p_bytes, t_bytes, t_count = struct.unpack_from("<H H H", h, 36)
    if slots != SLOTS_TEMP or sample_ms != 1000:
        raise ValueError(f"双通道尺寸不符：slots={slots} interval={sample_ms}")
    if p_bytes != 300 or t_bytes != 300:
        raise ValueError("双通道差分区尺寸不符")
    p_raw = data[header_len:header_len + 300]
    t_raw = data[header_len + 300:header_len + 600]
    p_bits = data[header_len + 600:header_len + 619]
    t_bits = data[header_len + 619:header_len + 638]
    mixed = bytes(sum(([p_bits[i], t_bits[i]] for i in range(19)), []))
    if crc != crc16(h[:22] + b"\x00\x00" + h[24:] + p_raw + t_raw + mixed):
        raise ValueError("CRC16 校验失败")
    pv = sum((p_bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_TEMP))
    tv = sum((t_bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_TEMP))
    if pv != p_count or tv != t_count:
        raise ValueError(f"有效位数不符：p={pv}/{p_count} t={tv}/{t_count}")
    rows = []
    for slot in range(SLOTS_TEMP):
        p_ok = (p_bits[slot // 8] >> (slot % 8)) & 1
        t_ok = (t_bits[slot // 8] >> (slot % 8)) & 1
        if not (p_ok or t_ok):
            continue
        p = p_base + struct.unpack_from("<h", p_raw, slot * 2)[0] if p_ok else None
        t = t_base + struct.unpack_from("<h", t_raw, slot * 2)[0] if t_ok else None
        rows.append((slot * sample_ms, p, t))
    stats = {
        "version": 3, "p_count": p_count, "t_count": t_count,
        "count": t_count, "start_ms": start_ms, "i2c_fail": i2c_fail,
        "dropped": dropped, "max_read_us": max_read,
        "max_gap_ms": max_gap, "missed_ticks": missed,
        "sample_ms": sample_ms, "p_base": p_base, "t_base": t_base,
        "init_failed": bool(h[5] & 1),
    }
    return rows, stats


def _parse_fixed(data):
    header_len = 56
    need = header_len + 540 + 36
    if len(data) < need:
        raise ValueError(f"记录截断：{len(data)} 字节，至少需要 {need}")
    h = data[:header_len]
    p_count = struct.unpack_from("<H", h, 6)[0]
    start_ms, i2c_fail, dropped, max_read, max_gap, missed = struct.unpack_from(
        "<I H H H H H", h, 8)
    crc = struct.unpack_from("<H", h, 22)[0]
    slots, sample_ms = struct.unpack_from("<H H", h, 24)
    p_base = struct.unpack_from("<i", h, 28)[0]
    t_base = struct.unpack_from("<i", h, 32)[0]
    p_bytes, t_bytes, f_count = struct.unpack_from("<H H H", h, 36)
    f_base = struct.unpack_from("<i", h, 42)[0]
    f_bytes = struct.unpack_from("<H", h, 46)[0]
    max_comp = struct.unpack_from("<H", h, 48)[0]
    if slots != SLOTS_FIXED or sample_ms != 1000:
        raise ValueError(f"定点尺寸不符：slots={slots} interval={sample_ms}")
    if p_bytes != 180 or t_bytes != 180 or f_bytes != 180:
        raise ValueError("定点差分区尺寸不符")
    p_raw = data[header_len:header_len + 180]
    t_raw = data[header_len + 180:header_len + 360]
    f_raw = data[header_len + 360:header_len + 540]
    p_bits = data[header_len + 540:header_len + 552]
    t_bits = data[header_len + 552:header_len + 564]
    f_bits = data[header_len + 564:header_len + 576]
    mixed = bytes(sum(([p_bits[i], t_bits[i], f_bits[i]] for i in range(12)), []))
    if crc != crc16(h[:22] + b"\x00\x00" + h[24:] + p_raw + t_raw + f_raw + mixed):
        raise ValueError("CRC16 校验失败")
    pv = sum((p_bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_FIXED))
    tv = sum((t_bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_FIXED))
    fv = sum((f_bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS_FIXED))
    # 温度有效数没有独立头部字段，按位图统计；固定结果数必须一致。
    if pv != p_count or fv != f_count:
        raise ValueError(f"有效位数不符：p={pv}/{p_count} f={fv}/{f_count}")
    rows = []
    for slot in range(SLOTS_FIXED):
        p_ok = (p_bits[slot // 8] >> (slot % 8)) & 1
        t_ok = (t_bits[slot // 8] >> (slot % 8)) & 1
        f_ok = (f_bits[slot // 8] >> (slot % 8)) & 1
        if not (p_ok or t_ok or f_ok):
            continue
        p = p_base + struct.unpack_from("<h", p_raw, slot * 2)[0] if p_ok else None
        t = t_base + struct.unpack_from("<h", t_raw, slot * 2)[0] if t_ok else None
        f = f_base + struct.unpack_from("<h", f_raw, slot * 2)[0] if f_ok else None
        rows.append((slot * 1000, p, t, f))
    stats = {
        "version": 4, "count": f_count, "p_count": p_count, "t_count": tv,
        "f_count": f_count, "start_ms": start_ms, "i2c_fail": i2c_fail,
        "dropped": dropped, "max_read_us": max_read, "max_gap_ms": max_gap,
        "missed_ticks": missed, "sample_ms": 1000,
        "max_comp_us": max_comp, "p_base": p_base, "t_base": t_base, "f_base": f_base,
        "init_failed": bool(h[5] & 1),
    }
    return rows, stats


def main():
    ap = argparse.ArgumentParser(description="导出 LPC2623 地面 BDT 诊断记录")
    ap.add_argument("--bin", default=r"E:\Tools\TelinkBDT\config\user\Read.bin")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    src = Path(args.bin)
    if not src.is_file():
        sys.exit(f"找不到 Read.bin：{src}")
    try:
        rows, stats = parse_record(src.read_bytes())
    except ValueError as exc:
        sys.exit(f"记录无效：{exc}")
    out = Path(args.out) if args.out else Path("data/logs") / (
        "baro_bdt_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        if stats["version"] == 4:
            w.writerow(["session_ms", "raw_pressure", "raw_temperature",
                        "pressure_01pa"])
        else:
            w.writerow(["session_ms", "raw_pressure", "raw_temperature"])
        for row in rows:
            w.writerow(row)
    print(f"记录有效：{len(rows)} 行，间隔 {stats['sample_ms']}ms，"
          f"I2C失败 {stats['i2c_fail']}，丢样 {stats['dropped']}，"
          f"最大读取 {stats['max_read_us']}us，最大温补 {stats.get('max_comp_us', 0)}us，"
          f"最大控制间隔 {stats['max_gap_ms']}ms，"
          f"漏拍 {stats['missed_ticks']}")
    print(f"CSV：{out}")


if __name__ == "__main__":
    main()
