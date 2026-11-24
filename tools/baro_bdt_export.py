#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LPC2623 地面 BDT 诊断记录导出（只读）。

兼容：
  v1/v2  单通道气压，100ms/500ms
  v3     气压+芯片温度双通道，1s
所有原始计数均按“32 位基准 + 16 位差值”还原；温度摄氏度需要厂商
校准系数 A'/B'，本工具先输出原始温度，待实测数据后离线换算。
"""
import argparse
import csv
import datetime
import struct
import sys
from pathlib import Path

MAGIC = b"BARO"
RAW_P_BYTES = 600
RAW_T_BYTES = 300
BIT_BYTES = 38
P_BIT_BYTES = 19
T_BIT_BYTES = 19
TEMP_BIT_BYTES = 19
SLOTS_OLD = 300
SLOTS_TEMP = 150


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_record(data):
    """返回 (rows, stats)，rows=(session_ms, raw_pressure, raw_temp|None)。"""
    if len(data) < 4 or data[:4] != MAGIC:
        raise ValueError("magic 不是 BARO")
    version = data[4]
    if version in (1, 2):
        return _parse_single(data, version)
    if version == 3:
        return _parse_temp(data)
    raise ValueError(f"不支持的记录版本 {version}")


def _parse_single(data, version):
    header_len = 40
    need = header_len + RAW_P_BYTES + BIT_BYTES
    if len(data) < need:
        raise ValueError(f"记录截断：{len(data)} 字节，至少需要 {need}")
    h = data[:header_len]
    sample_ms = struct.unpack_from("<H", h, 34)[0] if version >= 2 else 100
    count, start_ms, i2c_fail, dropped, max_read, max_gap, missed = struct.unpack_from(
        "<H I H H H H H", h, 6)
    crc = struct.unpack_from("<H", h, 22)[0]
    raw_bytes, flags, slots = struct.unpack_from("<H H H", h, 24)
    base = struct.unpack_from("<i", h, 30)[0]
    if raw_bytes != RAW_P_BYTES or slots != SLOTS_OLD:
        raise ValueError(f"记录尺寸不符：raw={raw_bytes} slots={slots}")
    if not (flags & 1):
        raise ValueError("记录未标记为完整")
    raw = data[header_len:header_len + RAW_P_BYTES]
    bits = data[header_len + RAW_P_BYTES:header_len + RAW_P_BYTES + BIT_BYTES]
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
    need = header_len + SLOTS_TEMP * 2 + RAW_T_BYTES + P_BIT_BYTES + T_BIT_BYTES
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
    if p_bytes != SLOTS_TEMP * 2 or t_bytes != RAW_T_BYTES:
        raise ValueError("双通道差分区尺寸不符")
    p_raw = data[header_len:header_len + SLOTS_TEMP * 2]
    t_off = header_len + SLOTS_TEMP * 2
    t_raw = data[t_off:t_off + RAW_T_BYTES]
    p_bits = data[t_off + RAW_T_BYTES:t_off + RAW_T_BYTES + P_BIT_BYTES]
    t_bits = data[t_off + RAW_T_BYTES + P_BIT_BYTES:
                  t_off + RAW_T_BYTES + P_BIT_BYTES + T_BIT_BYTES]
    p_t_bits = bytes(sum(([p_bits[i], t_bits[i]] for i in range(P_BIT_BYTES)), []))
    if crc != crc16(h[:22] + b"\x00\x00" + h[24:]
                    + p_raw + t_raw + p_t_bits):
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
        w.writerow(["session_ms", "raw_pressure", "raw_temperature"])
        for t, p, temp in rows:
            w.writerow([t, "" if p is None else p, "" if temp is None else temp])
    print(f"记录有效：{len(rows)} 行，间隔 {stats['sample_ms']}ms，"
          f"I2C失败 {stats['i2c_fail']}，丢样 {stats['dropped']}，"
          f"最大读取 {stats['max_read_us']}us，最大控制间隔 {stats['max_gap_ms']}ms，"
          f"漏拍 {stats['missed_ticks']}")
    print(f"CSV：{out}")


if __name__ == "__main__":
    main()
