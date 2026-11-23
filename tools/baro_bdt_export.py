#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LPC2623 地面 BDT 诊断记录导出（只读）。

固件在 0x10000 的 4KB 扇区写一条记录：
  0..31      头部
  32..631    300 个 s16 原始压力计数
  632..669   300 位有效位图
头部字段（小端）：
  magic, version, init_flag, count, start_ms, i2c_fail, dropped,
  max_read_us, max_gap_ms, missed_ticks, crc16, raw_bytes, flags, slots
"""
import argparse
import csv
import datetime
import struct
import sys
from pathlib import Path

HEADER_LEN = 32
RAW_BYTES = 600
BIT_BYTES = 38
SLOTS = 300
MAGIC = b"BARO"


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_record(data):
    """返回 (samples, stats)；不完整/校验错时抛 ValueError。"""
    if len(data) < HEADER_LEN + RAW_BYTES + BIT_BYTES:
        raise ValueError(f"记录截断：{len(data)} 字节，至少需要 {HEADER_LEN + RAW_BYTES + BIT_BYTES}")
    h = data[:HEADER_LEN]
    if h[0:4] != MAGIC:
        raise ValueError("magic 不是 BARO")
    version = h[4]
    if version != 1:
        raise ValueError(f"不支持的记录版本 {version}")
    count, start_ms, i2c_fail, dropped, max_read, max_gap, missed = struct.unpack_from(
        "<H I H H H H H", h, 6)
    crc = struct.unpack_from("<H", h, 22)[0]
    raw_bytes, flags, slots = struct.unpack_from("<H H H", h, 24)
    if raw_bytes != RAW_BYTES or slots != SLOTS:
        raise ValueError(f"记录尺寸不符：raw={raw_bytes} slots={slots}")
    if not (flags & 1):
        raise ValueError("记录未标记为完整")
    raw = data[HEADER_LEN:HEADER_LEN + RAW_BYTES]
    bits = data[HEADER_LEN + RAW_BYTES:HEADER_LEN + RAW_BYTES + BIT_BYTES]
    if crc != crc16(h[:22] + raw + bits):
        raise ValueError("CRC16 校验失败")
    valid = sum((bits[i // 8] >> (i % 8)) & 1 for i in range(SLOTS))
    if valid != count:
        raise ValueError(f"有效位数 {valid} 与头部 count={count} 不符")
    samples = []
    for slot in range(SLOTS):
        if (bits[slot // 8] >> (slot % 8)) & 1:
            raw_val = struct.unpack_from("<h", raw, slot * 2)[0]
            samples.append((slot * 100, raw_val))
    stats = {
        "version": version,
        "count": count,
        "start_ms": start_ms,
        "i2c_fail": i2c_fail,
        "dropped": dropped,
        "max_read_us": max_read,
        "max_gap_ms": max_gap,
        "missed_ticks": missed,
        "init_failed": bool(h[5] & 1),
    }
    return samples, stats


def main():
    ap = argparse.ArgumentParser(description="导出 LPC2623 地面 BDT 诊断记录")
    ap.add_argument("--bin", default=r"E:\Tools\TelinkBDT\config\user\Read.bin",
                    help="BDT 读回的 Read.bin")
    ap.add_argument("--out", default="",
                    help="输出 CSV；默认 data/logs/baro_bdt_时间.csv")
    args = ap.parse_args()

    src = Path(args.bin)
    if not src.is_file():
        sys.exit(f"找不到 Read.bin：{src}")
    data = src.read_bytes()
    try:
        samples, stats = parse_record(data)
    except ValueError as exc:
        sys.exit(f"记录无效：{exc}")

    out = Path(args.out) if args.out else Path("data/logs") / (
        "baro_bdt_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["session_ms", "raw_pressure"])
        for t_ms, raw in samples:
            w.writerow([t_ms, raw])
    print(f"记录有效：{stats['count']} 样本，I2C失败 {stats['i2c_fail']}，"
          f"丢样 {stats['dropped']}，最大读取 {stats['max_read_us']}us，"
          f"最大控制间隔 {stats['max_gap_ms']}ms，漏拍 {stats['missed_ticks']}")
    print(f"CSV：{out}")


if __name__ == "__main__":
    main()
