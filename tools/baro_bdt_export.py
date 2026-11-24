#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LPC2623 地面 BDT 导出，兼容 v1/v2/v3/v4/v5。

v4：含出厂校准块的旧温补记录，输出 Pa / °C。
v5：定点诊断，输出原始气压、实际温度、板端 0.1Pa 补偿值。
"""
import argparse
import csv
import datetime
import struct
import sys
from pathlib import Path

MAGIC = b"BARO"
SCALES = (524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960)
SLOTS_FIXED = 90


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _signed(value, width):
    sign = 1 << (width - 1)
    return (value ^ sign) - sign


def _decode_cal(block, cfg):
    p = block[6:]
    osr = cfg & 0x0F
    if osr >= len(SCALES) or p in (bytes(20), b"\xff" * 20):
        return None
    gain = _signed(block[0], 8)
    dvbe_trim = _signed(block[1] >> 1, 7)
    vbe_trim = _signed((block[2] << 1) | (block[1] & 1), 9)
    adc = gain * 8.4375e-5 + 0.675
    vbe = (vbe_trim * 1.05031e-4 + 0.463232422) / adc
    dvbe = (dvbe_trim * 1.25885e-5 + 0.04027621) / adc
    tcal = 5030 * dvbe - 273.15
    vbe27 = vbe - (tcal - 27) * (-1.735e-3)
    ptat = (0.687027 - vbe27) * (-0.8) + 0.039
    a = 5030 * (vbe + 9.45 * dvbe) * (1 + ptat)
    b = -273.15 * (1 + ptat) - ptat * tcal
    c = (
        _signed((p[0] << 12) | (p[1] << 4) | (p[2] >> 4), 20),
        _signed(((p[2] & 0x0F) << 16) | (p[3] << 8) | p[4], 20),
        _signed((p[5] << 12) | (p[6] << 4) | (p[7] >> 4), 20),
        _signed(((p[7] & 0x0F) << 16) | (p[8] << 8) | p[9], 20),
        _signed(((p[10] & 0x7F) << 8) | p[11], 15),
        _signed(((p[12] & 0x0F) << 8) | p[13], 12),
        _signed((p[14] << 9) | (p[15] << 1) | (p[16] >> 7), 17),
        _signed(((p[16] & 0x7F) << 10) | (p[17] << 2) | (p[18] >> 6), 17),
        _signed(((p[18] & 0x3F) << 8) | p[19], 14),
    )
    return SCALES[osr], c, a, b


def reference_v4(p_raw, t_raw, cal):
    if cal is None or t_raw is None:
        return None, None
    scale, c, a, b = cal
    tx = t_raw / 1048576
    tc = a * tx / (1 + 9.45 * tx) + b
    if p_raw is None:
        return None, tc
    p = (~p_raw) / scale
    s = 8.5 * tx / (1 + 8.8 * tx)
    c00, c10, c01, c02, c20, c30, c11, c12, c21 = c
    pa = (c00 + c10 * p + c01 * s + c02 * s * s + c20 * p * p
          + c30 * p * p * p + c11 * p * s + c12 * p * s * s + c21 * p * p * s)
    return pa, tc


def parse_record(data):
    if len(data) < 5 or data[:4] != MAGIC:
        raise ValueError("magic 不是 BARO")
    v = data[4]
    if v in (1, 2):
        return _parse_single(data, v)
    if v == 3:
        return _parse_v3(data)
    if v == 4:
        return _parse_v4(data)
    if v == 5:
        return _parse_v5(data)
    raise ValueError(f"不支持的记录版本 {v}")


def _parse_single(data, v):
    if len(data) < 678:
        raise ValueError("单通道记录截断")
    h = data[:40]
    sample_ms = struct.unpack_from("<H", h, 34)[0] if v >= 2 else 100
    count, start, i2c, drop, mread, mgap, miss = struct.unpack_from("<H I H H H H H", h, 6)
    crc = struct.unpack_from("<H", h, 22)[0]
    raw_bytes, flags, slots = struct.unpack_from("<H H H", h, 24)
    base = struct.unpack_from("<i", h, 30)[0]
    if slots != 300 or raw_bytes != 600 or sample_ms not in (100, 500) or not (flags & 1):
        raise ValueError("单通道记录的长度、间隔或完成标记无效")
    raw = data[40:640]
    bits = data[640:678]
    if crc != crc16(h[:22] + b"\0\0" + h[24:] + raw + bits):
        raise ValueError("CRC16 校验失败")
    rows = []
    for s in range(300):
        if (bits[s // 8] >> (s % 8)) & 1:
            rows.append((s * sample_ms, base + struct.unpack_from("<h", raw, s * 2)[0], None))
    if len(rows) != count:
        raise ValueError("单通道有效样本计数不符")
    return rows, {"version": v, "sample_ms": sample_ms, "start_ms": start, "base": base,
                  "init_failed": bool(h[5] & 1), "i2c_fail": i2c,
                  "dropped": drop, "max_read_us": mread, "max_gap_ms": mgap,
                  "missed_ticks": miss, "count": count}


def _parse_v3(data):
    if len(data) < 686:
        raise ValueError("双通道 v3 记录截断")
    h = data[:48]
    p_base = struct.unpack_from("<i", h, 28)[0]
    t_base = struct.unpack_from("<i", h, 32)[0]
    pcount, start, i2c, drop, mread, mgap, miss = struct.unpack_from("<H I H H H H H", h, 6)
    tcount = struct.unpack_from("<H", h, 40)[0]
    crc = struct.unpack_from("<H", h, 22)[0]
    p = data[48:348]
    t = data[348:648]
    pb = data[648:667]
    tb = data[667:686]
    mixed = bytes(sum(([pb[i], tb[i]] for i in range(19)), []))
    if crc != crc16(h[:22] + b"\0\0" + h[24:] + p + t + mixed):
        raise ValueError("CRC16 校验失败")
    rows = []
    for s in range(150):
        po = (pb[s // 8] >> (s % 8)) & 1
        to = (tb[s // 8] >> (s % 8)) & 1
        if po or to:
            rows.append((s * 1000,
                         p_base + struct.unpack_from("<h", p, s * 2)[0] if po else None,
                         t_base + struct.unpack_from("<h", t, s * 2)[0] if to else None))
    return rows, {"version": 3, "sample_ms": 1000, "start_ms": start, "i2c_fail": i2c,
                  "dropped": drop, "max_read_us": mread, "max_gap_ms": mgap,
                  "missed_ticks": miss, "count": len(rows), "p_count": pcount, "t_count": tcount}


def _parse_v4(data):
    if len(data) < 712:
        raise ValueError("双通道 v4 记录截断")
    h = data[:74]
    pcount, start, i2c, drop, mread, mgap, miss = struct.unpack_from("<H I H H H H H", h, 6)
    p_base = struct.unpack_from("<i", h, 28)[0]
    t_base = struct.unpack_from("<i", h, 32)[0]
    pbytes, tbytes, tcount = struct.unpack_from("<H H H", h, 36)
    cfg = h[42]
    crc = struct.unpack_from("<H", h, 22)[0]
    p = data[74:374]
    t = data[374:674]
    pb = data[674:693]
    tb = data[693:712]
    mixed = bytes(sum(([pb[i], tb[i]] for i in range(19)), []))
    if crc != crc16(h[:22] + b"\0\0" + h[24:] + p + t + mixed):
        raise ValueError("CRC16 校验失败")
    cal = _decode_cal(h[48:74], cfg) if h[5] & 2 else None
    rows = []
    for s in range(150):
        po = (pb[s // 8] >> (s % 8)) & 1
        to = (tb[s // 8] >> (s % 8)) & 1
        if po or to:
            pr = p_base + struct.unpack_from("<h", p, s * 2)[0] if po else None
            tr = t_base + struct.unpack_from("<h", t, s * 2)[0] if to else None
            rows.append((s * 1000, pr, tr))
    return rows, {"version": 4, "sample_ms": 1000, "start_ms": start, "i2c_fail": i2c,
                  "dropped": drop, "max_read_us": mread, "max_gap_ms": mgap,
                  "missed_ticks": miss, "count": pcount,
                  "p_count": pcount, "t_count": tcount, "calibration": cal}


def _parse_v5(data):
    if len(data) < 632:
        raise ValueError("定点 v5 记录截断")
    h = data[:56]
    pcount = struct.unpack_from("<H", h, 6)[0]
    start, i2c, drop, mread, mgap, miss = struct.unpack_from("<I H H H H H", h, 8)
    crc = struct.unpack_from("<H", h, 22)[0]
    slots, sample = struct.unpack_from("<H H", h, 24)
    p_base = struct.unpack_from("<i", h, 28)[0]
    t_base = struct.unpack_from("<i", h, 32)[0]
    pbytes, tbytes, fcount = struct.unpack_from("<H H H", h, 36)
    f_base = struct.unpack_from("<i", h, 42)[0]
    fbytes = struct.unpack_from("<H", h, 46)[0]
    maxcomp = struct.unpack_from("<H", h, 48)[0]
    if slots != SLOTS_FIXED or sample != 1000 or (pbytes, tbytes, fbytes) != (180, 180, 180):
        raise ValueError("定点 v5 记录的长度或采样间隔无效")
    p = data[56:236]
    t = data[236:416]
    f = data[416:596]
    pb = data[596:608]
    tb = data[608:620]
    fb = data[620:632]
    mixed = bytes(sum(([pb[i], tb[i], fb[i]] for i in range(12)), []))
    if crc != crc16(h[:22] + b"\0\0" + h[24:] + p + t + f + mixed):
        raise ValueError("CRC16 校验失败")
    rows = []
    for s in range(SLOTS_FIXED):
        po = (pb[s // 8] >> (s % 8)) & 1
        to = (tb[s // 8] >> (s % 8)) & 1
        fo = (fb[s // 8] >> (s % 8)) & 1
        if po or to or fo:
            pr = p_base + struct.unpack_from("<h", p, s * 2)[0] if po else None
            tr = t_base + struct.unpack_from("<h", t, s * 2)[0] if to else None
            fr = f_base + struct.unpack_from("<h", f, s * 2)[0] if fo else None
            rows.append((s * 1000, pr, tr, fr))
    if sum(row[1] is not None for row in rows) != pcount or sum(row[3] is not None for row in rows) != fcount:
        raise ValueError("定点 v5 有效样本计数不符")
    return rows, {"version": 5, "sample_ms": sample, "start_ms": start, "i2c_fail": i2c,
                  "dropped": drop, "max_read_us": mread, "max_gap_ms": mgap,
                  "missed_ticks": miss, "max_comp_us": maxcomp,
                  "p_count": pcount, "t_count": sum(row[2] is not None for row in rows),
                  "f_count": fcount, "count": fcount}


def main():
    ap = argparse.ArgumentParser(description="导出 LPC2623 地面 BDT 记录")
    ap.add_argument("--bin", default=r"E:\Tools\TelinkBDT\config\user\Read.bin")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    src = Path(a.bin)
    if not src.is_file():
        sys.exit(f"找不到 Read.bin：{src}")
    try:
        rows, st = parse_record(src.read_bytes())
    except ValueError as e:
        sys.exit(f"记录无效：{e}")
    out = Path(a.out) if a.out else Path("data/logs") / (
        "baro_bdt_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        if st["version"] == 4:
            w.writerow(["session_ms", "raw_pressure", "raw_temperature",
                        "pressure_pa", "temperature_c"])
            for t, p, temp in rows:
                pa, tc = reference_v4(p, temp, st.get("calibration"))
                w.writerow([t, p, temp,
                            "" if pa is None else f"{pa:.6f}",
                            "" if tc is None else f"{tc:.6f}"])
        elif st["version"] == 5:
            w.writerow(["session_ms", "raw_pressure", "raw_temperature",
                        "pressure_01pa"])
            for row in rows:
                w.writerow(row)
        else:
            w.writerow(["session_ms", "raw_pressure", "raw_temperature"])
            for row in rows:
                w.writerow(row)
    print(f"记录有效：{len(rows)} 行，间隔 {st['sample_ms']}ms，"
          f"I2C失败 {st['i2c_fail']}，丢样 {st['dropped']}，"
          f"最大读取 {st['max_read_us']}us，最大控制间隔 {st['max_gap_ms']}ms，"
          f"漏拍 {st['missed_ticks']}")
    print(f"CSV：{out}")


if __name__ == "__main__":
    main()
