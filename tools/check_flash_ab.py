#!/usr/bin/env python3
"""只读校验 Flash A/B 诊断写入的两个独立扇区，不依赖首扇区头。"""
import argparse
import struct
from pathlib import Path
from flight_log_export import SECTOR, REC, MAGIC, crc16, crc16_seed

STAT_MAGIC = 0x41425453

def check(raw, sec):
    base = sec * SECTOR
    hdr = raw[base:base + 16]
    magic, ver, session, seq, count, pcrc, hcrc = struct.unpack("<IHHHHHH", hdr)
    if magic != MAGIC or ver != 2 or crc16(hdr[:14]) != hcrc or count != 24:
        return sec, 0, 0, 0, 0, 0, False, "header"
    calc = 0xFFFF
    for i in range(count):
        off = base + 256 + (i // 4) * 256 + (i % 4) * REC
        rec = raw[off:off + REC]
        if struct.unpack("<H", rec[54:56])[0] != crc16(rec[:54]):
            return sec, i, calc, 0, 0, 0, False, "record-crc"
        calc = crc16_seed(calc, rec)
    if calc != pcrc:
        return sec, count, calc, 0, 0, 0, False, "page-crc"
    smagic, total_us, max_us, over, scrc = struct.unpack("<IIIHH", raw[base + 16:base + 32])
    if smagic != STAT_MAGIC or crc16(raw[base + 16:base + 30]) != scrc:
        return sec, count, calc, 0, 0, 0, False, "stats"
    return sec, count, calc, total_us, max_us, over, True, "ok"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--sector", type=int, action="append", default=[10, 11])
    a = ap.parse_args()
    raw = Path(a.bin).read_bytes()
    need = 13 * SECTOR
    if len(raw) < need:
        raw += b"\xFF" * (need - len(raw))
    ok = True
    for sec in a.sector:
        s, n, crc, total, mx, ov, good, why = check(raw, sec)
        print(f"sector={s} records={n} total_us={total} max_us={mx} over={ov} "
              f"page_crc=0x{crc:04X} valid={int(good)} reason={why}")
        ok = ok and good
    print("RESULT OK" if ok else "RESULT FAIL")
    raise SystemExit(0 if ok else 1)

if __name__ == "__main__":
    main()
