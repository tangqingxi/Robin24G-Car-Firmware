#!/usr/bin/env python3
"""BDT 原始 Flash -> CSV，兼容 v1 连续记录和 v2 256B 物理页布局。"""
import argparse
import csv
import math
import struct
from pathlib import Path

BASE = 0x10000
TOTAL = 0x1D000 - BASE
SECTORS = 13
SECTOR = 4096
MAGIC = 0x4C4F4731
REC = 56

# v1: 32B 头 + 连续 56B；v2: 256B 头 + 每 256B 物理页 4 条
LAYOUT = {
    1: {"hdr": 32, "per_phys": 0, "max": 72},
    2: {"hdr": 256, "per_phys": 4, "max": 60},
}


def crc16_seed(crc, data):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16(data):
    return crc16_seed(0xFFFF, data)


def q_to_euler(q):
    w, x, y, z = [v / 32768.0 for v in q]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rec_offset(base, index, layout, per_phys):
    if layout == 1:
        return base + 32 + index * REC
    return base + 256 + (index // per_phys) * 256 + (index % per_phys) * REC


def kind_name(rtype):
    return {0: "sample", 1: "event", 2: "summary", 3: "baseline", 4: "tail"}.get(rtype, "unknown")


def parse_meta(rec, rtype, page, recovered):
    start, end, max_us = struct.unpack("<III", rec[8:20])
    late, skips = struct.unpack("<HH", rec[20:24])
    flash_us, prep_ms = struct.unpack("<HH", rec[24:28])
    records, drops = struct.unpack("<HH", rec[28:32])
    return {
        "page": page, "session": struct.unpack("<H", rec[4:6])[0],
        "record_seq": struct.unpack("<H", rec[6:8])[0],
        "record_type": kind_name(rtype), "board_ms": struct.unpack("<I", rec[0:4])[0],
        "start_ms": start, "end_ms": end, "max_dt_us": max_us,
        "late": late, "skips": skips, "flash_peak_us": flash_us,
        "prep_ms": prep_ms, "data_records": records, "dropped": drops,
        "end_reason": rec[32], "recovered": int(recovered),
    }


def parse_record(rec, page, recovered):
    if len(rec) != REC or struct.unpack("<H", rec[54:56])[0] != crc16(rec[:54]):
        return None
    flags = struct.unpack("<H", rec[52:54])[0]
    rtype = (flags >> 12) & 0x0F
    if rtype in (2, 3, 4):
        return parse_meta(rec, rtype, page, recovered)
    q = struct.unpack("<iiii", rec[8:24])
    gyro = struct.unpack("<hhh", rec[24:30])
    acc = struct.unpack("<hhh", rec[30:36])
    yaw_s, yaw_t, yaw_c, thrust = struct.unpack("<hhhh", rec[36:44])
    pwm = struct.unpack("<HHHH", rec[44:52])
    roll, pitch, yaw = q_to_euler(list(q))
    return {
        "page": page, "session": struct.unpack("<H", rec[4:6])[0],
        "record_seq": struct.unpack("<H", rec[6:8])[0],
        "record_type": kind_name(rtype), "board_ms": struct.unpack("<I", rec[0:4])[0],
        "q_w": q[0], "q_x": q[1], "q_y": q[2], "q_z": q[3],
        "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
        "acc_x": acc[0], "acc_y": acc[1], "acc_z": acc[2],
        "yaw_stick": yaw_s, "yaw_target": yaw_t, "yaw_corr": yaw_c,
        "thrust": thrust, "pwm_front": pwm[0], "pwm_rear": pwm[1],
        "pwm_left": pwm[2], "pwm_right": pwm[3],
        "imu_ok": flags & 1, "att_ready": (flags >> 1) & 1,
        "att_updated": (flags >> 2) & 1, "armed": (flags >> 3) & 1,
        "limited": (flags >> 4) & 1, "estop": (flags >> 5) & 1,
        "link_loss": (flags >> 6) & 1, "locked": (flags >> 7) & 1,
        "event": (flags >> 8) & 0x0F,
        "roll_deg": round(roll, 3), "pitch_deg": round(pitch, 3),
        "yaw_deg": round(yaw, 3), "recovered": int(recovered),
    }


def export(raw, out):
    length_ok = len(raw) == TOTAL
    if len(raw) < TOTAL:
        raw += b"\xFF" * (TOTAL - len(raw))
    raw = raw[:TOTAL]
    hdr0 = raw[:16]
    magic, ver, _, _, _, _, hcrc = struct.unpack("<IHHHHHH", hdr0)
    if magic != MAGIC or crc16(hdr0[:14]) != hcrc or ver not in LAYOUT:
        raise ValueError("不是支持格式的 FlightLog 区")
    layout = LAYOUT[ver]
    rows, pages, recovered = [], 0, False
    for sec in range(SECTORS):
        base = sec * SECTOR
        hdr = raw[base:base + 16]
        hmagic, hver, session, seq, count, pcrc, hcrc = struct.unpack("<IHHHHHH", hdr)
        valid_hdr = (hmagic == MAGIC and hver == ver and crc16(hdr[:14]) == hcrc
                     and count <= layout["max"])
        if valid_hdr:
            calc, ok = 0xFFFF, True
            recs = []
            for i in range(count):
                off = rec_offset(base, i, ver, layout["per_phys"])
                rec = raw[off:off + REC]
                if struct.unpack("<H", rec[54:56])[0] != crc16(rec[:54]):
                    ok = False
                    break
                calc = crc16_seed(calc, rec)
                recs.append(rec)
            if ok and calc == pcrc:
                pages += 1
                for rec in recs:
                    row = parse_record(rec, seq, False)
                    if row:
                        rows.append(row)
                continue
        # 未封口页恢复：只接受 CRC 正确且会话明确的记录
        for i in range(layout["max"]):
            off = rec_offset(base, i, ver, layout["per_phys"])
            rec = raw[off:off + REC]
            row = parse_record(rec, seq, True)
            if row and row["session"]:
                recovered = True
                rows.append(row)

    # yaw 展开
    last_yaw = None
    offset = 0.0
    for row in rows:
        if row.get("record_type") == "sample" and "yaw_deg" in row:
            yaw = row["yaw_deg"]
            if last_yaw is not None:
                while yaw + offset - last_yaw > 180.0:
                    offset -= 360.0
                while yaw + offset - last_yaw < -180.0:
                    offset += 360.0
            row["yaw_unwrapped_deg"] = round(yaw + offset, 3)
            last_yaw = row["yaw_unwrapped_deg"]

    metas = [r for r in rows if r["record_type"] in ("summary", "tail")]
    tails = [r for r in rows if r["record_type"] == "tail"]
    has_summary = any(r["record_type"] == "summary" for r in rows)
    has_tail = bool(tails)
    sealed = has_summary and has_tail
    summary = (tails or metas or [None])[-1]
    samples = [r for r in rows if r["record_type"] == "sample"]
    data_rows = [r for r in rows if r["record_type"] in ("sample", "event")]
    baseline = [r for r in rows if r["record_type"] == "baseline"]
    seqs = [r["record_seq"] for r in rows if r["record_type"] not in ("summary", "tail", "baseline")]
    dups = sum(1 for a, b in zip(seqs, seqs[1:]) if a == b)
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFFFF) not in (0, 1))
    sessions = {r["session"] for r in rows if r.get("session")}
    complete = bool(
        sealed and summary and length_ok and not recovered and gaps == 0 and dups == 0
        and len(sessions) == 1 and summary.get("end_reason", 0) != 0
        and summary.get("data_records", 0) == len(data_rows)
    )
    for r in rows:
        r["session_complete"] = int(complete)
        r["session_gaps"] = gaps
        r["session_duplicates"] = dups
        r["format_version"] = ver
        r["recovered"] = int(r.get("recovered", 0) or recovered)
        r.setdefault("yaw_unwrapped_deg", "")
        r.setdefault("start_ms", "")
        r.setdefault("end_ms", "")
        r.setdefault("max_dt_us", "")
        r.setdefault("late", "")
        r.setdefault("skips", "")
        r.setdefault("flash_peak_us", "")
        r.setdefault("prep_ms", "")
        r.setdefault("data_records", "")
        r.setdefault("dropped", "")
        r.setdefault("end_reason", "")

    fields = [
        "format_version", "page", "session", "record_seq", "record_type", "board_ms",
        "start_ms", "end_ms", "max_dt_us", "late", "skips", "flash_peak_us",
        "prep_ms", "data_records", "dropped", "end_reason",
        "q_w", "q_x", "q_y", "q_z", "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z", "yaw_stick", "yaw_target", "yaw_corr",
        "thrust", "pwm_front", "pwm_rear", "pwm_left", "pwm_right",
        "flags", "event", "imu_ok", "att_ready", "att_updated", "armed",
        "limited", "estop", "link_loss", "locked", "roll_deg", "pitch_deg",
        "yaw_deg", "yaw_unwrapped_deg", "recovered", "session_complete",
        "session_gaps", "session_duplicates",
    ]
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return {"rows": len(rows), "pages": pages, "complete": complete,
            "recovered": recovered, "summary": summary, "gaps": gaps,
            "dups": dups, "version": ver, "baseline": baseline,
            "length_ok": length_ok, "sessions": sorted(sessions)}


def main():
    ap = argparse.ArgumentParser(description="FlightLog v1/v2 原始 Flash 转 CSV")
    ap.add_argument("--bin", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    info = export(Path(args.bin).read_bytes(), Path(args.out))
    print(f"version={info['version']} rows={info['rows']} pages={info['pages']} "
          f"complete={info['complete']} recovered={info['recovered']} "
          f"gaps={info['gaps']} duplicates={info['dups']} "
          f"length_ok={info['length_ok']} sessions={info['sessions']} out={args.out}")
    if info["summary"]:
        s = info["summary"]
        print(f"session={s['session']} records={s['data_records']} drops={s['dropped']} "
              f"end={s['end_reason']} max_dt_us={s['max_dt_us']} late={s['late']} "
              f"skips={s['skips']} flash_peak_us={s['flash_peak_us']} prep_ms={s['prep_ms']}")
    for b in info["baseline"]:
        print(f"baseline session={b['session']} max_dt_us={b['max_dt_us']} "
              f"late={b['late']} skips={b['skips']}")
    return 0 if info["rows"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
