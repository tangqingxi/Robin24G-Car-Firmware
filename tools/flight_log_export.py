#!/usr/bin/env python3
"""把 BDT 读出的 0x10000..0x1CFFF 原始日志转成 CSV。

每扇区 4KiB：起始 256B 头，之后 15 个 256B 物理页，每物理页 4 条 56B 记录。
页头/记录 CRC16；未封口尾页若有 CRC 正确的记录，标记为 recovered。
"""
import argparse
import csv
import math
import struct
from pathlib import Path

BASE = 0x10000
SECTORS = 13
SECTOR = 4096
HDR = 256
REC = 56
REC_PER_PHYS = 4
PHYS_PER_SEC = 15
REC_PER_SEC = 60
MAGIC = 0x4C4F4731


def crc16_seed(crc: int, data: bytes) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    return crc16_seed(0xFFFF, data)


def q_to_euler(q: list[int]) -> tuple[float, float, float]:
    w, x, y, z = [v / 32768.0 for v in q]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rec_offset(base: int, index: int) -> int:
    return base + HDR + (index // REC_PER_PHYS) * 256 + (index % REC_PER_PHYS) * REC


def parse_record(rec: bytes, page: int, recovered: bool) -> dict | None:
    if len(rec) != REC:
        return None
    if struct.unpack("<H", rec[54:56])[0] != crc16(rec[:54]):
        return None
    t_ms, session, seq = struct.unpack("<IHH", rec[0:8])
    flags = struct.unpack("<H", rec[52:54])[0]
    rtype = (flags >> 12) & 0x0F
    row = {
        "page": page, "session": session, "record_seq": seq,
        "record_type": "sample" if rtype == 0 else ("event" if rtype == 1 else "summary"),
        "board_ms": t_ms, "flags": flags, "event": (flags >> 8) & 0x0F,
        "recovered": int(recovered),
    }
    if rtype == 2:
        start_ms, end_ms, max_us = struct.unpack("<III", rec[8:20])
        late, skips = struct.unpack("<HH", rec[20:24])
        flash_us, prep_ms = struct.unpack("<HH", rec[24:28])
        records, drops = struct.unpack("<HH", rec[28:32])
        row.update({
            "start_ms": start_ms, "end_ms": end_ms,
            "max_dt_us": max_us, "late": late, "skips": skips,
            "flash_peak_us": flash_us, "prep_ms": prep_ms,
            "data_records": records, "dropped": drops,
            "end_reason": rec[32],
        })
        return row
    q = struct.unpack("<iiii", rec[8:24])
    gyro = struct.unpack("<hhh", rec[24:30])
    acc = struct.unpack("<hhh", rec[30:36])
    yaw_s, yaw_t, yaw_c, thrust = struct.unpack("<hhhh", rec[36:44])
    pwm = struct.unpack("<HHHH", rec[44:52])
    roll, pitch, yaw = q_to_euler(list(q))
    row.update({
        "q_w": q[0], "q_x": q[1], "q_y": q[2], "q_z": q[3],
        "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
        "acc_x": acc[0], "acc_y": acc[1], "acc_z": acc[2],
        "yaw_stick": yaw_s, "yaw_target": yaw_t, "yaw_corr": yaw_c,
        "thrust": thrust,
        "pwm_front": pwm[0], "pwm_rear": pwm[1],
        "pwm_left": pwm[2], "pwm_right": pwm[3],
        "imu_ok": flags & 1, "att_ready": (flags >> 1) & 1,
        "att_updated": (flags >> 2) & 1, "armed": (flags >> 3) & 1,
        "limited": (flags >> 4) & 1, "estop": (flags >> 5) & 1,
        "link_loss": (flags >> 6) & 1, "locked": (flags >> 7) & 1,
        "roll_deg": round(roll, 3), "pitch_deg": round(pitch, 3),
        "yaw_deg": round(yaw, 3),
    })
    return row


def export(raw: bytes, out: Path) -> dict:
    rows = []
    pages = []
    complete = False
    recovered = False
    summary = None
    raw = raw[:SECTORS * SECTOR]
    if len(raw) < SECTORS * SECTOR:
        raw += b"\xFF" * (SECTORS * SECTOR - len(raw))

    for sec in range(SECTORS):
        base = sec * SECTOR
        hdr = raw[base:base + 16]
        magic, ver, session, seq, count, pcrc, hcrc = struct.unpack("<IHHHHHH", hdr)
        valid_hdr = (magic == MAGIC and ver == 1 and crc16(hdr[:14]) == hcrc
                     and count <= REC_PER_SEC)
        records = []
        if valid_hdr:
            calc = 0xFFFF
            ok = True
            for i in range(count):
                off = rec_offset(base, i)
                rec = raw[off:off + REC]
                if struct.unpack("<H", rec[54:56])[0] != crc16(rec[:54]):
                    ok = False
                    break
                calc = crc16_seed(calc, rec)
                records.append(rec)
            if ok and calc == pcrc:
                pages.append((seq, count, session))
                for rec in records:
                    row = parse_record(rec, seq, False)
                    if row:
                        rows.append(row)
                continue
        # 未封口尾页恢复：扫描固定物理页槽，只接受 CRC 正确且会话一致的记录。
        recs = []
        for i in range(REC_PER_SEC):
            off = rec_offset(base, i)
            rec = raw[off:off + REC]
            row = parse_record(rec, seq, True)
            if row:
                recs.append((i, rec, row))
        if recs:
            recovered = True
            for _, _, row in recs:
                rows.append(row)

    summaries = [r for r in rows if r["record_type"] == "summary"]
    if summaries:
        summary = summaries[-1]
        complete = (summary.get("end_reason", 0) != 0
                    and summary.get("data_records", 0) ==
                    len([r for r in rows if r["record_type"] != "summary"]))
    session = rows[0]["session"] if rows else 0
    # 序号缺口（忽略未封口/恢复尾部）
    seqs = [r["record_seq"] for r in rows if r["record_type"] != "summary"]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if (b - a) & 0xFFFF != 1)
    for r in rows:
        r["session_complete"] = int(complete)
        r["session_gaps"] = gaps
        r["summary_session"] = session
    if summary:
        summary["session_complete"] = int(complete)

    fields = [
        "page", "session", "record_seq", "record_type", "board_ms",
        "start_ms", "end_ms", "max_dt_us", "late", "skips",
        "flash_peak_us", "prep_ms", "data_records", "dropped", "end_reason",
        "q_w", "q_x", "q_y", "q_z", "gyro_x", "gyro_y", "gyro_z",
        "acc_x", "acc_y", "acc_z", "yaw_stick", "yaw_target", "yaw_corr",
        "thrust", "pwm_front", "pwm_rear", "pwm_left", "pwm_right",
        "flags", "event", "imu_ok", "att_ready", "att_updated", "armed",
        "limited", "estop", "link_loss", "locked",
        "roll_deg", "pitch_deg", "yaw_deg",
        "recovered", "session_complete", "session_gaps", "summary_session",
    ]
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return {"rows": len(rows), "pages": len(pages), "complete": complete,
            "recovered": recovered, "summary": summary, "gaps": gaps}


def main() -> int:
    ap = argparse.ArgumentParser(description="FlightLog 原始 Flash 转 CSV")
    ap.add_argument("--bin", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    raw = Path(args.bin).read_bytes()
    info = export(raw, Path(args.out))
    print(f"rows={info['rows']} pages={info['pages']} complete={info['complete']} "
          f"recovered={info['recovered']} gaps={info['gaps']} out={args.out}")
    if info["summary"]:
        s = info["summary"]
        print(f"session={s['session']} records={s['data_records']} "
              f"drops={s['dropped']} end={s['end_reason']} "
              f"max_dt_us={s['max_dt_us']} late={s['late']} skips={s['skips']} "
              f"flash_peak_us={s['flash_peak_us']} prep_ms={s['prep_ms']}")
    return 0 if info["rows"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
