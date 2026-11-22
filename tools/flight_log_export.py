#!/usr/bin/env python3
"""把 BDT 读出的 0x10000..0x1CFFF 原始日志转成 CSV。

日志格式：13 个 4KiB 扇区，每扇区 32B 页头 + 72 条 56B 记录。
只接受页头/记录 CRC 都正确的数据；半页断电时跳过该页。
"""
import argparse
import csv
import math
import struct
from pathlib import Path

BASE = 0x10000
SECTORS = 13
SECTOR = 4096
HDR = 32
REC = 56
MAGIC = 0x4C4F4731


def crc16_seed(crc: int, data: bytes) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc16(data: bytes) -> int:
    return crc16_seed(0xFFFF, data)


def s16(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def q_to_euler(q: list[int]) -> tuple[float, float, float]:
    w, x, y, z = [v / 32768.0 for v in q]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def export(raw: bytes, out: Path) -> tuple[int, int]:
    rows = []
    page_seq = -1
    yaw_unwrap = 0.0
    yaw_prev = None
    while len(raw) < SECTORS * SECTOR:
        raw += b"\xFF" * (SECTORS * SECTOR - len(raw))
    for sec in range(SECTORS):
        base = sec * SECTOR
        hdr = raw[base:base + 16]
        if len(hdr) < 16:
            continue
        magic, ver, session, seq, count, pcrc, hcrc = struct.unpack("<IHHHHHH", hdr)
        if magic != MAGIC or ver != 1:
            continue
        if crc16(hdr[:14]) != hcrc:
            continue
        calc_pcrc = 0xFFFF
        recs = []
        valid = True
        for i in range(min(count, (SECTOR - HDR) // REC)):
            off = base + HDR + i * REC
            rec = raw[off:off + REC]
            if len(rec) != REC or crc16(rec[:54]) != struct.unpack("<H", rec[54:56])[0]:
                valid = False
                break
            calc_pcrc = crc16_seed(calc_pcrc, rec)
            recs.append(rec)
        if not valid or calc_pcrc != pcrc:
            continue
        page_seq = seq
        for rec in recs:
            (t_ms, sess, rseq, qw, qx, qy, qz) = struct.unpack("<IHHiiii", rec[0:24])
            gx, gy, gz = struct.unpack("<hhh", rec[24:30])
            ax, ay, az = struct.unpack("<hhh", rec[30:36])
            yaw_stick, yaw_target, yaw_corr, thrust = struct.unpack("<hhhh", rec[36:44])
            pwm = struct.unpack("<HHHH", rec[44:52])
            flags = struct.unpack("<H", rec[52:54])[0]
            event = (flags >> 8) & 0x0F
            roll, pitch, yaw = q_to_euler([qw, qx, qy, qz])
            if yaw_prev is not None:
                d = yaw - yaw_prev
                if d > 180:
                    yaw_unwrap -= 360
                elif d < -180:
                    yaw_unwrap += 360
            yaw_prev = yaw
            rows.append({
                "page": seq, "board_ms": t_ms, "session": sess,
                "record_seq": rseq,
                "q_w": qw, "q_x": qx, "q_y": qy, "q_z": qz,
                "gyro_x": gx, "gyro_y": gy, "gyro_z": gz,
                "acc_x": ax, "acc_y": ay, "acc_z": az,
                "yaw_stick": yaw_stick, "yaw_target": yaw_target,
                "yaw_corr": yaw_corr, "thrust": thrust,
                "pwm_front": pwm[0], "pwm_rear": pwm[1],
                "pwm_left": pwm[2], "pwm_right": pwm[3],
                "flags": flags, "event": event,
                "imu_ok": (flags >> 0) & 1,
                "att_ready": (flags >> 1) & 1,
                "att_updated": (flags >> 2) & 1,
                "armed": (flags >> 3) & 1,
                "limited": (flags >> 4) & 1,
                "estop": (flags >> 5) & 1,
                "link_loss": (flags >> 6) & 1,
                "locked": (flags >> 7) & 1,
                "roll_deg": round(roll, 3),
                "pitch_deg": round(pitch, 3),
                "yaw_deg": round(yaw, 3),
                "yaw_unwrapped_deg": round(yaw + yaw_unwrap, 3),
            })
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    return len(rows), page_seq + 1


def main() -> int:
    ap = argparse.ArgumentParser(description="FlightLog 原始 Flash 转 CSV")
    ap.add_argument("--bin", required=True, help="0x10000..0x1CFFF 的 BDT 原始 dump")
    ap.add_argument("--out", required=True, help="输出 CSV")
    args = ap.parse_args()
    raw = Path(args.bin).read_bytes()
    n, pages = export(raw, Path(args.out))
    print(f"rows={n} pages={pages} out={args.out}")
    return 0 if n else 1


if __name__ == "__main__":
    raise SystemExit(main())
