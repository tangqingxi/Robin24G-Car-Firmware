#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robin_24G_Car 串口协议（0x269A 六轴帧 / 0x269B 性能帧）的轻量解析与组帧

监视器（uart_sensor_monitor.py）和六面校准工具（accel_six_face_cal.py）共用本模块。
本模块只依赖标准库：不导入串口、numpy、matplotlib，也不涉及任何界面代码，
因此校准工具可以在没有绘图环境的情况下使用。

帧格式（多字节字段一律大端）：
  0x269A 六轴帧（27B）
    0:0x26 1:0x9A 2:seq 3~6:上电毫秒(u32)
    7~12:ax ay az(s16, 2048 LSB/g)  13~18:gx gy gz(s16, 16.4 LSB/dps)
    19~24:陀螺零偏(s16)  25:flags  26:校验和
  0x269B 性能帧（44B）：字段见固件 PerfMeas.c 文件头
"""

import struct

# ---- 六轴波形帧 ----
FRAME_HEAD = b"\x26\x9a"
FRAME_LEN = 27
# ---- 性能统计帧 ----
PERF_HEAD = b"\x26\x9b"
PERF_LEN = 44


def build_frame(seq, t_ms, ax, ay, az, gx, gy, gz, bx, by, bz, flags):
    """按固件格式组一个 27 字节六轴帧（自测/模拟用）"""
    body = bytes([seq & 0xFF]) + t_ms.to_bytes(4, "big") + \
        struct.pack(">hhhhhhhhh", ax, ay, az, gx, gy, gz, bx, by, bz) + \
        bytes([flags])
    return FRAME_HEAD + body + bytes([sum(body) & 0xFF])


def build_perf_frame(seq, build_id, window_ms, task_cnt, task_avg, task_max,
                     imu_cnt, imu_avg, imu_max, late_max,
                     int_min, int_max, miss_win, miss_total, drop_total):
    """按固件格式组一个 44 字节性能统计帧（自测/模拟用，字段见 PerfMeas.c）"""
    body = bytes([1, seq & 0xFF]) + \
        int(build_id).to_bytes(4, "big") + \
        int(window_ms).to_bytes(2, "big") + \
        int(task_cnt).to_bytes(2, "big") + \
        int(task_avg).to_bytes(2, "big") + \
        int(task_max).to_bytes(4, "big") + \
        int(imu_cnt).to_bytes(2, "big") + \
        int(imu_avg).to_bytes(2, "big") + \
        int(imu_max).to_bytes(4, "big") + \
        int(late_max).to_bytes(4, "big") + \
        int(int_min).to_bytes(2, "big") + \
        int(int_max).to_bytes(2, "big") + \
        int(miss_win).to_bytes(2, "big") + \
        int(miss_total).to_bytes(4, "big") + \
        int(drop_total).to_bytes(2, "big") + \
        bytes([0])                      # 保留字节
    return PERF_HEAD + body + bytes([sum(body) & 0xFF])


class FrameParser:
    """
    字节流 → (类型, 帧字典)
      'sensor' = 0x269A 六轴帧（27B）
      'perf'   = 0x269B 性能统计帧（44B）
    两类帧可以任意交错；校验失败时滑动 1 字节重新同步。
    """

    def __init__(self):
        self.buf = bytearray()
        self.good = 0        # 六轴帧数
        self.good_perf = 0   # 性能帧数
        self.bad = 0         # 校验失败次数（两类合计）
        self.t_ms_wrap = 0   # 设备毫秒计数回绕补偿
        self._last_t = None  # 上一帧的设备毫秒（用于判断回绕）

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            pos = self.buf.find(b"\x26")
            if pos < 0:
                self.buf.clear()            # 没有帧头，全部丢弃
                break
            if pos > 0:
                del self.buf[:pos]
            if len(self.buf) < 2:
                break
            kind = self.buf[1]
            if kind == 0x9A:
                flen = FRAME_LEN
            elif kind == 0x9B:
                flen = PERF_LEN
            else:
                del self.buf[0]             # 不是已知帧头，滑一个字节再找
                continue
            if len(self.buf) < flen:
                break

            raw = bytes(self.buf[:flen])
            if (sum(raw[2:flen - 1]) & 0xFF) != raw[flen - 1]:
                self.bad += 1
                del self.buf[0]             # 校验失败：滑一个字节重新同步
                continue

            del self.buf[:flen]
            if kind == 0x9A:
                self.good += 1
                frames.append(("sensor", self._parse_sensor(raw)))
            else:
                self.good_perf += 1
                frames.append(("perf", self._parse_perf(raw)))
        return frames

    def _parse_sensor(self, raw):
        seq = raw[2]
        t_ms = int.from_bytes(raw[3:7], "big")

        # 设备侧毫秒是 u32（约 49.7 天回绕），这里做一次展开保证时间轴单调
        if self._last_t is not None and t_ms < self._last_t - (1 << 31):
            self.t_ms_wrap += 1 << 32
        self._last_t = t_ms
        t_ms += self.t_ms_wrap

        ax, ay, az, gx, gy, gz, bx, by, bz = struct.unpack(">hhhhhhhhh", raw[7:25])
        return {
            "t_ms": t_ms,
            "seq": seq,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "bx": bx, "by": by, "bz": bz,
            "flags": raw[25],
        }

    def _parse_perf(self, raw):
        """性能统计帧（字段偏移与固件 PerfMeas.c 文件头一致，全部大端）"""
        return {
            "ver": raw[2],
            "seq": raw[3],
            "build_id": int.from_bytes(raw[4:8], "big"),
            "window_ms": int.from_bytes(raw[8:10], "big"),
            "task_cnt": int.from_bytes(raw[10:12], "big"),
            "task_avg_us": int.from_bytes(raw[12:14], "big"),
            "task_max_us": int.from_bytes(raw[14:18], "big"),
            "imu_cnt": int.from_bytes(raw[18:20], "big"),
            "imu_avg_us": int.from_bytes(raw[20:22], "big"),
            "imu_max_us": int.from_bytes(raw[22:26], "big"),
            "late_max_us": int.from_bytes(raw[26:30], "big"),
            "int_min_us": int.from_bytes(raw[30:32], "big"),
            "int_max_us": int.from_bytes(raw[32:34], "big"),
            "miss_win": int.from_bytes(raw[34:36], "big"),
            "miss_total": int.from_bytes(raw[36:40], "big"),
            "drop_total": int.from_bytes(raw[40:42], "big"),
        }
