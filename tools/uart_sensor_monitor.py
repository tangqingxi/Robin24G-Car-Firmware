#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robin_24G_Car 六轴传感器串口监视器（配合固件 0x26 0x9A 波形帧）

线程模型（把"接收"和"界面"彻底分开）：
  ┌ 采集线程 ─────────────────────────────────────────────┐
  │ 串口读取 → 组帧校验 → 逐帧写 CSV → 帧对象放入有界队列 │
  └───────────────────────────────────────────────────────┘
  ┌ 主线程（Tk 事件循环）─────────────────────────────────┐
  │ 定时器每 100ms 取走队列里所有帧 → 追加到绘图环形缓冲   │
  │ 约 10Hz 重画一次曲线；拖动/缩放期间只收数据不重画      │
  └───────────────────────────────────────────────────────┘
  * CSV 由采集线程直接写，不经过界面，因此界面再忙也不会丢 100Hz 数据；
  * 队列有上限，界面消费不及时时丢弃的是"绘图用"的旧帧，不影响 CSV。

帧格式（27 字节，多字节字段大端，详见固件 UserUart.c / FlightCtrl.h 7.12）：
  偏移  长度  内容
   0     1    0x26 帧头1
   1     1    0x9A 帧头2
   2     1    seq        帧序号
   3     4    上电毫秒   u32
   7     6    ax ay az   s16，16g 量程 → 2048 LSB/g
  13     6    gx gy gz   s16，2000dps 量程 → 16.4 LSB/dps
  19     6    bx by bz   s16，陀螺零偏（原始 LSB）
  25     1    flags      位0=已中值滤波 位1=已扣零偏 位2=零偏已标定 位3=已解锁
  26     1    校验和 = sum(byte[2..25]) & 0xFF

快捷键：F 切换"跟随最新数据"，Q 关闭窗口并结束采集。

依赖：pyserial、numpy、matplotlib（只记录 CSV 时可加 --no-plot）
用法示例：
  python uart_sensor_monitor.py --list
  python uart_sensor_monitor.py --port COM3
  python uart_sensor_monitor.py --port COM3 --csv bench.csv --window 20
  python uart_sensor_monitor.py --port COM3 --no-plot --duration 60
  python uart_sensor_monitor.py --simulate --duration 10     # 无硬件自测

台架姿态估计固件（文本协议，--attitude）：
  python uart_sensor_monitor.py --port COM3 --attitude
  python uart_sensor_monitor.py --port COM3 --attitude --duration 120 --csv att.csv
  * 文本行形如 ATT t=... rdy=1 r=+12.34 p=-45.67 y=+120.15 与 STA n=...，
    r/p/y 已经是度；--attitude 下不解析 0x269A/0x269B 二进制帧。
"""

import argparse
import csv
import datetime
import math
import os
import queue
import re
import struct
import sys
import threading
import time
from collections import deque

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("缺少 pyserial：请先执行  python -m pip install pyserial")

# ----------------------------------------------------------------------------
# 帧协议：解析/组帧与六面校准工具共用 uart_proto.py（纯标准库，无绘图依赖）
# ----------------------------------------------------------------------------
from uart_proto import (FRAME_HEAD, FRAME_LEN, PERF_HEAD, PERF_LEN,
                        FrameParser, build_frame, build_perf_frame)

# 传感器量程（与固件 my_t3488.c 的配置一致：±16g / ±2000dps）
DEFAULT_ACC_LSB_PER_G = 2048.0
DEFAULT_GYRO_LSB_PER_DPS = 16.4

# flags 位定义（与 FlightCtrl.h 一致）
FLAG_FILTERED = 0x01
FLAG_BIAS_APPLIED = 0x02
FLAG_CALIBRATED = 0x04
FLAG_ARMED = 0x08

# 界面刷新周期：80ms → 约 10Hz（数据仍按 100Hz 全量接收并写盘）
PLOT_PERIOD_MS = 80

# 六轴 CSV 列
CSV_FIELDS = [
    "t_ms", "seq",
    "ax_lsb", "ay_lsb", "az_lsb",
    "gx_lsb", "gy_lsb", "gz_lsb",
    "bx_lsb", "by_lsb", "bz_lsb",
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "gx_c_dps", "gy_c_dps", "gz_c_dps",
    "flags",
]

# 性能统计 CSV 列（窗内值每秒重置，累计值单调递增；单位写进列名）
PERF_CSV_FIELDS = [
    "recv_s",           # 采集开始后的秒数（本地）
    "recv_epoch",       # 本地 Unix 时间戳（s）
    "seq",              # 统计帧序号（0~255，跳变=漏帧）
    "build_id",         # 固件构建标识（0xXXXXXXXX）
    "window_ms",        # 本窗口长度 ms
    "task_cnt",         # 窗口内控制任务次数
    "task_avg_us",      # 控制任务平均耗时 us（含中断/等待）
    "task_max_us",      # 控制任务最大耗时 us
    "imu_cnt",          # 窗口内 IMU 区间次数
    "imu_avg_us",       # IMU 读+中值滤波平均耗时 us
    "imu_max_us",       # IMU 读+中值滤波最大耗时 us
    "late_max_us",      # 最大启动迟到 us
    "int_min_us",       # 相邻两轮实际启动间隔最小 us
    "int_max_us",       # 相邻两轮实际启动间隔最大 us
    "miss_win",         # 窗口内截止违约次数（>5000us）
    "miss_total",       # 上电累计截止违约次数
    "drop_total",       # 统计帧发送丢弃累计次数
]


# ----------------------------------------------------------------------------
# 串口 / 模拟数据源
# ----------------------------------------------------------------------------
def list_ports():
    """打印本机串口，供 --list 使用"""
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("未发现任何串口设备")
        return
    print("可用串口：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
        if p.hwid:
            print(f"           {p.hwid}")


def pick_port(explicit_port):
    """
    未指定 --port 时自动挑选：
      优先 USB 转串口芯片（CH340/CP210/FT232/PL2303 等），
      否则仅当只有一个串口时直接使用，多于一个则报错让用户指定。
    """
    if explicit_port:
        return explicit_port

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("未发现串口，请检查 USB-TTL 是否插好，或用 --port 指定")

    keywords = ("CH340", "CH341", "CP210", "FT232", "FTDI", "PL2303",
                "USB-SERIAL", "USB SERIAL", "USB 串行", "Silicon Labs")
    for p in ports:
        text = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".upper()
        if any(k.upper() in text for k in keywords):
            print(f"自动选择串口：{p.device} ({p.description})")
            return p.device

    if len(ports) == 1:
        print(f"自动选择串口：{ports[0].device} ({ports[0].description})")
        return ports[0].device

    print("检测到多个串口，请用 --port 指定：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
    sys.exit(1)


class SerialSource:
    """真实串口数据源：只负责按块读出原始字节"""

    def __init__(self, port, baud):
        self.port = port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as exc:
            sys.exit(f"打开串口 {port} 失败：{exc}")

    def read(self):
        return self.ser.read(4096)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SimSource:
    """
    无硬件模拟源：按 100Hz 生成六轴帧（正弦 + 噪声 + 固定零偏），
    用于验证界面响应、退出流程和 CSV 完整性。
    """

    def __init__(self, hz=100.0, silent=False):
        self.dt = 1.0 / hz
        self.silent = silent          # True 时一帧都不发，用于测试"无数据"时的界面响应
        self.t0 = time.time()
        self.n = 0
        self.perf_n = 0
        self.t_ms0 = int(time.time() * 1000) & 0xFFFFFFFF

    def read(self):
        if self.silent:
            time.sleep(0.02)
            return b""
        now = time.time()
        target = int((now - self.t0) / self.dt)
        if target <= self.n:
            time.sleep(self.dt / 2)
            return b""
        out = []
        while self.n < target:
            t = self.n * self.dt
            out.append(build_frame(
                self.n % 256,
                (self.t_ms0 + int(t * 1000)) & 0xFFFFFFFF,
                int(0.05 * np.sin(2 * np.pi * 0.7 * t) * 2048 + np.random.normal(0, 6)),
                int(0.04 * np.sin(2 * np.pi * 0.9 * t + 1) * 2048 + np.random.normal(0, 6)),
                int((1.0 + 0.02 * np.sin(2 * np.pi * 0.3 * t)) * 2048 + np.random.normal(0, 6)),
                int((3.0 * np.sin(2 * np.pi * 0.5 * t) + 0.6) * 16.4 + np.random.normal(0, 2)),
                int((2.0 * np.sin(2 * np.pi * 0.35 * t + 2) - 0.4) * 16.4 + np.random.normal(0, 2)),
                int((1.0 * np.sin(2 * np.pi * 0.2 * t) + 0.2) * 16.4 + np.random.normal(0, 2)),
                12, -7, 3, FLAG_FILTERED))
            self.n += 1
            # 每 100 帧（1 秒）插一帧性能统计，模拟真实固件的 1Hz 统计报文
            if self.n % 100 == 0:
                out.append(self._perf_frame())
        return b"".join(out)

    def _perf_frame(self):
        """造一帧性能统计：数值随窗口轻微抖动，每 7 个窗口出现一次超期"""
        self.perf_n += 1
        miss = 1 if (self.perf_n % 7 == 0) else 0
        return build_perf_frame(
            seq=self.perf_n % 256,
            build_id=0x20261119,
            window_ms=1000,
            task_cnt=200,
            task_avg=int(1180 + np.random.normal(0, 25)),
            task_max=int(2100 + abs(np.random.normal(0, 320))),
            imu_cnt=200,
            imu_avg=int(620 + np.random.normal(0, 18)),
            imu_max=int(900 + abs(np.random.normal(0, 90))),
            late_max=int(abs(np.random.normal(0, 40))),
            int_min=int(4990 + np.random.normal(0, 8)),
            int_max=int(5060 + abs(np.random.normal(0, 30))),
            miss_win=miss,
            miss_total=self.perf_n // 7,
            drop_total=0)

    def close(self):
        pass


# ----------------------------------------------------------------------------
# 组帧解析
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# 台架固件文本协议（--attitude 模式，按当前 UserRxdFunc.c 的实际输出解析）
#   ATT init ok: T3488 16g/2000dps 800Hz, gyro_lpf_sel=3 (...), readback=3
#   ATT init fail st=N lpf_rb=M
#   ATT wait still 0.32/0.50s
#   ATT ready: r=+0.00 p=+0.00 (still 500ms)
#   ATT t=12345 rdy=1 q=32768/0/0/0                  ← 新：Q15 四元数 w/x/y/z
#   ATT t=12345 rdy=1 r=+12.34 p=-45.67 y=+120.15    ← 旧：姿态样本，字段已是度
#   STA n=1200 f=0 g=0 dt=4999/5001 rd=210/260 up=35/48
#   STA2 c0=12000/410/0 ... sta=210/260 diag=120 mrg=1800 lat=0
#                                                     ← 分项计时（四类拍 + STA 单项 + 诊断）
# ----------------------------------------------------------------------------
ATT_CSV_FIELDS = ["recv_s", "recv_epoch", "t_ms", "rdy",
                  "roll_deg", "pitch_deg", "yaw_deg"]
ATT_CAT_FIELDS = ["recv_s", "recv_epoch",
                  "c0_cnt", "c0_max_us", "c0_over",
                  "c1_cnt", "c1_max_us", "c1_over",
                  "c2_cnt", "c2_max_us", "c2_over",
                  "c3_cnt", "c3_max_us", "c3_over",
                  "sta_avg_us", "sta_max_us",
                  "diag_us", "margin_us", "lat_ms"]

ATT_STAT_FIELDS = ["recv_s", "recv_epoch", "valid", "fail", "gap",
                   "dt_min_us", "dt_max_us", "rd_avg_us", "rd_max_us",
                   "up_avg_us", "up_max_us",
                   # 400Hz 验收追加（旧固件的 STA 没有这些字段，留空）
                   "cyc_avg_us", "cyc_max_us", "miss", "skip", "euler_ok",
                   "z_pre_lsb", "z_post_lsb"]

_ATT_LINE_RE = re.compile(
    r"^ATT\s+t=(\d+)\s+rdy=(\d)\s+r=([+-]?\d+\.\d+)\s+p=([+-]?\d+\.\d+)"
    r"\s+y=([+-]?\d+\.\d+)\s*$")
# 基础字段（旧固件）+ 可选扩展字段（400Hz 验收：整拍耗时/超期/跳拍/更新次数）
_ATT_QUAT_RE = re.compile(
    r"^ATT\s+t=(\d+)\s+rdy=(\d)\s+q=(-?\d+)/(-?\d+)/(-?\d+)/(-?\d+)\s*$")

_STA_LINE_RE = re.compile(
    r"^STA\s+n=(\d+)\s+f=(\d+)\s+g=(\d+)\s+dt=(\d+)/(\d+)\s+"
    r"rd=(\d+)/(\d+)\s+up=(\d+)/(\d+)"
    r"(?:\s+cyc=(\d+)/(\d+)\s+miss=(\d+)\s+skip=(\d+)\s+eul=(\d+))?"
    r"(?:\s+zb=([+-]\d+\.\d+)/([+-]\d+\.\d+))?\s*$")


def quat_q15_to_euler_deg(qw, qx, qy, qz):
    """
    Q15 四元数（w/x/y/z，32768 = 1.0）→ 欧拉角（度）。
    约定与固件一致：右侧下沉横滚为正、抬头俯仰为正、俯视顺时针偏航为正。
    先归一化；全零（或过短）四元数返回 None，由调用方按无效帧丢弃。
    """
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-6:
        return None

    w, x, y, z = qw / n, qx / n, qy / n, qz / n

    # 机体 FRD → 水平系的旋转矩阵（ZYX 欧拉角）
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + w * z)
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)

    roll = math.degrees(math.atan2(r21, r22))
    # 数值误差可能让 |sin(pitch)| 略超 1，限幅后再做反正弦
    sin_p = max(-1.0, min(1.0, -r20))
    pitch = math.degrees(math.asin(sin_p))
    yaw = math.degrees(math.atan2(r10, r00))
    return roll, pitch, yaw


_STA2_LINE_RE = re.compile(
    r"^STA2"
    r"\s+c0=(\d+)/(\d+)/(\d+)\s+c1=(\d+)/(\d+)/(\d+)"
    r"\s+c2=(\d+)/(\d+)/(\d+)\s+c3=(\d+)/(\d+)/(\d+)"
    r"\s+sta=(\d+)/(\d+)"
    r"(?:\s+diag=(\d+)\s+mrg=(\d+)\s+lat=(\d+))?\s*$")


def attitude_state_of(line):
    """把一行 ATT 状态文字归类成界面显示的状态"""
    if line.startswith("ATT init fail"):
        return "初始化失败"
    if line.startswith("ATT init ok"):
        return "初始化完成"
    if line.startswith("ATT wait still"):
        return "等待静止"
    if line.startswith("ATT ready"):
        return "姿态就绪"
    return None


def attitude_detail_of(line):
    """从状态行里取一小段说明（状态栏空间有限，只留关键信息）"""
    if line.startswith("ATT init fail"):
        m = re.search(r"st=(\d+)\s+lpf_rb=(\d+)", line)
        return f"st={m.group(1)} 回读={m.group(2)}" if m else ""
    if line.startswith("ATT init ok"):
        m = re.search(r"readback=(\d+)", line)
        return f"低通回读={m.group(1)}" if m else ""
    if line.startswith("ATT wait still"):
        parts = line.split()
        return parts[3] if len(parts) > 3 else ""
    if line.startswith("ATT ready"):
        return line[len("ATT ready"):].strip(" :")
    return ""


def parse_attitude_line(line):
    """解析一行文本，返回 ("att", dict) / ("sta", dict) / ("status", 原文) / None"""
    m = _ATT_QUAT_RE.match(line)
    if m:
        angles = quat_q15_to_euler_deg(int(m.group(3)), int(m.group(4)),
                                       int(m.group(5)), int(m.group(6)))
        if angles is None:
            return None               # 全零四元数：无效帧
        return ("att", {
            "t_ms": int(m.group(1)),
            "rdy": int(m.group(2)),
            "roll": angles[0],
            "pitch": angles[1],
            "yaw": angles[2],
        })

    m = _ATT_LINE_RE.match(line)
    if m:
        return ("att", {
            "t_ms": int(m.group(1)),
            "rdy": int(m.group(2)),
            "roll": float(m.group(3)),
            "pitch": float(m.group(4)),
            "yaw": float(m.group(5)),
        })
    m = _STA_LINE_RE.match(line)
    if m:
        # 扩展字段是可选的：缺失时保持 None，界面/CSV 里不显示
        raw = m.groups()
        g = [int(x) if x is not None else None for x in raw[:14]]
        stat = {
            "valid": g[0], "fail": g[1], "gap": g[2],
            "dt_min_us": g[3], "dt_max_us": g[4],
            "rd_avg_us": g[5], "rd_max_us": g[6],
            "up_avg_us": g[7], "up_max_us": g[8],
            # 旧固件没有扩展字段：保持 None，界面上不显示
            "cyc_avg_us": None, "cyc_max_us": None,
            "miss": None, "skip": None, "euler_ok": None,
        }
        if g[9] is not None:
            stat.update({
                "cyc_avg_us": g[9], "cyc_max_us": g[10],
                "miss": g[11], "skip": g[12], "euler_ok": g[13],
            })
        # Z 轴扣偏前/后的窗口均值（单位 0.01 LSB），用于静置漂移核对
        if raw[14] is not None:          # 第 15 组：Z 轴扣偏前均值
            stat["z_pre_lsb"] = float(raw[14])
            stat["z_post_lsb"] = float(raw[15])
        else:
            stat["z_pre_lsb"] = None
            stat["z_post_lsb"] = None
        return ("sta", stat)
    m = _STA2_LINE_RE.match(line)
    if m:
        g = [int(x) if x is not None else None for x in m.groups()]
        return ("sta2", {
            "c0_cnt": g[0], "c0_max_us": g[1], "c0_over": g[2],
            "c1_cnt": g[3], "c1_max_us": g[4], "c1_over": g[5],
            "c2_cnt": g[6], "c2_max_us": g[7], "c2_over": g[8],
            "c3_cnt": g[9], "c3_max_us": g[10], "c3_over": g[11],
            "sta_avg_us": g[12], "sta_max_us": g[13],
            # 诊断：上一回 STA2 的生成耗时、发完余量、排队时长
            "diag_us": g[14], "margin_us": g[15], "lat_ms": g[16],
        })

    if attitude_state_of(line):
        return ("status", line)
    return None


def euler_deg_to_quat_q15(roll_deg, pitch_deg, yaw_deg):
    """欧拉角（度，ZYX 顺序）→ Q15 四元数 w/x/y/z，与固件约定一致"""
    hr = math.radians(roll_deg) * 0.5
    hp = math.radians(pitch_deg) * 0.5
    hy = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (int(round(w * 32768)), int(round(x * 32768)),
            int(round(y * 32768)), int(round(z * 32768)))


class SimAttitudeSource:
    """
    模拟台架固件的文本输出（无硬件自测）：
      init ok → wait still 递增 → ready → 14Hz ATT（缓慢变化）+ 1Hz STA
    读取时故意把行切碎，验证分块到达的粘包处理。
    """

    def __init__(self, hz=14.0):
        self.dt = 1.0 / hz
        self.t0 = time.time()
        self.buf = b""
        self.n_att = 0
        self.n_sta = 0
        self.sent_init = False
        self.sent_ready = False
        self.t_ready = None
        self._next_att = 0.0
        self._next_sta = 0.0

    def _push(self, text):
        self.buf += (text + "\n").encode("ascii", "replace")

    def _fill(self):
        now = time.time()
        el = now - self.t0
        if not self.sent_init:
            self.sent_init = True
            self._push("ATT init ok: T3488 16g/2000dps 800Hz, gyro_lpf_sel=3 "
                       "(ODR/16, ~50Hz), readback=3")
        if el < 0.5:
            if el >= self._next_att:
                self._next_att += 0.25
                self._push(f"ATT wait still {min(el, 0.5):.2f}/0.50s")
        elif not self.sent_ready:
            self.sent_ready = True
            self.t_ready = now
            self._push("ATT ready (still 500ms)")
        else:
            if now >= self._next_att:
                self._next_att = now + self.dt
                self.n_att += 1
                t_ms = int((now - self.t_ready) * 1000)
                # 缓慢变化的姿态：偏航慢慢漂，横滚/俯仰小幅摆动；
                # 与固件一样只发四元数，欧拉角由接收端换算
                yaw = 0.8 * (now - self.t_ready) % 360.0
                roll = 3.0 * math.sin(now * 0.7)
                pitch = -5.0 + 2.0 * math.cos(now * 0.5)
                qw, qx, qy, qz = euler_deg_to_quat_q15(roll, pitch, yaw)
                self._push(f"ATT t={t_ms} rdy=1 q={qw}/{qx}/{qy}/{qz}")
            if now >= self._next_sta:
                self._next_sta = now + 1.0
                self.n_sta += 1
                valid = self.n_att
                # 与 400Hz 固件的扩展 STA 一致（含整拍/超期/跳拍/更新次数）
                self._push(f"STA n={valid} f=0 g=0 dt=2499/2501 "
                           f"rd=110/130 up=30/42 cyc=620/780 miss=0 skip=0 eul={valid} "
                           f"zb=-1.32/+0.00")
                # 分项：普通拍/只 ATT/只 STA/两者，以及 STA 单项耗时
                self._push(f"STA2 c0={max(0, valid - 28)}/410/0 c1=28/520/0 "
                           f"c2=0/0/0 c3=0/0/0 sta=210/260 diag=120 mrg=1800 lat=0")

    def read(self):
        self._fill()
        if not self.buf:
            time.sleep(0.01)
            return b""
        # 故意只取一小段，甚至切断一行，模拟串口分块到达
        n = min(len(self.buf), 1 + (self.n_att * 7 + self.n_sta * 3) % 23)
        chunk, self.buf = self.buf[:n], self.buf[n:]
        return chunk

    def close(self):
        pass


class AttitudeWorker(threading.Thread):
    """台架文本采集线程：拆行 → 解析 ATT/STA → 写两份 CSV → 姿态样本入队"""

    def __init__(self, source, csv_path, stat_path, sample_queue, cat_path=None):
        super().__init__(daemon=True)
        self.source = source
        self.csv_path = csv_path
        self.stat_path = stat_path
        self.cat_path = cat_path
        self.queue = sample_queue
        self.stop_event = threading.Event()

        self.frames = 0          # 姿态样本条数
        self.perf_frames = 0     # STA 条数
        self.bad = 0             # 无法识别的行数
        self.state = "等待数据"  # 初始化完成 / 等待静止 / 姿态就绪 / 初始化失败
        self.detail = ""         # 状态的简短说明
        self.last_stat = None    # 最近一条 STA
        self.last_cat = None     # 最近一条 STA2（分项计时）
        self.error = ""
        self.start_wall = time.time()
        self._buf = b""

    def stop(self):
        self.stop_event.set()

    def summary_line(self):
        return (f"姿态 {self.frames} 条  统计 {self.perf_frames} 条  "
                f"无法识别 {self.bad} 行  状态 {self.state}")

    def _handle_line(self, line, writer, stat_writer, cat_writer=None):
        parsed = parse_attitude_line(line)
        if parsed is None:
            self.bad += 1
            return

        kind, payload = parsed
        now = time.time()

        if kind == "att":
            self.frames += 1
            writer.writerow({
                "recv_s": f"{now - self.start_wall:.3f}",
                "recv_epoch": f"{now:.3f}",
                "t_ms": payload["t_ms"],
                "rdy": payload["rdy"],
                "roll_deg": f"{payload['roll']:.2f}",
                "pitch_deg": f"{payload['pitch']:.2f}",
                "yaw_deg": f"{payload['yaw']:.2f}",
            })
            try:
                self.queue.put_nowait(payload)
            except queue.Full:
                pass
            return

        if kind == "sta":
            self.perf_frames += 1
            self.last_stat = payload
            stat_writer.writerow({
                "recv_s": f"{now - self.start_wall:.3f}",
                "recv_epoch": f"{now:.3f}",
                "valid": payload["valid"],
                "fail": payload["fail"],
                "gap": payload["gap"],
                "dt_min_us": payload["dt_min_us"],
                "dt_max_us": payload["dt_max_us"],
                "rd_avg_us": payload["rd_avg_us"],
                "rd_max_us": payload["rd_max_us"],
                "up_avg_us": payload["up_avg_us"],
                "up_max_us": payload["up_max_us"],
                "cyc_avg_us": "" if payload["cyc_avg_us"] is None else payload["cyc_avg_us"],
                "cyc_max_us": "" if payload["cyc_max_us"] is None else payload["cyc_max_us"],
                "miss": "" if payload["miss"] is None else payload["miss"],
                "skip": "" if payload["skip"] is None else payload["skip"],
                "euler_ok": "" if payload["euler_ok"] is None else payload["euler_ok"],
                "z_pre_lsb": "" if payload.get("z_pre_lsb") is None else payload["z_pre_lsb"],
                "z_post_lsb": "" if payload.get("z_post_lsb") is None else payload["z_post_lsb"],
            })
            line = (f"[统计] 有效{payload['valid']} 读失败{payload['fail']} "
                    f"长间断{payload['gap']} dt {payload['dt_min_us']}~{payload['dt_max_us']}us "
                    f"rd {payload['rd_avg_us']}/{payload['rd_max_us']}us "
                    f"up {payload['up_avg_us']}/{payload['up_max_us']}us")
            if payload["cyc_max_us"] is not None:
                line += (f" 整拍 {payload['cyc_avg_us']}/{payload['cyc_max_us']}us "
                         f"超期{payload['miss']} 跳拍{payload['skip']} "
                         f"姿态更新{payload['euler_ok']}")
            if payload.get("z_post_lsb") is not None:
                line += (f" Z轴偏前{payload['z_pre_lsb']:+.2f} 偏后{payload['z_post_lsb']:+.2f}LSB")
            print(line, flush=True)
            return

        if kind == "sta2":
            self.last_cat = payload
            if cat_writer is not None:
                row = {"recv_s": f"{now - self.start_wall:.3f}",
                       "recv_epoch": f"{now:.3f}"}
                for key in ("diag_us", "margin_us", "lat_ms"):
                    if payload.get(key) is None:
                        payload[key] = ""
                row.update(payload)
                cat_writer.writerow(row)
            print(f"[分项] 普通 {payload['c0_cnt']}/{payload['c0_max_us']}us/超{payload['c0_over']}  "
                  f"只ATT {payload['c1_cnt']}/{payload['c1_max_us']}us/超{payload['c1_over']}  "
                  f"只STA {payload['c2_cnt']}/{payload['c2_max_us']}us/超{payload['c2_over']}  "
                  f"ATT+STA {payload['c3_cnt']}/{payload['c3_max_us']}us/超{payload['c3_over']}  "
                  f"STA单项 {payload['sta_avg_us']}/{payload['sta_max_us']}us"
                  + (f"  诊断 耗时{payload['diag_us']}us 余量{payload['margin_us']}us"
                     f" 排队{payload['lat_ms']}ms"
                     if payload.get("diag_us") not in (None, "") else ""), flush=True)
            return

        # 状态行
        self.state = attitude_state_of(line) or self.state
        self.detail = attitude_detail_of(line)
        print(f"[台架] {line}", flush=True)

    def run(self):
        cat_fp = None
        try:
            cat_fp = open(self.cat_path, "w", newline="", encoding="utf-8-sig") \
                if self.cat_path else None
            with open(self.csv_path, "w", newline="", encoding="utf-8-sig") as fp, \
                    open(self.stat_path, "w", newline="", encoding="utf-8-sig") as sfp:
                writer = csv.DictWriter(fp, fieldnames=ATT_CSV_FIELDS)
                writer.writeheader()
                stat_writer = csv.DictWriter(sfp, fieldnames=ATT_STAT_FIELDS)
                stat_writer.writeheader()
                cat_writer = None
                if cat_fp is not None:
                    cat_writer = csv.DictWriter(cat_fp, fieldnames=ATT_CAT_FIELDS)
                    cat_writer.writeheader()

                while not self.stop_event.is_set():
                    data = self.source.read()
                    if not data:
                        continue
                    self._buf += data
                    # 一行一行处理：串口分块到达时，残行留在缓冲里等下一块
                    while b"\n" in self._buf:
                        raw, self._buf = self._buf.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").strip()
                        if line:
                            self._handle_line(line, writer, stat_writer, cat_writer)
                    if len(self._buf) > 512:
                        # 迟迟没有换行：多半是波特率/接线不对，丢掉避免无限增长
                        self.bad += 1
                        self._buf = b""
        except Exception as exc:          # 串口异常：记录下来给界面/控制台
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            try:
                if cat_fp is not None:
                    cat_fp.close()
            except Exception:
                pass
            try:
                self.source.close()
            except Exception:
                pass


def _nice_bounds(lo, hi):
    """把区间上下界取整到 1/2/2.5/5×10ⁿ 的"好看刻度"上"""
    span = max(hi - lo, 1e-9)
    base = 10.0 ** math.floor(math.log10(span / 4.0))
    step = 10.0 * base
    for m in (1.0, 2.0, 2.5, 5.0, 10.0):
        if span / 4.0 <= m * base:
            step = m * base
            break
    return math.floor(lo / step) * step, math.ceil(hi / step) * step


def flags_text(flags):
    """把 flags 翻译成人能看懂的说明"""
    items = ["已滤波" if flags & FLAG_FILTERED else "未滤波",
             "已扣零偏" if flags & FLAG_BIAS_APPLIED else "未扣零偏",
             "零偏已标定" if flags & FLAG_CALIBRATED else "零偏未标定"]
    if flags & FLAG_ARMED:
        items.append("已解锁")
    return " / ".join(items)


def frame_to_row(fr, acc_scale, gyro_scale):
    """帧 → CSV 行（同时给出原始 LSB 与换算后的物理量）"""
    ax = fr["ax"] / acc_scale
    ay = fr["ay"] / acc_scale
    az = fr["az"] / acc_scale
    gx = fr["gx"] / gyro_scale
    gy = fr["gy"] / gyro_scale
    gz = fr["gz"] / gyro_scale
    return {
        "t_ms": fr["t_ms"], "seq": fr["seq"],
        "ax_lsb": fr["ax"], "ay_lsb": fr["ay"], "az_lsb": fr["az"],
        "gx_lsb": fr["gx"], "gy_lsb": fr["gy"], "gz_lsb": fr["gz"],
        "bx_lsb": fr["bx"], "by_lsb": fr["by"], "bz_lsb": fr["bz"],
        "ax_g": f"{ax:.4f}", "ay_g": f"{ay:.4f}", "az_g": f"{az:.4f}",
        "gx_dps": f"{gx:.3f}", "gy_dps": f"{gy:.3f}", "gz_dps": f"{gz:.3f}",
        "gx_c_dps": f"{(fr['gx'] - fr['bx']) / gyro_scale:.3f}",
        "gy_c_dps": f"{(fr['gy'] - fr['by']) / gyro_scale:.3f}",
        "gz_c_dps": f"{(fr['gz'] - fr['bz']) / gyro_scale:.3f}",
        "flags": fr["flags"],
    }


# ----------------------------------------------------------------------------
# 采集线程：串口 → 解析 → CSV → 队列（界面只从队列取）
# ----------------------------------------------------------------------------
class SensorWorker(threading.Thread):
    def __init__(self, source, csv_path, perf_path, frame_queue, acc_scale, gyro_scale):
        super().__init__(daemon=True)
        self.source = source
        self.csv_path = csv_path
        self.perf_path = perf_path
        self.queue = frame_queue
        self.acc_scale = acc_scale
        self.gyro_scale = gyro_scale
        self.parser = FrameParser()
        self.stop_event = threading.Event()

        # 供界面读取的统计量（整数赋值是原子的，不做锁）
        self.frames = 0
        self.bad = 0
        self.lost = 0
        self.perf_frames = 0     # 收到的性能统计帧数
        self.perf_lost = 0       # 性能帧序号跳变累计
        self.last_perf = None    # 最近一帧性能统计（界面显示用）
        self.error = ""
        self.start_wall = time.time()
        self._last_seq = None
        self._last_perf_seq = None

    def stop(self):
        self.stop_event.set()

    def _on_perf(self, fr, pwriter):
        """性能统计帧：写独立 CSV + 控制台打印一行（1Hz）"""
        self.perf_frames += 1
        self.last_perf = fr
        if self._last_perf_seq is not None:
            step = (fr["seq"] - self._last_perf_seq) & 0xFF
            if step != 1:
                self.perf_lost += (step - 1) & 0xFF
        self._last_perf_seq = fr["seq"]

        now = time.time()
        pwriter.writerow({
            "recv_s": f"{now - self.start_wall:.3f}",
            "recv_epoch": f"{now:.3f}",
            "seq": fr["seq"],
            "build_id": f"0x{fr['build_id']:08X}",
            "window_ms": fr["window_ms"],
            "task_cnt": fr["task_cnt"],
            "task_avg_us": fr["task_avg_us"],
            "task_max_us": fr["task_max_us"],
            "imu_cnt": fr["imu_cnt"],
            "imu_avg_us": fr["imu_avg_us"],
            "imu_max_us": fr["imu_max_us"],
            "late_max_us": fr["late_max_us"],
            "int_min_us": fr["int_min_us"],
            "int_max_us": fr["int_max_us"],
            "miss_win": fr["miss_win"],
            "miss_total": fr["miss_total"],
            "drop_total": fr["drop_total"],
        })
        print(f"[性能] seq={fr['seq']:3d} 窗口{fr['window_ms']}ms "
              f"任务 平均{fr['task_avg_us']}/最大{fr['task_max_us']}us "
              f"IMU 平均{fr['imu_avg_us']}/最大{fr['imu_max_us']}us "
              f"迟到{fr['late_max_us']}us 间隔{fr['int_min_us']}~{fr['int_max_us']}us "
              f"超期{fr['miss_win']}(累计{fr['miss_total']}) 丢弃{fr['drop_total']}",
              flush=True)

    def summary_line(self):
        """无界面模式下的进度摘要（两种采集线程都有这个方法）"""
        return (f"六轴 {self.frames} 帧  性能 {self.perf_frames} 帧  "
                f"校验错 {self.bad} 丢帧≈{self.lost}")

    def run(self):
        try:
            with open(self.csv_path, "w", newline="", encoding="utf-8-sig") as fp, \
                    open(self.perf_path, "w", newline="", encoding="utf-8-sig") as pfp:
                writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
                writer.writeheader()
                pwriter = csv.DictWriter(pfp, fieldnames=PERF_CSV_FIELDS)
                pwriter.writeheader()
                while not self.stop_event.is_set():
                    data = self.source.read()
                    if not data:
                        continue
                    for kind, fr in self.parser.feed(data):
                        if kind == "perf":
                            self._on_perf(fr, pwriter)
                            pfp.flush()          # 1Hz 一帧，直接落盘
                            continue

                        self.frames += 1
                        self.bad = self.parser.bad
                        # 序号连续性统计（只提示，不丢数据）
                        if self._last_seq is not None:
                            step = (fr["seq"] - self._last_seq) & 0xFF
                            if step != 1:
                                self.lost += (step - 1) & 0xFF
                        self._last_seq = fr["seq"]

                        writer.writerow(frame_to_row(fr, self.acc_scale, self.gyro_scale))

                        # 队列满时丢最旧的一帧：只影响绘图，CSV 已经落盘
                        while True:
                            try:
                                self.queue.put_nowait(fr)
                                break
                            except queue.Full:
                                try:
                                    self.queue.get_nowait()
                                except queue.Empty:
                                    pass
                        # 约每 200 帧（2 秒）落一次盘，异常退出时最多丢 2 秒
                        if self.frames and self.frames % 200 == 0:
                            fp.flush()
        except Exception as exc:                  # 串口掉线等：记录后让界面提示
            self.error = str(exc)
        finally:
            self.bad = self.parser.bad
            try:
                self.source.close()
            except Exception:
                pass


# ----------------------------------------------------------------------------
# 界面：主线程运行，只做"取队列 + 画图"
# ----------------------------------------------------------------------------
class SensorMonitor:
    def __init__(self, worker, csv_path, window=10.0, duration=0, mode="sensor"):
        self.worker = worker
        self.csv_path = csv_path
        self.window = window
        self.duration = duration
        self.mode = mode          # "sensor"=六轴二进制帧；"attitude"=台架文本

        self.follow = True          # 是否自动跟随最新数据
        self.fig = None
        self.axes = []
        self.lines = []
        self.text = None
        self.timer = None
        self.toolbar = None
        self._ticks = 0             # 定时器回调次数（自测用）
        self._last_text = ""        # 状态栏缓存，文字没变就不重画
        self.t0 = 0                 # 首帧的设备毫秒（时间轴零点）
        self._bg = None             # blit 用的背景位图（整图）
        self._ax_bgs = None         # 每个子图的背景位图
        self._text_ts = 0.0         # 状态栏上次刷新的时间
        self._dirty = False         # 有新数据待绘制

        # 绘图用的环形缓冲（按窗口时长和 100Hz 估算容量）
        self.cap = int(max(200, window * 150))
        self.hist_t = deque(maxlen=self.cap)
        self.hist = [deque(maxlen=self.cap) for _ in range(3)]   # 加速度 g
        self.hist_gyro = [deque(maxlen=self.cap) for _ in range(3)]  # 陀螺原始 dps
        self.hist_gyro_c = [deque(maxlen=self.cap) for _ in range(3)]  # 陀螺扣零偏 dps
        self.bias = (0, 0, 0)
        self.flags = 0

    # ---------------- 界面搭建 ----------------
    def build(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        self.plt = plt

        # 中文字体：雅黑/黑体，避免坐标轴和状态栏出现方块
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei",
                                           "Noto Sans CJK SC", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False

        colors = ["#d62728", "#2ca02c", "#1f77b4"]
        if self.mode == "attitude":
            # 姿态模式：一张图 + 三条角度曲线（单位：度）
            self.fig, ax_obj = plt.subplots(1, 1, figsize=(11, 5))
            self.fig.canvas.manager.set_window_title(
                "Robin_24G_Car 姿态监视器（台架固件文本）")
            self.axes = [ax_obj]
            ax_obj.grid(True, alpha=0.3)
            ax_obj.set_ylabel("角度 (°)")
            lines = [ax_obj.plot([], [], color=c, lw=1.0, label=l)[0]
                     for c, l in zip(colors, ("横滚", "俯仰", "相对偏航"))]
            for line in lines:
                line.set_animated(True)
            self.lines.append(lines)
        else:
            self.fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
            self.fig.canvas.manager.set_window_title("Robin_24G_Car 六轴传感器监视器")
            self.axes = list(axes)

            titles = ["加速度 (g)", "陀螺原始值 (dps)", "陀螺扣零偏 (dps)"]
            labels = [("ax", "ay", "az"), ("gx", "gy", "gz"), ("gx-c", "gy-c", "gz-c")]
            for ax_obj, title, lab in zip(self.axes, titles, labels):
                ax_obj.grid(True, alpha=0.3)
                ax_obj.set_ylabel(title)
                lines = [ax_obj.plot([], [], color=c, lw=0.9, label=l)[0]
                         for c, l in zip(colors, lab)]
                # 曲线设为 animated：全量重画时不画曲线，背景位图才干净，
                # 之后每次 blit 只把曲线补上去，不会留残影
                for line in lines:
                    line.set_animated(True)
                self.lines.append(lines)
        self.axes[-1].set_xlabel("时间 (s，0 = 当前)")
        for ax_obj in self.axes:
            # 图例放到坐标框上方（框外）：曲线被裁剪在框内，永远不会压到图例，
            # 这样 blit 时不必每帧重画图例，也不会出现"曲线盖图例"的残影
            ax_obj.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0),
                          ncol=3, fontsize=8, frameon=False, borderaxespad=0.2)
            # x 轴固定 [-window, 0]：数据整体左移实现滚动，坐标轴刻度不动，
            # 这样平时只需重画曲线（blit），拖动/缩放才走全量重画
            ax_obj.set_xlim(-self.window, 0)

        # 状态栏用默认无衬线中文字体（不要用 monospace，否则中文是方块）
        self.text = self.fig.text(0.01, 0.005, "等待数据…", fontsize=9)
        self.text.set_animated(True)      # 同曲线，避免文字变短时留残影
        self.fig.subplots_adjust(left=0.07, right=0.99, top=0.95, bottom=0.08)

        self.toolbar = self.fig.canvas.toolbar
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_mouse)
        self.fig.canvas.mpl_connect("scroll_event", self._on_mouse)
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)

        # 定时器驱动：即使一帧数据都没有，Tk 事件循环也照常响应
        self.timer = self.fig.canvas.new_timer(interval=PLOT_PERIOD_MS)
        self.timer.add_callback(self._on_timer)
        self.timer.start()

        if self.duration > 0:
            self.fig.canvas.get_tk_widget().after(
                int(self.duration * 1000), lambda: self.plt.close(self.fig))

    # ---------------- 事件 ----------------
    def _on_close(self, _event):
        """关窗即停：先停定时器，再通知采集线程收尾"""
        self.timer.stop()
        self.worker.stop()

    def _on_key(self, event):
        if event.key in ("q", "Q"):
            self.plt.close(self.fig)
        elif event.key in ("f", "F"):
            self.follow = not self.follow
            self._dirty = True              # 恢复跟随时立即重画
            self._refresh()

    def _on_mouse(self, _event):
        """用工具栏平移/缩放鼠标滚轮时，自动退出"跟随"以免和用户抢视野"""
        mode = getattr(self.toolbar, "mode", "")
        if mode or (_event.name == "scroll_event"):
            self.follow = False

    def _busy(self):
        """工具栏处于平移/缩放状态时只收数据不重画，保证拖动顺滑"""
        return bool(getattr(self.toolbar, "mode", ""))

    # ---------------- 周期刷新 ----------------
    def _on_timer(self):
        self._ticks += 1
        self._drain()
        if self._busy():
            return                      # 用户正在拖拽/缩放：跳过本次重画
        self._refresh()

    def _drain(self):
        """把队列里的帧全部取出（100Hz 数据、10Hz 刷新 → 每次约 10 帧）"""
        while True:
            try:
                fr = self.worker.queue.get_nowait()
            except queue.Empty:
                break
            self._append(fr)

    def _append(self, fr):
        # 关掉"跟随"（用户正在缩放/平移查看）时不追加新点，画面冻结便于观察
        if not self.follow:
            return
        if not self.hist_t:
            self.t0 = fr["t_ms"]
        t = (fr["t_ms"] - self.t0) / 1000.0
        self.hist_t.append(t)
        if self.mode == "attitude":
            # 文本协议里的 r/p/y 已经是度，直接进三条曲线
            for arr, val in zip(self.hist, (fr["roll"], fr["pitch"], fr["yaw"])):
                arr.append(val)
            self._dirty = True
            return
        acc = (fr["ax"] / self.worker.acc_scale,
               fr["ay"] / self.worker.acc_scale,
               fr["az"] / self.worker.acc_scale)
        gyr = (fr["gx"] / self.worker.gyro_scale,
               fr["gy"] / self.worker.gyro_scale,
               fr["gz"] / self.worker.gyro_scale)
        gyr_c = ((fr["gx"] - fr["bx"]) / self.worker.gyro_scale,
                 (fr["gy"] - fr["by"]) / self.worker.gyro_scale,
                 (fr["gz"] - fr["bz"]) / self.worker.gyro_scale)
        for arr, val in zip(self.hist, acc):
            arr.append(val)
        for arr, val in zip(self.hist_gyro, gyr):
            arr.append(val)
        for arr, val in zip(self.hist_gyro_c, gyr_c):
            arr.append(val)
        self.bias = (fr["bx"], fr["by"], fr["bz"])
        self.flags = fr["flags"]
        self._dirty = True

    def _series_pairs(self):
        """返回 [(数据序列组, 曲线组)]：姿态模式只有一组（三条角度曲线）"""
        if self.mode == "attitude":
            return [(self.hist, self.lines[0])]
        return [(self.hist, self.lines[0]),
                (self.hist_gyro, self.lines[1]),
                (self.hist_gyro_c, self.lines[2])]

    def _status(self):
        if self.mode == "attitude":
            return self._status_attitude()
        if not self.hist_t:
            return (f"等待数据…（六轴 {self.worker.frames} 帧 / 性能 {self.worker.perf_frames} 帧，"
                    f"校验错 {self.worker.bad}）")
        span = self.hist_t[-1] - self.hist_t[max(0, len(self.hist_t) - 101)]
        rate = 100.0 / span if span > 0.01 else 0.0
        state = "跟随开" if self.follow else "跟随关"
        msg = (f"{state} | 帧 {self.worker.frames} 速率≈{rate:.1f}Hz "
               f"校验错 {self.worker.bad} 丢帧≈{self.worker.lost} | "
               f"零偏({self.bias[0]},{self.bias[1]},{self.bias[2]}) "
               f"{flags_text(self.flags)}")
        # 性能统计（每秒一帧，取最近一帧显示）
        perf = self.worker.last_perf
        if perf:
            msg += (f" | 任务 {perf['task_avg_us']}/{perf['task_max_us']}us "
                    f"IMU {perf['imu_avg_us']}/{perf['imu_max_us']}us "
                    f"迟到{perf['late_max_us']}us 超期{perf['miss_total']}")
        msg += " | F 跟随 / Q 退出"
        if self.worker.error:
            msg += f" | 串口错误：{self.worker.error}"
        return msg

    def _status_attitude(self):
        """姿态模式状态栏：固件状态 + 最新角度 + 最新 STA 统计"""
        w = self.worker
        msg = f"{w.state}"
        if w.detail:
            msg += f" {w.detail}"
        msg += f" | 样本 {w.frames} 条"
        if self.hist_t:
            msg += (f" | 最新 横滚 {self.hist[0][-1]:+.2f}° 俯仰 {self.hist[1][-1]:+.2f}°"
                    f" 相对偏航 {self.hist[2][-1]:+.2f}°")
        else:
            msg = "等待姿态数据…（先上电、静置） | " + msg
        s = w.last_stat
        if s:
            msg += (f" | 有效 {s['valid']} 读失败 {s['fail']} 长间断 {s['gap']}"
                    f" dt {s['dt_min_us']}~{s['dt_max_us']}us"
                    f" rd {s['rd_avg_us']}/{s['rd_max_us']}us"
                    f" up {s['up_avg_us']}/{s['up_max_us']}us")
            if s["cyc_max_us"] is not None:
                msg += (f" 整拍 {s['cyc_avg_us']}/{s['cyc_max_us']}us"
                        f" 超期 {s['miss']} 跳拍 {s['skip']} 更新 {s['euler_ok']}")
            if s.get("z_post_lsb") is not None:
                msg += f" Z偏前{s['z_pre_lsb']:+.2f}/偏后{s['z_post_lsb']:+.2f}"
        c = w.last_cat
        if c:
            msg += (f" | 分项max {c['c0_max_us']}/{c['c1_max_us']}"
                    f"/{c['c2_max_us']}/{c['c3_max_us']}us")
        else:
            msg += " | 等待统计…"
        msg += " | F 跟随 / Q 退出"
        if w.error:
            msg += f" | 串口错误：{w.error}"
        return msg

    def _refresh(self):
        if not self.hist_t:
            # 没有数据时只更新一次状态栏，避免空转重画拖慢窗口
            msg = self._status()
            if msg != self._last_text:
                self._last_text = msg
                self.text.set_text(msg)
                self._blit_dynamic(True)
            return

        # 没有新数据（例如已冻结）且状态栏没变：什么都不用画，事件循环保持空闲
        if not self._dirty:
            msg = self._status()
            if msg == self._last_text:
                return
            now = time.time()
            if now - self._text_ts < 1.0:   # 状态栏也限到 1Hz，冻结时不影响操作
                return
            self._text_ts = now
            self._last_text = msg
            self.text.set_text(msg)
            self._blit_dynamic(True)
            return
        self._dirty = False

        # 只保留窗口内的点，超出即从左侧丢弃
        t_end = self.hist_t[-1]
        t_min = t_end - self.window
        while self.hist_t and self.hist_t[0] < t_min:
            self.hist_t.popleft()
            for series, _lines in self._series_pairs():
                for arr in series:
                    arr.popleft()

        # x = 相对当前时刻（最新点在 0），配合固定 xlim 实现"数据左移"的滚动
        xs = np.fromiter((t - t_end for t in self.hist_t),
                         dtype=float, count=len(self.hist_t))
        for series, line_set in self._series_pairs():
            for arr, line in zip(series, line_set):
                line.set_data(xs, np.fromiter(arr, dtype=float, count=len(arr)))

        # 只有量程真的变了才重画对应子图；否则 blit，10Hz 刷新几乎不占事件循环
        changed_axes = self._auto_ylim() if self.follow else []
        self._last_text = self._status()
        self.text.set_text(self._last_text)

        now = time.time()
        if self._bg is None or changed_axes:
            # 量程/布局变化：整图静态重绘 + 重新缓存干净背景 + 补画动态内容
            self._redraw_static()
        elif now - self._text_ts >= 1.0:
            # 状态栏约 1Hz 刷新一次（draw_text 一次要 6~12ms），必须整图恢复背景，
            # 这样文字变短时旧字也会被清掉
            self._text_ts = now
            self._blit_dynamic(True)
        else:
            self._blit_lines()

    def _auto_ylim(self):
        """
        自动量程：按当前窗口数据取整到"好看的刻度"（1/2/2.5/5×10ⁿ），
        只有整档变化时才改；返回需要重画的子图下标列表。
        """
        changed = []
        for i, (ax_obj, (series, _lines)) in enumerate(
                zip(self.axes, self._series_pairs())):
            lo = min(min(a) for a in series)
            hi = max(max(a) for a in series)
            if hi - lo < 1e-6:
                hi = lo + 1.0
            margin = (hi - lo) * 0.15
            lo_n, hi_n = _nice_bounds(lo - margin, hi + margin)
            cur_lo, cur_hi = ax_obj.get_ylim()
            if abs(lo_n - cur_lo) > 1e-9 or abs(hi_n - cur_hi) > 1e-9:
                ax_obj.set_ylim(lo_n, hi_n)
                changed.append(i)
        return changed

    # ---------------- 绘图（blit）----------------
    def _ax_bbox(self, ax_obj):
        """
        子图区域外扩 2 像素：既覆盖坐标轴/刻度/图例，也能盖住曲线在
        坐标框边界上的抗锯齿像素，避免边缘留残影。
        """
        from matplotlib.transforms import Bbox
        b = ax_obj.bbox
        return Bbox.from_extents(b.x0 - 2, b.y0 - 2, b.x1 + 2, b.y1 + 2)

    def _on_draw(self, _event):
        """
        整图重画（首次 / 量程变化 / 窗口缩放 / 布局变化）后缓存背景。
        曲线和状态栏都是 animated，正常重画不会画进画布，所以这里拿到的是
        **不含动态内容的干净背景**。
        注意：绝不能在补画曲线之后再缓存背景，否则下一帧恢复出来的背景
        会带上旧曲线，形成重影。
        """
        try:
            self._bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            self._ax_bgs = [self.fig.canvas.copy_from_bbox(self._ax_bbox(ax))
                            for ax in self.axes]
        except Exception:
            self._bg = None
            self._ax_bgs = None
        self._text_ts = 0.0     # 重画后状态栏是空的，下一次刷新要补上
        self._dirty = True      # 重画后曲线也不在画面上，逼下一次刷新补画

    def _draw_dynamic_artists(self, ax_obj, line_set):
        """补画 9 条曲线（图例在坐标框外，曲线不会压到它，无需重画）"""
        for line in line_set:
            ax_obj.draw_artist(line)

    def _blit_lines(self):
        """只补画 9 条曲线：恢复各子图背景 → 画曲线 → blit 子图区域（不碰状态栏）"""
        if self._ax_bgs is None:
            self._redraw_static()
            return
        try:
            for ax_obj, line_set, bg_ax in zip(self.axes, self.lines, self._ax_bgs):
                self.fig.canvas.restore_region(bg_ax)
                self._draw_dynamic_artists(ax_obj, line_set)
                self.fig.canvas.blit(self._ax_bbox(ax_obj))
        except Exception:
            # 个别后端不支持 blit：退化为整图重画
            self._bg = None
            self._ax_bgs = None
            self.fig.canvas.draw()

    def _blit_dynamic(self, show_text=True):
        """恢复干净背景 → 补画曲线 → 补画状态栏 → 整图 blit"""
        if self._bg is None:
            self._redraw_static()
            return
        try:
            self.fig.canvas.restore_region(self._bg)
            for ax_obj, line_set in zip(self.axes, self.lines):
                self._draw_dynamic_artists(ax_obj, line_set)
            if show_text:
                self.text.draw(self.fig.canvas.get_renderer())
            self.fig.canvas.blit(self.fig.bbox)
        except Exception:
            self._bg = None
            self.fig.canvas.draw()

    def _redraw_static(self):
        """
        量程 / 窗口尺寸 / 布局变化后的完整重绘：
          ① 整图静态重画（曲线与状态栏是 animated，不参与，画面只剩坐标轴）；
          ② draw_event 里重新缓存干净背景；
          ③ 立即补画曲线与状态栏，避免闪一下空白。
        整图重画会把坐标框外的旧刻度、旧轴标签一起清掉，不会留残影。
        """
        self.fig.canvas.draw()
        if self._bg is not None:
            self._blit_dynamic(True)
        self._text_ts = time.time()

    # ---------------- 运行 ----------------
    def show(self):
        """进入 Tk 事件循环，直到窗口关闭（调用前需已 build）"""
        try:
            self.plt.show()
        finally:
            try:
                self.timer.stop()
            except Exception:
                pass
            self.worker.stop()
            self.worker.join(timeout=2.0)

    def run(self):
        self.build()
        self.show()


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def make_csv_path(explicit):
    """默认文件名：uart_sensor_年月日_时分秒.csv"""
    if explicit:
        return explicit
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"uart_sensor_{stamp}.csv"


def make_perf_csv_path(explicit, data_csv):
    """
    性能统计 CSV 路径：
      --stats-csv 指定则用它；否则跟数据 CSV 同目录同名，加 _perf 后缀。
    """
    if explicit:
        return explicit
    base, _ = os.path.splitext(data_csv)
    return f"{base}_perf.csv"


def run_no_plot(worker, duration):
    """无界面模式：只采集 + 写 CSV"""
    worker.start()
    start = time.time()
    last = 0.0
    try:
        while worker.is_alive():
            time.sleep(0.2)
            now = time.time()
            if now - last > 2.0:
                last = now
                print(worker.summary_line(), flush=True)
            if duration and (now - start) >= duration:
                break
            if worker.error:
                print(f"串口错误：{worker.error}")
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        worker.stop()
        worker.join(timeout=2.0)


def make_attitude_csv_path(explicit):
    """姿态模式默认文件名：attitude_年月日_时分秒.csv"""
    if explicit:
        return explicit
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"attitude_{stamp}.csv"


def make_att_stat_path(explicit, data_csv):
    """姿态统计 CSV：--stats-csv 指定则用它，否则数据 CSV 名后加 _stats"""
    if explicit:
        return explicit
    base, _ = os.path.splitext(data_csv)
    return f"{base}_stats.csv"


def run_attitude(args):
    """--attitude：按台架固件的文本协议（ATT/STA）采集与显示"""
    if args.simulate:
        source = SimAttitudeSource()
        port_name = "模拟台架数据源"
    else:
        port_name = pick_port(args.port)
        source = SerialSource(port_name, args.baud)

    csv_path = make_attitude_csv_path(args.csv)
    stat_path = make_att_stat_path(args.stats_csv, csv_path)
    cat_path = f"{os.path.splitext(csv_path)[0]}_cat.csv"
    sample_queue = queue.Queue(maxsize=2000)
    worker = AttitudeWorker(source, csv_path, stat_path, sample_queue, cat_path)

    print(f"数据源：{port_name}（姿态模式，文本协议）")
    print(f"姿态 CSV：{csv_path}")
    print(f"统计 CSV：{stat_path}")
    print(f"分项 CSV：{cat_path}（四类拍的拍数/最大耗时/超期次数）")
    print("提示：先看 ATT init ok / ATT ready；ATT wait still 表示还在等飞机静止。")

    if args.no_plot:
        run_no_plot(worker, args.duration)
        print(f"姿态 CSV：{csv_path}（{worker.frames} 条 ATT）")
        print(f"统计 CSV：{stat_path}（{worker.perf_frames} 条 STA，"
              f"无法识别 {worker.bad} 行）")
        print(f"分项 CSV：{cat_path}")
        return

    worker.start()
    monitor = SensorMonitor(worker, csv_path, args.window, args.duration,
                            mode="attitude")
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        worker.stop()
        worker.join(timeout=2.0)
    print(f"姿态 CSV：{csv_path}（{worker.frames} 条 ATT）")
    print(f"统计 CSV：{stat_path}（{worker.perf_frames} 条 STA，"
          f"无法识别 {worker.bad} 行）")
    print(f"分项 CSV：{cat_path}")


def run(args):
    if args.attitude:
        run_attitude(args)
        return
    if args.simulate:
        source = SimSource()
        port_name = "模拟数据源"
    else:
        port_name = pick_port(args.port)
        source = SerialSource(port_name, args.baud)

    csv_path = make_csv_path(args.csv)
    perf_path = make_perf_csv_path(args.stats_csv, csv_path)
    frame_queue = queue.Queue(maxsize=2000)
    worker = SensorWorker(source, csv_path, perf_path, frame_queue,
                          args.acc_lsb_per_g, args.gyro_lsb_per_dps)
    print(f"数据源：{port_name}")
    print(f"六轴 CSV：{csv_path}")
    print(f"性能 CSV：{perf_path}")
    print("提示：性能帧约 1Hz，收到时控制台会打印 [性能] 一行；没有该行说明没收到统计报文。")

    if args.no_plot:
        run_no_plot(worker, args.duration)
        print(f"六轴 CSV：{csv_path}（{worker.frames} 帧，校验错 {worker.bad}，丢帧≈{worker.lost}）")
        print(f"性能 CSV：{perf_path}（{worker.perf_frames} 帧，序号跳变≈{worker.perf_lost}）")
        return

    worker.start()
    monitor = SensorMonitor(worker, csv_path, args.window, args.duration)
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        worker.stop()
        worker.join(timeout=2.0)
    print(f"六轴 CSV：{csv_path}（{worker.frames} 帧，校验错 {worker.bad}，丢帧≈{worker.lost}）")
    print(f"性能 CSV：{perf_path}（{worker.perf_frames} 帧，序号跳变≈{worker.perf_lost}）")


def main():
    ap = argparse.ArgumentParser(
        description="Robin_24G_Car 六轴 / 台架姿态串口监视与记录")
    ap.add_argument("--list", action="store_true", help="列出所有串口后退出")
    ap.add_argument("--port", help="串口号，例如 COM3；不填则自动选择")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--csv", help="六轴 CSV 保存路径；默认按时间戳生成")
    ap.add_argument("--stats-csv", help="性能统计 CSV 路径；默认在六轴 CSV 名后加 _perf")
    ap.add_argument("--window", type=float, default=10.0, help="曲线显示窗口秒数（默认 10s）")
    ap.add_argument("--no-plot", action="store_true", help="不画图，只写 CSV")
    ap.add_argument("--duration", type=float, default=0, help="采集时长秒，0=一直采")
    ap.add_argument("--simulate", action="store_true",
                    help="用模拟数据源代替串口（无硬件自测界面/退出/CSV）")
    ap.add_argument("--acc-lsb-per-g", type=float, default=DEFAULT_ACC_LSB_PER_G,
                    help="加速度灵敏度（默认 2048 LSB/g，±16g）")
    ap.add_argument("--gyro-lsb-per-dps", type=float, default=DEFAULT_GYRO_LSB_PER_DPS,
                    help="陀螺灵敏度（默认 16.4 LSB/dps，±2000dps）")
    ap.add_argument("--attitude", action="store_true",
                    help="按台架固件的文本协议采集（ATT/STA 行，画横滚/俯仰/相对偏航）")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return
    run(args)


if __name__ == "__main__":
    main()
