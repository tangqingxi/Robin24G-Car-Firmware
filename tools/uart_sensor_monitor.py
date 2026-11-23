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

200Hz 地面联调固件（文本协议，--l400）：
  python uart_sensor_monitor.py --port COM12 --l400
  python uart_sensor_monitor.py --port COM12 --l400 --duration 120 --csv l400.csv
  * 解析 10Hz 的 L400 行和 1Hz 的 L400S 行；
  * g/c 是 Q10 rad/s，界面换算成 °/s 显示，它们不是欧拉角；
  * 观察数据写 --csv，统计写 --stats-csv（默认在观察文件名后加 _stats）。
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
#   STA n=0012C0 f=0000 ... z=FFFFFDF0/00000000/0190   ← 新：定宽十六进制原始值
#   STA n=1200 f=0 g=0 dt=4999/5001 rd=210/260 up=35/48   ← 旧：十进制（含均值）
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
                   # 台架 400Hz 验收追加（旧固件的 STA 没有这些字段，留空）
                   "cyc_avg_us", "cyc_max_us", "miss", "skip", "euler_ok",
                   "z_pre_lsb", "z_post_lsb"]

# ----------------------------------------------------------------------------
# 地面联调固件文本协议（--l400，兼容旧 7d01167 长行和当前 L400Q 短行）
#   L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 g=+12/-3/+0 c=-3/+0/-0
#        ta=+3000/-1500 tw=+512/-256/+0 on=1
#   L400S bad=0 n=12345 f=0 eul=12000 miss=0 cmiss=0 skip=0 txd=0 ctl=2490/3100
#   L400 init ok lpf_rb=3
#   L400 init fail st=N lpf_rb=M
# 说明：
#   * g 是扣偏后的三轴角速度，c 是内环 PID 纠偏量，二者都是 Q10 rad/s；
#     上位机按 °/s 画图，不能当欧拉角。
#   * ta 是外环目标横滚/俯仰（0.01°）；tw 是外环/偏航目标角速度（Q10 rad/s）；
#     on=1 表示控制输出使能。旧固件没有这三个字段，仍可解析。
#   * L400 没有设备时间戳，观察 CSV 与曲线的时间轴用上位机接收时刻。
#   * L400S 是累计统计，1Hz 一行；miss 是含串口报告的整拍超期，
#     cmiss 是控制计算完成超期；ctl 是控制完成时延（本周期最大/上电峰值），us。
# ----------------------------------------------------------------------------
# Q10 rad/s → °/s：1 rad/s = 180/π °/s，Q10 的 1024 对应 1 rad/s
Q10_RAD_S_TO_DPS = 180.0 / math.pi / 1024.0

# 超过这么久没有新的 L400 观察帧，就把 lk/vld 标为已过期。
# 用主机单调时间（time.monotonic），不受系统改时间影响。
L400_STALE_S = 1.0

# 短的定宽姿态观测：Q15 q + Q10 原始角速度/纠偏 + 状态位/包龄
#   L400Q f=F a=0023 v=0012 k=001F q=7FFF/0000/0000/0000
#         r=00000/00000/00000 c=0000/0000/0000
_L400_SHORT_RE = re.compile(
    r"^(?:L400Q|FCQ)\s+f=([0-9A-F])\s+a=([0-9A-F]{4})\s+v=([0-9A-F]{4})\s+"
    r"k=([0-9A-F]{4})\s+"
    r"q=([0-9A-F]{4})/([0-9A-F]{4})/([0-9A-F]{4})/([0-9A-F]{4})\s+"
    r"r=([0-9A-F]{5})/([0-9A-F]{5})/([0-9A-F]{5})\s+"
    r"c=([0-9A-F]{4})/([0-9A-F]{4})/([0-9A-F]{4})\s+"
    r"m=([0-9A-F])\s+x=([0-9A-F])\s+"
    r"pwm=([0-9A-F]{3})/([0-9A-F]{3})/([0-9A-F]{3})/([0-9A-F]{3})\s*$")

L400_LOCK_NAMES = {
    0: "无",
    1: "上电",
    2: "失联",
    3: "指令超时",
    4: "姿态异常",
    5: "手动急停",
    6: "诊断",
}

L400_CSV_FIELDS = [
    "recv_s", "recv_epoch", "t_ms",
    "rdy", "lk", "vld",
    "alk_ms", "avld_ms", "gpk_ms",
    "gx_q10", "gy_q10", "gz_q10",
    "cx_q10", "cy_q10", "cz_q10",
    "gx_dps", "gy_dps", "gz_dps",
    "cx_dps", "cy_dps", "cz_dps",
    "tgt_roll_deg100", "tgt_pitch_deg100",
    "tgt_rate_x_q10", "tgt_rate_y_q10", "tgt_rate_z_q10", "ctrl_ok",
    "qw_q15", "qx_q15", "qy_q15", "qz_q15",
    # 电机安全/实际输出：前/后/左/右
    "motor_armed", "motor_lock", "pwm_front", "pwm_rear",
    "pwm_left", "pwm_right",
]

L400_STAT_FIELDS = [
    "recv_s", "recv_epoch",
    "bad_pkt", "n", "f", "gap", "eul", "miss", "cmiss", "skip", "txd",
    "cyc_max_us", "cyc_peak_us",
    # 控制器联调：控制计算完成超期次数、控制完成时延（本周期/上电峰值）
    "ctrl_miss", "ctrl_max_us", "ctrl_peak_us",
    # 分项耗时：迟到 / IMU+估计+控制 / 无线收包 / 串口报告，单位 us
    "late_avg_us", "late_max_us",
    "exec_avg_us", "exec_max_us",
    "rx_avg_us", "rx_max_us",
    "tx_avg_us", "tx_max_us",
]

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


# 定宽十六进制 STA：n/f/g/dt/rd/up/cyc/miss/skip/eul 全是原始值，
# 平均值与 Z 轴均值由上位机算；旧固件的十进制行仍走下面的 _STA_LINE_RE
_STA_HEX_RE = re.compile(
    r"^STA\s+n=([0-9A-F]{6})\s+f=([0-9A-F]{4})\s+g=([0-9A-F]{4})\s+"
    r"dt=([0-9A-F]{4})/([0-9A-F]{4})\s+rd=([0-9A-F]{8})/([0-9A-F]{4})\s+"
    r"up=([0-9A-F]{8})/([0-9A-F]{4})\s+cyc=([0-9A-F]{8})/([0-9A-F]{4})\s+"
    r"miss=([0-9A-F]{6})\s+skip=([0-9A-F]{6})\s+eul=([0-9A-F]{6})\s+"
    r"z=([0-9A-F]{8})/([0-9A-F]{8})/([0-9A-F]{6})\s*$")

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
    m = _STA_HEX_RE.match(line)
    if m:
        g = [int(x, 16) for x in m.groups()]
        n = g[0]
        z_cnt = g[16]

        def _s32(v):
            """按 32 位补码解释有符号字段"""
            return v - 0x100000000 if v >= 0x80000000 else v

        return ("sta", {
            "valid": g[0], "fail": g[1], "gap": g[2],
            "dt_min_us": g[3], "dt_max_us": g[4],
            # 平均值由累加值 / 样本数现算，和旧固件板端算的一致
            "rd_avg_us": (g[5] // n) if n else 0,
            "rd_max_us": g[6],
            "up_avg_us": (g[7] // n) if n else 0,
            "up_max_us": g[8],
            "cyc_avg_us": (g[9] // n) if n else 0,
            "cyc_max_us": g[10],
            "miss": g[11], "skip": g[12], "euler_ok": g[13],
            # Z 轴：累加值 / 样本数，扣偏后是 1/64 LSB
            "z_pre_lsb": round(_s32(g[14]) / z_cnt, 2) if z_cnt else None,
            "z_post_lsb": round((_s32(g[15]) / z_cnt) / 64.0, 2) if z_cnt else None,
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


# L400 观察行：rdy/lk/vld、两种包龄、最大包间隔、g/c 三轴 Q10 rad/s
_L400_LINE_RE = re.compile(
    r"^L400\s+rdy=([01])\s+lk=([01])\s+vld=([01])\s+"
    r"alk=(\d+)\s+avld=(\d+)\s+gpk=(\d+)\s+"
    r"g=([+-]?\d+)/([+-]?\d+)/([+-]?\d+)\s+"
    r"c=([+-]?\d+)/([+-]?\d+)/([+-]?\d+)"
    r"(?:\s+ta=([+-]?\d+)/([+-]?\d+)\s+"
    r"tw=([+-]?\d+)/([+-]?\d+)/([+-]?\d+)\s+on=([01])"
    r"(?:\s+q=([0-9A-F]{4})/([0-9A-F]{4})/([0-9A-F]{4})/([0-9A-F]{4}))?)?\s*$")

# L400S 统计行旧格式（7d01167）：十进制计数 + 整拍耗时
_L400_STAT_RE = re.compile(
    r"^L400S\s+bad=(\d+)\s+n=(\d+)\s+f=(\d+)\s+gap=(\d+)\s+"
    r"eul=(\d+)\s+miss=(\d+)\s+skip=(\d+)\s+txd=(\d+)\s+"
    r"cyc=(\d+)/(\d+)\s*$")

# A/B 分段诊断汇总：A=只收包+姿态，B=再开双环；耗时单位 us
_L400_DIAG_RE = re.compile(
    r"^L400D\s+([AB])\s+late=(\d+)/(\d+)\s+exec=(\d+)/(\d+)\s+"
    r"cmiss=(\d+)\s+skip=(\d+)\s+rfirq=(\d+)/(\d+)\s+"
    r"rf=(\d+)/(\d+)\s+app=(\d+)/(\d+)\s*$")

# 掉链中止：保留已完成 A 段标志和原因
_L400_DIAG_ABORT_RE = re.compile(
    r"^L400D\s+ABORT\s+seg=([AB])\s+reason=(link|cmd)\s+haveA=([01])\s*$")

# 常规报告延期原因：余量不足 / UART(DMA) 忙
_L400_TXDIAG_RE = re.compile(
    r"^L400T\s+budget=(\d+)\s+busy=(\d+)\s*$")

# K1/K2 原始收包抓包：key/rk/原始字节/包间隔
_L400_KEY_RE = re.compile(
    r"^L400K\s+key=([0-9A-F]{4})\s+rk=([0-9A-F]{2})\s+"
    r"b=([0-9A-F]{2})/([0-9A-F]{2})/([0-9A-F]{2})/([0-9A-F]{2})\s+"
    r"dt=(\d+)\s*$")

# prep / Euler_Update 两段耗时（us，1Hz）
_L400_PREP_RE = re.compile(
    r"^L400P\s+prep=(\d+)/(\d+)\s+eul=(\d+)/(\d+)\s+"
    r"n=(\d+)\s+e=(\d+)\s*$")

# L400S 统计行新格式：定宽十六进制；miss=整拍超期，cmiss=控制完成超期，
# ctl=控制完成时延（本周期最大/上电峰值）；late/exec/rx/tx 为分项耗时。
_L400_STAT_HEX_RE = re.compile(
    r"^(?:L400S|FCS)\s+bad=([0-9A-F]{4})\s+n=([0-9A-F]{6})\s+f=([0-9A-F]{4})\s+"
    r"eul=([0-9A-F]{6})\s+miss=([0-9A-F]{6})\s+cmiss=([0-9A-F]{6})\s+"
    r"skip=([0-9A-F]{6})\s+txd=([0-9A-F]{6})\s+ctl=([0-9A-F]{4})/([0-9A-F]{4})\s+"
    r"late=([0-9A-F]{4})/([0-9A-F]{4})\s+exec=([0-9A-F]{4})/([0-9A-F]{4})\s+"
    r"rx=([0-9A-F]{4})/([0-9A-F]{4})\s+tx=([0-9A-F]{4})/([0-9A-F]{4})\s*$")


def q10_rad_s_to_dps(value):
    """Q10 rad/s → °/s（1024 = 1 rad/s，1 rad/s = 180/π °/s）"""
    return value * Q10_RAD_S_TO_DPS


def _hex_s16(text):
    """4 位十六进制 → 有符号 16 位（Q15/Q10 分量的两补码表示）"""
    value = int(text, 16)
    return value - 65536 if value >= 32768 else value


def _hex_s20(text):
    """5 位十六进制 → 有符号 20 位（Q10 角速度的两补码表示）"""
    value = int(text, 16)
    return value - 0x100000 if value >= 0x80000 else value


def q15_to_euler_deg(q):
    """
    Q15 四元数 [w,x,y,z] → 欧拉角 (横滚, 俯仰, 相对偏航)，单位度。

    与固件 UserEuler 的机体 FRD→水平系约定一致（ZYX 顺序）：
      roll  = atan2(2(wx+yz), 1-2(x²+y²))
      pitch = asin(2(wy-zx))
      yaw   = atan2(2(wz+xy), 1-2(y²+z²))
    四元数无效或模长过小时返回 None。
    """
    if not q or len(q) != 4:
        return None
    w, x, y, z = (v / 32768.0 for v in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return None
    w, x, y, z = (v / norm for v in (w, x, y, z))
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    sin_p = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_p)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def l400_state_of(line):
    """把 L400 初始化状态文字归类成界面显示的状态"""
    if line.startswith("L400 init fail"):
        return "初始化失败"
    if line.startswith("L400 init ok"):
        return "初始化完成"
    return None


def l400_detail_of(line):
    """从 L400 初始化行里取 st / lpf_rb，放在状态栏说明里"""
    if line.startswith("L400 init fail"):
        m = re.search(r"st=(\d+)\s+lpf_rb=(\d+)", line)
        return f"st={m.group(1)} 回读={m.group(2)}" if m else ""
    if line.startswith("L400 init ok"):
        m = re.search(r"lpf_rb=(\d+)", line)
        return f"低通回读={m.group(1)}" if m else ""
    return ""


def parse_l400_line(line):
    """
    解析一行 L400 文本：
      返回 ("l400", 观察数据) / ("l400s", 统计数据) / ("status", 原文) / None
    观察数据同时保留 Q10 原值和换算后的 °/s，CSV 两种都写，方便复核。
    """
    m = _L400_LINE_RE.match(line)
    if m:
        g_q10 = [int(m.group(i)) for i in (7, 8, 9)]
        c_q10 = [int(m.group(i)) for i in (10, 11, 12)]
        tgt_att = None
        tgt_rate = None
        ctrl_ok = None
        q15 = None
        if m.group(13) is not None:
            tgt_att = [int(m.group(13)), int(m.group(14))]
            tgt_rate = [int(m.group(i)) for i in (15, 16, 17)]
            ctrl_ok = int(m.group(18))
        if m.group(19) is not None:
            q15 = [_hex_s16(m.group(i)) for i in (19, 20, 21, 22)]
        return ("l400", {
            "rdy": int(m.group(1)),
            "lk": int(m.group(2)),
            "vld": int(m.group(3)),
            "alk_ms": int(m.group(4)),
            "avld_ms": int(m.group(5)),
            "gpk_ms": int(m.group(6)),
            "g_q10": g_q10,
            "c_q10": c_q10,
            "g_dps": [q10_rad_s_to_dps(v) for v in g_q10],
            "c_dps": [q10_rad_s_to_dps(v) for v in c_q10],
            "tgt_att_deg100": tgt_att,
            "tgt_rate_q10": tgt_rate,
            "ctrl_ok": ctrl_ok,
            "q15": q15,
        })

    m = _L400_SHORT_RE.match(line)
    if m:
        flags = int(m.group(1), 16)
        q15 = [_hex_s16(m.group(i)) for i in (5, 6, 7, 8)]
        g_q10 = [_hex_s20(m.group(i)) for i in (9, 10, 11)]
        c_q10 = [_hex_s16(m.group(i)) for i in (12, 13, 14)]
        return ("l400", {
            "rdy": flags & 0x01,
            "lk": (flags >> 1) & 0x01,
            "vld": (flags >> 2) & 0x01,
            "ctrl_ok": (flags >> 3) & 0x01,
            "alk_ms": int(m.group(2), 16),
            "avld_ms": int(m.group(3), 16),
            "gpk_ms": int(m.group(4), 16),
            "g_q10": g_q10,
            "c_q10": c_q10,
            "g_dps": [q10_rad_s_to_dps(v) for v in g_q10],
            "c_dps": [q10_rad_s_to_dps(v) for v in c_q10],
            "tgt_att_deg100": None,
            "tgt_rate_q10": None,
            "q15": q15,
            "motor_armed": int(m.group(15), 16),
            "motor_lock": int(m.group(16), 16),
            "pwm_front": int(m.group(17), 16),
            "pwm_rear": int(m.group(18), 16),
            "pwm_left": int(m.group(19), 16),
            "pwm_right": int(m.group(20), 16),
        })

    m = _L400_TXDIAG_RE.match(line)
    if m:
        return ("l400t", {
            "budget": int(m.group(1)),
            "busy": int(m.group(2)),
        })

    m = _L400_KEY_RE.match(line)
    if m:
        return ("l400k", {
            "key": int(m.group(1), 16),
            "rk": int(m.group(2), 16),
            "b8": int(m.group(3), 16),
            "b9": int(m.group(4), 16),
            "b10": int(m.group(5), 16),
            "b11": int(m.group(6), 16),
            "dt_ms": int(m.group(7)),
        })

    m = _L400_PREP_RE.match(line)
    if m:
        return ("l400p", {
            "prep_avg_us": int(m.group(1)),
            "prep_max_us": int(m.group(2)),
            "eul_avg_us": int(m.group(3)),
            "eul_max_us": int(m.group(4)),
            "prep_n": int(m.group(5)),
            "eul_n": int(m.group(6)),
        })

    m = _L400_DIAG_ABORT_RE.match(line)
    if m:
        return ("l400d_abort", {
            "seg": m.group(1),
            "reason": m.group(2),
            "have_a": int(m.group(3)),
        })

    m = _L400_DIAG_RE.match(line)
    if m:
        return ("l400d", {
            "seg": m.group(1),
            "late_avg_us": int(m.group(2)),
            "late_max_us": int(m.group(3)),
            "exec_avg_us": int(m.group(4)),
            "exec_max_us": int(m.group(5)),
            "cmiss": int(m.group(6)),
            "skip": int(m.group(7)),
            "rfirq_avg_us": int(m.group(8)),
            "rfirq_max_us": int(m.group(9)),
            "rf_avg_us": int(m.group(10)),
            "rf_max_us": int(m.group(11)),
            "app_avg_us": int(m.group(12)),
            "app_max_us": int(m.group(13)),
        })

    m = _L400_STAT_HEX_RE.match(line)
    if m:
        g = [int(x, 16) for x in m.groups()]
        return ("l400s", {
            "bad_pkt": g[0],
            "n": g[1],
            "f": g[2],
            "gap": None,               # 新格式不再输出长间断计数
            "eul": g[3],
            "miss": g[4],
            "cmiss": g[5],
            "skip": g[6],
            "txd": g[7],
            "cyc_max_us": None,        # 旧格式的整拍最大/峰值
            "cyc_peak_us": None,
            "ctrl_miss": g[5],
            "ctrl_max_us": g[8],
            "ctrl_peak_us": g[9],
            "late_avg_us": g[10],
            "late_max_us": g[11],
            "exec_avg_us": g[12],
            "exec_max_us": g[13],
            "rx_avg_us": g[14],
            "rx_max_us": g[15],
            "tx_avg_us": g[16],
            "tx_max_us": g[17],
        })

    m = _L400_STAT_RE.match(line)
    if m:
        # 旧 7d01167 格式没有分项耗时：保持 None，CSV 留空
        return ("l400s", {
            "bad_pkt": int(m.group(1)),
            "n": int(m.group(2)),
            "f": int(m.group(3)),
            "gap": int(m.group(4)),
            "eul": int(m.group(5)),
            "miss": int(m.group(6)),
            "cmiss": None,
            "skip": int(m.group(7)),
            "txd": int(m.group(8)),
            "cyc_max_us": int(m.group(9)),
            "cyc_peak_us": int(m.group(10)),
            "ctrl_miss": None,
            "ctrl_max_us": None,
            "ctrl_peak_us": None,
            "late_avg_us": None,
            "late_max_us": None,
            "exec_avg_us": None,
            "exec_max_us": None,
            "rx_avg_us": None,
            "rx_max_us": None,
            "tx_avg_us": None,
            "tx_max_us": None,
        })

    if l400_state_of(line):
        return ("status", line)
    return None


def _hex(v, digits):
    """按固定宽度输出十六进制（模拟源用，与固件一致）"""
    return f"{v & ((1 << (4 * digits)) - 1):0{digits}X}"


def _q15_hex(v):
    """浮点 Q15 分量 → 4 位有符号十六进制（+32768 夹到 +32767）"""
    raw = int(round(v * 32768.0))
    raw = max(-32768, min(32767, raw))
    return _hex(raw & 0xFFFF, 4)


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
                # 与固件一致：定宽十六进制原始值，均值交给接收端算
                self._push(f"STA n={_hex(valid, 6)} f={_hex(0, 4)} g={_hex(0, 4)} "
                           f"dt={_hex(2499, 4)}/{_hex(2501, 4)} "
                           f"rd={_hex(110 * valid, 8)}/{_hex(130, 4)} "
                           f"up={_hex(30 * valid, 8)}/{_hex(42, 4)} "
                           f"cyc={_hex(620 * valid, 8)}/{_hex(780, 4)} "
                           f"miss={_hex(0, 6)} skip={_hex(0, 6)} eul={_hex(valid, 6)} "
                           f"z={_hex(int(round(-1.32 * valid)) & 0xFFFFFFFF, 8)}/{_hex(0, 8)}/{_hex(valid, 6)}")
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


class SimL400Source:
    """
    模拟 7d01167 联调固件的文本输出（无硬件自测）：
      init ok → 0.5s 内 rdy=0 → rdy=1；10Hz L400 + 1Hz L400S。
      g/c 用 Q10 rad/s 的正弦量，c 约为 -g/4；读取时故意把行切碎，
      验证串口分块到达时的粘包处理。
    """

    def __init__(self, hz=10.0):
        self.dt = 1.0 / hz
        self.t0 = time.time()
        self.buf = b""
        self.n_line = 0
        self.n_stat = 0
        self.sent_init = False
        self._next_line = 0.0
        self._next_stat = 0.0
        self.sample = 0
        self.euler = 0
        self.miss = 0
        self.cmiss = 0
        self.skip = 0
        self.txd = 0
        self.peak = 2400
        self.ctrl_peak = 2500

    def _push(self, text):
        self.buf += (text + "\n").encode("ascii", "replace")

    def _fill(self):
        now = time.time()
        el = now - self.t0
        if not self.sent_init:
            self.sent_init = True
            self._push("L400 init ok lpf_rb=3")

        # 10Hz 观察行：g/c 用缓慢正弦，数值量级与静止/轻晃时接近
        if el >= self._next_line:
            self._next_line = el + self.dt
            self.n_line += 1
            gx = int(360 * math.sin(2 * math.pi * 0.50 * el))
            gy = int(220 * math.sin(2 * math.pi * 0.35 * el + 1.0))
            gz = int(90 * math.sin(2 * math.pi * 0.20 * el + 2.0))
            # 与固件一致：corr = -(g * 64) >> 8，即约 -g/4
            cx = -((gx * 64) >> 8)
            cy = -((gy * 64) >> 8)
            cz = -((gz * 64) >> 8)
            rdy = 1 if el >= 0.5 else 0
            avld = 8 + (self.n_line * 7) % 45
            alk = avld + 5
            gpk = 25 + (self.n_line * 3) % 12
            self.sample += 20
            if rdy:
                self.euler += 20
            # 偶尔累加统计量，验证界面数字变化；频率远低于 1Hz
            if self.n_line % 97 == 0:
                self.miss += 1
            if self.n_line % 131 == 0:
                self.skip += 1
            if self.n_line % 149 == 0:
                self.txd += 1
            if self.n_line % 181 == 0:
                self.cmiss += 1
            ctl = 2250 + (self.n_line * 13) % 260
            if ctl > self.ctrl_peak:
                self.ctrl_peak = ctl
            on = 1 if rdy else 0
            # 模拟实测姿态：用缓慢变化的横滚/俯仰构造 Q15 四元数
            rr = math.radians(10.0 * math.sin(2 * math.pi * 0.18 * el))
            pp = math.radians(6.0 * math.sin(2 * math.pi * 0.13 * el + 0.4))
            qw = math.cos(pp / 2) * math.cos(rr / 2)
            qx = math.cos(pp / 2) * math.sin(rr / 2)
            qy = math.sin(pp / 2) * math.cos(rr / 2)
            qz = -math.sin(pp / 2) * math.sin(rr / 2)
            flags = (1 if rdy else 0) | 0x02 | 0x04 | (0x08 if on else 0)
            self._push(
                f"FCQ f={flags:X} a={_hex(alk, 4)} v={_hex(avld, 4)} "
                f"k={_hex(gpk, 4)} "
                f"q={_q15_hex(qw)}/{_q15_hex(qx)}/{_q15_hex(qy)}/{_q15_hex(qz)} "
                f"r={_hex(gx & 0xFFFFF, 5)}/{_hex(gy & 0xFFFFF, 5)}/"
                f"{_hex(gz & 0xFFFFF, 5)} "
                f"c={_hex(cx & 0xFFFF, 4)}/{_hex(cy & 0xFFFF, 4)}/"
                f"{_hex(cz & 0xFFFF, 4)} "
                f"m=0 x=1 pwm=000/000/000/000")

        # 1Hz 统计行：n/eul 与上面的 200Hz 计数保持同一比例
        if el >= self._next_stat:
            self._next_stat = el + 1.0
            self.n_stat += 1
            ctl = 2300 + (self.n_stat * 41) % 260
            if ctl > self.ctrl_peak:
                self.ctrl_peak = ctl
            # 模拟新固件的定宽十六进制统计行：控制超期 + 四个分项耗时
            late_avg = 900 + (self.n_stat * 7) % 200
            exec_avg = 1000 + (self.n_stat * 11) % 200
            rx_avg = 30 + (self.n_stat * 3) % 20
            tx_avg = 45 + (self.n_stat * 5) % 25
            self._push(
                f"FCS bad={_hex(0, 4)} n={_hex(self.sample, 6)} "
                f"f={_hex(0, 4)} eul={_hex(self.euler, 6)} "
                f"miss={_hex(self.miss, 6)} cmiss={_hex(self.cmiss, 6)} "
                f"skip={_hex(self.skip, 6)} txd={_hex(self.txd, 6)} "
                f"ctl={_hex(ctl, 4)}/{_hex(self.ctrl_peak, 4)} "
                f"late={_hex(late_avg, 4)}/{_hex(late_avg + 300, 4)} "
                f"exec={_hex(exec_avg, 4)}/{_hex(exec_avg + 250, 4)} "
                f"rx={_hex(rx_avg, 4)}/{_hex(rx_avg + 30, 4)} "
                f"tx={_hex(tx_avg, 4)}/{_hex(tx_avg + 60, 4)}")

    def read(self):
        self._fill()
        if not self.buf:
            time.sleep(0.01)
            return b""
        # 故意只取一小段，甚至切断一行，模拟串口分块到达
        n = min(len(self.buf), 1 + (self.n_line * 11 + self.n_stat * 5) % 37)
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


class L400Worker(threading.Thread):
    """联调文本采集线程：拆行 → 解析 L400/L400Q/L400S → 两份 CSV → 观察样本入队"""

    def __init__(self, source, csv_path, stat_path, sample_queue):
        super().__init__(daemon=True)
        self.source = source
        self.csv_path = csv_path
        self.stat_path = stat_path
        # A/B 诊断汇总写在统计 CSV 旁边的 .diag.txt，方便整份回传
        self.diag_path = stat_path + ".diag.txt"
        self.queue = sample_queue
        self.stop_event = threading.Event()

        self.frames = 0          # L400 观察行条数
        self.stats = 0           # L400S 统计行条数
        self.diag = 0            # L400D A/B 汇总行条数
        self.last_diag = None
        self.bad = 0             # 无法识别的行数
        self.state = "等待数据"  # 初始化完成 / 初始化失败 / 等待数据
        self.detail = ""
        self.last_stat = None    # 最近一条 L400S
        self.last_obs = None     # 最近一条 L400
        self.last_obs_mono = 0.0 # 最近一条有效观察帧的主机单调时间（秒）
        self.error = ""
        self.start_wall = time.time()
        self._buf = b""

    def stop(self):
        self.stop_event.set()

    def summary_line(self):
        return (f"L400 {self.frames} 条  统计 {self.stats} 条  "
                f"诊断 {self.diag} 条  无法识别 {self.bad} 行  状态 {self.state}")

    def _handle_line(self, line, writer, stat_writer):
        parsed = parse_l400_line(line)
        if parsed is None:
            self.bad += 1
            return

        kind, payload = parsed
        now = time.time()

        if kind == "l400":
            self.frames += 1
            self.last_obs = payload
            self.last_obs_mono = time.monotonic()
            t_ms = int((now - self.start_wall) * 1000)
            payload["t_ms"] = t_ms
            writer.writerow({
                "recv_s": f"{now - self.start_wall:.3f}",
                "recv_epoch": f"{now:.3f}",
                "t_ms": t_ms,
                "rdy": payload["rdy"],
                "lk": payload["lk"],
                "vld": payload["vld"],
                "alk_ms": payload["alk_ms"],
                "avld_ms": payload["avld_ms"],
                "gpk_ms": payload["gpk_ms"],
                "gx_q10": payload["g_q10"][0],
                "gy_q10": payload["g_q10"][1],
                "gz_q10": payload["g_q10"][2],
                "cx_q10": payload["c_q10"][0],
                "cy_q10": payload["c_q10"][1],
                "cz_q10": payload["c_q10"][2],
                "gx_dps": f"{payload['g_dps'][0]:.3f}",
                "gy_dps": f"{payload['g_dps'][1]:.3f}",
                "gz_dps": f"{payload['g_dps'][2]:.3f}",
                "cx_dps": f"{payload['c_dps'][0]:.3f}",
                "cy_dps": f"{payload['c_dps'][1]:.3f}",
                "cz_dps": f"{payload['c_dps'][2]:.3f}",
                # 控制器联调字段；旧固件没有，留空
                "tgt_roll_deg100": "" if payload.get("tgt_att_deg100") is None else payload["tgt_att_deg100"][0],
                "tgt_pitch_deg100": "" if payload.get("tgt_att_deg100") is None else payload["tgt_att_deg100"][1],
                "tgt_rate_x_q10": "" if payload.get("tgt_rate_q10") is None else payload["tgt_rate_q10"][0],
                "tgt_rate_y_q10": "" if payload.get("tgt_rate_q10") is None else payload["tgt_rate_q10"][1],
                "tgt_rate_z_q10": "" if payload.get("tgt_rate_q10") is None else payload["tgt_rate_q10"][2],
                "ctrl_ok": "" if payload.get("ctrl_ok") is None else payload["ctrl_ok"],
                "qw_q15": "" if payload.get("q15") is None else payload["q15"][0],
                "qx_q15": "" if payload.get("q15") is None else payload["q15"][1],
                "qy_q15": "" if payload.get("q15") is None else payload["q15"][2],
                "qz_q15": "" if payload.get("q15") is None else payload["q15"][3],
                "motor_armed": "" if payload.get("motor_armed") is None else payload["motor_armed"],
                "motor_lock": "" if payload.get("motor_lock") is None else payload["motor_lock"],
                "pwm_front": "" if payload.get("pwm_front") is None else payload["pwm_front"],
                "pwm_rear": "" if payload.get("pwm_rear") is None else payload["pwm_rear"],
                "pwm_left": "" if payload.get("pwm_left") is None else payload["pwm_left"],
                "pwm_right": "" if payload.get("pwm_right") is None else payload["pwm_right"],
            })
            try:
                self.queue.put_nowait(payload)
            except queue.Full:
                pass
            return

        if kind in ("l400d", "l400d_abort", "l400t", "l400p", "l400k"):
            self.diag += 1
            self.last_diag = payload
            try:
                with open(self.diag_path, "a", encoding="utf-8") as fp:
                    fp.write(line + "\n")
            except OSError as exc:
                self.error = f"写诊断文件失败：{exc}"
            if kind == "l400d":
                tag = "诊断"
            elif kind == "l400d_abort":
                tag = "诊断-中止"
            elif kind == "l400p":
                tag = "耗时"
            elif kind == "l400k":
                tag = "按键"
            else:
                tag = "报告延期"
            print(f"[{tag}] {line}", flush=True)
            return

        if kind == "l400s":
            self.stats += 1
            self.last_stat = payload
            stat_writer.writerow({
                "recv_s": f"{now - self.start_wall:.3f}",
                "recv_epoch": f"{now:.3f}",
                "bad_pkt": payload["bad_pkt"],
                "n": payload["n"],
                "f": payload["f"],
                "gap": payload["gap"],
                "eul": payload["eul"],
                "miss": payload["miss"],
                "cmiss": "" if payload.get("cmiss") is None else payload["cmiss"],
                "skip": payload["skip"],
                "txd": payload["txd"],
                "cyc_max_us": "" if payload.get("cyc_max_us") is None else payload["cyc_max_us"],
                "cyc_peak_us": "" if payload.get("cyc_peak_us") is None else payload["cyc_peak_us"],
                "ctrl_miss": "" if payload.get("ctrl_miss") is None else payload["ctrl_miss"],
                "ctrl_max_us": "" if payload.get("ctrl_max_us") is None else payload["ctrl_max_us"],
                "ctrl_peak_us": "" if payload.get("ctrl_peak_us") is None else payload["ctrl_peak_us"],
                # 旧 7d01167 格式没有分项耗时：CSV 留空
                "late_avg_us": "" if payload.get("late_avg_us") is None else payload["late_avg_us"],
                "late_max_us": "" if payload.get("late_max_us") is None else payload["late_max_us"],
                "exec_avg_us": "" if payload.get("exec_avg_us") is None else payload["exec_avg_us"],
                "exec_max_us": "" if payload.get("exec_max_us") is None else payload["exec_max_us"],
                "rx_avg_us": "" if payload.get("rx_avg_us") is None else payload["rx_avg_us"],
                "rx_max_us": "" if payload.get("rx_max_us") is None else payload["rx_max_us"],
                "tx_avg_us": "" if payload.get("tx_avg_us") is None else payload["tx_avg_us"],
                "tx_max_us": "" if payload.get("tx_max_us") is None else payload["tx_max_us"],
            })
            line = (f"[L400统计] 采样{payload['n']} 姿态更新{payload['eul']} "
                    f"读失败{payload['f']} 坏包{payload['bad_pkt']}")
            if payload.get("cmiss") is not None:
                line += (f" 整拍超期{payload['miss']} 控制超期{payload['cmiss']}")
            else:
                line += f" 超时{payload['miss']}"
            line += f" 跳拍{payload['skip']} 串口推迟{payload['txd']}"
            if payload.get("ctrl_max_us") is not None:
                line += (f" 控制完成{payload['ctrl_max_us']}/"
                         f"{payload['ctrl_peak_us']}us")
            elif payload.get("cyc_max_us") is not None:
                line += f" 整拍{payload['cyc_max_us']}/{payload['cyc_peak_us']}us"
            if payload.get("late_avg_us") is not None:
                line += (f" 迟到{payload['late_avg_us']}/{payload['late_max_us']}us"
                         f" 执行{payload['exec_avg_us']}/{payload['exec_max_us']}us"
                         f" 收包{payload['rx_avg_us']}/{payload['rx_max_us']}us"
                         f" 报告{payload['tx_avg_us']}/{payload['tx_max_us']}us")
            print(line, flush=True)
            return

        # 初始化状态行
        self.state = l400_state_of(line) or self.state
        self.detail = l400_detail_of(line)
        print(f"[联调] {line}", flush=True)

    def run(self):
        try:
            with open(self.csv_path, "w", newline="", encoding="utf-8-sig") as fp, \
                    open(self.stat_path, "w", newline="", encoding="utf-8-sig") as sfp:
                writer = csv.DictWriter(fp, fieldnames=L400_CSV_FIELDS)
                writer.writeheader()
                stat_writer = csv.DictWriter(sfp, fieldnames=L400_STAT_FIELDS)
                stat_writer.writeheader()

                while not self.stop_event.is_set():
                    data = self.source.read()
                    if not data:
                        continue
                    self._buf += data
                    # 串口分块到达时，残行留在缓冲里等下一块
                    while b"\n" in self._buf:
                        raw, self._buf = self._buf.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").strip()
                        if line:
                            self._handle_line(line, writer, stat_writer)
                    if len(self._buf) > 512:
                        # 迟迟没有换行：多半是波特率/接线不对，丢掉避免无限增长
                        self.bad += 1
                        self._buf = b""
                    # 10Hz 观察行，约每 10 条落一次盘；1Hz 统计行每条都落盘
                    if self.frames and self.frames % 10 == 0:
                        fp.flush()
                        sfp.flush()
        except Exception as exc:          # 串口异常：记录下来给界面/控制台
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
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
        # "sensor"=六轴二进制帧；"attitude"=台架文本；"l400"=200Hz 联调文本
        self.mode = mode

        self.follow = True          # 是否自动跟随最新数据
        self.fig = None
        self.axes = []
        self.lines = []
        self.text = None
        self.stat_text = None       # L400 统计区（多行文字），其他模式为 None
        self.timer = None
        self.toolbar = None
        self._ticks = 0             # 定时器回调次数（自测用）
        self._last_text = ""        # 状态栏缓存，文字没变就不重画
        self._last_stat_text = ""   # 统计区缓存
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
        self.hist_g = [deque(maxlen=self.cap) for _ in range(3)]     # L400 实测角速度 dps
        self.hist_c = [deque(maxlen=self.cap) for _ in range(3)]     # L400 内环纠偏 dps
        self.hist_att = [deque(maxlen=self.cap) for _ in range(3)]   # L400 四元数换算的欧拉角
        self.last_l400 = None
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
        elif self.mode == "l400":
            # 200Hz 联调模式：三张图。前两张是实测角速度 g 与内环纠偏 c
            # （Q10 rad/s 换算的 °/s）；第三张是低频观测里的 Q15 四元数
            # 在上位机换算出的实测欧拉角，固件控制路径不参与欧拉角计算。
            self.fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 7.8))
            self.fig.canvas.manager.set_window_title(
                "Robin_24G_Car 200Hz 联调监视器（g/c 角速度 + 实测姿态）")
            self.axes = list(axes)
            ax_g, ax_c, ax_att = self.axes
            for ax_obj, ylabel, lab in (
                    (ax_g, "实测角速度 g (°/s)", ("g-X", "g-Y", "g-Z")),
                    (ax_c, "内环纠偏 c (°/s)", ("c-X", "c-Y", "c-Z")),
                    (ax_att, "实测姿态 (°)", ("横滚", "俯仰", "相对偏航"))):
                ax_obj.grid(True, alpha=0.3)
                ax_obj.set_ylabel(ylabel)
                lines = [ax_obj.plot([], [], color=color, lw=0.9, label=name)[0]
                         for color, name in zip(colors, lab)]
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
        self.text = self.fig.text(0.01, 0.010, "等待数据…", fontsize=9)
        self.text.set_animated(True)      # 同曲线，避免文字变短时留残影
        if self.mode == "l400":
            # L400 统计区：采样/姿态更新/读失败/超时/跳拍/串口推迟等累计量。
            # 放到状态栏上方，底部留出 22% 空间给两行统计文字和 x 轴标签。
            self.stat_text = self.fig.text(
                0.01, 0.040, "统计：等待 L400S（1Hz）…",
                fontsize=8, va="bottom")
            self.stat_text.set_animated(True)
            # 控制器联调统计区最多五行（累计量、超期、耗时两行、控制目标），
            # 底部留 32%，避免文字压到 x 轴标签
            self.fig.subplots_adjust(left=0.07, right=0.99, top=0.95, bottom=0.32)
        else:
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

        if self.mode == "l400":
            # 高 DPI 屏幕上窗口可能被窗口管理器放到屏幕外，底部统计区被裁掉；
            # 这里只设一次初始大小/位置，之后仍可正常拖动、缩放。
            try:
                win = self.fig.canvas.manager.window
                screen_w = win.winfo_screenwidth()
                screen_h = win.winfo_screenheight()
                fig_w = int(self.fig.get_size_inches()[0] * self.fig.dpi)
                fig_h = int(self.fig.get_size_inches()[1] * self.fig.dpi)
                win_w = min(fig_w + 20, screen_w - 80)
                win_h = min(fig_h + 90, screen_h - 120)
                win.geometry(f"{win_w}x{win_h}+40+40")
            except Exception:
                pass

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
        if self.mode == "l400":
            # g/c 已在解析时换算成 °/s；这里只负责入环形缓冲
            for arr, val in zip(self.hist_g, fr["g_dps"]):
                arr.append(val)
            for arr, val in zip(self.hist_c, fr["c_dps"]):
                arr.append(val)
            # 旧固件/旧镜像没有 q：填 NaN 让第三张图留空，长度仍与时间轴一致
            euler = q15_to_euler_deg(fr.get("q15"))
            if euler is None:
                euler = (float("nan"),) * 3
            for arr, val in zip(self.hist_att, euler):
                arr.append(val)
            self.last_l400 = fr
            self._dirty = True
            return
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
        """返回 [(数据序列组, 曲线组)]，顺序与 self.axes 一一对应"""
        if self.mode == "attitude":
            return [(self.hist, self.lines[0])]
        if self.mode == "l400":
            return [(self.hist_g, self.lines[0]),
                    (self.hist_c, self.lines[1]),
                    (self.hist_att, self.lines[2])]
        return [(self.hist, self.lines[0]),
                (self.hist_gyro, self.lines[1]),
                (self.hist_gyro_c, self.lines[2])]

    def _status(self):
        if self.mode == "attitude":
            return self._status_attitude()
        if self.mode == "l400":
            return self._status_l400()
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

    def _status_l400(self):
        """L400 状态栏：姿态就绪、遥控连接、指令有效、有效包龄、峰值/控制状态"""
        w = self.worker
        # 用 worker 的最新观察行，而不是绘图缓冲：关闭“跟随”时状态栏仍然实时
        o = w.last_obs or self.last_l400
        s = w.last_stat
        if s and s.get("ctrl_peak_us") is not None:
            peak = f"{s['ctrl_peak_us']}us"
        elif s and s.get("cyc_peak_us") is not None:
            peak = f"{s['cyc_peak_us']}us"
        else:
            peak = "--"
        if o is None:
            msg = f"等待 L400 数据…（{w.state}）"
            if w.detail:
                msg += f" {w.detail}"
            msg += f" | 观察 {w.frames} 条 统计 {w.stats} 条"
            msg += " | F 跟随 / Q 退出"
            if w.error:
                msg += f" | 串口错误：{w.error}"
            return msg

        # 串口停发后 worker.last_obs 不会变；若只看它，会一直显示旧的
        # “遥控连接/指令有效”。用最近一条观察帧的主机单调时间判断过期。
        age = time.monotonic() - w.last_obs_mono
        if age > L400_STALE_S:
            msg = (f"串口数据中断（{age:.1f}s 无新观察帧） | "
                   f"遥控/指令状态已过期 | "
                   f"有效包龄 {o['avld_ms']}ms（旧） | 峰值 {peak}")
            msg += " | F 跟随 / Q 退出"
            if w.error:
                msg += f" | 串口错误：{w.error}"
            return msg

        msg = (f"姿态 {'就绪' if o['rdy'] else '未就绪'} | "
               f"遥控 {'连接' if o['lk'] else '失联'} | "
               f"指令 {'有效' if o['vld'] else '失效'} | "
               f"有效包龄 {o['avld_ms']}ms | 峰值 {peak}")
        # 控制使能/目标放在统计区，状态栏保持单行不越界
        msg += " | F 跟随 / Q 退出"
        if w.error:
            msg += f" | 串口错误：{w.error}"
        return msg

    def _stat_text(self):
        """L400 统计区：累计采样/更新/失败/超时/跳拍/串口推迟与分项耗时"""
        w = self.worker
        s = w.last_stat
        if s is None:
            return f"统计：等待 L400S（1Hz）…  已收观察 {w.frames} 条"
        if s.get("cmiss") is not None:
            text = (f"统计  采样 n={s['n']}  姿态更新 eul={s['eul']}  "
                    f"读失败 f={s['f']}  坏包 bad={s['bad_pkt']}\n"
                    f"      整拍超期 miss={s['miss']}  控制超期 cmiss={s['cmiss']}  "
                    f"跳拍 skip={s['skip']}  串口推迟 txd={s['txd']}\n"
                    f"      控制完成 ctl={s['ctrl_max_us']}/{s['ctrl_peak_us']}us")
        else:
            text = (f"统计  采样 n={s['n']}  姿态更新 eul={s['eul']}  "
                    f"读失败 f={s['f']}  长间断 gap={s['gap']}  坏包 bad={s['bad_pkt']}\n"
                    f"      超时 miss={s['miss']}  跳拍 skip={s['skip']}  "
                    f"串口推迟 txd={s['txd']}  "
                    f"整拍 cyc={s['cyc_max_us']}/{s['cyc_peak_us']}us")
        if s.get("late_avg_us") is not None:
            text += (f"  迟到 late={s['late_avg_us']}/{s['late_max_us']}us"
                     f"  执行 exec={s['exec_avg_us']}/{s['exec_max_us']}us"
                     f"\n      收包 rx={s['rx_avg_us']}/{s['rx_max_us']}us"
                     f"  报告 tx={s['tx_avg_us']}/{s['tx_max_us']}us")
        o = w.last_obs or self.last_l400
        if (o is not None and o.get("ctrl_ok") is not None
                and o.get("tgt_att_deg100") is not None):
            ta = o["tgt_att_deg100"]
            tw = o["tgt_rate_q10"]
            text += (f"\n控制 {'使能' if o['ctrl_ok'] else '禁止'} 目标 "
                     f"横滚{ta[0] / 100.0:+.1f}° 俯仰{ta[1] / 100.0:+.1f}° "
                     f"偏航{q10_rad_s_to_dps(tw[2]):+.1f}°/s")
        elif o is not None and o.get("ctrl_ok") is not None:
            # 短报文只带使能位，不带目标姿态/角速度
            text += f"\n控制 {'使能' if o['ctrl_ok'] else '禁止'}"
        if o is not None and o.get("motor_armed") is not None:
            lock = L400_LOCK_NAMES.get(o["motor_lock"], str(o["motor_lock"]))
            text += (f"  电机 {'解锁' if o['motor_armed'] else '锁定(' + lock + ')'}"
                     f"  PWM 前/后/左/右="
                     f"{o['pwm_front']}/{o['pwm_rear']}/"
                     f"{o['pwm_left']}/{o['pwm_right']}")
        return text

    def _compose_texts(self):
        """生成状态栏与统计区文字（统计区只有 L400 模式才有）"""
        msg = self._status()
        stat_msg = self._stat_text() if self.stat_text is not None else None
        return msg, stat_msg

    def _apply_texts(self, msg, stat_msg):
        """把文字写到画布对象上，返回是否有变化；真正 blit 由调用方决定"""
        changed = False
        if msg != self._last_text:
            self._last_text = msg
            self.text.set_text(msg)
            changed = True
        if stat_msg is not None and stat_msg != self._last_stat_text:
            self._last_stat_text = stat_msg
            self.stat_text.set_text(stat_msg)
            changed = True
        return changed

    def _refresh(self):
        if not self.hist_t:
            # 没有数据时只更新一次状态栏/统计区，避免空转重画拖慢窗口
            msg, stat_msg = self._compose_texts()
            if self._apply_texts(msg, stat_msg):
                self._blit_dynamic(True)
            return

        # 没有新数据（例如已冻结）且文字没变：什么都不用画，事件循环保持空闲
        if not self._dirty:
            msg, stat_msg = self._compose_texts()
            if msg == self._last_text and (stat_msg is None
                                           or stat_msg == self._last_stat_text):
                return
            now = time.time()
            if now - self._text_ts < 1.0:   # 状态栏/统计区限到 1Hz
                return
            self._text_ts = now
            self._apply_texts(msg, stat_msg)
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
        msg, stat_msg = self._compose_texts()
        stat_changed = (self.stat_text is not None
                        and stat_msg != self._last_stat_text)
        if self._apply_texts(msg, stat_msg) and stat_changed:
            # 统计区是 1Hz 累计量，变化时立刻整图 blit；状态栏仍按 1Hz 节流，
            # 避免 avld 每 100ms 变一次就把 200Hz 窗口拖进 10Hz 全图重绘
            self._text_ts = 0.0

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
            values = [v for a in series for v in a if math.isfinite(v)]
            if values:
                lo = min(values)
                hi = max(values)
            else:
                lo, hi = -1.0, 1.0       # 旧镜像没有 q：第三张图先给默认量程
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
        """补画当前子图的三条曲线（图例在坐标框外，曲线不会压到它，无需重画）"""
        for line in line_set:
            ax_obj.draw_artist(line)

    def _blit_lines(self):
        """只补画曲线：恢复各子图背景 → 画曲线 → blit 子图区域（不碰状态栏）"""
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
                renderer = self.fig.canvas.get_renderer()
                self.text.draw(renderer)
                if self.stat_text is not None:
                    self.stat_text.draw(renderer)
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


def ensure_distinct_outputs(named_paths):
    """
    检查多个输出 CSV 是否指向同一文件。

    named_paths: [(界面显示名, 路径), ...]
    只做路径判断，不打开、不写入任何文件；发现重复直接退出，
    这样已有的 CSV 内容不会被截断。同时处理大小写、相对路径和硬链接。
    """
    seen = []
    for name, path in named_paths:
        real = os.path.normcase(os.path.realpath(os.path.abspath(path)))
        for other_name, other_path, other_real in seen:
            same = (real == other_real)
            if not same and os.path.exists(path) and os.path.exists(other_path):
                try:
                    same = os.path.samefile(path, other_path)
                except OSError:
                    same = False
            if same:
                sys.exit(f"错误：{name} 与 {other_name} 指向同一文件：{path}\n"
                         "请改用不同的输出路径；已有文件不会被修改。")
        seen.append((name, path, real))


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
    csv_path = make_attitude_csv_path(args.csv)
    stat_path = make_att_stat_path(args.stats_csv, csv_path)
    cat_path = f"{os.path.splitext(csv_path)[0]}_cat.csv"
    # 先检查三份输出，再打开串口/文件，避免把同一文件截断两次
    ensure_distinct_outputs([("--csv", csv_path),
                             ("--stats-csv", stat_path),
                             ("分项 CSV", cat_path)])

    if args.simulate:
        source = SimAttitudeSource()
        port_name = "模拟台架数据源"
    else:
        port_name = pick_port(args.port)
        source = SerialSource(port_name, args.baud)

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


def make_l400_csv_path(explicit):
    """L400 观察 CSV 默认文件名：l400_年月日_时分秒.csv"""
    if explicit:
        return explicit
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"l400_{stamp}.csv"


def make_l400_stat_path(explicit, data_csv):
    """L400 统计 CSV：--stats-csv 指定则用它，否则观察 CSV 名后加 _stats"""
    if explicit:
        return explicit
    base, _ = os.path.splitext(data_csv)
    return f"{base}_stats.csv"


def run_l400(args):
    """--l400：按 7d01167 联调固件的 L400/L400S 文本协议采集与显示"""
    csv_path = make_l400_csv_path(args.csv)
    stat_path = make_l400_stat_path(args.stats_csv, csv_path)
    # 先检查两份输出，再打开串口/文件，避免把同一文件截断两次
    ensure_distinct_outputs([("--csv", csv_path),
                             ("--stats-csv", stat_path)])

    if args.simulate:
        source = SimL400Source()
        port_name = "模拟 L400 数据源"
    else:
        port_name = pick_port(args.port)
        source = SerialSource(port_name, args.baud)

    sample_queue = queue.Queue(maxsize=2000)
    worker = L400Worker(source, csv_path, stat_path, sample_queue)

    print(f"数据源：{port_name}（L400 文本，10Hz 观察 + 1Hz 统计）")
    print(f"观察 CSV：{csv_path}")
    print(f"统计 CSV：{stat_path}")
    print("提示：g/c 是 Q10 rad/s，界面按 °/s 显示，不是欧拉角。")

    if args.no_plot:
        run_no_plot(worker, args.duration)
        print(f"观察 CSV：{csv_path}（{worker.frames} 条 L400）")
        print(f"统计 CSV：{stat_path}（{worker.stats} 条 L400S，"
              f"无法识别 {worker.bad} 行）")
        return

    worker.start()
    monitor = SensorMonitor(worker, csv_path, args.window, args.duration,
                            mode="l400")
    try:
        monitor.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        worker.stop()
        worker.join(timeout=2.0)
    print(f"观察 CSV：{csv_path}（{worker.frames} 条 L400）")
    print(f"统计 CSV：{stat_path}（{worker.stats} 条 L400S，"
          f"无法识别 {worker.bad} 行）")


def run(args):
    if args.l400:
        run_l400(args)
        return
    if args.attitude:
        run_attitude(args)
        return

    csv_path = make_csv_path(args.csv)
    perf_path = make_perf_csv_path(args.stats_csv, csv_path)
    # 先检查两份输出，再打开串口/文件，避免把同一文件截断两次
    ensure_distinct_outputs([("--csv", csv_path),
                             ("--stats-csv", perf_path)])

    if args.simulate:
        source = SimSource()
        port_name = "模拟数据源"
    else:
        port_name = pick_port(args.port)
        source = SerialSource(port_name, args.baud)

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
        description="Robin_24G_Car 六轴 / 台架姿态 / 200Hz 联调串口监视与记录")
    ap.add_argument("--list", action="store_true", help="列出所有串口后退出")
    ap.add_argument("--port", help="串口号，例如 COM12；不填则自动选择")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--csv", help="观察数据 CSV 路径；默认按模式和时间戳生成")
    ap.add_argument("--stats-csv",
                    help="统计 CSV 路径；默认在观察 CSV 名后加 _perf（六轴）或 _stats（文本模式）")
    ap.add_argument("--window", type=float, default=10.0, help="曲线显示窗口秒数（默认 10s）")
    ap.add_argument("--no-plot", action="store_true", help="不画图，只写 CSV")
    ap.add_argument("--duration", type=float, default=0, help="采集时长秒，0=一直采")
    ap.add_argument("--simulate", action="store_true",
                    help="用模拟数据源代替串口（无硬件自测界面/退出/CSV）")
    ap.add_argument("--acc-lsb-per-g", type=float, default=DEFAULT_ACC_LSB_PER_G,
                    help="加速度灵敏度（默认 2048 LSB/g，±16g）")
    ap.add_argument("--gyro-lsb-per-dps", type=float, default=DEFAULT_GYRO_LSB_PER_DPS,
                    help="陀螺灵敏度（默认 16.4 LSB/dps，±2000dps）")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--attitude", action="store_true",
                      help="按台架固件的文本协议采集（ATT/STA 行，画横滚/俯仰/相对偏航）")
    mode.add_argument("--flight-control", "--l400", dest="l400", action="store_true",
                      help="按 200Hz 飞控协议采集（FCQ/FCS；--l400 为兼容别名）")
    mode.add_argument("--arm-debug", action="store_true",
                      help="200Hz 飞控专用解锁诊断（默认 GUI；ADB/ADU/ADO/ADL/ADE）")
    mode.add_argument("--ground-diag", action="store_true",
                      help="地面 YAW 10Hz 诊断（四元数/偏航控制/PWM/振动四组曲线）")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return
    if args.arm_debug:
        from arm_debug_monitor import run_arm_debug
        return run_arm_debug(args)
    if args.ground_diag:
        from ground_diag_monitor import run_ground_diag
        return run_ground_diag(args, SerialSource, pick_port)
    run(args)


if __name__ == "__main__":
    main()
