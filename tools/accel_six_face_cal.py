#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
T-3488 加速度计六面校准工具（命令行交互，第一版）

用途
----
把飞机依次摆到 6 个静止姿态（+X / -X / +Y / -Y / +Z / -Z 朝下或朝上各一次），
工具从现有 0x269A 六轴帧里自动识别姿态、判断静止、采样，最后算出三轴零偏与
比例系数，供后续四元数姿态估计使用。

数据来源
--------
现有测量固件 115200-8-N1 的 0x269A 六轴帧（飞机 TX → USB 串口 RX，只收不发）。
帧里的加速度已做 5 点中值滤波，但**没有做六面校准**——中值滤波只是去尖峰，
本工具要标定的正是"零偏 + 比例误差"，两者是不同概念。

判定口径
--------
- 面识别：取绝对值最大的轴 + 符号；要求该轴接近 ±1g、另外两轴足够小；
- 静止：连续窗口内（默认 1.5s）三轴加速度标准差小，且三轴角速度都接近 0
  （防止"匀速转动"被误判为静止）；
- 采样：默认 5s，按设备时间戳计时；采样期间姿态改变、晃动、丢帧都作废重采；
- 只接受未解锁（flags 的 ARMED 位为 0）的样本；电机必须断开、飞控保持上锁。

输出
----
JSON（格式见 README / 本文件 write_result）：
  每轴 offset_lsb = (P + N) / 2
  每轴 scale      = (2 * LSB_PER_G) / (P - N)
  修正：corrected_lsb = (raw_lsb - offset_lsb) * scale
其中 P/N 是该轴正向/负向两面的平均计数。

用法
----
  python accel_six_face_cal.py --list
  python accel_six_face_cal.py --port COM3 --out accel_cal.json

  # 带 FRD 轴映射：界面、完成列表、--face 都用"机体 ±X/±Y/±Z 朝下"
  python accel_six_face_cal.py --port COM3 --axis-map ..\data\imu_axis_map_frd.json \
         --out ..\data\accel_cal.json

  # 单面补测：加载已有六面结果，只重采一面，其余五面保留
  python accel_six_face_cal.py --port COM3 --resume ..\data\accel_cal.json \
         --face=-Z --out ..\data\accel_cal_retest.json
  # 同上，但校验通过后直接覆盖原文件（不再生成新文件）
  python accel_six_face_cal.py --port COM3 --resume ..\data\accel_cal.json \
         --face=-Z --replace
  注意负号面要写成 --face=-Z（等号形式），否则 argparse 会把它当成选项。
  不带 --axis-map 时，面名/--face 是"传感器面"（原始读数：-Z 表示 az≈-1g）；
  带 --axis-map 时，面名/--face 是"机体摆法"（-Z 表示机体 -Z 朝下，即倒放）。
  FRD 轴映射与加速度校准是两个独立文件，本工具只读映射、分别输出两套参数。
"""

import argparse
import datetime
import json
import math
import os
import sys
import time
from collections import deque

from uart_proto import FrameParser
from calib_common import (SerialSource, list_ports, pick_port, unique_path,
                          load_axis_map, mean_std)

# ----------------------------------------------------------------------------
# 阈值集中定义（都在这里改；README 有说明）
# ----------------------------------------------------------------------------
CONFIG = {
    "lsb_per_g": 2048.0,        # ±16g 量程下的灵敏度（1g = 2048 LSB）
    "gyro_lsb_per_dps": 16.4,   # ±2000dps 下的灵敏度（只用于静止判定）

    "stable_sec": 1.5,          # 判定静止所需的连续稳定时长（设备时间戳，秒）
    "sample_sec": 5.0,          # 每个面的采样时长（秒）
    "min_samples": 200,         # 采样点数下限（5s@100Hz=500，留余量）

    "main_min_g": 0.90,         # 主轴分量下限（g）
    "main_max_g": 1.10,         # 主轴分量上限（g）
    "cross_max_g": 0.25,        # 另外两轴分量上限（g）
    "accel_std_max_g": 0.02,    # 静止判定：三轴加速度标准差上限（g）
    "gyro_max_dps": 5.0,        # 静止判定：三轴角速度绝对值上限（dps）

    "max_gap_s": 0.10,          # 相邻帧最大间隔（秒），超过即认为丢帧
    "stable_min_samples": 20,   # 判定静止前窗口内最少帧数

    "span_min_lsb": 3400.0,     # 正负面跨度合理范围（名义 2g = 4096 LSB）
    "span_max_lsb": 4800.0,
}

# flags 位定义（历史 0x269A 协议）：bit3 = 已解锁；
# 当前六轴校准固件固定发 0，解析器保留该位兼容旧记录。
FLAG_ARMED = 0x08

# 校准文件的格式标识（单面补测加载时校验）
CAL_FORMAT = "robin24g_accel_six_face_cal"
CAL_VERSION = 1
CAL_MODEL = "T-3488"
CAL_RANGE_G = 16

FACES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}

# 补测时给用户的面提示：这里的"面"指 UART 原始读数的主轴与符号
FACE_PROMPT = {
    "+X": "传感器 +X 读数约 +1g（ax ≈ +2048）",
    "-X": "传感器 -X 读数约 -1g（ax ≈ -2048）",
    "+Y": "传感器 +Y 读数约 +1g（ay ≈ +2048）",
    "-Y": "传感器 -Y 读数约 -1g（ay ≈ -2048）",
    "+Z": "传感器 +Z 读数约 +1g（az ≈ +2048）",
    "-Z": "传感器 -Z 读数约 -1g（az ≈ -2048，本机倒放）",
}


# ----------------------------------------------------------------------------
# 小工具
# ----------------------------------------------------------------------------
def face_axis(face):
    """'+X' → ('X', +1)；'-Y' → ('Y', -1)"""
    return face[1], (1 if face[0] == "+" else -1)


def classify(ax_g, ay_g, az_g, cfg):
    """
    用当前读数判断处于哪个面。
    返回 (面名 or None, 不合格原因)；面名形如 '+X'。
    """
    v = (ax_g, ay_g, az_g)
    axis = max(range(3), key=lambda k: abs(v[k]))
    main = v[axis]
    cross = [abs(v[k]) for k in range(3) if k != axis]
    name = ("+" if main > 0 else "-") + "XYZ"[axis]

    if abs(main) < cfg["main_min_g"]:
        return None, f"三轴读数都偏小（主轴 {main:+.2f}g），可能还在移动或没停稳"
    if abs(main) > cfg["main_max_g"]:
        return None, f"主轴 {main:+.2f}g 超过 1g，可能正在加速或手还没离开"
    if max(cross) > cfg["cross_max_g"]:
        return None, (f"当前接近 {name}，但机身仍倾斜"
                      f"（横向分量最大 {max(cross):.2f}g），请摆正")
    return name, ""


def cross_text(ax_g, ay_g, az_g):
    """显示两条横向分量（除主轴外的两轴），便于用户把面摆正"""
    v = (ax_g, ay_g, az_g)
    k = max(range(3), key=lambda i: abs(v[i]))
    names = ("ax", "ay", "az")
    return "横向 " + " ".join(f"{names[i]}={v[i]:+.2f}" for i in range(3) if i != k)


def solve_faces(faces, cfg):
    """
    用六个面的均值求解三轴零偏与比例：
        offset_lsb = (P + N) / 2
        scale      = 2 * lsb_per_g / (P - N)
    只取该轴在正/负面对应分量的均值；跨度超出合理范围记入 bad（需要重采该轴）。
    返回 (结果 dict, 需要重采的轴列表)。六面模式与单面补测共用。
    """
    res = {}
    bad = []
    for axis in "XYZ":
        p = faces.get("+" + axis)
        n = faces.get("-" + axis)
        if not p or not n:
            bad.append(axis)
            continue
        i = AXIS_INDEX[axis]
        pv = p["mean_lsb"][i]
        nv = n["mean_lsb"][i]
        span = pv - nv
        if not (cfg["span_min_lsb"] <= span <= cfg["span_max_lsb"]):
            bad.append(axis)
            continue
        res[axis] = {
            "offset_lsb": (pv + nv) / 2.0,
            "scale": (2.0 * cfg["lsb_per_g"]) / span,
            "span_lsb": span,
            "positive_mean_lsb": pv,
            "negative_mean_lsb": nv,
        }
    return res, bad


# ----------------------------------------------------------------------------
# 可选的 FRD 轴映射：把"传感器面"与"机体摆法"互相转换
# ----------------------------------------------------------------------------
# 机体摆法的物理提示（用于补测横幅和完成提示）
BODY_HINT = {
    "+X": "机头朝下", "-X": "机尾朝下",
    "+Y": "右侧朝下", "-Y": "左侧朝下",
    "+Z": "正放，机腹朝下", "-Z": "倒放，机腹朝上",
}


def face_unit(face):
    """'+X' → [1,0,0]；'-Z' → [0,0,-1]"""
    v = [0.0, 0.0, 0.0]
    v[AXIS_INDEX[face[1]]] = 1.0 if face[0] == "+" else -1.0
    return v


def face_of_vec(vec):
    """取绝对值最大的分量 → 面名"""
    k = max(range(3), key=lambda i: abs(vec[i]))
    return ("+" if vec[k] > 0 else "-") + "XYZ"[k]


def body_to_sensor_face(matrix, body_face):
    """
    机体某轴朝下 → 传感器读数落在哪个面。
    静止时比力（加速度读数）指向“上”，方向与朝下的轴相反：
        a_frd = -b（b 是朝下的机体轴），a_sensor = R^T · a_frd
    """
    b = face_unit(body_face)
    a_frd = [-x for x in b]
    a_sensor = [sum(matrix[r][c] * a_frd[r] for r in range(3)) for c in range(3)]  # R^T
    return face_of_vec(a_sensor)


def sensor_to_body_face(matrix, sensor_face):
    """传感器读数落在某面 → 机体哪个轴朝下：a_frd = R·a_sensor，朝下轴 = -a_frd"""
    a_sensor = face_unit(sensor_face)
    a_frd = [sum(matrix[r][c] * a_sensor[c] for c in range(3)) for r in range(3)]
    return face_of_vec([-x for x in a_frd])


def frd_params(sensor_cal, matrix):
    """
    传感器系参数 → FRD 系参数：
      零偏按映射重排并变号（o_frd = R·o_sensor）
      比例系数只重排（不取反）
    """
    res = {}
    for r, axis in enumerate("XYZ"):
        c = next(j for j in range(3) if matrix[r][j] != 0)
        sign = matrix[r][c]
        src = "XYZ"[c]
        s = sensor_cal[src]
        res[axis] = {
            "offset_lsb": sign * s["offset_lsb"],
            "scale": s["scale"],
            "span_lsb": s["span_lsb"],
            "from_sensor_axis": src,
            "sign": sign,
        }
    return res


# ----------------------------------------------------------------------------
# 校准状态机（不依赖串口，方便用模拟数据测试）
# ----------------------------------------------------------------------------
class Calibrator:
    def __init__(self, cfg=None, only_face=None, preset_faces=None, face_name=None):
        self.cfg = dict(CONFIG)
        if cfg:
            self.cfg.update(cfg)

        # 六面模式：只有 only_face 为空、preset_faces 为空
        # 补测模式：preset_faces 是先加载的另外五面，only_face 是本次要重采的面
        self.only_face = only_face
        self.faces = dict(preset_faces) if preset_faces else {}
        # 显示名：默认就是传感器面名；带轴映射时换成机体摆法名
        self.face_name = face_name or (lambda f: f)

        self.state = "wait"      # wait / sample
        self.target = None       # 正在采样的面
        self.samples = []        # 采样中的加速度读数 [(ax,ay,az) 单位 g]
        self.sample_start = None

        self.window = deque()    # 静止判定窗口 [(t, face, ax..gz)]
        self.last_t = None
        self.last_seq = None
        self.skipped = 0         # 因丢帧/跳变作废的次数
        self.aborted = 0         # 因晃动/姿态改变/波动过大作废的次数
        self.hint_face = None    # 已完成面的提示去重
        self.wrong_face = None   # 补测时"摆错面"的提示去重
        self.pending_hint = ""   # 上一次的不合格原因（状态行显示）
        self.cur = None          # 最近一帧 (t, face, ax, ay, az)，状态行用
        self.done = False

    # ---------------- 显示名（带轴映射时显示机体摆法） ----------------
    def _name(self, face):
        return self.face_name(face) if face else face

    def _missing(self):
        left = [f for f in FACES if f not in self.faces]
        return "、".join(self._name(f) for f in left) if left else "无"

    # ---------------- 对外接口 ----------------
    def feed(self, fr):
        """喂一帧六轴数据，返回要打印的提示列表"""
        if self.done:
            return []
        msgs = []
        t = fr["t_ms"] / 1000.0
        cfg = self.cfg

        # 丢帧/时间跳变：当前面作废（已完成的面保留）
        if self.last_t is not None:
            gap = t - self.last_t
            step = (fr["seq"] - self.last_seq) & 0xFF
            if gap > cfg["max_gap_s"] or step != 1:
                if self.state == "sample":
                    msgs.append(f"检测到丢帧/时间跳变（间隔 {gap*1000:.0f}ms），{self._name(self.target)} 重新采")
                    self._restart_face()
                self.skipped += 1
        self.last_t = t
        self.last_seq = fr["seq"]

        # 已解锁：拒绝采样（电机可能转起来，数据也不能用）。
        # 注意要连采样窗口一起清掉：解锁前采的数据不能再接着用。
        if fr["flags"] & FLAG_ARMED:
            if self.state == "sample" or self.samples:
                msgs.append("检测到已解锁，已清空本面采样窗口；请保持飞控上锁、电机断开")
            self._restart_face()
            return msgs

        ax = fr["ax"] / cfg["lsb_per_g"]
        ay = fr["ay"] / cfg["lsb_per_g"]
        az = fr["az"] / cfg["lsb_per_g"]
        gx = fr["gx"] / cfg["gyro_lsb_per_dps"]
        gy = fr["gy"] / cfg["gyro_lsb_per_dps"]
        gz = fr["gz"] / cfg["gyro_lsb_per_dps"]
        face, reason = classify(ax, ay, az, cfg)
        self.pending_hint = reason
        self.cur = (t, face, ax, ay, az)

        if self.state == "sample":
            return msgs + self._feed_sampling(t, face, (ax, ay, az, gx, gy, gz))
        return msgs + self._feed_wait(t, face, (ax, ay, az, gx, gy, gz))

    def status(self):
        """一行状态：当前姿态 / 读数 / 进度 / 还缺哪些面"""
        face = self.cur[1] if self.cur else None
        ax, ay, az = (self.cur[2], self.cur[3], self.cur[4]) if self.cur else (0.0, 0.0, 0.0)
        if self.state == "sample":
            t = self.cur[0] if self.cur else 0.0
            done_s = max(0.0, t - (self.sample_start or t))
            state = f"正在采样 {self._name(self.target)} {done_s:.1f}/{self.cfg['sample_sec']:.0f}s"
        elif face:
            stable = (self.window[-1][0] - self.window[0][0]) if len(self.window) >= 2 else 0.0
            state = (f"请停稳 {min(stable, self.cfg['stable_sec']):.1f}/"
                     f"{self.cfg['stable_sec']:.1f}s")
        else:
            state = self.pending_hint or "请把机身摆到某个面"
        head = (f"目标 {self._name(self.only_face)} | 当前 {self._name(face) or '--'}" if self.only_face
                else f"姿态 {self._name(face) or '--'}")
        return (f"{head} | a=({ax:+.2f},{ay:+.2f},{az:+.2f})g | {cross_text(ax, ay, az)} | "
                f"{state} | 还缺: {self._missing()}")

    # ---------------- 内部：等待/采样 ----------------
    def _reset_window(self):
        self.window.clear()

    def _restart_face(self):
        """当前面作废：回到等待状态"""
        self.state = "wait"
        self.target = None
        self.samples = []
        self.sample_start = None
        self._reset_window()

    def _feed_wait(self, t, face, sample):
        cfg = self.cfg
        ax, ay, az, gx, gy, gz = sample

        if face is None:
            # 姿态不合格：窗口作废，状态行显示具体原因
            self._reset_window()
            return []
        # 补测模式：只接受目标面，摆到其他面时提示调整（提示去重）
        if self.only_face and face != self.only_face:
            if self.wrong_face != face:
                self.wrong_face = face
                return [f"当前是 {self._name(face)}，本工具只补测 {self._name(self.only_face)}；请调整摆放"]
            self._reset_window()
            return []
        self.wrong_face = None
        if face in self.faces:
            if self.hint_face != face:
                self.hint_face = face
                return [f"{self._name(face)} 已完成，请翻到未完成的面：{self._missing()}"]
            self._reset_window()
            return []
        self.hint_face = None

        # 同一面的连续窗口；换面则重新开始
        if self.window and self.window[-1][1] != face:
            self.window.clear()
        self.window.append((t, face, ax, ay, az, gx, gy, gz))

        # 丢掉超出稳定时长 + 0.3s 的旧点
        keep_from = t - cfg["stable_sec"] - 0.3
        while self.window and self.window[0][0] < keep_from:
            self.window.popleft()

        if self._is_stable():
            self.state = "sample"
            self.target = face
            self.samples = []
            self.sample_start = t
            return [f"{self._name(face)} 已停稳，开始采样 {cfg['sample_sec']:.0f}s …"]
        return []

    def _is_stable(self):
        """连续窗口是否满足静止条件"""
        cfg = self.cfg
        if len(self.window) < cfg["stable_min_samples"]:
            return False
        t0 = self.window[0][0]
        t1 = self.window[-1][0]
        if (t1 - t0) < cfg["stable_sec"]:
            return False

        axes = [[s[2] for s in self.window],
                [s[3] for s in self.window],
                [s[4] for s in self.window]]
        for vals in axes:
            _, std = mean_std(vals)
            if std > cfg["accel_std_max_g"]:
                return False

        for k in (5, 6, 7):      # gx, gy, gz
            if max(abs(s[k]) for s in self.window) > cfg["gyro_max_dps"]:
                return False
        return True

    def _feed_sampling(self, t, face, sample):
        cfg = self.cfg
        ax, ay, az, gx, gy, gz = sample

        if face != self.target:
            target = self.target
            self.aborted += 1
            self._restart_face()
            return [f"姿态改变（{self._name(target)} → {self._name(face) or '未识别'}），本面重新采"]

        # 采样中仍要求静止：角速度超限说明在转动/晃动，本面作废
        if max(abs(gx), abs(gy), abs(gz)) > cfg["gyro_max_dps"]:
            target = self.target
            self.aborted += 1
            self._restart_face()
            return [f"{self._name(target)} 采样中检测到转动/晃动，本面重新采"]

        self.samples.append((ax, ay, az))
        if len(self.samples) >= cfg["min_samples"] and (t - self.sample_start) >= cfg["sample_sec"]:
            return self._finish_face(t)
        return []

    def _finish_face(self, t):
        cfg = self.cfg
        face = self.target
        n = len(self.samples)
        means, stds = [], []
        for k in range(3):
            vals = [s[k] for s in self.samples]
            m, sd = mean_std(vals)
            means.append(m * cfg["lsb_per_g"])       # 记录成 LSB
            stds.append(sd * cfg["lsb_per_g"])
        dur = t - self.sample_start

        # 结束前再检查整段采样的波动：判稳只看窗口，仍可能整段缓慢漂移
        worst = max(stds) / cfg["lsb_per_g"]
        if worst > cfg["accel_std_max_g"]:
            self.aborted += 1
            self._restart_face()
            return [f"{self._name(face)} 采样段波动过大（最大 {worst:.4f}g > "
                    f"{cfg['accel_std_max_g']:.3f}g），本面重新采"]

        self.faces[face] = {
            "mean_lsb": means,
            "std_lsb": stds,
            "samples": n,
            "duration_s": dur,
        }
        self._restart_face()

        msg = [f"{self._name(face)} 完成：均值({means[0]:+.1f},{means[1]:+.1f},{means[2]:+.1f}) LSB，"
               f"波动(±{stds[0]:.1f},±{stds[1]:.1f},±{stds[2]:.1f})，{n} 点 / {dur:.1f}s"]
        if len(self.faces) == len(FACES):
            msg += self._finish_all()
        else:
            msg.append(f"还缺：{self._missing()}")
        return msg

    # ---------------- 六面齐全后的求解与检查 ----------------
    def solve(self):
        """按 (P+N)/2 与 2*2048/(P-N) 求解；返回 (结果 dict, 需要重采的轴列表)"""
        return solve_faces(self.faces, self.cfg)

    def _finish_all(self):
        res, bad = self.solve()
        if bad:
            if self.only_face:
                # 补测模式：只重新采目标面，其余已加载的面原样保留
                self.faces.pop(self.only_face, None)
                self.state = "wait"
                return [f"跨度检查不通过（{'、'.join(bad)}）；请重新补测 "
                        f"{self._name(self.only_face)}（其余面保留不动）"]
            # 六面模式：把该轴的正负面作废，要求重采
            for axis in bad:
                self.faces.pop("+" + axis, None)
                self.faces.pop("-" + axis, None)
            self.state = "wait"
            return [f"跨度检查不通过（{'、'.join(bad)}），请重采这些轴的正负面："
                    f"{self._missing()}"]
        self.result = res
        self.done = True
        return ["六个面全部完成，校准参数已求解"]

    # ---------------- 修正 ----------------
    def correct(self, raw_lsb):
        """按解出的参数修正一组三轴原始计数"""
        out = []
        for i, axis in enumerate("XYZ"):
            c = self.result[axis]
            out.append((raw_lsb[i] - c["offset_lsb"]) * c["scale"])
        return out


# ----------------------------------------------------------------------------
# 结果输出
# ----------------------------------------------------------------------------
def load_calibration(path, cfg):
    """
    读取已有的六面校准 JSON，校验格式版本、型号、量程、LSB/g 和六面记录。
    任何不兼容都在这里直接退出（发生在打开串口之前），不会写任何文件。
    返回 (faces, 原始 JSON, 旧的三轴结果)。
    """
    if not os.path.exists(path):
        sys.exit(f"找不到校准文件：{path}")
    try:
        with open(path, encoding="utf-8") as fp:
            data = json.load(fp)
    except json.JSONDecodeError as exc:
        sys.exit(f"校准文件不是合法 JSON：{path}\n  原因：{exc}")

    def bad(msg):
        sys.exit(f"校准文件不可用：{path}\n  原因：{msg}")

    if data.get("format") != CAL_FORMAT:
        bad(f"format 应为 {CAL_FORMAT!r}，实际 {data.get('format')!r}")
    if data.get("version") != CAL_VERSION:
        bad(f"版本应为 {CAL_VERSION}，实际 {data.get('version')!r}")
    sensor = data.get("sensor") or {}
    if sensor.get("model") != CAL_MODEL:
        bad(f"传感器型号应为 {CAL_MODEL!r}，实际 {sensor.get('model')!r}")
    if sensor.get("range_g") != CAL_RANGE_G:
        bad(f"量程应为 ±{CAL_RANGE_G}g，实际 {sensor.get('range_g')!r}")
    lsb = sensor.get("lsb_per_g")
    if not isinstance(lsb, (int, float)) or abs(float(lsb) - cfg["lsb_per_g"]) > 0.5:
        bad(f"LSB/g 应为 {cfg['lsb_per_g']}，实际 {lsb!r}（确实不同请用 --acc-lsb-per-g 指定）")

    faces_raw = data.get("faces")
    if not isinstance(faces_raw, dict):
        bad("缺少 faces 记录")
    missing = [f for f in FACES if f not in faces_raw]
    if missing:
        bad(f"缺少面记录：{'、'.join(missing)}")
    faces = {}
    for face in FACES:
        rec = faces_raw[face]
        try:
            mean = [float(v) for v in rec["mean_lsb"]]
            std = [float(v) for v in rec["std_lsb"]]
            samples = int(rec["samples"])
            dur = float(rec["duration_s"])
        except (KeyError, TypeError, ValueError):
            bad(f"{face} 面记录不完整（需要 mean_lsb/std_lsb/samples/duration_s）")
        if len(mean) != 3 or len(std) != 3:
            bad(f"{face} 面的三轴数据不是 3 个分量")
        if samples <= 0 or dur <= 0:
            bad(f"{face} 面的样本数或时长无效（{samples}, {dur}）")
        faces[face] = {"mean_lsb": mean, "std_lsb": std,
                       "samples": samples, "duration_s": dur}

    # 用已有数据先解一遍，跨度不合理说明原文件本身就有问题
    old_result, bad_axes = solve_faces(faces, cfg)
    if bad_axes:
        bad(f"已有数据的跨度不合理（{'、'.join(bad_axes)}），请先用完整六面模式重测")
    return faces, data, old_result


def write_result(path, cal, build_id=None, axis_map=None, body_faces=None):
    """
    axis_map: {"source": 路径, "matrix": 3×3}，给了就同时写 FRD 参数；
    body_faces: {机体面: 传感器面}，说明 --face 用的机体摆法怎么对应原始面。
    """
    cfg = cal.cfg
    data = {
        "format": CAL_FORMAT,
        "version": CAL_VERSION,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "sensor": {
            "model": CAL_MODEL,
            "range_g": CAL_RANGE_G,
            "lsb_per_g": cfg["lsb_per_g"],
        },
        "gravity": {
            "local_mps2": 9.788,
            "note": "广州本地重力约 9.788 m/s²（标准 9.80665）；标定以 1g=2048LSB 为基准，"
                    "两者差 0.19%，对拟合无影响，只在模长验证时注意",
        },
        "faces": {},
        "calibration": {},
        "checks": {},
    }
    for face in FACES:
        rec = cal.faces[face]
        data["faces"][face] = {
            "mean_lsb": [round(v, 3) for v in rec["mean_lsb"]],
            "std_lsb": [round(v, 3) for v in rec["std_lsb"]],
            "samples": rec["samples"],
            "duration_s": round(rec["duration_s"], 3),
        }
    for axis in "XYZ":
        c = cal.result[axis]
        data["calibration"][axis] = {
            "offset_lsb": round(c["offset_lsb"], 3),
            "scale": round(c["scale"], 6),
            "span_lsb": round(c["span_lsb"], 3),
            "positive_mean_lsb": round(c["positive_mean_lsb"], 3),
            "negative_mean_lsb": round(c["negative_mean_lsb"], 3),
        }
    # 明确标出 calibration 用的是传感器坐标（旧文件没有这个键，读取时按传感器处理）
    data["calibration_frame"] = "sensor"
    if axis_map:
        data["axis_map"] = {
            "source": os.path.abspath(axis_map["source"]),
            "matrix_sensor_to_frd": axis_map["matrix"],
            "convention": "v_frd = R * v_sensor（行=FRD X/Y/Z，列=传感器 X/Y/Z）",
        }
        data["faces_frame"] = "sensor"
        data["body_faces"] = dict(body_faces or {})
        frd = frd_params(cal.result, axis_map["matrix"])
        data["calibration_frd"] = {
            axis: {
                "offset_lsb": round(frd[axis]["offset_lsb"], 3),
                "scale": round(frd[axis]["scale"], 6),
                "span_lsb": round(frd[axis]["span_lsb"], 3),
                "from_sensor_axis": frd[axis]["from_sensor_axis"],
                "sign": frd[axis]["sign"],
            } for axis in "XYZ"
        }
    data["checks"] = {
        "span_min_lsb": cfg["span_min_lsb"],
        "span_max_lsb": cfg["span_max_lsb"],
        "all_faces_complete": True,
        "skipped_frames": cal.skipped,
        "aborted_segments": cal.aborted,
        "firmware_build": f"0x{build_id:08X}" if build_id else None,
    }
    # 原子写：先写临时文件再改名，避免写一半把原文件弄坏（--replace 时尤其重要）
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    os.replace(tmp_path, path)
    return data


def print_summary(cal, data):
    """校准结果与"修正前 / 修正后"读数摘要（传感器坐标 + 可选的 FRD 坐标）"""
    cfg = cal.cfg
    print("\n================ 六面校准结果 ================")
    print("传感器坐标（calibration，原始读数所在坐标系）：")
    print(f"  {'轴':<3}{'offset(LSB)':>14}{'scale':>12}{'跨度(LSB)':>12}")
    for axis in "XYZ":
        c = data["calibration"][axis]
        print(f"  {axis:<3}{c['offset_lsb']:>14.1f}{c['scale']:>12.5f}{c['span_lsb']:>12.1f}")

    if "calibration_frd" in data:
        print("\n机体 FRD 坐标（calibration_frd，R × 传感器；零偏随映射变号，比例只重排）：")
        print(f"  {'轴':<3}{'offset(LSB)':>14}{'scale':>12}{'来源':>10}")
        for axis in "XYZ":
            c = data["calibration_frd"][axis]
            print(f"  {axis:<3}{c['offset_lsb']:>14.1f}{c['scale']:>12.5f}"
                  f"{c['sign']:+d}*{c['from_sensor_axis']:>4}")

    print("\n各面修正前后的加速度模长（应为 1.000 g）：")
    for face in FACES:
        rec = cal.faces[face]
        raw = rec["mean_lsb"]
        corr = cal.correct(raw)              # 修正后仍是 LSB
        before = math.sqrt(sum((v / cfg["lsb_per_g"]) ** 2 for v in raw))
        after = math.sqrt(sum((v / cfg["lsb_per_g"]) ** 2 for v in corr))
        print(f"  {face:<3} 修正前 {before:.4f} g → 修正后 {after:.4f} g")

    print("\n说明：这 6 个面同时参与了拟合，模长接近 1g 只说明自洽，不等于任意姿态都准。")
    print("      请另外摆几种静止斜放姿态，用修正后的三轴读数算模长，应在 1.000±0.005 g 内；")
    print("      若明显偏离，说明某个面没摆正或采样时晃动，重采对应面即可。")


def print_retest_summary(old_faces, cal, old_result, face, axis_map=None):
    """单面补测：打印目标面的旧/新均值、标准差，以及受影响轴的旧/新参数"""
    axis = face[1]
    o, n = old_faces[face], cal.faces[face]

    def fmt(rec):
        m, s = rec["mean_lsb"], rec["std_lsb"]
        return (f"均值({m[0]:+.1f},{m[1]:+.1f},{m[2]:+.1f}) LSB  "
                f"标准差(±{s[0]:.1f},±{s[1]:.1f},±{s[2]:.1f})  "
                f"{rec['samples']} 点 / {rec['duration_s']:.2f}s")

    print("\n============ 单面补测结果 ============")
    print(f"传感器面 {face}：{FACE_PROMPT[face]}")
    print(f"  旧：{fmt(o)}")
    print(f"  新：{fmt(n)}")

    oc, nc = old_result[axis], cal.result[axis]
    print(f"\n传感器 {axis} 轴（受本次补测影响）：")
    print(f"  offset  {oc['offset_lsb']:+9.2f} → {nc['offset_lsb']:+9.2f} LSB")
    print(f"  跨度    {oc['span_lsb']:9.1f} → {nc['span_lsb']:9.1f} LSB")
    print(f"  scale   {oc['scale']:9.6f} → {nc['scale']:9.6f}")

    print("\n传感器其余两轴（应保持不变）：")
    for other in "XYZ":
        if other == axis:
            continue
        a, b = old_result[other], cal.result[other]
        print(f"  {other} 轴 offset {a['offset_lsb']:+9.2f} → {b['offset_lsb']:+9.2f} LSB，"
              f"scale {a['scale']:.6f} → {b['scale']:.6f}")

    if axis_map:
        of = frd_params(old_result, axis_map["matrix"])
        nf = frd_params(cal.result, axis_map["matrix"])
        print("\n机体 FRD 参数（同样受影响的是补测面对应的 FRD 轴）：")
        for a in "XYZ":
            print(f"  {a} 轴 offset {of[a]['offset_lsb']:+9.2f} → {nf[a]['offset_lsb']:+9.2f} LSB，"
                  f"scale {of[a]['scale']:.6f} → {nf[a]['scale']:.6f}")

    print("\n说明：以上都是拟合用数据（六个面同时参与），换一面后自然吻合，不能当作精度指标；")
    print("      最终精度请另外摆几种静止斜放姿态，用修正后的三轴读数算模长验证"
          "（应在 1.000±0.005 g 内）。")


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def run(args):
    if args.list:
        list_ports()
        return 0

    cfg = dict(CONFIG)
    if args.stable is not None:
        cfg["stable_sec"] = args.stable
    if args.sample is not None:
        cfg["sample_sec"] = args.sample
    if args.acc_lsb_per_g is not None:
        cfg["lsb_per_g"] = args.acc_lsb_per_g

    # ---- 可选的 FRD 轴映射：给了就全程用"机体摆法" ---
    axis_map = None
    body_to_sensor = {}
    sensor_to_body = {}
    names = {}
    if args.axis_map:
        matrix, _map_json = load_axis_map(args.axis_map)
        body_to_sensor = {b: body_to_sensor_face(matrix, b) for b in FACES}
        if len(set(body_to_sensor.values())) != len(FACES):
            sys.exit("轴映射不合法：六个机体摆法没有一一对应到六个传感器面")
        sensor_to_body = {sf: bf for bf, sf in body_to_sensor.items()}
        names = {sf: f"机体 {bf} 朝下" for sf, bf in sensor_to_body.items()}
        axis_map = {"source": args.axis_map, "matrix": matrix}

    # ---- 单面补测的参数检查与加载：全部在打开串口之前完成 ----
    retest = bool(args.resume)
    if retest != bool(args.face):
        sys.exit("--resume 与 --face 必须同时使用：--resume 指定已有校准文件，"
                 "--face 指定要重采的面（负号面写 --face=-Z）")
    if args.replace:
        if not retest:
            sys.exit("--replace 只能和 --resume/--face 一起使用（单面补测时覆盖原文件）")
        if args.out:
            sys.exit("--replace 与 --out 只能二选一：--replace 直接覆盖 --resume 指定的文件")
    old_faces = old_result = None
    preset = None
    target = None
    target_body = None
    if retest:
        if args.face not in FACES:
            sys.exit(f"--face 只能是 {'、'.join(FACES)}；负号面请写成 --face=-Z")
        target_body = args.face
        # 带映射时 --face 是机体摆法；按映射换算出旧文件里对应的传感器面
        target = body_to_sensor[target_body] if axis_map else target_body
        old_faces, old_json, old_result = load_calibration(args.resume, cfg)
        if axis_map and "axis_map" in old_json:
            old_matrix = (old_json["axis_map"] or {}).get("matrix_sensor_to_frd")
            if old_matrix and old_matrix != axis_map["matrix"]:
                print("注意：已有结果文件记录的映射与本次 --axis-map 不同；"
                      "faces 仍按传感器面替换，FRD 参数按本次映射重算")
        preset = {f: r for f, r in old_faces.items() if f != target}

    port = pick_port(args.port)
    if retest:
        title = names.get(target, target) if axis_map else target
        print("=" * 68)
        print(f"T-3488 加速度计单面补测：{title}")
        if axis_map:
            print(f"  --face 坐标系：机体 FRD（经 {os.path.basename(args.axis_map)} 映射）")
            print(f"  机体摆法：{title}（{BODY_HINT[target_body]}）")
            print(f"  对应传感器面：{target}（{FACE_PROMPT[target]}）")
        else:
            print("  --face 坐标系：传感器（原始计数）")
            print(f"  面提示：{FACE_PROMPT[target]}")
        print(f"  已有结果：{os.path.abspath(args.resume)}")
        print("  只重采这一面，其余五面沿用原值")
        print("  1) 请先关闭串口监视器，释放 COM 口")
        print("  2) 电机断开，飞控保持上锁（不要解锁）")
        print(f"  3) 摆好后停稳 {cfg['stable_sec']:.1f}s 再采样 {cfg['sample_sec']:.0f}s")
        if args.replace:
            print(f"  校验通过后直接覆盖：{os.path.abspath(args.resume)}")
        print("=" * 68)
    else:
        print("=" * 68)
        print("T-3488 加速度计六面校准")
        if axis_map:
            print(f"  轴映射：{os.path.abspath(args.axis_map)}")
            print(f"  矩阵  ：{axis_map['matrix']}（v_frd = R × v_sensor）")
            print("  面名都是机体摆法：" + "、".join(
                f"{b}={BODY_HINT[b]}" for b in ("+Z", "-Z", "+X", "-X", "+Y", "-Y")))
        else:
            print("  --face / 面名使用传感器坐标（不加 --axis-map 时的默认）")
        print("  1) 请先关闭串口监视器，释放 COM 口")
        print("  2) 电机断开，飞控保持上锁（不要解锁）")
        print("  3) 依次把飞机摆到 6 个面，每个面停稳后保持不动")
        print(f"  4) 顺序随意；每面需稳定 {cfg['stable_sec']:.1f}s，再采样 {cfg['sample_sec']:.0f}s")
        print("=" * 68)

    src = SerialSource(port, args.baud)
    parser = FrameParser()
    cal = Calibrator(cfg, only_face=target, preset_faces=preset,
                     face_name=(lambda f: names.get(f, f)) if axis_map else None)
    last_status = 0.0
    last_print = 0.0
    last_perf = None

    try:
        while not cal.done:
            data = src.read()
            for kind, fr in parser.feed(data):
                if kind == "perf":
                    last_perf = fr          # 只记录固件构建标识，不参与校准
                    continue
                for msg in cal.feed(fr):
                    if sys.stdout.isatty():
                        print("\r" + " " * 110 + "\r" + msg)
                    else:
                        print(msg, flush=True)
            now = time.time()
            if now - last_status >= 0.2:
                last_status = now
                line = cal.status()
                if sys.stdout.isatty():
                    print("\r" + line.ljust(110), end="", flush=True)
                elif now - last_print >= 2.0:
                    last_print = now
                    print(line, flush=True)
    except KeyboardInterrupt:
        if retest:
            print("\n已中断，未生成新文件；原校准文件未改动")
        else:
            print("\n已中断，未生成校准文件（已完成的面不会保存）")
        return 1
    finally:
        src.close()

    if not cal.done:
        return 1

    build_id = last_perf["build_id"] if last_perf else None
    if retest and args.replace:
        # 直接覆盖原文件：不做 unique_path，失败时（校验不过/中断）根本走不到这里
        out_path = args.resume
    elif retest:
        out_path = unique_path(args.out or
                               f"accel_cal_retest_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    else:
        out_path = unique_path(args.out or
                               f"accel_cal_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    data = write_result(out_path, cal, build_id, axis_map=axis_map,
                        body_faces=body_to_sensor if axis_map else None)
    print()
    if retest:
        print_retest_summary(old_faces, cal, old_result, target, axis_map)
    print_summary(cal, data)
    if retest and args.replace:
        print(f"\n已覆盖原校准文件：{os.path.abspath(out_path)}")
    else:
        print(f"\n校准文件：{os.path.abspath(out_path)}")
        if retest:
            print(f"原文件未改动：{os.path.abspath(args.resume)}")
    print("后续：把 calibration（传感器）或 calibration_frd（机体 FRD）里的参数"
          "写进固件即可；固件接入属于后续任务。")
    return 0


def main():
    ap = argparse.ArgumentParser(description="T-3488 加速度计六面校准工具")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", help="串口号，例如 COM3；不填则自动选择")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--out", help="校准结果 JSON 路径；默认按时间戳生成，同名不覆盖")
    ap.add_argument("--stable", type=float, help="静止判定时长秒（默认 1.5）")
    ap.add_argument("--sample", type=float, help="每面采样时长秒（默认 5.0）")
    ap.add_argument("--acc-lsb-per-g", type=float, help="加速度灵敏度（默认 2048）")
    ap.add_argument("--resume", help="已有六面校准 JSON（单面补测用，需与 --face 同时给出）")
    ap.add_argument("--face",
                    help="要重采的面：+X/-X/+Y/-Y/+Z/-Z；负号面必须写 --face=-Z"
                         "（带 --axis-map 时按机体摆法解释，否则按传感器面）")
    ap.add_argument("--axis-map",
                    help="传感器→FRD 映射 JSON；给了以后面名与 --face 都用机体摆法")
    ap.add_argument("--replace", action="store_true",
                    help="单面补测时校验通过后直接覆盖 --resume 指定的文件（与 --out 互斥）")
    args = ap.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
