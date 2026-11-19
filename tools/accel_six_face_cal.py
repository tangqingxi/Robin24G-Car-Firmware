#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
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

# flags 位定义（固件 FlightCtrl.h 7.12）：bit3 = 已解锁
FLAG_ARMED = 0x08

FACES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


# ----------------------------------------------------------------------------
# 小工具
# ----------------------------------------------------------------------------
def face_axis(face):
    """'+X' → ('X', +1)；'-Y' → ('Y', -1)"""
    return face[1], (1 if face[0] == "+" else -1)


def missing_text(done):
    """还缺哪些面"""
    left = [f for f in FACES if f not in done]
    return "、".join(left) if left else "无"


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


def mean_std(vals):
    """均值与总体标准差（纯标准库，避免引入 numpy）"""
    n = len(vals)
    if n == 0:
        return 0.0, 0.0
    m = sum(vals) / n
    var = sum((x - m) ** 2 for x in vals) / n
    return m, math.sqrt(var)


# ----------------------------------------------------------------------------
# 校准状态机（不依赖串口，方便用模拟数据测试）
# ----------------------------------------------------------------------------
class Calibrator:
    def __init__(self, cfg=None):
        self.cfg = dict(CONFIG)
        if cfg:
            self.cfg.update(cfg)

        self.faces = {}          # 面名 -> {mean_lsb, std_lsb, samples, duration_s}
        self.state = "wait"      # wait / sample
        self.target = None       # 正在采样的面
        self.samples = []        # 采样中的加速度读数 [(ax,ay,az) 单位 g]
        self.sample_start = None

        self.window = deque()    # 静止判定窗口 [(t, face, ax..gz)]
        self.last_t = None
        self.last_seq = None
        self.skipped = 0         # 因丢帧/跳变作废的次数
        self.aborted = 0         # 因晃动/姿态改变作废的次数
        self.hint_face = None    # 已完成面的提示去重
        self.pending_hint = ""   # 上一次的不合格原因（状态行显示）
        self.cur = None          # 最近一帧 (t, face, ax, ay, az)，状态行用
        self.done = False

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
                    msgs.append(f"检测到丢帧/时间跳变（间隔 {gap*1000:.0f}ms），{self.target} 重新采")
                    self._restart_face()
                self.skipped += 1
        self.last_t = t
        self.last_seq = fr["seq"]

        # 已解锁：拒绝采样（电机可能转起来，数据也不能用）
        if fr["flags"] & FLAG_ARMED:
            if self.state == "sample":
                msgs.append("检测到已解锁，停止采样；请保持飞控上锁、电机断开")
            self._reset_window()
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
            state = f"正在采样 {self.target} {done_s:.1f}/{self.cfg['sample_sec']:.0f}s"
        elif face:
            stable = (self.window[-1][0] - self.window[0][0]) if len(self.window) >= 2 else 0.0
            state = (f"请停稳 {min(stable, self.cfg['stable_sec']):.1f}/"
                     f"{self.cfg['stable_sec']:.1f}s")
        else:
            state = self.pending_hint or "请把机身摆到某个面"
        return (f"姿态 {face or '--'} | a=({ax:+.2f},{ay:+.2f},{az:+.2f})g | {state} | "
                f"还缺: {missing_text(self.faces)}")

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
        if face in self.faces:
            if self.hint_face != face:
                self.hint_face = face
                return [f"{face} 已完成，请翻到未完成的面：{missing_text(self.faces)}"]
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
            return [f"{face} 已停稳，开始采样 {cfg['sample_sec']:.0f}s …"]
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
            return [f"姿态改变（{target} → {face or '未识别'}），本面重新采"]

        # 采样中仍要求静止：角速度超限说明在转动/晃动，本面作废
        if max(abs(gx), abs(gy), abs(gz)) > cfg["gyro_max_dps"]:
            target = self.target
            self.aborted += 1
            self._restart_face()
            return [f"{target} 采样中检测到转动/晃动，本面重新采"]

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
        self.faces[face] = {
            "mean_lsb": means,
            "std_lsb": stds,
            "samples": n,
            "duration_s": dur,
        }
        self._restart_face()

        msg = [f"{face} 完成：均值({means[0]:+.1f},{means[1]:+.1f},{means[2]:+.1f}) LSB，"
               f"波动(±{stds[0]:.1f},±{stds[1]:.1f},±{stds[2]:.1f})，{n} 点 / {dur:.1f}s"]
        if len(self.faces) == len(FACES):
            msg += self._finish_all()
        else:
            msg.append(f"还缺：{missing_text(self.faces)}")
        return msg

    # ---------------- 六面齐全后的求解与检查 ----------------
    def solve(self):
        """按 (P+N)/2 与 2*2048/(P-N) 求解；返回 (结果 dict, 需要重采的轴列表)"""
        cfg = self.cfg
        res = {}
        bad = []
        for axis in "XYZ":
            p = self.faces.get("+" + axis)
            n = self.faces.get("-" + axis)
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

    def _finish_all(self):
        res, bad = self.solve()
        if bad:
            # 跨度异常：把该轴的正负面作废，要求重采
            for axis in bad:
                self.faces.pop("+" + axis, None)
                self.faces.pop("-" + axis, None)
            self.state = "wait"
            return [f"跨度检查不通过（{'、'.join(bad)}），请重采这些轴的正负面："
                    f"{missing_text(self.faces)}"]
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
# 串口数据源（只收）
# ----------------------------------------------------------------------------
class SerialSource:
    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            sys.exit("缺少 pyserial：请先执行  python -m pip install pyserial")
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as exc:
            sys.exit(f"打开串口 {port} 失败：{exc}")

    def read(self):
        return self.ser.read(4096)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def list_ports():
    import serial.tools.list_ports
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("未发现任何串口设备")
        return
    print("可用串口：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")


def pick_port(explicit):
    if explicit:
        return explicit
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("未发现串口，请用 --port 指定")
    for p in ports:
        text = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".upper()
        if any(k in text for k in ("CH340", "CH341", "CP210", "FT232", "FTDI", "PL2303", "USB 串行")):
            print(f"自动选择串口：{p.device} ({p.description})")
            return p.device
    if len(ports) == 1:
        print(f"自动选择串口：{ports[0].device} ({ports[0].description})")
        return ports[0].device
    print("检测到多个串口，请用 --port 指定：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
    sys.exit(1)


# ----------------------------------------------------------------------------
# 结果输出
# ----------------------------------------------------------------------------
def unique_path(path):
    """避免覆盖已有校准结果：同名时自动加序号"""
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    i = 1
    while os.path.exists(f"{root}_{i}{ext}"):
        i += 1
    return f"{root}_{i}{ext}"


def write_result(path, cal, build_id=None):
    cfg = cal.cfg
    data = {
        "format": "robin24g_accel_six_face_cal",
        "version": 1,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "sensor": {
            "model": "T-3488",
            "range_g": 16,
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
    data["checks"] = {
        "span_min_lsb": cfg["span_min_lsb"],
        "span_max_lsb": cfg["span_max_lsb"],
        "all_faces_complete": True,
        "skipped_frames": cal.skipped,
        "aborted_segments": cal.aborted,
        "firmware_build": f"0x{build_id:08X}" if build_id else None,
    }
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    return data


def print_summary(cal, data):
    """校准结果与"修正前 / 修正后"读数摘要"""
    cfg = cal.cfg
    print("\n================ 六面校准结果 ================")
    print(f"{'轴':<3}{'offset(LSB)':>14}{'scale':>12}{'跨度(LSB)':>12}")
    for axis in "XYZ":
        c = data["calibration"][axis]
        print(f"{axis:<3}{c['offset_lsb']:>14.1f}{c['scale']:>12.5f}{c['span_lsb']:>12.1f}")

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

    port = pick_port(args.port)
    print("=" * 64)
    print("T-3488 加速度计六面校准")
    print("  1) 请先关闭串口监视器，释放 COM 口")
    print("  2) 电机断开，飞控保持上锁（不要解锁）")
    print("  3) 依次把飞机摆到 6 个面，每个面停稳后保持不动")
    print(f"  4) 顺序随意；每面需稳定 {cfg['stable_sec']:.1f}s，再采样 {cfg['sample_sec']:.0f}s")
    print("=" * 64)

    src = SerialSource(port, args.baud)
    parser = FrameParser()
    cal = Calibrator(cfg)
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
        print("\n已中断，未生成校准文件（已完成的面不会保存）")
        return 1
    finally:
        src.close()

    if not cal.done:
        return 1

    build_id = last_perf["build_id"] if last_perf else None
    out_path = unique_path(args.out or
                           f"accel_cal_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    data = write_result(out_path, cal, build_id)
    print()
    print_summary(cal, data)
    print(f"\n校准文件：{os.path.abspath(out_path)}")
    print("后续：把 calibration 里的 offset_lsb / scale 写进固件参数加载即可"
          "（本次只交付校准文件，固件接入属于后续任务）。")
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
    args = ap.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
