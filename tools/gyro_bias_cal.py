#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
T-3488 陀螺仪静止零偏标定工具（单向 UART，只收不发）

用途
----
飞机放在桌上不动，测陀螺仪三轴零偏（bias），结果按机体 FRD 坐标系保存，
供固件后续做"映射 → 扣零偏 → 转 rad/s"。

流程
----
  1) 读 data/imu_axis_map_frd.json（传感器 → FRD 的映射，必须是 det=+1 的带符号置换矩阵）；
  2) 提示断开电机、关闭遥控器、把飞机放稳，按 Enter 开始；
  3) 先确认连续静止 3 s，再采 15 s 求三轴均值（拟合段），
     最后另采 10 s 复核（复核段只算扣偏后的残余均值与标准差）；
  4) 复核通过写 data/gyro_bias_frd.json，失败则打印每轴残差、不写文件。

取数
----
- 用 0x269A 帧里未扣偏的 gx/gy/gz（s16，2000 dps 量程，16.4 LSB/dps）；
- 旧固件上报的 bx/by/bz 不参与求解；
- flags 的 ARMED(0x08) 或 BIAS_APPLIED(0x02) 置位时整段作废；
- 先做 gyro_frd_lsb = R × gyro_sensor_lsb，再对 FRD 三轴分别求均值；
- 序号/时间戳回绕由 uart_proto 的解析器处理，连续性判断按序号步进 (u8) 与时间差。

用法
----
  python gyro_bias_cal.py --list
  python gyro_bias_cal.py --port COM12 --axis-map ..\data\imu_axis_map_frd.json \
         --out ..\data\gyro_bias_frd.json
"""

import argparse
import datetime
import json
import math
import os
import sys
import time

from uart_proto import FrameParser
from calib_common import (SerialSource, list_ports, pick_port, unique_path,
                          load_axis_map, mean_std)

# ----------------------------------------------------------------------------
# 参数集中定义（阈值、时长都在这里；README 有说明）
# ----------------------------------------------------------------------------
CONFIG = {
    "gyro_lsb_per_dps": 16.4,      # ±2000 dps 量程下的灵敏度
    "gyro_range_dps": 2000,        # 固件当前档位
    "acc_lsb_per_g": 2048.0,       # 只用于晃动辅助判断

    "stable_sec": 3.0,             # 先确认连续静止的时长
    "fit_sec": 15.0,               # 拟合段时长（求零偏）
    "review_sec": 10.0,            # 复核段时长
    "min_frames_ratio": 0.8,       # 段内有效帧 / 理论帧数 的下限

    # 静止判定（bench_A 实测：陀螺标准差 0.05 dps、峰峰 0.6 dps、加速度标准差 ~0.001g）
    "gyro_std_max_dps": 0.5,       # 段内三轴标准差上限
    "gyro_jump_max_dps": 1.5,      # 相邻帧变化上限（抓突然抖动）
    "gyro_abs_max_dps": 10.0,      # 单轴绝对值上限（要允许正常的待测零偏）
    "acc_std_max_g": 0.03,         # 加速度标准差上限（辅助判断晃动）
    "acc_norm_min_g": 0.80,        # 加速度模长合理范围
    "acc_norm_max_g": 1.20,
    "max_gap_s": 0.10,             # 相邻帧最大时间间隔，超过算明显间断

    # 复核判据（扣偏后的残余）
    "review_mean_max_dps": 0.30,   # 残余均值上限（≈5 LSB）
    "review_std_max_dps": 0.20,    # 残余标准差上限
}

FLAG_BIAS_APPLIED = 0x02           # 帧内陀螺已扣零偏（本版固件恒为 0）
FLAG_ARMED = 0x08                  # 已解锁

AXIS_NAMES = ("X", "Y", "Z")
GYRO_MODEL = "T-3488"


def apply_matrix(matrix, vec):
    """v_frd = R × v_sensor（元素只有 0/±1，直接相乘相加）"""
    return [sum(matrix[r][c] * vec[c] for c in range(3)) for r in range(3)]


# ----------------------------------------------------------------------------
# 分段采样与静止判定
# ----------------------------------------------------------------------------
class Session:
    """
    状态机：stable（判稳） → fit（拟合） → review（复核） → done。
    每帧进来都做状态与连续性检查，不合格就作废"当前段"并给出中文原因。
    """

    def __init__(self, cfg, matrix):
        self.cfg = cfg
        self.matrix = matrix
        self.phase = "stable"
        self.seg = []              # 当前段 [(t_s, g_frd_lsb[3], a_lsb[3])]
        self.last_t = None
        self.last_seq = None

        self.fit = None            # 拟合段结果
        self.review = None         # 复核段结果
        self.bias = None           # 拟合得到的零偏（FRD，LSB）
        self.result_ok = False
        self.done = False

        self.rejects = {"armed": 0, "bias_applied": 0, "gap": 0, "seq": 0}
        self.restarts = 0

    # ---------------- 对外 ----------------
    def feed(self, fr):
        cfg = self.cfg
        msgs = []

        if self.done:
            return msgs

        # 先更新上一帧的时间/序号：即使本帧被判废，下一帧也应以它为参照，
        # 否则会一直拿旧值比较、连续触发"跳变/间断"
        t = fr["t_ms"] / 1000.0
        prev_t, prev_seq = self.last_t, self.last_seq
        self.last_t, self.last_seq = t, fr["seq"]

        # 1) 状态：已解锁 / 帧内已扣零偏 都不能用
        if fr["flags"] & FLAG_ARMED:
            self.rejects["armed"] += 1
            return self._restart("检测到已解锁（请保持飞控上锁）")
        if fr["flags"] & FLAG_BIAS_APPLIED:
            self.rejects["bias_applied"] += 1
            return self._restart("该帧陀螺已扣零偏（BIAS_APPLIED），不能用来测零偏")

        # 2) 连续性：时间间隔与序号步进（序号 u8 回绕由 &0xFF 处理）
        if prev_t is not None:
            gap = t - prev_t
            step = (fr["seq"] - prev_seq) & 0xFF
            if gap > cfg["max_gap_s"]:
                self.rejects["gap"] += 1
                return self._restart(f"数据间断（间隔 {gap*1000:.0f}ms）")
            if step != 1:
                self.rejects["seq"] += 1
                return self._restart(f"帧序不连续（跳了 {step-1} 帧）")

        # 3) 映射到 FRD
        g_sensor = [fr["gx"], fr["gy"], fr["gz"]]
        g_frd = apply_matrix(self.matrix, g_sensor)
        a_lsb = [fr["ax"], fr["ay"], fr["az"]]
        self.seg.append((t, g_frd, a_lsb))

        # 4) 按阶段推进
        if self.phase == "stable":
            msgs += self._step_stable(t)
        elif self.phase == "fit":
            msgs += self._step_fit(t)
        elif self.phase == "review":
            msgs += self._step_review(t)
        return msgs

    def status(self):
        """一行状态：阶段、进度、当前 FRD 陀螺与加速度"""
        if not self.seg:
            return f"[{self._phase_cn()}] 等待数据 …"
        t, g, a = self.seg[-1]
        span = t - self.seg[0][0]
        if self.phase == "stable":
            need, done = self.cfg["stable_sec"], span
        elif self.phase == "fit":
            need, done = self.cfg["fit_sec"], span
        elif self.phase == "review":
            need, done = self.cfg["review_sec"], span
        else:
            need, done = 1.0, 1.0
        gd = [v / self.cfg["gyro_lsb_per_dps"] for v in g]
        return (f"[{self._phase_cn()}] {min(done, need):.1f}/{need:.0f}s "
                f"({len(self.seg)} 帧) | g_frd=({g[0]:+.0f},{g[1]:+.0f},{g[2]:+.0f}) LSB "
                f"=({gd[0]:+.2f},{gd[1]:+.2f},{gd[2]:+.2f}) dps | "
                f"a=({a[0]/self.cfg['acc_lsb_per_g']:+.2f},"
                f"{a[1]/self.cfg['acc_lsb_per_g']:+.2f},"
                f"{a[2]/self.cfg['acc_lsb_per_g']:+.2f})g")

    # ---------------- 内部 ----------------
    def _phase_cn(self):
        return {"stable": "判稳", "fit": "拟合", "review": "复核", "done": "完成"}.get(
            self.phase, self.phase)

    def _restart(self, reason):
        """当前段作废：拟合段退回判稳，复核段重采，判稳段清窗口"""
        self.restarts += 1
        if self.phase == "fit":
            self.phase = "stable"
            self.seg = []
            return [f"拟合段作废：{reason}；重新判稳"]
        if self.phase == "review":
            self.seg = []
            return [f"复核段作废：{reason}；重新复核"]
        self.seg = []
        return [f"判稳窗口清空：{reason}"]

    def _still(self, seg):
        """判断一段数据是否静止；返回 (是否通过, 原因)"""
        cfg = self.cfg
        if len(seg) < 2:
            return False, "数据太少"
        g = [[s[1][k] for s in seg] for k in range(3)]
        a = [[s[2][k] for s in seg] for k in range(3)]

        for k in range(3):
            m, sd = mean_std(g[k])
            sd_dps = sd / cfg["gyro_lsb_per_dps"]
            if sd_dps > cfg["gyro_std_max_dps"]:
                return False, f"陀螺 {AXIS_NAMES[k]} 轴抖动 {sd_dps:.2f} dps，请停稳"
            if abs(m) / cfg["gyro_lsb_per_dps"] > cfg["gyro_abs_max_dps"]:
                return False, f"陀螺 {AXIS_NAMES[k]} 轴读数 {m/16.4:+.1f} dps 过大，检查是否在转"
        # 相邻帧跳变
        for i in range(1, len(seg)):
            for k in range(3):
                d = abs(seg[i][1][k] - seg[i - 1][1][k]) / cfg["gyro_lsb_per_dps"]
                if d > cfg["gyro_jump_max_dps"]:
                    return False, f"陀螺 {AXIS_NAMES[k]} 轴突然跳变 {d:.2f} dps，请松手"

        for k in range(3):
            _, sd = mean_std(a[k])
            if sd / cfg["acc_lsb_per_g"] > cfg["acc_std_max_g"]:
                return False, f"加速度 {AXIS_NAMES[k]} 轴变化 {sd/2048:.3f}g，机身在动"
        for s in seg:
            n = math.sqrt(sum(v * v for v in s[2])) / cfg["acc_lsb_per_g"]
            if not (cfg["acc_norm_min_g"] <= n <= cfg["acc_norm_max_g"]):
                return False, f"加速度模长 {n:.2f}g 不合理，请摆正"
        return True, ""

    def _check_ratio(self, seg, need_sec):
        """有效帧比例：100Hz 下应有 need_sec*100 帧"""
        expect = need_sec * 100.0
        return len(seg) >= expect * self.cfg["min_frames_ratio"]

    def _step_stable(self, t):
        cfg = self.cfg
        # 只保留最近 stable_sec + 0.2s
        keep = t - cfg["stable_sec"] - 0.2
        while len(self.seg) > 2 and self.seg[0][0] < keep:
            self.seg.pop(0)
        span = self.seg[-1][0] - self.seg[0][0]
        if span < cfg["stable_sec"]:
            return []
        ok, reason = self._still(self.seg)
        if not ok:
            # 静不下来就保持窗口滑动，只清掉过期数据，不反复打印
            return []
        self.phase = "fit"
        self.seg = []
        return [f"已静止 {cfg['stable_sec']:.0f}s，开始拟合 {cfg['fit_sec']:.0f}s …"]

    def _step_fit(self, t):
        cfg = self.cfg
        span = t - self.seg[0][0]
        if len(self.seg) % 10 == 0:
            ok, reason = self._still(self.seg)
            if not ok:
                return self._restart(reason)
        if span < cfg["fit_sec"]:
            return []
        ok, reason = self._still(self.seg)
        if not ok:
            return self._restart(reason)
        if not self._check_ratio(self.seg, cfg["fit_sec"]):
            return self._restart(f"有效帧比例不足（{len(self.seg)} 帧）")

        g = [[s[1][k] for s in self.seg] for k in range(3)]
        mean_lsb = [mean_std(g[k])[0] for k in range(3)]
        std_lsb = [mean_std(g[k])[1] for k in range(3)]
        self.bias = mean_lsb
        self.fit = {
            "frames": len(self.seg),
            "duration_s": round(span, 3),
            "mean_lsb": mean_lsb,
            "std_lsb": std_lsb,
            "mean_dps": [v / cfg["gyro_lsb_per_dps"] for v in mean_lsb],
            "std_dps": [v / cfg["gyro_lsb_per_dps"] for v in std_lsb],
        }
        self.phase = "review"
        self.seg = []
        return [f"拟合完成：bias_frd = ({mean_lsb[0]:+.1f},{mean_lsb[1]:+.1f},"
                f"{mean_lsb[2]:+.1f}) LSB，开始复核 {cfg['review_sec']:.0f}s …"]

    def _step_review(self, t):
        cfg = self.cfg
        span = t - self.seg[0][0]
        if len(self.seg) % 10 == 0:
            ok, reason = self._still(self.seg)
            if not ok:
                return self._restart(reason)
        if span < cfg["review_sec"]:
            return []
        ok, reason = self._still(self.seg)
        if not ok:
            return self._restart(reason)
        if not self._check_ratio(self.seg, cfg["review_sec"]):
            return self._restart(f"有效帧比例不足（{len(self.seg)} 帧）")

        g = [[s[1][k] for s in self.seg] for k in range(3)]
        mean_lsb = [mean_std(g[k])[0] for k in range(3)]
        std_lsb = [mean_std(g[k])[1] for k in range(3)]
        resid = [mean_lsb[k] - self.bias[k] for k in range(3)]
        self.review = {
            "frames": len(self.seg),
            "duration_s": round(span, 3),
            "mean_lsb": mean_lsb,
            "std_lsb": std_lsb,
            "residual_lsb": resid,
            "residual_dps": [v / cfg["gyro_lsb_per_dps"] for v in resid],
            "std_dps": [v / cfg["gyro_lsb_per_dps"] for v in std_lsb],
        }
        self.done = True
        bad = [AXIS_NAMES[k] for k in range(3)
               if abs(resid[k]) / cfg["gyro_lsb_per_dps"] > cfg["review_mean_max_dps"]
               or std_lsb[k] / cfg["gyro_lsb_per_dps"] > cfg["review_std_max_dps"]]
        self.result_ok = not bad
        if bad:
            return [f"复核不通过：{'、'.join(bad)} 轴残余超限（详见下面的残差）"]
        return ["复核通过"]


# ----------------------------------------------------------------------------
# 输出
# ----------------------------------------------------------------------------
def write_result(path, cfg, matrix, map_path, session, build_id=None):
    s = session
    data = {
        "format": "robin24g_gyro_bias_frd",
        "version": 1,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "sensor": {
            "model": GYRO_MODEL,
            "gyro_range_dps": cfg["gyro_range_dps"],
            "gyro_lsb_per_dps": cfg["gyro_lsb_per_dps"],
            "acc_lsb_per_g": cfg["acc_lsb_per_g"],
        },
        "target_frame": {
            "name": "FRD", "x": "forward", "y": "right", "z": "down",
            "handedness": "right",
        },
        "axis_map": {
            "source": os.path.abspath(map_path),
            "matrix_sensor_to_frd": matrix,
            "convention": "v_frd = R * v_sensor（行=FRD X/Y/Z，列=传感器 X/Y/Z）",
        },
        "bias_frd_lsb": [round(v, 4) for v in s.bias],
        "bias_frd_dps": [round(v / cfg["gyro_lsb_per_dps"], 6) for v in s.bias],
        "segments": {
            "fit": {k: v for k, v in s.fit.items()},
            "review": {k: v for k, v in s.review.items()},
        },
        "checks": {
            "stable_sec": cfg["stable_sec"],
            "gyro_std_max_dps": cfg["gyro_std_max_dps"],
            "gyro_jump_max_dps": cfg["gyro_jump_max_dps"],
            "gyro_abs_max_dps": cfg["gyro_abs_max_dps"],
            "acc_std_max_g": cfg["acc_std_max_g"],
            "max_gap_s": cfg["max_gap_s"],
            "review_mean_max_dps": cfg["review_mean_max_dps"],
            "review_std_max_dps": cfg["review_std_max_dps"],
            "rejected_armed": s.rejects["armed"],
            "rejected_bias_applied": s.rejects["bias_applied"],
            "rejected_gap": s.rejects["gap"],
            "rejected_seq": s.rejects["seq"],
            "segment_restarts": s.restarts,
            "firmware_build": f"0x{build_id:08X}" if build_id else None,
        },
        "notes": [
            "这是静止零偏（bias），对应本次上电、本次采集时的状态；温度/上电批次变化后需要重测",
            "固件使用顺序：gyro_frd = R * gyro_sensor → 减去 bias_frd_lsb → 再按 "
            "1/16.4 dps/LSB、π/180 换成 rad/s",
            "求解只用 0x269A 的 gx/gy/gz 原始计数；bx/by/bz 未参与",
        ],
    }
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    os.replace(tmp, path)
    return data


def print_summary(session, cfg):
    l2d = cfg["gyro_lsb_per_dps"]
    print("\n============ 陀螺仪零偏结果 ============")
    print(f"bias_frd_lsb = ({session.bias[0]:+.2f}, {session.bias[1]:+.2f}, "
          f"{session.bias[2]:+.2f}) LSB")
    print(f"bias_frd_dps = ({session.bias[0]/l2d:+.3f}, {session.bias[1]/l2d:+.3f}, "
          f"{session.bias[2]/l2d:+.3f}) °/s")

    fit, rev = session.fit, session.review
    print(f"\n拟合段：{fit['frames']} 帧 / {fit['duration_s']:.2f}s，"
          f"标准差 ({fit['std_dps'][0]:.3f},{fit['std_dps'][1]:.3f},"
          f"{fit['std_dps'][2]:.3f}) °/s")
    print(f"复核段：{rev['frames']} 帧 / {rev['duration_s']:.2f}s，残余 "
          f"({rev['residual_dps'][0]:+.3f},{rev['residual_dps'][1]:+.3f},"
          f"{rev['residual_dps'][2]:+.3f}) °/s，"
          f"标准差 ({rev['std_dps'][0]:.3f},{rev['std_dps'][1]:.3f},"
          f"{rev['std_dps'][2]:.3f}) °/s")
    print(f"复核判据：残余均值 ≤ {cfg['review_mean_max_dps']:.2f} °/s，"
          f"标准差 ≤ {cfg['review_std_max_dps']:.2f} °/s")
    print("\n说明：零偏是“当前上电、当前温度”下的静止值；固件里先映射到 FRD，"
          "再减 bias，再乘 0.0010641 得到 rad/s。")


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def run(args):
    if args.list:
        list_ports()
        return 0

    cfg = dict(CONFIG)
    for key, val in (("stable_sec", args.stable), ("fit_sec", args.fit),
                     ("review_sec", args.review)):
        if val is not None:
            cfg[key] = val
    if args.acc_lsb_per_g is not None:
        cfg["acc_lsb_per_g"] = args.acc_lsb_per_g

    map_path = args.axis_map or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "data", "imu_axis_map_frd.json")
    matrix, map_json = load_axis_map(map_path)

    print("=" * 68)
    print("T-3488 陀螺仪静止零偏标定")
    print(f"  轴映射：{os.path.abspath(map_path)}")
    print(f"  矩阵  ：{matrix}（v_frd = R × v_sensor，det=+1）")
    print("  1) 断开电机，关闭遥控器；飞机放在桌上，松手让它静止")
    print("  2) 可以稍微倾斜，只要不动即可；不要碰线缆")
    print(f"  3) 确认静止 {cfg['stable_sec']:.0f}s → 拟合 {cfg['fit_sec']:.0f}s → "
          f"复核 {cfg['review_sec']:.0f}s")
    print("=" * 68)

    port = pick_port(args.port)
    # 先打开串口：端口不对时立刻报错，不用先按 Enter
    src = SerialSource(port, args.baud)
    ans = input("摆好后按 Enter 开始（q 退出）：").strip().lower()
    if ans == "q":
        src.close()
        print("已退出")
        return 1

    parser = FrameParser()
    session = Session(cfg, matrix)
    last_status = 0.0
    last_print = 0.0
    last_perf = None

    src.drain()
    try:
        while not session.done:
            data = src.read()
            for kind, fr in parser.feed(data):
                if kind == "perf":
                    last_perf = fr
                    continue
                for msg in session.feed(fr):
                    if sys.stdout.isatty():
                        print("\r" + " " * 110 + "\r" + msg)
                    else:
                        print(msg, flush=True)
            now = time.time()
            if now - last_status >= 0.2:
                last_status = now
                line = session.status()
                if sys.stdout.isatty():
                    print("\r" + line.ljust(110), end="", flush=True)
                elif now - last_print >= 2.0:
                    last_print = now
                    print(line, flush=True)
    except KeyboardInterrupt:
        print("\n已中断，未生成零偏文件")
        return 1
    finally:
        src.close()

    build_id = last_perf["build_id"] if last_perf else None
    print()
    print_summary(session, cfg)

    if not session.result_ok:
        print("\n复核未通过：不写结果文件。请确认飞控上锁、电机断开、"
              "遥控器关闭后重新运行本工具。")
        return 1

    out_path = unique_path(args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "data", "gyro_bias_frd.json"))
    data = write_result(out_path, cfg, matrix, map_path, session, build_id)
    print(f"\n零偏文件：{os.path.abspath(out_path)}")
    print("固件接入（后续任务）：gyro_frd = R * gyro_sensor；"
          "gyro = (gyro_frd - bias_frd_lsb) * 0.0010641 rad/s")
    return 0


def main():
    ap = argparse.ArgumentParser(description="T-3488 陀螺仪静止零偏标定")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", help="串口号，例如 COM12；不填则自动选择")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--axis-map", help="传感器→FRD 映射 JSON（默认 ../data/imu_axis_map_frd.json）")
    ap.add_argument("--out", help="零偏结果 JSON（默认 ../data/gyro_bias_frd.json，同名不覆盖）")
    ap.add_argument("--stable", type=float, help="判稳时长秒（默认 3）")
    ap.add_argument("--fit", type=float, help="拟合段时长秒（默认 15）")
    ap.add_argument("--review", type=float, help="复核段时长秒（默认 10）")
    ap.add_argument("--acc-lsb-per-g", type=float, help="加速度灵敏度（默认 2048）")
    args = ap.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
