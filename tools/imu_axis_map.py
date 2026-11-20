#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
T-3488 传感器轴 → 机体系 FRD 的安装映射确认工具（单向 UART，只收不发）

机体系（右手 FRD）：+X 向前，+Y 向右，+Z 向下。

原理
----
静止时加速度计测的是"比力"，方向指向上方。把机体某个轴摆成朝下时，该 FRD 轴的
期望读数是 -1g。例如：机体 +X 朝下时传感器 +Y 读数约 +1g，则 X_frd = -Y_sensor。

工具按固定顺序提示 6 个"朝下"姿态：+Z、+Y、+X、-Z、-Y、-X
（正放 / 右侧朝下 / 机头朝下 / 倒放 / 左侧朝下 / 机尾朝下），用户摆好后在电脑上
按 Enter，工具采一小段静止数据求均值、识别主轴与符号，最后解出 3×3 映射矩阵：

    v_frd = R_sensor_to_frd × v_sensor
    R 的行按 FRD 的 X/Y/Z，列按传感器的 X/Y/Z

校验：每个 FRD 轴在两个相反姿态（如 +Z 朝下 与 -Z 朝下）必须落在同一传感器轴上
且符号相反；三行三列各只有一个非零元素，元素只能是 0/±1，行列式必须为 +1（右手系）。
任何一项不通过都会提示复核对应摆法，且不输出映射文件。

本工具只做轴映射，不读取 accel_cal.json，也不计算零偏/比例系数
（那些由 accel_six_face_cal.py 负责）；加速度计与陀螺仪在各自校准完成后
共用同一份安装映射。

用法
----
  python imu_axis_map.py --list
  python imu_axis_map.py --port COM12 --out "..\data\imu_axis_map_frd.json"
"""

import argparse
import datetime
import json
import math
import os
import queue
import sys
import threading
import time

from uart_proto import FrameParser
from calib_common import (MAP_FORMAT, SerialSource, list_ports, pick_port,
                          unique_path)

# ----------------------------------------------------------------------------
# 参数集中定义
# ----------------------------------------------------------------------------
CONFIG = {
    "lsb_per_g": 2048.0,        # ±16g 量程：1g = 2048 LSB
    "window_sec": 1.2,          # 每个姿态的采样时长（设备时间戳，秒）
    "min_samples": 60,          # 采样点数下限
    "main_min_g": 0.50,         # 主轴分量下限（允许轻微倾斜，但要能认出主轴）
    "dominance_ratio": 1.5,     # 主轴 / 次大轴 的最小比值
    "std_max_g": 0.04,          # 静止判定：三轴标准差上限（g）
    "max_gap_s": 0.20,          # 相邻帧最大间隔，超过算数据间断
    "first_frame_timeout_s": 3.0,   # 按 Enter 后等第一帧的超时
    "frame_stall_s": 0.6,       # 采样中超过该时长没有新帧 = 数据中断
}

FLAG_ARMED = 0x08               # 固件 0x269A 的 flags bit3：已解锁

# 六个实体姿态：标题统一为"机体 ±轴朝下"。
# 注意：id 与 JSON 字段沿用旧命名，保持兼容——
#   Z_down = +Z 朝下（正放）    Y_down = +Y 朝下（右侧朝下）  X_down = +X 朝下（机头朝下）
#   Z_up   = -Z 朝下（倒放）    Y_up   = -Y 朝下（左侧朝下）  X_up   = -X 朝下（机尾朝下）
# body 为轴字母；down=+1 表示 +轴朝下，-1 表示 +轴朝上（等价于 -轴朝下）。
POSES = [
    {"id": "Z_down", "body": "Z", "down": 1,
     "title": "机体 +Z 朝下（正放）",
     "hint": "机腹朝下，机头朝前，平放在桌面/垫块上"},
    {"id": "Y_down", "body": "Y", "down": 1,
     "title": "机体 +Y 朝下（右侧朝下）",
     "hint": "右机翼朝下，机身侧立，机头仍朝前"},
    {"id": "X_down", "body": "X", "down": 1,
     "title": "机体 +X 朝下（机头朝下）",
     "hint": "机头垂直朝下，机尾朝上"},
    {"id": "Z_up", "body": "Z", "down": -1,
     "title": "机体 -Z 朝下（倒放）",
     "hint": "机腹朝上，机背朝下，机头仍朝前"},
    {"id": "Y_up", "body": "Y", "down": -1,
     "title": "机体 -Y 朝下（左侧朝下）",
     "hint": "左机翼朝下，机身侧立，机头仍朝前"},
    {"id": "X_up", "body": "X", "down": -1,
     "title": "机体 -X 朝下（机尾朝下）",
     "hint": "机头垂直朝上，机尾朝下"},
]

AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


# ----------------------------------------------------------------------------
# 串口读取线程：只收 0x269A 六轴帧（0x269B 直接跳过）
# ----------------------------------------------------------------------------
class FrameReader(threading.Thread):
    def __init__(self, source):
        super().__init__(daemon=True)
        self.source = source
        self.parser = FrameParser()
        self.q = queue.Queue(maxsize=4000)
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            data = self.source.read()
            if not data:
                continue
            for kind, fr in self.parser.feed(data):
                if kind != "sensor":
                    continue            # 性能帧与轴映射无关，跳过
                try:
                    self.q.put_nowait(fr)
                except queue.Full:
                    pass                # 队列满只影响实时显示，不影响采样窗口

    def stop(self):
        self.stop_event.set()

    def get(self, timeout=0.2):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self):
        """丢掉积压的旧帧（按 Enter 之前的数据不能算进本次姿态）"""
        while True:
            try:
                self.q.get_nowait()
            except queue.Empty:
                return


# ----------------------------------------------------------------------------
# 单个姿态的数据分析
# ----------------------------------------------------------------------------
def analyze_window(frames, cfg):
    """
    对一段静止数据求均值并识别主轴。
    返回 (是否通过, 原因, 统计字典)；统计字典包含 mean_g/axis/sign/samples/duration_s。
    """
    if len(frames) < cfg["min_samples"]:
        return False, f"采样点太少（{len(frames)} < {cfg['min_samples']}）", None

    t0 = frames[0]["t_ms"]
    t1 = frames[-1]["t_ms"]
    duration = (t1 - t0) / 1000.0
    if duration < cfg["window_sec"] * 0.8:
        return False, f"采样时间不足（{duration:.2f}s）", None

    # 数据间断
    last_t = frames[0]["t_ms"]
    max_gap = 0.0
    for fr in frames[1:]:
        gap = (fr["t_ms"] - last_t) / 1000.0
        if gap > max_gap:
            max_gap = gap
        last_t = fr["t_ms"]
    if max_gap > cfg["max_gap_s"]:
        return False, f"数据间断（最大间隔 {max_gap*1000:.0f}ms），请重采", None

    # 任何一帧显示已解锁都拒绝
    for fr in frames:
        if fr["flags"] & FLAG_ARMED:
            return False, "检测到飞控已解锁，请保持上锁、电机断开后重采", None

    vals = [[fr["ax"] / cfg["lsb_per_g"] for fr in frames],
            [fr["ay"] / cfg["lsb_per_g"] for fr in frames],
            [fr["az"] / cfg["lsb_per_g"] for fr in frames]]
    means = [sum(v) / len(v) for v in vals]
    stds = []
    for v, m in zip(vals, means):
        stds.append(math.sqrt(sum((x - m) ** 2 for x in v) / len(v)))

    if max(stds) > cfg["std_max_g"]:
        return False, f"读数在晃动（标准差最大 {max(stds):.3f}g），请停稳后重采", None

    # 主轴（按均值）
    axis = max(range(3), key=lambda k: abs(means[k]))
    main = means[axis]
    others = [abs(means[k]) for k in range(3) if k != axis]
    if abs(main) < cfg["main_min_g"]:
        return False, f"主轴分量只有 {main:+.2f}g，识别不出方向，请摆正后重采", None
    if abs(main) < cfg["dominance_ratio"] * max(others):
        return False, (f"主轴不明确（{abs(main):.2f}g vs 次大 {max(others):.2f}g），"
                       f"请把机体摆正一些"), None

    # 采样过程中主轴不能变（避免"一边转一边采"）
    for fr in frames:
        v = (fr["ax"] / cfg["lsb_per_g"], fr["ay"] / cfg["lsb_per_g"], fr["az"] / cfg["lsb_per_g"])
        k = max(range(3), key=lambda i: abs(v[i]))
        if k != axis:
            return False, "采样中姿态发生变化，请重新摆好本姿态", None

    stats = {
        "mean_g": means,
        "std_g": stds,
        "axis": "XYZ"[axis],
        "sign": 1 if main > 0 else -1,
        "samples": len(frames),
        "duration_s": duration,
    }
    return True, "", stats


# ----------------------------------------------------------------------------
# 映射求解与校验
# ----------------------------------------------------------------------------
def build_mapping(records):
    """
    records: {pose_id: stats}，stats 含 body/down/axis/sign/mean_g。
    返回 (matrix, mapping, det, problems)：
      matrix  3×3，v_frd = matrix × v_sensor
      mapping {'X_frd': '+Y_sensor', ...}
      det     行列式（整数）
      problems 列表，元素为 (说明, 需要重采的 pose_id 列表)
    """
    problems = []
    # 每个机体轴：+轴朝下 与 -轴朝下 两个姿态必须同轴反号
    col = {}
    for body in "XYZ":
        down = records.get(body + "_down")
        up = records.get(body + "_up")
        if not down or not up:
            problems.append((f"机体 {body} 轴缺少姿态数据", [body + "_down", body + "_up"]))
            continue
        if down["axis"] != up["axis"]:
            problems.append((f"机体 {body} 轴：+{body} 朝下 与 -{body} 朝下 检测到的传感器轴"
                             f"不一致（{down['axis']} / {up['axis']}）",
                             [body + "_down", body + "_up"]))
            continue
        if down["sign"] == up["sign"]:
            problems.append((f"机体 {body} 轴：+{body} 朝下 与 -{body} 朝下 的符号相同"
                             f"（都 {down['sign']:+d}），说明有一面摆反了",
                             [body + "_down", body + "_up"]))
            continue
        col[body] = down["axis"]

    if problems:
        return None, None, None, problems

    # 三行必须映射到不同传感器轴
    if len(set(col.values())) != 3:
        problems.append(("三个 FRD 轴映射到了同一个/重复的传感器轴，请复核全部摆法",
                         [p["id"] for p in POSES]))
        return None, None, None, problems

    # R[B][j] = -sign * down（down=+1 朝下；见文件头推导）
    matrix = [[0, 0, 0] for _ in range(3)]
    for body in "XYZ":
        down = records[body + "_down"]
        r = AXIS_INDEX[body]
        c = AXIS_INDEX[col[body]]
        matrix[r][c] = -down["sign"] * down["down"]

    # 每行每列各一个非零、元素为 ±1
    for r in range(3):
        nz = [v for v in matrix[r] if v != 0]
        if len(nz) != 1 or abs(nz[0]) != 1:
            problems.append((f"矩阵第 {r+1} 行不满足“只有一个 ±1”", [p["id"] for p in POSES]))
    for c in range(3):
        nz = [matrix[r][c] for r in range(3) if matrix[r][c] != 0]
        if len(nz) != 1 or abs(nz[0]) != 1:
            problems.append((f"矩阵第 {c+1} 列不满足“只有一个 ±1”", [p["id"] for p in POSES]))
    if problems:
        return None, None, None, problems

    det = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
           - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
           + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
    if det != 1:
        problems.append((f"行列式为 {det:+d}（必须是 +1，右手系），"
                         f"请复核六个摆法是否与图示一致", [p["id"] for p in POSES]))
        return None, None, None, problems

    mapping = {}
    for r, body in enumerate("XYZ"):
        c = next(i for i in range(3) if matrix[r][i] != 0)
        sign = "+" if matrix[r][c] > 0 else "-"
        mapping[f"{body}_frd"] = f"{sign}{'XYZ'[c]}_sensor"
    return matrix, mapping, det, []


def matrix_text(matrix):
    """把矩阵排成易读的三行"""
    return "\n".join("  [" + "  ".join(f"{v:+d}" for v in row) + "]" for row in matrix)


# ----------------------------------------------------------------------------
# 交互采集
# ----------------------------------------------------------------------------
def collect_pose(reader, pose, cfg):
    """
    提示用户摆好 → 按 Enter → 采一小段 → 分析。
    返回 (stats, 原因)；成功时原因为空字符串。
    """
    print()
    print("=" * 70)
    print(f"姿态：{pose['title']}")
    print(f"  摆法：{pose['hint']}")
    ans = input("  摆好后按 Enter 开始采样（q 退出）：").strip().lower()
    if ans == "q":
        return None, "QUIT"

    reader.drain()                      # 丢掉按 Enter 之前的旧数据
    t0 = None
    frames = []
    wait_start = time.time()
    last_rx = time.time()

    while True:
        fr = reader.get(0.2)
        if fr is None:
            if t0 is None:
                if time.time() - wait_start > cfg["first_frame_timeout_s"]:
                    return None, "没有收到 0x269A 六轴数据（检查串口/固件/波特率）"
            elif time.time() - last_rx > cfg["frame_stall_s"]:
                return None, "采样中数据中断，请重采本姿态"
            continue

        last_rx = time.time()
        t = fr["t_ms"] / 1000.0
        if t0 is None:
            t0 = t
        frames.append(fr)

        # 实时显示：读数 + 当前主轴 + 稳定进度
        ax, ay, az = (fr["ax"] / cfg["lsb_per_g"],
                      fr["ay"] / cfg["lsb_per_g"],
                      fr["az"] / cfg["lsb_per_g"])
        k = max(range(3), key=lambda i: abs((ax, ay, az)[i]))
        cur_axis = ("+" if (ax, ay, az)[k] > 0 else "-") + "XYZ"[k]
        prog = min(1.0, max(0.0, (t - t0) / cfg["window_sec"]))
        line = (f"  XYZ=({ax:+.2f},{ay:+.2f},{az:+.2f})g  主轴 {cur_axis}  "
                f"进度 {prog*100:3.0f}%  ({len(frames)} 点)")
        if sys.stdout.isatty():
            print("\r" + line.ljust(78), end="", flush=True)
        elif prog < 1.0 and len(frames) % 20 == 1:
            print(line, flush=True)

        if (t - t0) >= cfg["window_sec"] and len(frames) >= cfg["min_samples"]:
            break
        # 解锁立即中止
        if fr["flags"] & FLAG_ARMED:
            return None, "检测到飞控已解锁，请保持上锁后重采"

    if sys.stdout.isatty():
        print()
    ok, reason, stats = analyze_window(frames, cfg)
    if not ok:
        return None, reason

    stats.update({"pose": pose["id"], "body": pose["body"], "down": pose["down"]})
    print(f"  均值 ({stats['mean_g'][0]:+.3f},{stats['mean_g'][1]:+.3f},"
          f"{stats['mean_g'][2]:+.3f})g → 主轴 {stats['sign']:+d}{stats['axis']}，"
          f"{stats['samples']} 点 / {stats['duration_s']:.2f}s")
    return stats, ""


def collect_all(reader, cfg, only_ids=None):
    """按 POSES 顺序采集；only_ids 非空时只采这些姿态"""
    records = {}
    poses = [p for p in POSES if (only_ids is None or p["id"] in only_ids)]
    for pose in poses:
        while True:
            stats, reason = collect_pose(reader, pose, cfg)
            if reason == "QUIT":
                return None
            if stats is not None:
                records[pose["id"]] = stats
                break
            print(f"  × 未通过：{reason}")
            ans = input("  按 Enter 重试本姿态（q 退出）：").strip().lower()
            if ans == "q":
                return None
    return records


# ----------------------------------------------------------------------------
# 结果输出
# ----------------------------------------------------------------------------
def write_json(path, records, matrix, mapping, det):
    data = {
        "format": MAP_FORMAT,
        "version": 1,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "sensor": {"model": "T-3488", "accel_lsb_per_g": CONFIG["lsb_per_g"]},
        "target_frame": {
            "name": "FRD",
            "x": "forward", "y": "right", "z": "down",
            "handedness": "right",
        },
        "matrix_sensor_to_frd": matrix,
        "matrix_convention": "v_frd = R * v_sensor；行 = FRD 的 X/Y/Z，列 = 传感器的 X/Y/Z",
        "mapping": mapping,
        "determinant": det,
        "poses": {},
        "notes": [
            "只做坐标轴映射，不含零偏/比例系数（用 accel_six_face_cal.py）",
            "加速度计与陀螺仪校准完成后共用本映射：v_frd = R * v_calibrated",
            "六个姿态的读数均为静止时三轴均值（g），仅用于轴/符号判定",
            "姿态 id 沿用旧命名：Z_up=-Z 朝下（倒放）、Y_up=-Y 朝下（左侧朝下）、"
            "X_up=-X 朝下（机尾朝下）；body_axis 为轴字母，axis_down 表示 +轴朝下",
        ],
    }
    for pose in POSES:
        st = records[pose["id"]]
        data["poses"][pose["id"]] = {
            "title": pose["title"],
            "body_axis": pose["body"],
            "axis_down": pose["down"] > 0,
            "sensor_axis": st["axis"],
            "sensor_sign": st["sign"],
            "mean_xyz_g": [round(v, 4) for v in st["mean_g"]],
            "std_xyz_g": [round(v, 4) for v in st["std_g"]],
            "samples": st["samples"],
            "duration_s": round(st["duration_s"], 3),
        }
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=2)
        fp.write("\n")
    return data


def print_result(records, matrix, mapping, det):
    print()
    print("=" * 70)
    print("轴映射结果：")
    for body in "XYZ":
        print(f"  {body}_frd = {mapping[body + '_frd']}")
    print()
    print("矩阵（v_frd = R × v_sensor；行 = FRD 的 X/Y/Z，列 = 传感器 X/Y/Z）：")
    print(matrix_text(matrix))
    print(f"  det(R) = {det:+d}")
    print()
    print("六面记录：")
    for pose in POSES:
        st = records[pose["id"]]
        print(f"  {pose['title']:<26} 主轴 {st['sign']:+d}{st['axis']}  "
              f"均值({st['mean_g'][0]:+.3f},{st['mean_g'][1]:+.3f},{st['mean_g'][2]:+.3f})g  "
              f"{st['samples']} 点")


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def run(args):
    if args.list:
        list_ports()
        return 0

    cfg = dict(CONFIG)
    if args.acc_lsb_per_g is not None:
        cfg["lsb_per_g"] = args.acc_lsb_per_g
    if args.window is not None:
        cfg["window_sec"] = args.window

    port = pick_port(args.port)
    print("=" * 70)
    print("T-3488 传感器轴 → 机体系 FRD 映射确认")
    print("  机体系：+X 向前，+Y 向右，+Z 向下（右手 FRD）")
    print("  只接收 UART，不向飞机发送任何指令")
    print("  请先关闭串口监视器释放 COM 口；电机断开、飞控保持上锁")
    print("  接下来按提示摆 6 个姿态：+Z、+Y、+X、-Z、-Y、-X 依次朝下")
    print("=" * 70)

    src = SerialSource(port, args.baud)
    reader = FrameReader(src)
    reader.start()

    try:
        while True:
            records = collect_all(reader, cfg)
            if records is None:
                print("已退出，未生成映射文件")
                return 1

            matrix, mapping, det, problems = build_mapping(records)
            if not problems:
                break

            print()
            print("× 映射校验未通过：")
            for msg, _ids in problems:
                print("  -", msg)
            redo = sorted({i for _msg, ids in problems for i in ids})
            print("  需要复核的姿态：" + "、".join(redo))
            ans = input("  按 Enter 重采这些姿态，输入 q 退出：").strip().lower()
            if ans == "q":
                print("已退出，未生成映射文件")
                return 1
            sub = collect_all(reader, cfg, only_ids=set(redo))
            if sub is None:
                print("已退出，未生成映射文件")
                return 1
            records.update(sub)
    except KeyboardInterrupt:
        print("\n已中断，未生成映射文件")
        return 1
    finally:
        reader.stop()
        src.close()

    print_result(records, matrix, mapping, det)
    out_path = unique_path(args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "data", "imu_axis_map_frd.json"))
    data = write_json(out_path, records, matrix, mapping, det)
    print()
    print(f"映射文件：{os.path.abspath(out_path)}")
    print("说明：该文件只描述轴/符号；加速度计零偏与比例系数请用 "
          "accel_six_face_cal.py 单独标定，两者都完成后按 v_frd = R * v_calibrated 使用。")
    return 0


def main():
    ap = argparse.ArgumentParser(description="T-3488 传感器轴 → 机体系 FRD 映射确认")
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", help="串口号，例如 COM12；不填则自动选择")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--out", help="映射 JSON 路径（默认 ../data/imu_axis_map_frd.json，同名不覆盖）")
    ap.add_argument("--window", type=float, help="每个姿态的采样时长秒（默认 1.2）")
    ap.add_argument("--acc-lsb-per-g", type=float, help="加速度灵敏度（默认 2048）")
    args = ap.parse_args()
    sys.exit(run(args))


if __name__ == "__main__":
    main()
