#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面 UART 诊断（YAW 文本）解析、CSV 与 GUI。

只处理电脑端，不复用飞控固件；串口源和旧监视模式由 uart_sensor_monitor.py 提供。
"""
import csv
import math
import queue
import re
import threading
import time
from collections import deque

import numpy as np

YAW_RE = re.compile(
    r"^YAW\s+t=(-?\d+)\s+th=(-?\d+)\s+y=(-?\d+)\s+"
    r"tr=(-?\d+),(-?\d+),(-?\d+)\s+"
    r"gy=(-?\d+),(-?\d+),(-?\d+)\s+"
    r"co=(-?\d+),(-?\d+),(-?\d+)\s+"
    r"p=(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s+"
    r"q=(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s+"
    r"j=(-?\d+),(-?\d+),(-?\d+)\s+"
    r"a=([01])\s+l=([01])\s*$"
)

CSV_FIELDS = [
    "board_ms", "throttle", "yaw_stick",
    "target_roll_dps", "target_pitch_dps", "target_yaw_dps",
    "gyro_roll_dps", "gyro_pitch_dps", "gyro_yaw_dps",
    "corr_roll", "corr_pitch", "corr_yaw",
    "pwm_front", "pwm_rear", "pwm_left", "pwm_right",
    "pwm_yaw_diff",
    "q_w", "q_x", "q_y", "q_z",
    "roll_deg", "pitch_deg", "yaw_deg", "yaw_unwrapped_deg",
    "jitter_x", "jitter_y", "jitter_z",
    "armed", "limited",
]


def q10_rad_s_to_dps(value):
    """飞控 Q10 rad/s 转 °/s。"""
    return value * (180.0 / math.pi) / 1024.0


def quat_q15_to_euler_deg(q):
    """Q15 四元数 [w,x,y,z] 转横滚/俯仰/偏航，约定与固件一致。"""
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n < 1e-6:
        return None
    w, x, y, z = [float(v) / n for v in q]
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r10 = 2.0 * (x * y + w * z)
    r20 = 2.0 * (x * z - w * y)
    r21 = 2.0 * (y * z + w * x)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(r21, r22))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -r20))))
    yaw = math.degrees(math.atan2(r10, r00))
    return roll, pitch, yaw


def unwrap_yaw(previous, yaw):
    """把 ±180° 的偏航角连续展开；previous=None 时原样返回。"""
    if previous is None:
        return yaw
    delta = (yaw - previous + 180.0) % 360.0 - 180.0
    return previous + delta


def parse_ground_line(line):
    """解析一行 YAW；残缺/未知行返回 None，不抛出。"""
    if not line:
        return None
    m = YAW_RE.match(line.strip())
    if not m:
        return None
    g = [int(m.group(i)) for i in range(1, 26)]
    p = {
        "board_ms": g[0], "throttle": g[1], "yaw_stick": g[2],
        "target": [q10_rad_s_to_dps(v) for v in g[3:6]],
        "gyro": [q10_rad_s_to_dps(v) for v in g[6:9]],
        "corr": g[9:12],
        "pwm": g[12:16],
        "q": g[16:20],
        "jitter": g[20:23],
        "armed": g[23], "limited": g[24],
    }
    angles = quat_q15_to_euler_deg(p["q"])
    if angles is None:
        return None
    p["roll"], p["pitch"], p["yaw"] = angles
    return p


def ground_frame_to_row(frame, unwrapped_yaw):
    pwm = frame["pwm"]
    return {
        "board_ms": frame["board_ms"], "throttle": frame["throttle"],
        "yaw_stick": frame["yaw_stick"],
        "target_roll_dps": f"{frame['target'][0]:.2f}",
        "target_pitch_dps": f"{frame['target'][1]:.2f}",
        "target_yaw_dps": f"{frame['target'][2]:.2f}",
        "gyro_roll_dps": f"{frame['gyro'][0]:.2f}",
        "gyro_pitch_dps": f"{frame['gyro'][1]:.2f}",
        "gyro_yaw_dps": f"{frame['gyro'][2]:.2f}",
        "corr_roll": frame["corr"][0], "corr_pitch": frame["corr"][1],
        "corr_yaw": frame["corr"][2],
        "pwm_front": pwm[0], "pwm_rear": pwm[1],
        "pwm_left": pwm[2], "pwm_right": pwm[3],
        "pwm_yaw_diff": pwm[2] + pwm[3] - pwm[0] - pwm[1],
        "q_w": frame["q"][0], "q_x": frame["q"][1],
        "q_y": frame["q"][2], "q_z": frame["q"][3],
        "roll_deg": f"{frame['roll']:.2f}", "pitch_deg": f"{frame['pitch']:.2f}",
        "yaw_deg": f"{frame['yaw']:.2f}",
        "yaw_unwrapped_deg": f"{unwrapped_yaw:.2f}",
        "jitter_x": frame["jitter"][0], "jitter_y": frame["jitter"][1],
        "jitter_z": frame["jitter"][2],
        "armed": frame["armed"], "limited": frame["limited"],
    }


class GroundDiagWorker(threading.Thread):
    """读串口、按行解析、写 CSV；坏行计数后跳过。"""

    def __init__(self, source, csv_path, frame_queue):
        super().__init__(daemon=True)
        self.source = source
        self.csv_path = csv_path
        self.q = frame_queue
        self.stop_event = threading.Event()
        self.frames = 0
        self.bad = 0
        self.dropped = 0
        self._prev_yaw = None

    def stop(self):
        self.stop_event.set()

    def run(self):
        buf = b""
        with open(self.csv_path, "w", newline="", encoding="utf-8-sig") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()
            while not self.stop_event.is_set():
                try:
                    chunk = self.source.read()
                except Exception:
                    break
                if not chunk:
                    time.sleep(0.01)
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", "ignore").strip("\r")
                    frame = parse_ground_line(line)
                    if frame is None:
                        if line:
                            self.bad += 1
                        continue
                    self.frames += 1
                    unwrapped = unwrap_yaw(self._prev_yaw, frame["yaw"])
                    self._prev_yaw = unwrapped
                    writer.writerow(ground_frame_to_row(frame, unwrapped))
                    fp.flush()
                    frame["yaw_unwrapped"] = unwrapped
                    if self.q.full():
                        try:
                            self.q.get_nowait()
                            self.dropped += 1
                        except queue.Empty:
                            pass
                    self.q.put(frame)
        try:
            self.source.close()
        except Exception:
            pass


class GroundDiagMonitor:
    """四组实时曲线；GUI 只消费最近数据，CSV 不受界面丢帧影响。"""

    def __init__(self, worker, frame_queue, duration, window):
        import tkinter as tk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure

        self.tk = tk
        self.worker = worker
        self.q = frame_queue
        self.duration = duration
        self.window = max(2.0, float(window))
        self.t0 = time.time()
        self.data = deque(maxlen=2000)
        self.root = tk.Tk()
        self.root.title("Robin 24G 地面 UART 诊断")
        self.status = tk.StringVar(value="等待 YAW 数据…")
        tk.Label(self.root, textvariable=self.status, anchor="w",
                 font=("Consolas", 11)).pack(fill="x", padx=6, pady=4)
        self.fig = Figure(figsize=(12, 8.5), dpi=100)
        self.ax = [self.fig.add_subplot(4, 1, i + 1) for i in range(4)]
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<KeyPress-q>", lambda e: self.close())
        self.root.after(100, self.tick)

    def close(self):
        self.worker.stop()
        self.root.quit()
        self.root.destroy()

    def tick(self):
        while True:
            try:
                self.data.append(self.q.get_nowait())
            except queue.Empty:
                break
        if self.data:
            self.draw()
        if self.duration and time.time() - self.t0 >= self.duration:
            self.close()
            return
        self.root.after(100, self.tick)

    def draw(self):
        rows = list(self.data)
        t = np.array([r["board_ms"] / 1000.0 for r in rows])
        pwm = np.array([r["pwm"] for r in rows], dtype=float)
        jit = np.array([r["jitter"] for r in rows], dtype=float)
        last = rows[-1]
        self.status.set(
            f"t={last['board_ms']}ms  thr={last['throttle']}  yaw_stick={last['yaw_stick']}  "
            f"armed={last['armed']}  limited={last['limited']}  "
            f"pwm_diff(L+R-F-R)={int(pwm[-1,2]+pwm[-1,3]-pwm[-1,0]-pwm[-1,1])}")

        ax = self.ax[0]
        ax.clear()
        ax.plot(t, [r["roll"] for r in rows], label="roll")
        ax.plot(t, [r["pitch"] for r in rows], label="pitch")
        ax.plot(t, [r["yaw_unwrapped"] for r in rows], label="yaw unwrapped")
        ax.set_ylabel("deg")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = self.ax[1]
        ax.clear()
        ax.plot(t, [r["target"][2] for r in rows], label="target yaw dps")
        ax.plot(t, [r["gyro"][2] for r in rows], label="gyro yaw dps")
        ax.set_ylabel("dps")
        ax.legend(loc="upper left", fontsize=8)
        ax2 = ax.twinx()
        ax2.plot(t, [r["corr"][2] for r in rows], color="tab:orange",
                 alpha=0.6, label="yaw corr")
        ax2.set_ylabel("corr")
        ax2.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = self.ax[2]
        ax.clear()
        labels = ["front", "rear", "left", "right"]
        for i, lab in enumerate(labels):
            ax.plot(t, pwm[:, i], label=lab)
        ax.plot(t, pwm[:, 2] + pwm[:, 3] - pwm[:, 0] - pwm[:, 1],
                "k--", label="L+R-F-R")
        ax.set_ylabel("PWM")
        ax.legend(loc="upper right", ncol=5, fontsize=7)
        ax.grid(True, alpha=0.3)

        ax = self.ax[3]
        ax.clear()
        for i, lab in enumerate(("X", "Y", "Z")):
            ax.plot(t, jit[:, i], label=f"jitter {lab}")
        ax.set_ylabel("acc pp")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
        armed = [r["armed"] for r in rows]
        for i in range(1, len(rows)):
            if armed[i] != armed[i - 1]:
                ax.axvline(t[i], color="tab:red" if armed[i] else "0.5",
                           alpha=0.35, linewidth=0.8)
        if last["limited"]:
            ax.text(0.01, 0.92, "LIMITED", transform=ax.transAxes,
                    color="tab:red", fontsize=9)

        if t[-1] > self.window:
            for a in self.ax:
                a.set_xlim(t[-1] - self.window, t[-1])
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def run(self):
        self.root.mainloop()


def euler_deg_to_quat_q15(roll, pitch, yaw):
    """仅 --simulate 使用：欧拉角转 Q15 四元数。"""
    r = math.radians(roll) / 2
    p = math.radians(pitch) / 2
    y = math.radians(yaw) / 2
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
         cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
    return [int(round(v * 32768)) for v in q]


def run_ground_diag(args, source_factory, port_picker):
    """--ground-diag 入口；source_factory/pick 由旧监视器传入以复用串口处理。"""
    csv_path = args.csv or f"ground_diag_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    port = port_picker(args.port) if not args.simulate else "模拟 YAW"
    if args.simulate:
        class _Sim:
            def __init__(self):
                self.i = 0
            def read(self):
                time.sleep(0.08)
                self.i += 1
                y = 170.0 if (self.i // 15) % 2 else -170.0
                q = euler_deg_to_quat_q15(10, -5, y)
                t = self.i * 100
                return (f"YAW t={t} th=180 y={int(120*math.sin(self.i/10))} "
                        f"tr=0,0,{int(20*math.sin(self.i/10))} "
                        f"gy=0,0,{-int(20*math.sin(self.i/10))} "
                        f"co=0,0,{int(8*math.sin(self.i/10))} "
                        f"p=150,150,160,160 "
                        f"q={q[0]},{q[1]},{q[2]},{q[3]} "
                        f"j={self.i%20},{self.i%15},{self.i%25} "
                        f"a={1 if self.i>10 else 0} l=0\n").encode()
            def close(self):
                pass
        source = _Sim()
    else:
        source = source_factory(port, args.baud)

    frames = queue.Queue(maxsize=2000)
    worker = GroundDiagWorker(source, csv_path, frames)
    print(f"数据源：{port}（地面 YAW 10Hz）")
    print(f"CSV：{csv_path}")
    if args.no_plot:
        worker.start()
        end = time.time() + args.duration if args.duration else None
        try:
            while end is None or time.time() < end:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        worker.stop()
        worker.join(2.0)
        print(f"YAW 帧 {worker.frames}，坏行 {worker.bad}，界面丢帧 {worker.dropped}")
        return
    worker.start()
    gui = GroundDiagMonitor(worker, frames, args.duration, args.window)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()
        worker.join(2.0)
    print(f"YAW 帧 {worker.frames}，坏行 {worker.bad}，界面丢帧 {worker.dropped}")
