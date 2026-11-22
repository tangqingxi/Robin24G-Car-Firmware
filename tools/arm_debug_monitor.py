#!/usr/bin/env python3
"""
Robin_24G_Car 200Hz 飞控专用解锁诊断上位机。

默认打开 Tk 图形窗口；--no-plot 退化为控制台 + CSV。
固件协议：ADB 启动、ADS 1Hz 摘要、ADU 0002 尝试、ADO 解锁后首拍、
ADL 解锁后锁回、ADE 急停。
"""
from __future__ import annotations

import argparse
import csv
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

CSV_FIELDS = [
    "board_ms", "host_time", "event", "attempt",
    "raw_thr", "cmd_throttle", "cmd_yaw", "cmd_pitch", "cmd_roll",
    "packet_valid", "thr_low", "center", "att_ready", "link_ok",
    "accepted", "why", "armed", "lock_reason", "pwm_ready",
    "pwm_front", "pwm_rear", "pwm_left", "pwm_right",
    "rx", "valid", "heartbeat", "e2", "still_ms", "boot_st", "raw_line",
]

_HEX_RE = re.compile(r"^[0-9A-Fa-f]+$")


def _to_int(text: str, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(text, 0)
    except (TypeError, ValueError):
        return default


def _hex4(text: str) -> int:
    return int(text, 16) if text and _HEX_RE.match(text) else 0


def parse_kv(line: str) -> tuple[str, dict[str, str]]:
    parts = line.strip().split()
    if not parts:
        return "", {}
    kind = parts[0]
    data = {}
    for item in parts[1:]:
        if "=" in item:
            key, value = item.split("=", 1)
            data[key] = value
    return kind, data


def pwm_values(text: str) -> list[int]:
    out = [0, 0, 0, 0]
    parts = text.split("/")
    for i in range(min(4, len(parts))):
        out[i] = _hex4(parts[i])
    return out


def cmd_values(text: str) -> list[int]:
    out = [0, 0, 0, 0]
    parts = text.split("/")
    for i in range(min(4, len(parts))):
        try:
            out[i] = int(parts[i], 10)
        except ValueError:
            out[i] = 0
    return out


def why_text(why: str, cmd: list[int]) -> str:
    if why == "pkt":
        return "包无效/摇杆无效"
    if why == "estop":
        return "同包急停优先"
    if why == "thr":
        return f"油门过高（{cmd[0]}，要求 ≤ -250）"
    if why == "roll":
        return f"横滚偏离中位（{cmd[3]}）"
    if why == "pitch":
        return f"俯仰偏离中位（{cmd[2]}）"
    if why == "yaw":
        return f"偏航偏离中位（{cmd[1]}）"
    if why == "att":
        return "姿态未就绪"
    if why == "link":
        return "链路无效"
    return "无"


@dataclass
class ArmDebugState:
    boot_st: Optional[int] = None
    still_ms: int = 0
    att_ready: Optional[int] = None
    pwm_ready: Optional[int] = None
    rx: int = 0
    valid: int = 0
    heartbeat: int = 0
    e2: int = 0
    attempts: int = 0
    armed: int = 0
    lock_reason: int = 0
    pwm: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    last_attempt: Optional[dict] = None
    last_output: Optional[dict] = None
    last_lock: Optional[dict] = None
    seen_boot: bool = False
    seen_serial: bool = False
    events: deque = field(default_factory=lambda: deque(maxlen=10))
    status: str = "等待串口"

    def apply(self, line: str, host_time: str) -> dict:
        kind, data = parse_kv(line)
        record = {key: "" for key in CSV_FIELDS}
        record["host_time"] = host_time
        record["raw_line"] = line
        record["event"] = kind
        record["board_ms"] = _to_int(data.get("t"), 0) or 0

        if kind == "ADB":
            self.seen_boot = True
            self.seen_serial = True
            self.boot_st = _to_int(data.get("st"), 0) or 0
            self.still_ms = _to_int(data.get("still"), 0) or 0
            self.att_ready = _to_int(data.get("att"), 0) or 0
            self.pwm_ready = _to_int(data.get("pwm"), 0) or 0
            record.update({
                "boot_st": self.boot_st, "still_ms": self.still_ms,
                "att_ready": self.att_ready, "pwm_ready": self.pwm_ready,
            })
            self.events.append(f"[{record['board_ms']}] 启动 st={self.boot_st} "
                               f"still={self.still_ms}ms att={self.att_ready} "
                               f"pwm={self.pwm_ready}")
            if self.boot_st != 0:
                self.status = f"IMU 初始化失败（st={self.boot_st}）"
            else:
                self.status = self._status_without_e2()

        elif kind == "ADS":
            self.seen_serial = True
            self.still_ms = _to_int(data.get("still"), self.still_ms) or 0
            self.att_ready = _to_int(data.get("att"), self.att_ready)
            self.pwm_ready = _to_int(data.get("pwm"), self.pwm_ready)
            self.rx = _to_int(data.get("rx"), self.rx) or 0
            self.valid = _to_int(data.get("vld"), self.valid) or 0
            self.heartbeat = _to_int(data.get("hb"), self.heartbeat) or 0
            self.e2 = _to_int(data.get("e2"), self.e2) or 0
            self.attempts = _to_int(data.get("n"), self.attempts) or 0
            self.armed = _to_int(data.get("arm"), self.armed) or 0
            self.lock_reason = _to_int(data.get("x"), self.lock_reason) or 0
            record.update({
                "rx": self.rx, "valid": self.valid, "heartbeat": self.heartbeat,
                "e2": self.e2, "attempt": self.attempts,
                "armed": self.armed, "lock_reason": self.lock_reason,
                "still_ms": self.still_ms, "att_ready": self.att_ready,
                "boot_st": self.boot_st,
            })
            if self.e2 == 0:
                self.status = self._status_without_e2()
            elif self.last_lock is not None and not self.armed:
                self.status = f"已解锁后又锁定（原因 {self.last_lock['reason']}）"
            elif self.armed:
                self.status = "已解锁（等待下一拍 PWM 输出）"
            elif self.last_attempt and self.last_attempt.get("accepted") == 0:
                self.status = "收到 0002 但被拒绝：" + why_text(
                    self.last_attempt["why"], self.last_attempt["cmd"])
            else:
                self.status = "已收到 0002，等待解锁结果"

        elif kind == "ADU":
            self.seen_serial = True
            self.seen_boot = True
            self.e2 = 1
            cmd = cmd_values(data.get("cmd", "0/0/0/0"))
            accepted = _to_int(data.get("ok"), 0) or 0
            why = data.get("why", "none")
            self.attempts = _to_int(data.get("n"), self.attempts) or 0
            if accepted:
                self.armed = 1
            attempt = {
                "n": self.attempts, "raw_thr": _hex4(data.get("rt", "0")),
                "cmd": cmd, "accepted": accepted, "why": why,
                "packet_valid": _to_int(data.get("pkt"), 0) or 0,
                "thr_low": _to_int(data.get("low"), 0) or 0,
                "center": _to_int(data.get("ctr"), 0) or 0,
                "att_ready": _to_int(data.get("att"), 0) or 0,
                "link_ok": _to_int(data.get("lk"), 0) or 0,
            }
            self.last_attempt = attempt
            record.update({
                "attempt": self.attempts, "raw_thr": attempt["raw_thr"],
                "cmd_throttle": cmd[0], "cmd_yaw": cmd[1],
                "cmd_pitch": cmd[2], "cmd_roll": cmd[3],
                "packet_valid": attempt["packet_valid"],
                "thr_low": attempt["thr_low"], "center": attempt["center"],
                "att_ready": attempt["att_ready"], "link_ok": attempt["link_ok"],
                "accepted": accepted, "why": why, "armed": self.armed,
            })
            if accepted:
                self.status = "收到 0002，条件全部通过，已接受"
                self.events.append(f"[{record['board_ms']}] 尝试 #{self.attempts} 接受")
            else:
                reason = why_text(why, cmd)
                self.status = f"收到 0002 但被拒绝：{reason}"
                self.events.append(f"[{record['board_ms']}] 尝试 #{self.attempts} "
                                   f"拒绝：{reason}")
            self._event_line(record, f"尝试 #{self.attempts} "
                                    f"{'接受' if accepted else '拒绝'}")

        elif kind == "ADO":
            self.seen_serial = True
            self.armed = _to_int(data.get("arm"), 0) or 0
            self.lock_reason = _to_int(data.get("x"), 0) or 0
            self.pwm = pwm_values(data.get("pwm", "0/0/0/0"))
            output = {
                "n": _to_int(data.get("n"), 0) or 0,
                "att_updated": _to_int(data.get("up"), 0) or 0,
                "ctrl_ok": _to_int(data.get("ctrl"), 0) or 0,
                "armed": self.armed, "reason": self.lock_reason,
                "pwm": self.pwm,
            }
            self.last_output = output
            record.update({
                "attempt": output["n"], "armed": self.armed,
                "lock_reason": self.lock_reason,
                "pwm_front": self.pwm[0], "pwm_rear": self.pwm[1],
                "pwm_left": self.pwm[2], "pwm_right": self.pwm[3],
            })
            if self.armed:
                self.status = "已解锁；PWM 指令已输出（是否转动看台架）"
            else:
                self.status = f"解锁后下一拍已锁定（原因 {self.lock_reason}）"
            self.events.append(f"[{record['board_ms']}] 首拍 "
                               f"up={output['att_updated']} ctrl={output['ctrl_ok']} "
                               f"arm={self.armed} x={self.lock_reason} "
                               f"pwm={self.pwm}")
            self._event_line(record, f"首拍 arm={self.armed} x={self.lock_reason}")

        elif kind == "ADL":
            self.seen_serial = True
            self.armed = 0
            self.lock_reason = _to_int(data.get("x"), 0) or 0
            self.pwm = pwm_values(data.get("pwm", "0/0/0/0"))
            lock = {"n": _to_int(data.get("n"), 0) or 0,
                    "reason": self.lock_reason, "pwm": self.pwm}
            self.last_lock = lock
            record.update({
                "attempt": lock["n"], "armed": 0,
                "lock_reason": self.lock_reason,
                "pwm_front": self.pwm[0], "pwm_rear": self.pwm[1],
                "pwm_left": self.pwm[2], "pwm_right": self.pwm[3],
            })
            self.status = f"已解锁后又锁定（原因 {self.lock_reason}）"
            self.events.append(f"[{record['board_ms']}] 锁回 x={self.lock_reason} "
                               f"pwm={self.pwm}")
            self._event_line(record, f"锁回 x={self.lock_reason}")

        elif kind == "ADE":
            self.seen_serial = True
            self.armed = 0
            self.lock_reason = 5
            self.pwm = pwm_values(data.get("pwm", "0/0/0/0"))
            record.update({
                "attempt": _to_int(data.get("n"), 0) or 0,
                "armed": 0, "lock_reason": 5,
                "pwm_front": self.pwm[0], "pwm_rear": self.pwm[1],
                "pwm_left": self.pwm[2], "pwm_right": self.pwm[3],
            })
            self.status = "已急停，四路 PWM 指令清零"
            self.events.append(f"[{record['board_ms']}] 急停 pwm={self.pwm}")
            self._event_line(record, "急停")

        elif kind.startswith("AD"):
            self.seen_serial = True
            self.events.append(f"[{record['board_ms']}] {line.strip()}")
        else:
            return {}

        return record

    def _status_without_e2(self) -> str:
        if not self.seen_serial:
            return "等待串口"
        if self.boot_st not in (None, 0):
            return f"IMU 初始化失败（st={self.boot_st}）"
        if self.e2 == 0:
            return "有串口状态但尚未收到 0002"
        return "已收到 0002，等待解锁结果"

    def _event_line(self, record: dict, text: str) -> None:
        # 尝试/输出/锁回已经写入 events；这里只保证 CSV 字段完整
        record["event"] = record.get("event", "")


def make_csv_writer(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fp = path.open("w", newline="", encoding="utf-8-sig")
    writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
    writer.writeheader()
    fp.flush()
    return fp, writer


def write_record(writer, fp, record: dict) -> None:
    if not record:
        return
    writer.writerow(record)
    fp.flush()


class SimArmSource:
    """无硬件模拟：启动、无 0002、拒绝、接受、首拍、锁回。"""

    def __init__(self):
        self.lines = [
            "ADB t=100 st=0 still=0 att=0 pwm=1",
            "ADS t=500 st=0 still=0 att=0 pwm=1 rx=10 vld=8 hb=2 e2=0 n=0 arm=0 x=1 drop=0",
            "ADS t=1500 st=0 still=1200 att=1 pwm=1 rx=30 vld=26 hb=4 e2=0 n=0 arm=0 x=1 drop=0",
            "ADU t=2000 n=1 rt=0400 cmd=-180/0/0/0 pkt=1 low=0 ctr=1 att=1 lk=1 ok=0 why=thr",
            "ADU t=2500 n=2 rt=0400 cmd=-307/0/0/0 pkt=1 low=1 ctr=1 att=1 lk=1 ok=1 why=none",
            "ADO t=2510 n=2 up=1 ctrl=1 arm=1 x=0 pwm=064/064/064/064",
            "ADS t=3500 st=0 still=1200 att=1 pwm=1 rx=60 vld=56 hb=4 e2=1 n=2 arm=1 x=0 drop=0",
            "ADL t=4000 n=2 x=2 pwm=000/000/000/000",
            "ADE t=4500 n=2 x=5 pwm=000/000/000/000",
        ]
        self.index = 0
        self.next_time = time.time()

    def read(self):
        if self.index >= len(self.lines):
            time.sleep(0.05)
            return b""
        now = time.time()
        if now < self.next_time:
            time.sleep(0.01)
            return b""
        line = self.lines[self.index]
        self.index += 1
        self.next_time = now + 0.35
        return (line + "\n").encode("ascii")

    def close(self):
        pass


def open_source(args):
    if getattr(args, "simulate", False):
        return SimArmSource()
    if serial is None:
        raise SystemExit("缺少 pyserial：python -m pip install pyserial")
    port = args.port
    if not port:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            raise SystemExit("没有可选串口，请用 --port COM12")
        port = ports[0]
    try:
        return serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        raise SystemExit(f"打开串口 {port} 失败：{exc}")


class LineReader(threading.Thread):
    def __init__(self, source, out_queue):
        super().__init__(daemon=True)
        self.source = source
        self.out_queue = out_queue
        self.stop_flag = threading.Event()
        self.buffer = bytearray()

    def run(self):
        while not self.stop_flag.is_set():
            try:
                data = self.source.read()
            except Exception as exc:
                self.out_queue.put(("error", str(exc)))
                return
            if not data:
                continue
            self.buffer.extend(data)
            while b"\n" in self.buffer:
                raw, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.out_queue.put(("line", line))

    def stop(self):
        self.stop_flag.set()


def run_console(args, source, csv_path: Path) -> int:
    state = ArmDebugState()
    fp, writer = make_csv_writer(csv_path)
    start = time.time()
    buffer = bytearray()
    try:
        while True:
            if args.duration and time.time() - start >= args.duration:
                break
            data = source.read()
            if data:
                buffer.extend(data)
                while b"\n" in buffer:
                    raw, _, rest = buffer.partition(b"\n")
                    buffer = bytearray(rest)
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    host = datetime.now().isoformat(timespec="milliseconds")
                    record = state.apply(line, host)
                    write_record(writer, fp, record)
                    print(f"\r{state.status} | {line}"[:160])
            else:
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        fp.close()
    print()
    print(f"CSV：{csv_path}")
    return 0


def run_gui(args, source, csv_path: Path) -> int:
    import tkinter as tk
    from tkinter import ttk

    state = ArmDebugState()
    fp, writer = make_csv_writer(csv_path)
    q = queue.Queue()
    reader = LineReader(source, q)
    reader.start()

    root = tk.Tk()
    root.title("Robin_24G_Car 200Hz 解锁诊断")
    root.geometry("980x620")
    root.minsize(760, 500)

    title = tk.Label(root, text="等待串口", anchor="w", font=("Microsoft YaHei", 13, "bold"))
    title.pack(fill="x", padx=10, pady=(8, 2))

    body = ttk.Frame(root)
    body.pack(fill="both", expand=True, padx=10, pady=4)

    def make_box(parent, title_text, row, col):
        box = ttk.LabelFrame(parent, text=title_text)
        box.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        label = tk.Label(box, text="—", anchor="nw", justify="left",
                         wraplength=440, font=("Microsoft YaHei", 10))
        label.pack(fill="both", expand=True, padx=6, pady=6)
        return label

    body.columnconfigure(0, weight=1)
    body.columnconfigure(1, weight=1)
    body.rowconfigure(0, weight=1)
    body.rowconfigure(1, weight=1)
    imu_label = make_box(body, "1. IMU 与姿态", 0, 0)
    rc_label = make_box(body, "2. 遥控与按键", 0, 1)
    unlock_label = make_box(body, "3. 解锁结果", 1, 0)
    pwm_label = make_box(body, "4. PWM 指令", 1, 1)

    events = tk.Listbox(root, height=8, font=("Consolas", 9))
    events.pack(fill="x", padx=14, pady=(2, 4))

    footer = tk.Label(root, text="", anchor="w", fg="#555")
    footer.pack(fill="x", padx=10, pady=(0, 4))

    start = time.time()
    closed = False

    def close():
        nonlocal closed
        if closed:
            return
        closed = True
        reader.stop()
        try:
            reader.join(timeout=0.5)
        except Exception:
            pass
        source.close()
        fp.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)

    def update_view():
        if closed:
            return
        try:
            while True:
                kind, payload = q.get_nowait()
                if kind == "error":
                    state.status = f"串口错误：{payload}"
                    continue
                host = datetime.now().isoformat(timespec="milliseconds")
                record = state.apply(payload, host)
                write_record(writer, fp, record)
        except queue.Empty:
            pass

        title.config(text=state.status)
        if state.boot_st is not None:
            imu_text = (
                f"IMU 初始化：{'成功' if state.boot_st == 0 else '失败'}（st={state.boot_st}）\n"
                f"静置累计：{state.still_ms} ms\n"
                f"姿态就绪：{'是' if state.att_ready else '否'}\n"
                f"PWM 初始化：{'完成' if state.pwm_ready else '未完成'}"
            )
        else:
            imu_text = "尚未收到启动状态"
        imu_label.config(text=imu_text)

        rc_text = (
            f"收包：{state.rx} 个\n有效摇杆包：{state.valid} 个\n"
            f"全零心跳：{state.heartbeat} 个\n"
            f"0002：{'已收到' if state.e2 else '尚未收到'}\n"
            f"解锁尝试：{state.attempts} 次"
        )
        rc_label.config(text=rc_text)

        if state.last_attempt:
            a = state.last_attempt
            cmd = a["cmd"]
            result = "接受" if a["accepted"] else "拒绝：" + why_text(a["why"], cmd)
            unlock_text = (
                f"尝试 #{a['n']}：{result}\n"
                f"原始油门：0x{a['raw_thr']:04X}\n"
                f"映射四轴：T={cmd[0]} Y={cmd[1]} P={cmd[2]} R={cmd[3]}\n"
                f"包有效={a['packet_valid']} 低油门={a['thr_low']} "
                f"回中={a['center']}\n"
                f"姿态={a['att_ready']} 链路={a['link_ok']}"
            )
        else:
            unlock_text = ("有串口状态但尚未收到 0002"
                           if state.seen_serial else "等待串口")
        if state.last_lock:
            unlock_text += f"\n最近锁回：x={state.last_lock['reason']}"
        unlock_label.config(text=unlock_text)

        pwm_text = (
            "PWM 指令（是否旋转看台架）：\n"
            f"前：{state.pwm[0]:03d}（0x{state.pwm[0]:03X}）\n"
            f"后：{state.pwm[1]:03d}（0x{state.pwm[1]:03X}）\n"
            f"左：{state.pwm[2]:03d}（0x{state.pwm[2]:03X}）\n"
            f"右：{state.pwm[3]:03d}（0x{state.pwm[3]:03X}）"
        )
        pwm_label.config(text=pwm_text)

        current = list(state.events)
        if current != list(events.get(0, tk.END)):
            events.delete(0, tk.END)
            for ev in current:
                events.insert(tk.END, ev)

        footer.config(text=f"CSV：{csv_path}   |   主机时间：{datetime.now():%H:%M:%S}")
        if args.duration and time.time() - start >= args.duration:
            close()
            return
        root.after(100, update_view)

    root.after(100, update_view)
    try:
        root.mainloop()
    finally:
        close()
    return 0


def run_arm_debug(args) -> int:
    csv_path = Path(args.csv) if args.csv else Path(
        f"arm_debug_{datetime.now():%Y%m%d_%H%M%S}.csv")
    source = open_source(args)
    try:
        if args.no_plot:
            return run_console(args, source, csv_path)
        return run_gui(args, source, csv_path)
    except Exception:
        source.close()
        raise


def main() -> int:
    ap = argparse.ArgumentParser(description="200Hz 飞控解锁诊断")
    ap.add_argument("--port", help="串口号，例如 COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="CSV 路径")
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--no-plot", action="store_true", help="控制台备用模式")
    ap.add_argument("--simulate", action="store_true", help="模拟数据源")
    args = ap.parse_args()
    return run_arm_debug(args)


if __name__ == "__main__":
    raise SystemExit(main())
