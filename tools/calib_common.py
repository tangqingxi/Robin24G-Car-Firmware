#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三个校准工具共用的串口、输出路径与 FRD 轴映射工具。

这里只放与具体采样/判定无关的公共代码：
  - 串口枚举、自动选口、只收数据源（含清空积压）；
  - 输出文件同名不覆盖；
  - FRD 轴映射 JSON 读取与校验；
  - 均值/标准差。

采样窗口、判稳规则、JSON 格式和单面补测仍留在各自的工具里，
避免把三套不同的标定流程硬合成一套。
"""

import json
import math
import os
import sys

# imu_axis_map.py 写出的格式标识，读取时用它确认方向
MAP_FORMAT = "robin24g_imu_axis_map_frd"


class SerialSource:
    """只读串口数据源；打开失败直接给出中文错误并退出"""

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
        """一次最多读 4096 字节；没有数据时由 pyserial 的 timeout 返回空"""
        return self.ser.read(4096)

    def drain(self):
        """丢掉按 Enter 之前积压的旧数据（陀螺标定开始前要调用）"""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def list_ports():
    """打印当前可用串口"""
    import serial.tools.list_ports
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("未发现任何串口设备")
        return
    print("可用串口：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")


def pick_port(explicit):
    """--port 优先；否则按常见 USB 串口芯片自动选，选不出就列出让用户指定"""
    if explicit:
        return explicit
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("未发现串口，请用 --port 指定")
    for p in ports:
        text = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".upper()
        if any(k in text for k in ("CH340", "CH341", "CP210", "FT232", "FTDI",
                                    "PL2303", "USB 串行")):
            print(f"自动选择串口：{p.device} ({p.description})")
            return p.device
    if len(ports) == 1:
        print(f"自动选择串口：{ports[0].device} ({ports[0].description})")
        return ports[0].device
    print("检测到多个串口，请用 --port 指定：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
    sys.exit(1)


def unique_path(path):
    """输出文件防覆盖：同名时自动加 _1、_2 后缀"""
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    i = 1
    while os.path.exists(f"{root}_{i}{ext}"):
        i += 1
    return f"{root}_{i}{ext}"


def mean_std(vals):
    """返回 (均值, 总体标准差)；空列表返回 (0.0, 0.0)"""
    n = len(vals)
    if n == 0:
        return 0.0, 0.0
    m = sum(vals) / n
    return m, math.sqrt(sum((x - m) ** 2 for x in vals) / n)


def load_axis_map(path):
    """
    读取 imu_axis_map_frd.json（传感器 → FRD），校验方向与矩阵性质。
    返回 (matrix 3×3, 原始 JSON)；不合法直接退出。

    校验比旧版加速度工具更完整：额外核对文件里记录的 determinant
    与实际行列式一致，避免手改矩阵后留下矛盾的 JSON。
    """
    if not os.path.exists(path):
        sys.exit(f"找不到轴映射文件：{path}\n  请先用 imu_axis_map.py 生成")
    try:
        with open(path, encoding="utf-8") as fp:
            data = json.load(fp)
    except json.JSONDecodeError as exc:
        sys.exit(f"轴映射文件不是合法 JSON：{path}\n  原因：{exc}")

    def bad(msg):
        sys.exit(f"轴映射文件不可用：{path}\n  原因：{msg}")

    if data.get("format") != MAP_FORMAT:
        bad(f"format 应为 {MAP_FORMAT!r}，实际 {data.get('format')!r}")
    frame = data.get("target_frame") or {}
    if frame.get("name") != "FRD":
        bad(f"target_frame.name 应为 'FRD'，实际 {frame.get('name')!r}")
    if data.get("matrix_convention", "").find("v_frd = R * v_sensor") < 0:
        bad("matrix_convention 不是“传感器 → FRD”方向（应写 v_frd = R * v_sensor）")

    matrix = data.get("matrix_sensor_to_frd")
    if not isinstance(matrix, list) or len(matrix) != 3 or any(
            not isinstance(row, list) or len(row) != 3 for row in matrix):
        bad("matrix_sensor_to_frd 必须是 3×3 数组")

    # 带符号置换矩阵：每行每列恰好一个 ±1，其余为 0
    for r in range(3):
        nz = [v for v in matrix[r] if v != 0]
        if len(nz) != 1 or abs(nz[0]) != 1:
            bad(f"矩阵第 {r+1} 行不满足“恰好一个 ±1”")
    for c in range(3):
        nz = [matrix[r][c] for r in range(3) if matrix[r][c] != 0]
        if len(nz) != 1 or abs(nz[0]) != 1:
            bad(f"矩阵第 {c+1} 列不满足“恰好一个 ±1”")

    det = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
           - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
           + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
    if det != 1:
        bad(f"矩阵行列式为 {det:+d}，必须是 +1（右手系）")
    if data.get("determinant") is not None and data["determinant"] != det:
        bad(f"文件里 determinant={data['determinant']} 与实际计算 {det} 不一致")
    return matrix, data
