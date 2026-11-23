#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面 YAW 监视器主机测试：正常/截断行、偏航展开、CSV 字段。"""
import csv
import io
import math
from pathlib import Path

import ground_diag_monitor as g


def make_line(t, yaw_deg, armed=1):
    q = g.euler_deg_to_quat_q15(5.0, -3.0, yaw_deg)
    return (f"YAW t={t} th=180 y=120 tr=0,0,512 gy=0,0,-512 "
            f"co=0,0,64 p=150,151,160,161 "
            f"q={q[0]},{q[1]},{q[2]},{q[3]} j=10,11,12 a={armed} l=1")


def main():
    line = make_line(1000, 179.0)
    f = g.parse_ground_line(line)
    assert f is not None, "正常行未解析"
    assert f["board_ms"] == 1000 and f["throttle"] == 180
    assert abs(f["target"][2] - 28.6479) < 0.1, f"Q10 rad/s 换算错: {f['target']}"
    assert g.parse_ground_line(line[:50]) is None, "截断行不应解析"
    assert g.parse_ground_line("YAW 坏行") is None

    f2 = g.parse_ground_line(make_line(1100, -179.0))
    assert f2 is not None
    y1 = g.unwrap_yaw(None, f["yaw"])
    y2 = g.unwrap_yaw(y1, f2["yaw"])
    assert abs(y2 - 181.0) < 0.2, f"偏航展开错误: {y2}"

    row = g.ground_frame_to_row(f2, y2)
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=g.CSV_FIELDS)
    w.writeheader(); w.writerow(row)
    text = buf.getvalue()
    for key in ("board_ms", "q_w", "yaw_unwrapped_deg", "jitter_z",
                "pwm_yaw_diff", "armed"):
        assert key in text
    print("PASS ground diag parser/truncated/wrap/CSV")
    print("RESULT OK")


if __name__ == "__main__":
    main()
