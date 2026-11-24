#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实际 v4 Read.bin：Python 浮点 vs C 定点逐条绝对误差。"""
import csv
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import baro_bdt_export as ex

data = Path("v4_actual.bin").read_bytes()
rows, stats = ex.parse_record(data)
expected = {t: ex.reference_v4(p, temp, stats["calibration"])[0]
            for t, p, temp in rows}
crows = list(csv.DictReader(open("baro_v4_c.csv", encoding="utf-8")))
max_err = 0.0
compared = 0
for row in crows:
    t = int(row["session_ms"])
    if t not in expected or expected[t] is None:
        continue
    err = abs(float(row["pressure_01pa"]) / 10.0 - expected[t])
    max_err = max(max_err, err)
    compared += 1
print(f"PASS v4 actual coefficients: rows={compared} max_abs_error={max_err:.4f}Pa")
if max_err > 0.2:
    raise SystemExit("FAIL error over 0.2Pa")
print("RESULT OK")
