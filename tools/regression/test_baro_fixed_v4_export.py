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
assert stats["version"] == 4 and stats["calibration"] is not None
expected = {(t, p, temp): ex.reference_v4(p, temp, stats["calibration"])[0]
            for t, p, temp in rows if p is not None and temp is not None}
with open("baro_v4_c.csv", encoding="utf-8", newline="") as fp:
    crows = list(csv.DictReader(fp))
actual = {(int(row["session_ms"]), int(row["raw_pressure"]), int(row["raw_temperature"])):
          int(row["pressure_01pa"]) / 10.0 for row in crows}
assert len(crows) == len(actual) == len(expected) == 147 and actual.keys() == expected.keys()
assert abs(expected[next(iter(expected))] - 101769.4804) < 0.001
max_err = max(abs(actual[key] - expected[key]) for key in expected)
if max_err > 0.2:
    raise SystemExit(f"FAIL max_abs_error={max_err:.4f}Pa exceeds 0.2Pa")
print(f"PASS v4 actual coefficients: rows={len(actual)} max_abs_error={max_err:.4f}Pa")
print("RESULT OK")
