#!/usr/bin/env python3
"""解锁诊断 GUI/CSV 状态机回归（无 Tk、无串口）。"""
import csv
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))

import arm_debug_monitor as arm


def feed(state, line):
    return state.apply(line, "2026-01-01T00:00:00.000")


class ArmDebugStateTest(unittest.TestCase):
    def test_imu_init_failure(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=1 still=0 att=0 pwm=0")
        self.assertIn("IMU 初始化失败", s.status)
        self.assertEqual(s.boot_st, 1)

    def test_no_0002(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=0 still=0 att=0 pwm=1")
        feed(s, "ADS t=500 st=0 still=0 att=1 pwm=1 rx=10 vld=8 hb=2 e2=0 n=0 arm=0 x=1 drop=0")
        self.assertIn("尚未收到 0002", s.status)
        self.assertEqual(s.e2, 0)

    def test_throttle_reject(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=0 still=1200 att=1 pwm=1")
        feed(s, "ADU t=2000 n=1 rt=0400 cmd=-180/0/0/0 pkt=1 low=0 ctr=1 att=1 lk=1 ok=0 why=thr")
        self.assertIn("油门过高", s.status)
        self.assertIn("-180", s.status)
        self.assertEqual(s.last_attempt["accepted"], 0)
        self.assertEqual(s.last_attempt["why"], "thr")

    def test_unlock_then_lock(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=0 still=1200 att=1 pwm=1")
        feed(s, "ADU t=2000 n=1 rt=0400 cmd=-307/0/0/0 pkt=1 low=1 ctr=1 att=1 lk=1 ok=1 why=none")
        feed(s, "ADO t=2010 n=1 up=1 ctrl=1 arm=1 x=0 pwm=064/064/064/064")
        feed(s, "ADL t=2500 n=1 x=2 pwm=000/000/000/000")
        self.assertIn("已解锁后又锁定", s.status)
        self.assertEqual(s.armed, 0)
        self.assertEqual(s.last_lock["reason"], 2)
        self.assertTrue(any("接受" in e for e in s.events))
        self.assertTrue(any("锁回" in e for e in s.events))

    def test_normal_unlock_pwm_nonzero(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=0 still=1200 att=1 pwm=1")
        feed(s, "ADU t=2000 n=1 rt=0400 cmd=-307/0/0/0 pkt=1 low=1 ctr=1 att=1 lk=1 ok=1 why=none")
        feed(s, "ADO t=2010 n=1 up=1 ctrl=1 arm=1 x=0 pwm=064/05A/03C/032")
        self.assertEqual(s.armed, 1)
        self.assertEqual(s.pwm, [0x64, 0x5A, 0x3C, 0x32])
        self.assertIn("已解锁", s.status)


class ArmDebugImuStatsTest(unittest.TestCase):
    def test_read_counts_still_break_and_reject_accel(self):
        s = arm.ArmDebugState()
        feed(s, "ADB t=100 st=0 rb=3 still=0 att=0 pwm=1")
        feed(s, "ADI t=1000 ok=199 f=1 sm=480 b=norm bn=2 bg=0 bj=0 bp=0 bi=0")
        self.assertEqual((s.read_ok, s.read_fail, s.still_max_ms), (199, 1, 480))
        self.assertEqual(s.break_reason, "norm")
        self.assertEqual(s.break_counts["norm"], 2)
        feed(s, "ADU t=1100 n=1 rt=0400 cmd=-307/0/0/0 pkt=1 low=1 ctr=0 att=1 lk=1 ok=0 why=roll")
        feed(s, "ADG t=1100 n=1 a=FFF6/0000/FC0E g=0000/0000/0000")
        self.assertEqual(s.last_acc[0], -10)
        self.assertEqual(s.last_acc[2], -1010)
        self.assertEqual(s.last_gyro, [0, 0, 0])

    def test_csv_imu_stats_and_samples(self):
        s = arm.ArmDebugState()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arm.csv"
            fp, writer = arm.make_csv_writer(path)
            for line in [
                "ADB t=100 st=0 rb=3 still=0 att=0 pwm=1",
                "ADI t=1000 ok=199 f=1 sm=480 b=gyro bn=0 bg=1 bj=0 bp=0 bi=0",
                "ADU t=1100 n=1 rt=0400 cmd=-180/0/0/0 pkt=1 low=0 ctr=1 att=1 lk=1 ok=0 why=thr",
                "ADG t=1100 n=1 a=FFF6/0000/FC0E g=0001/0002/0003",
            ]:
                arm.write_record(writer, fp, feed(s, line))
            fp.close()
            with path.open(encoding="utf-8-sig", newline="") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(rows[1]["read_ok"], "199")
        self.assertEqual(rows[1]["break_reason"], "gyro")
        self.assertEqual(rows[3]["acc_x"], "-10")
        self.assertEqual(rows[3]["acc_z"], "-1010")
        self.assertEqual(rows[3]["gyro_z"], "3")


class ArmDebugCsvTest(unittest.TestCase):
    def test_csv_has_board_host_attempt_and_raw(self):
        s = arm.ArmDebugState()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arm.csv"
            fp, writer = arm.make_csv_writer(path)
            record = feed(
                s, "ADU t=1234 n=7 rt=0400 cmd=-307/0/0/0 pkt=1 "
                   "low=1 ctr=1 att=1 lk=1 ok=1 why=none")
            arm.write_record(writer, fp, record)
            fp.close()
            with path.open(encoding="utf-8-sig", newline="") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row["board_ms"], "1234")
        self.assertEqual(row["attempt"], "7")
        self.assertEqual(row["accepted"], "1")
        self.assertIn("ADU t=1234", row["raw_line"])
        self.assertTrue(row["host_time"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
