#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_sensor_monitor.py 的 L400 模式自测。

覆盖：
  1. 7d01167 固件实际输出的 L400 / L400S / init 行解析；
  2. Q10 rad/s → °/s 换算；
  3. 串口分块到达时 L400Worker 的拆行、两份 CSV 与统计；
  4. 模拟源在 10Hz / 1Hz 下产生可解析的文本；
  5. --l400 --simulate --no-plot 端到端写 CSV。

用法（在 tools 目录下）：
  python test_l400_monitor.py
"""

import csv
import os
import queue
import subprocess
import sys
import tempfile
import time
import unittest

import uart_sensor_monitor as mon

# 7d01167 联调固件的实际输出样例（字段顺序与固件 l400_put_* 一致）
L400_SAMPLE = ("L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 "
               "g=+512/-256/+0 c=-128/+64/-0")
L400_STAT_SAMPLE = ("L400S bad=0 n=12345 f=0 gap=0 eul=12000 "
                    "miss=0 skip=0 txd=0 cyc=2490/3100")
L400_INIT_OK = "L400 init ok lpf_rb=3"
L400_INIT_FAIL = "L400 init fail st=1 lpf_rb=255"


class FakeLineSource:
    """按给定字节块返回数据；块可以在一行中间切断，验证粘包处理"""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.index = 0

    def read(self):
        if self.index < len(self.chunks):
            data = self.chunks[self.index]
            self.index += 1
            return data
        time.sleep(0.01)
        return b""

    def close(self):
        pass


class L400ParserTest(unittest.TestCase):
    def test_q10_to_dps(self):
        # 1024 Q10 = 1 rad/s = 57.2958 °/s
        self.assertAlmostEqual(mon.q10_rad_s_to_dps(1024), 57.2957795, places=5)
        self.assertAlmostEqual(mon.q10_rad_s_to_dps(512), 28.6478898, places=5)

    def test_parse_observation(self):
        kind, data = mon.parse_l400_line(L400_SAMPLE)
        self.assertEqual(kind, "l400")
        self.assertEqual(data["rdy"], 1)
        self.assertEqual(data["lk"], 1)
        self.assertEqual(data["vld"], 1)
        self.assertEqual((data["alk_ms"], data["avld_ms"], data["gpk_ms"]),
                         (35, 18, 31))
        self.assertEqual(data["g_q10"], [512, -256, 0])
        self.assertEqual(data["c_q10"], [-128, 64, 0])
        self.assertAlmostEqual(data["g_dps"][0], 28.6478898, places=5)
        self.assertAlmostEqual(data["c_dps"][0], -7.1619724, places=5)

    def test_parse_statistics(self):
        kind, data = mon.parse_l400_line(L400_STAT_SAMPLE)
        self.assertEqual(kind, "l400s")
        self.assertEqual(data["bad_pkt"], 0)
        self.assertEqual(data["n"], 12345)
        self.assertEqual(data["f"], 0)
        self.assertEqual(data["gap"], 0)
        self.assertEqual(data["eul"], 12000)
        self.assertEqual(data["miss"], 0)
        self.assertEqual(data["skip"], 0)
        self.assertEqual(data["txd"], 0)
        self.assertEqual((data["cyc_max_us"], data["cyc_peak_us"]), (2490, 3100))

    def test_parse_status_and_unknown(self):
        kind, text = mon.parse_l400_line(L400_INIT_OK)
        self.assertEqual((kind, text), ("status", L400_INIT_OK))
        self.assertIn("低通回读=3", mon.l400_detail_of(L400_INIT_OK))
        kind, text = mon.parse_l400_line(L400_INIT_FAIL)
        self.assertEqual((kind, text), ("status", L400_INIT_FAIL))
        self.assertIn("st=1", mon.l400_detail_of(L400_INIT_FAIL))
        self.assertIsNone(mon.parse_l400_line("STA n=0012C0 f=0000"))
        self.assertIsNone(mon.parse_l400_line("乱码 / 上电启动信息"))

    def test_negative_sticks_accepted(self):
        kind, data = mon.parse_l400_line(
            "L400 rdy=0 lk=0 vld=0 alk=9999 avld=9999 gpk=0 "
            "g=-1/-2/-3 c=+1/+2/+3")
        self.assertEqual(kind, "l400")
        self.assertEqual(data["g_q10"], [-1, -2, -3])
        self.assertEqual(data["c_q10"], [1, 2, 3])
        self.assertEqual(data["lk"], 0)


class L400WorkerTest(unittest.TestCase):
    def test_worker_splits_lines_and_writes_two_csvs(self):
        text = "\n".join([L400_INIT_OK, L400_SAMPLE, L400_STAT_SAMPLE]) + "\n"
        raw = text.encode("ascii")
        # 故意切成多段，有的段落在行中间
        chunks = [raw[:9], raw[9:23], raw[23:71], raw[71:130], raw[130:]]
        with tempfile.TemporaryDirectory() as tmp:
            obs_path = os.path.join(tmp, "obs.csv")
            stat_path = os.path.join(tmp, "stats.csv")
            src = FakeLineSource(chunks)
            worker = mon.L400Worker(src, obs_path, stat_path, queue.Queue(maxsize=100))
            worker.start()

            deadline = time.time() + 2.0
            while time.time() < deadline and (worker.frames < 1 or worker.stats < 1):
                time.sleep(0.01)
            worker.stop()
            worker.join(timeout=2.0)

            self.assertEqual(worker.frames, 1)
            self.assertEqual(worker.stats, 1)
            self.assertEqual(worker.bad, 0)
            self.assertIn("初始化完成", worker.state)

            with open(obs_path, newline="", encoding="utf-8-sig") as fp:
                rows = list(csv.DictReader(fp))
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row["rdy"], "1")
            self.assertEqual(row["avld_ms"], "18")
            self.assertEqual(row["gx_q10"], "512")
            self.assertEqual(row["cx_q10"], "-128")
            self.assertAlmostEqual(float(row["gx_dps"]), 28.6478898, places=3)

            with open(stat_path, newline="", encoding="utf-8-sig") as fp:
                stats = list(csv.DictReader(fp))
            self.assertEqual(len(stats), 1)
            self.assertEqual(stats[0]["n"], "12345")
            self.assertEqual(stats[0]["eul"], "12000")
            self.assertEqual(stats[0]["cyc_peak_us"], "3100")


class L400SimulationTest(unittest.TestCase):
    def test_simulated_source_is_parseable(self):
        src = mon.SimL400Source()
        buf = b""
        lines = []
        t0 = time.time()
        while time.time() - t0 < 1.3:
            buf += src.read()
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    lines.append(line)
            time.sleep(0.02)

        parsed = [mon.parse_l400_line(line) for line in lines]
        obs = [p for p in parsed if p and p[0] == "l400"]
        stats = [p for p in parsed if p and p[0] == "l400s"]
        status = [p for p in parsed if p and p[0] == "status"]
        self.assertGreaterEqual(len(obs), 5)
        self.assertGreaterEqual(len(stats), 1)
        self.assertEqual(len(obs) + len(stats) + len(status),
                         len([p for p in parsed if p]))

    def test_no_plot_cli_writes_csvs(self):
        with tempfile.TemporaryDirectory() as tmp:
            obs_path = os.path.join(tmp, "cli_obs.csv")
            stat_path = os.path.join(tmp, "cli_stats.csv")
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "uart_sensor_monitor.py")
            env = dict(os.environ, PYTHONIOENCODING="utf-8")
            result = subprocess.run(
                [sys.executable, script, "--l400", "--simulate", "--no-plot",
                 "--duration", "1.5", "--csv", obs_path, "--stats-csv", stat_path],
                cwd=os.path.dirname(script), timeout=30,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                encoding="utf-8", errors="replace", env=env)
            self.assertEqual(result.returncode, 0, result.stdout)
            with open(obs_path, newline="", encoding="utf-8-sig") as fp:
                obs_rows = list(csv.DictReader(fp))
            with open(stat_path, newline="", encoding="utf-8-sig") as fp:
                stat_rows = list(csv.DictReader(fp))
            self.assertGreaterEqual(len(obs_rows), 5)
            self.assertGreaterEqual(len(stat_rows), 1)
            self.assertIn("gx_dps", obs_rows[0])
            self.assertIn("cyc_peak_us", stat_rows[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
