#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_sensor_monitor.py 的 L400 模式自测。

覆盖：
  1. 7d01167 旧十进制 L400S，以及带控制超期/控制完成时延的新十六进制 L400S；
  2. 控制器联调观察行 ta/tw/on/q，Q15 四元数转欧拉角，Q10 rad/s → °/s；
  3. 串口分块到达时 L400Worker 的拆行、两份 CSV 与统计；
  4. 模拟源在 10Hz / 1Hz 下产生可解析的文本；
  5. --l400 --simulate --no-plot 端到端写 CSV；
  6. L400D A/B 汇总解析与 .diag.txt 落盘。

用法（在 tools 目录下）：
  python test_l400_monitor.py
"""

import csv
import math
import os
import queue
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace

import uart_sensor_monitor as mon

# 7d01167 联调固件的实际输出样例（字段顺序与固件 l400_put_* 一致）
L400_SAMPLE = ("L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 "
               "g=+512/-256/+0 c=-128/+64/-0")
# 控制器联调新增：目标姿态 ta（0.01°）、目标角速度 tw（Q10）、控制使能 on
L400_SAMPLE_CTRL = (L400_SAMPLE +
                    " ta=+3000/-1500 tw=+512/-256/+0 on=1 "
                    "q=7FFF/0000/0000/0000")
# 新短报文：状态位 + 包龄 + Q15 q + Q10 角速度/纠偏（定宽十六进制）
L400_SHORT_SAMPLE = ("L400Q f=F a=0023 v=0012 k=001F "
                     "q=7FFF/0000/0000/0000 "
                     "r=00064/00000/FFFFF c=FFC0/0000/0040 "
                     "m=1 x=0 pwm=064/05A/03C/032")
L400_STAT_SAMPLE = ("L400S bad=0 n=12345 f=0 gap=0 eul=12000 "
                    "miss=0 skip=0 txd=0 cyc=2490/3100")
# 新固件格式：定宽十六进制；miss=整拍超期，cmiss=控制超期，ctl=控制完成时延
L400_STAT_HEX_SAMPLE = ("L400S bad=0000 n=0012C0 f=0000 eul=0012C0 "
                        "miss=000003 cmiss=000001 skip=000000 txd=000002 "
                        "ctl=09BA/0C1C late=04B0/09C4 exec=0384/0514 "
                        "rx=0028/005A tx=0032/0096")
L400_DIAG_A = ("L400D A late=900/1500 exec=1000/1600 cmiss=0 skip=0 "
               "rfirq=40/80 rf=30/60 app=20/50")
L400_DIAG_ABORT = "L400D ABORT seg=B reason=cmd haveA=1"
L400_TXDIAG = "L400T budget=12 busy=3"
L400_PREP = "L400P prep=120/180 eul=45/70 n=200 e=200"
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
        # 旧固件没有控制目标/四元数字段
        self.assertIsNone(data["ctrl_ok"])
        self.assertIsNone(data["tgt_att_deg100"])
        self.assertIsNone(data["q15"])

    def test_parse_short_observation(self):
        kind, data = mon.parse_l400_line(L400_SHORT_SAMPLE)
        self.assertEqual(kind, "l400")
        self.assertEqual((data["rdy"], data["lk"], data["vld"], data["ctrl_ok"]),
                         (1, 1, 1, 1))
        self.assertEqual((data["alk_ms"], data["avld_ms"], data["gpk_ms"]),
                         (35, 18, 31))
        self.assertEqual(data["q15"], [32767, 0, 0, 0])
        self.assertEqual(data["g_q10"], [100, 0, -1])
        self.assertEqual(data["c_q10"], [-64, 0, 64])
        self.assertAlmostEqual(data["g_dps"][0], 100 * mon.Q10_RAD_S_TO_DPS,
                               places=5)
        self.assertIsNone(data["tgt_att_deg100"])
        self.assertEqual(data["motor_armed"], 1)
        self.assertEqual(data["motor_lock"], 0)
        self.assertEqual(
            (data["pwm_front"], data["pwm_rear"],
             data["pwm_left"], data["pwm_right"]),
            (100, 90, 60, 50))

    def test_parse_observation_control_fields(self):
        kind, data = mon.parse_l400_line(L400_SAMPLE_CTRL)
        self.assertEqual(kind, "l400")
        self.assertEqual(data["tgt_att_deg100"], [3000, -1500])
        self.assertEqual(data["tgt_rate_q10"], [512, -256, 0])
        self.assertEqual(data["ctrl_ok"], 1)
        self.assertEqual(data["q15"], [32767, 0, 0, 0])

    def test_q15_to_euler(self):
        # 单位四元数 → 零姿态
        roll, pitch, yaw = mon.q15_to_euler_deg([32767, 0, 0, 0])
        self.assertAlmostEqual(roll, 0.0, places=3)
        self.assertAlmostEqual(pitch, 0.0, places=3)
        self.assertAlmostEqual(yaw, 0.0, places=3)
        # 横滚 +30°：q = (cos15°, sin15°, 0, 0)
        qw = int(round(math.cos(math.radians(15.0)) * 32768))
        qx = int(round(math.sin(math.radians(15.0)) * 32768))
        roll, pitch, yaw = mon.q15_to_euler_deg([qw, qx, 0, 0])
        self.assertAlmostEqual(roll, 30.0, places=2)
        self.assertAlmostEqual(pitch, 0.0, places=2)
        self.assertAlmostEqual(yaw, 0.0, places=2)

    def test_parse_statistics_old_decimal(self):
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
        # 旧 7d01167 没有分项耗时/控制超期：保持 None，CSV 留空
        self.assertIsNone(data["late_avg_us"])
        self.assertIsNone(data["tx_max_us"])
        self.assertIsNone(data["cmiss"])
        self.assertIsNone(data["ctrl_miss"])

    def test_parse_statistics_hex_timing(self):
        kind, data = mon.parse_l400_line(L400_STAT_HEX_SAMPLE)
        self.assertEqual(kind, "l400s")
        self.assertEqual(data["bad_pkt"], 0)
        self.assertEqual(data["n"], 0x12C0)
        self.assertEqual(data["eul"], 0x12C0)
        self.assertEqual(data["miss"], 3)
        self.assertEqual(data["cmiss"], 1)
        self.assertEqual(data["ctrl_miss"], 1)
        self.assertEqual((data["ctrl_max_us"], data["ctrl_peak_us"]),
                         (0x09BA, 0x0C1C))
        self.assertIsNone(data["gap"])
        self.assertIsNone(data["cyc_max_us"])
        self.assertEqual((data["late_avg_us"], data["late_max_us"]), (0x04B0, 0x09C4))
        self.assertEqual((data["exec_avg_us"], data["exec_max_us"]), (0x0384, 0x0514))
        self.assertEqual((data["rx_avg_us"], data["rx_max_us"]), (0x0028, 0x005A))
        self.assertEqual((data["tx_avg_us"], data["tx_max_us"]), (0x0032, 0x0096))

    def test_parse_ab_diagnostic(self):
        kind, data = mon.parse_l400_line(L400_DIAG_A)
        self.assertEqual(kind, "l400d")
        self.assertEqual(data["seg"], "A")
        self.assertEqual((data["late_avg_us"], data["late_max_us"]), (900, 1500))
        self.assertEqual((data["exec_avg_us"], data["exec_max_us"]), (1000, 1600))
        self.assertEqual((data["cmiss"], data["skip"]), (0, 0))
        self.assertEqual((data["rfirq_avg_us"], data["rfirq_max_us"]), (40, 80))
        self.assertEqual((data["rf_avg_us"], data["rf_max_us"]), (30, 60))
        self.assertEqual((data["app_avg_us"], data["app_max_us"]), (20, 50))

    def test_parse_ab_abort(self):
        kind, data = mon.parse_l400_line(L400_DIAG_ABORT)
        self.assertEqual(kind, "l400d_abort")
        self.assertEqual(data["seg"], "B")
        self.assertEqual(data["reason"], "cmd")
        self.assertEqual(data["have_a"], 1)

    def test_parse_tx_defer_reasons(self):
        kind, data = mon.parse_l400_line(L400_TXDIAG)
        self.assertEqual(kind, "l400t")
        self.assertEqual(data["budget"], 12)
        self.assertEqual(data["busy"], 3)

    def test_parse_prep_euler_timing(self):
        kind, data = mon.parse_l400_line(L400_PREP)
        self.assertEqual(kind, "l400p")
        self.assertEqual((data["prep_avg_us"], data["prep_max_us"]), (120, 180))
        self.assertEqual((data["eul_avg_us"], data["eul_max_us"]), (45, 70))
        self.assertEqual((data["prep_n"], data["eul_n"]), (200, 200))

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
            # 有效观察帧要记录主机单调时间，供界面判断状态是否过期
            self.assertGreater(worker.last_obs_mono, 0.0)
            self.assertLess(abs(time.monotonic() - worker.last_obs_mono), 2.0)

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

    def test_worker_writes_diag_sidecar(self):
        text = (L400_DIAG_A + "\n").encode("ascii")
        with tempfile.TemporaryDirectory() as tmp:
            obs_path = os.path.join(tmp, "obs.csv")
            stat_path = os.path.join(tmp, "stats.csv")
            worker = mon.L400Worker(FakeLineSource([text]), obs_path, stat_path,
                                    queue.Queue(maxsize=100))
            worker.start()
            deadline = time.time() + 2.0
            while time.time() < deadline and worker.diag < 1:
                time.sleep(0.01)
            worker.stop()
            worker.join(timeout=2.0)
            self.assertEqual(worker.diag, 1)
            self.assertEqual(worker.bad, 0)
            with open(stat_path + ".diag.txt", encoding="utf-8") as fp:
                saved = fp.read().strip()
            self.assertEqual(saved, L400_DIAG_A)

    def test_worker_writes_timing_columns(self):
        text = "\n".join([L400_INIT_OK, L400_SAMPLE, L400_STAT_HEX_SAMPLE]) + "\n"
        with tempfile.TemporaryDirectory() as tmp:
            obs_path = os.path.join(tmp, "obs.csv")
            stat_path = os.path.join(tmp, "stats.csv")
            worker = mon.L400Worker(FakeLineSource([text.encode("ascii")]),
                                    obs_path, stat_path, queue.Queue(maxsize=100))
            worker.start()
            deadline = time.time() + 2.0
            while time.time() < deadline and (worker.frames < 1 or worker.stats < 1):
                time.sleep(0.01)
            worker.stop()
            worker.join(timeout=2.0)

            with open(stat_path, newline="", encoding="utf-8-sig") as fp:
                stats = list(csv.DictReader(fp))
            self.assertEqual(len(stats), 1)
            row = stats[0]
            self.assertEqual(row["n"], str(0x12C0))
            self.assertEqual(row["cmiss"], "1")
            self.assertEqual(row["ctrl_miss"], "1")
            self.assertEqual(row["ctrl_max_us"], str(0x09BA))
            self.assertEqual(row["ctrl_peak_us"], str(0x0C1C))
            self.assertEqual(row["late_avg_us"], str(0x04B0))
            self.assertEqual(row["late_max_us"], str(0x09C4))
            self.assertEqual(row["exec_avg_us"], str(0x0384))
            self.assertEqual(row["exec_max_us"], str(0x0514))
            self.assertEqual(row["rx_avg_us"], str(0x0028))
            self.assertEqual(row["rx_max_us"], str(0x005A))
            self.assertEqual(row["tx_avg_us"], str(0x0032))
            self.assertEqual(row["tx_max_us"], str(0x0096))


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
            self.assertIn("tgt_roll_deg100", obs_rows[0])
            self.assertIn("qw_q15", obs_rows[0])
            self.assertIn("ctrl_peak_us", stat_rows[0])


class L400StatusStaleTest(unittest.TestCase):
    """串口停发 1s 后状态栏必须标过期，恢复新帧后回到正常显示"""

    def _worker(self, age_s):
        obs = {
            "rdy": 1, "lk": 1, "vld": 1,
            "alk_ms": 35, "avld_ms": 18, "gpk_ms": 31,
            "g_q10": [0, 0, 0], "c_q10": [0, 0, 0],
            "g_dps": [0.0, 0.0, 0.0], "c_dps": [0.0, 0.0, 0.0],
        }
        stat = {"cyc_peak_us": 3100}
        return SimpleNamespace(
            last_obs=obs, last_obs_mono=time.monotonic() - age_s,
            last_stat=stat, frames=10, stats=1,
            state="初始化完成", detail="", error="")

    def test_stale_then_recover(self):
        monitor = mon.SensorMonitor(self._worker(1.3), "", mode="l400")
        stale = monitor._status_l400()
        self.assertIn("串口数据中断", stale)
        self.assertIn("已过期", stale)
        # 过期时不能再显示“遥控 连接 / 指令 有效”
        self.assertNotIn("遥控 连接", stale)
        self.assertNotIn("指令 有效", stale)

        fresh = mon.SensorMonitor(self._worker(0.4), "", mode="l400")
        text = fresh._status_l400()
        self.assertNotIn("串口数据中断", text)
        self.assertIn("遥控 连接", text)
        self.assertIn("指令 有效", text)

    def test_threshold_is_about_one_second(self):
        # 刚过 1s 就标过期，0.5s 内仍按新帧显示
        self.assertEqual(mon.L400_STALE_S, 1.0)
        self.assertIn("串口数据中断",
                      mon.SensorMonitor(self._worker(1.05), "", mode="l400")._status_l400())
        self.assertNotIn("串口数据中断",
                         mon.SensorMonitor(self._worker(0.5), "", mode="l400")._status_l400())


class CsvSameFileTest(unittest.TestCase):
    """--csv 与 --stats-csv 指向同一文件时必须先报错，不能截断已有内容"""

    def setUp(self):
        self.script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "uart_sensor_monitor.py")
        self.env = dict(os.environ, PYTHONIOENCODING="utf-8")

    def _run(self, args):
        return subprocess.run(
            [sys.executable, self.script] + args,
            cwd=os.path.dirname(self.script), timeout=20,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding="utf-8", errors="replace", env=self.env)

    def _write_keep(self, path):
        with open(path, "w", encoding="utf-8") as fp:
            fp.write("KEEP")

    def _read_keep(self, path):
        with open(path, encoding="utf-8") as fp:
            return fp.read()

    def test_same_path_rejected_in_all_modes(self):
        # L400、姿态、默认六轴三种模式都要在打开文件前拦住
        for mode_args in (["--l400", "--simulate"], ["--attitude", "--simulate"], ["--simulate"]):
            with self.subTest(mode=mode_args), tempfile.TemporaryDirectory() as tmp:
                path = os.path.join(tmp, "same.csv")
                self._write_keep(path)
                result = self._run(mode_args + ["--no-plot", "--duration", "0.2",
                                                "--csv", path, "--stats-csv", path])
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn("同一文件", result.stdout)
                self.assertEqual(self._read_keep(path), "KEEP")

    def test_relative_alias_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            sub = os.path.join(tmp, "sub")
            os.makedirs(sub)
            path = os.path.join(tmp, "same.csv")
            alias = os.path.join(sub, "..", "same.csv")
            self._write_keep(path)
            result = self._run(["--l400", "--simulate", "--no-plot", "--duration", "0.2",
                                "--csv", path, "--stats-csv", alias])
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("同一文件", result.stdout)
            self.assertEqual(self._read_keep(path), "KEEP")

    def test_check_runs_before_opening_serial(self):
        # 用不存在的 COM99 证明：先报路径重复，而不是先报打开串口失败
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "same.csv")
            self._write_keep(path)
            result = self._run(["--l400", "--port", "COM99", "--no-plot", "--duration", "0.2",
                                "--csv", path, "--stats-csv", path])
            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("同一文件", result.stdout)
            self.assertNotIn("打开串口", result.stdout)
            self.assertEqual(self._read_keep(path), "KEEP")


if __name__ == "__main__":
    unittest.main(verbosity=2)
