# data 目录迁移清单（2026-11-22）

旧路径 → 新路径，逐项 SHA256 与字节数已核对。

| 旧路径 | 新路径 | 字节 | SHA256 |
|---|---|---:|---|
| `data/flight_log_k3_01.csv` | `data/logs/flight_log_k3_01.csv` | 52855 | `D09FC1DB7D21A66617A6E036E4320B7D09DAF652FDCC033C75557DC2223AEA9A` |
| `data/flight_log_k3_uart.csv` | `data/logs/flight_log_k3_uart.csv` | 27090 | `6CC769F41A0ABA16688FB88AB726928FB282DEE9B24D35336C053E81A21989EF` |
| `data/flight_log_k3_uart.txt` | `data/logs/flight_log_k3_uart.txt` | 28424 | `785C7C9BD59A696D78EA39CB6C912D3CC5365BC640172D84AFD1CD87CD8CB563` |
| `data/logkey_rf_stats02.csv.diag.txt` | `data/logs/logkey_rf_stats02.csv.diag.txt` | 699 | `9ED5317C71748D327A261DFF1836222B98690C83DFD61691358045AACB9A6F4F` |

校准 JSON 仍在 `data/` 根目录（`accel_cal.json`、`gyro_bias_frd.json`、`imu_axis_map_frd.json`），以保持现有校准命令默认路径不变，不纳入本次迁移。

## 唯一 BIN 归档

| 旧路径 | 新路径 | 字节 | SHA256 |
|---|---|---:|---|
| `data/archive/bench_before_flight_20261122/firmware_builds/Robin_24G_Car_rc_loop400_keycap.bin` | `firmware_delivery/archive/Robin_24G_Car_rc_loop400_keycap.bin` | 27076 | `4571D27456AAD9A86D231243484EB3043B75C58CD882613B46E5FFD1E7D13DDF` |
| `data/archive/bench_before_flight_20261122/firmware_builds/Robin_24G_Car_rc_loop400_sched.bin` | `firmware_delivery/archive/Robin_24G_Car_rc_loop400_sched.bin` | 26644 | `CAFEAC6C8551889CD7F81225B204408841BA331D878BD4E6D9D681368166BF16` |

## 根目录临时日志 dump 归档

| 旧路径 | 新路径 | 字节 | SHA256 |
|---|---|---:|---|
| `flight_log_clean.bin` | `data/archive/flight_log_dumps_20261122/flight_log_clean.bin` | 53248 | `DF03276D39786A1C113634751DD938A36317ECA341573E73658F17F0DA87A0D8` |
| `flight_log_dump.bin` | `data/archive/flight_log_dumps_20261122/flight_log_dump.bin` | 53248 | `946BB2D6F0E7695CCA6D35FE263B0E985D91F5FA8CCA068D3F86C21E6DABF2F9` |
| `flight_log_integration.bin` | `data/archive/flight_log_dumps_20261122/flight_log_integration.bin` | 53248 | `2B65BE2CC5D717301097624AEB0C0A602A832E406EF4FB2F14A99633F3A00F77` |
| `flight_log_locked.bin` | `data/archive/flight_log_dumps_20261122/flight_log_locked.bin` | 53248 | `7A69A61C414E2995A60662A87253D5BF790776F0E1580543F1015F4A305D185B` |
| `flight_log_unlock.bin` | `data/archive/flight_log_dumps_20261122/flight_log_unlock.bin` | 53248 | `D7397B87D466F3F01E5724EAA6325C700B35C7EE6FC127446D9E39AB615294BC` |

迁移后 `data/` 根目录只保留 3 个校准 JSON 和本清单；`data/logs` 4 个文本/CSV；原始 dump 5 个已归档。
