# 工具入口

> 本页只列当前常用命令。详细历史说明见 [`README_DETAIL.md`](README_DETAIL.md)。
> 回归测试已移到 [`regression/`](regression/)，数据已按用途整理。

## 当前命令

从仓库根目录核验交付镜像：

```powershell
python firmware_delivery\verify_hashes.py
```

地面 UART 采集需刷地面诊断固件。归档里的两份 UART 固件用于复现偏航 P=0.25 的旧实验；当前飞行基线的偏航 P 为 2.0。
烧录后断开 BDT，再接 UART，在仓库根目录运行：

```powershell
python tools\uart_sensor_monitor.py --port COM12 --ground-diag --csv "data\logs\ground_yaw_$(Get-Date -Format yyyyMMdd_HHmmss).csv"
```

历史 BDT 数据的导出方法见 [历史说明](README_DETAIL.md)。

## 校准命令

校准 JSON 仍保留在 `data/` 根目录，现有默认命令不变：

```powershell
python tools\imu_axis_map.py --port COM12 --out data\imu_axis_map_frd.json
python tools\accel_six_face_cal.py --port COM12 --axis-map data\imu_axis_map_frd.json --out data\accel_cal.json
python tools\gyro_bias_cal.py --port COM12 --axis-map data\imu_axis_map_frd.json --out data\gyro_bias_frd.json
```

维护固件：`firmware_delivery/daily/Robin_24G_Car_maint_269a.bin`，烧 `0x4000`。

## 回归

```powershell
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

覆盖 CRC、真实 FlightLoop 启动/解锁/急停/失联、地面 UART 诊断、偏航对称、boot 校验和打包尺寸。

历史 v1 导出夹具：

```powershell
python tools\flight_log_export.py --bin tools\regression\fixtures\flight_log_v1_history.bin --out .\history_v1.csv
```

## 数据

```text
data\            当前校准 JSON
data\logs\       历史日志/导出文本
data\archive\    归档原始数据（含 flight_log_dumps_20261122）
```

`tools/regression/DATA_MIGRATION.md` 记录迁移路径、字节数和 SHA256。

## 旧件

- 交付镜像：`firmware_delivery/INVENTORY.md`
- 历史镜像：`firmware_delivery/archive/`
- 原始基线：`firmware_baseline/`
- 旧工具说明：`tools/README_DETAIL.md`
- 旧交付说明：`firmware_delivery/README_DETAIL.md`
