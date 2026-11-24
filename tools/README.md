# 工具入口

本页只列当前常用工具；历史实验和旧命令见
[`README_DETAIL.md`](README_DETAIL.md)。

## 常用入口

| 类别 | 工具 | 用途 | 运行位置 |
|---|---|---|---|
| 构建与交付 | `check_bin_size.py` | 检查最终 BIN 是否超出应用区上限 | 仓库根目录 |
| 构建与交付 | `package_flight_control.py` | 检查后原子复制最终 BIN 到交付路径 | 仓库根目录 |
| 构建与交付 | `patch_flight_build.py` | 向 Telink `subdir.mk` 补齐飞控源文件和编译项目 | 仓库根目录 |
| 构建与交付 | `patch_bootloader_no_ota.py` | 从 bootloader 工程移除 OTA 源文件 | 仓库根目录 |
| 六轴与串口 | `imu_axis_map.py` | 采集并生成 IMU 轴向映射 | 仓库根目录 |
| 六轴与串口 | `accel_six_face_cal.py` | 六面加速度校准 | 仓库根目录 |
| 六轴与串口 | `gyro_bias_cal.py` | 陀螺零偏校准 | 仓库根目录 |
| 六轴与串口 | `uart_sensor_monitor.py` | 串口观察传感器/地面诊断并输出 CSV | 仓库根目录 |
| 气压诊断 | `baro_bdt_export.py` | 解析 BDT 回读的 `Read.bin`，导出气压/温度/补偿结果 | 仓库根目录 |
| 回归 | `regression/run_key_regression.ps1` | 运行关键主机回归 | 仓库根目录 |

## 构建与交付

```powershell
python tools\check_bin_size.py path\to\Robin_24G_Car_Flash.bin
python tools\package_flight_control.py path\to\Robin_24G_Car_Flash.bin firmware_delivery\candidates\Robin_24G_Car_flight_control_baseline.bin
```

正常飞行构建开关：

```text
-DC_FLIGHT_CONTROL_EN=1
```

气压测量默认关闭；显式开启才使用：

```text
-DC_FLIGHT_BARO_EN=1
```

## 六轴校准与串口

```powershell
python tools\imu_axis_map.py --port COM12 --out data\imu_axis_map_frd.json
python tools\accel_six_face_cal.py --port COM12 --axis-map data\imu_axis_map_frd.json --out data\accel_cal.json
python tools\gyro_bias_cal.py --port COM12 --axis-map data\imu_axis_map_frd.json --out data\gyro_bias_frd.json
```

六轴维护固件：

```text
firmware_delivery\daily\Robin_24G_Car_maint_269a.bin
烧录地址 0x4000
```

地面 UART：

```powershell
python tools\uart_sensor_monitor.py --port COM12 --ground-diag --csv "data\logs\ground_diag.csv"
```

## 气压 BDT 导出

BDT 回读：

```text
TC122X rf 10000 -s 4k
```

导出：

```powershell
python tools\baro_bdt_export.py --bin "E:\Tools\TelinkBDT\config\user\Read.bin" --out "data\logs\baro_bdt.csv"
```

支持的记录版本和 CSV 列见脚本顶部说明；v4 输出 Pa/°C，v5 输出板端 `0.1 Pa` 补偿值。

## 回归

```powershell
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

回归覆盖构建尺寸、Flash/boot 校验、FlightLoop 启动与保护、地面诊断、偏航对称、LPC2623 就绪/定点温补、气压测量分步与导出。

## 数据与历史

```text
data\             当前校准 JSON
data\logs\        导出 CSV/文本
data\archive\     归档原始数据
```

历史工具说明：

```text
tools\README_DETAIL.md
tools\regression\DATA_MIGRATION.md
firmware_delivery\README_DETAIL.md
```
