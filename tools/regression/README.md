# 回归目录

原来的 `tools/test_*.c`、`test_*.py` 和 `flight_log_stub.c` 已集中到这里；用户入口
`uart_sensor_monitor.py`、`flight_log_export.py` 和校准脚本仍留在 `tools/` 根目录。

运行关键回归：

```powershell
Set-Location "E:\泡沫纸飞机\code"
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

覆盖：

- Flash CRC / boot 校验；
- 日志完整、恢复、K3 启录、收尾；
- 真实 FlightLoop 状态；
- 解锁 GUI、L400 窗口和打包尺寸；
- Python 工具回归。

历史 v1 导出夹具：

```text
fixtures/flight_log_v1_history.bin
```

由 `data/logs/flight_log_k3_01.csv` 重建，仅用于验证旧版导出路径，不是实飞原始 dump。
