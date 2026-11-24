# 鍥炲綊鐩綍

鍘熸潵鐨?`tools/test_*.c`銆乣test_*.py` 鍜?`flight_log_stub.c` 宸查泦涓埌杩欓噷锛涚敤鎴峰叆鍙?`uart_sensor_monitor.py`銆乣flight_log_export.py` 鍜屾牎鍑嗚剼鏈粛鐣欏湪 `tools/` 鏍圭洰褰曘€?
杩愯鍏抽敭鍥炲綊锛?
```powershell
Set-Location "E:\娉℃搏绾搁鏈篭code"
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

瑕嗙洊锛?
- Flash CRC / boot 鏍￠獙锛?- 鏃ュ織瀹屾暣銆佹仮澶嶃€並3 鍚綍銆佹敹灏撅紱
- 鐪熷疄 FlightLoop 鐘舵€侊紱
- 瑙ｉ攣 GUI銆丩400 绐楀彛鍜屾墦鍖呭昂瀵革紱
- Python 宸ュ叿鍥炲綊銆?
鍘嗗彶 v1 瀵煎嚭澶瑰叿锛?
```text
fixtures/flight_log_v1_history.bin
```

鐢?`data/logs/flight_log_k3_01.csv` 閲嶅缓锛屼粎鐢ㄤ簬楠岃瘉鏃х増瀵煎嚭璺緞锛屼笉鏄疄椋炲師濮?dump銆?