# 閸ョ偛缍婇惄顔肩秿

閸樼喐娼甸惃?`tools/test_*.c`閵嗕梗test_*.py` 閸?`flight_log_stub.c` 瀹告煡娉︽稉顓炲煂鏉╂瑩鍣烽敍娑氭暏閹村嘲鍙嗛崣?`uart_sensor_monitor.py`閵嗕梗flight_log_export.py` 閸滃本鐗庨崙鍡氬壖閺堫兛绮涢悾娆忔躬 `tools/` 閺嶅湱娲拌ぐ鏇樷偓?
鏉╂劘顢戦崗鎶芥暛閸ョ偛缍婇敍?
```powershell
Set-Location "E:\濞夆剝鎼忕痪鎼侇棧閺堢code"
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

鐟曞棛娲婇敍?
- Flash CRC / boot 閺嶏繝鐛欓敍?- 閺冦儱绻旂€瑰本鏆ｉ妴浣逛划婢跺秲鈧甫3 閸氼垰缍嶉妴浣规暪鐏忔拝绱?- 閻喎鐤?FlightLoop 閻樿埖鈧緤绱?- 鐟欙綁鏀?GUI閵嗕俯400 缁愭褰涢崪灞惧ⅵ閸栧懎鏄傜€甸潻绱?- Python 瀹搞儱鍙块崶鐐茬秺閵?
閸樺棗褰?v1 鐎电厧鍤径鐟板徔閿?
```text
fixtures/flight_log_v1_history.bin
```

閻?`data/logs/flight_log_k3_01.csv` 闁插秴缂撻敍灞肩矌閻劋绨宀冪槈閺冄呭鐎电厧鍤捄顖氱窞閿涘奔绗夐弰顖氱杽妞嬬偛甯慨?dump閵?
## 整链 IMU 回放

回放源：`replay_imu_chain.c`。同一输入分别对 `a76d5b6` 与 `b914f7c` 编译：

```powershell
gcc -std=c99 -O2 -fno-builtin -DFLIGHT_HOST_TEST=1 -DFLIGHT_STARTUP_TEST=1 ^
  -DCLOCK_SYS_CLOCK_HZ=24000000 -DC_GROUND_UART_DIAG=1 -fpack-struct ^
  -I<root>\telink_b85m_platform_src1015\common ^
  -I<root>\telink_b85m_platform_src1015\demo\vendor\common\common ^
  -I<root>\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
  -I<root>\telink_b85m_platform_src1015\chip\tc122x\drivers ^
  replay_imu_chain.c FlightLoop.c ImuMeasure.c RcInput.c FlightSafety.c ^
  MotorMixer.c AttitudeControl.c RateControl.c UserEuler.c GroundDiag.c ^
  -o replay.exe
```

输出逐拍 CSV，比较两份输出应为 `diff=0`；脚本内断言包括静止就绪、解锁非零 PWM、急停、失联、读失败恢复和 100/105 ms。
