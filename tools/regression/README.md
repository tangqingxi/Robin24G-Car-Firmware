# 闁搞儳鍋涚紞濠囨儎椤旇偐绉?
闁告鍠愬鐢告儍?`tools/test_*.c`闁靛棔姊梩est_*.py` 闁?`flight_log_stub.c` 鐎瑰憡鐓″▔锔界▔椤撶偛鐓傞弶鈺傜懇閸ｇ兘鏁嶅☉姘殢闁规潙鍢查崣鍡涘矗?`uart_sensor_monitor.py`闁靛棔姊梖light_log_export.py` 闁告粌鏈悧搴ㄥ礄閸℃艾澹栭柡鍫厸缁盯鎮惧▎蹇旇含 `tools/` 闁哄秴婀卞ú鎷屻亹閺囨ǚ鍋?
閺夆晜鍔橀、鎴﹀礂閹惰姤鏆涢柛銉у仜缂嶅﹪鏁?
```powershell
Set-Location "E:\婵炲鍓濋幖蹇曠棯閹间緡妫ч柡鍫㈩嚔code"
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

閻熸洖妫涘ú濠囨晬?
- Flash CRC / boot 闁哄稄绻濋悰娆撴晬?- 闁哄啨鍎辩换鏃傗偓鐟版湰閺嗭綁濡存担閫涘垝濠㈣泛绉查埀顑跨敨3 闁告凹鍨扮紞宥夊Υ娴ｈ鏆悘蹇旀嫕缁?- 闁活亞鍠庨悿?FlightLoop 闁绘鍩栭埀顑跨筏缁?- 閻熸瑱缍侀弨?GUI闁靛棔淇?00 缂佹劖顨呰ぐ娑㈠椽鐏炴儳鈪甸柛鏍ф噹閺勫倻鈧敻娼荤槐?- Python 鐎规悶鍎遍崣鍧楀炊閻愯尙绉洪柕?
闁告ê妫楄ぐ?v1 閻庣數鍘ч崵顓熷緞閻熸澘寰旈柨?
```text
fixtures/flight_log_v1_history.bin
```

闁?`data/logs/flight_log_k3_01.csv` 闂佹彃绉寸紓鎾绘晬鐏炶偐鐭岄柣顫妺缁剚顨ュ畝鍐闁哄唲鍛暭閻庣數鍘ч崵顓犳崉椤栨氨绐為柨娑樺缁楀寮伴姘辨澖濡炲鍋涚敮顐ｆ叏?dump闁?
## 鏁撮摼 IMU 鍥炴斁

鍥炴斁婧愶細`replay_imu_chain.c`銆傚悓涓€杈撳叆鍒嗗埆瀵?`a76d5b6` 涓?`b914f7c` 缂栬瘧锛?
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

杈撳嚭閫愭媿 CSV锛屾瘮杈冧袱浠借緭鍑哄簲涓?`diff=0`锛涜剼鏈唴鏂█鍖呮嫭闈欐灏辩华銆佽В閿侀潪闆?PWM銆佹€ュ仠銆佸け鑱斻€佽澶辫触鎭㈠鍜?100/105 ms銆?
### 一条 PowerShell 对比命令

```powershell
powershell -ExecutionPolicy Bypass -File tools\regression\run_imu_chain_compare.ps1
```

脚本用同一份 `replay_imu_chain.c` 编译 `a76d5b6` 对照副本和当前源码，逐拍输出 CSV 并比较；`-Werror=format` 已加入编译参数。基线对照副本只额外包含 `FLIGHT_STARTUP_TEST` 下的观测接口。
