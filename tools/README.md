# Robin_24G_Car 传感器校准与监视工具（操作手册）

四个命令行工具 + 两个共用模块 + 一个自测脚本：

| 文件 | 用途 |
|---|---|
| `imu_axis_map.py` | 传感器轴 → 机体 FRD 映射（六面摆法） |
| `accel_six_face_cal.py` | 加速度计六面校准 / 单面补测 |
| `gyro_bias_cal.py` | 陀螺仪静止零偏 |
| `uart_sensor_monitor.py` | 六轴/台架/历史联调解析；`--arm-debug` 进入专用解锁诊断 GUI |
| `arm_debug_monitor.py` | 解锁诊断 GUI/CSV 状态机：启动、收包、解锁判定、PWM、锁回 |
| `uart_proto.py` | 0x269A / 0x269B 帧解析（纯标准库，被上面几个工具共用） |
| `calib_common.py` | 三个校准工具共用的串口枚举/只收数据源、输出防覆盖、FRD 映射校验、均值/标准差 |
| `test_l400_monitor.py` | L400 解析、分块收包、两份 CSV 与模拟采集自测 |
| `test_euler_normalize.c` | UserEuler 正常 2.5ms、90ms 长间隔、大单步和损坏四元数的主机侧回归 |
| `test_euler_div.c` | 积分 `%10000` 改为商求余后，正负输入/q/dq_rem 逐点一致和异常步长保护 |
| `test_l400_motor.c` | 混控俯仰/偏航符号；200ms 0002 脉冲/重复包/无效包；0001 急停优先和新事件解锁 |
| `test_flight_modules.c` | RcInput 映射/心跳/200ms/500ms 超时、外环目标/偏航直通、内环 P=64/256 |
| `test_flight_order.c` | 真实 FlightLoop 包路径：0001 先清 PWM；采样前失联/指令超时清 PWM |
| `test_flight_startup.c` | 真实启动路径：−10/0/−1010mg 静置 500ms 后就绪，再发合法 0002 解锁 |
| `test_flight_arm_startup.c` | 诊断启动路径：ADB/ADI 真实发送，静置统计非零 |
| `test_flight_compat.c` | b3cb56f 参考公式 vs 当前：四路 PWM、正负差动、限幅、故障重解锁 |
| `test_flight_log.c` | FlightLog 空盘/写满/半页断电/CRC/地址/Q15+32768 回归 |
| `test_flight_log_integration.c` | 真实 FlightLoop 启动/采样/急停 + BDT 命令 + Python 导出 |
| `test_flight_log_states.c` | 持续锁定采集、解锁→失联/超时、PWM 先归零、Dropped |
| `test_flash_fw_check.c` | 真实 CRC 函数 + Flash 桩：0xB000/256 边界/坏 CRC/越界 |
| `flight_log_export.py` | BDT 原始日志区转 CSV，校验页/记录 CRC 并计算欧拉角 |
| `test_flight_arm_debug.c` | 诊断协议/队列：ADB/ADS/ADI/ADU/ADG/ADO/ADL/ADE、长度 ≤160、UART 忙重试 |
| `test_arm_debug_monitor.py` | 五种 GUI/CSV 场景：IMU 失败、无 0002、油门拒绝、锁回、正常 PWM |
| `test_package_bin.py` | 0x4000+bin ≤0xF000 打包门：超限非零且不覆盖交付文件 |
| `check_bin_size.py` | 最终 .bin 应用区上限检查；超限非零退出 |
| `package_flight_control.py` | 检查尺寸后原子发布交付镜像 |
| `patch_flight_build.py` | 把拆分后的 7 个飞控 .c/.o/规则写入 TC32 subdir.mk（可重复运行） |
| `test_l400_diag.c` | A/B 状态机 2s/10s、失效重计时、掉链中止、UART 忙待发回归 |

三个校准工具的命令、参数、采样判定、JSON 格式和单面补测能力保持不变；
重复的串口/输出/FRD 校验代码已收进 `calib_common.py`，陀螺标定开始时
清空串口积压的行为也保留。

---

## 0. 准备

**依赖**（Python 3.9+）：

```powershell
python -m pip install pyserial
```

（只有 `uart_sensor_monitor.py` 画曲线时还需要 `matplotlib`、`numpy`；`--no-plot` 和三个校准工具不需要。）

**接线**：飞机 UART **TX → USB 串口 RX**，两板**共地（GND）**，115200-8-N1。
工具**只接收**，不向飞机发送任何指令。

**串口独占**：同一时间只能有一个程序占用 COM 口。开校准工具前先关掉监视器、串口助手、
IoT Studio 的串口窗口等；否则会提示"打开串口失败"。

**找 COM 口**：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python imu_axis_map.py --list
```

下面所有命令都在 `E:\泡沫纸飞机\code\tools` 目录下执行，把 `COM12` 换成实际端口。
结果统一写到 `..\data\`（该目录不入 git）。所有工具都**不覆盖**同名结果文件：
已存在时会自动加 `_1`、`_2` 后缀（单面补测加 `--replace` 时才直接覆盖）。

---

## 1. FRD 轴映射（先做这一步）

确定传感器的 X/Y/Z 与机体坐标（**+X 前、+Y 右、+Z 下**）的对应关系，后面两个校准都用到它。

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python imu_axis_map.py --port COM12 --out "..\data\imu_axis_map_frd.json"
```

**摆放**：工具按顺序提示 6 个姿态，每个摆好后在电脑上按 Enter，采 1.2 s 自动记录。
顺序固定为 +Z、+Y、+X、-Z、-Y、-X，全部是"机体某轴朝下"：

| 顺序 | 姿态 | 物理摆法 |
|---|---|---|
| 1 | 机体 +Z 朝下 | 正放：机腹朝下、机头朝前 |
| 2 | 机体 +Y 朝下 | 右侧朝下：右机翼在下、机身侧立 |
| 3 | 机体 +X 朝下 | 机头朝下：机头垂直向下、机尾朝上 |
| 4 | 机体 -Z 朝下 | 倒放：机腹朝上、机背朝下 |
| 5 | 机体 -Y 朝下 | 左侧朝下：左机翼在下、机身侧立 |
| 6 | 机体 -X 朝下 | 机尾朝下：机头垂直向上、机尾朝下 |

允许轻微倾斜（主轴 ≥0.5g），但晃动、主轴不明会要求重采。工具会检查正反对称性、
三轴一一对应、行列式 +1，不通过就提示复核对应摆法且不写文件。

**输出**：`..\data\imu_axis_map_frd.json`（矩阵方向 `v_frd = R × v_sensor`）。

---

## 2. 加速度计六面校准

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python accel_six_face_cal.py --port COM12 --axis-map "..\data\imu_axis_map_frd.json" --out "..\data\accel_cal.json"
```

**摆放**：提示同时显示机体摆法和传感器读数；随便什么顺序，每个面停稳约 1.5 s 后自动采 5 s：

- 机体 **+Z 朝下** = 正放；**-Z 朝下** = 倒放
- 机体 **+X 朝下** = 机头朝下；**-X 朝下** = 机尾朝下
- 机体 **+Y 朝下** = 右侧朝下；**-Y 朝下** = 左侧朝下

要求：电机断开、飞控保持上锁；画面会显示"横向 ax/ay 偏大多少"，按提示摆正即可。

**输出**：`..\data\accel_cal.json`。文件里同时有：

- `calibration`：**传感器坐标**的零偏/比例（原始读数所在坐标系）；
- `calibration_frd`：**机体 FRD 坐标**的参数（零偏按映射重排并变号，比例只重排）；
- `axis_map` / `body_faces`：本次用到的映射和"机体摆法 → 传感器面"对照。

不加 `--axis-map` 时，面名与 `--face` 按**传感器面**解释（旧用法），文件里不会有 FRD 参数。

### FRD 单面补测

某一面没采好时不用六面重来：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python accel_six_face_cal.py --port COM12 --axis-map "..\data\imu_axis_map_frd.json" --resume "..\data\accel_cal.json" --face=-Z --replace
```

- **`--face=-Z` 的物理摆法**：机体 **-Z 轴朝下**，也就是**倒放（机腹朝上）**。
  静止时加速度计测的是比力，方向指向上方，所以倒放对应 FRD 加速度 **+Z ≈ +1g**；
  对应到传感器读数则取决于映射（本机是 az ≈ -1g）。
- 负号面必须写成 `--face=-Z`（等号形式），写成 `--face -Z` 会被当成选项。
- 只重采这一面，其余五面保留原值，合并后重新求解；`--replace` 表示校验通过后直接覆盖原文件，
  不加则另存一个新文件（原文件不动）。
- 打开串口前会校验输入 JSON 的格式、型号、量程、LSB/g 和六面记录，不兼容直接报错退出。

---

## 3. 陀螺仪静止零偏

### 3.1 先刷地面维护镜像（校准专用）

校准脚本吃的是 **0x269A 原始六轴帧**，当前飞控正常运行不发实时文本，两者不兼容，
所以标定前必须换成维护镜像：

| 用途 | 交付镜像 | 串口输出 |
|---|---|---|
| 六轴校准（本节目录下的三个脚本） | `firmware_delivery/daily/Robin_24G_Car_maint_269a.bin` | 只有 0x269A 二进制帧，约 100 Hz |
| 200 Hz 飞控（正常无实时串口） | `firmware_delivery/daily/Robin_24G_Car_flight_control.bin` | 无；历史报文可用 `--l400` 解析 |
| 历史 400 Hz 联调（仅回退） | `firmware_delivery/archive/Robin_24G_Car_rc_loop400.bin` | 旧 L400 长行 |

两份都烧到应用区 **0x4000**（bootloader 0x0–0x3FFF 与 0x10000 以上的 Flash 区域都不动）。
完整的路径、字节数、SHA256、归档镜像和构建模式说明见
[`firmware_delivery/README.md`](../firmware_delivery/README.md)。

**注意**：构建目录里的通用 `.bin` 与按模式命名的构建副本已清理；重新编译才会再生成，
**不能按文件名判断用途**，烧录请以交付目录里的文件名和 SHA256 为准。

维护镜像的特性：沿用台架固件的 T-3488 初始化（±16g、±2000°/s、800 Hz ODR、片内低通 SEL=3），
电机保持停转；帧里的六轴是**传感器原坐标系、轴映射/扣偏/软件滤波之前**的原始计数，
`bx/by/bz` 填 0、`flags` 也是 0（未解锁、未扣偏、未软件滤波）。

**上电顺序**：烧录完成 → **断电重新上电**（不要只按复位，避免残留状态）→ 等 1 秒让固件完成传感器初始化 → 再运行脚本。

### 3.2 跑标定（只维护这一份 JSON）

零偏结果固定写 `..\data\gyro_bias_frd.json`，**不另存复测版本**。
脚本本身不覆盖同名文件（存在时会自动加 `_1`、`_2` 后缀），所以重测时先删掉旧文件再跑，
结果仍然落回同一个名字，`data\` 下始终只有一份零偏 JSON：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
Remove-Item "..\data\gyro_bias_frd.json" -Force -ErrorAction SilentlyContinue
python gyro_bias_cal.py --port COM12 --axis-map "..\data\imu_axis_map_frd.json" --out "..\data\gyro_bias_frd.json"
```

判稳 3 s → 拟合 15 s → 复核 10 s；**复核不通过时脚本不写文件**（此时旧文件已删），
排除晃动后重跑一次即可。

**摆放**：**断开电机、关闭遥控器**，飞机放桌上松手，可以稍微倾斜，只要不动。
按 Enter 后：判稳 3 s → 拟合 15 s → 复核 10 s，全程约半分钟；晃动会被提示重采该段。

**输出**：`..\data\gyro_bias_frd.json`，含 `bias_frd_lsb` / `bias_frd_dps`、
映射矩阵、量程与灵敏度、两段的帧数与统计、判定阈值。

复核不通过（残余均值 >0.30 °/s 或标准差 >0.20 °/s）时不写文件，只打印每轴残差。

这是**本次上电、本次温度**下的静止零偏；换电池或温度变化大时重测。

---

## 4. 串口监视器（只看数据）

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12
```

加 `--no-plot --duration 60 --csv "..\data\monitor.csv"` 可以不画图、定时记录。

### 200 Hz 飞控（无实时串口遥测）

本版主飞控只保留 5ms 控制、姿态估计、遥控和安全状态机，**正常运行时不再发送
FCQ/FCS、A/B 诊断、分项计时或串口延期报告**。`FlightTelemetry` 模块及其发送/
诊断实现已从主镜像链接中移除；`UserUart` 底层接口仍保留，供后续停机导出日志。

- 无实时串口图；`--flight-control` 和 `--l400` 仅保留给历史镜像和旧报文解析。
- 历史 `L400Q/L400S/FCQ/FCS` 行仍可由 `uart_sensor_monitor.py` 解析，
  CSV 列和旧样本兼容性不变；不要用当前主镜像期待这些行。
- A/B 台架诊断不再随主镜像提供；维护校准镜像、0x269A 原始六轴协议保留。
- 主循环仍是 200 Hz：`FlightLoop_Process` 到点后先做采样前安全门，再读 IMU、
  更新四元数、双环和混控；失联/指令超时在采样前先清四路。
- 收包路径先处理 `0001/0002`，再做同包目标换算；`0001` 当包立即清 PWM，
  不等待下一控制拍。急停/失联后必须新的 `0002` 才解锁。

主镜像验收不依赖串口图，按无桨电机行为检查：约 200 Hz、正负偏航差动、
左侧下键急停四路归零、解锁后关遥控四路归零、重连后仍锁定并需重新长按上键。

### TC32 结构对齐修复（候选，待无桨）

`-fpack-struct` 下把跨模块指针接口逐成员对齐，并在编译期锁住偏移：

```text
s_fl 基址（正常镜像） 0x8416E8，大小 0x108
rc       +0x00
attc     +0x20
ratec    +0x54
safety   +0x60
att      +0x68
lpf      +0xA8
gyro_rem +0xB4
rate     +0xC0
prev_acc +0xCC
	tgt_rate +0x38（attc 内）
```

TC32 反汇编核对：`Euler_IsStill` 收到 `s_fl+0xA8`/`s_fl+0xCC`，
`Euler_GyroToQ10` 的三轴余数为 `+0xB4/+0xB8/+0xBC`，
`RateControl_Update` 收到 `+0x38`/`+0xC0`，`Euler_Update` 收到
`att=+0x68, lpf=+0xA8`。`test_flight_arm_startup.c` 同时验证
`OnStillSample` 已接回真实判稳路径，`ADI sm=500`，不再是全零计数。

候选镜像在 `firmware_delivery/candidates/`，日常交付件暂不替换。

### 无 OTA + FlightLog（任务二候选）

- 日志区：`0x10000..0x1CFFF`，13 个 4KiB 扇区；应用仍限制到 `0xF000`。
- 记录：56B/条，25Hz（每 8 个 5ms 拍），最多 750 条，约 30s。
- 每条含原始 `q[w,x,y,z]`（s32 Q15，保留 +32768）、陀螺 Q10、估计器加速度 mg、
  偏航杆/目标/纠偏、推力、四路 PWM、标志；急停/失联另写事件。
- 页头 32B，记录 56B，页头/记录 CRC16；掉电时未完成页不写入页头，导出跳过。
- 外部启动/擦除命令：BDT 在 `0x1D000` 写 4 字节 `4C 4F 47 31`（"LOG1" 小端），
  四路锁定、PWM 全零时应用会擦除日志区并开始一趟记录。
- BDT 读回 `0x10000` 起 `0xD000` 字节，导出：

```powershell
python flight_log_export.py --bin .\flight_log_dump.bin --out .\flight_log.csv
```

> 控制拍只把 56B 快照入 2 条队列；主循环 Service 每次最多写一条。Flash 驱动仍会关中断等待，写页阻塞时延尚未上板实测；控制/急停优先，
> 不做飞行中 UART 导出。

### 日志时序测量（调试候选）

`candidates/Robin_24G_Car_flight_log_debug.bin`（32420 字节，
SHA256 `C1C61909C56DE1415EB82B90D258F7193B235B1A0AC9938BD178728C13C291E2`）
同时启用 FlightLog 和 `C_FLIGHT_ARM_DEBUG`。GUI 每秒额外收：

```text
ADT t=... dm=<控制拍间隔峰值ms> sk=<跳拍数> ld=<日志丢样数>
```

CSV 对应 `ctrl_dt_max_ms / ctrl_skips / log_drops`。无桨时用现有
`--arm-debug` GUI 跑 30 秒以上，记录 `dm` 最大值、`sk` 总计和 `ld` 总计。
该镜像有 UART 诊断，不能用于正式飞行；正式静默版仍是 `..._log.bin`。

### 专用解锁诊断（--arm-debug）

诊断镜像单独构建，正式镜像仍静默：

```powershell
# 在 subdir.mk 的编译行加入 -DC_FLIGHT_ARM_DEBUG=1 后重新 make
# 产物：Robin_24G_Car_flight_arm_debug.bin，烧 0x4000
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --arm-debug --duration 60 --csv "..\data\arm_debug.csv"
```

默认打开 GUI；`--no-plot` 是控制台备用。窗口四块固定顺序：

1. IMU 与姿态：初始化返回码、静置毫秒、姿态就绪、PWM 初始化；
2. 遥控与按键：收包/有效摇杆/心跳/是否收到 0002/尝试次数；
3. 解锁结果：每次 0002 的原始油门、映射四轴、各门限、接受/拒绝原因；
4. 四路 PWM：标为“PWM 指令”，实际转动由台架观察。

板端协议（每行 ≤160 字节）：

```text
ADB t=... st=.. rb=.. still=... att=... pwm=...
ADS t=... st=.. still=.. att=.. pwm=.. rx=.. vld=.. hb=.. e2=.. n=.. arm=.. x=.. drop=..
ADI t=... ok=.. f=.. sm=.. b=.. bn=.. bg=.. bj=.. bp=.. bi=..
ADU t=... n=.. rt=.... cmd=t/y/p/r pkt=.. low=.. ctr=.. att=.. lk=.. ok=.. why=...
ADG t=... n=.. a=xxxx/xxxx/xxxx g=xxxx/xxxx/xxxx
ADO t=... n=.. up=.. ctrl=.. arm=.. x=.. pwm=fff/fff/fff/fff
ADL t=... n=.. x=.. pwm=fff/fff/fff/fff
ADE t=... n=.. x=5 pwm=fff/fff/fff/fff
```

- `ADB st` 是 `t3488_bench_init` 的真实返回码，`rb` 是 lpf 回读；
- `ADI` 每秒统计：`ok/f` 六轴读取成功/失败，`sm` 最长连续静置毫秒，
  `b` 最近一次打断原因；`bn/bg/bj/bp/bi` 分别累计模长越界、陀螺超限、
  相邻跳变、采样长间隔、500ms 后姿态初始化失败；
- 拒绝尝试额外发 `ADG`，`a/g` 是当时三轴加速度（mg）和角速度（Q10）的有符号 16 位十六进制；
- 除新增诊断外，校准值、`Euler_IsStill` 判稳阈值和所有解锁条件未改。

`why` 具体为 `pkt/estop/thr/roll/pitch/yaw/att/link`；GUI 会翻译成
“油门过高（-180，要求 ≤ -250）”等中文。CSV 同时写板端时间、主机时间、
尝试编号、诊断字段和原始行。UART 忙时板端队列保留，下一主循环重试。

### 左侧上/下键原始收包抓取

1. 电机断开，烧 `Robin_24G_Car_rc_loop400_keycap.bin`；该镜像不初始化电机 PWM。
2. 打开串口监视器（`--flight-control --no-plot`），先长按左侧上键、再短按左侧下键，
   两次间隔 1s 以上。
3. 控制台出现 `[按键] L400K ...`，同时写入 `<统计 CSV>.diag.txt`。
4. 把左侧上键的按下、保持、松开三行（包括 `key=0000` 的释放行）留档；
   左侧下键同样留档。

已实测确认：`0002` 是解锁短事件，`0001` 是急停短事件，脉冲约 200ms。
正常镜像已按事件沿处理；抓包镜像保留，用于以后复核原始字节。

### 无桨台架电机验收

1. **上电锁定**：无串口可看，直接用无桨电机确认四路不转；未解锁前推油门无输出。
2. **解锁**：油门低位、三轴回中、姿态就绪、链路有效时，长按一次左侧上键；
   四路应进入低速运转（45..400 范围）。脉冲内重复包不能造成二次动作；
   条件不满足时上键应被丢弃，松开后重新长按新的 `0002` 才可能解锁。
3. **急停**：短按一次左侧下键，四路必须立即停转；下键按住期间长按上键不得解锁，
   松开下键后也必须重新长按上键产生新的 `0002`。
4. **失联**：解锁后关遥控器，指令/链路超时后四路必须停转；重开遥控器后仍锁定，
   必须重新长按左侧上键产生新的 `0002` 才能解锁。
5. **姿态异常**：上电晃动或断开 IMU，四路应保持/回到停转；恢复姿态后仍需新的
   `0002`。
6. **三轴反向**：无桨固定机架，油门保持低位/略高，分别打横滚、俯仰、偏航，
   观察四路转速差动方向；反向拨杆应完全相反。
7. **电机编号/转向**：按前=LF/PWM1、后=LB/PWM3、左=RB/PWM0、右=RF/PWM2
   对照 V12；确认转向与桨的 CW/CCW 要求一致。任何一项不符先不要装桨。

- 当前主联调镜像：
  `telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\Robin_24G_Car_flight_control.bin`，
  28500 字节，SHA256 `AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60`，
  烧 0x4000。`0x4000 + 28500 = 0xAF54 <= 0xF000`，应用区剩余 16556 字节。
- 专用解锁诊断镜像（C_FLIGHT_ARM_DEBUG=1，不改正式件）：
  `firmware_delivery/arm_debug/Robin_24G_Car_flight_arm_debug.bin`，
  33876 字节，SHA256 `DFDBE4E4C6888388EAC037DF88333A55E796A7C054A80C2C045A2705458A3955`，
  烧 0x4000，应用区剩余 11180 字节。
- 对齐修复候选（待无桨）：
  - `candidates/Robin_24G_Car_flight_control_alignfix.bin`：24900 字节，
    SHA256 `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087`；
  - `candidates/Robin_24G_Car_flight_arm_debug_alignfix.bin`：30452 字节，
    SHA256 `1FA9323EF35333E201C87A32E4AE6B00F87508C0CC725FFE93BDA10817A1E189`。
  - 两者均烧 0x4000；日常 `AA3D...` 件未替换。
- 任务二无 OTA + 日志候选：
  - `candidates/Robin_24G_Bootloader_no_ota.bin`：6844 字节，
    SHA256 `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096`，烧 0x0；
  - `candidates/Robin_24G_Car_flight_control_log.bin`：26484 字节，
    SHA256 `BB3984569C9AC007EE83879CD99AEBE4EC4388D7C9593FAA4F94F4297D5FBAF8`，烧 0x4000。
- 同选项移除前后对比：

  ```text
  项          移除前      移除后      省下
  .bin        42612       28500       14112
  .text       42088       27976       14112
  .data       440         440         0
  .bss        2070        1246        824
  ```

  未来日志候选为原 OTA 暂存区 `0x10000–0x1BFFF`（48KB，12 扇区）和
  `0x1C000–0x1CFFF`（4KB 标记扇区），本轮不改分区/不写日志。
- 拆分前回退件 `firmware_delivery/archive/Robin_24G_Car_flight_control_b3cb56f.bin`
  （35108 / `F8607D15…`）仅作回退，不再作为当前联调镜像。
- 上一版调度测试镜像 `_sched.bin` 继续保留（26644 / `CAFEAC6C…`）。
- 左侧上/下键原始抓包镜像（无 PWM，仅台架）：
  `Robin_24G_Car_rc_loop400_keycap.bin`（27076 字节，
  SHA256 `4571D27456AAD9A86D231243484EB3043B75C58CD882613B46E5FFD1E7D13DDF`）。

- 构建目录被 `.gitignore` 忽略，新增的 7 个飞控 .c 需要用脚本写入生成清单：

  ```powershell
  Set-Location "E:\泡沫纸飞机\code"
  python tools\patch_flight_build.py `
    "telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\vendor\Robin_24G_Car\subdir.mk"
  # 在 subdir.mk 的 UserRxdFunc.c 编译行加入 -DC_FLIGHT_CONTROL_EN=1 后：
  mingw32-make clean
  mingw32-make all
  ```

  抓包构建再加 `-DC_L400_KEY_CAPTURE=1`；新入口只用 `C_FLIGHT_CONTROL_EN`，
  旧 `C_RC_LOOP_400_EN` 仅作兼容别名。
- 最终 `.bin` 打包门：`make all` 的 post-build 先跑
  `tools/check_bin_size.py ... --limit 45056`，超限非零退出、停止后续发布；
  发布交付副本用：

  ```powershell
  python tools\package_flight_control.py `
    "telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\Robin_24G_Car_flight_control.bin" `
    "firmware_delivery\daily\Robin_24G_Car_flight_control.bin"
  ```

  超限或输入缺失时返回非零，且不创建/覆盖交付文件。

**已跑的主机回归**

- 偏航/俯仰符号、200 ms 0002 重复包/无效包、0001 优先与重新解锁
  （`test_l400_motor.c`，链接 `MotorMixer.c` + `FlightSafety.c`）；
- RcInput 映射/心跳/200 ms/500 ms 超时、外环偏航直通、内环 P
  （`test_flight_modules.c`）；
- 真实 `FlightLoop_OnRcPacket`/`FlightLoop_PreSampleSafety`：0001 先清 PWM，
  目标姿态换算不在急停包执行；失联/指令超时在采样前清 PWM（`test_flight_order.c`）；
- 真实启动路径：静置 −10/0/−1010mg 满 500ms 后 `att.ready`，再送合法低油门
  `0002` 能解锁（`test_flight_startup.c`，链接真实 UserEuler）；
- 诊断协议：ADB/ADS/ADI/ADU/ADG/ADO/ADL/ADE、行长 ≤159、UART 忙重试
  （`test_flight_arm_debug.c`，链接真实 FlightArmDebug.c）；
- GUI/CSV 五种状态：IMU 失败、无 0002、油门拒绝、解锁后锁回、正常 PWM
  （`python test_arm_debug_monitor.py`，8 项）；
- b3cb56f 参考公式对照：240 个检查点，油门/四路混控/正负差动/限幅/
  故障后重新解锁完全一致（`test_flight_compat.c`）；
- 诊断启动：`ADB/ADI` 真实发送，`ADI sm=500`，证明 `OnStillSample` 已接回
  （`test_flight_arm_startup.c`）；
- `0x4000 + bin <= 0xF000` 打包门：超限返回非零且不覆盖旧交付件
  （`test_package_bin.py`）；
- 25 项监视器解析/模拟/旧报文兼容；四元数、积分除余回归。

**待做的上板无桨验收**

1. 5ms 节拍由调试器/构建调度确认，不依赖串口；无实时串口图属预期；
2. 正/负偏航时 `左+右` 与 `前+后` 的大小关系正确；
3. 左侧下键短按后四路马上为 0；
4. 解锁后关遥控器，四路回 0；
5. 重新连接后仍锁定，必须重新长按左侧上键产生新的 `0002` 才解锁。

- 历史报文解析自测：`python test_l400_monitor.py`（25 项，覆盖旧
  `L400Q/L400S` 和曾用过的 `FCQ/FCS`）；当前主飞控不再输出这些行。
- 四元数异常更新回归（主机侧，不烧板）：

  ```powershell
  gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car `
      test_euler_normalize.c `
      ..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\UserEuler.c `
      -o ..\data\test_euler_normalize.exe
  ..\data\test_euler_normalize.exe
  ```

  覆盖正常 2.5ms 连续更新、90ms 长间隔拒绝、异常后恢复、50ms 大单步拒绝、
  损坏四元数拒绝；异常时必须返回失败且四元数保持不变。

- 积分除余优化验证（主机侧，旧/新各编译一次并逐字节比较输出）：

  ```powershell
  $gcc = "D:\app\Visual Studio Code2025\c\MinGW\bin\gcc.exe"
  $inc = "-I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car"
  $src = "..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\UserEuler.c"
  & $gcc -std=c99 -O2 -fno-builtin -DC_EULER_DIV_OPT=0 $inc test_euler_div.c $src -o ..\data\euler_div_base.exe
  & $gcc -std=c99 -O2 -fno-builtin -DC_EULER_DIV_OPT=1 $inc test_euler_div.c $src -o ..\data\euler_div_opt.exe
  ..\data\euler_div_base.exe > ..\data\euler_div_base.out
  ..\data\euler_div_opt.exe > ..\data\euler_div_opt.out
  fc.exe /b ..\data\euler_div_base.out ..\data\euler_div_opt.out
  ```

  输出逐字节一致；正负整数除余公式同时扫 `INT_MIN/INT_MAX` 附近和
  `-35000..35000` 的点。`C_EULER_DIV_OPT=1` 的 TC32 目标文件不再引用
  `__modsi3`，只保留 `__divsi3`，text 少 28 字节。

- 电机混控/急停回归（主机侧）：

  ```powershell
  gcc -std=c99 -O2 -fno-builtin -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car `
      test_l400_motor.c -o ..\data\test_l400_motor.exe
  ..\data\test_l400_motor.exe
  ```

  覆盖机头抬高时 `后 > 前`（小信号和限幅后都成立）、正偏航时
  `左+右 > 前+后` / 负偏航反过来（同样覆盖限幅），200ms 的 0002
  脉冲只处理一次、重复包不重复解锁、无效摇杆包的 0002 被丢弃，
  以及 0001 急停优先、急停/失联后必须新的 0002 才重新解锁。

- A/B 状态机回归（主机侧）：

  ```powershell
  gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car `
      test_l400_diag.c -o ..\data\test_l400_diag.exe
  ..\data\test_l400_diag.exe
  ```

  覆盖连续有效满 2s 才进 A、A/B 各 10s、失效后重新计时、A/B 掉链中止
  保留已完成阶段/原因，以及 UART 忙后 A/B 两行仍能按序发出。

- 旧 7d01167 十进制格式已在 COM12 实机抓取验证；新十六进制/控制器字段
  已由模拟源、解析和 CSV 测试覆盖，实机控制器验收需烧录 `Robin_24G_Car_flight_control.bin` 后再采集。

### 台架姿态固件（文本协议，--attitude）

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --attitude --duration 120 --csv "..\data\att.csv"
```

- `--attitude` 按固件的 `ATT ...` / `STA ...` 文本行解析（r/p/y 已经是度），
  画横滚、俯仰、相对偏航三条曲线，状态栏同时显示最新三个角度与固件状态
  （ATT init / wait still / ready / 初始化失败）；
- 状态栏还显示最新一条 STA 的有效帧数、读失败、长间断与 dt/rd/up 耗时；
- 姿态样本存 `att.csv`，统计存 `att_stats.csv`，分项计时（四类拍的拍数/最大耗时/超期
  次数，来自固件的 `STA2` 行）存 `att_cat.csv`，两分钟静置验收直接看这三个文件；
- 不加文本模式参数时，仍是原来的 0x269A / 0x269B 二进制帧模式，行为和参数不变。

### 三种模式对照

| 烧录镜像 | 监视器参数 | 图形内容 |
|---|---|---|
| 200 Hz 飞控联调版（当前） | `--flight-control` | g / c 三轴角速度，单位 °/s |
| 历史姿态版（归档） | `--attitude` | 横滚 / 俯仰 / 相对偏航，单位 ° |
| 六轴校准维护版 | 不加文本参数（默认二进制帧） | 加速度 g / 陀螺 dps |

镜像路径、SHA256 与烧录地址见 [`firmware_delivery/README.md`](../firmware_delivery/README.md)；
**当前交付镜像为 200 Hz 飞控联调版；`--l400` 仍作为兼容别名可用。**

---

## 5. 气压计（后续）

本轮不提供气压计脚本。后续集成到高度估计阶段时再做两件事：
**起飞前置零**（地面静置取当前气压作基准）和**已知高度差验证**（在楼梯/升降台等
已知高差处对比气压高度与真实高度）。

---

## 6. 单位与坐标约定

| 项 | 约定 |
|---|---|
| 机体系 | FRD：+X 前、+Y 右、+Z 下（右手系） |
| 加速度 | ±16g，**2048 LSB/g**；1g 按本地重力约 9.788 m/s²（与标准值差 0.19%，可忽略） |
| 陀螺 | ±2000 dps，**16.4 LSB/dps**；1 LSB ≈ 0.0010641 rad/s |
| 联调 g/c | 固件里是 Q10 rad/s；上位机按 `°/s = Q10 × 180/π ÷ 1024` 换算，只在界面/CSV 显示，**不是欧拉角** |
| 固件接入 | 先映射到 FRD，再扣 FRD 零偏，最后换算：`omega = (R·g - bias_frd) * 0.0010641` |
