# Robin_24G_Car 传感器校准与监视工具（操作手册）

四个命令行工具 + 两个共用模块 + 一个自测脚本：

| 文件 | 用途 |
|---|---|
| `imu_axis_map.py` | 传感器轴 → 机体 FRD 映射（六面摆法） |
| `accel_six_face_cal.py` | 加速度计六面校准 / 单面补测 |
| `gyro_bias_cal.py` | 陀螺仪静止零偏 |
| `uart_sensor_monitor.py` | 六轴波形、台架姿态（`--attitude`）与 200Hz 联调（`--l400`）监视与 CSV 记录（只查看，不校准） |
| `uart_proto.py` | 0x269A / 0x269B 帧解析（纯标准库，被上面几个工具共用） |
| `calib_common.py` | 三个校准工具共用的串口枚举/只收数据源、输出防覆盖、FRD 映射校验、均值/标准差 |
| `test_l400_monitor.py` | L400 解析、分块收包、两份 CSV 与模拟采集自测 |
| `test_euler_normalize.c` | UserEuler 正常 2.5ms、90ms 长间隔、大单步和损坏四元数的主机侧回归 |
| `test_euler_div.c` | 积分 `%10000` 改为商求余后，正负输入/q/dq_rem 逐点一致和异常步长保护 |
| `test_l400_motor.c` | 俯仰限幅前后符号、急停按住禁止解锁/重新长按回归 |
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

校准脚本吃的是 **0x269A 原始六轴帧**，当前联调固件发的是 L400Q 短文本，两者不兼容，
所以标定前必须换成维护镜像：

| 用途 | 交付镜像 | 串口输出 |
|---|---|---|
| 六轴校准（本节目录下的三个脚本） | `firmware_delivery/daily/Robin_24G_Car_maint_269a.bin` | 只有 0x269A 二进制帧，约 100 Hz |
| 400Hz 历史联调 / 200Hz 测试镜像（`--l400`） | 日常 `Robin_24G_Car_rc_loop400.bin` / 测试 `_ctrl.bin` | L400 长行 / L400Q 短行 + L400S |

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

### 200Hz 联调固件（文本协议，--l400）

本测试镜像把 IMU 读取、四元数估计和双环统一到 **200Hz（5ms）**；姿态积分
继续使用实际采样间隔，遥控指令/链路超时仍是原来的 200ms/500ms，超期门限
随周期改为 5ms。A/B 仍是 2s 连续有效后各跑 10s，默认关闭、按特技键开启。


7d01167 固件输出 10Hz 的 `L400 ...` 观察行和 1Hz 的 `L400S ...` 统计行；
带分项耗时的新 L400 固件把 `L400S` 改成定宽十六进制，并追加
`late/exec/rx/tx` 四个耗时字段，监视器同时兼容新旧两种格式。
图形监视命令：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --l400 --duration 120 --csv "..\data\l400.csv"
```

- 常规路径默认开启，A/B 默认关闭（按需按键才启动）。验收时遥控关闭，
  先启动监视器再给板子上电，应能从 `rdy=0` 到 `rdy=1` 持续收到 `L400Q`；
  监视器关闭后重新打开，应继续接收，不需要复位板子。
- 三张图：第一张 **实测角速度 g (°/s)**，第二张 **内环 PID 纠偏 c (°/s)**，
  第三张 **实测姿态 (°)**。g/c 是机体 FRD 角速度量与纠偏量，**不是欧拉角**；
  固件里的 Q10 rad/s 已在接收端换算成 °/s。第三张由低频观测行里的 Q15 四元数
  在上位机换算成横滚/俯仰/相对偏航，固件控制路径不参与欧拉角计算。
  旧镜像没有 `q=` 字段时第三张图留空，不影响 g/c。
- 状态栏：姿态就绪、遥控连接、指令有效、有效包龄、峰值（新固件为控制完成峰值）。
  控制使能/禁止和目标姿态显示在下方统计区。包龄最大显示 9999ms，
  从未收到过对应包时也显示 9999。
- 状态新鲜度：用最近一条 L400 观察帧的**主机单调时间**判断；超过约 1s 没有新观察帧时，
  状态栏显示“串口数据中断”，并把遥控/指令状态标为“已过期”；恢复接收后按新帧更新。
- CSV 保护：打开串口和文件之前检查 `--csv` 与 `--stats-csv` 是否指向同一文件；
  相同会明确报错退出，已有文件内容不会被截断。默认六轴和 `--attitude` 模式也做同样检查。
- 调度（新固件）：200Hz 到点先执行 L400 采样/控制，再做 `d24_rx_main_loop`
  的配对/ACK 和应用层收包；射频中断仍保留，控制优先。
- 报告（新固件）：`UartTrySend` 非阻塞提交。观测行和统计行分别用**上次实测的
  组帧+提交耗时**（首次用保守默认值）加 100us 余量判断本拍够不够；不够就延期。
  统计行放不下时，如果短观测到点且放得下，允许先发短观测，统计留到下一拍。
  `txd` 表示**报告延期次数**（预算不足或串口忙），不再表示“跨过下一拍截止”；
  `tx` 只统计真正拼过文本的拍。
- 控制（新固件）：外环 P 用四元数推算的向上比力方向与目标方向做叉积，直接得到
  横滚/俯仰误差；摇杆目标在**有效收包时**换算，200Hz 控制拍不再算欧拉角或三角函数。
  目标横滚/俯仰满杆 ±30°，偏航目标角速度满杆 ±3rad/s；内环 PID 的 P=1/4，
  I/D 为 0 时对应的每拍状态更新不编译，只观察不写电机。欧拉角显示留给上位机。
  失联、指令超时、姿态未就绪或本拍没有新姿态时，目标/纠偏清零并显示“控制 禁止”。
- A/B 分段诊断（本控制器镜像，**默认关闭**）：台架按一次遥控“特技”键
  才开启（会先锁电机）；不按键时走常规采样/遥测路径，诊断函数不逐拍调用。
  开启后用“连续有效状态起始 tick”与当前 tick 的差判断满 2s；中途失效
  重新计时。满 2s 后 A 段 10s 只跑收包+姿态估计（双环关闭），B 段 10s
  再开双环。两段内 **暂缓所有常规 L400/L400S 行**。两段结果先存快照，
  再用 `UartTrySend()` 逐行发送两行 `L400D` 汇总；串口忙时保留待发，
  发完自动恢复正常遥测；队列为空后不再逐拍调用诊断。`rfirq` 只统计
  确实有射频中断源的 ISR 调用。A/B 期间 GUI 约 20s 无新观察帧、
  显示“串口数据中断”是预期现象。
  链路中途失效会中止：保留已完成 A 段快照和中止原因，发送
  `L400D ABORT ...`（如有 A 段则先发 A 行），发完后恢复遥测；再按一次
  特技键可重跑。横滚/俯仰目标只在有效杆量变化时重算；所有有效包照常
  刷新包龄，失联/超时后的首个有效包强制重算。
- 报告延期原因：发生余量不足或 UART/DMA 忙时，优先在剩余 >200us 或
  无常规报告占用本拍时补发一行 `L400T budget=... busy=...`；监视器打印
  `[报告延期]` 并写入 `<统计 CSV>.diag.txt`。L400S 里的 `txd` 仍是两类之和，
  排查时看 `L400T` 的独立计数。
- 统计区：采样 `n`、姿态更新 `eul`、读失败 `f`、坏包 `bad`；
  新固件显示“整拍超期 `miss`、控制超期 `cmiss`、跳拍 `skip`、串口推迟 `txd`、
  控制完成 `ctl=本周期最大/上电峰值`”，以及分项耗时：
  `late=原定拍点→采样开始`、`exec=IMU+估计+控制`、`rx=无线整包处理`、
  `tx=串口报告格式化与提交`（本窗口平均/峰值，us）。
  旧 7d01167 没有 `cmiss/ctl`，仍显示 `gap/cyc`。
- 观察数据写 `--csv`（默认 `l400_时间戳.csv`），统计写 `--stats-csv`
  （默认在观察文件名后加 `_stats.csv`），两份分开保存。
  观察 CSV 追加 `tgt_roll_deg100/tgt_pitch_deg100/tgt_rate_x_q10/tgt_rate_y_q10/
  tgt_rate_z_q10/ctrl_ok/qw_q15/qx_q15/qy_q15/qz_q15`；统计 CSV 追加
  `cmiss/ctrl_miss/ctrl_max_us/ctrl_peak_us`，旧固件/旧格式没有的字段留空。
- 实际串口行样例（旧 7d01167）：

  ```text
  L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 g=+512/-256/+0 c=-128/+64/-0
  L400S bad=0 n=12345 f=0 gap=0 eul=12000 miss=0 skip=0 txd=0 cyc=2490/3100
  ```

  当前固件的常规姿态/电机观测是短的定宽 `L400Q`（最长 118 字节）：

  ```text
  L400Q f=F a=0023 v=0012 k=001F q=7FFF/0000/0000/0000 r=00064/00000/FFFFF c=FFC0/0000/0040 m=1 x=0 pwm=064/05A/03C/032
  L400S bad=0000 n=0012C0 f=0000 eul=0012C0 miss=000003 cmiss=000001 skip=000000 txd=000002 ctl=09BA/0C1C late=04B0/09C4 exec=0384/0514 rx=0028/005A tx=0032/0096
  ```

  - `f`：状态位，bit0=姿态就绪、bit1=链路、bit2=指令有效、bit3=控制使能；
  - `a/v/k`：链路包龄、有效摇杆包龄、有效包最大间隔，ms 定宽十六进制；
  - `q`：四个 Q15 两补码分量，+32768 输出为 +32767；
  - `r`：三轴实测角速度 Q10，5 位有符号二十位十六进制；
  - `c`：三轴纠偏量 Q10，4 位有符号十六位十六进制；
  - `m`：电机解锁位（1=已解锁，0=锁定）；
  - `x`：锁定原因（0=无、1=上电、2=失联、3=指令超时、4=姿态异常、
    5=手动急停、6=诊断）；
  - `pwm`：四路**实际**占空比，3 位十六进制，固定顺序 **前/后/左/右**；
  - 角度、°/s、欧拉角全部由 Python 换算，固件不再拼长小数文本。

  `L400S` 仍是 160 字节（等于 `UART_DATA_LEN`）。旧 `7d01167`/`81748c4`
  的 `L400 ...` 长观察行仍可解析；当前镜像不再发送。

  报告延期原因短行（最多 1s 一行）：

  ```text
  L400T budget=12 busy=3
  ```

  prep/Euler_Update 拆分计时（1s 一行，us）：

  ```text
  L400P prep=120/180 eul=45/70 n=200 e=200
  ```

  - `prep`：从采样开始到 `Euler_Update` 之前（IMU 读取、校准、低通、启动判定）；
  - `eul`：只包住 `Euler_Update`；
  - `n`/`e`：本窗口 prep 拍数和 Euler_Update 调用数。

### 手动油门 / 四电机混控 / 锁定

- 手动油门取 FB 通道：`thrust = 190 + fb*140/307`，夹到 `60..340 PWM`；
  中心约悬停，满行程约 50/330 后限到 60/340。
- 三轴纠偏 Q10（±4096）→ PWM 差动：横滚/俯仰 `corr*2880>>16`（满量程约
  180，限 ±160），偏航 `corr*1920>>16`（满量程约 120，限 ±160）；系数已含
  十字 + 型相对 X 型的 1.414 补偿。
- 混控（固定前/后/左/右）：
  `前=thr+dp+dy`、`后=thr-dp+dy`、`左=thr+dr-dy`、`右=thr-dr-dy`；
  机头抬高时 `corr_pitch<0`，`dp<0`，所以前减小、后增大（后 > 前）。
  任一路越界时三轴差动按同一比例缩放，保持平均推力不变，再夹到 `45..400`。
- 上电默认锁定，四路 0。解锁：油门拉到低位（`fb<=-250`），横滚/俯仰/偏航
  回中（各轴 `|cmd|<=45`），长按“一键起降”键 **1s**；“速度转换”键在解锁后
  作为手动急停；“特技”键仍是 A/B 诊断，不用于电机。台架可短按“无头模式”
  键注入姿态异常，效果与真实 IMU 读失败/Euler 更新失败相同（立即 ATT 锁定）。
- 失联、指令超时、姿态未更新/Euler 异常、A/B 诊断开始、手动急停都会立即
  清四路并置锁定；恢复后必须松开解锁键，再重新长按 1s。上电/失联后按住
  解锁键不会自动解锁。**急停键按住期间四路强制为 0，解锁键完全无效**；
  松开急停后还要先松开解锁键，再重新长按。
- 物理映射沿用 V12：前=LF/PWM1，后=LB/PWM3，左=RB/PWM0，右=RF/PWM2。
  无桨逐项检查时以 `pwm` 的前/后/左/右顺序和实际机臂位置对照。

  A/B 结束后的一次性汇总（耗时单位 us，A=无内环，B=有内环）：

  ```text
  L400D A late=900/1500 exec=1000/1600 cmiss=0 skip=0 rfirq=40/80 rf=30/60 app=20/50
  L400D B late=950/1600 exec=1400/2200 cmiss=0 skip=0 rfirq=42/85 rf=32/64 app=22/55
  ```

  监视器会在控制台打印 `[诊断]`，并把两行原样追加到
  `<统计 CSV>.diag.txt`；A/B 期间不写常规 CSV 行。掉链中止会额外输出：

  ```text
  L400D ABORT seg=B reason=cmd haveA=1
  ```

  若 A 已完成，会先补发 A 行；`reason=link/cmd`、`haveA` 说明中止原因和
  已保留的阶段。

### 无桨台架电机验收

1. **上电锁定**：四路 `pwm=000/000/000/000 m=0 x=1`；未解锁前推油门不应有输出。
2. **解锁**：油门低位，长按“一键起降”1s；`m=1 x=0`，四路进入 45..400 范围。
   解锁后立刻按“速度转换”键，四路应回 0，`m=0 x=5`（手动急停）；再长按
   解锁键必须重新开始。
3. **失联**：解锁后关遥控器，指令超时/链路超时后四路必须回 0，`m=0 x=3/2`；
   重开遥控器后仍保持锁定，必须松开再长按解锁键。
4. **姿态异常**：上电时晃动使 `rdy=0`，或断开 IMU 制造读失败；四路应保持/回到
   0，`m=0 x=4`。解锁后也可短按“无头模式”键注入 ATT 锁定；恢复/重新解锁后
   再试。
5. **三轴反向**：无桨固定机架，油门保持在低位/略高，分别向左右打横滚、前后打
   俯仰、左右打偏航，确认 `pwm` 前/后/左/右的增减方向正确；再反向拨杆，
   四路差动方向应完全相反。
6. **电机编号/转向**：按 `pwm` 前/后/左/右顺序对照 V12 的
   前=LF/PWM1、后=LB/PWM3、左=RB/PWM0、右=RF/PWM2；确认转向与桨的 CW/CCW
   要求一致。任何一项不符先不要装桨。

- 本轮控制器联调测试镜像（未纳入 `firmware_delivery/daily`）：
  `telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\Robin_24G_Car_rc_loop400_ctrl.bin`，
  35252 字节，SHA256 `C18D6F1B5162E12D2056A48C809DF7B207EA0F77D5F48540E4A5AB40E0E43D39`，
  烧 0x4000。上一版调度测试镜像 `_sched.bin` 继续保留（26644 / `CAFEAC6C…`）。

- 无硬件自测：`python uart_sensor_monitor.py --l400 --simulate --duration 5`；
  只记录不画图：把 `--simulate` 换成实际串口并加 `--no-plot`。
- 解析/分块收包/CSV/四元数换算自测：`python test_l400_monitor.py`（23 项）。
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

  覆盖机头抬高时 `后 > 前`（小信号和限幅后都成立），以及急停键按住
  期间四路为 0、禁止解锁；松开急停后必须先松开解锁键再重新长按。

- A/B 状态机回归（主机侧）：

  ```powershell
  gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car `
      test_l400_diag.c -o ..\data\test_l400_diag.exe
  ..\data\test_l400_diag.exe
  ```

  覆盖连续有效满 2s 才进 A、A/B 各 10s、失效后重新计时、A/B 掉链中止
  保留已完成阶段/原因，以及 UART 忙后 A/B 两行仍能按序发出。

- 旧 7d01167 十进制格式已在 COM12 实机抓取验证；新十六进制/控制器字段
  已由模拟源、解析和 CSV 测试覆盖，实机控制器验收需烧录 `_ctrl.bin` 后再采集。

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
| 400Hz 联调版（当前 7d01167） | `--l400` | g / c 三轴角速度，单位 °/s |
| 历史姿态版（归档） | `--attitude` | 横滚 / 俯仰 / 相对偏航，单位 ° |
| 六轴校准维护版 | 不加文本参数（默认二进制帧） | 加速度 g / 陀螺 dps |

镜像路径、SHA256 与烧录地址见 [`firmware_delivery/README.md`](../firmware_delivery/README.md)；
**当前板上固件是 7d01167 联调版，直接用 `--l400` 即可，不需要重烧。**

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
