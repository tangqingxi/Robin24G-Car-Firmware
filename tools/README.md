# Robin_24G_Car 传感器校准与监视工具（操作手册）

四个命令行工具 + 两个共用模块 + 一个自测脚本：

| 文件 | 用途 |
|---|---|
| `imu_axis_map.py` | 传感器轴 → 机体 FRD 映射（六面摆法） |
| `accel_six_face_cal.py` | 加速度计六面校准 / 单面补测 |
| `gyro_bias_cal.py` | 陀螺仪静止零偏 |
| `uart_sensor_monitor.py` | 六轴波形、台架姿态（`--attitude`）与 400Hz 联调（`--l400`）监视与 CSV 记录（只查看，不校准） |
| `uart_proto.py` | 0x269A / 0x269B 帧解析（纯标准库，被上面几个工具共用） |
| `calib_common.py` | 三个校准工具共用的串口枚举/只收数据源、输出防覆盖、FRD 映射校验、均值/标准差 |
| `test_l400_monitor.py` | L400 解析、分块收包、两份 CSV 与模拟采集自测 |

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

校准脚本吃的是 **0x269A 原始六轴帧**，当前 400Hz 联调固件发的是 L400 文本，两者不兼容，
所以标定前必须换成维护镜像：

| 用途 | 交付镜像 | 串口输出 |
|---|---|---|
| 六轴校准（本节目录下的三个脚本） | `firmware_delivery/daily/Robin_24G_Car_maint_269a.bin` | 只有 0x269A 二进制帧，约 100 Hz |
| 400Hz 联调（`--l400`） | `firmware_delivery/daily/Robin_24G_Car_rc_loop400.bin` | L400 / L400S 文本 |

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

### 400Hz 联调固件（文本协议，--l400）

7d01167 固件输出 10Hz 的 `L400 ...` 观察行和 1Hz 的 `L400S ...` 统计行；
带分项耗时的新 L400 固件把 `L400S` 改成定宽十六进制，并追加
`late/exec/rx/tx` 四个耗时字段，监视器同时兼容新旧两种格式。
图形监视命令：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --l400 --duration 120 --csv "..\data\l400.csv"
```

- 两张图：上图 **实测角速度 g (°/s)**，下图 **内环 PID 纠偏 c (°/s)**。g 是机体 FRD
  三轴实测角速度，c 是内环 PID 输出，**不是欧拉角**；固件里的 Q10 rad/s
  已在接收端换算成 °/s，图例写成 g-X/g-Y/g-Z、c-X/c-Y/c-Z。
  外环目标姿态/偏航目标角速度在统计区文字和 CSV 里，不额外画曲线。
- 状态栏：姿态就绪、遥控连接、指令有效、有效包龄、峰值（新固件为控制完成峰值）。
  控制使能/禁止和目标姿态显示在下方统计区。包龄最大显示 9999ms，
  从未收到过对应包时也显示 9999。
- 状态新鲜度：用最近一条 L400 观察帧的**主机单调时间**判断；超过约 1s 没有新观察帧时，
  状态栏显示“串口数据中断”，并把遥控/指令状态标为“已过期”；恢复接收后按新帧更新。
- CSV 保护：打开串口和文件之前检查 `--csv` 与 `--stats-csv` 是否指向同一文件；
  相同会明确报错退出，已有文件内容不会被截断。默认六轴和 `--attitude` 模式也做同样检查。
- 调度（新固件）：400Hz 到点先执行 L400 采样/控制，再做 `d24_rx_main_loop`
  的配对/ACK 和应用层收包；射频中断仍保留，控制优先。
- 报告（新固件）：`UartTrySend` 非阻塞提交；离下一拍不足 400us 或串口忙时
  本拍不拼报，延期到下一拍。`txd` 表示**报告延期次数**（预算不足或串口忙），
  不再表示“跨过下一拍截止”；`tx` 只统计真正拼过文本的拍。
- 控制（新固件）：外环 P 用四元数推算的向上比力方向与目标方向做叉积，直接得到
  横滚/俯仰误差；摇杆目标在**有效收包时**换算，400Hz 控制拍不再算欧拉角或三角函数。
  目标横滚/俯仰满杆 ±30°，偏航目标角速度满杆 ±3rad/s；内环 PID 的 P=1/4，
  I/D 为 0 时对应的每拍状态更新不编译，只观察不写电机。欧拉角显示留给上位机。
  失联、指令超时、姿态未就绪或本拍没有新姿态时，目标/纠偏清零并显示“控制 禁止”。
- 统计区：采样 `n`、姿态更新 `eul`、读失败 `f`、坏包 `bad`；
  新固件显示“整拍超期 `miss`、控制超期 `cmiss`、跳拍 `skip`、串口推迟 `txd`、
  控制完成 `ctl=本周期最大/上电峰值`”，以及分项耗时：
  `late=原定拍点→采样开始`、`exec=IMU+估计+控制`、`rx=无线整包处理`、
  `tx=串口报告格式化与提交`（本窗口平均/峰值，us）。
  旧 7d01167 没有 `cmiss/ctl`，仍显示 `gap/cyc`。
- 观察数据写 `--csv`（默认 `l400_时间戳.csv`），统计写 `--stats-csv`
  （默认在观察文件名后加 `_stats.csv`），两份分开保存。
  观察 CSV 追加 `tgt_roll_deg100/tgt_pitch_deg100/tgt_rate_x_q10/tgt_rate_y_q10/
  tgt_rate_z_q10/ctrl_ok`；统计 CSV 追加 `cmiss/ctrl_miss/ctrl_max_us/ctrl_peak_us`，
  旧固件/旧格式没有的字段留空。
- 实际串口行样例（旧 7d01167）：

  ```text
  L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 g=+512/-256/+0 c=-128/+64/-0
  L400S bad=0 n=12345 f=0 gap=0 eul=12000 miss=0 skip=0 txd=0 cyc=2490/3100
  ```

  控制器联调新格式（观察行追加 ta/tw/on；L400S 定宽十六进制，最长 160 字节）：

  ```text
  L400 rdy=1 lk=1 vld=1 alk=35 avld=18 gpk=31 g=+512/-256/+0 c=-128/+64/-0 ta=+3000/-1500 tw=+512/-256/+0 on=1
  L400S bad=0000 n=0012C0 f=0000 eul=0012C0 miss=000003 cmiss=000001 skip=000000 txd=000002 ctl=09BA/0C1C late=04B0/09C4 exec=0384/0514 rx=0028/005A tx=0032/0096
  ```

- 本轮控制器联调测试镜像（未纳入 `firmware_delivery/daily`）：
  `telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\Robin_24G_Car_rc_loop400_ctrl.bin`，
  27636 字节，SHA256 `3D5DEF73451FB13855281A129EAA96DBBA17FD74F933337F91EC2FAFBAE0BD18`，
  烧 0x4000。上一版调度测试镜像 `_sched.bin` 继续保留（26644 / `CAFEAC6C…`）。

- 无硬件自测：`python uart_sensor_monitor.py --l400 --simulate --duration 5`；
  只记录不画图：把 `--simulate` 换成实际串口并加 `--no-plot`。
- 解析/分块收包/CSV 自测：`python test_l400_monitor.py`（16 项）。
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
