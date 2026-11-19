# Robin_24G_Car 传感器校准与监视工具（操作手册）

四个命令行工具 + 一个共用协议模块：

| 文件 | 用途 |
|---|---|
| `imu_axis_map.py` | 传感器轴 → 机体 FRD 映射（六面摆法） |
| `accel_six_face_cal.py` | 加速度计六面校准 / 单面补测 |
| `gyro_bias_cal.py` | 陀螺仪静止零偏 |
| `uart_sensor_monitor.py` | 六轴波形监视与 CSV 记录（只查看，不校准） |
| `uart_proto.py` | 0x269A / 0x269B 帧解析（纯标准库，被上面几个工具共用） |

---

## 0. 准备

**依赖**（Python 3.9+）：

```powershell
python -m pip install pyserial
```

（只有 `uart_sensor_monitor.py` 画曲线时还需要 `matplotlib`、`numpy`；三个校准工具不需要。）

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

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python gyro_bias_cal.py --port COM12 --axis-map "..\data\imu_axis_map_frd.json" --out "..\data\gyro_bias_frd.json"
```

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

### 台架姿态固件（文本协议，--attitude）

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --attitude --duration 120 --csv "..\data\att.csv"
```

- `--attitude` 按固件的 `ATT ...` / `STA ...` 文本行解析（r/p/y 已经是度），
  画横滚、俯仰、相对偏航三条曲线，状态栏同时显示最新三个角度与固件状态
  （ATT init / wait still / ready / 初始化失败）；
- 状态栏还显示最新一条 STA 的有效帧数、读失败、长间断与 dt/rd/up 耗时；
- 姿态样本存 `att.csv`，统计存 `att_stats.csv`，两分钟静置验收直接看这两个文件；
- 不加 `--attitude` 时仍是原来的 0x269A / 0x269B 二进制帧模式，行为和参数不变。

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
| 固件接入 | 先映射到 FRD，再扣 FRD 零偏，最后换算：`omega = (R·g - bias_frd) * 0.0010641` |
