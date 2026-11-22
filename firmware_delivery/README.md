# Robin_24G_Car 烧录镜像交付说明

本目录只放**实际可烧录的镜像副本**和清单；构建目录里的同名文件是编译产物，不作为交付依据。
所有 Car 应用镜像都烧到 **0x4000**（bootloader 占 0x0–0x3FFF；本轮不写 0x10000 以上的 OTA/日志候选区）。
当前主联调版为 **200 Hz 飞控联调**；历史 400 Hz 镜像只作回退参考。

## 0. 对齐修复候选（待无桨验收，不替换日常件）

| 用途 | 候选文件 | 字节 | SHA256 | 烧录地址 | 构建开关 |
|---|---|---:|---|---|---|
| 正常飞控 | [`candidates/Robin_24G_Car_flight_control_alignfix.bin`](candidates/Robin_24G_Car_flight_control_alignfix.bin) | 24900 | `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087` | 0x4000 | `-DC_FLIGHT_CONTROL_EN=1` |
| 解锁诊断 | [`candidates/Robin_24G_Car_flight_arm_debug_alignfix.bin`](candidates/Robin_24G_Car_flight_arm_debug_alignfix.bin) | 30452 | `1FA9323EF35333E201C87A32E4AE6B00F87508C0CC725FFE93BDA10817A1E189` | 0x4000 | `-DC_FLIGHT_CONTROL_EN=1 -DC_FLIGHT_ARM_DEBUG=1` |

- 两份候选均从同一修复提交构建；正常候选无 UART 诊断，诊断候选复用现有
  `--arm-debug` GUI/CSV。
- 应用区：`0x4000+24900=0xA144`、`0x4000+30452=0xB6F4`，均不超过 `0xF000`。
- **日常 28500/`AA3D...` 件和 b3cb56f 回退件保持原样**；候选未通过无桨验收前
  不替换日常件。
- 诊断采集命令（新 CSV 名）：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --arm-debug --duration 60 --csv "..\data\arm_debug_alignfix.csv"
```

## 1. 日常只列这两份

| 用途 | 交付文件 | 字节 | SHA256 | 烧录地址 | 上位机 |
|---|---|---|---|---|---|
| **当前 200 Hz 飞控（无实时串口）** | [`daily/Robin_24G_Car_flight_control.bin`](daily/Robin_24G_Car_flight_control.bin) | 28500 | `AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60` | 0x4000 | 正常运行无实时图；历史报文仍可用 `--l400` 解析 |
| **六轴校准维护版** | [`daily/Robin_24G_Car_maint_269a.bin`](daily/Robin_24G_Car_maint_269a.bin) | 17716 | `F8E59BD29D35C2D8138F9CE60D5202C32907DFB50C7B469EFF60DF123B5C5C38` | 0x4000 | 三个校准脚本；默认 0x269A 六轴模式 |

- **200 Hz 飞控**：构建源码 `f7cf2ae`，行为基线单独记录
  `b3cb56f`；构建开关 `-DC_FLIGHT_CONTROL_EN=1`。5ms 节拍，正常运行不再发送
  `FCQ/FCS`、A/B、计时或延期报告；`FlightTelemetry` 已从主镜像链接移除。
  UART 底层保留给后续停机日志，历史报文解析能力保留。历史 400 Hz 镜像已移到
  `archive/`，仅回退用。
- **应用空间**：`0x4000 + 28500 = 0xAF54 <= 0xF000`，剩余 **16556 字节
  （0x40AC）**；相比移除前 42612 字节，镜像缩小 **14112 字节**。
- **未来日志候选 Flash**：原 OTA 暂存区 `0x10000–0x1BFFF`（48KB，12 个 4KB
  扇区）和标记扇区 `0x1C000–0x1CFFF`（4KB），合计 13 个扇区；本轮不改分区、
  不写日志，留待任务二统一处理。
- **六轴校准维护版**：构建开关 `-DC_MAINT_269A_EN=1`；只发 0x269A 原始六轴二进制帧，
  约 100Hz。`imu_axis_map.py`、`accel_six_face_cal.py`、`gyro_bias_cal.py` 三个脚本都用它；
  默认（不加 `--attitude` / `--flight-control`）的监视器也能看原始六轴波形。

> 维护版是历史原件，SHA256 以现存文件为准。用 e5f07f5/6395fba/当前源码做干净重建会得到
> 同样 17716 字节但哈希为 `E08B51FE...` 的文件，说明历史件不是用当前 makefile 的链接顺序生成的；
> **不要用重建件覆盖这份已在用的原件**，除非先上板验证。

## 2. 归档参考（保留可回退原件）

| 用途 | 文件 | 字节 | SHA256 | 烧录地址 | 说明 |
|---|---|---|---|---|---|
| 拆分前飞控回退件（`b3cb56f`） | [`archive/Robin_24G_Car_flight_control_b3cb56f.bin`](archive/Robin_24G_Car_flight_control_b3cb56f.bin) | 35108 | `F8607D150B56FCDABEDC10F1DE8AE514039CED1CCEB6F500DC5B35A3C8824BF5` | 0x4000 | 拆分前 200Hz 联调镜像，仅回退 |
| 历史 400 Hz 联调版 | [`archive/Robin_24G_Car_rc_loop400.bin`](archive/Robin_24G_Car_rc_loop400.bin) | 25156 | `3F8F62093D97F48E98F8F1B287C05011B4CDCA534CB74E49657F19AD547AA821` | 0x4000 | `7d01167`，`-DC_RC_LOOP_400_EN=1`，L400 长行，仅回退 |
| 单独的 RC 联调版 | [`archive/Robin_24G_Car_rc_bringup.bin`](archive/Robin_24G_Car_rc_bringup.bin) | 18228 | `476628BD9A3FE679E857104A36956E611ECCE775E327FD8D3CC76235EE152F0F` | 0x4000 | 构建开关 `-DC_RC_BRINGUP_EN=1`，串口输出 `RC lk=...`；当前监视器未解析这种文本，仅归档 |
| 历史姿态版（重建回退件） | [`archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin`](archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin) | 24852 | `54CE488F2F0AB045A924CFF2CFF25772798D0F02D12F841AD88276746D781BD1` | 0x4000 | 默认台架姿态模式（`C_ATT_BENCH_EN=1`，无联调宏），串口输出 `ATT ...` / `STA ...`；监视器用 `--attitude` |

历史姿态版的说明要单独看：

- 原来的 `Robin_24G_Car_Flash.bin`（24852 字节，提交 `766ae75` 记录 SHA256 前缀 `DFFB6463...`）
  在 `40cae79` 的 L400 构建中被同名通用文件覆盖，仓库和构建目录里都已经找不到原件。
- 本目录的归档件是用 `dad49ff` 源码重新构建的姿态回退件，大小同为 24852 字节、模式为
  `ATT/STA` 文本，但**没有和丢失的原件逐字节比对过**，哈希是新文件的 `54CE488F...`。
  它能作为 `--attitude` 的回退镜像，但不能冒充原件。
- 如果以后从旧电脑/旧烧录目录找回原件，应先比对 `DFFB6463...` 前缀和 24852 字节，再替换本归档件。

### 专用解锁诊断镜像

[`arm_debug/Robin_24G_Car_flight_arm_debug.bin`](arm_debug/Robin_24G_Car_flight_arm_debug.bin)

```text
33876 字节
SHA256 DFDBE4E4C6888388EAC037DF88333A55E796A7C054A80C2C045A2705458A3955
烧录地址 0x4000
0x4000 + 33876 = 0xC454 <= 0xF000，剩余 11180 字节
```

构建开关 `-DC_FLIGHT_ARM_DEBUG=1`；正式镜像仍静默，交付件哈希不变。
诊断镜像走同一 IMU/遥控/解锁/PWM 路径，仅额外从主循环非阻塞发送：

- `ADB` 启动，`st` 是 IMU 初始化真实返回码，`rb` 是 lpf 回读；
- `ADS` 每秒摘要；
- `ADI` 每秒六轴读取成功/失败、最长连续静置、静置打断原因计数；
- `ADU` 每次新 `0002` 的原始油门/四轴/门限/接受或拒绝；
- `ADG` 被拒绝时的三轴加速度（mg）和角速度（Q10）；
- `ADO` 解锁后第一拍；
- `ADL` 解锁后锁回；
- `ADE` 急停后的四路零 PWM。

GUI 命令：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
python uart_sensor_monitor.py --port COM12 --arm-debug --duration 60 --csv "..\data\arm_debug.csv"
```

默认图形窗口，四块依次为 IMU/姿态、遥控/按键、解锁结果、四路 PWM；
`--no-plot` 是控制台备用。窗口关闭会释放串口并写完 CSV。
CSV 含板端时间、主机接收时间、尝试编号、诊断字段和原始行。

## 3. 构建产物 ≠ 交付镜像

构建目录：

```text
telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\
```

这里的通用 `.bin` 与按模式命名的构建副本**已清理**，不再作为交付路径保留：
`Robin_24G_Car_Flash.bin`、`Robin_24G_Car_rc_loop400.bin`、`Robin_24G_Car_maint_269a.bin`、
`Robin_24G_Car_rc_bringup.bin`、`Robin_24G_Remote_Flash.bin`、
`Robin_24G_Bootloader_Flash.bin` 共 6 份，删除前都已与下方保留件逐个核对哈希。

构建目录当前主产物：

```text
Robin_24G_Car_flight_control.bin   28500 字节
SHA256 AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60
.text 27976 / .data 440 / .bss 1246 / 链接器栈预留 600 字节
```

构建 post-build 已接入 `tools/check_bin_size.py --limit 45056`：超限返回非零并
停止发布；交付副本由 `tools/package_flight_control.py` 检查后再原子复制。
清单中 `build_commit` 记录**实际构建源码**，`behavior_baseline` 单独记录
`b3cb56f`，两者不再混用。

拆分后 `UserRxdFunc.c` 只保留板级入口和无线解码，5ms 调度在 `FlightLoop`；
`RcInput`、`AttitudeControl`、`RateControl`、`MotorMixer`、`FlightSafety`
各自持有状态。主飞控不再链接 `FlightTelemetry`，正常运行时无 FCQ/FCS、
A/B、分项计时或延期报告；`UserUart` 底层保留。上电默认锁定；左侧上键长按
发 `0002`，在油门低位、三轴回中、姿态就绪、链路有效且该包有效时解锁；
左侧下键短按发 `0001`，收到即急停并清四路。每个事件沿只处理一次；
急停/失联/姿态异常后必须新的 `0002`。

拆分前 `_ctrl` 已移入 `archive/Robin_24G_Car_flight_control_b3cb56f.bin`
（35108 / `F8607D15...`）；`_sched.bin` 仍留在构建目录作调度回退参考，
两者都不再作为当前主联调镜像。A/B 台架诊断已不随主镜像提供。

**OTA 决定（本轮）**：调试期间停用 OTA；原 OTA 暂存区只作为飞行日志候选，
不在本轮改 Flash 分区、OTA 启动路径或日志写入。这些与日志落盘一起放到任务二。

以后重新编译仍会生成 `Robin_24G_Car_Flash.bin`、`.lst`、`.elf` 等通用产物，
**不要**按文件名判断它是姿态版、维护版还是 L400 版；烧录优先使用
`firmware_delivery/daily/` 下的交付副本，并按上表的 SHA256 核对。

## 4. 构建模式核对方法

本轮不是只看文件名，做了三层核对：

1. **哈希**：对受版本管理的日常/归档件、基线和构建目录的当前/历史镜像
   分别计算 SHA256。
2. **镜像内字符串**：
   - 200 Hz 飞控：不得包含 `FCQ`/`FCS`、`L400Q`/`L400S`、`L400D` 发送字符串；
   - RC 联调：包含 `RC lk=`；
   - 姿态：包含 `ATT t=`、`STA n=`；
   - 维护：以上文本都没有，只有 0x269A 二进制帧逻辑。
3. **重建对照**：
   - 当前飞控在 `-DC_FLIGHT_CONTROL_EN=1` 下干净重建两次，哈希都是 `AA3D5359...`；
   - 旧 `C_RC_LOOP_400_EN` 仅作为兼容别名，不再作为新构建入口；
   - 维护/RC/姿态的历史原件与“同源码 + 当前 makefile 链接顺序”的重建件大小相同、哈希不同，
     说明历史件使用了不同的链接顺序/构建环境。历史件保留原样，不用重建件替换。

## 5. 快速核对

```powershell
Set-Location "E:\泡沫纸飞机\code"
Get-FileHash "firmware_delivery\daily\Robin_24G_Car_flight_control.bin" -Algorithm SHA256
Get-FileHash "firmware_delivery\daily\Robin_24G_Car_maint_269a.bin" -Algorithm SHA256
```

期望值就是第 1 节表格里的两个 SHA256。

也可以直接跑清单核对：

```powershell
python firmware_delivery\verify_hashes.py
```

**退出码口径**：只由受版本管理的日常镜像、归档镜像和原始基线决定。构建目录里的
通用 `Robin_24G_Car_Flash.bin` 已清理，重新编译才可能出现，干净检出时也不存在；
脚本只打印 `[参考-缺失]` / `[参考-变化]` / `[OK]`，**不影响退出码**，
交付镜像仍能通过核对。

这两个场景（构建产物缺失/哈希变化仍通过，日常镜像变化才失败）的自测：

```powershell
python firmware_delivery\test_verify_hashes.py
```

## 6. 和监视器/校准脚本的对应关系

| 镜像 | 监视器参数 | 校准脚本 |
|---|---|---|
| 200 Hz 飞控（无实时串口） | 无；历史报文可用 `--l400` 解析 | 不适用（正常运行不发实时文本） |
| 六轴校准维护版 | 默认二进制帧模式（不加 `--attitude` / `--flight-control`） | `imu_axis_map.py`、`accel_six_face_cal.py`、`gyro_bias_cal.py` |
| 历史姿态版（归档） | `--attitude` | 不适用 |
| RC 联调版（归档） | 当前未解析（文本 `RC lk=...`） | 不适用 |

具体串口命令见 [`../tools/README.md`](../tools/README.md) 第 4 节。
