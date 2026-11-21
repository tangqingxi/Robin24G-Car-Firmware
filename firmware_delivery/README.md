# Robin_24G_Car 烧录镜像交付说明

本目录只放**实际可烧录的镜像副本**和清单；构建目录里的同名文件是编译产物，不作为交付依据。
所有 Car 应用镜像都烧到 **0x4000**（bootloader 占 0x0–0x3FFF，0x10000 以上区域不动）。
当前板上运行的是 **7d01167 联调固件**，继续使用即可，不需要因为本轮文档改动而重烧。

## 1. 日常只列这两份

| 用途 | 交付文件 | 字节 | SHA256 | 烧录地址 | 上位机 |
|---|---|---|---|---|---|
| **当前 400Hz 联调版** | [`daily/Robin_24G_Car_rc_loop400.bin`](daily/Robin_24G_Car_rc_loop400.bin) | 25156 | `3F8F62093D97F48E98F8F1B287C05011B4CDCA534CB74E49657F19AD547AA821` | 0x4000 | `uart_sensor_monitor.py --l400` |
| **六轴校准维护版** | [`daily/Robin_24G_Car_maint_269a.bin`](daily/Robin_24G_Car_maint_269a.bin) | 17716 | `F8E59BD29D35C2D8138F9CE60D5202C32907DFB50C7B469EFF60DF123B5C5C38` | 0x4000 | 三个校准脚本；默认 0x269A 六轴模式 |

- **400Hz 联调版**：对应提交 `7d01167`，构建开关 `-DC_RC_LOOP_400_EN=1`；串口输出
  10Hz 的 `L400 ...` 和 1Hz 的 `L400S ...`。监视器用 `--l400`，画 g/c 三轴角速度（°/s），
  **不是欧拉角**。
- **六轴校准维护版**：构建开关 `-DC_MAINT_269A_EN=1`；只发 0x269A 原始六轴二进制帧，
  约 100Hz。`imu_axis_map.py`、`accel_six_face_cal.py`、`gyro_bias_cal.py` 三个脚本都用它；
  默认（不加 `--attitude` / `--l400`）的监视器也能看原始六轴波形。

> 维护版是历史原件，SHA256 以现存文件为准。用 e5f07f5/6395fba/当前源码做干净重建会得到
> 同样 17716 字节但哈希为 `E08B51FE...` 的文件，说明历史件不是用当前 makefile 的链接顺序生成的；
> **不要用重建件覆盖这份已在用的原件**，除非先上板验证。

## 2. 归档参考（保留可回退原件）

| 用途 | 文件 | 字节 | SHA256 | 烧录地址 | 说明 |
|---|---|---|---|---|---|
| 单独的 RC 联调版 | [`archive/Robin_24G_Car_rc_bringup.bin`](archive/Robin_24G_Car_rc_bringup.bin) | 18228 | `476628BD9A3FE679E857104A36956E611ECCE775E327FD8D3CC76235EE152F0F` | 0x4000 | 构建开关 `-DC_RC_BRINGUP_EN=1`，串口输出 `RC lk=...`；当前监视器未解析这种文本，仅归档 |
| 历史姿态版（重建回退件） | [`archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin`](archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin) | 24852 | `54CE488F2F0AB045A924CFF2CFF25772798D0F02D12F841AD88276746D781BD1` | 0x4000 | 默认台架姿态模式（`C_ATT_BENCH_EN=1`，无联调宏），串口输出 `ATT ...` / `STA ...`；监视器用 `--attitude` |

历史姿态版的说明要单独看：

- 原来的 `Robin_24G_Car_Flash.bin`（24852 字节，提交 `766ae75` 记录 SHA256 前缀 `DFFB6463...`）
  在 `40cae79` 的 L400 构建中被同名通用文件覆盖，仓库和构建目录里都已经找不到原件。
- 本目录的归档件是用 `dad49ff` 源码重新构建的姿态回退件，大小同为 24852 字节、模式为
  `ATT/STA` 文本，但**没有和丢失的原件逐字节比对过**，哈希是新文件的 `54CE488F...`。
  它能作为 `--attitude` 的回退镜像，但不能冒充原件。
- 如果以后从旧电脑/旧烧录目录找回原件，应先比对 `DFFB6463...` 前缀和 24852 字节，再替换本归档件。

## 3. 构建产物 ≠ 交付镜像

构建目录：

```text
telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\
```

这里的通用 `.bin` 与按模式命名的构建副本**已清理**，不再作为交付路径保留：
`Robin_24G_Car_Flash.bin`、`Robin_24G_Car_rc_loop400.bin`、`Robin_24G_Car_maint_269a.bin`、
`Robin_24G_Car_rc_bringup.bin`、`Robin_24G_Remote_Flash.bin`、
`Robin_24G_Bootloader_Flash.bin` 共 6 份，删除前都已与下方保留件逐个核对哈希。

构建目录目前保留两个联调测试镜像：

```text
Robin_24G_Car_rc_loop400_sched.bin   26644 字节
SHA256 CAFEAC6C8551889CD7F81225B204408841BA331D878BD4E6D9D681368166BF16

Robin_24G_Car_rc_loop400_ctrl.bin    30756 字节
SHA256 45422095F1BA8A23BB5D05CCB368BC810E577256CADF88912140659EA200B365
```

`_sched.bin` 是 `2b7d3c3` 调度测试版；`_ctrl.bin` 是四元数重力方向外环 +
角速度 PID 内环（P=1/4，I=D=0）并带自动 A/B 分段诊断的控制器联调版。
上电链路稳定 2s 后自动跑 A（无内环）10s、B（有内环）10s，期间暂缓常规
遥测，结束后输出 `L400D` 汇总并恢复正常遥测。两份都未纳入
`firmware_delivery/daily`，暂不退役。

以后重新编译仍会生成 `Robin_24G_Car_Flash.bin`、`.lst`、`.elf` 等通用产物，
**不要**按文件名判断它是姿态版、维护版还是 L400 版；烧录优先使用
`firmware_delivery/daily/` 下的交付副本，并按上表的 SHA256 核对。

## 4. 构建模式核对方法

本轮不是只看文件名，做了三层核对：

1. **哈希**：对受版本管理的日常/归档件、三份基线和两个联调测试镜像
   （`_sched.bin`、`_ctrl.bin`）分别计算 SHA256。
2. **镜像内字符串**：
   - L400：包含 `L400 rdy=`、`L400S bad=`、`L400 init ok`；
   - RC 联调：包含 `RC lk=`；
   - 姿态：包含 `ATT t=`、`STA n=`；
   - 维护：以上文本都没有，只有 0x269A 二进制帧逻辑。
3. **重建对照**：
   - 当前 L400 在 `7d01167` + `-DC_RC_LOOP_400_EN=1` 下干净重建两次，哈希都是 `3F8F6209...`；
   - 维护/RC/姿态的历史原件与“同源码 + 当前 makefile 链接顺序”的重建件大小相同、哈希不同，
     说明历史件使用了不同的链接顺序/构建环境。历史件保留原样，不用重建件替换。

## 5. 快速核对

```powershell
Set-Location "E:\泡沫纸飞机\code"
Get-FileHash "firmware_delivery\daily\Robin_24G_Car_rc_loop400.bin" -Algorithm SHA256
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
| 400Hz 联调版 | `--l400` | 不适用（发 L400 文本，不是 0x269A） |
| 六轴校准维护版 | 默认二进制帧模式（不加 `--attitude` / `--l400`） | `imu_axis_map.py`、`accel_six_face_cal.py`、`gyro_bias_cal.py` |
| 历史姿态版（归档） | `--attitude` | 不适用 |
| RC 联调版（归档） | 当前未解析（文本 `RC lk=...`） | 不适用 |

具体串口命令见 [`../tools/README.md`](../tools/README.md) 第 4 节。
