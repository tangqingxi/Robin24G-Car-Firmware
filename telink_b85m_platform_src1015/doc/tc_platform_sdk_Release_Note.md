## V0.0.2(ER)

### Version
* SDK Version: tc122x_remote_car v0.0.2

* Chip Version
  - TC122X A1

* Hardware EVK Version
  - tc122x QFN24:C1T389A20_V1.0
  - tc122x SOP16:C1T412A20_V1.0

* Base SDK Version
  - tc_platform_sdk V3.3.1

* Toolchain Version
  - TC122X: TC32 ELF GCC4.3 ( IDE: [Telink IoT Studio](https://www.telink-semi.com/development-tools) )

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### Bug Fixes
* **[rf]**
    1. Filter out frequency points with poor hardware performance

* **[remote]**
    1. Change battery voltage sampling interval to 500ms

### BREAKING CHANGES
* N/A

### Features
* **[system]**
    1. Add flash protect functionality
    2. Update rom_boot.link to ensure sufficient space for EMI execution
    3. Merge driver SDK (commit: b2d48ada08629c33108039f25e405857ffe1ee31) changes, including ADC and watchdog modules

### Refactoring
* N/A

### Performance Improvements
* N/A

### 版本
* SDK 版本: tc122x_remote_car v0.0.2

* 芯片版本
  - TC122X A1

* 硬件评估板版本
  - tc122x QFN24:C1T389A20_V1.0
  - tc122x SOP16:C1T412A20_V1.0

* 基础 SDK 版本
  - tc_platform_sdk V3.3.1

* 工具链版本
  - TC122X: TC32 ELF GCC4.3 ( IDE: [Telink IoT Studio](https://www.telink-semi.com/development-tools) )

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### Bug 修复
* **[射频]**
    1. 过滤硬件性能不好的频点

* **[遥控器]**
    1. 遥控器采集电量间隔改为 500ms

### 破坏性变更
* N/A

### 特性
* **[系统]**
    1. 增加 flash protect 功能
    2. 更新 rom_boot.link，确保有足够的空间运行 EMI
    3. 合并驱动SDK(commit: b2d48ada08629c33108039f25e405857ffe1ee31)改动，包含 ADC、watchdog 模块

### 重构
* N/A

### 性能优化
* N/A

---

## V0.0.1(ER)

### Version
* SDK Version: tc122x_remote_car v0.0.1

* Chip Version
  - TC122X A1

* Hardware EVK Version
  - tc122x QFN24:C1T389A20_V1.0
  - tc122x SOP16:C1T412A20_V1.0

* Base SDK Version
  - tc_platform_sdk V3.3.1

* Toolchain Version
  - TC122X: TC32 ELF GCC4.3 ( IDE: [Telink IoT Studio](https://www.telink-semi.com/development-tools) )

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### Bug Fixes
* N/A

### BREAKING CHANGES
* N/A

### Features
* **[S10_remote]**
    1. Support 2.4G pairing and reconnection with frequency hopping
    2. Support tower key scanning (5 GPIOs for multi-key input)
    3. Support battery detection (shutdown voltage and low battery warning)
    4. Support idle timeout auto-sleep
    5. Support LED indicator (pairing status, low battery, charging status)
    6. Support efuse parameter configuration

* **[S10_car]**
    1. Support 2.4G pairing and reconnection with frequency hopping
    2. Motor control: 4-channel PWM differential drive or dual-motor drive (front/rear PWM + left/right GPIO)
    3. Support spray function (PWM 108kHz, configurable duration and duty cycle)
    4. Support LED status indicator
    5. Support battery detection and low-voltage shutdown protection
    6. Support direction switch delay protection (0-300ms configurable)
    7. Support auto-sleep (configurable timeout: 5min/10min/15min/never)
    8. Support PWM frequency adjustment (250Hz/500Hz/16kHz)
    9. Support independent PWM duty cycle for left/right wheels
    10. Support efuse parameter configuration

* **[S16_car]**
    1. Support 2.4G pairing and reconnection with frequency hopping
    2. Extended peripherals: 11 GPIOs and 6 PWM channels
    3. Support spray function (W1-W4 multi-channel selectable)
    4. Support excavator function: turntable (W1/W2) and arm (W3/W4)
    5. Support charging detection (charging and fully charged)
    6. Support demo mode: figure-8, circle, square
    7. Support LED status indicator
    8. Support dual-motor chassis mode
    9. Support independent PWM duty cycle for left/right wheels and W channels
    10. Support efuse parameter configuration

### Refactoring
* N/A

### Performance Improvements
* N/A

### 版本
* SDK 版本: tc122x_remote_car v0.0.1

* 芯片版本
  - TC122X A1

* 硬件评估板版本
  - tc122x QFN24:C1T389A20_V1.0
  - tc122x SOP16:C1T412A20_V1.0

* 基础 SDK 版本
  - tc_platform_sdk V3.3.1

* 工具链版本
  - TC122X: TC32 ELF GCC4.3 ( IDE: [Telink IoT Studio](https://www.telink-semi.com/development-tools) )

<hr style="border-bottom:2.5px solid rgb(146, 240, 161)">

### Bug 修复
* N/A

### 破坏性变更
* N/A

### 特性
* **[S10_remote]**
    1. 支持2.4G配对和回连，支持跳频
    2. 支持塔键扫描（5个GPIO实现多按键输入）
    3. 支持电池检测（关机电压、低电电压告警）
    4. 支持空闲超时自动休眠
    5. 支持LED指示灯（配对状态、低电量、充电状态）
    6. 支持efuse参数配置

* **[S10_car]**
    1. 支持2.4G配对和回连，支持跳频
    2. 电机控制：4路PWM差速驱动 或 双电机驱动（前后PWM+左右GPIO）
    3. 支持喷雾功能（PWM 108kHz，可配置喷雾时长和占空比）
    4. 支持LED状态指示灯
    5. 支持电池检测与低电量关机保护
    6. 支持方向切换延迟保护（0-300ms可配置）
    7. 支持自动休眠（可配置超时时间：5分钟/10分钟/15分钟/从不）
    8. 支持PWM频率调节（250Hz/500Hz/16kHz）
    9. 支持左右轮独立PWM占空比控制
    10. 支持efuse配置参数

* **[S16_car]**
    1. 支持2.4G配对和回连，支持跳频
    2. 扩展外设：11个GPIO和6路PWM通道
    3. 支持喷雾功能（W1-W4多通道可选）
    4. 支持挖掘机功能：转盘（W1/W2）和挖臂（W3/W4）
    5. 支持充电检测（充电中、充满检测）
    6. 支持演示模式：8字形、圆形、正方形
    7. 支持LED状态指示灯
    8. 支持双电机底盘模式
    9. 支持左右轮及W通道独立PWM占空比控制
    10. 支持efuse配置参数

### 重构
* N/A

### 性能优化
* N/A

