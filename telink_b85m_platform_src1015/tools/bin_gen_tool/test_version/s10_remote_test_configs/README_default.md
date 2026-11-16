# S10 遥控器测试配置文档 - DEFAULT 模式

本目录包含 S10 遥控器 **default** 测试模式的配置文件。

## 测试模式说明

默认配置 - 所有参数使用默认值

## 配置统计

- default: 1 个
- **总计**: 1 个配置

## 管脚映射说明

SOP10 封装遥控器引脚功能定义（参考 GPIO_PinTypeDef S10RmotePin）：

| 引脚索引 | GPIO引脚 | 功能说明 |
|----------|----------|----------|
| pin0 | PA0 | KEY_COM (按键公共端) |
| pin1 | PA6 | KEY_F_B (前进/后退按键) |
| pin2 | PB0 | KEY_L_R (左转/右转按键) |
| pin3 | PA3 | KEY_K_S (功能按键K/S) |
| pin4 | PA5 | KEY_P_LED (配对按键/LED输出) |

### 引脚功能详细说明

| 功能名称 | enum值 | 说明 |
|----------|--------|------|
| KEY_COM | 0 | 按键公共端 (接地参考) |
| KEY_F_B | 1 | 前进/后退方向按键输入 |
| KEY_L_R | 2 | 左转/右转方向按键输入 |
| KEY_K_S | 3 | 功能按键K/S输入 |
| KEY_P_LED | 4 | 配对按键输入/LED状态输出 |

## 配置文件列表

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| S10_REMOTE_DEFAULT_0x0f000.bin | default | pin1=KEY_F_B, pin2=KEY_L_R, pin3=KEY_K_S, pin4=KEY_P_LED, shutdown_volt=DEFAULT, low_batt_volt=DEFAULT | S10 遥控器 - 默认配置（所有参数使用默认值） |

## 参数配置说明

### S10_REMOTE_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | KEY_COM | 引脚0功能配置 |
| pin1 | KEY_F_B | 引脚1功能配置 |
| pin2 | KEY_L_R | 引脚2功能配置 |
| pin3 | KEY_K_S | 引脚3功能配置 |
| pin4 | KEY_P_LED | 引脚4功能配置 |
| reserved0 | 0 | 保留位 |
| reserved1 | 0 | 保留位 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | DEFAULT (不检测) | 关机电压 |
| low_batt_volt | DEFAULT (不检测) | 低电电压 |
| sleep_timeout | 10s | 休眠超时 |
| paired_indicator | DEFAULT (默认值是OFF) | 配对指示灯 |

## 可测试参数说明

### shutdown_volt - 关机电压
可选值: 0(DEFAULT (不检测)), 1(2.0V), 2(2.1V), 3(2.2V), 4(OFF)

### low_batt_volt - 低电电压
可选值: 0(DEFAULT (不检测)), 1(2.2V), 2(2.3V), 3(2.4V), 4(OFF)

### sleep_timeout - 休眠超时
可选值: 0(DEFAULT), 1(5s), 2(10s), 3(30s), 4(60s)

### paired_indicator - 配对指示灯
可选值: 0(DEFAULT), 1(OFF), 2(ALWAYS)

## 烧录说明

- **烧录地址**: 0xF000 (Flash)
- **烧录地址**: 0x00000 (EFuse)
- **文件大小**: 256 bytes (Flash), 32 bytes (EFuse)
- **未写数据**: 0xFF

