# 模式配置文件说明

本目录包含 SOP10 Remote 和根据 `TOY_S10_CAR_DEFAULT_MODE`、`TOY_S16_CAR_DEFAULT_MODE` 宏生成的配置文件。

## 文件命名规则

```
{宏名}_{烧录地址}.bin
{宏名}_{目标类型}.json
```

例如:
- `TOY_S10_CAR_MODE_DIFF_4PWM_LED_0x0f000.bin` - S10四驱LED模式，Flash烧录地址 0xF000
- `TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_0x00000.bin` - S16双电机挖掘机模式，EFuse烧录地址 0x00

## S10 遥控器配置 (SOP10_REMOTE_DEFAULT)

### SOP10_REMOTE_DEFAULT
**SOP10 封装遥控器默认配置**

| 参数 | 值 | 说明 |
|-----|-----|------|
| toy_type | 0 | 遥控器 |
| pin0 | KEY_COM | 公共按键 |
| pin1 | KEY_F_B | 前后按键 |
| pin2 | KEY_L_R | 左右按键 |
| pin3 | KEY_K_S | 功能按键 |
| pin4 | KEY_P_LED | LED控制 |
| shutdown_volt | OFF | 不检测关机电压 |
| low_batt_volt | OFF | 不检测低电电压 |
| sleep_timeout | DEFAULT (10s) | 休眠超时 |

文件:
- `SOP10_REMOTE_DEFAULT_0x0f000.bin` (Flash)
- `SOP10_REMOTE_DEFAULT_0x00000.bin` (EFuse)
- `SOP10_REMOTE_DEFAULT_flash.json`
- `SOP10_REMOTE_DEFAULT_efuse.json`

---

## S10 小车配置 (TOY_S10_CAR_DEFAULT_MODE)

### 1. TOY_S10_CAR_MODE_DIFF_4PWM_LED
**四驱/差速底盘 + 4路PWM + LED**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 0 | 四驱/差速 |
| special_func | LED | LED状态灯 |
| pin0 | PWM_RF | PA0 - 右侧轮子向前 |
| pin1 | PWM_RB | PA6 - 右侧轮子向后 |
| pin2 | PWM_LF | PB0 - 左侧轮子向前 |
| pin3 | PWM_SPRAY_LED | PA3 - LED (GPIO) |
| pin4 | PWM_LB | PA5 - 左侧轮子向后 |

文件:
- `TOY_S10_CAR_MODE_DIFF_4PWM_LED_0x0f000.bin` (Flash)
- `TOY_S10_CAR_MODE_DIFF_4PWM_LED_0x00000.bin` (EFuse)
- `TOY_S10_CAR_MODE_DIFF_4PWM_LED_flash.json`
- `TOY_S10_CAR_MODE_DIFF_4PWM_LED_efuse.json`

---

### 2. TOY_S10_CAR_MODE_2MOTOR_SPRAY
**双电机底盘 + 2路PWM + 喷雾**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 1 | 双电机 |
| special_func | SPRAY | 喷雾功能 |
| pin0 | PWM_RF | PA0 - 右侧轮子向前 |
| pin1 | PWM_RB | PA6 - 右侧轮子向后 |
| pin2 | PWM_SPRAY_LED | PB0 - 喷雾 (PWM 108kHz) |
| pin3 | PWM_LF | PA3 - 左侧轮子向前 (GPIO) |
| pin4 | PWM_LB | PA5 - 左侧轮子向后 (GPIO) |

文件:
- `TOY_S10_CAR_MODE_2MOTOR_SPRAY_0x0f000.bin` (Flash)
- `TOY_S10_CAR_MODE_2MOTOR_SPRAY_0x00000.bin` (EFuse)
- `TOY_S10_CAR_MODE_2MOTOR_SPRAY_flash.json`
- `TOY_S10_CAR_MODE_2MOTOR_SPRAY_efuse.json`

---

### 3. TOY_S10_CAR_MODE_2MOTOR_LED
**双电机底盘 + 2路PWM + LED**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 1 | 双电机 |
| special_func | LED | LED状态灯 |
| pin0 | PWM_RF | PA0 - 右侧轮子向前 |
| pin1 | PWM_RB | PA6 - 右侧轮子向后 |
| pin2 | PWM_LF | PB0 - 左侧轮子向前 (GPIO) |
| pin3 | PWM_SPRAY_LED | PA3 - LED (GPIO) |
| pin4 | PWM_LB | PA5 - 左侧轮子向后 (GPIO) |

文件:
- `TOY_S10_CAR_MODE_2MOTOR_LED_0x0f000.bin` (Flash)
- `TOY_S10_CAR_MODE_2MOTOR_LED_0x00000.bin` (EFuse)
- `TOY_S10_CAR_MODE_2MOTOR_LED_flash.json`
- `TOY_S10_CAR_MODE_2MOTOR_LED_efuse.json`

---

## S16 大车配置 (TOY_S16_CAR_DEFAULT_MODE)

### 1. TOY_S16_CAR_MODE_DIFF_SPRAY
**差速底盘 + 喷雾**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 0 | 四驱/差速 |
| special_func | SPRAY | 喷雾功能 |
| spray_w_mask | 0x03 | W1,W2 用于喷雾 |
| pin0 | PWM_LF | PA0 - 左侧轮子向前 |
| pin1 | PWM_RF | PA1 - 右侧轮子向前 |
| pin2 | PWM_W1 | PA2 - 喷雾W1 |
| pin3 | PWM_W3 | PA3 - 未使用 (SWS) |
| pin4 | GPIO_LED_OUT | PA4 - LED |
| pin5 | CHG_FULL | PA5 - 充电充满检测 |
| pin6 | CHG_DETECT | PA6 - 充电检测 |
| pin7 | PWM_W4 | PA7 - 未使用 |
| pin8 | PWM_LB | PB0 - 左侧轮子向后 |
| pin9 | PWM_RB | PB1 - 右侧轮子向后 |
| pin10 | PWM_W2 | PB2 - 喷雾W2 |

文件:
- `TOY_S16_CAR_MODE_DIFF_SPRAY_0x0f000.bin` (Flash)
- `TOY_S16_CAR_MODE_DIFF_SPRAY_0x00000.bin` (EFuse)
- `TOY_S16_CAR_MODE_DIFF_SPRAY_flash.json`
- `TOY_S16_CAR_MODE_DIFF_SPRAY_efuse.json`

---

### 2. TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR
**双电机底盘 + 挖掘机功能**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 1 | 双电机 |
| special_func | EXCAVATOR | 挖掘机功能 |
| spray_w_mask | 0x00 | 不使用喷雾 |
| pin0 | PWM_RF | PA0 - 右侧轮子向前 |
| pin1 | PWM_RB | PA1 - 右侧轮子向后 |
| pin2 | PWM_W1 | PA2 - 转盘左转 |
| pin3 | GPIO_LED_OUT | PA3 - LED (SWS) |
| pin4 | PWM_LF | PA4 - 左侧轮子向前 (GPIO) |
| pin5 | PWM_LB | PA5 - 左侧轮子向后 (GPIO) |
| pin6 | CHG_DETECT | PA6 - 充电检测 |
| pin7 | CHG_FULL | PA7 - 充电充满检测 |
| pin8 | PWM_W2 | PB0 - 转盘右转 |
| pin9 | PWM_W3 | PB1 - 抓臂向下 |
| pin10 | PWM_W4 | PB2 - 抓臂向上 |

文件:
- `TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_0x0f000.bin` (Flash)
- `TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_0x00000.bin` (EFuse)
- `TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_flash.json`
- `TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_efuse.json`

---

### 3. TOY_S16_CAR_MODE_2MOTOR_SPRAY
**双电机底盘 + 喷雾**

| 参数 | 值 | 说明 |
|-----|-----|------|
| drive_mode | 1 | 双电机 |
| special_func | SPRAY | 喷雾功能 |
| spray_w_mask | 0x0F | W1-W4 可用于喷雾 |
| pin0 | PWM_LF | PA0 - 左侧轮子向前 (GPIO) |
| pin1 | PWM_RF | PA1 - 右侧轮子向前 |
| pin2 | PWM_W1 | PA2 - 喷雾W1 |
| pin3 | GPIO_LED_OUT | PA3 - LED (SWS) |
| pin4 | PWM_LB | PA4 - 左侧轮子向后 (GPIO) |
| pin5 | PWM_RB | PA5 - 右侧轮子向后 |
| pin6 | CHG_DETECT | PA6 - 充电检测 |
| pin7 | CHG_FULL | PA7 - 充电充满检测 |
| pin8 | PWM_W2 | PB0 - 未使用 |
| pin9 | PWM_W3 | PB1 - 未使用 |
| pin10 | PWM_W4 | PB2 - 未使用 |

文件:
- `TOY_S16_CAR_MODE_2MOTOR_SPRAY_0x0f000.bin` (Flash)
- `TOY_S16_CAR_MODE_2MOTOR_SPRAY_0x00000.bin` (EFuse)
- `TOY_S16_CAR_MODE_2MOTOR_SPRAY_flash.json`
- `TOY_S16_CAR_MODE_2MOTOR_SPRAY_efuse.json`

---

## 烧录说明

### Flash 烧录
- **地址**: 0xF000
- **文件后缀**: `_0x0f000.bin`
- **大小**: 256 bytes
- **未写数据**: 0xFF

### EFuse 烧录
- **地址**: 0x00
- **文件后缀**: `_0x00000.bin`
- **大小**: 32 bytes
- **未写数据**: 0x00
- **注意**: EFuse 只能烧录一次！

## 快速使用

```bash
# 进入测试版目录
cd tools/bin_gen_tool/test_version

# 重新生成所有模式配置
python3 generate_mode_configs.py

# 查看生成的文件
ls -la mode_configs/
```
