# Bin 文件生成工具

用于生成包含 `efuse_dev_info_t` 和 `CfgParamUnion` 配置数据的二进制 bin 文件，支持烧录到 Flash 或 EFuse。

## 特性

- 支持三种固件类型：SOP10遥控器、SOP10遥控车、SOP16遥控车
- 支持两种目标位置：Flash（未写数据默认 0xFF）、EFuse（未写数据默认 0x00）
- 可通过 JSON 配置文件自定义参数
- 自动生成 JSON 模板便于编辑

## 数据结构

### efuse_dev_info_t (16 字节)

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0x00-0x05 | chip_id[6] | Chip ID，默认 0x12 0x34 0x56 0x00 0x00 0x00 |
| 0x06 | union_byte_0 | [3]paring_type: 0=自动配对, 1=手动配对 |
| 0x07 | union_byte_1 | [2:0]toy_type: 玩具类型 |
| 0x08 | union_byte_2 | [3:0]access_code_group, [7]flash_disable |
| 0x09 | union_byte_3 | [7:4]normal_tx_power: 发射功率 |
| 0x0A | union_byte_4 | [3:0]ldo_output_trim, [5:4]capless, [7:6]bin_select |
| 0x0B | union_byte_5 | [5:0]xtal_calibration, [6]xtal_type, [7]dig_ldo_mode |
| 0x0C-0x0F | adc_calib | ADC 校准值 (4字节) |

### CfgParamUnion (根据固件类型)

| 固件类型 | 结构体 | 大小 |
|---------|--------|------|
| SOP10 遥控器 | SOP10RemoteCfgParam | 6 字节 |
| SOP10 遥控车 | SOP10CarCfgParam | 8 字节 |
| SOP16 遥控车 | SOP16CarCfgParam | 12 字节 |

## 使用方法

### 1. 生成默认配置

```bash
# SOP10 遥控器 -> Flash
python bin_gen.py --target flash --fw-type remote --output remote_flash.bin

# SOP10 遥控车 -> EFuse
python bin_gen.py --target efuse --fw-type car10 --output car10_efuse.bin

# SOP16 遥控车 -> Flash
python bin_gen.py --target flash --fw-type car16 --output car16_flash.bin
```

### 2. 生成 JSON 模板

```bash
# SOP10 遥控器模板
python bin_gen.py --template --fw-type remote --output remote_template.json

# SOP10 遥控车模板
python bin_gen.py --template --fw-type car10 --output car10_template.json

# SOP16 遥控车模板
python bin_gen.py --template --fw-type car16 --output car16_template.json
```

### 3. 从 JSON 配置文件生成

```bash
# 使用自定义 JSON 配置
python bin_gen.py --target flash --fw-type car16 --json my_config.json --output config.bin
```

## JSON 配置示例

### SOP10 遥控器配置

```json
{
  "efuse_dev_info": {
    "chip_id": [18, 52, 86, 0, 0, 0],
    "paring_type": 0,
    "toy_type": 0,
    "access_code_group": 0,
    "flash_disable": 0,
    "normal_tx_power": 0,
    "ldo_output_trim": 0,
    "capless": 0,
    "bin_select": 0,
    "xtal_calibration": 0,
    "xtal_type": 0,
    "dig_ldo_mode": 0,
    "adc_calib": {
      "gpio_gain": 0,
      "gpio_offset": 0,
      "vbat_gain": 0,
      "vbat_offset": 0
    }
  },
  "cfg_param": {
    "pin0": 0,
    "pin1": 1,
    "pin2": 2,
    "pin3": 3,
    "pin4": 4,
    "shutdown_volt": 4,
    "low_batt_volt": 4,
    "sleep_timeout": 0,
    "led_polar": 0
  }
}
```

### SOP10 遥控车配置

```json
{
  "efuse_dev_info": {
    "chip_id": [18, 52, 86, 0, 0, 0],
    "toy_type": 1
  },
  "cfg_param": {
    "pin0": 0,
    "pin1": 1,
    "pin2": 2,
    "pin3": 3,
    "pin4": 4,
    "special_func": 1,
    "shutdown_volt": 8,
    "pwm_freq": 0,
    "rev_delay": 0,
    "spray_duration": 0,
    "spray_pwm_duty": 0,
    "paired_indicator": 0,
    "sleep_timeout": 4,
    "drive_mode": 0,
    "led_polar": 0
  }
}
```

### SOP16 遥控车配置

```json
{
  "efuse_dev_info": {
    "chip_id": [18, 52, 86, 0, 0, 0],
    "toy_type": 2
  },
  "cfg_param": {
    "pin0": 0,
    "pin1": 1,
    "pin2": 2,
    "pin3": 3,
    "pin4": 4,
    "pin5": 5,
    "pin6": 6,
    "pin7": 7,
    "pin8": 8,
    "pin9": 9,
    "pin10": 10,
    "shutdown_volt": 8,
    "pwm_freq": 0,
    "rev_delay": 0,
    "spray_duration": 0,
    "spray_pwm_duty": 0,
    "sleep_timeout": 4,
    "paired_indicator": 0,
    "demo_mode": 0,
    "drive_mode": 0,
    "spray_w_mask": 15,
    "special_func": 0,
    "led_polar": 0
  }
}
```

## 枚举值参考

### SOP10 遥控器

**Pin 定义:**
- 0: KEY_COM (公共按键)
- 1: KEY_F_B (前后按键)
- 2: KEY_L_R (左右按键)
- 3: KEY_K_S (功能按键)
- 4: KEY_P_LED (LED控制)

**关机电压:**
- 0: 默认
- 1: 2.0V
- 2: 2.1V
- 3: 2.2V
- 4: 不检测

**低电电压:**
- 0: 默认
- 1: 2.2V
- 2: 2.3V
- 3: 2.4V
- 4: 不检测

**休眠超时:**
- 0: 默认(10s)
- 1: 5秒
- 2: 10秒
- 3: 30秒
- 4: 60秒

### SOP10 遥控车

**Pin 定义:**
- 0: PWM_RF (右侧轮子向前)
- 1: PWM_RB (右侧轮子向后)
- 2: PWM_LF (左侧轮子向前)
- 3: PWM_LB (左侧轮子向后)
- 4: PWM_SPRAY_LED (喷雾/LED)

**特殊功能:**
- 0: 默认
- 1: 喷雾
- 2: LED

**关机电压:**
- 0: 默认
- 1-7: 2.6V-3.2V (0.1V步进)
- 8: 不检测

**PWM 频率:**
- 0: 默认(250Hz)
- 1: 250Hz
- 2: 500Hz
- 3: 16kHz

**正反转延迟:**
- 0: 默认(100ms)
- 1: 0ms
- 2: 50ms
- 3: 100ms
- 4: 150ms
- 5: 200ms
- 6: 250ms
- 7: 300ms

**喷雾时长:**
- 0: 默认(30s)
- 1: 30秒
- 2: 1分钟
- 3: 2分钟
- 4: 持续喷雾

**休眠超时:**
- 0: 默认(从不)
- 1: 5分钟
- 2: 10分钟
- 3: 15分钟
- 4: 从不休眠

### SOP16 遥控车

**Pin 定义:**
- 0: PWM_RF (右侧轮子向前)
- 1: PWM_RB (右侧轮子向后)
- 2: PWM_LF (左侧轮子向前)
- 3: PWM_LB (左侧轮子向后)
- 4: PWM_W1 (扩展电机1/挖掘机转盘)
- 5: PWM_W2 (扩展电机2/挖掘机挖臂)
- 6: PWM_W3 (扩展电机3)
- 7: PWM_W4 (扩展电机4)
- 8: GPIO_LED_OUT (GPIO LED)
- 9: CHG_DETECT (充电检测)
- 10: CHG_FULL (电池充满检测)

**特殊功能:**
- 0: 默认
- 1: 喷雾
- 2: 挖臂/转盘

**演示模式:**
- 0: 默认
- 1: 8字形
- 2: 圆形
- 3: 正方形

## Bin 文件布局

```
Flash/EFuse 地址布局:
+----------------------------------+
| efuse_dev_info_t (16 bytes)      |
|   - chip_id[6]                   |
|   - union_byte_0 ~ union_byte_5  |
|   - adc_calib_t (4 bytes)        |
+----------------------------------+
| CfgParamUnion (6/8/12 bytes)     |
|   - remote: 6 bytes              |
|   - car10: 8 bytes               |
|   - car16: 12 bytes              |
+----------------------------------+
| 填充至目标大小                   |
|   - Flash: 填充 0xFF             |
|   - EFuse: 填充 0x00             |
+----------------------------------+

烧录地址:
- Flash: 0xF000
- EFuse: 0x00
```

## 烧录说明

### 烧录到 Flash

Flash 烧录地址: **0xF000**

Flash 通常有较大的存储空间（如 1MB），bin 文件放在 0xF000 地址。
默认未写数据为 `0xFF`。

生成的文件名格式: `xxx_0x0f000.bin`

### 烧录到 EFuse

EFuse 烧录地址: **0x00**

TC122X 系列 EFuse 大小为 32 字节，地址范围 0x00-0x1F。
默认未写数据为 `0x00`。

生成的文件名格式: `xxx_0x00000.bin`

**注意:** EFuse 只能烧录一次，请确认配置正确后再烧录。
