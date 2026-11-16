# Bin 文件生成工具 - 测试版

测试版本，只包含关键配置参数。

## 与完整版的区别

### efuse_dev_info_t (简化)

测试版只支持 **3 个** 可配置参数：

| 参数 | 说明 | 默认值 |
|-----|------|-------|
| `toy_type` | 玩具类型: 0=遥控器, 1=SOP10车, 2=SOP16车 | 根据 fw-type 自动设置 |
| `access_code_group` | Access Code 组: 0-15 | 0 |
| `normal_tx_power` | 正常发射功率: 0-15 | 0 |

其他字段（chip_id, adc_calib 等）使用默认值。

### CfgParamUnion (完整)

CfgParamUnion 支持 **全部参数**，与完整版一致。

## 使用方法

### 1. 生成默认配置

```bash
# SOP10 遥控器 -> Flash (生成 config_0x0f000.bin)
python bin_gen_test.py --target flash --fw-type remote --output config.bin

# SOP10 遥控车 -> EFuse (生成 config_0x00000.bin)
python bin_gen_test.py --target efuse --fw-type car10 --output config.bin

# SOP16 遥控车 -> Flash (生成 config_0x0f000.bin)
python bin_gen_test.py --target flash --fw-type car16 --output config.bin
```

### 2. 生成 JSON 模板

```bash
# 生成 SOP10 遥控器模板
python bin_gen_test.py --template --fw-type remote --output template_remote.json

# 生成 SOP10 遥控车模板
python bin_gen_test.py --template --fw-type car10 --output template_car10.json

# 生成 SOP16 遥控车模板
python bin_gen_test.py --template --fw-type car16 --output template_car16.json
```

### 3. 从 JSON 配置文件生成

```bash
# 编辑模板后使用
python bin_gen_test.py --target flash --fw-type car16 --json my_config.json --output config.bin
```

## JSON 配置示例

### SOP10 遥控器

```json
{
  "efuse_dev_info": {
    "toy_type": 0,
    "access_code_group": 0,
    "normal_tx_power": 0
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

### SOP10 遥控车

```json
{
  "efuse_dev_info": {
    "toy_type": 1,
    "access_code_group": 0,
    "normal_tx_power": 0
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

### SOP16 遥控车

```json
{
  "efuse_dev_info": {
    "toy_type": 2,
    "access_code_group": 0,
    "normal_tx_power": 0
  },
  "cfg_param": {
    "pin0": 2,
    "pin1": 3,
    "pin2": 0,
    "pin3": 1,
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

与完整版一致，请参考完整版 README.md。

## 烧录地址

- **Flash**: 0xF000 → 生成文件名 `xxx_0x0f000.bin`
- **EFuse**: 0x00 → 生成文件名 `xxx_0x00000.bin`
