# S16 小车测试配置文档 - DEFAULT 模式

本目录包含 S16 小车 **default** 测试模式的配置文件。

## 测试模式说明

默认配置 - 所有参数使用默认值

## 配置统计

- default: 3 个
- **总计**: 3 个配置

## 管脚映射说明

SOP16 封装芯片引脚功能定义（参考 GPIO_PinTypeDef S16CarPin 和 enum SOP16CarPin）：

| 引脚索引 | 引脚编号 | enum SOP16CarPin | 四驱/差速模式 | 双电机模式 |
|----------|----------|------------------|---------------|------------|
| pin0 | PA0 | SOP16_CAR_PIN_PWM_RF | PWM驱动右侧轮子向前 | PWM驱动轮子向前 |
| pin1 | PA1 | SOP16_CAR_PIN_PWM_RB | PWM驱动右侧轮子向后 | PWM驱动轮子向后 |
| pin2 | PA2 | SOP16_CAR_PIN_PWM_LF | PWM驱动左侧轮子向前 | GPIO驱动轮子向左 |
| pin3 | PA3 | SOP16_CAR_PIN_PWM_LB | PWM驱动左侧轮子向后 | GPIO驱动轮子向右 |
| pin4 | PA4 | SOP16_CAR_PIN_PWM_W1 | PWM喷雾 | PWM喷雾/PWM转盘左转 |
| pin5 | PA5 | SOP16_CAR_PIN_PWM_W2 | PWM喷雾 | PWM喷雾/PWM挖掘机转盘  |
| pin6 | PA6 | SOP16_CAR_PIN_PWM_W3 | PWM喷雾 | PWM喷雾/PWM挖掘机挖臂 |
| pin7 | PA7 | SOP16_CAR_PIN_PWM_W4 | PWM喷雾 | PWM喷雾/PWM挖掘机挖臂 |
| pin8 | PB0 | SOP16_CAR_PIN_GPIO_LED_OUT | GPIO控制LED输出 |
| pin9 | PB1 | SOP16_CAR_PIN_CHG_DETECT | 充电检测 | 充电检测 |
| pin10 | PB2 | SOP16_CAR_PIN_CHG_FULL | 电池充满检测 | 电池充满检测 |

### 各模式管脚功能映射

#### TOY_S16_CAR_MODE_DIFF_SPRAY 模式 (差速底盘 + 喷雾 + LED + 充电)

**资源配置**: 4路PWM(前后左右) + 2路PWM(喷雾W1/W2) + LED + 充电(2个GPIO)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP16_CAR_PIN_PWM_LF | PWM Left Forward | PWM驱动左侧轮子向前 |
| pin1 | SOP16_CAR_PIN_PWM_RF | PWM Right Forward | PWM驱动右侧轮子向前 |
| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM Spray W1 | PWM喷雾泵W通道1控制 |
| pin3 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | 未使用 |
| pin4 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |
| pin5 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |
| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |
| pin7 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | 未使用 |
| pin8 | SOP16_CAR_PIN_PWM_LB | PWM Left Backward | PWM驱动左侧轮子向后 |
| pin9 | SOP16_CAR_PIN_PWM_RB | PWM Right Backward | PWM驱动右侧轮子向后 |
| pin10 | SOP16_CAR_PIN_PWM_W2 | PWM Spray W2 | PWM喷雾泵W通道2控制 |

#### TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR 模式 (双电机底盘 + 转盘/挖臂 + LED + 充电)

**资源配置**: 2路PWM(前后) + 2路GPIO(左右) + 4路PWM(转盘W1/W2 + 挖臂W3/W4) + LED + 充电(2个GPIO)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP16_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |
| pin1 | SOP16_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |
| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM W1 | PWM转盘左转控制 |
| pin3 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |
| pin4 | SOP16_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |
| pin5 | SOP16_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |
| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |
| pin7 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |
| pin8 | SOP16_CAR_PIN_PWM_W2 | PWM W2 | PWM转盘右转控制 |
| pin9 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | PWM挖臂向下控制 |
| pin10 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | PWM挖臂向上控制 |

#### TOY_S16_CAR_MODE_2MOTOR_SPRAY 模式 (双电机底盘 + 喷雾 + LED + 充电)

**资源配置**: 2路PWM(前后) + 2路GPIO(左右) + 1路PWM(喷雾W1) + LED + 充电(2个GPIO)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP16_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |
| pin1 | SOP16_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |
| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM Spray W1 | PWM喷雾泵W通道1控制 |
| pin3 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |
| pin4 | SOP16_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |
| pin5 | SOP16_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |
| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |
| pin7 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |
| pin8 | SOP16_CAR_PIN_PWM_W2 | PWM W2 | 扩展PWM W2 |
| pin9 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | 扩展PWM W3 |
| pin10 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | 扩展PWM W4 |

## 配置文件列表

### 2MOTOR_SPRAY

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S16_CAR_MODE_2MOTOR_SPRAY_DEFAULT_0x0f000.bin | default | pin0=PWM_RF, pin2=PWM_W1, pin3=GPIO_LED_OUT, pin4=PWM_RB, pin5=PWM_LB, pin6=CHG_DETECT, pin7=CHG_FULL, pin8=PWM_W2, pin9=PWM_W3, pin10=PWM_W4, pwm_freq=250Hz, sleep_timeout=never, paired_indicator=OFF, special_func=SPRAY, demo_mode=FIGURE8, drive_mode=2MOTOR | S16: 双电机 + 喷雾 - 默认配置（所有参数使用默认值） |

### DIFF_SPRAY

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S16_CAR_MODE_DIFF_SPRAY_DEFAULT_0x0f000.bin | default | pin0=PWM_RF, pin2=PWM_W1, pin3=PWM_W3, pin4=GPIO_LED_OUT, pin5=CHG_FULL, pin6=CHG_DETECT, pin8=PWM_RB, pin9=PWM_LB, pin10=PWM_W2, pwm_freq=250Hz, spray_w_mask=0x3, sleep_timeout=never, paired_indicator=OFF, special_func=SPRAY, demo_mode=FIGURE8 | S16: 差速 + 喷雾 - 默认配置（所有参数使用默认值） |

### EXCAVATOR

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_DEFAULT_0x0f000.bin | default | pin2=PWM_W1, pin3=GPIO_LED_OUT, pin4=PWM_RF, pin5=PWM_RB, pin6=CHG_DETECT, pin7=CHG_FULL, pin8=PWM_W4, pin9=PWM_W3, pin10=PWM_W2, pwm_freq=250Hz, spray_w_mask=0x0, sleep_timeout=never, paired_indicator=OFF, special_func=EXCAVATOR, demo_mode=FIGURE8, drive_mode=2MOTOR | S16: 挖掘机模式 - 默认配置（所有参数使用默认值） |

## 参数配置说明

### TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_RF | 引脚0功能配置 |
| pin1 | PWM_RB | 引脚1功能配置 |
| pin2 | PWM_W1 | 引脚2功能配置 |
| pin3 | GPIO_LED_OUT | 引脚3功能配置 |
| pin4 | PWM_LF | 引脚4功能配置 |
| pin5 | PWM_LB | 引脚5功能配置 |
| pin6 | CHG_DETECT | 引脚6功能配置 |
| pin7 | CHG_FULL | 引脚7功能配置 |
| pin8 | PWM_W4 | 引脚8功能配置 |
| pin9 | PWM_W3 | 引脚9功能配置 |
| pin10 | PWM_W2 | 引脚10功能配置 |
| special_func | EXCAVATOR | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| demo_mode | FIGURE8 | 演示模式 |
| drive_mode | 1 | 驱动模式 (0=正常, 1=差速) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| reserved0 | 0 | 保留位0 |
| reserved1 | 0 | 保留位1 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | DEFAULT (不检测) | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_duty_w | 100 | W通道PWM占空比 |
| pwm_freq | DEFAULT (250Hz) | PWM频率 |
| rev_delay | DEFAULT (100ms) | 正反转延迟 |
| spray_duration | DEFAULT (30s) | 喷雾时长 |
| spray_pwm_duty | DEFAULT (40%) | 喷雾PWM占空比 |
| sleep_timeout | DEFAULT (从不休眠) | 休眠超时 |
| spray_w_mask | All | 喷雾通道掩码 |

### TOY_S16_CAR_MODE_DIFF_SPRAY_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_LF | 引脚0功能配置 |
| pin1 | PWM_RF | 引脚1功能配置 |
| pin2 | PWM_W1 | 引脚2功能配置 |
| pin3 | PWM_W3 | 引脚3功能配置 |
| pin4 | GPIO_LED_OUT | 引脚4功能配置 |
| pin5 | CHG_FULL | 引脚5功能配置 |
| pin6 | CHG_DETECT | 引脚6功能配置 |
| pin7 | PWM_W4 | 引脚7功能配置 |
| pin8 | PWM_LB | 引脚8功能配置 |
| pin9 | PWM_RB | 引脚9功能配置 |
| pin10 | PWM_W2 | 引脚10功能配置 |
| special_func | SPRAY | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| demo_mode | FIGURE8 | 演示模式 |
| drive_mode | 0 | 驱动模式 (0=正常, 1=差速) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| reserved0 | 0 | 保留位0 |
| reserved1 | 0 | 保留位1 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | DEFAULT (不检测) | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_duty_w | 100 | W通道PWM占空比 |
| pwm_freq | DEFAULT (250Hz) | PWM频率 |
| rev_delay | DEFAULT (100ms) | 正反转延迟 |
| spray_duration | DEFAULT (30s) | 喷雾时长 |
| spray_pwm_duty | DEFAULT (40%) | 喷雾PWM占空比 |
| sleep_timeout | DEFAULT (从不休眠) | 休眠超时 |
| spray_w_mask | All | 喷雾通道掩码 |

### TOY_S16_CAR_MODE_2MOTOR_SPRAY_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_LF | 引脚0功能配置 |
| pin1 | PWM_RF | 引脚1功能配置 |
| pin2 | PWM_W1 | 引脚2功能配置 |
| pin3 | GPIO_LED_OUT | 引脚3功能配置 |
| pin4 | PWM_LB | 引脚4功能配置 |
| pin5 | PWM_RB | 引脚5功能配置 |
| pin6 | CHG_DETECT | 引脚6功能配置 |
| pin7 | CHG_FULL | 引脚7功能配置 |
| pin8 | PWM_W2 | 引脚8功能配置 |
| pin9 | PWM_W3 | 引脚9功能配置 |
| pin10 | PWM_W4 | 引脚10功能配置 |
| special_func | SPRAY | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| demo_mode | FIGURE8 | 演示模式 |
| drive_mode | 1 | 驱动模式 (0=正常, 1=差速) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| reserved0 | 0 | 保留位0 |
| reserved1 | 0 | 保留位1 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | DEFAULT (不检测) | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_duty_w | 100 | W通道PWM占空比 |
| pwm_freq | DEFAULT (250Hz) | PWM频率 |
| rev_delay | DEFAULT (100ms) | 正反转延迟 |
| spray_duration | DEFAULT (30s) | 喷雾时长 |
| spray_pwm_duty | DEFAULT (40%) | 喷雾PWM占空比 |
| sleep_timeout | DEFAULT (从不休眠) | 休眠超时 |
| spray_w_mask | All | 喷雾通道掩码 |

## 参数详细说明

### shutdown_volt - 关机电压

**默认值**: 0  **选项**:  0(DEFAULT (不检测)), 1(2.6V), 2(2.7V), 3(2.8V), 4(2.9V), 5(3.0V), 6(3.1V), 7(3.2V), 8(OFF)

### pwm_duty_left - 左轮PWM占空比

**默认值**: 0  **选项**:  0(100), 1(40), 2(50), 3(60), 4(70), 5(80), 6(90), 7(100)

### pwm_duty_right - 右轮PWM占空比

**默认值**: 0  **选项**:  0(100), 1(40), 2(50), 3(60), 4(70), 5(80), 6(90), 7(100)

### pwm_duty_w - W通道PWM占空比

**默认值**: 0  **选项**:  0(100), 1(40), 2(50), 3(60), 4(70), 5(80), 6(90), 7(100)

### pwm_freq - PWM频率

**默认值**: 0  **选项**:  0(DEFAULT (250Hz)), 1(250Hz), 2(500Hz), 3(16kHz)

### rev_delay - 正反转延迟

**默认值**: 0  **选项**:  0(DEFAULT (100ms)), 1(0ms), 2(50ms), 3(100ms), 4(150ms), 5(200ms), 6(250ms), 7(300ms)

### spray_duration - 喷雾时长

**默认值**: 0  **选项**:  0(DEFAULT (30s)), 1(30s), 2(1min), 3(2min), 4(always)

### spray_pwm_duty - 喷雾PWM占空比

**默认值**: 0  **选项**:  0(DEFAULT (40%)), 1(7), 2(10), 3(15), 4(20), 5(25), 6(30), 7(35), 8(40)

### sleep_timeout - 休眠超时

**默认值**: 0  **选项**:  0(DEFAULT (从不休眠)), 1(5min), 2(10min), 3(15min), 4(never)

### spray_w_mask - 喷雾通道掩码

**默认值**: 15  **选项**:  0(None), 1(W1), 2(W2), 4(W3), 8(W4), 3(W1_W2), 5(W1_W3), 10(W2_W4), 15(All)

## 烧录说明

- **烧录地址**: 0xF000 (Flash)
- **文件大小**: 256 bytes
- **未写数据**: 0xFF
- **固件类型**: CAR16 (SOP16 封装遥控车)

## 测试建议

1. **单参数测试**: 用于验证每个参数的独立功能
2. **双参数组合**: 用于验证参数间的兼容性
3. **场景测试**: 用于验证实际使用场景
4. **全参数组合**: 用于验证极端非默认配置
