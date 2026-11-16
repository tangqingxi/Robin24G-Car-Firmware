# S10 小车测试配置文档 - DEFAULT 模式

本目录包含 S10 小车 **default** 测试模式的配置文件。

## 测试模式说明

默认配置 - 所有参数使用默认值

## 配置统计

- default: 3 个
- **总计**: 3 个配置

## 管脚映射说明

SOP10 封装芯片引脚功能定义（参考 enum SOP10CarPin）：

| 引脚编号 | 引脚名称 | enum SOP10CarPin | 功能说明 |
|----------|----------|------------------|----------|
| PA0 | pin0 | SOP10_CAR_PIN_PWM_RF | 四驱/差速: PWM驱动右侧轮子向前 (Right Forward)<br>双电机: PWM驱动轮子向前 (Forward) |
| PA6 | pin1 | SOP10_CAR_PIN_PWM_RB | 四驱/差速: PWM驱动右侧轮子向后 (Right Backward)<br>双电机: PWM驱动轮子向后 (Backward) |
| PB0 | pin2 | SOP10_CAR_PIN_PWM_LF | 四驱/差速: PWM驱动左侧轮子向前 (Left Forward)<br>双电机: GPIO驱动轮子向左 (Left) |
| PA3 | pin3 | SOP10_CAR_PIN_PWM_LB | 四驱/差速: PWM驱动左侧轮子向后 (Left Backward)<br>双电机: GPIO驱动轮子向右 (Right) |
| PA5 | pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | PWM驱动喷雾 或 GPIO驱动LED |

### 各模式管脚功能映射

#### 2MOTOR_LED 模式 (双电机 + 2路PWM前后 + 2路GPIO左右 + LED)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |
| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |
| pin2 | SOP10_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |
| pin3 | SOP10_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |
| pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | LED | GPIO驱动LED |

#### 2MOTOR_SPRAY 模式 (双电机 + 2路PWM前后 + 2路GPIO左右 + 喷雾)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |
| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |
| pin2 | SOP10_CAR_PIN_PWM_SPRAY_LED | PWM Spray | PWM驱动喷雾泵 |
| pin3 | SOP10_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |
| pin4 | SOP10_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |

#### DIFF_4PWM_LED 模式 (四驱/差速 + 4路PWM + LED)

| 引脚 | enum值 | 功能 | 说明 |
|------|--------|------|------|
| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Right Forward | PWM驱动右侧轮子向前 |
| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Right Backward | PWM驱动右侧轮子向后 |
| pin2 | SOP10_CAR_PIN_PWM_LF | PWM Left Forward | PWM驱动左侧轮子向前 |
| pin3 | SOP10_CAR_PIN_PWM_LB | PWM Left Backward | PWM驱动左侧轮子向后 |
| pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | LED | GPIO驱动LED |

## 配置文件列表

### 2MOTOR_LED

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S10_CAR_MODE_2MOTOR_LED_DEFAULT_0x0f000.bin | default | pin3=PWM_SPRAY_LED, pin4=PWM_LB, pwm_freq=250Hz, rev_delay=100ms, spray_duration=30s, spray_pwm_duty=40%, sleep_timeout=never, paired_indicator=OFF | 双电机 + 2路PWM + LED - 默认配置（所有参数使用默认值） |

### 2MOTOR_SPRAY

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S10_CAR_MODE_2MOTOR_SPRAY_DEFAULT_0x0f000.bin | default | pin2=PWM_SPRAY_LED, pin3=PWM_LF, pin4=PWM_LB, pwm_freq=250Hz, rev_delay=100ms, spray_duration=30s, spray_pwm_duty=40%, sleep_timeout=never, paired_indicator=OFF | 双电机 + 2路PWM + 喷雾 - 默认配置（所有参数使用默认值） |

### DIFF_4PWM_LED

| 文件名 | 测试类型 | 参数设置 | 说明 |
|--------|----------|----------|------|
| TOY_S10_CAR_MODE_DIFF_4PWM_LED_DEFAULT_0x0f000.bin | default | pin3=PWM_SPRAY_LED, pin4=PWM_LB, pwm_freq=250Hz, rev_delay=100ms, spray_duration=30s, spray_pwm_duty=40%, sleep_timeout=never, paired_indicator=OFF | 四驱/差速 + 4路PWM + LED - 默认配置（所有参数使用默认值） |

## 参数配置说明

### TOY_S10_CAR_MODE_DIFF_4PWM_LED_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_RF | 引脚0功能配置 |
| pin1 | PWM_RB | 引脚1功能配置 |
| pin2 | PWM_LF | 引脚2功能配置 |
| pin3 | PWM_SPRAY_LED | 引脚3功能配置 |
| pin4 | PWM_LB | 引脚4功能配置 |
| special_func | LED | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| drive_mode | 0 | 驱动模式 (0=四驱/差速, 1=双电机) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | OFF | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_freq | 250Hz | PWM频率 |
| rev_delay | 100ms | 正反转延迟 |
| spray_duration | 30s | 喷雾时长 |
| spray_pwm_duty | 40 | 喷雾PWM占空比 |
| sleep_timeout | never | 休眠超时 |

### TOY_S10_CAR_MODE_2MOTOR_SPRAY_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_RF | 引脚0功能配置 |
| pin1 | PWM_RB | 引脚1功能配置 |
| pin2 | PWM_SPRAY_LED | 引脚2功能配置 |
| pin3 | PWM_LF | 引脚3功能配置 |
| pin4 | PWM_LB | 引脚4功能配置 |
| special_func | SPRAY | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| drive_mode | 1 | 驱动模式 (0=四驱/差速, 1=双电机) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | OFF | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_freq | 250Hz | PWM频率 |
| rev_delay | 100ms | 正反转延迟 |
| spray_duration | 30s | 喷雾时长 |
| spray_pwm_duty | 40 | 喷雾PWM占空比 |
| sleep_timeout | never | 休眠超时 |

### TOY_S10_CAR_MODE_2MOTOR_LED_DEFAULT

| 参数 | 值 | 说明 |
|------|-----|------|
| pin0 | PWM_RF | 引脚0功能配置 |
| pin1 | PWM_RB | 引脚1功能配置 |
| pin2 | PWM_LF | 引脚2功能配置 |
| pin3 | PWM_SPRAY_LED | 引脚3功能配置 |
| pin4 | PWM_LB | 引脚4功能配置 |
| special_func | LED | 特殊功能 |
| paired_indicator | OFF | 配对指示灯 |
| drive_mode | 1 | 驱动模式 (0=四驱/差速, 1=双电机) |
| led_polar | 0 | LED极性 (0=低电平亮, 1=高电平亮) |
| pwm_duty_dual_en | 0 | 双电机PWM独立使能 |
| access_code_group | 0 | 访问码组 |
| normal_tx_power | 0 | 正常发射功率 |
| shutdown_volt | OFF | 关机电压 |
| pwm_duty_left | 100 | 左轮PWM占空比 |
| pwm_duty_right | 100 | 右轮PWM占空比 |
| pwm_freq | 250Hz | PWM频率 |
| rev_delay | 100ms | 正反转延迟 |
| spray_duration | 30s | 喷雾时长 |
| spray_pwm_duty | 40 | 喷雾PWM占空比 |
| sleep_timeout | never | 休眠超时 |

## 可测试参数说明

### shutdown_volt - 关机电压
可选值: 0(DEFAULT (不检测)), 1(2.6V), 2(2.7V), 3(2.8V), 4(2.9V), 5(3.0V), 6(3.1V), 7(3.2V), 8(OFF)

### pwm_duty_left - 左轮PWM占空比
可选值: 0(100), 1(40), 2(50), 3(60), 4(70), 5(80), 6(90), 7(100)

### pwm_duty_right - 右轮PWM占空比
可选值: 0(100), 1(40), 2(50), 3(60), 4(70), 5(80), 6(90), 7(100)

### pwm_freq - PWM频率
可选值: 0(DEFAULT (250Hz)), 1(250Hz), 2(500Hz), 3(16kHz)

### rev_delay - 正反转延迟
可选值: 0(DEFAULT (100ms)), 1(0ms), 2(50ms), 3(100ms), 4(150ms), 5(200ms), 6(250ms), 7(300ms)

### spray_duration - 喷雾时长
可选值: 0(DEFAULT (30s)), 1(30s), 2(1min), 3(2min), 4(always)

### spray_pwm_duty - 喷雾PWM占空比
可选值: 0(DEFAULT (40%)), 1(7), 2(10), 3(15), 4(20), 5(25), 6(30), 7(35), 8(40)

### sleep_timeout - 休眠超时
可选值: 0(DEFAULT (从不休眠)), 1(5min), 2(10min), 3(15min), 4(never)

## 烧录说明

- **烧录地址**: 0xF000 (Flash)
- **烧录地址**: 0x00000 (EFuse)
- **文件大小**: 256 bytes (Flash), 32 bytes (EFuse)
- **未写数据**: 0xFF

