#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S16 小车参数组合测试配置生成工具 V2

功能说明:
    本工具用于生成 S16 小车（SOP16 封装）的测试配置文件，支持多种测试模式。
    可同时生成 Flash 版本（256字节，烧录地址 0xF000）和 EFuse 版本（32字节，烧录地址 0x00000）的 bin 文件。

支持多种测试模式：
    1. 单参数测试 (single)    - 每次只变化一个参数，其他保持默认值
    2. 双参数组合测试 (pair)  - 两个参数同时变化，测试参数间的影响
    3. 场景组合 (scenario)    - 预定义的实际使用场景配置
    4. 全参数组合 (full)      - 所有参数同时设置为非默认值
    5. 默认配置 (default)     - 生成三种工作模式的默认参数配置
    6. 全部模式 (all)         - 生成上述所有测试模式（默认）

覆盖的工作模式：
    - 2MOTOR_SPRAY  : 双电机 + 喷雾
    - 2MOTOR_EXCAVATOR: 双电机 + 挖掘机挖臂
    - DIFF_SPRAY    : 差速 + 喷雾

生成的文件：
    - Bin 文件: {MODE}_{TEST_TYPE}_{PARAM}_{VALUE}_0x0f000.bin  (Flash 版本)
               {MODE}_{TEST_TYPE}_{PARAM}_{VALUE}_0x00000.bin (EFuse 版本)
    - JSON 文件: 包含完整的参数配置信息
    - MD 文档  : README_{mode}.md 记录所有生成的配置文件

使用方法:
    python3 generate_s16_car_tests_v2.py --mode single    # 单参数测试
    python3 generate_s16_car_tests_v2.py --mode pair      # 双参数组合测试
    python3 generate_s16_car_tests_v2.py --mode scenario  # 场景组合测试
    python3 generate_s16_car_tests_v2.py --mode full      # 全参数组合测试
    python3 generate_s16_car_tests_v2.py --mode default   # 生成三种模式的默认配置
    python3 generate_s16_car_tests_v2.py --mode all       # 生成所有测试模式（默认）

高级选项:
    python3 generate_s16_car_tests_v2.py --mode default --output-dir ./output  # 指定输出目录
    python3 generate_s16_car_tests_v2.py --mode single --modes TOY_S16_CAR_MODE_2MOTOR_SPRAY  # 只生成指定模式

参数说明:
    --mode      测试模式: single/pair/scenario/full/default/all (默认: all)
    --output-dir/-o  输出目录 (默认: 当前目录)
    --modes     要生成的模式列表 (默认: 三种模式都生成)
"""

import json
import os
import sys
import argparse
import itertools
from typing import List, Dict, Tuple, Any

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from bin_gen_test import (
    ConfigGenerator, TargetType, FwType,
    SOP16CarPin, SOP16CarSpecialFunc, SOP16CarShutdownVoltage,
    SOP16CarPwmFreq, SOP16CarPwmDutyCycle, SOP16CarDirSwitchDelay,
    SOP16CarSprayDuration, SOP16CarSprayPwmDuty, SOP16CarPairedIndicator,
    SOP16CarSleepTimeout, SOP16CarDemoMode
)


# =============================================================================
# 参数定义 - 扩展所有参数的可选项，用于测试非默认参数
# =============================================================================

PARAM_DEFINITIONS = {
    "shutdown_volt": {
        "name": "关机电压",
        "default": SOP16CarShutdownVoltage.DEFAULT,
        "options": [
            (SOP16CarShutdownVoltage.DEFAULT, "DEFAULT (不检测)"),
            (SOP16CarShutdownVoltage.VOLT_2_6V, "2.6V"),
            (SOP16CarShutdownVoltage.VOLT_2_7V, "2.7V"),
            (SOP16CarShutdownVoltage.VOLT_2_8V, "2.8V"),
            (SOP16CarShutdownVoltage.VOLT_2_9V, "2.9V"),
            (SOP16CarShutdownVoltage.VOLT_3_0V, "3.0V"),
            (SOP16CarShutdownVoltage.VOLT_3_1V, "3.1V"),
            (SOP16CarShutdownVoltage.VOLT_3_2V, "3.2V"),
            (SOP16CarShutdownVoltage.OFF, "OFF"),
        ]
    },
    "pwm_duty_left": {
        "name": "左轮PWM占空比",
        "default": SOP16CarPwmDutyCycle.DEFAULT,
        "options": [
            (SOP16CarPwmDutyCycle.DEFAULT, "100"),
            (SOP16CarPwmDutyCycle.DUTY_40_PERCENT, "40"),
            (SOP16CarPwmDutyCycle.DUTY_50_PERCENT, "50"),
            (SOP16CarPwmDutyCycle.DUTY_60_PERCENT, "60"),
            (SOP16CarPwmDutyCycle.DUTY_70_PERCENT, "70"),
            (SOP16CarPwmDutyCycle.DUTY_80_PERCENT, "80"),
            (SOP16CarPwmDutyCycle.DUTY_90_PERCENT, "90"),
            (SOP16CarPwmDutyCycle.DUTY_100_PERCENT, "100"),
        ]
    },
    "pwm_duty_right": {
        "name": "右轮PWM占空比",
        "default": SOP16CarPwmDutyCycle.DEFAULT,
        "options": [
            (SOP16CarPwmDutyCycle.DEFAULT, "100"),
            (SOP16CarPwmDutyCycle.DUTY_40_PERCENT, "40"),
            (SOP16CarPwmDutyCycle.DUTY_50_PERCENT, "50"),
            (SOP16CarPwmDutyCycle.DUTY_60_PERCENT, "60"),
            (SOP16CarPwmDutyCycle.DUTY_70_PERCENT, "70"),
            (SOP16CarPwmDutyCycle.DUTY_80_PERCENT, "80"),
            (SOP16CarPwmDutyCycle.DUTY_90_PERCENT, "90"),
            (SOP16CarPwmDutyCycle.DUTY_100_PERCENT, "100"),
        ]
    },
    "pwm_duty_w": {
        "name": "W通道PWM占空比",
        "default": SOP16CarPwmDutyCycle.DEFAULT,
        "options": [
            (SOP16CarPwmDutyCycle.DEFAULT, "100"),
            (SOP16CarPwmDutyCycle.DUTY_40_PERCENT, "40"),
            (SOP16CarPwmDutyCycle.DUTY_50_PERCENT, "50"),
            (SOP16CarPwmDutyCycle.DUTY_60_PERCENT, "60"),
            (SOP16CarPwmDutyCycle.DUTY_70_PERCENT, "70"),
            (SOP16CarPwmDutyCycle.DUTY_80_PERCENT, "80"),
            (SOP16CarPwmDutyCycle.DUTY_90_PERCENT, "90"),
            (SOP16CarPwmDutyCycle.DUTY_100_PERCENT, "100"),
        ]
    },
    "pwm_freq": {
        "name": "PWM频率",
        "default": SOP16CarPwmFreq.DEFAULT,
        "options": [
            (SOP16CarPwmFreq.DEFAULT, "DEFAULT (250Hz)"),
            (SOP16CarPwmFreq.FREQ_250HZ, "250Hz"),
            (SOP16CarPwmFreq.FREQ_500HZ, "500Hz"),
            (SOP16CarPwmFreq.FREQ_16KHZ, "16kHz"),
        ]
    },
    "rev_delay": {
        "name": "正反转延迟",
        "default": SOP16CarDirSwitchDelay.DEFAULT,
        "options": [
            (SOP16CarDirSwitchDelay.DEFAULT, "DEFAULT (100ms)"),
            (SOP16CarDirSwitchDelay.DELAY_0MS, "0ms"),
            (SOP16CarDirSwitchDelay.DELAY_50MS, "50ms"),
            (SOP16CarDirSwitchDelay.DELAY_100MS, "100ms"),
            (SOP16CarDirSwitchDelay.DELAY_150MS, "150ms"),
            (SOP16CarDirSwitchDelay.DELAY_200MS, "200ms"),
            (SOP16CarDirSwitchDelay.DELAY_250MS, "250ms"),
            (SOP16CarDirSwitchDelay.DELAY_300MS, "300ms"),
        ]
    },
    "spray_duration": {
        "name": "喷雾时长",
        "default": SOP16CarSprayDuration.DEFAULT,
        "options": [
            (SOP16CarSprayDuration.DEFAULT, "DEFAULT (30s)"),
            (SOP16CarSprayDuration.DURATION_30S, "30s"),
            (SOP16CarSprayDuration.DURATION_1MIN, "1min"),
            (SOP16CarSprayDuration.DURATION_2MIN, "2min"),
            (SOP16CarSprayDuration.ALWAYS, "always"),
        ]
    },
    "spray_pwm_duty": {
        "name": "喷雾PWM占空比",
        "default": SOP16CarSprayPwmDuty.DEFAULT,
        "options": [
            (SOP16CarSprayPwmDuty.DEFAULT, "DEFAULT (40%)"),
            (SOP16CarSprayPwmDuty.DUTY_7_PERCENT, "7"),
            (SOP16CarSprayPwmDuty.DUTY_10_PERCENT, "10"),
            (SOP16CarSprayPwmDuty.DUTY_15_PERCENT, "15"),
            (SOP16CarSprayPwmDuty.DUTY_20_PERCENT, "20"),
            (SOP16CarSprayPwmDuty.DUTY_25_PERCENT, "25"),
            (SOP16CarSprayPwmDuty.DUTY_30_PERCENT, "30"),
            (SOP16CarSprayPwmDuty.DUTY_35_PERCENT, "35"),
            (SOP16CarSprayPwmDuty.DUTY_40_PERCENT, "40"),
        ]
    },
    "sleep_timeout": {
        "name": "休眠超时",
        "default": SOP16CarSleepTimeout.DEFAULT,
        "options": [
            (SOP16CarSleepTimeout.DEFAULT, "DEFAULT (从不休眠)"),
            (SOP16CarSleepTimeout.TIMEOUT_5MIN, "5min"),
            (SOP16CarSleepTimeout.TIMEOUT_10MIN, "10min"),
            (SOP16CarSleepTimeout.TIMEOUT_15MIN, "15min"),
            (SOP16CarSleepTimeout.NEVER, "never"),
        ]
    },
    "spray_w_mask": {
        "name": "喷雾通道掩码",
        "default": 0x0F,  # All W channels
        "options": [
            (0x00, "None"),
            (0x01, "W1"),
            (0x02, "W2"),
            (0x04, "W3"),
            (0x08, "W4"),
            (0x03, "W1_W2"),
            (0x05, "W1_W3"),
            (0x0A, "W2_W4"),
            (0x0F, "All"),
        ]
    },
}


# =============================================================================
# 场景定义
# =============================================================================

SCENARIOS = {
    "all_open_paired_indicatorled_always_on": {
        "name": "全开模式1",
        "description": "所有参数都非默认值,配网后LED常亮",
        "params": {
            "shutdown_volt": SOP16CarShutdownVoltage.VOLT_2_6V,
            "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_50_PERCENT,
            "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_70_PERCENT,
            "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_90_PERCENT,
            "pwm_freq": SOP16CarPwmFreq.FREQ_500HZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "sleep_timeout": SOP16CarSleepTimeout.TIMEOUT_5MIN,
            "spray_w_mask": 0x03,
            "paired_indicator": SOP16CarPairedIndicator.ALWAYS,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_open_paired_indicatorled_off": {
        "name": "全开模式2",
        "description": "所有参数都非默认值，配网后LED常灭",
        "params": {
            "shutdown_volt": SOP16CarShutdownVoltage.VOLT_2_6V,
            "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_40_PERCENT,
            "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_70_PERCENT,
            "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_80_PERCENT,
            "pwm_freq": SOP16CarPwmFreq.FREQ_16KHZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_300MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_20_PERCENT,
            "sleep_timeout": SOP16CarSleepTimeout.TIMEOUT_5MIN,
            "spray_w_mask": 0x03,
            "paired_indicator": SOP16CarPairedIndicator.OFF,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_min_values": {
        "name": "全部最小值模式",
        "description": "所有参数配置为最小值",
        "params": {
            "shutdown_volt": SOP16CarShutdownVoltage.DEFAULT,
            "pwm_duty_left": SOP16CarPwmDutyCycle.DEFAULT,
            "pwm_duty_right": SOP16CarPwmDutyCycle.DEFAULT,
            "pwm_duty_w": SOP16CarPwmDutyCycle.DEFAULT,
            "pwm_freq": SOP16CarPwmFreq.DEFAULT,
            "rev_delay": SOP16CarDirSwitchDelay.DEFAULT,
            "spray_duration": SOP16CarSprayDuration.DEFAULT,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DEFAULT,
            "sleep_timeout": SOP16CarSleepTimeout.DEFAULT,
            "spray_w_mask": 0x00,
            "paired_indicator": SOP16CarPairedIndicator.DEFAULT,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_max_values": {
        "name": "全部最大值模式",
        "description": "所有参数配置为最大值",
        "params": {
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
            "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
            "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
            "pwm_freq": SOP16CarPwmFreq.FREQ_16KHZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_300MS,
            "spray_duration": SOP16CarSprayDuration.ALWAYS,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "spray_w_mask": 0x0F,
            "paired_indicator": SOP16CarPairedIndicator.ALWAYS,
            "access_code_group": 15,
            "normal_tx_power": 15,
            "led_polar": 1,
        }
    },
}


# =============================================================================
# 辅助函数
# =============================================================================

def set_config_param(config: Dict, param_name: str, value: Any):
    """设置配置参数，并处理依赖关系"""
    config["cfg_param"][param_name] = value
    # 如果设置了左右轮或W通道占空比，强制开启独立控制，确保参数生效
    if param_name in ["pwm_duty_left", "pwm_duty_right", "pwm_duty_w"]:
        config["cfg_param"]["pwm_duty_dual_en"] = 1


def validate_config(config: Dict) -> bool:
    """验证配置参数组合的有效性"""
    cfg = config.get("cfg_param", {})
    drive_mode = cfg.get("drive_mode")
    special_func = cfg.get("special_func")
    
    # 规则: 挖掘机功能 (EXCAVATOR) 必须配合双电机模式 (drive_mode=1)
    if special_func == SOP16CarSpecialFunc.EXCAVATOR and drive_mode != 1:
        return False
    return True


# =============================================================================
# 基础配置
# =============================================================================

def get_base_config(mode_name: str) -> Dict:
    """获取基础配置模板"""
    base = {
        "name": mode_name,
        "efuse_dev_info": {
            "toy_type": 3
        },
        "cfg_param": {
            # 默认引脚配置
            "pin0": SOP16CarPin.PWM_LF,
            "pin1": SOP16CarPin.PWM_LB,
            "pin2": SOP16CarPin.PWM_RF,
            "pin3": SOP16CarPin.PWM_RB,
            "pin4": SOP16CarPin.PWM_W1,
            "pin5": SOP16CarPin.PWM_W2,
            "pin6": SOP16CarPin.PWM_W3,
            "pin7": SOP16CarPin.PWM_W4,
            "pin8": SOP16CarPin.GPIO_LED_OUT,
            "pin9": SOP16CarPin.CHG_DETECT,
            "pin10": SOP16CarPin.CHG_FULL,
            
            # 默认参数值
            "demo_mode": SOP16CarDemoMode.FIGURE8,
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0b111,  # 100%
            "pwm_duty_right": 0b111,  # 100%
            "pwm_duty_w": 0b111,  # 100%
            "pwm_freq": SOP16CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP16CarPairedIndicator.OFF,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "drive_mode": 0,
            "spray_w_mask": 0x0F,
            "special_func": SOP16CarSpecialFunc.DEFAULT,
            "led_polar": 0,
            # 移动到尾部的字段（原先在 efuse_dev_info_t）
            "access_code_group": 0,
            "normal_tx_power": 0,
        }
    }
    
    # 模式1: 四驱/差速 + 喷雾 (TOY_S16_CAR_MODE_DIFF_SPRAY)
    if "DIFF_SPRAY" in mode_name:
        base["description"] = "S16: 差速 + 喷雾"
        base["mode_label"] = "DIFF_SPRAY"
        base["cfg_param"].update({
            "drive_mode": 0,
            "special_func": SOP16CarSpecialFunc.SPRAY,
            "pin0": SOP16CarPin.PWM_LF,
            "pin1": SOP16CarPin.PWM_RF,
            "pin2": SOP16CarPin.PWM_W1,
            "pin3": SOP16CarPin.PWM_W3,
            "pin4": SOP16CarPin.GPIO_LED_OUT,
            "pin5": SOP16CarPin.CHG_FULL,
            "pin6": SOP16CarPin.CHG_DETECT,
            "pin7": SOP16CarPin.PWM_W4,
            "pin8": SOP16CarPin.PWM_LB,
            "pin9": SOP16CarPin.PWM_RB,
            "pin10": SOP16CarPin.PWM_W2,
            "spray_w_mask": 0x03,
        })
    
    # 模式2: 双电机 + 挖臂/转盘 (TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR)
    elif "2MOTOR_EXCAVATOR" in mode_name:
        base["description"] = "S16: 挖掘机模式"
        base["mode_label"] = "EXCAVATOR"
        base["cfg_param"].update({
            "drive_mode": 1,
            "special_func": SOP16CarSpecialFunc.EXCAVATOR,
            "spray_w_mask": 0x00,
            "pin0": SOP16CarPin.PWM_RF,
            "pin1": SOP16CarPin.PWM_RB,
            "pin2": SOP16CarPin.PWM_W1,
            "pin3": SOP16CarPin.GPIO_LED_OUT,
            "pin4": SOP16CarPin.PWM_LF,
            "pin5": SOP16CarPin.PWM_LB,
            "pin6": SOP16CarPin.CHG_DETECT,
            "pin7": SOP16CarPin.CHG_FULL,
            "pin8": SOP16CarPin.PWM_W4,
            "pin9": SOP16CarPin.PWM_W3,
            "pin10": SOP16CarPin.PWM_W2,
        })
    
    # 模式3: 双电机 + 喷雾 (TOY_S16_CAR_MODE_2MOTOR_SPRAY)
    elif "2MOTOR_SPRAY" in mode_name:
        base["description"] = "S16: 双电机 + 喷雾"
        base["mode_label"] = "2MOTOR_SPRAY"
        base["cfg_param"].update({
            "drive_mode": 1,
            "special_func": SOP16CarSpecialFunc.SPRAY,
            "pin0": SOP16CarPin.PWM_LF,
            "pin1": SOP16CarPin.PWM_RF,
            "pin2": SOP16CarPin.PWM_W1,
            "pin3": SOP16CarPin.GPIO_LED_OUT,
            "pin4": SOP16CarPin.PWM_LB,
            "pin5": SOP16CarPin.PWM_RB,
            "pin6": SOP16CarPin.CHG_DETECT,
            "pin7": SOP16CarPin.CHG_FULL,
            "pin8": SOP16CarPin.PWM_W2,
            "pin9": SOP16CarPin.PWM_W3,
            "pin10": SOP16CarPin.PWM_W4,
            "spray_w_mask": 0x0F,
        })
    
    else:
        base["description"] = "S16: 未知模式"
        base["mode_label"] = "UNKNOWN"
    
    return base


# =============================================================================
# 配置生成器
# =============================================================================

def generate_single_param_tests(mode: str) -> List[Dict]:
    """生成单参数测试配置 - 遍历所有参数的所有非默认值"""
    configs = []
    base = get_base_config(mode)
    
    for param_name, param_def in PARAM_DEFINITIONS.items():
        # 挖掘机模式过滤喷雾参数
        if "EXCAVATOR" in base["mode_label"] and "spray" in param_name:
            continue
        
        for value, desc in param_def["options"]:
            # 跳过默认值，只测试非默认值
            if value == param_def["default"]:
                continue
                
            safe_desc = sanitize_filename(desc)
            config = {
                "name": f"{mode}_SINGLE_{param_name.upper()}_{safe_desc}",
                "description": f"{base['description']} - 单参数测试: {param_def['name']}={desc}",
                "efuse_dev_info": base["efuse_dev_info"].copy(),
                "cfg_param": base["cfg_param"].copy(),
                "test_type": "single",
                "mode_label": base["mode_label"],
                "test_params": {param_name: {"value": value, "desc": desc, "name": param_def["name"]}}
            }
            set_config_param(config, param_name, value)
            if validate_config(config):
                configs.append(config)
    
    return configs


def generate_pair_tests(mode: str, selected_params: List[str] = None) -> List[Dict]:
    """生成双参数组合测试配置"""
    configs = []
    base = get_base_config(mode)
    
    # 获取要组合的参数列表
    if selected_params:
        params = [(p, PARAM_DEFINITIONS[p]) for p in selected_params if p in PARAM_DEFINITIONS]
    else:
        params = list(PARAM_DEFINITIONS.items())
    
    # 挖掘机模式过滤
    if "EXCAVATOR" in base["mode_label"]:
        params = [(p, d) for p, d in params if "spray" not in p]
    
    # 生成参数对
    param_pairs = list(itertools.combinations(params, 2))
    
    seen_configs = set()
    
    for (name1, def1), (name2, def2) in param_pairs:
        # 为每对参数生成关键组合
        # 策略: 最小+最小, 最小+最大, 最大+最小, 中间+中间
        opts1 = def1["options"]
        opts2 = def2["options"]
        
        # 过滤掉默认值的选项
        opts1_non_default = [(v, d) for v, d in opts1 if v != def1["default"]]
        opts2_non_default = [(v, d) for v, d in opts2 if v != def2["default"]]
        
        if not opts1_non_default or not opts2_non_default:
            continue
        
        key_combinations = [
            (0, 0),  # 最小 + 最小
            (0, -1),  # 最小 + 最大
            (-1, 0),  # 最大 + 最小
            (len(opts1_non_default)//2, len(opts2_non_default)//2),  # 中间 + 中间
        ]
        
        for idx, (i, j) in enumerate(key_combinations):
            val1, desc1_raw = opts1_non_default[i][0], opts1_non_default[i][1]
            val2, desc2_raw = opts2_non_default[j][0], opts2_non_default[j][1]
            desc1 = sanitize_filename(desc1_raw)
            desc2 = sanitize_filename(desc2_raw)
            
            config = {
                "name": f"{mode}_PAIR_{name1.upper()}_{desc1}_{name2.upper()}_{desc2}",
                "description": f"{base['description']} - 双参数组合: {def1['name']}={desc1_raw}, {def2['name']}={desc2_raw}",
                "efuse_dev_info": base["efuse_dev_info"].copy(),
                "cfg_param": base["cfg_param"].copy(),
                "test_type": "pair",
                "mode_label": base["mode_label"],
                "test_params": {
                    name1: {"value": val1, "desc": desc1_raw, "name": def1["name"]},
                    name2: {"value": val2, "desc": desc2_raw, "name": def2["name"]}
                }
            }
            
            if config["name"] in seen_configs:
                continue
            seen_configs.add(config["name"])
            
            set_config_param(config, name1, val1)
            set_config_param(config, name2, val2)
            if validate_config(config):
                configs.append(config)
    
    return configs


def generate_scenario_tests(mode: str) -> List[Dict]:
    """生成场景测试配置"""
    configs = []
    base = get_base_config(mode)
    
    for scenario_id, scenario in SCENARIOS.items():
        # 过滤规则
        is_excavator_mode = "EXCAVATOR" in base["mode_label"]
        is_spray_mode = "SPRAY" in base["mode_label"]
        
        # 挖掘机专属场景
        if not is_excavator_mode and scenario_id in ["excavator_ops"]:
            continue
        # 喷雾专属场景
        if not is_spray_mode and scenario_id in ["spray_max", "spray_min"]:
            continue
        
        config = {
            "name": f"{mode}_SCENE_{scenario_id.upper()}",
            "description": f"{base['description']} - 场景: {scenario['name']} ({scenario['description']})",
            "efuse_dev_info": base["efuse_dev_info"].copy(),
            "cfg_param": base["cfg_param"].copy(),
            "test_type": "scenario",
            "mode_label": base["mode_label"],
            "test_params": {}
        }
        
        # 应用场景参数
        for param_name, value in scenario["params"].items():
            # 参数级过滤
            if "EXCAVATOR" in base["mode_label"] and "spray" in param_name:
                continue
            
            set_config_param(config, param_name, value)
            
            # 记录用于文档
            if param_name in PARAM_DEFINITIONS:
                for v, d in PARAM_DEFINITIONS[param_name]["options"]:
                    if v == value:
                        config["test_params"][param_name] = {
                            "value": value,
                            "desc": d,
                            "name": PARAM_DEFINITIONS[param_name]["name"]
                        }
                        break
        
        if validate_config(config):
            configs.append(config)
    
    return configs


def generate_default_config(mode: str) -> List[Dict]:
    """生成默认配置（所有参数使用默认值）"""
    configs = []
    base = get_base_config(mode)
    
    config = {
        "name": f"{mode}_DEFAULT",
        "description": f"{base['description']} - 默认配置（所有参数使用默认值）",
        "efuse_dev_info": base["efuse_dev_info"].copy(),
        "cfg_param": base["cfg_param"].copy(),
        "test_type": "default",
        "mode_label": base["mode_label"],
        "test_params": {}
    }
    
    configs.append(config)
    return configs


def generate_full_combination_tests(mode: str) -> List[Dict]:
    """生成全参数组合测试 - 选择关键参数进行组合"""
    configs = []
    base = get_base_config(mode)
    
    # 定义关键参数组合 - 测试所有参数同时非默认的情况
    full_combos = [
        {
            "name": "ALL_NON_DEFAULT_1",
            "desc": "所有参数非默认组合1",
            "params": {
                "shutdown_volt": SOP16CarShutdownVoltage.VOLT_2_6V,
                "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_50_PERCENT,
                "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_70_PERCENT,
                "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_90_PERCENT,
                "pwm_freq": SOP16CarPwmFreq.FREQ_500HZ,
                "rev_delay": SOP16CarDirSwitchDelay.DELAY_0MS,
                "spray_duration": SOP16CarSprayDuration.DURATION_1MIN,
                "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_20_PERCENT,
                "sleep_timeout": SOP16CarSleepTimeout.TIMEOUT_5MIN,
                "spray_w_mask": 0x03,
            }
        },
        {
            "name": "ALL_NON_DEFAULT_2",
            "desc": "所有参数非默认组合2",
            "params": {
                "shutdown_volt": SOP16CarShutdownVoltage.VOLT_3_2V,
                "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_40_PERCENT,
                "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
                "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_80_PERCENT,
                "pwm_freq": SOP16CarPwmFreq.FREQ_16KHZ,
                "rev_delay": SOP16CarDirSwitchDelay.DELAY_300MS,
                "spray_duration": SOP16CarSprayDuration.ALWAYS,
                "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_7_PERCENT,
                "sleep_timeout": SOP16CarSleepTimeout.TIMEOUT_15MIN,
                "spray_w_mask": 0x0F,
            }
        },
        {
            "name": "ALL_NON_DEFAULT_3",
            "desc": "所有参数非默认组合3",
            "params": {
                "shutdown_volt": SOP16CarShutdownVoltage.OFF,
                "pwm_duty_left": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
                "pwm_duty_right": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
                "pwm_duty_w": SOP16CarPwmDutyCycle.DUTY_100_PERCENT,
                "pwm_freq": SOP16CarPwmFreq.FREQ_16KHZ,
                "rev_delay": SOP16CarDirSwitchDelay.DELAY_0MS,
                "spray_duration": SOP16CarSprayDuration.DURATION_2MIN,
                "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_35_PERCENT,
                "sleep_timeout": SOP16CarSleepTimeout.NEVER,
                "spray_w_mask": 0x0A,
            }
        },
    ]
    
    for combo in full_combos:
        config = {
            "name": f"{mode}_FULL_{combo['name']}",
            "description": f"{base['description']} - 全参数组合: {combo['desc']}",
            "efuse_dev_info": base["efuse_dev_info"].copy(),
            "cfg_param": base["cfg_param"].copy(),
            "test_type": "full",
            "mode_label": base["mode_label"],
            "test_params": {}
        }
        
        for param_name, value in combo["params"].items():
            # 过滤挖掘机模式的喷雾参数
            if "EXCAVATOR" in base["mode_label"] and "spray" in param_name:
                continue
            
            set_config_param(config, param_name, value)
            
            # 记录用于文档
            if param_name in PARAM_DEFINITIONS:
                for v, d in PARAM_DEFINITIONS[param_name]["options"]:
                    if v == value:
                        config["test_params"][param_name] = {
                            "value": value,
                            "desc": d,
                            "name": PARAM_DEFINITIONS[param_name]["name"]
                        }
                        break
        
        if validate_config(config):
            configs.append(config)
    
    return configs


# =============================================================================
# 文件生成
# =============================================================================

def sanitize_filename(name: str) -> str:
    """清理文件名，替换特殊字符"""
    return name.replace(".", "_").replace("%", "pct")


def convert_enums(obj):
    """转换枚举为值"""
    from enum import IntEnum
    if isinstance(obj, IntEnum):
        return obj.value
    elif isinstance(obj, list):
        return [convert_enums(i) for i in obj]
    elif isinstance(obj, dict):
        return {k: convert_enums(v) for k, v in obj.items()}
    return obj


def generate_files(configs: List[Dict], output_dir: str, doc_entries: List[Dict]):
    """生成 bin 和 json 文件 (同时生成 Flash 和 EFuse 版本，使用相同的 chip_id)"""
    
    for config in configs:
        config_name = config["name"]
        generated_files = []
        chip_id = None
        
        # 先生成 Flash 版本
        generator_flash = ConfigGenerator(TargetType.FLASH, FwType.CAR16)
        generator_flash.set_efuse_dev_info(**config["efuse_dev_info"])
        generator_flash.set_cfg_param(**config["cfg_param"])
        flash_bin_file = generator_flash.generate(config_name)
        flash_final_path = os.path.join(output_dir, os.path.basename(flash_bin_file))
        if os.path.abspath(flash_bin_file) != os.path.abspath(flash_final_path):
            if os.path.exists(flash_bin_file):
                if os.path.exists(flash_final_path):
                    os.remove(flash_final_path)
                os.rename(flash_bin_file, flash_final_path)
        
        generated_files.append(os.path.basename(flash_final_path))
        
        # 读取 chip_id
        with open(flash_final_path, 'rb') as f:
            bin_data = f.read()
            chip_id = list(bin_data[0:6])
        
        # 生成 EFuse 版本（使用相同的 chip_id）
        efuse_dev_info_with_chipid = config["efuse_dev_info"].copy()
        efuse_dev_info_with_chipid["chip_id"] = chip_id
        
        generator_efuse = ConfigGenerator(TargetType.EFUSE, FwType.CAR16)
        generator_efuse.set_efuse_dev_info(**efuse_dev_info_with_chipid)
        generator_efuse.set_cfg_param(**config["cfg_param"])
        efuse_bin_file = generator_efuse.generate(config_name)
        efuse_final_path = os.path.join(output_dir, os.path.basename(efuse_bin_file))
        if os.path.abspath(efuse_bin_file) != os.path.abspath(efuse_final_path):
            if os.path.exists(efuse_bin_file):
                if os.path.exists(efuse_final_path):
                    os.remove(efuse_final_path)
                os.rename(efuse_bin_file, efuse_final_path)
        
        generated_files.append(os.path.basename(efuse_final_path))
        
        # 保存 JSON (只保存一份，包含 Flash 和 EFuse 两个文件名)
        json_data = convert_enums(config)
        json_data["efuse_dev_info"]["chip_id"] = chip_id
        json_data["efuse_dev_info"]["chip_id_hex"] = "".join(f"{b:02x}" for b in chip_id)
        json_data["bin_files"] = {
            "flash": generated_files[0],
            "efuse": generated_files[1]
        }
        
        json_file = os.path.join(output_dir, f"{config_name}.json")
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)
        
        # 记录文档 (使用 Flash 版本作为代表)
        doc_entries.append({
            "filename": generated_files[0],
            "efuse_file": generated_files[1],
            "mode": config.get("mode_label", "UNKNOWN"),
            "type": config["test_type"],
            "params": config.get("test_params", {}),
            "description": config["description"],
            "cfg_param": config.get("cfg_param", {})
        })
        
        print(f"  Generated: {generated_files[0]} (Flash) + {generated_files[1]} (EFuse)")


def generate_documentation(doc_entries: List[Dict], output_file: str, mode: str):
    """生成文档"""
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(f"# S16 小车测试配置文档 - {mode.upper()} 模式\n\n")
        f.write(f"本目录包含 S16 小车 **{mode}** 测试模式的配置文件。\n\n")
        
        # 测试模式说明
        mode_descriptions = {
            "single": "单参数测试 - 每次只变化一个参数，其他保持默认值",
            "pair": "双参数组合测试 - 两个参数同时变化，测试参数间的影响",
            "scenario": "场景测试 - 预定义的实际使用场景配置",
            "full": "全参数组合测试 - 所有参数同时设置为非默认值",
            "default": "默认配置 - 所有参数使用默认值",
            "all": "全部测试模式 - 包含上述所有测试类型"
        }
        f.write(f"## 测试模式说明\n\n")
        f.write(f"{mode_descriptions.get(mode, '自定义测试')}\n\n")
        
        # 统计
        type_counts = {}
        for entry in doc_entries:
            t = entry["type"]
            type_counts[t] = type_counts.get(t, 0) + 1
        
        f.write("## 配置统计\n\n")
        for t, count in sorted(type_counts.items()):
            f.write(f"- {t}: {count} 个\n")
        f.write(f"- **总计**: {len(doc_entries)} 个配置\n\n")
        
        # 管脚映射说明
        f.write("## 管脚映射说明\n\n")
        f.write("SOP16 封装芯片引脚功能定义（参考 GPIO_PinTypeDef S16CarPin 和 enum SOP16CarPin）：\n\n")
        f.write("| 引脚索引 | 引脚编号 | enum SOP16CarPin | 四驱/差速模式 | 双电机模式 |\n")
        f.write("|----------|----------|------------------|---------------|------------|\n")
        f.write("| pin0 | PA0 | SOP16_CAR_PIN_PWM_RF | PWM驱动右侧轮子向前 | PWM驱动轮子向前 |\n")
        f.write("| pin1 | PA1 | SOP16_CAR_PIN_PWM_RB | PWM驱动右侧轮子向后 | PWM驱动轮子向后 |\n")
        f.write("| pin2 | PA2 | SOP16_CAR_PIN_PWM_LF | PWM驱动左侧轮子向前 | GPIO驱动轮子向左 |\n")
        f.write("| pin3 | PA3 | SOP16_CAR_PIN_PWM_LB | PWM驱动左侧轮子向后 | GPIO驱动轮子向右 |\n")
        f.write("| pin4 | PA4 | SOP16_CAR_PIN_PWM_W1 | PWM喷雾 | PWM喷雾/PWM转盘左转 |\n")
        f.write("| pin5 | PA5 | SOP16_CAR_PIN_PWM_W2 | PWM喷雾 | PWM喷雾/PWM挖掘机转盘  |\n")
        f.write("| pin6 | PA6 | SOP16_CAR_PIN_PWM_W3 | PWM喷雾 | PWM喷雾/PWM挖掘机挖臂 |\n")
        f.write("| pin7 | PA7 | SOP16_CAR_PIN_PWM_W4 | PWM喷雾 | PWM喷雾/PWM挖掘机挖臂 |\n")
        f.write("| pin8 | PB0 | SOP16_CAR_PIN_GPIO_LED_OUT | GPIO控制LED输出 |\n")
        f.write("| pin9 | PB1 | SOP16_CAR_PIN_CHG_DETECT | 充电检测 | 充电检测 |\n")
        f.write("| pin10 | PB2 | SOP16_CAR_PIN_CHG_FULL | 电池充满检测 | 电池充满检测 |\n")
        f.write("\n")
        
        f.write("### 各模式管脚功能映射\n\n")
        f.write("#### TOY_S16_CAR_MODE_DIFF_SPRAY 模式 (差速底盘 + 喷雾 + LED + 充电)\n\n")
        f.write("**资源配置**: 4路PWM(前后左右) + 2路PWM(喷雾W1/W2) + LED + 充电(2个GPIO)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP16_CAR_PIN_PWM_LF | PWM Left Forward | PWM驱动左侧轮子向前 |\n")
        f.write("| pin1 | SOP16_CAR_PIN_PWM_RF | PWM Right Forward | PWM驱动右侧轮子向前 |\n")
        f.write("| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM Spray W1 | PWM喷雾泵W通道1控制 |\n")
        f.write("| pin3 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | 未使用 |\n")
        f.write("| pin4 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |\n")
        f.write("| pin5 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |\n")
        f.write("| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |\n")
        f.write("| pin7 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | 未使用 |\n")
        f.write("| pin8 | SOP16_CAR_PIN_PWM_LB | PWM Left Backward | PWM驱动左侧轮子向后 |\n")
        f.write("| pin9 | SOP16_CAR_PIN_PWM_RB | PWM Right Backward | PWM驱动右侧轮子向后 |\n")
        f.write("| pin10 | SOP16_CAR_PIN_PWM_W2 | PWM Spray W2 | PWM喷雾泵W通道2控制 |\n")
        f.write("\n")
        
        f.write("#### TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR 模式 (双电机底盘 + 转盘/挖臂 + LED + 充电)\n\n")
        f.write("**资源配置**: 2路PWM(前后) + 2路GPIO(左右) + 4路PWM(转盘W1/W2 + 挖臂W3/W4) + LED + 充电(2个GPIO)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP16_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |\n")
        f.write("| pin1 | SOP16_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |\n")
        f.write("| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM W1 | PWM转盘左转控制 |\n")
        f.write("| pin3 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |\n")
        f.write("| pin4 | SOP16_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |\n")
        f.write("| pin5 | SOP16_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |\n")
        f.write("| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |\n")
        f.write("| pin7 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |\n")
        f.write("| pin8 | SOP16_CAR_PIN_PWM_W2 | PWM W2 | PWM转盘右转控制 |\n")
        f.write("| pin9 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | PWM挖臂向下控制 |\n")
        f.write("| pin10 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | PWM挖臂向上控制 |\n")
        f.write("\n")
        
        f.write("#### TOY_S16_CAR_MODE_2MOTOR_SPRAY 模式 (双电机底盘 + 喷雾 + LED + 充电)\n\n")
        f.write("**资源配置**: 2路PWM(前后) + 2路GPIO(左右) + 1路PWM(喷雾W1) + LED + 充电(2个GPIO)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP16_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |\n")
        f.write("| pin1 | SOP16_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |\n")
        f.write("| pin2 | SOP16_CAR_PIN_PWM_W1 | PWM Spray W1 | PWM喷雾泵W通道1控制 |\n")
        f.write("| pin3 | SOP16_CAR_PIN_GPIO_LED_OUT | LED | GPIO控制LED状态输出 |\n")
        f.write("| pin4 | SOP16_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |\n")
        f.write("| pin5 | SOP16_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |\n")
        f.write("| pin6 | SOP16_CAR_PIN_CHG_DETECT | CHG_DETECT | GPIO充电检测 |\n")
        f.write("| pin7 | SOP16_CAR_PIN_CHG_FULL | CHG_FULL | GPIO充电完成检测 |\n")
        f.write("| pin8 | SOP16_CAR_PIN_PWM_W2 | PWM W2 | 扩展PWM W2 |\n")
        f.write("| pin9 | SOP16_CAR_PIN_PWM_W3 | PWM W3 | 扩展PWM W3 |\n")
        f.write("| pin10 | SOP16_CAR_PIN_PWM_W4 | PWM W4 | 扩展PWM W4 |\n")
        f.write("\n")
        
        # 文件列表
        f.write("## 配置文件列表\n\n")
        
        # 按模式分组
        modes = {}
        for entry in doc_entries:
            m = entry["mode"]
            if m not in modes:
                modes[m] = []
            modes[m].append(entry)
        
        # 模式名称映射
        mode_name_mapping = {
            "DEFAULT": "默认模式",
        }
        
        for mode_name, entries in sorted(modes.items()):
            display_mode_name = mode_name_mapping.get(mode_name, mode_name)
            f.write(f"### {display_mode_name}\n\n")
            f.write("| 文件名 | 测试类型 | 参数设置 | 说明 |\n")
            f.write("|--------|----------|----------|------|\n")
            
            for entry in entries:
                # 格式化参数显示
                params_str = ""
                shown_params = set()  # 跟踪已显示的参数
                
                for pname, pval in entry.get("params", {}).items():
                    params_str += f"{pname}={pval.get('desc', '')}, "
                    shown_params.add(pname)
                
                # 添加其他关键参数（避免重复）
                cfg_param = entry.get("cfg_param", {})
                
                # Pin 配置 (显示非默认的)
                pin_names = {0: "PWM_LF", 1: "PWM_LB", 2: "PWM_RF", 3: "PWM_RB", 4: "PWM_W1",
                            5: "PWM_W2", 6: "PWM_W3", 7: "PWM_W4", 8: "GPIO_LED_OUT", 
                            9: "CHG_DETECT", 10: "CHG_FULL"}
                for i in range(11):
                    pin_val = cfg_param.get(f"pin{i}", i if i < 8 else i)
                    if pin_val != i and pin_val != 0 and f"pin{i}" not in shown_params:
                        params_str += f"pin{i}={pin_names.get(pin_val, pin_val)}, "
                
                # 电压参数（避免重复）
                if "shutdown_volt" not in shown_params and cfg_param.get("shutdown_volt", 8) != 8:
                    shutdown_names = {0: "DEFAULT", 1: "2.6V", 2: "2.7V", 3: "2.8V", 4: "2.9V", 
                                     5: "3.0V", 6: "3.1V", 7: "3.2V", 8: "OFF"}
                    params_str += f"shutdown_volt={shutdown_names.get(cfg_param['shutdown_volt'], cfg_param['shutdown_volt'])}, "
                
                # PWM占空比（避免重复）
                if cfg_param.get("pwm_duty_dual_en", 0) == 1:
                    duty_left = cfg_param.get("pwm_duty_left", 0)
                    duty_right = cfg_param.get("pwm_duty_right", 0)
                    duty_w = cfg_param.get("pwm_duty_w", 0)
                    duty_map = {0: "100", 1: "40", 2: "50", 3: "60", 4: "70", 5: "80", 6: "90", 7: "100"}
                    pwm_duty_left_default = PARAM_DEFINITIONS["pwm_duty_left"]["default"]
                    pwm_duty_right_default = PARAM_DEFINITIONS["pwm_duty_right"]["default"]
                    pwm_duty_w_default = PARAM_DEFINITIONS["pwm_duty_w"]["default"]
                    if duty_left != pwm_duty_left_default and "pwm_duty_left" not in shown_params:
                        params_str += f"pwm_duty_left={duty_map.get(duty_left, duty_left)}%, "
                    if duty_right != pwm_duty_right_default and "pwm_duty_right" not in shown_params:
                        params_str += f"pwm_duty_right={duty_map.get(duty_right, duty_right)}%, "
                    if duty_w != pwm_duty_w_default and "pwm_duty_w" not in shown_params:
                        params_str += f"pwm_duty_w={duty_map.get(duty_w, duty_w)}%, "
                
                # 其他功能参数（避免重复）
                if "pwm_freq" not in shown_params and cfg_param.get("pwm_freq", 0) != 0:
                    freq_names = {0: "DEFAULT", 1: "250Hz", 2: "500Hz", 3: "16kHz"}
                    params_str += f"pwm_freq={freq_names.get(cfg_param['pwm_freq'], cfg_param['pwm_freq'])}, "
                
                if "rev_delay" not in shown_params and cfg_param.get("rev_delay", 0) != 0 and cfg_param.get("rev_delay", 0) != 3:
                    delay_names = {0: "DEFAULT", 1: "0ms", 2: "50ms", 3: "100ms", 4: "150ms", 5: "200ms", 6: "250ms", 7: "300ms"}
                    params_str += f"rev_delay={delay_names.get(cfg_param['rev_delay'], cfg_param['rev_delay'])}, "
                
                if "spray_duration" not in shown_params and cfg_param.get("spray_duration", 0) != 0 and cfg_param.get("spray_duration", 0) != 1:
                    spray_names = {0: "DEFAULT", 1: "30s", 2: "1min", 3: "2min", 4: "always"}
                    params_str += f"spray_duration={spray_names.get(cfg_param['spray_duration'], cfg_param['spray_duration'])}, "
                
                if "spray_pwm_duty" not in shown_params and cfg_param.get("spray_pwm_duty", 0) != 0 and cfg_param.get("spray_pwm_duty", 0) != 8:
                    spray_duty_names = {0: "DEFAULT", 1: "7%", 2: "10%", 3: "15%", 4: "20%", 5: "25%", 6: "30%", 7: "35%", 8: "40%"}
                    params_str += f"spray_pwm_duty={spray_duty_names.get(cfg_param['spray_pwm_duty'], cfg_param['spray_pwm_duty'])}, "
                
                if "spray_w_mask" not in shown_params and cfg_param.get("spray_w_mask", 0x0F) != 0x0F:
                    params_str += f"spray_w_mask=0x{cfg_param['spray_w_mask']:X}, "
                
                if "sleep_timeout" not in shown_params and cfg_param.get("sleep_timeout", 0) != 0:
                    sleep_names = {0: "DEFAULT", 1: "5min", 2: "10min", 3: "15min", 4: "never"}
                    params_str += f"sleep_timeout={sleep_names.get(cfg_param['sleep_timeout'], cfg_param['sleep_timeout'])}, "
                
                if "paired_indicator" not in shown_params and cfg_param.get("paired_indicator", 0) != 0:
                    paired_names = {0: "DEFAULT", 1: "OFF", 2: "BLINK"}
                    params_str += f"paired_indicator={paired_names.get(cfg_param['paired_indicator'], cfg_param['paired_indicator'])}, "
                
                # 特殊功能和演示模式（避免重复）
                if "special_func" not in shown_params and cfg_param.get("special_func", 0) != 0:
                    special_names = {0: "DEFAULT", 1: "SPRAY", 2: "EXCAVATOR"}
                    params_str += f"special_func={special_names.get(cfg_param['special_func'], cfg_param['special_func'])}, "
                
                if "demo_mode" not in shown_params and cfg_param.get("demo_mode", 0) != 0:
                    demo_names = {0: "DEFAULT", 1: "FIGURE8", 2: "CIRCLE", 3: "SQUARE"}
                    params_str += f"demo_mode={demo_names.get(cfg_param['demo_mode'], cfg_param['demo_mode'])}, "
                
                if cfg_param.get("drive_mode", 0) != 0:
                    params_str += f"drive_mode={'DIFF' if cfg_param['drive_mode']==0 else '2MOTOR'}, "
                
                # 访问码组和发射功率
                if cfg_param.get("access_code_group", 0) != 0:
                    params_str += f"access_code_group={cfg_param['access_code_group']}, "
                if cfg_param.get("normal_tx_power", 0) != 0:
                    params_str += f"normal_tx_power={cfg_param['normal_tx_power']}, "
                
                params_str = params_str.rstrip(", ") if params_str else "默认"
                
                f.write(f"| {entry['filename']} | {entry['type']} | {params_str} | {entry['description']} |\n")
            
            f.write("\n")
        
        # 参数配置说明表格
        f.write("## 参数配置说明\n\n")
        
        # Pin 名称映射
        pin_value_to_name = {
            0: "PWM_RF", 1: "PWM_RB", 2: "PWM_LF", 3: "PWM_LB",
            4: "PWM_W1", 5: "PWM_W2", 6: "PWM_W3", 7: "PWM_W4",
            8: "GPIO_LED_OUT", 9: "CHG_DETECT", 10: "CHG_FULL"
        }
        
        # 特殊功能映射
        special_func_names = {0: "DEFAULT", 1: "SPRAY", 2: "EXCAVATOR"}
        paired_indicator_names = {0: "DEFAULT", 1: "OFF", 2: "ALWAYS"}
        demo_mode_names = {0: "DEFAULT", 1: "FIGURE8", 2: "SPIN"}
        
        # 为每个条目显示完整的参数配置
        for entry in doc_entries:
            cfg_param = entry.get("cfg_param", {})
            test_type = entry.get("type", "")
            params = entry.get("params", {})
            config_name = entry.get("filename", "").replace("_0x0f000.bin", "")
            
            f.write(f"### {config_name}\n\n")
            f.write("| 参数 | 值 | 说明 |\n")
            f.write("|------|-----|------|\n")
            
            # pin 配置
            for i in range(11):
                pin_val = cfg_param.get(f"pin{i}", 0)
                pin_name = pin_value_to_name.get(pin_val, f"VALUE_{pin_val}")
                f.write(f"| pin{i} | {pin_name} | 引脚{i}功能配置 |\n")
            
            # 其他固定配置
            special_func = cfg_param.get('special_func', 0)
            f.write(f"| special_func | {special_func_names.get(special_func, special_func)} | 特殊功能 |\n")
            
            paired_indicator = cfg_param.get('paired_indicator', 0)
            paired_desc = paired_indicator_names.get(paired_indicator, paired_indicator)
            if paired_indicator == 0:  # DEFAULT
                paired_desc = "DEFAULT (默认是OFF)"
            f.write(f"| paired_indicator | {paired_desc} | 配对指示灯 |\n")
            
            demo_mode = cfg_param.get('demo_mode', 0)
            demo_desc = demo_mode_names.get(demo_mode, demo_mode)
            if demo_mode == 0:  # DEFAULT
                demo_desc = "DEFAULT (8字形演示)"
            f.write(f"| demo_mode | {demo_desc} | 演示模式 |\n")
            
            f.write(f"| drive_mode | {cfg_param.get('drive_mode', 0)} | 驱动模式 (0=正常, 1=差速) |\n")
            f.write(f"| led_polar | {cfg_param.get('led_polar', 0)} | LED极性 (0=低电平亮, 1=高电平亮) |\n")
            f.write(f"| pwm_duty_dual_en | {cfg_param.get('pwm_duty_dual_en', 0)} | 双电机PWM独立使能 |\n")
            f.write(f"| reserved0 | {cfg_param.get('reserved0', 0)} | 保留位0 |\n")
            f.write(f"| reserved1 | {cfg_param.get('reserved1', 0)} | 保留位1 |\n")
            
            # 新增的字段（原先在 efuse_dev_info_t，现在移到 CfgParamUnion 尾部）
            f.write(f"| access_code_group | {cfg_param.get('access_code_group', 0)} | 访问码组 |\n")
            f.write(f"| normal_tx_power | {cfg_param.get('normal_tx_power', 0)} | 正常发射功率 |\n")
            
            # 可测试配置参数
            if test_type == "default":
                # default 模式：显示所有参数的默认值
                for param_name, param_def in PARAM_DEFINITIONS.items():
                    default_val = param_def['default']
                    desc = ""
                    for v, d in param_def['options']:
                        if v == default_val:
                            desc = d
                            break
                    f.write(f"| {param_name} | {desc} | {param_def['name']} |\n")
            else:
                # 其他模式：显示所有在 PARAM_DEFINITIONS 中的参数
                for param_name, param_def in PARAM_DEFINITIONS.items():
                    if param_name in cfg_param:
                        param_val = cfg_param[param_name]
                        desc = "-"
                        for v, d in param_def["options"]:
                            if v == param_val:
                                desc = d
                                break
                        f.write(f"| {param_name} | {desc} | {param_def['name']} |\n")
            
            f.write("\n")
        
        # 参数详细说明
        f.write("## 参数详细说明\n\n")
        for param_name, param_def in PARAM_DEFINITIONS.items():
            f.write(f"### {param_name} - {param_def['name']}\n\n")
            f.write(f"**默认值**: {param_def['default']}  ")
            f.write(f"**选项**:  ")
            opts = [f"{v}({d})" for v, d in param_def["options"]]
            f.write(", ".join(opts))
            f.write("\n\n")
        
        f.write("## 烧录说明\n\n")
        f.write("- **烧录地址**: 0xF000 (Flash)\n")
        f.write("- **文件大小**: 256 bytes\n")
        f.write("- **未写数据**: 0xFF\n")
        f.write("- **固件类型**: CAR16 (SOP16 封装遥控车)\n\n")
        
        f.write("## 测试建议\n\n")
        f.write("1. **单参数测试**: 用于验证每个参数的独立功能\n")
        f.write("2. **双参数组合**: 用于验证参数间的兼容性\n")
        f.write("3. **场景测试**: 用于验证实际使用场景\n")
        f.write("4. **全参数组合**: 用于验证极端非默认配置\n")
    
    print(f"\nDocumentation generated: {output_file}")


# =============================================================================
# 主函数
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='S16 小车参数组合测试配置生成工具 V2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python %(prog)s --mode single                    # 单参数测试
  python %(prog)s --mode pair                      # 双参数组合
  python %(prog)s --mode scenario                  # 场景测试
  python %(prog)s --mode full                      # 全参数组合
  python %(prog)s --mode default                   # 生成三种模式的默认配置
  python %(prog)s --mode all                       # 生成所有模式
        '''
    )
    
    parser.add_argument('--mode', choices=['single', 'pair', 'scenario', 'full', 'default', 'all'], 
                       default='all',
                       help='测试模式 (默认: all)')
    parser.add_argument('--output-dir', '-o', default='.',
                       help='输出目录 (默认: 当前目录)')
    parser.add_argument('--modes', nargs='+', 
                       default=['TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR', 
                               'TOY_S16_CAR_MODE_DIFF_SPRAY',
                               'TOY_S16_CAR_MODE_2MOTOR_SPRAY'],
                       help='要生成的模式列表')
    
    args = parser.parse_args()
    
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 70)
    print("S16 小车参数组合测试配置生成工具 V2")
    print("=" * 70)
    print(f"测试模式: {args.mode}")
    print(f"输出目录: {output_dir}")
    print(f"覆盖模式: {', '.join(args.modes)}")
    print()
    
    all_configs = []
    
    for mode in args.modes:
        print(f"\n处理模式: {mode}")
        
        if args.mode in ['single', 'all']:
            print("  生成单参数测试...")
            configs = generate_single_param_tests(mode)
            all_configs.extend(configs)
            print(f"    -> {len(configs)} 个配置")
        
        if args.mode in ['pair', 'all']:
            print("  生成双参数组合测试...")
            configs = generate_pair_tests(mode)
            all_configs.extend(configs)
            print(f"    -> {len(configs)} 个配置")
        
        if args.mode in ['scenario', 'all']:
            print("  生成场景测试...")
            configs = generate_scenario_tests(mode)
            all_configs.extend(configs)
            print(f"    -> {len(configs)} 个配置")
        
        if args.mode in ['full', 'all']:
            print("  生成全参数组合测试...")
            configs = generate_full_combination_tests(mode)
            all_configs.extend(configs)
            print(f"    -> {len(configs)} 个配置")
        
        if args.mode in ['default']:
            print("  生成默认配置...")
            configs = generate_default_config(mode)
            all_configs.extend(configs)
            print(f"    -> {len(configs)} 个配置")
    
    print(f"\n总共生成 {len(all_configs)} 个测试配置")
    print()
    
    # 生成文件
    doc_entries = []
    generate_files(all_configs, output_dir, doc_entries)
    
    # 生成文档
    doc_file = os.path.join(output_dir, f"README_{args.mode}.md")
    generate_documentation(doc_entries, doc_file, args.mode)
    
    print()
    print("=" * 70)
    print("生成完成!")
    print("=" * 70)
    print(f"\n输出文件统计:")
    print(f"  - Bin 文件: {len([f for f in os.listdir(output_dir) if f.endswith('.bin')])} 个")
    print(f"  - JSON 文件: {len([f for f in os.listdir(output_dir) if f.endswith('.json')])} 个")
    print(f"  - 文档: {os.path.basename(doc_file)}")


if __name__ == '__main__':
    main()
