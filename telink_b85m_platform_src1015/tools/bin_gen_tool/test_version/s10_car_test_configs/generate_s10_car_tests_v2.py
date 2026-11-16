#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S10 小车参数组合测试配置生成工具 V2

功能说明:
    本工具用于生成 S10 小车（SOP10 封装）的测试配置文件，支持多种测试模式。
    可同时生成 Flash 版本（256字节，烧录地址 0xF000）和 EFuse 版本（32字节，烧录地址 0x00000）的 bin 文件。

支持多种测试模式：
    1. 单参数测试 (single)    - 每次只变化一个参数，其他保持默认值
    2. 双参数组合测试 (pair)  - 两个参数同时变化，测试参数间的影响
    3. 场景组合 (scenario)    - 预定义的实际使用场景配置
    4. 默认配置 (default)     - 生成三种工作模式的默认参数配置
    5. 全部模式 (all)         - 生成上述所有测试模式（默认）

覆盖的工作模式：
    - 2MOTOR_SPRAY : 双电机 + 2路PWM + 喷雾
    - 2MOTOR_LED   : 双电机 + 2路PWM + LED
    - DIFF_4PWM_LED: 四驱/差速 + 4路PWM + LED

生成的文件：
    - Bin 文件: {MODE}_{TEST_TYPE}_{PARAM}_{VALUE}_0x0f000.bin  (Flash 版本)
               {MODE}_{TEST_TYPE}_{PARAM}_{VALUE}_0x00000.bin (EFuse 版本)
    - JSON 文件: 包含完整的参数配置信息
    - MD 文档  : README_{mode}.md 记录所有生成的配置文件

使用方法:
    python3 generate_s10_car_tests_v2.py --mode single    # 单参数测试
    python3 generate_s10_car_tests_v2.py --mode pair      # 双参数组合测试
    python3 generate_s10_car_tests_v2.py --mode scenario  # 场景组合测试
    python3 generate_s10_car_tests_v2.py --mode default   # 生成三种模式的默认配置
    python3 generate_s10_car_tests_v2.py --mode all       # 生成所有测试模式（默认）

高级选项:
    python3 generate_s10_car_tests_v2.py --mode default --output-dir ./output  # 指定输出目录
    python3 generate_s10_car_tests_v2.py --mode single --modes TOY_S10_CAR_MODE_2MOTOR_SPRAY  # 只生成指定模式

参数说明:
    --mode      测试模式: single/pair/scenario/default/all (默认: all)
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
    SOP10CarPin, SOP10CarSpecialFunc, SOP10CarShutdownVoltage,
    SOP10CarPwmFreq, SOP10CarPwmDutyCycle, SOP10CarDirSwitchDelay,
    SOP10CarSprayDuration, SOP10CarSprayPwmDuty, SOP10CarPairedIndicator,
    SOP10CarSleepTimeout
)


# =============================================================================
# 参数定义
# =============================================================================

PARAM_DEFINITIONS = {
    "shutdown_volt": {
        "name": "关机电压",
        "default": SOP10CarShutdownVoltage.DEFAULT,
        "options": [
            (SOP10CarShutdownVoltage.DEFAULT, "DEFAULT (不检测)"),
            (SOP10CarShutdownVoltage.VOLT_2_6V, "2.6V"),
            (SOP10CarShutdownVoltage.VOLT_2_7V, "2.7V"),
            (SOP10CarShutdownVoltage.VOLT_2_8V, "2.8V"),
            (SOP10CarShutdownVoltage.VOLT_2_9V, "2.9V"),
            (SOP10CarShutdownVoltage.VOLT_3_0V, "3.0V"),
            (SOP10CarShutdownVoltage.VOLT_3_1V, "3.1V"),
            (SOP10CarShutdownVoltage.VOLT_3_2V, "3.2V"),
            (SOP10CarShutdownVoltage.OFF, "OFF"),
        ]
    },
    "pwm_duty_left": {
        "name": "左轮PWM占空比",
        "default": SOP10CarPwmDutyCycle.DEFAULT,
        "options": [
            (SOP10CarPwmDutyCycle.DEFAULT, "100"),
            (SOP10CarPwmDutyCycle.DUTY_40_PERCENT, "40"),
            (SOP10CarPwmDutyCycle.DUTY_50_PERCENT, "50"),
            (SOP10CarPwmDutyCycle.DUTY_60_PERCENT, "60"),
            (SOP10CarPwmDutyCycle.DUTY_70_PERCENT, "70"),
            (SOP10CarPwmDutyCycle.DUTY_80_PERCENT, "80"),
            (SOP10CarPwmDutyCycle.DUTY_90_PERCENT, "90"),
            (SOP10CarPwmDutyCycle.DUTY_100_PERCENT, "100"),
        ]
    },
    "pwm_duty_right": {
        "name": "右轮PWM占空比",
        "default": SOP10CarPwmDutyCycle.DEFAULT,
        "options": [
            (SOP10CarPwmDutyCycle.DEFAULT, "100"),
            (SOP10CarPwmDutyCycle.DUTY_40_PERCENT, "40"),
            (SOP10CarPwmDutyCycle.DUTY_50_PERCENT, "50"),
            (SOP10CarPwmDutyCycle.DUTY_60_PERCENT, "60"),
            (SOP10CarPwmDutyCycle.DUTY_70_PERCENT, "70"),
            (SOP10CarPwmDutyCycle.DUTY_80_PERCENT, "80"),
            (SOP10CarPwmDutyCycle.DUTY_90_PERCENT, "90"),
            (SOP10CarPwmDutyCycle.DUTY_100_PERCENT, "100"),
        ]
    },
    "pwm_freq": {
        "name": "PWM频率",
        "default": SOP10CarPwmFreq.DEFAULT,
        "options": [
            (SOP10CarPwmFreq.DEFAULT, "DEFAULT (250Hz)"),
            (SOP10CarPwmFreq.FREQ_250HZ, "250Hz"),
            (SOP10CarPwmFreq.FREQ_500HZ, "500Hz"),
            (SOP10CarPwmFreq.FREQ_16KHZ, "16kHz"),
        ]
    },
    "rev_delay": {
        "name": "正反转延迟",
        "default": SOP10CarDirSwitchDelay.DEFAULT,
        "options": [
            (SOP10CarDirSwitchDelay.DEFAULT, "DEFAULT (100ms)"),
            (SOP10CarDirSwitchDelay.DELAY_0MS, "0ms"),
            (SOP10CarDirSwitchDelay.DELAY_50MS, "50ms"),
            (SOP10CarDirSwitchDelay.DELAY_100MS, "100ms"),
            (SOP10CarDirSwitchDelay.DELAY_150MS, "150ms"),
            (SOP10CarDirSwitchDelay.DELAY_200MS, "200ms"),
            (SOP10CarDirSwitchDelay.DELAY_250MS, "250ms"),
            (SOP10CarDirSwitchDelay.DELAY_300MS, "300ms"),
        ]
    },
    "spray_duration": {
        "name": "喷雾时长",
        "default": SOP10CarSprayDuration.DEFAULT,
        "options": [
            (SOP10CarSprayDuration.DEFAULT, "DEFAULT (30s)"),
            (SOP10CarSprayDuration.DURATION_30S, "30s"),
            (SOP10CarSprayDuration.DURATION_1MIN, "1min"),
            (SOP10CarSprayDuration.DURATION_2MIN, "2min"),
            (SOP10CarSprayDuration.ALWAYS, "always"),
        ]
    },
    "spray_pwm_duty": {
        "name": "喷雾PWM占空比",
        "default": SOP10CarSprayPwmDuty.DEFAULT,
        "options": [
            (SOP10CarSprayPwmDuty.DEFAULT, "DEFAULT (40%)"),
            (SOP10CarSprayPwmDuty.DUTY_7_PERCENT, "7"),
            (SOP10CarSprayPwmDuty.DUTY_10_PERCENT, "10"),
            (SOP10CarSprayPwmDuty.DUTY_15_PERCENT, "15"),
            (SOP10CarSprayPwmDuty.DUTY_20_PERCENT, "20"),
            (SOP10CarSprayPwmDuty.DUTY_25_PERCENT, "25"),
            (SOP10CarSprayPwmDuty.DUTY_30_PERCENT, "30"),
            (SOP10CarSprayPwmDuty.DUTY_35_PERCENT, "35"),
            (SOP10CarSprayPwmDuty.DUTY_40_PERCENT, "40"),
        ]
    },
    "sleep_timeout": {
        "name": "休眠超时",
        "default": SOP10CarSleepTimeout.DEFAULT,
        "options": [
            (SOP10CarSleepTimeout.DEFAULT, "DEFAULT (从不休眠)"),
            (SOP10CarSleepTimeout.TIMEOUT_5MIN, "5min"),
            (SOP10CarSleepTimeout.TIMEOUT_10MIN, "10min"),
            (SOP10CarSleepTimeout.TIMEOUT_15MIN, "15min"),
            (SOP10CarSleepTimeout.NEVER, "never"),
        ]
    },
}


# =============================================================================
# 场景定义
# =============================================================================

SCENARIOS = {
    "all_open_paired_indicatorled_always_on1": {
        "name": "全开模式1",
        "description": "所有参数都非默认值, 配网后led一直亮",
        "params": {
            "shutdown_volt": SOP10CarShutdownVoltage.VOLT_2_6V,
            "pwm_duty_left": SOP10CarPwmDutyCycle.DUTY_50_PERCENT,
            "pwm_duty_right": SOP10CarPwmDutyCycle.DUTY_70_PERCENT,
            "pwm_freq": SOP10CarPwmFreq.FREQ_500HZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP10CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DUTY_40_PERCENT,
            "sleep_timeout": SOP10CarSleepTimeout.TIMEOUT_5MIN,
            "paired_indicator": SOP10CarPairedIndicator.ALWAYS,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_open_paired_indicatorled_off": {
        "name": "全开模式2",
        "description": "所有参数都非默认值，配网后led灭灯",
        "params": {
            "shutdown_volt": SOP10CarShutdownVoltage.VOLT_2_6V,
            "pwm_duty_left": SOP10CarPwmDutyCycle.DUTY_50_PERCENT,
            "pwm_duty_right": SOP10CarPwmDutyCycle.DUTY_70_PERCENT,
            "pwm_freq": SOP10CarPwmFreq.FREQ_500HZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_100MS,
            "sleep_timeout": SOP10CarSleepTimeout.TIMEOUT_5MIN,
            "paired_indicator": SOP10CarPairedIndicator.OFF,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_open_paired_indicatorled_always_on2": {
        "name": "全开模式2",
        "description": "所有参数都非默认值，配网后led灭灯",
        "params": {
            "shutdown_volt": SOP10CarShutdownVoltage.VOLT_2_6V,
            "pwm_duty_left": SOP10CarPwmDutyCycle.DUTY_50_PERCENT,
            "pwm_duty_right": SOP10CarPwmDutyCycle.DUTY_70_PERCENT,
            "pwm_freq": SOP10CarPwmFreq.FREQ_500HZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_100MS,
            "sleep_timeout": SOP10CarSleepTimeout.TIMEOUT_5MIN,
            "paired_indicator": SOP10CarPairedIndicator.ALWAYS,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_min_values": {
        "name": "全部最小值模式",
        "description": "所有参数配置为最小值",
        "params": {
            "shutdown_volt": SOP10CarShutdownVoltage.DEFAULT,
            "pwm_duty_left": SOP10CarPwmDutyCycle.DEFAULT,
            "pwm_duty_right": SOP10CarPwmDutyCycle.DEFAULT,
            "pwm_freq": SOP10CarPwmFreq.DEFAULT,
            "rev_delay": SOP10CarDirSwitchDelay.DEFAULT,
            "spray_duration": SOP10CarSprayDuration.DEFAULT,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DEFAULT,
            "sleep_timeout": SOP10CarSleepTimeout.DEFAULT,
            "paired_indicator": SOP10CarPairedIndicator.DEFAULT,
            "access_code_group": 0,
            "normal_tx_power": 0,
            "led_polar": 0,
        }
    },
    "all_max_values": {
        "name": "全部最大值模式",
        "description": "所有参数配置为最大值",
        "params": {
            "shutdown_volt": SOP10CarShutdownVoltage.OFF,
            "pwm_duty_left": SOP10CarPwmDutyCycle.DUTY_100_PERCENT,
            "pwm_duty_right": SOP10CarPwmDutyCycle.DUTY_100_PERCENT,
            "pwm_freq": SOP10CarPwmFreq.FREQ_16KHZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_300MS,
            "spray_duration": SOP10CarSprayDuration.ALWAYS,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DUTY_40_PERCENT,
            "sleep_timeout": SOP10CarSleepTimeout.NEVER,
            "paired_indicator": SOP10CarPairedIndicator.ALWAYS,
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
    """设置配置参数，并处理依赖关系(如PWM独立控制)"""
    config["cfg_param"][param_name] = value
    # 如果设置了左右轮占空比，强制开启独立控制，确保参数生效
    if param_name in ["pwm_duty_left", "pwm_duty_right"]:
        config["cfg_param"]["pwm_duty_dual_en"] = 1

# =============================================================================
# 基础配置
# =============================================================================

def get_base_config(mode_name: str) -> Dict:
    """获取基础配置模板"""
    base = {
        "name": mode_name,
        "efuse_dev_info": {
            "toy_type": 2
        },
        "cfg_param": {
            "pin0": SOP10CarPin.PWM_RF,
            "pin1": SOP10CarPin.PWM_RB,
            "pin2": SOP10CarPin.PWM_LF,
            "pin3": SOP10CarPin.PWM_SPRAY_LED,
            "pin4": SOP10CarPin.PWM_LB,
            "special_func": SOP10CarSpecialFunc.LED,
            "shutdown_volt": SOP10CarShutdownVoltage.OFF,
            "pwm_freq": SOP10CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP10CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP10CarPairedIndicator.OFF,
            "sleep_timeout": SOP10CarSleepTimeout.NEVER,
            "drive_mode": 0,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "access_code_group": 0,
            "normal_tx_power": 0
        }
    }
    
    if "2MOTOR_SPRAY" in mode_name:
        base["cfg_param"].update({
            "pin2": SOP10CarPin.PWM_SPRAY_LED,
            "pin3": SOP10CarPin.PWM_LF,
            "special_func": SOP10CarSpecialFunc.SPRAY,
            "drive_mode": 1,
        })
        base["description"] = "双电机 + 2路PWM + 喷雾"
        base["mode_label"] = "2MOTOR_SPRAY"
    elif "2MOTOR_LED" in mode_name:
        base["cfg_param"].update({
            "drive_mode": 1,
        })
        base["description"] = "双电机 + 2路PWM + LED"
        base["mode_label"] = "2MOTOR_LED"
    else:  # DIFF_4PWM_LED
        base["description"] = "四驱/差速 + 4路PWM + LED"
        base["mode_label"] = "DIFF_4PWM_LED"
    
    return base


# =============================================================================
# 配置生成器
# =============================================================================

def generate_single_param_tests(mode: str) -> List[Dict]:
    """生成单参数测试配置"""
    configs = []
    base = get_base_config(mode)
    
    for param_name, param_def in PARAM_DEFINITIONS.items():
        # 跳过不相关参数
        if "2MOTOR_LED" in mode and param_name in ["spray_duration", "spray_pwm_duty"]:
            continue
        if "DIFF_4PWM_LED" in mode and param_name in ["spray_duration", "spray_pwm_duty"]:
            continue
            
        for value, desc in param_def["options"]:
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
    
    # 生成参数对
    param_pairs = list(itertools.combinations(params, 2))
    
    seen_configs = set()
    
    for (name1, def1), (name2, def2) in param_pairs:
        # 跳过不相关参数组合
        if "2MOTOR_LED" in mode or "DIFF_4PWM_LED" in mode:
            if name1 in ["spray_duration", "spray_pwm_duty"] or name2 in ["spray_duration", "spray_pwm_duty"]:
                continue
        
        # 为每对参数生成几个关键组合
        # 策略: 最小+最小, 最小+最大, 最大+最小, 中间+中间
        opts1 = def1["options"]
        opts2 = def2["options"]
        
        key_combinations = [
            (0, 0),  # 最小 + 最小
            (0, -1),  # 最小 + 最大
            (-1, 0),  # 最大 + 最小
            (len(opts1)//2, len(opts2)//2),  # 中间 + 中间
        ]
        
        for idx, (i, j) in enumerate(key_combinations):
            val1, desc1_raw = opts1[i][0], opts1[i][1]
            val2, desc2_raw = opts2[j][0], opts2[j][1]
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
            configs.append(config)
    
    return configs


def generate_scenario_tests(mode: str) -> List[Dict]:
    """生成场景测试配置"""
    configs = []
    base = get_base_config(mode)
    
    for scenario_id, scenario in SCENARIOS.items():
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
            # 在 2MOTOR_LED 和 DIFF_4PWM_LED 模式下跳过喷雾相关参数
            if ("2MOTOR_LED" in mode or "DIFF_4PWM_LED" in mode) and param_name in ["spray_duration", "spray_pwm_duty"]:
                continue
            
            set_config_param(config, param_name, value)
            if param_name in PARAM_DEFINITIONS:
                # 查找描述
                for v, d in PARAM_DEFINITIONS[param_name]["options"]:
                    if v == value:
                        config["test_params"][param_name] = {
                            "value": value,
                            "desc": d,
                            "name": PARAM_DEFINITIONS[param_name]["name"]
                        }
                        break
        
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


def generate_custom_combination(mode: str, combinations: List[Dict[str, Any]]) -> List[Dict]:
    """生成自定义组合配置"""
    configs = []
    base = get_base_config(mode)
    
    for idx, combo in enumerate(combinations):
        config = {
            "name": f"{mode}_CUSTOM_{idx+1:03d}",
            "description": f"{base['description']} - 自定义组合 #{idx+1}",
            "efuse_dev_info": base["efuse_dev_info"].copy(),
            "cfg_param": base["cfg_param"].copy(),
            "test_type": "custom",
            "mode_label": base["mode_label"],
            "test_params": {}
        }
        
        for param_name, value in combo.items():
            if param_name in PARAM_DEFINITIONS:
                set_config_param(config, param_name, value)
                # 查找描述
                for v, d in PARAM_DEFINITIONS[param_name]["options"]:
                    if v == value:
                        config["test_params"][param_name] = {
                            "value": value,
                            "desc": d,
                            "name": PARAM_DEFINITIONS[param_name]["name"]
                        }
                        break
        
        configs.append(config)
    
    return configs


# =============================================================================
# 文件生成
# =============================================================================

def sanitize_filename(name: str) -> str:
    """清理文件名，替换特殊字符"""
    # 将 . 替换为 _ 避免 os.path.splitext() 问题
    return name.replace(".", "_")


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
    from bin_gen_test import ConfigGenerator, TargetType, FwType
    
    for config in configs:
        config_name = config["name"]
        generated_files = []
        chip_id = None
        
        # 先生成 Flash 版本
        generator_flash = ConfigGenerator(TargetType.FLASH, FwType.CAR10)
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
        
        generator_efuse = ConfigGenerator(TargetType.EFUSE, FwType.CAR10)
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
            "cfg_param": config["cfg_param"],
            "description": config["description"]
        })
        
        print(f"  Generated: {generated_files[0]} (Flash) + {generated_files[1]} (EFuse)")


def generate_documentation(doc_entries: List[Dict], output_file: str, mode: str):
    """生成文档"""
    is_default_mode = (mode == "default")
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(f"# S10 小车测试配置文档 - {mode.upper()} 模式\n\n")
        f.write(f"本目录包含 S10 小车 **{mode}** 测试模式的配置文件。\n\n")
        
        # 测试模式说明
        mode_descriptions = {
            "single": "单参数测试 - 每次只变化一个参数，其他保持默认值",
            "pair": "双参数组合测试 - 两个参数同时变化，测试参数间的影响",
            "scenario": "场景测试 - 预定义的实际使用场景配置",
            "default": "默认配置 - 所有参数使用默认值",
            "custom": "自定义组合 - 用户指定的特定参数组合"
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
        f.write("SOP10 封装芯片引脚功能定义（参考 enum SOP10CarPin）：\n\n")
        f.write("| 引脚编号 | 引脚名称 | enum SOP10CarPin | 功能说明 |\n")
        f.write("|----------|----------|------------------|----------|\n")
        f.write("| PA0 | pin0 | SOP10_CAR_PIN_PWM_RF | 四驱/差速: PWM驱动右侧轮子向前 (Right Forward)<br>双电机: PWM驱动轮子向前 (Forward) |\n")
        f.write("| PA6 | pin1 | SOP10_CAR_PIN_PWM_RB | 四驱/差速: PWM驱动右侧轮子向后 (Right Backward)<br>双电机: PWM驱动轮子向后 (Backward) |\n")
        f.write("| PB0 | pin2 | SOP10_CAR_PIN_PWM_LF | 四驱/差速: PWM驱动左侧轮子向前 (Left Forward)<br>双电机: GPIO驱动轮子向左 (Left) |\n")
        f.write("| PA3 | pin3 | SOP10_CAR_PIN_PWM_LB | 四驱/差速: PWM驱动左侧轮子向后 (Left Backward)<br>双电机: GPIO驱动轮子向右 (Right) |\n")
        f.write("| PA5 | pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | PWM驱动喷雾 或 GPIO驱动LED |\n")
        f.write("\n")
        
        f.write("### 各模式管脚功能映射\n\n")
        f.write("#### 2MOTOR_LED 模式 (双电机 + 2路PWM前后 + 2路GPIO左右 + LED)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |\n")
        f.write("| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |\n")
        f.write("| pin2 | SOP10_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |\n")
        f.write("| pin3 | SOP10_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |\n")
        f.write("| pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | LED | GPIO驱动LED |\n")
        f.write("\n")
        
        f.write("#### 2MOTOR_SPRAY 模式 (双电机 + 2路PWM前后 + 2路GPIO左右 + 喷雾)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Forward | PWM驱动轮子向前 |\n")
        f.write("| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Backward | PWM驱动轮子向后 |\n")
        f.write("| pin2 | SOP10_CAR_PIN_PWM_SPRAY_LED | PWM Spray | PWM驱动喷雾泵 |\n")
        f.write("| pin3 | SOP10_CAR_PIN_PWM_LF | GPIO Left | GPIO驱动轮子向左 |\n")
        f.write("| pin4 | SOP10_CAR_PIN_PWM_LB | GPIO Right | GPIO驱动轮子向右 |\n")
        f.write("\n")
        
        f.write("#### DIFF_4PWM_LED 模式 (四驱/差速 + 4路PWM + LED)\n\n")
        f.write("| 引脚 | enum值 | 功能 | 说明 |\n")
        f.write("|------|--------|------|------|\n")
        f.write("| pin0 | SOP10_CAR_PIN_PWM_RF | PWM Right Forward | PWM驱动右侧轮子向前 |\n")
        f.write("| pin1 | SOP10_CAR_PIN_PWM_RB | PWM Right Backward | PWM驱动右侧轮子向后 |\n")
        f.write("| pin2 | SOP10_CAR_PIN_PWM_LF | PWM Left Forward | PWM驱动左侧轮子向前 |\n")
        f.write("| pin3 | SOP10_CAR_PIN_PWM_LB | PWM Left Backward | PWM驱动左侧轮子向后 |\n")
        f.write("| pin4 | SOP10_CAR_PIN_PWM_SPRAY_LED | LED | GPIO驱动LED |\n")
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
        
        for mode_name, entries in sorted(modes.items()):
            f.write(f"### {mode_name}\n\n")
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
                pin_names = {0: "PWM_RF", 1: "PWM_RB", 2: "PWM_LF", 3: "PWM_LB", 4: "PWM_SPRAY_LED"}
                default_pins = [0, 1, 2, 3, 4]
                for i in range(5):
                    pin_val = cfg_param.get(f"pin{i}", default_pins[i])
                    if pin_val != default_pins[i] and f"pin{i}" not in shown_params:
                        params_str += f"pin{i}={pin_names.get(pin_val, pin_val)}, "
                
                # 电压参数（避免重复）
                if "shutdown_volt" not in shown_params and cfg_param.get("shutdown_volt", 8) != 8:
                    shutdown_names = {0: "DEFAULT", 1: "2.6V", 2: "2.7V", 3: "2.8V", 4: "2.9V", 
                                     5: "3.0V", 6: "3.1V", 7: "3.2V", 8: "OFF"}
                    params_str += f"shutdown_volt={shutdown_names.get(cfg_param['shutdown_volt'], cfg_param['shutdown_volt'])}, "
                
                # PWM占空比
                if cfg_param.get("pwm_duty_dual_en", 0) == 1:
                    duty_left = cfg_param.get("pwm_duty_left", 0)
                    duty_right = cfg_param.get("pwm_duty_right", 0)
                    duty_map = {0: "100", 1: "40", 2: "50", 3: "60", 4: "70", 5: "80", 6: "90", 7: "100"}
                    pwm_duty_left_default = PARAM_DEFINITIONS["pwm_duty_left"]["default"]
                    pwm_duty_right_default = PARAM_DEFINITIONS["pwm_duty_right"]["default"]
                    if duty_left != pwm_duty_left_default and "pwm_duty_left" not in shown_params:
                        params_str += f"pwm_duty_left={duty_map.get(duty_left, duty_left)}%, "
                    if duty_right != pwm_duty_right_default and "pwm_duty_right" not in shown_params:
                        params_str += f"pwm_duty_right={duty_map.get(duty_right, duty_right)}%, "
                
                # 其他功能参数（避免重复）
                if "pwm_freq" not in shown_params and cfg_param.get("pwm_freq", 0) != 0:
                    freq_names = {0: "DEFAULT", 1: "250Hz", 2: "500Hz", 3: "16kHz"}
                    params_str += f"pwm_freq={freq_names.get(cfg_param['pwm_freq'], cfg_param['pwm_freq'])}, "
                
                if "rev_delay" not in shown_params and cfg_param.get("rev_delay", 0) != 0:
                    delay_names = {0: "DEFAULT", 1: "0ms", 2: "50ms", 3: "100ms", 4: "150ms", 5: "200ms", 6: "250ms", 7: "300ms"}
                    params_str += f"rev_delay={delay_names.get(cfg_param['rev_delay'], cfg_param['rev_delay'])}, "
                
                if "spray_duration" not in shown_params and cfg_param.get("spray_duration", 0) != 0:
                    spray_names = {0: "DEFAULT", 1: "30s", 2: "1min", 3: "2min", 4: "always"}
                    params_str += f"spray_duration={spray_names.get(cfg_param['spray_duration'], cfg_param['spray_duration'])}, "
                
                if "spray_pwm_duty" not in shown_params and cfg_param.get("spray_pwm_duty", 0) != 0:
                    spray_duty_names = {0: "DEFAULT", 1: "7%", 2: "10%", 3: "15%", 4: "20%", 5: "25%", 6: "30%", 7: "35%", 8: "40%"}
                    params_str += f"spray_pwm_duty={spray_duty_names.get(cfg_param['spray_pwm_duty'], cfg_param['spray_pwm_duty'])}, "
                
                if "sleep_timeout" not in shown_params and cfg_param.get("sleep_timeout", 0) != 0:
                    sleep_names = {0: "DEFAULT", 1: "5min", 2: "10min", 3: "15min", 4: "never"}
                    params_str += f"sleep_timeout={sleep_names.get(cfg_param['sleep_timeout'], cfg_param['sleep_timeout'])}, "
                
                if "paired_indicator" not in shown_params and cfg_param.get("paired_indicator", 0) != 0:
                    paired_names = {0: "DEFAULT", 1: "OFF", 2: "ALWAYS"}
                    params_str += f"paired_indicator={paired_names.get(cfg_param['paired_indicator'], cfg_param['paired_indicator'])}, "
                
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
        
        # Pin 名称映射 (SOP10CarPin 枚举值)
        pin_value_to_name = {
            0: "PWM_RF",
            1: "PWM_RB",
            2: "PWM_LF",
            3: "PWM_LB",
            4: "PWM_SPRAY_LED"
        }
        
        # 特殊功能映射
        special_func_names = {0: "DEFAULT", 1: "SPRAY", 2: "LED"}
        paired_indicator_names = {0: "DEFAULT", 1: "OFF", 2: "ALWAYS"}
        
        # 为每个条目显示完整的参数配置
        for entry in doc_entries:
            cfg_param = entry.get("cfg_param", {})
            mode_label = entry.get("mode", "UNKNOWN")
            test_type = entry.get("type", "")
            params = entry.get("params", {})
            config_name = entry.get("filename", "").replace("_0x0f000.bin", "")
            
            f.write(f"### {config_name}\n\n")
            f.write("| 参数 | 值 | 说明 |\n")
            f.write("|------|-----|------|\n")
            
            # 显示所有 pin 配置
            for i in range(5):
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
            
            f.write(f"| drive_mode | {cfg_param.get('drive_mode', 0)} | 驱动模式 (0=四驱/差速, 1=双电机) |\n")
            f.write(f"| led_polar | {cfg_param.get('led_polar', 0)} | LED极性 (0=低电平亮, 1=高电平亮) |\n")
            f.write(f"| pwm_duty_dual_en | {cfg_param.get('pwm_duty_dual_en', 0)} | 双电机PWM独立使能 |\n")
            
            # 新增的字段（原先在 efuse_dev_info_t，现在移到 CfgParamUnion 尾部）
            f.write(f"| access_code_group | {cfg_param.get('access_code_group', 0)} | 访问码组 |\n")
            f.write(f"| normal_tx_power | {cfg_param.get('normal_tx_power', 0)} | 正常发射功率 |\n")
            
            # 可测试参数
            if is_default_mode:
                # default 模式：显示所有参数的默认值
                for param_name, param_def in PARAM_DEFINITIONS.items():
                    if param_name in cfg_param:
                        param_val = cfg_param[param_name]
                        desc = "-"
                        for v, d in param_def["options"]:
                            if v == param_val:
                                desc = d
                                break
                        f.write(f"| {param_name} | {desc} | {param_def['name']} |\n")
            else:
                # 其他模式：显示所有在 PARAM_DEFINITIONS 中的参数（包括场景设置的和默认的）
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
        
        # 参数说明
        f.write("## 可测试参数说明\n\n")
        for param_name, param_def in PARAM_DEFINITIONS.items():
            f.write(f"### {param_name} - {param_def['name']}\n")
            opts = [f"{v}({d})" for v, d in param_def["options"]]
            f.write(f"可选值: {', '.join(opts)}\n\n")
        
        f.write("## 烧录说明\n\n")
        f.write("- **烧录地址**: 0xF000 (Flash)\n")
        f.write("- **烧录地址**: 0x00000 (EFuse)\n")
        f.write("- **文件大小**: 256 bytes (Flash), 32 bytes (EFuse)\n")
        f.write("- **未写数据**: 0xFF\n\n")
    
    print(f"\nDocumentation generated: {output_file}")


# =============================================================================
# 主函数
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='S10 小车参数组合测试配置生成工具 V2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python %(prog)s --mode single                    # 单参数测试
  python %(prog)s --mode pair                      # 双参数组合
  python %(prog)s --mode scenario                  # 场景测试
  python %(prog)s --mode default                   # 生成三种模式的默认配置
  python %(prog)s --mode all                       # 生成所有模式
        '''
    )
    
    parser.add_argument('--mode', choices=['single', 'pair', 'scenario', 'default', 'all'], 
                       default='all',
                       help='测试模式 (默认: all)')
    parser.add_argument('--output-dir', '-o', default='.',
                       help='输出目录 (默认: 当前目录)')
    parser.add_argument('--modes', nargs='+', 
                       default=['TOY_S10_CAR_MODE_DIFF_4PWM_LED', 
                               'TOY_S10_CAR_MODE_2MOTOR_SPRAY', 
                               'TOY_S10_CAR_MODE_2MOTOR_LED'],
                       help='要生成的模式列表')
    
    args = parser.parse_args()
    
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 70)
    print("S10 小车参数组合测试配置生成工具 V2")
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
