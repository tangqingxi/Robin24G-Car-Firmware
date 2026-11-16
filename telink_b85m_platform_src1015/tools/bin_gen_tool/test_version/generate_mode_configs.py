#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
根据 TOY_S10_CAR_DEFAULT_MODE 和 TOY_S16_CAR_DEFAULT_MODE 宏生成对应的 JSON 和 bin 文件

S10 小车 3 种模式:
  - TOY_S10_CAR_MODE_DIFF_4PWM_LED   (四驱/差速 + 4路PWM + LED)
  - TOY_S10_CAR_MODE_2MOTOR_SPRAY    (双电机 + 2路PWM + 喷雾)
  - TOY_S10_CAR_MODE_2MOTOR_LED      (双电机 + 2路PWM + LED)

S16 大车 3 种模式:
  - TOY_S16_CAR_MODE_DIFF_SPRAY      (差速 + 喷雾)
  - TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR (双电机 + 挖掘机)
  - TOY_S16_CAR_MODE_2MOTOR_SPRAY    (双电机 + 喷雾)
"""

import json
import os
import sys

# 将当前目录加入路径，以便导入 bin_gen_test
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bin_gen_test import (
    ConfigGenerator, TargetType, FwType,
    SOP10RemotePin, SOP10RemoteShutdownVoltage, SOP10RemoteLowBatteryVoltage,
    SOP10RemoteIdleTimeout,
    SOP10CarPin, SOP10CarSpecialFunc, SOP10CarShutdownVoltage,
    SOP10CarPwmFreq, SOP10CarDirSwitchDelay, SOP10CarSprayDuration,
    SOP10CarSprayPwmDuty, SOP10CarPairedIndicator, SOP10CarSleepTimeout,
    SOP16CarPin, SOP16CarSpecialFunc, SOP16CarShutdownVoltage,
    SOP16CarPwmFreq, SOP16CarDirSwitchDelay, SOP16CarSprayDuration,
    SOP16CarSprayPwmDuty, SOP16CarPairedIndicator, SOP16CarSleepTimeout,
    SOP16CarDemoMode
)


def create_s10_remote_default():
    """S10 遥控器默认配置"""
    return {
        "name": "SOP10_REMOTE_DEFAULT",
        "efuse_dev_info": {
            "toy_type": 1,
            "access_code_group": 0,
            "normal_tx_power": 15
        },
        "cfg_param": {
            "pin0": SOP10RemotePin.KEY_COM,
            "pin1": SOP10RemotePin.KEY_F_B,
            "pin2": SOP10RemotePin.KEY_L_R,
            "pin3": SOP10RemotePin.KEY_K_S,
            "pin4": SOP10RemotePin.KEY_P_LED,
            "shutdown_volt": SOP10RemoteShutdownVoltage.OFF,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.OFF,
            "sleep_timeout": SOP10RemoteIdleTimeout.DEFAULT,
            "led_polar": 0,
            "reserved0": 0,
            "reserved1": 0
        }
    }


def create_s10_mode_diff_4pwm_led():
    """S10 模式1: 四驱/差速 + 4路PWM + LED"""
    return {
        "name": "TOY_S10_CAR_MODE_DIFF_4PWM_LED",
        "efuse_dev_info": {
            "toy_type": 2,
            "access_code_group": 0,
            "normal_tx_power": 15
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
            "pwm_duty_right": 0
        }
    }


def create_s10_mode_2motor_spray():
    """S10 模式2: 双电机 + 2路PWM + 喷雾"""
    return {
        "name": "TOY_S10_CAR_MODE_2MOTOR_SPRAY",
        "efuse_dev_info": {
            "toy_type": 2,
            "access_code_group": 0,
            "normal_tx_power": 15
        },
        "cfg_param": {
            "pin0": SOP10CarPin.PWM_RF,
            "pin1": SOP10CarPin.PWM_RB,
            "pin2": SOP10CarPin.PWM_SPRAY_LED,
            "pin3": SOP10CarPin.PWM_LF,
            "pin4": SOP10CarPin.PWM_LB,
            "special_func": SOP10CarSpecialFunc.SPRAY,
            "shutdown_volt": SOP10CarShutdownVoltage.OFF,
            "pwm_freq": SOP10CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP10CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP10CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP10CarPairedIndicator.OFF,
            "sleep_timeout": SOP10CarSleepTimeout.NEVER,
            "drive_mode": 1,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0
        }
    }


def create_s10_mode_2motor_led():
    """S10 模式3: 双电机 + 2路PWM + LED"""
    return {
        "name": "TOY_S10_CAR_MODE_2MOTOR_LED",
        "efuse_dev_info": {
            "toy_type": 2,
            "access_code_group": 0,
            "normal_tx_power": 15
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
            "drive_mode": 1,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0
        }
    }


def create_s16_mode_diff_spray():
    """S16 模式1: 差速 + 喷雾"""
    return {
        "name": "TOY_S16_CAR_MODE_DIFF_SPRAY",
        "efuse_dev_info": {
            "toy_type": 3,
            "access_code_group": 0,
            "normal_tx_power": 15
        },
        "cfg_param": {
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
            "demo_mode": SOP16CarDemoMode.FIGURE8,
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_freq": SOP16CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP16CarPairedIndicator.OFF,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "drive_mode": 0,
            "spray_w_mask": 0x03,
            "special_func": SOP16CarSpecialFunc.SPRAY,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "pwm_duty_w": 0
        }
    }


def create_s16_mode_2motor_excavator():
    """S16 模式2: 双电机 + 挖掘机"""
    return {
        "name": "TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR",
        "efuse_dev_info": {
            "toy_type": 3,
            "access_code_group": 0,
            "normal_tx_power": 15
        },
        "cfg_param": {
            "pin0": SOP16CarPin.PWM_RF,
            "pin1": SOP16CarPin.PWM_RB,
            "pin2": SOP16CarPin.PWM_W1,
            "pin3": SOP16CarPin.GPIO_LED_OUT,
            "pin4": SOP16CarPin.PWM_LF,
            "pin5": SOP16CarPin.PWM_LB,
            "pin6": SOP16CarPin.CHG_DETECT,
            "pin7": SOP16CarPin.CHG_FULL,
            "pin8": SOP16CarPin.PWM_W2,
            "pin9": SOP16CarPin.PWM_W3,
            "pin10": SOP16CarPin.PWM_W4,
            "demo_mode": SOP16CarDemoMode.FIGURE8,
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_freq": SOP16CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP16CarPairedIndicator.OFF,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "drive_mode": 1,
            "spray_w_mask": 0x00,
            "special_func": SOP16CarSpecialFunc.EXCAVATOR,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "pwm_duty_w": 0
        }
    }


def create_s16_mode_2motor_spray():
    """S16 模式3: 双电机 + 喷雾"""
    return {
        "name": "TOY_S16_CAR_MODE_2MOTOR_SPRAY",
        "efuse_dev_info": {
            "toy_type": 3,
            "access_code_group": 0,
            "normal_tx_power": 15
        },
        "cfg_param": {
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
            "demo_mode": SOP16CarDemoMode.FIGURE8,
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_freq": SOP16CarPwmFreq.FREQ_250HZ,
            "rev_delay": SOP16CarDirSwitchDelay.DELAY_100MS,
            "spray_duration": SOP16CarSprayDuration.DURATION_30S,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DUTY_40_PERCENT,
            "paired_indicator": SOP16CarPairedIndicator.OFF,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "drive_mode": 1,
            "spray_w_mask": 0x0F,
            "special_func": SOP16CarSpecialFunc.SPRAY,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "pwm_duty_w": 0
        }
    }


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


def generate_mode_files(mode_config, target, output_dir="."):
    """生成指定模式的 JSON 和 bin 文件"""
    mode_name = mode_config["name"]
    
    # 创建生成器
    if "REMOTE" in mode_name:
        fw_type = FwType.REMOTE
    elif "S10" in mode_name:
        fw_type = FwType.CAR10
    else:
        fw_type = FwType.CAR16
    generator = ConfigGenerator(target, fw_type)
    
    # 设置配置
    generator.set_efuse_dev_info(**mode_config["efuse_dev_info"])
    generator.set_cfg_param(**mode_config["cfg_param"])
    
    # 生成文件名
    target_str = "flash" if target == TargetType.FLASH else "efuse"
    addr_str = "0x0f000" if target == TargetType.FLASH else "0x00000"
    
    json_file = os.path.join(output_dir, f"{mode_name}_{target_str}.json")
    
    # 生成 bin
    bin_name = f"{mode_name}_{target_str}.bin"
    bin_file = generator.generate(bin_name)
    
    # 从生成的 bin 文件中读取 chip_id
    with open(bin_file, 'rb') as f:
        bin_data = f.read()
        chip_id = list(bin_data[0:6])
    
    # 更新 JSON 数据，添加 chip_id
    json_data = convert_enums(mode_config)
    json_data["efuse_dev_info"]["chip_id"] = chip_id
    json_data["efuse_dev_info"]["chip_id_hex"] = "".join(f"{b:02x}" for b in chip_id)
    
    # 保存 JSON
    with open(json_file, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent=2, ensure_ascii=False)
    print(f"  JSON: {json_file}")
    
    # 重命名为带宏名的文件
    final_bin_name = f"{mode_name}_{addr_str}.bin"
    final_bin_path = os.path.join(output_dir, final_bin_name)
    if os.path.exists(bin_file):
        os.rename(bin_file, final_bin_path)
        print(f"  BIN:  {final_bin_path}")


def main():
    """主函数"""
    output_dir = "mode_configs"
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 60)
    print("生成 SOP10_REMOTE 和 TOY_S10/S16_CAR_DEFAULT_MODE 配置")
    print("=" * 60)
    
    # S10 遥控器
    remote_modes = [
        create_s10_remote_default()
    ]
    
    # S10 小车 3 种模式
    s10_modes = [
        create_s10_mode_diff_4pwm_led(),
        create_s10_mode_2motor_spray(),
        create_s10_mode_2motor_led()
    ]
    
    # S16 大车 3 种模式
    s16_modes = [
        create_s16_mode_diff_spray(),
        create_s16_mode_2motor_excavator(),
        create_s16_mode_2motor_spray()
    ]
    
    all_modes = remote_modes + s10_modes + s16_modes
    
    print(f"\n共 {len(all_modes)} 种模式配置需要生成")
    print(f"输出目录: {output_dir}/")
    print()
    
    # 为每种模式生成 Flash 和 EFuse 配置
    for mode in all_modes:
        mode_name = mode["name"]
        print(f"\n【{mode_name}】")
        
        # Flash 配置
        print(f"  -> Flash (烧录地址 0xF000):")
        generate_mode_files(mode, TargetType.FLASH, output_dir)
        
        # EFuse 配置
        print(f"  -> EFuse (烧录地址 0x00):")
        generate_mode_files(mode, TargetType.EFUSE, output_dir)
    
    print("\n" + "=" * 60)
    print("生成完成!")
    print("=" * 60)
    print(f"\n输出文件列表:")
    for f in sorted(os.listdir(output_dir)):
        file_path = os.path.join(output_dir, f)
        size = os.path.getsize(file_path)
        print(f"  {f:<60} ({size:>4} bytes)")


if __name__ == '__main__':
    main()
