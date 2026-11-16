#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试版 Bin 文件生成工具 (简化版)

只包含关键配置参数：
- efuse_dev_info_t: toy_type
- CfgParamUnion: 完整参数 (包含 access_code_group, normal_tx_power)

用法:
    python bin_gen_test.py --help
    python bin_gen_test.py --target flash --fw-type remote --output config.bin
    python bin_gen_test.py --target efuse --fw-type car10 --output config.bin
"""

import argparse
import struct
import json
import os
import random
from dataclasses import dataclass, asdict
from typing import Optional
from enum import IntEnum


# =============================================================================
# 枚举定义
# =============================================================================

class TargetType(IntEnum):
    FLASH = 0xFF  # Flash 未写数据默认 0xFF
    EFUSE = 0x00  # EFuse 未写数据默认 0x00


class FwType(IntEnum):
    DEFAULT = 0  # 默认值 (FW_TYPE_DEFAULT)
    REMOTE = 1   # SOP10 遥控器 (FW_TYPE_S10_REMOTE)
    CAR10 = 2    # SOP10 遥控车 (FW_TYPE_S10_CAR)
    CAR16 = 3    # SOP16 遥控车 (FW_TYPE_S16_CAR)


# SOP10 Remote 枚举
class SOP10RemotePin(IntEnum):
    KEY_COM = 0b0000
    KEY_F_B = 0b0001
    KEY_L_R = 0b0010
    KEY_K_S = 0b0011
    KEY_P_LED = 0b0100


class SOP10RemoteShutdownVoltage(IntEnum):
    DEFAULT = 0b000
    VOLT_2_0V = 0b001
    VOLT_2_1V = 0b010
    VOLT_2_2V = 0b011
    OFF = 0b100


class SOP10RemoteLowBatteryVoltage(IntEnum):
    DEFAULT = 0b000
    VOLT_2_2V = 0b001
    VOLT_2_3V = 0b010
    VOLT_2_4V = 0b011
    OFF = 0b100


class SOP10RemoteIdleTimeout(IntEnum):
    DEFAULT = 0b000  # 10s
    TIMEOUT_5S = 0b001
    TIMEOUT_10S = 0b010
    TIMEOUT_30S = 0b011
    TIMEOUT_60S = 0b100


class SOP10RemotePairedIndicator(IntEnum):
    DEFAULT = 0b00  # 默认熄灭
    OFF = 0b01      # 配对后指示灯熄灭
    ALWAYS = 0b10   # 配对后指示灯常亮


# SOP10 Car 枚举
class SOP10CarPin(IntEnum):
    PWM_RF = 0b0000
    PWM_RB = 0b0001
    PWM_LF = 0b0010
    PWM_LB = 0b0011
    PWM_SPRAY_LED = 0b0100


class SOP10CarSpecialFunc(IntEnum):
    DEFAULT = 0b00
    SPRAY = 0b01
    LED = 0b10


class SOP10CarShutdownVoltage(IntEnum):
    DEFAULT = 0b0000
    VOLT_2_6V = 0b0001
    VOLT_2_7V = 0b0010
    VOLT_2_8V = 0b0011
    VOLT_2_9V = 0b0100
    VOLT_3_0V = 0b0101
    VOLT_3_1V = 0b0110
    VOLT_3_2V = 0b0111
    OFF = 0b1000


class SOP10CarPwmFreq(IntEnum):
    DEFAULT = 0b000  # 250Hz
    FREQ_250HZ = 0b001
    FREQ_500HZ = 0b010
    FREQ_16KHZ = 0b011


class SOP10CarPwmDutyCycle(IntEnum):
    DEFAULT = 0b000  # 100%
    DUTY_40_PERCENT = 0b001
    DUTY_50_PERCENT = 0b010
    DUTY_60_PERCENT = 0b011
    DUTY_70_PERCENT = 0b100
    DUTY_80_PERCENT = 0b101
    DUTY_90_PERCENT = 0b110
    DUTY_100_PERCENT = 0b111


class SOP10CarDirSwitchDelay(IntEnum):
    DEFAULT = 0b000  # 100ms
    DELAY_0MS = 0b001
    DELAY_50MS = 0b010
    DELAY_100MS = 0b011
    DELAY_150MS = 0b100
    DELAY_200MS = 0b101
    DELAY_250MS = 0b110
    DELAY_300MS = 0b111


class SOP10CarSprayDuration(IntEnum):
    DEFAULT = 0b000  # 30s
    DURATION_30S = 0b001
    DURATION_1MIN = 0b010
    DURATION_2MIN = 0b011
    ALWAYS = 0b100


class SOP10CarSprayPwmDuty(IntEnum):
    DEFAULT = 0b0000  # 40%
    DUTY_7_PERCENT = 0b0001
    DUTY_10_PERCENT = 0b0010
    DUTY_15_PERCENT = 0b0011
    DUTY_20_PERCENT = 0b0100
    DUTY_25_PERCENT = 0b0101
    DUTY_30_PERCENT = 0b0110
    DUTY_35_PERCENT = 0b0111
    DUTY_40_PERCENT = 0b1000


class SOP10CarPairedIndicator(IntEnum):
    DEFAULT = 0b00  # 熄灭
    OFF = 0b01
    ALWAYS = 0b10


class SOP10CarSleepTimeout(IntEnum):
    DEFAULT = 0b000  # 从不休眠
    TIMEOUT_5MIN = 0b001
    TIMEOUT_10MIN = 0b010
    TIMEOUT_15MIN = 0b011
    NEVER = 0b100


# SOP16 Car 枚举
class SOP16CarPin(IntEnum):
    PWM_RF = 0b0000
    PWM_RB = 0b0001
    PWM_LF = 0b0010
    PWM_LB = 0b0011
    PWM_W1 = 0b0100
    PWM_W2 = 0b0101
    PWM_W3 = 0b0110
    PWM_W4 = 0b0111
    GPIO_LED_OUT = 0b1000
    CHG_DETECT = 0b1001
    CHG_FULL = 0b1010


class SOP16CarDemoMode(IntEnum):
    DEFAULT = 0b00
    FIGURE8 = 0b01
    CIRCLE = 0b10
    SQUARE = 0b11


class SOP16CarSpecialFunc(IntEnum):
    DEFAULT = 0b00
    SPRAY = 0b01
    EXCAVATOR = 0b10


class SOP16CarShutdownVoltage(IntEnum):
    DEFAULT = 0b0000
    VOLT_2_6V = 0b0001
    VOLT_2_7V = 0b0010
    VOLT_2_8V = 0b0011
    VOLT_2_9V = 0b0100
    VOLT_3_0V = 0b0101
    VOLT_3_1V = 0b0110
    VOLT_3_2V = 0b0111
    OFF = 0b1000


class SOP16CarPwmFreq(IntEnum):
    DEFAULT = 0b000
    FREQ_250HZ = 0b001
    FREQ_500HZ = 0b010
    FREQ_16KHZ = 0b011


class SOP16CarPwmDutyCycle(IntEnum):
    DEFAULT = 0b000  # 100%
    DUTY_40_PERCENT = 0b001
    DUTY_50_PERCENT = 0b010
    DUTY_60_PERCENT = 0b011
    DUTY_70_PERCENT = 0b100
    DUTY_80_PERCENT = 0b101
    DUTY_90_PERCENT = 0b110
    DUTY_100_PERCENT = 0b111


class SOP16CarDirSwitchDelay(IntEnum):
    DEFAULT = 0b000
    DELAY_0MS = 0b001
    DELAY_50MS = 0b010
    DELAY_100MS = 0b011
    DELAY_150MS = 0b100
    DELAY_200MS = 0b101
    DELAY_250MS = 0b110
    DELAY_300MS = 0b111


class SOP16CarSprayDuration(IntEnum):
    DEFAULT = 0b000
    DURATION_30S = 0b001
    DURATION_1MIN = 0b010
    DURATION_2MIN = 0b011
    ALWAYS = 0b100


class SOP16CarSprayPwmDuty(IntEnum):
    DEFAULT = 0b000
    DUTY_7_PERCENT = 0b001
    DUTY_10_PERCENT = 0b010
    DUTY_15_PERCENT = 0b011
    DUTY_20_PERCENT = 0b100
    DUTY_25_PERCENT = 0b101
    DUTY_30_PERCENT = 0b110
    DUTY_35_PERCENT = 0b111
    DUTY_40_PERCENT = 0b1000


class SOP16CarPairedIndicator(IntEnum):
    DEFAULT = 0b00
    OFF = 0b01
    ALWAYS = 0b10


class SOP16CarSleepTimeout(IntEnum):
    DEFAULT = 0b000
    TIMEOUT_5MIN = 0b001
    TIMEOUT_10MIN = 0b010
    TIMEOUT_15MIN = 0b011
    NEVER = 0b100


# =============================================================================
# 简化版 efuse_dev_info_t (测试版)
# 只包含: toy_type
# =============================================================================

@dataclass
class EfuseDevInfoTest:
    """
    测试版 efuse_dev_info_t (16字节)
    只包含 1 个可配置成员，其他为默认值
    
    可配置:
    - toy_type: 玩具类型
    - chip_id: 可选的 chip_id (6字节列表)，如未设置则随机生成
    """
    # 可配置参数
    toy_type: int = 1              # [2:0] 玩具类型: 1=S10_REMOTE, 2=S10_CAR, 3=S16_CAR
    chip_id: list = None           # 可选的 chip_id (6字节列表)
    
    def pack(self) -> bytes:
        # union_byte_0: [2:0]rsv, [3]paring_type, [7:4]rsv -> 0x00
        union_byte_0 = 0x00
        
        # union_byte_1: [2:0]toy_type, [7:3]rsv
        union_byte_1 = (self.toy_type & 0x07)
        
        # union_byte_2: [3:0]rsv, [6:4]rsv, [7]flash_disable
        union_byte_2 = 0x00
        
        # union_byte_3: [3:0]rsv, [7:4]rsv
        union_byte_3 = 0x00
        
        # union_byte_4: [3:0]ldo_output_trim, [5:4]capless, [7:6]bin_Select -> 0x00
        union_byte_4 = 0x00
        
        # union_byte_5: [5:0]xtal_calibration, [6]xtal_type, [7]dig_ldo_mode -> 0x00
        union_byte_5 = 0x00

        # 使用指定的 chip_id 或随机生成
        if self.chip_id is not None and len(self.chip_id) == 6:
            chip_id = list(self.chip_id)
        else:
            chip_id = [random.randint(0, 255) for _ in range(6)]
        
        data = bytearray(16)
        data[0:6] = chip_id
        data[6] = union_byte_0
        data[7] = union_byte_1
        data[8] = union_byte_2
        data[9] = union_byte_3
        data[10] = union_byte_4
        data[11] = union_byte_5
        data[12:16] = [0x00, 0x00, 0x00, 0x00]  # 默认 adc_calib
        
        return bytes(data)


# =============================================================================
# CfgParamUnion (完整版)
# =============================================================================

@dataclass
class SOP10RemoteCfg:
    """SOP10 遥控器配置参数 (6字节)"""
    pin0: int = SOP10RemotePin.KEY_COM      # 4 bits
    pin1: int = SOP10RemotePin.KEY_F_B      # 4 bits
    pin2: int = SOP10RemotePin.KEY_L_R      # 4 bits
    pin3: int = SOP10RemotePin.KEY_K_S      # 4 bits
    pin4: int = SOP10RemotePin.KEY_P_LED    # 4 bits
    shutdown_volt: int = SOP10RemoteShutdownVoltage.VOLT_2_0V  # 3 bits
    low_batt_volt: int = SOP10RemoteLowBatteryVoltage.VOLT_2_4V  # 3 bits
    sleep_timeout: int = SOP10RemoteIdleTimeout.TIMEOUT_10S  # 3 bits
    paired_indicator: int = SOP10RemotePairedIndicator.DEFAULT  # 2 bits
    reserved0: int = 0                      # 1 bit
    access_code_group: int = 0              # 4 bits
    normal_tx_power: int = 0                # 4 bits
    reserved1: int = 0                      # 8 bits

    def pack(self) -> bytes:
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.shutdown_volt & 0x07) << 20) | \
                ((self.low_batt_volt & 0x07) << 23) | \
                ((self.sleep_timeout & 0x07) << 26) | \
                ((self.paired_indicator & 0x03) << 29) | \
                ((self.reserved0 & 0x01) << 31)
        
        # word1: access_code_group(4) + normal_tx_power(4) + reserved1(8) = 16 bits
        word1 = (self.access_code_group & 0x0F) | \
                ((self.normal_tx_power & 0x0F) << 4)
        
        data = bytearray(6)
        data[0:4] = struct.pack('<I', word0)
        data[4:6] = struct.pack('<H', word1)
        return bytes(data)


@dataclass
class SOP10CarCfg:
    """SOP10 遥控车配置参数 (10字节)"""
    pin0: int = SOP10CarPin.PWM_RF          # 4 bits
    pin1: int = SOP10CarPin.PWM_RB          # 4 bits
    pin2: int = SOP10CarPin.PWM_LF          # 4 bits
    pin3: int = SOP10CarPin.PWM_LB          # 4 bits
    pin4: int = SOP10CarPin.PWM_SPRAY_LED   # 4 bits
    special_func: int = SOP10CarSpecialFunc.SPRAY  # 2 bits
    shutdown_volt: int = SOP10CarShutdownVoltage.OFF  # 4 bits
    reserved0: int = 0                      # 6 bits
    pwm_freq: int = SOP10CarPwmFreq.DEFAULT  # 3 bits
    rev_delay: int = SOP10CarDirSwitchDelay.DEFAULT  # 3 bits
    spray_duration: int = SOP10CarSprayDuration.DEFAULT  # 3 bits
    spray_pwm_duty: int = SOP10CarSprayPwmDuty.DEFAULT  # 4 bits
    paired_indicator: int = SOP10CarPairedIndicator.DEFAULT  # 2 bits
    sleep_timeout: int = SOP10CarSleepTimeout.NEVER  # 3 bits
    reserved00: int = 0                     # 2 bits
    drive_mode: int = 0                     # 1 bit (0:四驱, 1:双电机)
    led_polar: int = 0                      # 1 bit
    pwm_duty_dual_en: int = 0               # 1 bit
    pwm_duty_left: int = 0                  # 3 bits
    pwm_duty_right: int = 0                 # 3 bits
    reserved1: int = 0                      # 3 bits
    # 移动到尾部的字段 (原先在 efuse_dev_info_t)
    access_code_group: int = 0              # 4 bits
    normal_tx_power: int = 0                # 4 bits

    def pack(self) -> bytes:
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.special_func & 0x03) << 20) | \
                ((self.shutdown_volt & 0x0F) << 22) | \
                ((self.reserved0 & 0x3F) << 26)
        
        word1 = (self.pwm_freq & 0x07) | \
                ((self.rev_delay & 0x07) << 3) | \
                ((self.spray_duration & 0x07) << 6) | \
                ((self.spray_pwm_duty & 0x0F) << 9) | \
                ((self.paired_indicator & 0x03) << 13) | \
                ((self.sleep_timeout & 0x07) << 15) | \
                ((self.reserved00 & 0x03) << 18) | \
                ((self.drive_mode & 0x01) << 20) | \
                ((self.led_polar & 0x01) << 21) | \
                ((self.pwm_duty_dual_en & 0x01) << 22) | \
                ((self.pwm_duty_left & 0x07) << 23) | \
                ((self.pwm_duty_right & 0x07) << 26) | \
                ((self.reserved1 & 0x07) << 29)
        
        # word2: access_code_group(4bits) + normal_tx_power(4bits) + reserved2(8bits) = 16bits
        word2 = (self.access_code_group & 0x0F) | \
                ((self.normal_tx_power & 0x0F) << 4)
        
        data = bytearray(10)
        data[0:4] = struct.pack('<I', word0)
        data[4:8] = struct.pack('<I', word1)
        data[8:10] = struct.pack('<H', word2)
        return bytes(data)


@dataclass
class SOP16CarCfg:
    """SOP16 遥控车配置参数 (14字节)"""
    pin0: int = SOP16CarPin.PWM_LF
    pin1: int = SOP16CarPin.PWM_LB
    pin2: int = SOP16CarPin.PWM_RF
    pin3: int = SOP16CarPin.PWM_RB
    pin4: int = SOP16CarPin.PWM_W1
    pin5: int = SOP16CarPin.PWM_W2
    pin6: int = SOP16CarPin.PWM_W3
    pin7: int = SOP16CarPin.PWM_W4
    pin8: int = SOP16CarPin.GPIO_LED_OUT
    pin9: int = SOP16CarPin.CHG_DETECT
    pin10: int = SOP16CarPin.CHG_FULL
    shutdown_volt: int = SOP16CarShutdownVoltage.OFF
    pwm_freq: int = SOP16CarPwmFreq.DEFAULT
    rev_delay: int = SOP16CarDirSwitchDelay.DEFAULT
    spray_duration: int = SOP16CarSprayDuration.DEFAULT
    spray_pwm_duty: int = SOP16CarSprayPwmDuty.DEFAULT
    sleep_timeout: int = SOP16CarSleepTimeout.NEVER
    paired_indicator: int = SOP16CarPairedIndicator.DEFAULT
    demo_mode: int = SOP16CarDemoMode.DEFAULT
    drive_mode: int = 0
    spray_w_mask: int = 0x0F
    special_func: int = SOP16CarSpecialFunc.DEFAULT
    led_polar: int = 0
    reserved0: int = 0
    pwm_duty_dual_en: int = 0
    pwm_duty_left: int = 0
    pwm_duty_right: int = 0
    pwm_duty_w: int = 0
    reserved1: int = 0
    # 移动到尾部的字段 (原先在 efuse_dev_info_t)
    access_code_group: int = 0              # 4 bits
    normal_tx_power: int = 0                # 4 bits

    def pack(self) -> bytes:
        # word0: pin0-pin7 (32 bits)
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.pin5 & 0x0F) << 20) | \
                ((self.pin6 & 0x0F) << 24) | \
                ((self.pin7 & 0x0F) << 28)
        
        # word1: pin8, pin9, pin10, shutdown_volt, pwm_freq, rev_delay, spray_duration, spray_pwm_duty, sleep_timeout (32 bits)
        word1 = (self.pin8 & 0x0F) | \
                ((self.pin9 & 0x0F) << 4) | \
                ((self.pin10 & 0x0F) << 8) | \
                ((self.shutdown_volt & 0x0F) << 12) | \
                ((self.pwm_freq & 0x07) << 16) | \
                ((self.rev_delay & 0x07) << 19) | \
                ((self.spray_duration & 0x07) << 22) | \
                ((self.spray_pwm_duty & 0x0F) << 25) | \
                ((self.sleep_timeout & 0x07) << 29)
        
        # word2: paired_indicator, demo_mode, drive_mode, spray_w_mask, special_func, led_polar, reserved0 (16 bits)
        word2 = (self.paired_indicator & 0x03) | \
                ((self.demo_mode & 0x03) << 2) | \
                ((self.drive_mode & 0x01) << 4) | \
                ((self.spray_w_mask & 0x0F) << 5) | \
                ((self.special_func & 0x03) << 9) | \
                ((self.led_polar & 0x01) << 11) | \
                ((self.reserved0 & 0x0F) << 12)
        
        # word3: pwm_duty_dual_en, pwm_duty_left, pwm_duty_right, pwm_duty_w, access_code_group, reserved1 (16 bits)
        word3 = (self.pwm_duty_dual_en & 0x01) | \
                ((self.pwm_duty_left & 0x07) << 1) | \
                ((self.pwm_duty_right & 0x07) << 4) | \
                ((self.pwm_duty_w & 0x07) << 7) | \
                ((self.access_code_group & 0x0F) << 10) | \
                ((self.reserved1 & 0x03) << 14)
        
        # word4: normal_tx_power, reserved2 (16 bits)
        word4 = (self.normal_tx_power & 0x0F)
        
        data = bytearray(14)
        data[0:4] = struct.pack('<I', word0)
        data[4:8] = struct.pack('<I', word1)
        data[8:10] = struct.pack('<H', word2)
        data[10:12] = struct.pack('<H', word3)
        data[12:14] = struct.pack('<H', word4)
        return bytes(data)


# =============================================================================
# 配置生成器
# =============================================================================

class ConfigGenerator:
    def __init__(self, target: TargetType, fw_type: FwType):
        self.target = target
        self.fw_type = fw_type
        self.efuse_dev_info = EfuseDevInfoTest()
        self.cfg_param = None
        
        # 根据固件类型设置默认 toy_type
        # 注意: FwType 枚举值与 firmware_type_t 定义一致
        # DEFAULT=0, REMOTE=1, CAR10=2, CAR16=3
        if fw_type == FwType.REMOTE:
            self.cfg_param = SOP10RemoteCfg()
            self.efuse_dev_info.toy_type = FwType.REMOTE.value
        elif fw_type == FwType.CAR10:
            self.cfg_param = SOP10CarCfg()
            self.efuse_dev_info.toy_type = FwType.CAR10.value
        elif fw_type == FwType.CAR16:
            self.cfg_param = SOP16CarCfg()
            self.efuse_dev_info.toy_type = FwType.CAR16.value
        else:
            # FwType.DEFAULT (0) 也使用 REMOTE 配置
            self.cfg_param = SOP10RemoteCfg()
            self.efuse_dev_info.toy_type = FwType.DEFAULT.value
    
    def set_efuse_dev_info(self, **kwargs):
        """设置 efuse_dev_info_t 成员 (测试版只支持 toy_type)"""
        for key, value in kwargs.items():
            if hasattr(self.efuse_dev_info, key):
                setattr(self.efuse_dev_info, key, value)
            else:
                print(f"Warning: 测试版不支持该字段: {key} (只支持 toy_type)")
    
    def set_cfg_param(self, **kwargs):
        """设置 CfgParamUnion 成员"""
        if self.cfg_param is None:
            return
        for key, value in kwargs.items():
            if hasattr(self.cfg_param, key):
                setattr(self.cfg_param, key, value)
            else:
                print(f"Warning: Unknown cfg_param field: {key}")
    
    def generate(self, output_file: str) -> str:
        """生成 bin 文件，返回实际文件名（包含地址）"""
        efuse_data = self.efuse_dev_info.pack()
        cfg_data = self.cfg_param.pack()
        
        combined = efuse_data + cfg_data
        
        if self.target == TargetType.FLASH:
            fill_byte = b'\xFF'
            burn_addr = 0xF000
        else:
            fill_byte = b'\x00'
            burn_addr = 0x00
        
        if self.target == TargetType.EFUSE:
            total_size = 32
        else:
            total_size = max(256, len(combined))
        
        if len(combined) < total_size:
            combined += fill_byte * (total_size - len(combined))
        
        # 处理输出文件名
        if output_file:
            base, ext = os.path.splitext(output_file)
            if not ext:
                ext = '.bin'
            actual_file = f"{base}_0x{burn_addr:05x}{ext}"
        else:
            addr_str = f"0x{burn_addr:05x}"
            actual_file = f"config_{addr_str}.bin"
        
        with open(actual_file, 'wb') as f:
            f.write(combined)
        
        print(f"Generated: {actual_file}")
        print(f"  Target: {'Flash' if self.target == TargetType.FLASH else 'EFuse'}")
        print(f"  Burn Address: 0x{burn_addr:05X}")
        print(f"  FW Type: {self.fw_type.name}")
        print(f"  Total size: {len(combined)} bytes")
        print(f"  efuse_dev_info_t: 16 bytes (测试版: 只配置 toy_type)")
        print(f"  CfgParamUnion: {len(cfg_data)} bytes")
        
        return actual_file


# =============================================================================
# JSON 配置管理
# =============================================================================

def create_default_config(fw_type: FwType) -> dict:
    """创建默认配置"""
    base = {
        "efuse_dev_info": {
            "toy_type": 1
        }
    }
    
    if fw_type == FwType.REMOTE:
        base["efuse_dev_info"]["toy_type"] = 1
        base["cfg_param"] = {
            "pin0": SOP10RemotePin.KEY_COM,
            "pin1": SOP10RemotePin.KEY_F_B,
            "pin2": SOP10RemotePin.KEY_L_R,
            "pin3": SOP10RemotePin.KEY_K_S,
            "pin4": SOP10RemotePin.KEY_P_LED,
            "shutdown_volt": SOP10RemoteShutdownVoltage.OFF,
            "low_batt_volt": SOP10RemoteLowBatteryVoltage.OFF,
            "sleep_timeout": SOP10RemoteIdleTimeout.DEFAULT,
            "paired_indicator": SOP10RemotePairedIndicator.DEFAULT,
            "access_code_group": 0,
            "normal_tx_power": 0
        }
    elif fw_type == FwType.CAR10:
        base["efuse_dev_info"]["toy_type"] = 1
        base["cfg_param"] = {
            "pin0": SOP10CarPin.PWM_RF,
            "pin1": SOP10CarPin.PWM_RB,
            "pin2": SOP10CarPin.PWM_LF,
            "pin3": SOP10CarPin.PWM_LB,
            "pin4": SOP10CarPin.PWM_SPRAY_LED,
            "special_func": SOP10CarSpecialFunc.SPRAY,
            "shutdown_volt": SOP10CarShutdownVoltage.OFF,
            "pwm_freq": SOP10CarPwmFreq.DEFAULT,
            "rev_delay": SOP10CarDirSwitchDelay.DEFAULT,
            "spray_duration": SOP10CarSprayDuration.DEFAULT,
            "spray_pwm_duty": SOP10CarSprayPwmDuty.DEFAULT,
            "paired_indicator": SOP10CarPairedIndicator.DEFAULT,
            "sleep_timeout": SOP10CarSleepTimeout.NEVER,
            "drive_mode": 0,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "access_code_group": 0,
            "normal_tx_power": 0
        }
    else:  # CAR16
        base["efuse_dev_info"]["toy_type"] = 2
        base["cfg_param"] = {
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
            "shutdown_volt": SOP16CarShutdownVoltage.OFF,
            "pwm_freq": SOP16CarPwmFreq.DEFAULT,
            "rev_delay": SOP16CarDirSwitchDelay.DEFAULT,
            "spray_duration": SOP16CarSprayDuration.DEFAULT,
            "spray_pwm_duty": SOP16CarSprayPwmDuty.DEFAULT,
            "sleep_timeout": SOP16CarSleepTimeout.NEVER,
            "paired_indicator": SOP16CarPairedIndicator.DEFAULT,
            "demo_mode": SOP16CarDemoMode.DEFAULT,
            "drive_mode": 0,
            "spray_w_mask": 0x0F,
            "special_func": SOP16CarSpecialFunc.DEFAULT,
            "led_polar": 0,
            "pwm_duty_dual_en": 0,
            "pwm_duty_left": 0,
            "pwm_duty_right": 0,
            "pwm_duty_w": 0,
            "access_code_group": 0,
            "normal_tx_power": 0
        }
    
    return base


def convert_enums(obj):
    """转换枚举为值"""
    if isinstance(obj, IntEnum):
        return obj.value
    elif isinstance(obj, list):
        return [convert_enums(i) for i in obj]
    elif isinstance(obj, dict):
        return {k: convert_enums(v) for k, v in obj.items()}
    return obj


def save_json_template(fw_type: FwType, output_file: str):
    """保存 JSON 模板文件"""
    config = create_default_config(fw_type)
    config = convert_enums(config)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"JSON template saved: {output_file}")


def load_json_config(json_file: str, generator: ConfigGenerator):
    """从 JSON 文件加载配置"""
    with open(json_file, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    if 'efuse_dev_info' in config:
        generator.set_efuse_dev_info(**config['efuse_dev_info'])
    if 'cfg_param' in config:
        generator.set_cfg_param(**config['cfg_param'])


# =============================================================================
# 主函数
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='测试版 Bin 文件生成工具 (简化版 efuse_dev_info_t)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  # 生成默认配置到 Flash
  python bin_gen_test.py --target flash --fw-type remote --output config.bin
  
  # 生成默认配置到 EFuse
  python bin_gen_test.py --target efuse --fw-type car10 --output config.bin
  
  # 从 JSON 配置文件生成
  python bin_gen_test.py --target flash --fw-type car16 --json config.json --output config.bin
  
  # 生成 JSON 模板
  python bin_gen_test.py --template --fw-type remote --output template.json

测试版说明:
  - efuse_dev_info_t: 只支持 toy_type
  - CfgParamUnion: 完整支持所有参数 (包含 access_code_group, normal_tx_power)
        '''
    )
    
    parser.add_argument('--target', choices=['flash', 'efuse'],
                        help='目标烧录位置 (flash: 0xF000, efuse: 0x00)')
    parser.add_argument('--fw-type', choices=['remote', 'car10', 'car16'], required=True,
                        help='固件类型 (remote: SOP10遥控器, car10: SOP10遥控车, car16: SOP16遥控车)')
    parser.add_argument('--output', '-o', type=str, default='config.bin',
                        help='输出 bin 文件名 (默认: config.bin)')
    parser.add_argument('--json', '-j', type=str,
                        help='输入 JSON 配置文件')
    parser.add_argument('--template', action='store_true',
                        help='生成 JSON 模板文件')
    
    args = parser.parse_args()
    
    # 生成模板
    if args.template:
        fw_type_map = {'remote': FwType.REMOTE, 'car10': FwType.CAR10, 'car16': FwType.CAR16}
        fw_type = fw_type_map[args.fw_type]
        save_json_template(fw_type, args.output)
        return
    
    # 检查 target 参数
    if args.target is None:
        parser.error('--target is required when not using --template')
    
    # 映射参数
    target = TargetType.FLASH if args.target == 'flash' else TargetType.EFUSE
    fw_type_map = {'remote': FwType.REMOTE, 'car10': FwType.CAR10, 'car16': FwType.CAR16}
    fw_type = fw_type_map[args.fw_type]
    
    # 创建生成器
    generator = ConfigGenerator(target, fw_type)
    
    # 加载 JSON 配置
    if args.json:
        if not os.path.exists(args.json):
            print(f"Error: JSON file not found: {args.json}")
            return
        load_json_config(args.json, generator)
    
    # 生成 bin 文件
    actual_file = generator.generate(args.output)
    
    # 打印详细信息
    print("\n=== efuse_dev_info_t (测试版 - 简化配置) ===")
    print(f"  toy_type: {generator.efuse_dev_info.toy_type}")
    
    print("\n=== CfgParamUnion (完整配置) ===")
    print(f"  access_code_group: {generator.cfg_param.access_code_group}")
    print(f"  normal_tx_power: {generator.cfg_param.normal_tx_power}")
    for key, value in vars(generator.cfg_param).items():
        if isinstance(value, int):
            print(f"  {key}: {value}")


if __name__ == '__main__':
    main()
