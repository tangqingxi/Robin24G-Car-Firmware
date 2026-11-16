#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 efuse_dev_info_t 和 CfgParamUnion 配置数据的 bin 文件工具

支持生成用于烧录到:
- Flash (未写数据默认 0xFF)
- EFuse (未写数据默认 0x00)

用法:
    python bin_gen.py --help
    python bin_gen.py --target flash --fw-type remote --output config.bin
    python bin_gen.py --target efuse --fw-type car10 --output config.bin
"""

import argparse
import struct
import json
import os
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
    REMOTE = 0  # SOP10 遥控器
    CAR10 = 1   # SOP10 遥控车
    CAR16 = 2   # SOP16 遥控车


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
    BLINK = 0b10


class SOP16CarSleepTimeout(IntEnum):
    DEFAULT = 0b000
    TIMEOUT_5MIN = 0b001
    TIMEOUT_10MIN = 0b010
    TIMEOUT_15MIN = 0b011
    NEVER = 0b100


# =============================================================================
# 数据类定义
# =============================================================================

@dataclass
class ADCCalib:
    """ADC 校准值 (4字节)"""
    gpio_gain: int = 0   # u8
    gpio_offset: int = 0  # u8
    vbat_gain: int = 0   # u8
    vbat_offset: int = 0  # u8

    def pack(self) -> bytes:
        return struct.pack('<BBBB', self.gpio_gain, self.gpio_offset, 
                          self.vbat_gain, self.vbat_offset)


@dataclass
class EfuseDevInfo:
    """
    efuse_dev_info_t 结构体 (16字节)
    
    typedef struct {
        u8 chip_id[6];          // 0x00-0x05: Chip ID
        u8 union_byte_0;        // 0x06: [2:0]rsv, [3]paring_type, [7:4]rsv
        u8 union_byte_1;        // 0x07: [2:0]toy_type, [7:3]rsv
        u8 union_byte_2;        // 0x08: [3:0]access_code_group, [6:4]rsv, [7]flash_disable
        u8 union_byte_3;        // 0x09: [3:0]rsv, [7:4]normal_tx_power
        u8 union_byte_4;        // 0x0A: [3:0]ldo_output_trim, [5:4]capless, [7:6]bin_Select
        u8 union_byte_5;        // 0x0B: [5:0]xtal_calibration, [6]xtal_type, [7]dig_ldo_mode
        adc_calib_t adc_calib;  // 0x0C-0x0F: ADC校准值
    } efuse_dev_info_t;
    """
    chip_id: list = None           # 6 bytes
    rsv_bit_00: int = 0            # 3 bits
    paring_type: int = 0           # 1 bit (0:auto pair, 1:manual pair)
    rsv_bit_01: int = 0            # 4 bits
    toy_type: int = 0              # 3 bits
    rsv_bit_2: int = 0             # 5 bits
    access_code_group: int = 0     # 4 bits
    rsv_bit_u2: int = 0            # 3 bits
    flash_disable: int = 0         # 1 bit
    rsv_bit_u3: int = 0            # 4 bits
    normal_tx_power: int = 0       # 4 bits
    ldo_output_trim: int = 0       # 4 bits
    capless: int = 0               # 2 bits
    bin_select: int = 0            # 2 bits
    xtal_calibration: int = 0      # 6 bits
    xtal_type: int = 0             # 1 bit
    dig_ldo_mode: int = 0          # 1 bit
    adc_calib: ADCCalib = None     # 4 bytes

    def __post_init__(self):
        if self.chip_id is None:
            self.chip_id = [0x12, 0x34, 0x56, 0x00, 0x00, 0x00]
        if self.adc_calib is None:
            self.adc_calib = ADCCalib()

    def pack(self) -> bytes:
        # union_byte_0: [2:0]rsv, [3]paring_type, [7:4]rsv
        union_byte_0 = (self.rsv_bit_00 & 0x07) | \
                       ((self.paring_type & 0x01) << 3) | \
                       ((self.rsv_bit_01 & 0x0F) << 4)
        
        # union_byte_1: [2:0]toy_type, [7:3]rsv
        union_byte_1 = (self.toy_type & 0x07) | \
                       ((self.rsv_bit_2 & 0x1F) << 3)
        
        # union_byte_2: [3:0]access_code_group, [6:4]rsv, [7]flash_disable
        union_byte_2 = (self.access_code_group & 0x0F) | \
                       ((self.rsv_bit_u2 & 0x07) << 4) | \
                       ((self.flash_disable & 0x01) << 7)
        
        # union_byte_3: [3:0]rsv, [7:4]normal_tx_power
        union_byte_3 = (self.rsv_bit_u3 & 0x0F) | \
                       ((self.normal_tx_power & 0x0F) << 4)
        
        # union_byte_4: [3:0]ldo_output_trim, [5:4]capless, [7:6]bin_Select
        union_byte_4 = (self.ldo_output_trim & 0x0F) | \
                       ((self.capless & 0x03) << 4) | \
                       ((self.bin_select & 0x03) << 6)
        
        # union_byte_5: [5:0]xtal_calibration, [6]xtal_type, [7]dig_ldo_mode
        union_byte_5 = (self.xtal_calibration & 0x3F) | \
                       ((self.xtal_type & 0x01) << 6) | \
                       ((self.dig_ldo_mode & 0x01) << 7)

        data = bytearray(16)
        data[0:6] = self.chip_id[0:6]
        data[6] = union_byte_0
        data[7] = union_byte_1
        data[8] = union_byte_2
        data[9] = union_byte_3
        data[10] = union_byte_4
        data[11] = union_byte_5
        data[12:16] = self.adc_calib.pack()
        
        return bytes(data)


@dataclass
class SOP10RemoteCfg:
    """
    SOP10 遥控器配置参数 (6字节)
    """
    pin0: int = SOP10RemotePin.KEY_COM      # 4 bits
    pin1: int = SOP10RemotePin.KEY_F_B      # 4 bits
    pin2: int = SOP10RemotePin.KEY_L_R      # 4 bits
    pin3: int = SOP10RemotePin.KEY_K_S      # 4 bits
    pin4: int = SOP10RemotePin.KEY_P_LED    # 4 bits
    shutdown_volt: int = SOP10RemoteShutdownVoltage.OFF  # 3 bits
    low_batt_volt: int = SOP10RemoteLowBatteryVoltage.OFF  # 3 bits
    sleep_timeout: int = SOP10RemoteIdleTimeout.DEFAULT  # 3 bits
    led_polar: int = 0                      # 1 bit (0:低电平亮, 1:高电平亮)
    reserved0: int = 3                      # 3 bits

    def pack(self) -> bytes:
        # 32 bits word 0
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.shutdown_volt & 0x07) << 20) | \
                ((self.low_batt_volt & 0x07) << 23) | \
                ((self.sleep_timeout & 0x07) << 26) | \
                ((self.reserved0 & 0x07) << 29)
        
        data = bytearray(4) # Changed from 6 to 4 because reserved1 (16 bits) is removed
        data[0:4] = struct.pack('<I', word0)
        return bytes(data)


@dataclass
class SOP10CarCfg:
    """
    SOP10 遥控车配置参数 (8字节)
    """
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

    def pack(self) -> bytes:
        # 32 bits word 0
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.special_func & 0x03) << 20) | \
                ((self.shutdown_volt & 0x0F) << 22) | \
                ((self.reserved0 & 0x3F) << 26)
        
        # 32 bits word 1
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
        
        data = bytearray(8)
        data[0:4] = struct.pack('<I', word0)
        data[4:8] = struct.pack('<I', word1)
        return bytes(data)


@dataclass
class SOP16CarCfg:
    """
    SOP16 遥控车配置参数 (12字节，实际使用80bit)
    """
    pin0: int = SOP16CarPin.PWM_RF
    pin1: int = SOP16CarPin.PWM_RB
    pin2: int = SOP16CarPin.PWM_LF
    pin3: int = SOP16CarPin.PWM_LB
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

    def pack(self) -> bytes:
        # 64 bits word 0
        word0 = (self.pin0 & 0x0F) | \
                ((self.pin1 & 0x0F) << 4) | \
                ((self.pin2 & 0x0F) << 8) | \
                ((self.pin3 & 0x0F) << 12) | \
                ((self.pin4 & 0x0F) << 16) | \
                ((self.pin5 & 0x0F) << 20) | \
                ((self.pin6 & 0x0F) << 24) | \
                ((self.pin7 & 0x0F) << 28) | \
                ((self.pin8 & 0x0F) << 32) | \
                ((self.pin9 & 0x0F) << 36) | \
                ((self.pin10 & 0x0F) << 40) | \
                ((self.shutdown_volt & 0x0F) << 44) | \
                ((self.pwm_freq & 0x07) << 48) | \
                ((self.rev_delay & 0x07) << 51) | \
                ((self.spray_duration & 0x07) << 54) | \
                ((self.spray_pwm_duty & 0x0F) << 57) | \
                ((self.sleep_timeout & 0x07) << 61)
        
        # 16 bits word 2 (lower part)
        word2_low = ((self.sleep_timeout & 0x07) >> 3) | \
                    ((self.paired_indicator & 0x03) << 1) | \
                    ((self.demo_mode & 0x03) << 3) | \
                    ((self.drive_mode & 0x01) << 5) | \
                    ((self.spray_w_mask & 0x0F) << 6) | \
                    ((self.special_func & 0x03) << 10) | \
                    ((self.led_polar & 0x01) << 12) | \
                    ((self.reserved0 & 0x0F) << 13)
        
        # 16 bits word 3
        word3 = (self.pwm_duty_dual_en & 0x01) | \
                ((self.pwm_duty_left & 0x07) << 1) | \
                ((self.pwm_duty_right & 0x07) << 4) | \
                ((self.pwm_duty_w & 0x07) << 7) | \
                ((self.reserved1 & 0x3F) << 10)
        
        data = bytearray(12)
        data[0:8] = struct.pack('<Q', word0 & 0xFFFFFFFFFFFFFFFF)
        data[8:10] = struct.pack('<H', word2_low)
        data[10:12] = struct.pack('<H', word3)
        return bytes(data)


# =============================================================================
# 配置管理
# =============================================================================

class ConfigGenerator:
    def __init__(self, target: TargetType, fw_type: FwType):
        self.target = target
        self.fw_type = fw_type
        self.efuse_dev_info = EfuseDevInfo()
        self.cfg_param = None
        
        if fw_type == FwType.REMOTE:
            self.cfg_param = SOP10RemoteCfg()
        elif fw_type == FwType.CAR10:
            self.cfg_param = SOP10CarCfg()
        elif fw_type == FwType.CAR16:
            self.cfg_param = SOP16CarCfg()
    
    def set_efuse_dev_info(self, **kwargs):
        """设置 efuse_dev_info_t 成员"""
        for key, value in kwargs.items():
            if hasattr(self.efuse_dev_info, key):
                setattr(self.efuse_dev_info, key, value)
            elif hasattr(self.efuse_dev_info.adc_calib, key):
                setattr(self.efuse_dev_info.adc_calib, key, value)
            else:
                print(f"Warning: Unknown efuse_dev_info field: {key}")
    
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
        # efuse_dev_info_t: 16 bytes
        efuse_data = self.efuse_dev_info.pack()
        
        # CfgParamUnion
        cfg_data = self.cfg_param.pack()
        
        # 合并数据
        # 布局: [efuse_dev_info_t (16 bytes)] [CfgParamUnion (根据类型)]
        combined = efuse_data + cfg_data
        
        # 根据目标类型填充
        if self.target == TargetType.FLASH:
            # Flash: 未写数据默认 0xFF
            fill_byte = b'\xFF'
            # Flash 烧录地址: 0xF000
            burn_addr = 0xF000
        else:
            # EFuse: 未写数据默认 0x00
            fill_byte = b'\x00'
            # EFuse 烧录地址: 0x00
            burn_addr = 0x00
        
        # 确定总大小 (Flash/EFuse 通常有固定大小)
        if self.target == TargetType.EFUSE:
            # EFuse 通常 32 bytes (TC122X)
            total_size = 32
        else:
            # Flash sector 通常 256 bytes 或 4K
            total_size = max(256, len(combined))
        
        # 填充到总大小
        if len(combined) < total_size:
            combined += fill_byte * (total_size - len(combined))
        
        # 处理输出文件名：添加烧录地址
        if output_file:
            # 分离文件名和扩展名
            base, ext = os.path.splitext(output_file)
            # 如果扩展名为空，使用 .bin
            if not ext:
                ext = '.bin'
            # 生成带地址的文件名: name_0x0f000.bin
            actual_file = f"{base}_0x{burn_addr:05x}{ext}"
        else:
            # 默认文件名
            addr_str = f"0x{burn_addr:05x}"
            actual_file = f"config_{addr_str}.bin"
        
        with open(actual_file, 'wb') as f:
            f.write(combined)
        
        print(f"Generated: {actual_file}")
        print(f"  Target: {'Flash' if self.target == TargetType.FLASH else 'EFuse'}")
        print(f"  Burn Address: 0x{burn_addr:05X}")
        print(f"  FW Type: {self.fw_type.name}")
        print(f"  Total size: {len(combined)} bytes")
        print(f"  efuse_dev_info_t: 16 bytes")
        print(f"  CfgParamUnion: {len(cfg_data)} bytes")
        
        return actual_file
        
        return combined


def create_default_config(fw_type: FwType) -> dict:
    """创建默认配置"""
    if fw_type == FwType.REMOTE:
        return {
            'efuse_dev_info': {
                'chip_id': [0x12, 0x34, 0x56, 0x00, 0x00, 0x00],
                'paring_type': 0,
                'toy_type': 0,
                'access_code_group': 0,
                'flash_disable': 0,
                'normal_tx_power': 0,
                'ldo_output_trim': 0,
                'capless': 0,
                'bin_select': 0,
                'xtal_calibration': 0,
                'xtal_type': 0,
                'dig_ldo_mode': 0,
                'adc_calib': {'gpio_gain': 0, 'gpio_offset': 0, 'vbat_gain': 0, 'vbat_offset': 0}
            },
            'cfg_param': {
                'pin0': SOP10RemotePin.KEY_COM,
                'pin1': SOP10RemotePin.KEY_F_B,
                'pin2': SOP10RemotePin.KEY_L_R,
                'pin3': SOP10RemotePin.KEY_K_S,
                'pin4': SOP10RemotePin.KEY_P_LED,
                'shutdown_volt': SOP10RemoteShutdownVoltage.OFF,
                'low_batt_volt': SOP10RemoteLowBatteryVoltage.OFF,
                'sleep_timeout': SOP10RemoteIdleTimeout.DEFAULT,
            } # Removed led_polar
        }
    elif fw_type == FwType.CAR10:
        return {
            'efuse_dev_info': {
                'chip_id': [0x12, 0x34, 0x56, 0x00, 0x00, 0x00],
                'paring_type': 0,
                'toy_type': 1,
                'access_code_group': 0,
                'flash_disable': 0,
                'normal_tx_power': 0,
                'ldo_output_trim': 0,
                'capless': 0,
                'bin_select': 0,
                'xtal_calibration': 0,
                'xtal_type': 0,
                'dig_ldo_mode': 0,
                'adc_calib': {'gpio_gain': 0, 'gpio_offset': 0, 'vbat_gain': 0, 'vbat_offset': 0}
            },
            'cfg_param': {
                'pin0': SOP10CarPin.PWM_RF,
                'pin1': SOP10CarPin.PWM_RB,
                'pin2': SOP10CarPin.PWM_LF,
                'pin3': SOP10CarPin.PWM_LB,
                'pin4': SOP10CarPin.PWM_SPRAY_LED,
                'special_func': SOP10CarSpecialFunc.SPRAY,
                'shutdown_volt': SOP10CarShutdownVoltage.OFF,
                'pwm_freq': SOP10CarPwmFreq.DEFAULT,
                'rev_delay': SOP10CarDirSwitchDelay.DEFAULT,
                'spray_duration': SOP10CarSprayDuration.DEFAULT,
                'spray_pwm_duty': SOP10CarSprayPwmDuty.DEFAULT,
                'paired_indicator': SOP10CarPairedIndicator.DEFAULT,
                'sleep_timeout': SOP10CarSleepTimeout.NEVER,
                'drive_mode': 0,
                'led_polar': 0,
                'pwm_duty_dual_en': 0,
                'pwm_duty_left': 0,
                'pwm_duty_right': 0
            }
        }
    else:  # CAR16
        return {
            'efuse_dev_info': {
                'chip_id': [0x12, 0x34, 0x56, 0x00, 0x00, 0x00],
                'paring_type': 0,
                'toy_type': 2,
                'access_code_group': 0,
                'flash_disable': 0,
                'normal_tx_power': 0,
                'ldo_output_trim': 0,
                'capless': 0,
                'bin_select': 0,
                'xtal_calibration': 0,
                'xtal_type': 0,
                'dig_ldo_mode': 0,
                'adc_calib': {'gpio_gain': 0, 'gpio_offset': 0, 'vbat_gain': 0, 'vbat_offset': 0}
            },
            'cfg_param': {
                'pin0': SOP16CarPin.PWM_RF,
                'pin1': SOP16CarPin.PWM_RB,
                'pin2': SOP16CarPin.PWM_LF,
                'pin3': SOP16CarPin.PWM_LB,
                'pin4': SOP16CarPin.PWM_W1,
                'pin5': SOP16CarPin.PWM_W2,
                'pin6': SOP16CarPin.PWM_W3,
                'pin7': SOP16CarPin.PWM_W4,
                'pin8': SOP16CarPin.GPIO_LED_OUT,
                'pin9': SOP16CarPin.CHG_DETECT,
                'pin10': SOP16CarPin.CHG_FULL,
                'shutdown_volt': SOP16CarShutdownVoltage.OFF,
                'pwm_freq': SOP16CarPwmFreq.DEFAULT,
                'rev_delay': SOP16CarDirSwitchDelay.DEFAULT,
                'spray_duration': SOP16CarSprayDuration.DEFAULT,
                'spray_pwm_duty': SOP16CarSprayPwmDuty.DEFAULT,
                'sleep_timeout': SOP16CarSleepTimeout.NEVER,
                'paired_indicator': SOP16CarPairedIndicator.DEFAULT,
                'demo_mode': SOP16CarDemoMode.DEFAULT,
                'drive_mode': 0,
                'spray_w_mask': 0x0F,
                'special_func': SOP16CarSpecialFunc.DEFAULT,
                'led_polar': 0,
                'pwm_duty_dual_en': 0,
                'pwm_duty_left': 0,
                'pwm_duty_right': 0,
                'pwm_duty_w': 0
            }
        }


def save_json_template(fw_type: FwType, output_file: str):
    """保存 JSON 模板文件"""
    config = create_default_config(fw_type)
    
    # 转换为可序列化的格式
    def convert_enum(obj):
        if isinstance(obj, IntEnum):
            return obj.value
        elif isinstance(obj, list):
            return obj
        elif isinstance(obj, dict):
            return {k: convert_enum(v) for k, v in obj.items()}
        return obj
    
    config = convert_enum(config)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print(f"JSON template saved: {output_file}")


def load_json_config(json_file: str, generator: ConfigGenerator):
    """从 JSON 文件加载配置"""
    with open(json_file, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    if 'efuse_dev_info' in config:
        efuse_info = config['efuse_dev_info'].copy()
        # 处理 adc_calib 嵌套对象
        if 'adc_calib' in efuse_info:
            adc_calib_data = efuse_info.pop('adc_calib')
            generator.efuse_dev_info.adc_calib = ADCCalib(**adc_calib_data)
        generator.set_efuse_dev_info(**efuse_info)
    if 'cfg_param' in config:
        generator.set_cfg_param(**config['cfg_param'])


def main():
    parser = argparse.ArgumentParser(
        description='生成 efuse_dev_info_t 和 CfgParamUnion 的 bin 文件工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  # 生成默认配置到 Flash
  python bin_gen.py --target flash --fw-type remote --output config.bin
  
  # 生成默认配置到 EFuse
  python bin_gen.py --target efuse --fw-type car10 --output config.bin
  
  # 从 JSON 配置文件生成
  python bin_gen.py --target flash --fw-type car16 --json config.json --output config.bin
  
  # 生成 JSON 模板
  python bin_gen.py --template --fw-type remote --output config_template.json
        '''
    )
    
    parser.add_argument('--target', choices=['flash', 'efuse'],
                        help='目标烧录位置 (flash: 默认填充 0xFF, efuse: 默认填充 0x00)')
    parser.add_argument('--fw-type', choices=['remote', 'car10', 'car16'], required=True,
                        help='固件类型 (remote: SOP10遥控器, car10: SOP10遥控车, car16: SOP16遥控车)')
    parser.add_argument('--output', '-o', type=str, default='config.bin',
                        help='输出 bin 文件名 (默认: config.bin)')
    parser.add_argument('--json', '-j', type=str,
                        help='输入 JSON 配置文件')
    parser.add_argument('--template', action='store_true',
                        help='生成 JSON 模板文件而不是 bin 文件')
    
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
    generator.generate(args.output)
    
    # 打印详细信息
    print("\nefuse_dev_info_t 详细数据:")
    print(f"  chip_id: {generator.efuse_dev_info.chip_id}")
    print(f"  paring_type: {generator.efuse_dev_info.paring_type}")
    print(f"  toy_type: {generator.efuse_dev_info.toy_type}")
    print(f"  access_code_group: {generator.efuse_dev_info.access_code_group}")
    print(f"  flash_disable: {generator.efuse_dev_info.flash_disable}")
    print(f"  normal_tx_power: {generator.efuse_dev_info.normal_tx_power}")
    print(f"  adc_calib: gpio_gain={generator.efuse_dev_info.adc_calib.gpio_gain}, "
          f"gpio_offset={generator.efuse_dev_info.adc_calib.gpio_offset}, "
          f"vbat_gain={generator.efuse_dev_info.adc_calib.vbat_gain}, "
          f"vbat_offset={generator.efuse_dev_info.adc_calib.vbat_offset}")
    
    print("\nCfgParamUnion 详细数据:")
    for key, value in vars(generator.cfg_param).items():
        if isinstance(value, int):
            print(f"  {key}: {value} (0x{value:X})")


if __name__ == '__main__':
    main()
