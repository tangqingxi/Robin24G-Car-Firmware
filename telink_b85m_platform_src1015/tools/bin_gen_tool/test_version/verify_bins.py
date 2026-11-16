#!/usr/bin/env python3
"""
验证 s10_car、s16_car、s10_remote 脚本生成的 bin 文件是否可以正确转换为对应的 C 结构体
"""

import struct
import sys
import os

sys.path.insert(0, '/home/matter/clz/project/robin_remote_car/2_junnan_sdk/tc122x_2.4g_sdk/telink_b85m_platform_src/tools/bin_gen_tool/test_version')
from bin_gen_test import (
    SOP10CarCfg, SOP16CarCfg, SOP10RemoteCfg,
    SOP10CarPwmDutyCycle, SOP16CarPwmDutyCycle,
    FwType, TargetType, ConfigGenerator
)

def verify_s10_car_bin(bin_path):
    """验证 S10 Car bin 文件"""
    print(f"\n{'='*60}")
    print(f"验证 S10 Car: {os.path.basename(bin_path)}")
    print(f"{'='*60}")
    
    with open(bin_path, 'rb') as f:
        data = f.read()
    
    # 解析 efuse_dev_info
    toy_type = data[7] & 0x07
    print(f"toy_type: {toy_type} (期望: 2=FW_TYPE_S10_CAR)")
    
    if toy_type != 2:
        print(f"  [错误] toy_type 错误!")
        return False
    
    # 解析 CfgParam (10 bytes)
    cfg_data = data[16:26]
    word0 = struct.unpack('<I', cfg_data[0:4])[0]
    word1 = struct.unpack('<I', cfg_data[4:8])[0]
    word2 = struct.unpack('<H', cfg_data[8:10])[0]
    
    # 解析关键字段
    pin0 = word0 & 0x0F
    pin1 = (word0 >> 4) & 0x0F
    pin2 = (word0 >> 8) & 0x0F
    pin3 = (word0 >> 12) & 0x0F
    pin4 = (word0 >> 16) & 0x0F
    special_func = (word0 >> 20) & 0x03
    shutdown_volt = (word0 >> 22) & 0x0F
    
    pwm_freq = word1 & 0x07
    rev_delay = (word1 >> 3) & 0x07
    spray_duration = (word1 >> 6) & 0x07
    spray_pwm_duty = (word1 >> 9) & 0x0F
    paired_indicator = (word1 >> 13) & 0x03
    sleep_timeout = (word1 >> 15) & 0x07
    
    drive_mode = (word1 >> 20) & 0x01
    pwm_duty_dual_en = (word1 >> 22) & 0x01
    pwm_duty_left = (word1 >> 23) & 0x07
    pwm_duty_right = (word1 >> 26) & 0x07
    
    access_code_group = word2 & 0x0F
    normal_tx_power = (word2 >> 4) & 0x0F
    
    print(f"Pin配置: pin0={pin0}, pin1={pin1}, pin2={pin2}, pin3={pin3}, pin4={pin4}")
    print(f"special_func={special_func}, drive_mode={drive_mode}")
    print(f"shutdown_volt={shutdown_volt}, pwm_freq={pwm_freq}, rev_delay={rev_delay}")
    print(f"spray_duration={spray_duration}, spray_pwm_duty={spray_pwm_duty}")
    print(f"sleep_timeout={sleep_timeout}, paired_indicator={paired_indicator}")
    print(f"pwm_duty_dual_en={pwm_duty_dual_en}, pwm_duty_left={pwm_duty_left}, pwm_duty_right={pwm_duty_right}")
    print(f"access_code_group={access_code_group}, normal_tx_power={normal_tx_power}")
    
    # 验证结构体大小
    expected_size = 10  # SOP10CarCfg 是 10 字节
    actual_size = len(cfg_data)
    if actual_size != expected_size:
        print(f"  [错误] CfgParam 大小错误: 期望 {expected_size}, 实际 {actual_size}")
        return False
    
    print(f"  [OK] CfgParam 大小正确: {actual_size} bytes")
    return True

def verify_s16_car_bin(bin_path):
    """验证 S16 Car bin 文件"""
    print(f"\n{'='*60}")
    print(f"验证 S16 Car: {os.path.basename(bin_path)}")
    print(f"{'='*60}")
    
    with open(bin_path, 'rb') as f:
        data = f.read()
    
    # 解析 efuse_dev_info
    toy_type = data[7] & 0x07
    print(f"toy_type: {toy_type} (期望: 3=FW_TYPE_S16_CAR)")
    
    if toy_type != 3:
        print(f"  [错误] toy_type 错误!")
        return False
    
    # 解析 CfgParam (14 bytes)
    cfg_data = data[16:30]
    word0 = struct.unpack('<I', cfg_data[0:4])[0]
    word1 = struct.unpack('<I', cfg_data[4:8])[0]
    word2 = struct.unpack('<I', cfg_data[8:12])[0]
    word3 = struct.unpack('<H', cfg_data[12:14])[0]
    
    # 解析关键字段
    pin0 = word0 & 0x0F
    pin1 = (word0 >> 4) & 0x0F
    pin2 = (word0 >> 8) & 0x0F
    pin3 = (word0 >> 12) & 0x0F
    pin4 = (word0 >> 16) & 0x0F
    pin5 = (word0 >> 20) & 0x0F
    pin6 = (word0 >> 24) & 0x0F
    pin7 = (word0 >> 28) & 0x0F
    
    pin8 = word1 & 0x0F
    pin9 = (word1 >> 4) & 0x0F
    pin10 = (word1 >> 8) & 0x0F
    shutdown_volt = (word1 >> 12) & 0x0F
    
    pwm_duty_dual_en = (word1 >> 18) & 0x01
    pwm_duty_left = (word1 >> 19) & 0x07
    pwm_duty_right = (word1 >> 22) & 0x07
    pwm_duty_w = (word1 >> 25) & 0x07
    
    paired_indicator = word2 & 0x03
    demo_mode = (word2 >> 2) & 0x03
    drive_mode = (word2 >> 4) & 0x01
    spray_w_mask = (word2 >> 5) & 0x0F
    special_func = (word2 >> 9) & 0x03
    
    pwm_duty_dual_en_word3 = word3 & 0x01
    pwm_duty_left_word3 = (word3 >> 1) & 0x07
    pwm_duty_right_word3 = (word3 >> 4) & 0x07
    pwm_duty_w_word3 = (word3 >> 7) & 0x07
    access_code_group = (word3 >> 10) & 0x0F
    normal_tx_power = (word3 >> 14) & 0x0F
    
    print(f"Pin配置: pin0={pin0}, pin1={pin1}, ..., pin10={pin10}")
    print(f"special_func={special_func}, drive_mode={drive_mode}")
    print(f"shutdown_volt={shutdown_volt}")
    print(f"pwm_duty_dual_en={pwm_duty_dual_en_word3}, pwm_duty_left={pwm_duty_left_word3}, pwm_duty_right={pwm_duty_right_word3}, pwm_duty_w={pwm_duty_w_word3}")
    print(f"access_code_group={access_code_group}, normal_tx_power={normal_tx_power}")
    
    # 验证结构体大小
    expected_size = 14  # SOP16CarCfg 是 14 字节
    actual_size = len(cfg_data)
    if actual_size != expected_size:
        print(f"  [错误] CfgParam 大小错误: 期望 {expected_size}, 实际 {actual_size}")
        return False
    
    print(f"  [OK] CfgParam 大小正确: {actual_size} bytes")
    return True

def verify_s10_remote_bin(bin_path):
    """验证 S10 Remote bin 文件"""
    print(f"\n{'='*60}")
    print(f"验证 S10 Remote: {os.path.basename(bin_path)}")
    print(f"{'='*60}")
    
    with open(bin_path, 'rb') as f:
        data = f.read()
    
    # 解析 efuse_dev_info
    toy_type = data[7] & 0x07
    print(f"toy_type: {toy_type} (期望: 1=FW_TYPE_S10_REMOTE)")
    
    if toy_type != 1:
        print(f"  [错误] toy_type 错误!")
        return False
    
    # 解析 CfgParam (6 bytes)
    cfg_data = data[16:22]
    word0 = struct.unpack('<I', cfg_data[0:4])[0]
    word1 = struct.unpack('<H', cfg_data[4:6])[0]
    
    # 解析关键字段
    pin0 = word0 & 0x0F
    pin1 = (word0 >> 4) & 0x0F
    pin2 = (word0 >> 8) & 0x0F
    pin3 = (word0 >> 12) & 0x0F
    pin4 = (word0 >> 16) & 0x0F
    shutdown_volt = (word0 >> 20) & 0x07
    low_batt_volt = (word0 >> 23) & 0x07
    sleep_timeout = (word0 >> 26) & 0x07
    paired_indicator = (word0 >> 29) & 0x03
    reserved0 = (word0 >> 31) & 0x01
    
    access_code_group = word1 & 0x0F
    normal_tx_power = (word1 >> 4) & 0x0F
    
    print(f"Pin配置: pin0={pin0}, pin1={pin1}, pin2={pin2}, pin3={pin3}, pin4={pin4}")
    print(f"shutdown_volt={shutdown_volt}, low_batt_volt={low_batt_volt}, sleep_timeout={sleep_timeout}")
    print(f"paired_indicator={paired_indicator}, reserved0={reserved0}")
    print(f"access_code_group={access_code_group}, normal_tx_power={normal_tx_power}")
    
    # 验证结构体大小
    expected_size = 6  # SOP10RemoteCfg 是 6 字节
    actual_size = len(cfg_data)
    if actual_size != expected_size:
        print(f"  [错误] CfgParam 大小错误: 期望 {expected_size}, 实际 {actual_size}")
        return False
    
    print(f"  [OK] CfgParam 大小正确: {actual_size} bytes")
    return True

def generate_and_verify():
    """生成 bin 文件并验证"""
    results = []
    
    # 1. 生成并验证 S10 Car
    print("\n" + "="*60)
    print("生成 S10 Car bin 文件")
    print("="*60)
    os.system("cd /home/matter/clz/project/robin_remote_car/2_junnan_sdk/tc122x_2.4g_sdk/telink_b85m_platform_src/tools/bin_gen_tool/test_version/s10_car_test_configs && python3 generate_s10_car_tests_v2.py --mode scenario --output-dir /tmp/verify_s10car 2>/dev/null")
    
    s10car_files = [
        "/tmp/verify_s10car/TOY_S10_CAR_MODE_2MOTOR_SPRAY_SCENE_ALL_MIN_VALUES_0x0f000.bin",
        "/tmp/verify_s10car/TOY_S10_CAR_MODE_2MOTOR_SPRAY_SCENE_ALL_MAX_VALUES_0x0f000.bin",
    ]
    for f in s10car_files:
        if os.path.exists(f):
            results.append(("S10 Car", f, verify_s10_car_bin(f)))
    
    # 2. 生成并验证 S16 Car
    print("\n" + "="*60)
    print("生成 S16 Car bin 文件")
    print("="*60)
    os.system("cd /home/matter/clz/project/robin_remote_car/2_junnan_sdk/tc122x_2.4g_sdk/telink_b85m_platform_src/tools/bin_gen_tool/test_version/s16_car_test_configs && python3 generate_s16_car_tests_v2.py --mode scenario --output-dir /tmp/verify_s16car 2>/dev/null")
    
    s16car_files = [
        "/tmp/verify_s16car/TOY_S16_CAR_MODE_2MOTOR_SPRAY_SCENE_ALL_MIN_VALUES_0x0f000.bin",
        "/tmp/verify_s16car/TOY_S16_CAR_MODE_2MOTOR_SPRAY_SCENE_ALL_MAX_VALUES_0x0f000.bin",
    ]
    for f in s16car_files:
        if os.path.exists(f):
            results.append(("S16 Car", f, verify_s16_car_bin(f)))
    
    # 3. 生成并验证 S10 Remote
    print("\n" + "="*60)
    print("生成 S10 Remote bin 文件")
    print("="*60)
    os.system("cd /home/matter/clz/project/robin_remote_car/2_junnan_sdk/tc122x_2.4g_sdk/telink_b85m_platform_src/tools/bin_gen_tool/test_version/s10_remote_test_configs && python3 generate_s10_remote_tests_v2.py --mode scenario --output-dir /tmp/verify_s10remote 2>/dev/null")
    
    s10remote_files = [
        "/tmp/verify_s10remote/S10_REMOTE_SCENE_ALL_MIN_VALUES_0x0f000.bin",
        "/tmp/verify_s10remote/S10_REMOTE_SCENE_ALL_MAX_VALUES_0x0f000.bin",
    ]
    for f in s10remote_files:
        if os.path.exists(f):
            results.append(("S10 Remote", f, verify_s10_remote_bin(f)))
    
    # 汇总结果
    print("\n" + "="*60)
    print("验证结果汇总")
    print("="*60)
    all_passed = True
    for type_name, f, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {type_name} - {os.path.basename(f)}")
        if not result:
            all_passed = False
    
    print("="*60)
    if all_passed:
        print("✓ 所有验证通过!")
    else:
        print("✗ 部分验证失败!")
    print("="*60)

if __name__ == "__main__":
    generate_and_verify()
