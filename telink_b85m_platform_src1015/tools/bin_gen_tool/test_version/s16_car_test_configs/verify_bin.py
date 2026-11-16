#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S16 Car bin 文件验证工具

验证生成的 bin 文件中的关键参数，特别是 led_polar、spray_duration、spray_pwm_duty、spray_w_mask

用法:
    python3 verify_bin.py <bin_file>
    python3 verify_bin.py --scenario-all  # 验证所有场景生成的 bin 文件
"""

import sys
import os
import struct
import glob

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def parse_s16_car_cfg_param(data: bytes, offset: int = 0) -> dict:
    """
    解析 SOP16CarCfgParam 结构体（14字节）
    
    数据布局（小端序）：
    - bytes 0-3:   word0 (pin0-pin7)
    - bytes 4-7:   word1 (pin8, pin9, pin10, shutdown_volt, pwm_freq, rev_delay, spray_duration, spray_pwm_duty, sleep_timeout)
    - bytes 8-9:   word2 (paired_indicator, demo_mode, drive_mode, spray_w_mask, special_func, led_polar, reserved0)
    - bytes 10-11: word3 (pwm_duty_dual_en, pwm_duty_left, pwm_duty_right, pwm_duty_w, access_code_group, reserved1)
    - bytes 12-13: word4 (normal_tx_power, reserved2)
    """
    if len(data) < offset + 14:
        raise ValueError(f"数据太短，需要至少 {offset + 14} 字节，实际 {len(data)} 字节")
    
    cfg_data = data[offset:offset+14]
    
    # 解析各个 word
    word0 = struct.unpack('<I', cfg_data[0:4])[0]
    word1 = struct.unpack('<I', cfg_data[4:8])[0]
    word2 = struct.unpack('<H', cfg_data[8:10])[0]
    word3 = struct.unpack('<H', cfg_data[10:12])[0]
    word4 = struct.unpack('<H', cfg_data[12:14])[0]
    
    # 解析 word0: pin0-pin7
    pin0 = word0 & 0x0F
    pin1 = (word0 >> 4) & 0x0F
    pin2 = (word0 >> 8) & 0x0F
    pin3 = (word0 >> 12) & 0x0F
    pin4 = (word0 >> 16) & 0x0F
    pin5 = (word0 >> 20) & 0x0F
    pin6 = (word0 >> 24) & 0x0F
    pin7 = (word0 >> 28) & 0x0F
    
    # 解析 word1: pin8, pin9, pin10, shutdown_volt, pwm_freq, rev_delay, spray_duration, spray_pwm_duty, sleep_timeout
    pin8 = word1 & 0x0F
    pin9 = (word1 >> 4) & 0x0F
    pin10 = (word1 >> 8) & 0x0F
    shutdown_volt = (word1 >> 12) & 0x0F
    pwm_freq = (word1 >> 16) & 0x07
    rev_delay = (word1 >> 19) & 0x07
    spray_duration = (word1 >> 22) & 0x07
    spray_pwm_duty = (word1 >> 25) & 0x0F
    sleep_timeout = (word1 >> 29) & 0x07
    
    # 解析 word2: paired_indicator, demo_mode, drive_mode, spray_w_mask, special_func, led_polar, reserved0
    paired_indicator = word2 & 0x03
    demo_mode = (word2 >> 2) & 0x03
    drive_mode = (word2 >> 4) & 0x01
    spray_w_mask = (word2 >> 5) & 0x0F
    special_func = (word2 >> 9) & 0x03
    led_polar = (word2 >> 11) & 0x01
    reserved0 = (word2 >> 12) & 0x0F
    
    # 解析 word3: pwm_duty_dual_en, pwm_duty_left, pwm_duty_right, pwm_duty_w, access_code_group, reserved1
    pwm_duty_dual_en = word3 & 0x01
    pwm_duty_left = (word3 >> 1) & 0x07
    pwm_duty_right = (word3 >> 4) & 0x07
    pwm_duty_w = (word3 >> 7) & 0x07
    access_code_group = (word3 >> 10) & 0x0F
    reserved1 = (word3 >> 14) & 0x03
    
    # 解析 word4: normal_tx_power, reserved2
    normal_tx_power = word4 & 0x0F
    reserved2 = (word4 >> 4) & 0xFFF
    
    return {
        "raw_bytes": cfg_data.hex(),
        "word0": f"0x{word0:08x}",
        "word1": f"0x{word1:08x}",
        "word2": f"0x{word2:04x}",
        "word3": f"0x{word3:04x}",
        "word4": f"0x{word4:04x}",
        "pins": {
            "pin0": pin0, "pin1": pin1, "pin2": pin2, "pin3": pin3, "pin4": pin4,
            "pin5": pin5, "pin6": pin6, "pin7": pin7, "pin8": pin8, "pin9": pin9, "pin10": pin10
        },
        "shutdown_volt": shutdown_volt,
        "pwm_freq": pwm_freq,
        "rev_delay": rev_delay,
        "spray_duration": spray_duration,
        "spray_pwm_duty": spray_pwm_duty,
        "sleep_timeout": sleep_timeout,
        "paired_indicator": paired_indicator,
        "demo_mode": demo_mode,
        "drive_mode": drive_mode,
        "spray_w_mask": spray_w_mask,
        "special_func": special_func,
        "led_polar": led_polar,
        "reserved0": reserved0,
        "pwm_duty_dual_en": pwm_duty_dual_en,
        "pwm_duty_left": pwm_duty_left,
        "pwm_duty_right": pwm_duty_right,
        "pwm_duty_w": pwm_duty_w,
        "access_code_group": access_code_group,
        "reserved1": reserved1,
        "normal_tx_power": normal_tx_power,
        "reserved2": reserved2,
    }


def verify_flash_bin(bin_file: str) -> dict:
    """验证 Flash 版本的 bin 文件"""
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    if len(data) < 30:
        raise ValueError(f"Flash bin 文件太短: {len(data)} 字节")
    
    # Flash 版本: efuse_dev_info (16 bytes) + CfgParamUnion (从 offset 16 开始)
    efuse_data = data[0:16]
    cfg_param = parse_s16_car_cfg_param(data, offset=16)
    
    return {
        "type": "Flash",
        "total_size": len(data),
        "efuse_dev_info": efuse_data.hex(),
        "cfg_param": cfg_param
    }


def verify_efuse_bin(bin_file: str) -> dict:
    """验证 EFuse 版本的 bin 文件"""
    with open(bin_file, 'rb') as f:
        data = f.read()
    
    if len(data) < 30:
        raise ValueError(f"EFuse bin 文件太短: {len(data)} 字节")
    
    # EFuse 版本: efuse_dev_info (16 bytes) + CfgParamUnion (从 offset 16 开始)
    # 注意: C 代码从 EFuse 地址 0x10 读取 CfgParamUnion，对应 bin 文件的 offset 16
    efuse_data = data[0:16]
    cfg_param = parse_s16_car_cfg_param(data, offset=16)
    
    return {
        "type": "EFuse",
        "total_size": len(data),
        "efuse_dev_info": efuse_data.hex(),
        "cfg_param": cfg_param
    }


def print_verification_result(result: dict, bin_file: str):
    """打印验证结果"""
    print("=" * 70)
    print(f"验证文件: {os.path.basename(bin_file)}")
    print(f"类型: {result['type']}")
    print(f"总大小: {result['total_size']} 字节")
    print("=" * 70)
    
    cfg = result['cfg_param']
    
    print(f"\n【原始数据】")
    print(f"  efuse_dev_info (16 bytes): {result['efuse_dev_info']}")
    print(f"  cfg_param (14 bytes):      {cfg['raw_bytes']}")
    
    print(f"\n【Word 解析】")
    print(f"  word0: {cfg['word0']} (pin0-pin7)")
    print(f"  word1: {cfg['word1']} (pin8-sleep_timeout)")
    print(f"  word2: {cfg['word2']} (paired_indicator-reserved0)")
    print(f"  word3: {cfg['word3']} (pwm_duty_dual_en-reserved1)")
    print(f"  word4: {cfg['word4']} (normal_tx_power-reserved2)")
    
    print(f"\n【关键参数验证】")
    
    # led_polar
    led_polar = cfg['led_polar']
    led_polar_status = "✓" if led_polar == 1 else "✗"
    print(f"  {led_polar_status} led_polar: {led_polar} (期望值: 1 for all_max_values)")
    
    # spray_duration
    spray_duration = cfg['spray_duration']
    spray_duration_desc = {0: "DEFAULT", 1: "30S", 2: "1MIN", 3: "2MIN", 4: "ALWAYS"}.get(spray_duration, f"UNKNOWN({spray_duration})")
    print(f"    spray_duration: {spray_duration} ({spray_duration_desc})")
    
    # spray_pwm_duty
    spray_pwm_duty = cfg['spray_pwm_duty']
    spray_duty_desc = {0: "DEFAULT", 1: "7%", 2: "10%", 3: "15%", 4: "20%", 5: "25%", 6: "30%", 7: "35%", 8: "40%"}.get(spray_pwm_duty, f"UNKNOWN({spray_pwm_duty})")
    print(f"    spray_pwm_duty: {spray_pwm_duty} ({spray_duty_desc})")
    
    # spray_w_mask
    spray_w_mask = cfg['spray_w_mask']
    print(f"    spray_w_mask: 0x{spray_w_mask:01x}")
    
    print(f"\n【其他参数】")
    print(f"  drive_mode: {cfg['drive_mode']} ({'差速' if cfg['drive_mode'] == 0 else '双电机'})")
    print(f"  special_func: {cfg['special_func']} ({'DEFAULT' if cfg['special_func'] == 0 else 'SPRAY' if cfg['special_func'] == 1 else 'EXCAVATOR'})")
    print(f"  paired_indicator: {cfg['paired_indicator']}")
    print(f"  demo_mode: {cfg['demo_mode']}")
    print(f"  pwm_duty_dual_en: {cfg['pwm_duty_dual_en']}")
    print(f"  pwm_duty_left: {cfg['pwm_duty_left']}")
    print(f"  pwm_duty_right: {cfg['pwm_duty_right']}")
    print(f"  pwm_duty_w: {cfg['pwm_duty_w']}")
    print(f"  access_code_group: {cfg['access_code_group']}")
    print(f"  normal_tx_power: {cfg['normal_tx_power']}")
    
    print(f"\n【Pin 配置】")
    for i in range(11):
        print(f"  pin{i}: {cfg['pins'][f'pin{i}']}")
    
    print("=" * 70)
    
    # 返回关键参数用于批量验证
    return {
        "led_polar": led_polar,
        "spray_duration": spray_duration,
        "spray_pwm_duty": spray_pwm_duty,
        "spray_w_mask": spray_w_mask,
        "drive_mode": cfg['drive_mode'],
        "special_func": cfg['special_func'],
    }


def verify_all_scenarios():
    """验证所有场景生成的 bin 文件"""
    print("\n" + "=" * 70)
    print("批量验证所有 S16 Car 场景的 bin 文件")
    print("=" * 70 + "\n")
    
    # 查找所有场景 bin 文件
    patterns = [
        "TOY_S16_CAR_MODE_*_SCENE_ALL_MAX_VALUES*_0x0f000.bin",  # Flash
        "TOY_S16_CAR_MODE_*_SCENE_ALL_MAX_VALUES*_0x00000.bin",  # EFuse
    ]
    
    all_files = []
    for pattern in patterns:
        all_files.extend(glob.glob(pattern))
    
    if not all_files:
        print("未找到场景 bin 文件，请先生成：")
        print("  python3 generate_s16_car_tests_v2.py --mode scenario")
        return
    
    # 按场景分组
    scenes = {}
    for f in sorted(all_files):
        # 提取场景名
        parts = f.split('_')
        if 'SCENE' in parts:
            scene_idx = parts.index('SCENE')
            mode = '_'.join(parts[:scene_idx])
            scene_name = parts[scene_idx + 1]
            key = f"{mode}_{scene_name}"
            if key not in scenes:
                scenes[key] = {}
            if '0x0f000' in f:
                scenes[key]['flash'] = f
            elif '0x00000' in f:
                scenes[key]['efuse'] = f
    
    # 验证每个场景
    results = []
    for scene_key, files in sorted(scenes.items()):
        print(f"\n{'='*70}")
        print(f"场景: {scene_key}")
        print(f"{'='*70}")
        
        for bin_type, bin_file in files.items():
            try:
                if bin_type == 'flash':
                    result = verify_flash_bin(bin_file)
                else:
                    result = verify_efuse_bin(bin_file)
                
                params = print_verification_result(result, bin_file)
                results.append({
                    "scene": scene_key,
                    "type": bin_type,
                    "file": bin_file,
                    "params": params,
                    "ok": True
                })
            except Exception as e:
                print(f"\n✗ 验证失败: {bin_file}")
                print(f"  错误: {e}")
                results.append({
                    "scene": scene_key,
                    "type": bin_type,
                    "file": bin_file,
                    "ok": False,
                    "error": str(e)
                })
    
    # 汇总报告
    print("\n" + "=" * 70)
    print("验证汇总")
    print("=" * 70)
    
    all_max_ok = True
    for r in results:
        if not r['ok']:
            print(f"✗ {r['scene']} ({r['type']}): 验证失败 - {r.get('error', 'Unknown')}")
            all_max_ok = False
        elif 'ALL_MAX_VALUES' in r['scene']:
            # 检查 all_max_values 场景的 led_polar
            if r['params']['led_polar'] != 1:
                print(f"✗ {r['scene']} ({r['type']}): led_polar = {r['params']['led_polar']} (期望值: 1)")
                all_max_ok = False
            else:
                print(f"✓ {r['scene']} ({r['type']}): led_polar = 1 ✓")
    
    if all_max_ok:
        print("\n✓ 所有 all_max_values 场景的 led_polar 都正确设置为 1")
    else:
        print("\n✗ 部分 all_max_values 场景的 led_polar 设置不正确")
    
    return results


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print(f"  python3 {sys.argv[0]} <bin_file>       # 验证单个 bin 文件")
        print(f"  python3 {sys.argv[0]} --scenario-all   # 验证所有场景 bin 文件")
        print(f"  python3 {sys.argv[0]} --help           # 显示帮助")
        sys.exit(1)
    
    if sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        sys.exit(0)
    
    if sys.argv[1] == '--scenario-all':
        verify_all_scenarios()
        sys.exit(0)
    
    bin_file = sys.argv[1]
    if not os.path.exists(bin_file):
        print(f"错误: 文件不存在: {bin_file}")
        sys.exit(1)
    
    # 判断 bin 类型
    if '0x0f000' in bin_file:
        result = verify_flash_bin(bin_file)
    elif '0x00000' in bin_file:
        result = verify_efuse_bin(bin_file)
    else:
        # 尝试根据文件大小判断
        with open(bin_file, 'rb') as f:
            data = f.read()
        if len(data) >= 256:
            result = verify_flash_bin(bin_file)
        else:
            result = verify_efuse_bin(bin_file)
    
    print_verification_result(result, bin_file)


if __name__ == '__main__':
    main()
