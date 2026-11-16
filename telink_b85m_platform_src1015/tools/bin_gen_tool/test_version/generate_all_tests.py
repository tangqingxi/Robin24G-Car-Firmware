#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
汇总测试配置生成脚本

功能说明:
    调用 s10_remote, s10_car, s16_car 三个脚本，生成 default 和 scenario 模式的测试配置
    生成的文件按模式分类保存到带时间戳的文件夹中

生成的文件夹结构:
    output/
    ├── default_YYYYMMDD_HHMMSS/
    │   ├── s10_remote/
    │   │   ├── *.bin
    │   │   ├── *.json
    │   │   └── README_default.md
    │   ├── s10_car/
    │   │   ├── *.bin
    │   │   ├── *.json
    │   │   └── README_default.md
    │   └── s16_car/
    │       ├── *.bin
    │       ├── *.json
    │       └── README_default.md
    └── scenario_YYYYMMDD_HHMMSS/
        ├── s10_remote/
        ├── s10_car/
        └── s16_car/

使用方法:
    python3 generate_all_tests.py                    # 生成 default 和 scenario 模式
    python3 generate_all_tests.py --mode default     # 只生成 default 模式
    python3 generate_all_tests.py --mode scenario    # 只生成 scenario 模式
    python3 generate_all_tests.py --output-dir ./out # 指定输出目录
"""

import os
import sys
import shutil
import argparse
import subprocess
from datetime import datetime
from pathlib import Path

# 脚本配置
SCRIPTS_CONFIG = [
    {
        "name": "s10_remote",
        "script": "s10_remote_test_configs/generate_s10_remote_tests_v2.py",
        "modes": ["default", "scenario"]
    },
    {
        "name": "s10_car",
        "script": "s10_car_test_configs/generate_s10_car_tests_v2.py",
        "modes": ["default", "scenario"]
    },
    {
        "name": "s16_car",
        "script": "s16_car_test_configs/generate_s16_car_tests_v2.py",
        "modes": ["default", "scenario"]
    }
]


def get_timestamp() -> str:
    """获取当前时间戳字符串"""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def run_script(script_path: str, mode: str, output_dir: str) -> bool:
    """
    运行单个脚本生成配置
    
    Args:
        script_path: 脚本路径
        mode: 测试模式 (default/scenario)
        output_dir: 输出目录
    
    Returns:
        bool: 是否成功
    """
    cmd = [
        sys.executable,
        script_path,
        "--mode", mode,
        "--output-dir", output_dir
    ]
    
    print(f"  执行: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120
        )
        
        if result.returncode != 0:
            print(f"  错误: {result.stderr}")
            return False
        
        # 打印脚本输出
        for line in result.stdout.strip().split('\n'):
            print(f"    {line}")
        
        return True
    
    except subprocess.TimeoutExpired:
        print(f"  错误: 脚本执行超时")
        return False
    except Exception as e:
        print(f"  错误: {e}")
        return False


def copy_generated_files(src_dir: str, dst_dir: str) -> int:
    """
    拷贝生成的文件到目标目录，按文件名分组存放
    
    文件组织结构:
        dst_dir/
        ├── <base_name_1>/
        │   ├── <base_name_1>_0x0f000.bin
        │   ├── <base_name_1>_0x00000.bin
        │   └── <base_name_1>.json
        ├── <base_name_2>/
        │   ├── <base_name_2>_0x0f000.bin
        │   ├── <base_name_2>_0x00000.bin
        │   └── <base_name_2>.json
        └── README_<mode>.md
    
    Args:
        src_dir: 源目录
        dst_dir: 目标目录
    
    Returns:
        int: 拷贝的文件数量
    """
    os.makedirs(dst_dir, exist_ok=True)
    
    copied_count = 0
    src_path = Path(src_dir)
    
    # 收集所有文件
    bin_files = list(src_path.glob("*.bin"))
    json_files = list(src_path.glob("*.json"))
    md_files = list(src_path.glob("*.md"))
    
    # 按基础名称分组文件
    file_groups = {}
    
    # 处理 bin 文件，提取基础名称
    for bin_file in bin_files:
        # 从文件名中移除 _0x0f000 或 _0x00000 后缀获取基础名称
        name = bin_file.stem  # 不含扩展名
        
        # 移除地址后缀 (_0x0f000 或 _0x00000)
        if "_0x0f000" in name:
            base_name = name.replace("_0x0f000", "")
        elif "_0x00000" in name:
            base_name = name.replace("_0x00000", "")
        else:
            base_name = name
        
        if base_name not in file_groups:
            file_groups[base_name] = {"bin": [], "json": None}
        file_groups[base_name]["bin"].append(bin_file)
    
    # 处理 json 文件，匹配到对应的组
    for json_file in json_files:
        base_name = json_file.stem
        if base_name in file_groups:
            file_groups[base_name]["json"] = json_file
        else:
            # 如果没有对应的bin文件，单独创建一组
            file_groups[base_name] = {"bin": [], "json": json_file}
    
    # 拷贝文件到分组目录
    for base_name, files in file_groups.items():
        # 创建子目录
        group_dir = os.path.join(dst_dir, base_name)
        os.makedirs(group_dir, exist_ok=True)
        
        # 拷贝 bin 文件
        for bin_file in files["bin"]:
            dst_file = os.path.join(group_dir, bin_file.name)
            shutil.copy2(bin_file, dst_file)
            copied_count += 1
            print(f"    拷贝: {base_name}/{bin_file.name}")
        
        # 拷贝 json 文件
        if files["json"]:
            json_file = files["json"]
            dst_file = os.path.join(group_dir, json_file.name)
            shutil.copy2(json_file, dst_file)
            copied_count += 1
            print(f"    拷贝: {base_name}/{json_file.name}")
    
    # 拷贝 md 文件到根目录
    for md_file in md_files:
        dst_file = os.path.join(dst_dir, md_file.name)
        shutil.copy2(md_file, dst_file)
        copied_count += 1
        print(f"    拷贝: {md_file.name}")
    
    return copied_count


def generate_mode_configs(mode: str, base_output_dir: str, timestamp: str) -> dict:
    """
    生成指定模式的配置
    
    Args:
        mode: 测试模式 (default/scenario)
        base_output_dir: 基础输出目录
        timestamp: 时间戳
    
    Returns:
        dict: 各脚本的生成结果
    """
    results = {}
    
    # 创建带时间戳的输出目录
    mode_output_dir = os.path.join(base_output_dir, f"{mode}_{timestamp}")
    os.makedirs(mode_output_dir, exist_ok=True)
    
    print(f"\n{'='*70}")
    print(f"生成 {mode.upper()} 模式配置")
    print(f"输出目录: {mode_output_dir}")
    print(f"{'='*70}")
    
    # 获取脚本所在目录
    script_base_dir = os.path.dirname(os.path.abspath(__file__))
    
    for config in SCRIPTS_CONFIG:
        script_name = config["name"]
        
        # 检查该脚本是否支持当前模式
        if mode not in config["modes"]:
            print(f"\n跳过 {script_name} (不支持 {mode} 模式)")
            continue
        
        print(f"\n处理 {script_name}...")
        
        # 创建临时输出目录
        temp_output_dir = os.path.join(mode_output_dir, f"temp_{script_name}")
        os.makedirs(temp_output_dir, exist_ok=True)
        
        # 运行脚本
        script_path = os.path.join(script_base_dir, config["script"])
        success = run_script(script_path, mode, temp_output_dir)
        
        if not success:
            results[script_name] = {"success": False, "files_copied": 0}
            continue
        
        # 创建最终输出子目录
        final_output_dir = os.path.join(mode_output_dir, script_name)
        os.makedirs(final_output_dir, exist_ok=True)
        
        # 拷贝生成的文件
        files_copied = copy_generated_files(temp_output_dir, final_output_dir)
        
        # 清理临时目录
        shutil.rmtree(temp_output_dir)
        
        results[script_name] = {
            "success": True,
            "files_copied": files_copied,
            "output_dir": final_output_dir
        }
    
    return results


def print_summary(results: dict, mode: str, output_dir: str):
    """打印生成摘要"""
    print(f"\n{'='*70}")
    print(f"{mode.upper()} 模式生成摘要")
    print(f"{'='*70}")
    
    total_files = 0
    success_count = 0
    
    for script_name, result in results.items():
        if result["success"]:
            success_count += 1
            total_files += result["files_copied"]
            print(f"{script_name}: 成功 ({result['files_copied']} 个文件)")
        else:
            print(f"{script_name}: 失败")
    
    print(f"\n总计: {success_count}/{len(results)} 个脚本成功")
    print(f"总文件数: {total_files}")
    print(f"输出目录: {output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description='汇总测试配置生成工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python3 generate_all_tests.py                    # 生成 default 和 scenario 模式
  python3 generate_all_tests.py --mode default     # 只生成 default 模式
  python3 generate_all_tests.py --mode scenario    # 只生成 scenario 模式
  python3 generate_all_tests.py --output-dir ./out # 指定输出目录
        '''
    )
    
    parser.add_argument('--mode', choices=['default', 'scenario', 'all'],
                       default='all',
                       help='生成模式 (默认: all)')
    parser.add_argument('--output-dir', '-o',
                       default='output',
                       help='输出目录 (默认: output)')
    
    args = parser.parse_args()
    
    # 获取时间戳
    timestamp = get_timestamp()
    print(f"时间戳: {timestamp}")
    
    # 确保输出目录存在
    base_output_dir = os.path.abspath(args.output_dir)
    os.makedirs(base_output_dir, exist_ok=True)
    
    # 确定要生成的模式
    modes_to_generate = []
    if args.mode in ['default', 'all']:
        modes_to_generate.append('default')
    if args.mode in ['scenario', 'all']:
        modes_to_generate.append('scenario')
    
    # 生成各模式的配置
    all_results = {}
    for mode in modes_to_generate:
        results = generate_mode_configs(mode, base_output_dir, timestamp)
        all_results[mode] = results
        
        # 打印摘要
        mode_output_dir = os.path.join(base_output_dir, f"{mode}_{timestamp}")
        print_summary(results, mode, mode_output_dir)
    
    # 打印总摘要
    print(f"\n{'='*70}")
    print("生成完成!")
    print(f"{'='*70}")
    print(f"\n生成的文件夹:")
    for mode in modes_to_generate:
        mode_output_dir = os.path.join(base_output_dir, f"{mode}_{timestamp}")
        print(f"  - {mode_output_dir}/")


if __name__ == '__main__':
    main()
