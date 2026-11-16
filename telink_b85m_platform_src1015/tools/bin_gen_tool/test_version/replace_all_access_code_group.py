#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量替换配置脚本中的 access_code_group 值

用法:
    python3 replace_all_access_code_group.py              # 使用默认值(2)替换所有目标文件
    python3 replace_all_access_code_group.py --value 5    # 将值替换为5
    python3 replace_all_access_code_group.py --files file1.py file2.py  # 指定文件
"""

import os
import re
import argparse
from pathlib import Path


# 默认目标文件列表（相对于脚本所在目录）
DEFAULT_TARGET_FILES = [
    "s10_remote_test_configs/generate_s10_remote_tests_v2.py",
    "s10_car_test_configs/generate_s10_car_tests_v2.py",
    "s16_car_test_configs/generate_s16_car_tests_v2.py",
]


def replace_in_file(file_path: Path, old_value: int, new_value: int) -> int:
    """
    在文件中替换 access_code_group 的值
    
    Args:
        file_path: 文件路径
        old_value: 旧值
        new_value: 新值
        
    Returns:
        替换的次数
    """
    if not file_path.exists():
        print(f"[警告] 文件不存在: {file_path}")
        return 0
    
    content = file_path.read_text(encoding='utf-8')
    
    # 正则匹配: "access_code_group": 0,
    # 支持不同空格格式
    pattern = rf'("access_code_group"\s*:\s*){old_value}(\s*,)'
    replacement = rf'\g<1>{new_value}\g<2>'
    
    new_content, count = re.subn(pattern, replacement, content)
    
    if count > 0:
        file_path.write_text(new_content, encoding='utf-8')
        print(f"[成功] {file_path}: 替换了 {count} 处 (value: {old_value} -> {new_value})")
    else:
        print(f"[信息] {file_path}: 未找到需要替换的内容 (旧值: {old_value})")
    
    return count


def main():
    parser = argparse.ArgumentParser(
        description="批量替换配置脚本中的 access_code_group 值",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用默认值将所有文件中的 access_code_group 改为 2
  python replace_all_access_code_group.py
  
  # 将所有文件中的 access_code_group 改为 5
  python replace_all_access_code_group.py --value 5
  
  # 只修改指定文件，将旧值 0 改为新值 3
  python replace_all_access_code_group.py --files a.py b.py --old-value 0 --value 3
        """
    )
    parser.add_argument(
        "--value", "-v",
        type=int,
        default=2,
        help="要设置的新值 (默认: 2)"
    )
    parser.add_argument(
        "--old-value", "-o",
        type=int,
        default=0,
        help="要替换的旧值 (默认: 0)"
    )
    parser.add_argument(
        "--files", "-f",
        nargs="+",
        help="指定要处理的文件路径 (默认使用预设的三个脚本文件)"
    )
    parser.add_argument(
        "--dry-run", "-d",
        action="store_true",
        help="试运行模式，只显示会替换的内容但不实际修改文件"
    )
    
    args = parser.parse_args()
    
    # 确定脚本所在目录
    script_dir = Path(__file__).parent.absolute()
    
    # 确定目标文件列表
    if args.files:
        target_files = [Path(f) for f in args.files]
    else:
        target_files = [script_dir / f for f in DEFAULT_TARGET_FILES]
    
    print(f"=" * 60)
    print(f"批量替换 access_code_group: {args.old_value} -> {args.value}")
    print(f"=" * 60)
    print()
    
    total_replacements = 0
    
    for file_path in target_files:
        if args.dry_run:
            # 试运行模式：只显示匹配内容
            if file_path.exists():
                content = file_path.read_text(encoding='utf-8')
                pattern = rf'("access_code_group"\s*:\s*){args.old_value}(\s*,)'
                matches = re.findall(pattern, content)
                print(f"[试运行] {file_path}: 找到 {len(matches)} 处可替换")
                total_replacements += len(matches)
            else:
                print(f"[警告] 文件不存在: {file_path}")
        else:
            # 实际替换
            total_replacements += replace_in_file(file_path, args.old_value, args.value)
    
    print()
    print(f"=" * 60)
    if args.dry_run:
        print(f"试运行完成，共找到 {total_replacements} 处可替换")
    else:
        print(f"替换完成，共修改 {total_replacements} 处")
    print(f"=" * 60)


if __name__ == "__main__":
    main()
