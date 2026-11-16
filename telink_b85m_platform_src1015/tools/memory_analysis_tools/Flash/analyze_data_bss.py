#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC32 ELF .lst 文件 Data/BSS 区域变量分析工具

功能:
    解析 TC32 编译器生成的 .lst 文件，提取 .data 和 .bss 区域的变量信息，
    生成详细的内存分析报告。

用法:
    python3 analyze_data_bss.py <input.lst> [options]
    
示例:
    # 基本用法
    python3 analyze_data_bss.py project/Robin_24G_Flash/Robin_24G_Flash.lst
    
    # 指定输出文件
    python3 analyze_data_bss.py input.lst -o output.md
    
    # 生成 CSV 格式
    python3 analyze_data_bss.py input.lst -f csv -o vars.csv
    
    # 只显示大变量(>64 bytes)
    python3 analyze_data_bss.py input.lst --min-size 64
    
    # 只分析 bss 区域
    python3 analyze_data_bss.py input.lst --section bss
"""

import re
import sys
import argparse
from pathlib import Path
from dataclasses import dataclass
from datetime import datetime
from typing import List, Dict, Optional, Tuple


@dataclass
class Variable:
    """变量信息数据类"""
    address: str
    name: str
    size: int
    section: str  # 'data' or 'bss'
    scope: str    # 'global' or 'local'
    raw_line: str
    
    @property
    def address_int(self) -> int:
        return int(self.address, 16)
    
    def __repr__(self) -> str:
        return f"Variable(0x{self.address}: {self.name}, {self.size} bytes, {self.section}, {self.scope})"


class LstAnalyzer:
    """.lst 文件分析器"""
    
    # 匹配符号表行的正则表达式
    # 格式: 008419b4 l     O .data\t00000001 last_print_cmd.4673
    SYMBOL_PATTERN = re.compile(
        r'^\s*([0-9a-fA-F]+)\s+([lg])\s+O\s+\.(data|bss)\s+([0-9a-fA-F]+)\s+(\S+)'
    )
    
    def __init__(self, lst_path: str):
        self.lst_path = Path(lst_path)
        self.variables: List[Variable] = []
        self.data_vars: List[Variable] = []
        self.bss_vars: List[Variable] = []
        
    def parse(self) -> bool:
        """解析 .lst 文件"""
        if not self.lst_path.exists():
            print(f"错误: 文件不存在 - {self.lst_path}")
            return False
            
        try:
            with open(self.lst_path, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    self._parse_line(line)
            
            # 按地址排序
            self.data_vars.sort(key=lambda v: v.address_int)
            self.bss_vars.sort(key=lambda v: v.address_int)
            self.variables = self.data_vars + self.bss_vars
            
            return True
            
        except Exception as e:
            print(f"错误: 解析文件失败 - {e}")
            return False
    
    def _parse_line(self, line: str):
        """解析单行"""
        match = self.SYMBOL_PATTERN.match(line)
        if not match:
            return
            
        addr, scope_char, section, size_hex, name = match.groups()
        
        scope = 'global' if scope_char == 'g' else 'local'
        size = int(size_hex, 16)
        
        var = Variable(
            address=addr.lower(),
            name=name,
            size=size,
            section=section,
            scope=scope,
            raw_line=line.strip()
        )
        
        if section == 'data':
            self.data_vars.append(var)
        else:
            self.bss_vars.append(var)
    
    def get_section_info(self, section: str) -> Tuple[int, int, int]:
        """获取区域信息 (start, end, size)"""
        vars_list = self.data_vars if section == 'data' else self.bss_vars
        
        if not vars_list:
            return 0, 0, 0
            
        start = vars_list[0].address_int
        end = max(v.address_int + v.size for v in vars_list)
        size = end - start
        
        return start, end, size
    
    def get_large_variables(self, min_size: int = 32) -> List[Variable]:
        """获取大变量列表"""
        return [v for v in self.variables if v.size > min_size]
    
    def get_variables_by_section(self, section: Optional[str] = None) -> List[Variable]:
        """按区域获取变量"""
        if section == 'data':
            return self.data_vars
        elif section == 'bss':
            return self.bss_vars
        return self.variables
    
    def get_total_size(self) -> Dict[str, int]:
        """获取总大小统计"""
        return {
            'data': sum(v.size for v in self.data_vars),
            'bss': sum(v.size for v in self.bss_vars),
            'total': sum(v.size for v in self.variables)
        }


class ReportGenerator:
    """报告生成器"""
    
    def __init__(self, analyzer: LstAnalyzer):
        self.analyzer = analyzer
        
    def generate_markdown(self, min_large_size: int = 32) -> str:
        """生成 Markdown 报告"""
        lines = []
        
        # 标题
        lines.append(f"# Data/BSS 区域变量分析报告")
        lines.append(f"")
        lines.append(f"> 分析文件: `{self.analyzer.lst_path}`")
        lines.append(f"> 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"")
        
        # 内存区域信息
        lines.append(self._generate_section_summary())
        
        # .data 区域详情
        if self.analyzer.data_vars:
            lines.append(self._generate_variable_table(
                ".data 区域变量 (已初始化)", 
                self.analyzer.data_vars
            ))
        
        # .bss 区域详情
        if self.analyzer.bss_vars:
            lines.append(self._generate_variable_table(
                ".bss 区域变量 (未初始化)", 
                self.analyzer.bss_vars
            ))
        
        # 大变量列表
        large_vars = self.analyzer.get_large_variables(min_large_size)
        if large_vars:
            lines.append(self._generate_large_variable_table(
                f"大变量列表 (>{min_large_size} bytes)", 
                large_vars
            ))
        
        # 统计信息
        lines.append(self._generate_statistics())
        
        return '\n'.join(lines)
    
    def _generate_section_summary(self) -> str:
        """生成区域摘要"""
        lines = []
        lines.append("## 内存区域信息")
        lines.append("")
        lines.append("| 区域 | 起始地址 | 结束地址 | 大小 | 变量数 |")
        lines.append("|------|----------|----------|------|--------|")
        
        for section in ['data', 'bss']:
            vars_list = self.analyzer.get_variables_by_section(section)
            if vars_list:
                start, end, size = self.analyzer.get_section_info(section)
                lines.append(
                    f"| .{section} | 0x{start:08x} | 0x{end:08x} | "
                    f"{size} bytes | {len(vars_list)} |"
                )
        
        total = self.analyzer.get_total_size()
        lines.append(f"| **总计** | - | - | **{total['total']} bytes** | **{len(self.analyzer.variables)}** |")
        lines.append("")
        
        return '\n'.join(lines)
    
    def _generate_variable_table(self, title: str, variables: List[Variable]) -> str:
        """生成变量表格"""
        lines = []
        lines.append(f"## {title}")
        lines.append("")
        lines.append("| 地址 | 变量名 | 大小(字节) | 类型 |")
        lines.append("|------|--------|-----------|------|")
        
        for var in variables:
            scope_cn = "全局" if var.scope == "global" else "局部"
            lines.append(f"| 0x{var.address} | `{var.name}` | {var.size} | {scope_cn} |")
        
        # 统计
        global_count = sum(1 for v in variables if v.scope == 'global')
        local_count = len(variables) - global_count
        lines.append("")
        lines.append(f"**统计**: 共 {len(variables)} 个变量 (全局: {global_count}, 局部: {local_count})")
        lines.append("")
        
        return '\n'.join(lines)
    
    def _generate_large_variable_table(self, title: str, variables: List[Variable]) -> str:
        """生成大变量表格"""
        lines = []
        lines.append(f"## {title}")
        lines.append("")
        lines.append("| 排名 | 地址 | 变量名 | 大小(字节) | 区域 | 类型 |")
        lines.append("|------|------|--------|-----------|------|------|")
        
        # 按大小降序排序
        sorted_vars = sorted(variables, key=lambda v: v.size, reverse=True)
        
        for i, var in enumerate(sorted_vars, 1):
            scope_cn = "全局" if var.scope == "global" else "局部"
            lines.append(
                f"| {i} | 0x{var.address} | `{var.name}` | {var.size} | "
                f".{var.section} | {scope_cn} |"
            )
        
        lines.append("")
        return '\n'.join(lines)
    
    def _generate_statistics(self) -> str:
        """生成统计信息"""
        lines = []
        lines.append("## 统计信息")
        lines.append("")
        
        total = self.analyzer.get_total_size()
        
        # 按大小分布
        size_ranges = [
            (0, 1, "1 byte"),
            (2, 4, "2-4 bytes"),
            (5, 8, "5-8 bytes"),
            (9, 16, "9-16 bytes"),
            (17, 32, "17-32 bytes"),
            (33, 64, "33-64 bytes"),
            (65, 128, "65-128 bytes"),
            (129, 512, "129-512 bytes"),
            (513, float('inf'), ">512 bytes")
        ]
        
        lines.append("### 变量大小分布")
        lines.append("")
        lines.append("| 大小范围 | .data | .bss | 合计 |")
        lines.append("|----------|-------|------|------|")
        
        for min_s, max_s, label in size_ranges:
            data_count = sum(1 for v in self.analyzer.data_vars if min_s <= v.size <= max_s)
            bss_count = sum(1 for v in self.analyzer.bss_vars if min_s <= v.size <= max_s)
            total_count = data_count + bss_count
            if total_count > 0:
                lines.append(f"| {label} | {data_count} | {bss_count} | {total_count} |")
        
        lines.append("")
        lines.append("### 内存占用汇总")
        lines.append("")
        lines.append(f"- **.data 区域**: {total['data']} bytes ({len(self.analyzer.data_vars)} 个变量)")
        lines.append(f"- **.bss 区域**: {total['bss']} bytes ({len(self.analyzer.bss_vars)} 个变量)")
        lines.append(f"- **总占用**: {total['total']} bytes")
        lines.append("")
        
        # 说明
        lines.append("## 说明")
        lines.append("")
        lines.append("- **.data**: 已初始化的全局/静态变量，占用 Flash 和 RAM")
        lines.append("- **.bss**: 未初始化的全局/静态变量，只占用 RAM，启动时清零")
        lines.append("- 变量名后缀如 `.4673` 是编译器生成的行号标识")
        lines.append("")
        
        return '\n'.join(lines)
    
    def generate_csv(self) -> str:
        """生成 CSV 格式"""
        lines = ["地址,变量名,大小(字节),区域,类型"]
        
        for var in self.analyzer.variables:
            lines.append(f"0x{var.address},{var.name},{var.size},{var.section},{var.scope}")
        
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='分析 TC32 ELF .lst 文件的 Data/BSS 区域变量',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本用法
  python analyze_data_bss.py project.lst
  
  # 指定输出文件
  python analyze_data_bss.py project.lst -o report.md
  
  # 生成 CSV 格式
  python analyze_data_bss.py project.lst -f csv -o vars.csv
  
  # 只显示大变量(>64 bytes)
  python analyze_data_bss.py project.lst --min-size 64
  
  # 只分析 bss 区域
  python analyze_data_bss.py project.lst --section bss
        """
    )
    
    parser.add_argument('input', help='输入的 .lst 文件路径')
    parser.add_argument('-o', '--output', help='输出文件路径')
    parser.add_argument('-f', '--format', choices=['md', 'csv'], default='md',
                       help='输出格式: md=Markdown (默认), csv=CSV')
    parser.add_argument('--min-size', type=int, default=32,
                       help='大变量阈值 (默认: 32 bytes)')
    parser.add_argument('--section', choices=['data', 'bss'],
                       help='只分析指定区域')
    parser.add_argument('-q', '--quiet', action='store_true',
                       help='静默模式，不输出到控制台')
    
    args = parser.parse_args()
    
    # 解析文件
    analyzer = LstAnalyzer(args.input)
    if not analyzer.parse():
        sys.exit(1)
    
    if not analyzer.variables:
        print("警告: 未找到 data/bss 变量")
        sys.exit(0)
    
    # 生成报告
    generator = ReportGenerator(analyzer)
    
    if args.format == 'csv':
        output = generator.generate_csv()
    else:
        output = generator.generate_markdown(args.min_size)
    
    # 输出
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(output)
        if not args.quiet:
            print(f"报告已保存: {args.output}")
    else:
        if not args.quiet:
            print(output)
    
    # 简要统计
    if not args.quiet:
        total = analyzer.get_total_size()
        print(f"\n分析完成:")
        print(f"  - .data: {len(analyzer.data_vars)} 个变量, {total['data']} bytes")
        print(f"  - .bss: {len(analyzer.bss_vars)} 个变量, {total['bss']} bytes")
        print(f"  - 总计: {len(analyzer.variables)} 个变量, {total['total']} bytes")


if __name__ == '__main__':
    main()
