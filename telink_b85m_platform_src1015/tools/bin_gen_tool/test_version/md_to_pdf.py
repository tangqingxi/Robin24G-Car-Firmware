#!/usr/bin/env python3
"""
将 output 目录下的所有 markdown 文件转换为 PDF
PDF 文件生成在 markdown 文件旁边

使用方法:
    python3 md_to_pdf.py                    # 转换 output 目录下的所有 md 文件
    python3 md_to_pdf.py /path/to/dir       # 转换指定目录下的所有 md 文件
    python3 md_to_pdf.py --compact          # 使用紧凑模式（简化配置文件列表）

依赖安装(方式一 - Python 库):
    pip3 install markdown weasyprint
    
依赖安装(方式二 - 系统包):
    sudo apt-get install python3-markdown python3-weasyprint
    
依赖安装(方式三 - 使用 pandoc):
    sudo apt-get install pandoc texlive-xetex  # 或使用 lualatex/pdflatex
"""

import os
import sys
import glob
import subprocess
import re
from pathlib import Path

# 尝试导入依赖
try:
    import markdown
    from weasyprint import HTML, CSS
    WEASYPRINT_AVAILABLE = True
except ImportError:
    WEASYPRINT_AVAILABLE = False


def check_pandoc():
    """检查是否安装了 pandoc"""
    try:
        result = subprocess.run(['pandoc', '--version'], 
                              capture_output=True, text=True, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def simplify_config_list(md_content: str) -> str:
    """
    简化"配置文件列表"部分，使表格更紧凑
    
    原始表格有4列：文件名、测试类型、参数设置、说明
    简化后保留2列：文件名、场景说明
    """
    lines = md_content.split('\n')
    result = []
    i = 0
    in_config_table = False
    
    while i < len(lines):
        line = lines[i]
        
        # 检测"配置文件列表"标题
        if re.match(r'^## 配置文件列表', line):
            in_config_table = True
            result.append(line)
            result.append('')  # 空行
            result.append('**说明：本列表仅显示文件名和场景说明，详细参数请查看下文"参数配置说明"部分。**')
            i += 1
            continue
        
        # 如果在配置文件列表区域内，处理表格
        if in_config_table:
            # 检测表格分隔符行 |--------|----------|...
            if re.match(r'^\|[-:]+\|', line) and i > 0 and '| 文件名 |' in lines[i-1]:
                # 将表头改为只有文件名和说明
                result.append('| 文件名 | 说明 |')
                result.append('|--------|------|')
                i += 1
                # 跳过原始的分隔行
                if i < len(lines) and re.match(r'^\|[-:]+\|', lines[i]):
                    i += 1
                continue
            
            # 处理表格行
            if line.startswith('|') and '| 文件名 |' not in line:
                parts = line.split('|')
                if len(parts) >= 5:
                    # 提取文件名（第2列）和说明（最后一列）
                    filename = parts[1].strip()
                    description = parts[-2].strip() if parts[-2].strip() else parts[4].strip() if len(parts) > 4 else ""
                    
                    # 简化文件名：去掉 _0x0f000.bin 后缀
                    filename = re.sub(r'_0x[0-9a-fA-F]+\.bin$', '', filename)
                    
                    result.append(f"| {filename} | {description} |")
                    i += 1
                    continue
            
            # 检测表格结束（空行后不是表格行）
            if line.strip() == '' and i + 1 < len(lines):
                if not lines[i + 1].startswith('|') and not lines[i + 1].startswith('#'):
                    in_config_table = False
        
        result.append(line)
        i += 1
    
    return '\n'.join(result)


# 简单的 CSS 样式，使 PDF 输出更美观
DEFAULT_CSS = """
@page {
    size: A4;
    margin: 2.5cm;
    @bottom-center {
        content: counter(page);
        font-size: 10pt;
    }
}

body {
    font-family: "Noto Sans CJK SC", "WenQuanYi Micro Hei", "Source Han Sans SC", "DejaVu Sans", sans-serif;
    font-size: 11pt;
    line-height: 1.6;
    color: #333;
}

h1 {
    font-size: 20pt;
    color: #1a1a1a;
    border-bottom: 2px solid #2c3e50;
    padding-bottom: 0.3cm;
    margin-top: 0;
}

h2 {
    font-size: 16pt;
    color: #2c3e50;
    margin-top: 1.2cm;
    border-bottom: 1px solid #ddd;
    padding-bottom: 0.2cm;
}

h3 {
    font-size: 13pt;
    color: #34495e;
    margin-top: 0.8cm;
}

/* 配置文件列表表格使用更小的字体 */
h2:has(+ p + table), h2:has(+ table) {
    page-break-before: auto;
}

/* 所有表格样式 */
table {
    width: 100%;
    border-collapse: collapse;
    margin: 0.5cm 0;
    font-size: 9pt;
    page-break-inside: avoid;
}

th, td {
    border: 1px solid #ddd;
    padding: 6px 8px;
    text-align: left;
    vertical-align: top;
}

th {
    background-color: #f5f5f5;
    font-weight: bold;
}

tr:nth-child(even) {
    background-color: #fafafa;
}

/* 让表格中的长文字自动换行 */
td {
    word-wrap: break-word;
    max-width: 0;
}

/* 代码样式 */
code {
    background-color: #f4f4f4;
    padding: 2px 6px;
    border-radius: 3px;
    font-family: "Consolas", "Monaco", "DejaVu Sans Mono", monospace;
    font-size: 9pt;
}

pre {
    background-color: #f8f8f8;
    padding: 12px;
    border-radius: 5px;
    overflow-x: auto;
    border-left: 4px solid #2c3e50;
    font-size: 9pt;
    page-break-inside: avoid;
}

pre code {
    background: none;
    padding: 0;
}

ul, ol {
    margin: 0.5cm 0;
    padding-left: 1.5cm;
}

li {
    margin: 0.2cm 0;
}

blockquote {
    border-left: 4px solid #ccc;
    margin: 0.5cm 0;
    padding-left: 1cm;
    color: #666;
}

a {
    color: #3498db;
    text-decoration: none;
}

/* 参数配置说明部分不要分页 */
h3 {
    page-break-after: avoid;
}

h3 + table {
    page-break-before: avoid;
}
"""


def convert_md_to_pdf_weasyprint(md_file_path: str, compact: bool = False) -> bool:
    """
    使用 weasyprint 将 markdown 转换为 PDF
    """
    md_path = Path(md_file_path)
    pdf_path = md_path.with_suffix('.pdf')
    
    try:
        with open(md_path, 'r', encoding='utf-8') as f:
            md_content = f.read()
        
        # 如果需要，简化配置文件列表
        if compact:
            md_content = simplify_config_list(md_content)
        
        html_content = markdown.markdown(
            md_content,
            extensions=['tables', 'fenced_code', 'toc', 'nl2br']
        )
        
        full_html = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>{md_path.stem}</title>
</head>
<body>
{html_content}
</body>
</html>"""
        
        HTML(string=full_html).write_pdf(
            str(pdf_path),
            stylesheets=[CSS(string=DEFAULT_CSS)]
        )
        
        return True
        
    except Exception as e:
        print(f"  [weasyprint 失败] {e}")
        return False


def convert_md_to_pdf_pandoc(md_file_path: str, compact: bool = False) -> bool:
    """
    使用 pandoc 将 markdown 转换为 PDF
    """
    md_path = Path(md_file_path)
    pdf_path = md_path.with_suffix('.pdf')
    
    try:
        with open(md_path, 'r', encoding='utf-8') as f:
            md_content = f.read()
        
        # 如果需要，简化配置文件列表
        if compact:
            md_content = simplify_config_list(md_content)
        
        # 创建临时 markdown 文件
        temp_md = md_path.with_suffix('.temp.md')
        with open(temp_md, 'w', encoding='utf-8') as f:
            f.write(md_content)
        
        # 使用 pandoc 转换为 PDF
        cmd = [
            'pandoc',
            str(temp_md),
            '-o', str(pdf_path),
            '-V', 'geometry:margin=2.5cm',
            '--toc'
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        # 删除临时文件
        temp_md.unlink(missing_ok=True)
        
        if result.returncode != 0:
            print(f"  [pandoc 失败] {result.stderr}")
            return False
        
        return True
        
    except Exception as e:
        print(f"  [pandoc 失败] {e}")
        return False


def convert_md_to_pdf(md_file_path: str, use_weasyprint: bool = True, compact: bool = False) -> bool:
    """
    将单个 markdown 文件转换为 PDF
    
    Args:
        md_file_path: markdown 文件路径
        use_weasyprint: 是否优先使用 weasyprint
        compact: 是否使用紧凑模式（简化配置文件列表）
        
    Returns:
        bool: 转换是否成功
    """
    md_path = Path(md_file_path)
    
    if not md_path.exists():
        print(f"  [跳过] 文件不存在: {md_file_path}")
        return False
    
    pdf_path = md_path.with_suffix('.pdf')
    
    mode_str = "紧凑" if compact else "完整"
    
    # 尝试使用 weasyprint
    if use_weasyprint and WEASYPRINT_AVAILABLE:
        if convert_md_to_pdf_weasyprint(md_file_path, compact=compact):
            print(f"  [成功] {md_path.name} -> {pdf_path.name} (weasyprint, {mode_str})")
            return True
    
    # 尝试使用 pandoc
    if check_pandoc():
        if convert_md_to_pdf_pandoc(md_file_path, compact=compact):
            print(f"  [成功] {md_path.name} -> {pdf_path.name} (pandoc, {mode_str})")
            return True
    
    print(f"  [失败] {md_path.name}: 没有可用的 PDF 转换工具")
    return False


def main():
    """主函数"""
    # 解析命令行参数
    compact_mode = False
    output_dir = None
    
    for arg in sys.argv[1:]:
        if arg == '--compact' or arg == '-c':
            compact_mode = True
        elif not arg.startswith('-'):
            output_dir = Path(arg).resolve()
    
    # 获取脚本所在目录
    script_dir = Path(__file__).parent.resolve()
    
    # 默认输出目录
    if output_dir is None:
        output_dir = script_dir / "output"
    
    if not output_dir.exists():
        print(f"错误: 目录不存在: {output_dir}")
        sys.exit(1)
    
    print("=" * 60)
    print("Markdown 转 PDF 工具")
    print("=" * 60)
    print(f"扫描目录: {output_dir}")
    print(f"模式: {'紧凑' if compact_mode else '完整'}")
    if compact_mode:
        print("说明: 配置文件列表将只显示文件名和场景说明")
    
    # 检查可用的转换工具
    if WEASYPRINT_AVAILABLE:
        print("使用工具: weasyprint")
    elif check_pandoc():
        print("使用工具: pandoc")
    else:
        print("\n错误: 没有可用的 PDF 转换工具")
        print("请安装以下依赖之一:")
        print("  1. pip3 install markdown weasyprint")
        print("  2. sudo apt-get install pandoc texlive-xetex")
        sys.exit(1)
    
    print()
    
    # 查找所有 markdown 文件
    md_files = list(output_dir.rglob("*.md"))
    
    if not md_files:
        print(f"未找到 markdown 文件")
        sys.exit(0)
    
    print(f"找到 {len(md_files)} 个 markdown 文件:")
    print()
    
    # 转换统计
    success_count = 0
    fail_count = 0
    
    # 按目录分组显示
    current_dir = None
    for md_file in sorted(md_files):
        file_dir = md_file.parent
        if file_dir != current_dir:
            current_dir = file_dir
            relative_dir = file_dir.relative_to(output_dir)
            print(f"\n[{relative_dir}]")
        
        if convert_md_to_pdf(str(md_file), use_weasyprint=WEASYPRINT_AVAILABLE, compact=compact_mode):
            success_count += 1
        else:
            fail_count += 1
    
    print()
    print("=" * 60)
    print(f"转换完成: 成功 {success_count} 个, 失败 {fail_count} 个")
    print("=" * 60)


if __name__ == "__main__":
    main()
