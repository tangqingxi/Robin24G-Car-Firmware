#!/bin/bash
# Windows Git Bash专用修复版

echo "========================================"
echo "      FW_TYPE 修改工具 - Windows修复版"
echo "========================================"

# 获取当前目录（Windows兼容）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "脚本目录: $SCRIPT_DIR"

# 配置文件路径 - 使用相对路径
CONFIG_FILE="$SCRIPT_DIR/../../../../demo/vendor/Robin_24G_Car/app_config.h"

# 检查文件是否存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 错误: 找不到文件 $CONFIG_FILE"
    echo "当前目录: $(pwd)"
    find . -name "cfg_common.h" 2>/dev/null
    exit 1
fi

echo "✅ 找到文件: $CONFIG_FILE"

NEW_VALUE="FW_TYPE_S16_CAR"

# 查找目标行
echo ""
echo "🔍 查找目标行..."
LINE_INFO=$(grep -n "DEFAULT_FW_TYPE" "$CONFIG_FILE" | head -1)

if [ -z "$LINE_INFO" ]; then
    echo "❌ 错误: 未找到 DEFAULT_FW_TYPE"
    exit 1
fi

LINE_NUM=$(echo "$LINE_INFO" | cut -d: -f1)
ORIGINAL_LINE=$(echo "$LINE_INFO" | cut -d: -f2-)

echo "✅ 找到第 $LINE_NUM 行"
echo "原始内容: '$ORIGINAL_LINE'"

# 显示原始空格（调试用）
echo "原始空格分析:"
echo -n "   " && echo "$ORIGINAL_LINE" | sed 's/./& /g'

# 方法1: 使用sed直接替换（最简单可靠）
echo ""
echo "⚡ 执行精确替换..."

# 先创建一个临时文件
TEMP_FILE="${CONFIG_FILE}.tmp.$$"

# 逐行处理，只替换目标行
line_count=0
while IFS= read -r line; do
    line_count=$((line_count + 1))
    
    if [ "$line_count" -eq "$LINE_NUM" ]; then
        # 这是目标行，只替换FW_TYPE部分
        # 使用bash内置字符串操作
        if [[ "$line" =~ ^(.*)(FW_TYPE_[A-Z0-9_]+)(.*)$ ]]; then
            prefix="${BASH_REMATCH[1]}"
            suffix="${BASH_REMATCH[3]}"
            new_line="${prefix}${NEW_VALUE}${suffix}"
            echo "$new_line" >> "$TEMP_FILE"
            echo "✅ 已修改第 $LINE_NUM 行"
            echo "   原: $line"
            echo "   新: $new_line"
        else
            # 如果没有匹配到FW_TYPE模式，保留原行
            echo "$line" >> "$TEMP_FILE"
        fi
    else
        echo "$line" >> "$TEMP_FILE"
    fi
done < "$CONFIG_FILE"

# 替换原文件
mv "$TEMP_FILE" "$CONFIG_FILE"

# 验证结果
echo ""
echo "✅ 检查结果..."
NEW_LINE=$(sed -n "${LINE_NUM}p" "$CONFIG_FILE")
echo "新内容: '$NEW_LINE'"

if echo "$NEW_LINE" | grep -q "$NEW_VALUE"; then
    echo ""
    echo "========================================"
    echo "🎉 成功！精确替换完成"
    echo "========================================"
    echo "修改前: '$ORIGINAL_LINE'"
    echo "修改后: '$NEW_LINE'"
    
    # 检查空格是否相同
    ORIG_SPACE=$(echo "$ORIGINAL_LINE" | sed 's/FW_TYPE_[A-Z0-9_]*.*//')
    NEW_SPACE=$(echo "$NEW_LINE" | sed 's/FW_TYPE_[A-Z0-9_]*.*//')
    
    if [ "$ORIG_SPACE" = "$NEW_SPACE" ]; then
        echo "✅ 所有空格格式完全保留"
    else
        echo "⚠ 注意: 空格格式可能有变化"
        echo "原空格: '$ORIG_SPACE'"
        echo "新空格: '$NEW_SPACE'"
    fi
    echo "========================================"
else
    echo ""
    echo "❌ 替换失败"
    echo "原内容: $ORIGINAL_LINE"
    echo "新内容: $NEW_LINE"
    exit 1
fi