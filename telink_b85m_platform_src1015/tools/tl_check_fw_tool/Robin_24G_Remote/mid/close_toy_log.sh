#!/bin/bash
# Windows Git Bash专用修复版

echo "========================================"
echo "      TOY_LOG_ENABLE 关闭工具"
echo "========================================"

# 获取当前目录（Windows兼容）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "脚本目录: $SCRIPT_DIR"

# 配置文件路径 - 使用相对路径
CONFIG_FILE="$SCRIPT_DIR/../../../../demo/vendor/Robin_24G_Remote/app_config.h"

# 检查文件是否存在
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 错误: 找不到文件 $CONFIG_FILE"
    echo "当前目录: $(pwd)"
    exit 1
fi

echo "✅ 找到文件: $CONFIG_FILE"

TARGET_MACRO="TOY_LOG_ENABLE"
NEW_VALUE="0"

# 查找目标行
echo ""
echo "🔍 查找目标行..."
LINE_INFO=$(grep -n "#define[[:space:]]*$TARGET_MACRO" "$CONFIG_FILE" | head -1)

if [ -z "$LINE_INFO" ]; then
    echo "❌ 错误: 未找到 $TARGET_MACRO"
    exit 1
fi

LINE_NUM=$(echo "$LINE_INFO" | cut -d: -f1)
ORIGINAL_LINE=$(echo "$LINE_INFO" | cut -d: -f2-)

echo "✅ 找到第 $LINE_NUM 行"
echo "原始内容: '$ORIGINAL_LINE'"

echo ""
echo "⚡ 执行精确替换..."

# 先创建一个临时文件
TEMP_FILE="${CONFIG_FILE}.tmp.$$"

# 逐行处理，只替换目标行
line_count=0
while IFS= read -r line || [ -n "$line" ]; do
    line_count=$((line_count + 1))
    
    if [ "$line_count" -eq "$LINE_NUM" ]; then
        # 使用bash正则匹配
        if [[ "$line" =~ ^(.*#define[[:space:]]+${TARGET_MACRO}[[:space:]]+)([0-9]+)(.*)$ ]]; then
            prefix="${BASH_REMATCH[1]}"
            suffix="${BASH_REMATCH[3]}"
            new_line="${prefix}${NEW_VALUE}${suffix}"
            echo "$new_line" >> "$TEMP_FILE"
            echo "✅ 已修改第 $LINE_NUM 行"
            echo "   原: $line"
            echo "   新: $new_line"
        else
            echo "$line" >> "$TEMP_FILE"
            echo "⚠ 警告: 无法解析该行，未修改"
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

if echo "$NEW_LINE" | grep -q "$TARGET_MACRO" && echo "$NEW_LINE" | grep -q "$NEW_VALUE"; then
    echo ""
    echo "========================================"
    echo "🎉 成功！精确替换完成"
    echo "========================================"
else
    echo ""
    echo "❌ 替换失败"
    exit 1
fi