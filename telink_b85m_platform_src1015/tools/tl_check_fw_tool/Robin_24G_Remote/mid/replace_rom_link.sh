#!/bin/bash
# Windows Git Bash专用修复版

echo "========================================"
echo "      Link File Replace Tool"
echo "========================================"

# 获取当前目录（Windows兼容）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "脚本目录: $SCRIPT_DIR"

# 定义路径
SOURCE_FILE="$SCRIPT_DIR/../../../../project/tlsr_tc32/tc122x/link/boot_rom.link"
DEST_FILE="$SCRIPT_DIR/../../../../project/tlsr_tc32/tc122x/boot.link"

# 检查源文件是否存在
if [ ! -f "$SOURCE_FILE" ]; then
    echo "❌ 错误: 找不到源文件 $SOURCE_FILE"
    exit 1
fi

echo "✅ 找到源文件: $SOURCE_FILE"

# 执行替换 (使用 cp 覆盖，保留源文件)
echo "⚡ 正在用 boot_rom.link 替换 boot.link..."
cp -f "$SOURCE_FILE" "$DEST_FILE"

if [ $? -eq 0 ]; then
    echo "✅ 替换成功！"
else
    echo "❌ 替换失败"
    exit 1
fi