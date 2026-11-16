#!/bin/bash

# 定义目录路径
TARGET_DIR="./"

# 检查目录是否存在
if [ ! -d "$TARGET_DIR" ]; then
    echo "错误：目录 $TARGET_DIR 不存在"
    exit 1
fi

# 替换所有.c文件中的printf(为TOY_LOG(
echo "正在替换文件中的printf(为TOY_LOG(..."

# 方法A：使用find递归处理
find "$TARGET_DIR" -name "*.c" -type f -exec sed -i 's/printf(/TOY_LOG(/g' {} \;

# 方法B：如果需要处理.h文件等其他类型
# find "$TARGET_DIR" \( -name "*.c" -o -name "*.h" -o -name "*.cpp" \) -type f -exec sed -i 's/printf(/TOY_LOG(/g' {} \;

echo "替换完成！"