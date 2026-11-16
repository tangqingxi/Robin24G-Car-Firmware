#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# 1. 根据操作系统设置路径
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || "$OSTYPE" == "win32" ]]; then
    # Windows 环境 (通常在 Git Bash 或 MSYS2 中运行)
    TELINK_IDE_EXE="C:/TelinkIoTStudio/TelinkIoTStudio.exe"
    PATCH_TELINK_IOT_STUDIO="C:/TelinkIoTStudio"
    # Windows 下建议使用绝对路径或确保相对于当前驱动器的路径正确
    PATCH_WORKSPECE="file:/$(cygpath -m "$SCRIPT_DIR/../../../../../telink_IDE")"
    PATCH_SDK="file:/$(cygpath -m "$SCRIPT_DIR/../../..")"
else
    # Linux 环境
    TELINK_IDE_EXE="/home/matter/ide2502/TelinkIoTStudio"
    PATCH_TELINK_IOT_STUDIO="/home/matter/ide2502"
    PATCH_WORKSPECE="$SCRIPT_DIR/../../../../../telink_IDE"
    PATCH_SDK="$SCRIPT_DIR/../../.."
fi

./mid/replace_rom_link.sh
./mid/replace_to_remote.sh
./mid/enable_toy_log.sh

mcu_type="tc122x"
project_name="TC_PLATFORM_SDK_122X"
project_suffix="Robin_24G_Remote_FPGA"

# 项目的具体路径
PROJECT_PATH="$PATCH_SDK/project/tlsr_tc32/$mcu_type"
#清理路径上的双斜杠
#PROJECT_PATH=${PROJECT_PATH//\/\//\/}

BUILD_TARGET="$project_name/$project_suffix"

echo "正在检测操作系统: $OSTYPE"
echo "使用 IDE 路径: $TELINK_IDE_EXE"
echo "使用 PROJECT_PATH 路径: $PROJECT_PATH"
echo "使用 BUILD_TARGET 路径: $BUILD_TARGET"

# 2. 清理 (调用项目 Makefile 进行清理)
echo "正在清理工程..."
CLEAN_PATH="$PROJECT_PATH/$project_suffix"
#清理路径上的双斜杠
#CLEAN_PATH=${CLEAN_PATH//\/\//\/}
#echo "使用 CLEAN_PATH 路径: $CLEAN_PATH"
#make clean -C "$CLEAN_PATH" > /dev/null 2>&1

# 3. 编译 (Headless Build)
echo "正在启动 Headless 编译: $BUILD_TARGET"

"$TELINK_IDE_EXE" --launcher.suppressErrors -nosplash \
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
    -data "$PATCH_WORKSPECE" \
    -import "$PROJECT_PATH" \
    -cleanBuild "$BUILD_TARGET"

if [ $? -eq 0 ]; then
    echo "编译完成成功。"
else
    echo "编译失败，请检查日志。"
    exit 1
fi
