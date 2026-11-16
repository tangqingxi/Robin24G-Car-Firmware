
# resolve tool dir from the script's own location, so it works no matter where
# the build system invokes it from (telink_ide project dir or cmake build dir)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "*****************************************************"
echo "this is post build!! current configure is :$1"

os=$(uname)
echo "OS from uname is: ${os}"


echo $1
echo $2
# telink_ide 传入 (ConfigName ProjName), elf 名为 $2.elf;
# cmake 传入 (TargetName PlatformName), elf 名为 $1.elf, 二者兼容处理
if [ -f "$1.elf" ]; then
    SRC_ELF="$1.elf"
else
    SRC_ELF="$2.elf"
fi
tc32-elf-objcopy -v -O binary "$SRC_ELF" "$1.bin"
#../tl_check_fw2.exe  $1.bin
if [ "${os}" = "Linux" ] ; then
    echo "Linux OS"
    echo "check_fw in Linux..."
    chmod 755 "$SCRIPT_DIR/check_fw"
    "$SCRIPT_DIR/check_fw" "$1.bin"
else
    echo "Windows OS"
    echo "check_fw in Windows..."
    "$SCRIPT_DIR/tl_check_fw2.exe" "$1.bin"
fi
echo  "---------------------------  SDK version info ---------------------------"
str=$(grep -E "[\$]{3}[a-zA-Z0-9 _.]+[\$]{3}" --text -o $1.bin | sed 's/\$//g')
if [ -z "$str" ]; then
    echo "no SDK version found at the end of firmware, please check sdk_version.c and sdk_version.h"
else
    echo "$str"
fi
echo  "---------------------------  SDK version end  ---------------------------"
echo "**************** end of post build ******************"
