#!/bin/bash
# Bin 生成工具使用示例脚本
# 生成的文件名会自动加上烧录地址后缀:
#   - Flash: xxx_0x0f000.bin (烧录地址 0xF000)
#   - EFuse: xxx_0x00000.bin (烧录地址 0x00)

echo "======================================"
echo "生成示例 Bin 文件 (带烧录地址)"
echo "======================================"

# 1. 生成 SOP10 遥控器配置到 Flash (生成 example_remote_flash_0x0f000.bin)
echo ""
echo "1. SOP10 遥控器 -> Flash (256 bytes, 烧录地址 0xF000, 填充 0xFF)"
python3 bin_gen.py --target flash --fw-type remote --output example_remote_flash.bin

# 2. 生成 SOP10 遥控车配置到 EFuse (生成 example_car10_efuse_0x00000.bin)
echo ""
echo "2. SOP10 遥控车 -> EFuse (32 bytes, 烧录地址 0x00, 填充 0x00)"
python3 bin_gen.py --target efuse --fw-type car10 --output example_car10_efuse.bin

# 3. 生成 SOP16 遥控车配置到 Flash (生成 example_car16_flash_0x0f000.bin)
echo ""
echo "3. SOP16 遥控车 -> Flash (256 bytes, 烧录地址 0xF000, 填充 0xFF)"
python3 bin_gen.py --target flash --fw-type car16 --output example_car16_flash.bin

# 4. 生成 JSON 模板
echo ""
echo "4. 生成 JSON 模板文件"
python3 bin_gen.py --template --fw-type remote --output template_remote.json
python3 bin_gen.py --template --fw-type car10 --output template_car10.json
python3 bin_gen.py --template --fw-type car16 --output template_car16.json

echo ""
echo "======================================"
echo "示例文件已生成:"
echo "  - example_remote_flash_0x0f000.bin  (Flash, 烧录地址 0xF000)"
echo "  - example_car10_efuse_0x00000.bin   (EFuse, 烧录地址 0x00)"
echo "  - example_car16_flash_0x0f000.bin   (Flash, 烧录地址 0xF000)"
echo "  - template_remote.json"
echo "  - template_car10.json"
echo "  - template_car16.json"
echo "======================================"
