#!/bin/bash
# 测试版 Bin 生成工具使用示例

echo "======================================"
echo "测试版 - 生成示例 Bin 文件"
echo "======================================"
echo ""
echo "测试版特点:"
echo "  - efuse_dev_info_t: 只支持 toy_type, access_code_group, normal_tx_power"
echo "  - CfgParamUnion: 完整支持所有参数"
echo ""

# 1. SOP10 遥控器 -> Flash
echo "1. SOP10 遥控器 -> Flash"
python3 bin_gen_test.py --target flash --fw-type remote --output remote_flash.bin

# 2. SOP10 遥控车 -> EFuse
echo ""
echo "2. SOP10 遥控车 -> EFuse"
python3 bin_gen_test.py --target efuse --fw-type car10 --output car10_efuse.bin

# 3. SOP16 遥控车 -> Flash
echo ""
echo "3. SOP16 遥控车 -> Flash"
python3 bin_gen_test.py --target flash --fw-type car16 --output car16_flash.bin

# 4. 生成 JSON 模板
echo ""
echo "4. 生成 JSON 模板文件"
python3 bin_gen_test.py --template --fw-type remote --output template_remote.json
python3 bin_gen_test.py --template --fw-type car10 --output template_car10.json
python3 bin_gen_test.py --template --fw-type car16 --output template_car16.json

echo ""
echo "======================================"
echo "示例文件已生成:"
ls -la *.bin 2>/dev/null || echo "  (无 bin 文件)"
echo ""
echo "JSON 模板:"
ls -la template_*.json 2>/dev/null || echo "  (无模板文件)"
echo "======================================"
