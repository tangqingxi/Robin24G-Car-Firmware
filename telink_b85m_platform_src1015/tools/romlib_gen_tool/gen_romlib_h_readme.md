# 编译 FPGA 固件并生成 romlib.h

## 1. 更新 boot.link

默认 `boot.link` 是 flash 的 boot.link，请替换成 rom 的 boot.link。

- **源文件**：`project/tlsr_tc32/tc122x/link/boot_rom.link`
- **目标文件**：`project/tlsr_tc32/tc122x/boot.link`

## 2. 编译 FPGA 固件

选择 `Robin_24G_FPGA` 编译固件。

## 3. 生成 romlib.h

1. 用以下文件替换 romlib_gen_tool 中的列表文件：
   - **源文件**：`project/tlsr_tc32/tc122x/Robin_24G_FPGA/Robin_24G_FPGA.lst`
   - **目标文件**：`tools/romlib_gen_tool/rom_lib.lst`

2. 将 list 文件改名为 `rom_lib.lst`

3. 运行脚本生成 romlib.h：
   ```bash
   python romlib_gen.py
   ```

生成的 `romlib.h` 即可提供给其他人使用。
