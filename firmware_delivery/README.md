# 当前交付入口

> 本页只列当前该烧哪份、该用哪个命令、旧件去哪找。完整清单见
> [`INVENTORY.md`](INVENTORY.md)，历史说明见 [`README_DETAIL.md`](README_DETAIL.md)。

## 当前该烧哪份

| 用途 | 文件 | 字节 | SHA256 | 烧录地址 |
|---|---|---:|---|---|
| **地面 UART 诊断版** | [`candidates/Robin_24G_Car_ground_uart_diag.bin`](candidates/Robin_24G_Car_ground_uart_diag.bin) | 26164 | `4A8B851CF902EFDBEEDA12EEF46BE3494B2D6BF0200BA4EBF773CD2F6192C3D2` | `0x4000` |
| **正常飞行版（无板载日志）** | [`candidates/Robin_24G_Car_flight_control_normal.bin`](candidates/Robin_24G_Car_flight_control_normal.bin) | 24820 | `C9F94E2B9C2EFEFCD3272C6746BAD6C254F3B5C3FFD3246AD4C0692EAC62D917` | `0x4000` |
| **偏航对称候选正常版** | [`candidates/Robin_24G_Car_flight_control_yaw_sym.bin`](candidates/Robin_24G_Car_flight_control_yaw_sym.bin) | 24948 | `0015E5D76F7D78410FD50023D4547EC87BDFD62DDCD1556A8E6D8059237F263F` | `0x4000` |
| **偏航对称候选地面诊断版** | [`candidates/Robin_24G_Car_ground_uart_yaw_sym.bin`](candidates/Robin_24G_Car_ground_uart_yaw_sym.bin) | 26292 | `A8C7D563416D8EE45288C79442780A1F83180A6F817D27F5D669BE9B128C776F` | `0x4000` |
| **无 OTA bootloader** | [`candidates/Robin_24G_Bootloader_no_ota.bin`](candidates/Robin_24G_Bootloader_no_ota.bin) | 6844 | `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096` | `0x0` |
| **六轴校准维护件** | [`daily/Robin_24G_Car_maint_269a.bin`](daily/Robin_24G_Car_maint_269a.bin) | 17716 | `F8E59BD29D35C2D8138F9CE60D5202C32907DFB50C7B469EFF60DF123B5C5C38` | `0x4000` |
| **已验收回退件 412d6b5** | [`candidates/Robin_24G_Car_flight_control_alignfix.bin`](candidates/Robin_24G_Car_flight_control_alignfix.bin) | 24900 | `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087` | `0x4000` |
| K3 录制键遥控 | [`candidates/Robin_24G_Remote_logkey.bin`](candidates/Robin_24G_Remote_logkey.bin) | 19880 | `C11CD83F5D19AFE5289DE60561A4D338B5290BB64974B407EA9C57596DE96F09` | `0x0` |
| 日常 200 Hz 正式件 | [`daily/Robin_24G_Car_flight_control.bin`](daily/Robin_24G_Car_flight_control.bin) | 28500 | `AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60` | `0x4000` |

正常飞行版和地面诊断版都不做板载 Flash 记录；诊断版只走 UART 10 Hz。

## 常用烧录目录

当前具名副本保留在：

```text
telink_b85m_platform_src1015\project\tlsr_tc32\tc122x\Robin_24G_Car_Flash\
    Robin_24G_Car_flight_control_normal.bin
    Robin_24G_Car_ground_uart_diag.bin
    Robin_24G_Bootloader_no_ota.bin
    Robin_24G_Car_flight_control_alignfix.bin
```

这些文件与 `candidates/` 对应文件完整 SHA256 一致。通用 `Robin_24G_Car_Flash.bin` 重复副本已清理。

## 当前命令

校验交付镜像：

```powershell
Set-Location "E:\泡沫纸飞机\code"
python firmware_delivery\verify_hashes.py
```

BDT 日志导出：

```powershell
Set-Location "E:\泡沫纸飞机\code\tools"
```

工具回归：

```powershell
Set-Location "E:\泡沫纸飞机\code"
powershell -ExecutionPolicy Bypass -File tools\regression\run_key_regression.ps1
```

## 旧件去哪找

- 历史镜像：`firmware_delivery/archive/`
- 原始基线：`firmware_baseline/`
- 旧解锁诊断：`firmware_delivery/arm_debug/`
- 旧说明：`firmware_delivery/README_DETAIL.md`
- 全部 BIN 用途/哈希/地址/保留位置：`firmware_delivery/INVENTORY.md`

本轮只做仓库整理和校验，没有烧录，也没有改飞控、日志格式或校准算法。
