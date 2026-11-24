# 当前固件交付

## 当前飞行基线

| 用途 | 镜像 | 烧录地址 |
|---|---|---|
| **正常飞行（唯一现行）** | [Robin_24G_Car_flight_control_baseline.bin](candidates/Robin_24G_Car_flight_control_baseline.bin) | `0x4000` |
| 无 OTA bootloader | [Robin_24G_Bootloader_no_ota.bin](candidates/Robin_24G_Bootloader_no_ota.bin) | `0x0` |
| 遥控器恢复镜像 | [Robin_24G_Remote_logkey.bin](candidates/Robin_24G_Remote_logkey.bin) | `0x0` |
| 六轴校准维护镜像 | [269A 维护固件](daily/Robin_24G_Car_maint_269a.bin) | `0x4000` |
| 定点温补地面诊断 | [Robin_24G_Car_ground_baro_fixed_bdt.bin](candidates/Robin_24G_Car_ground_baro_fixed_bdt.bin) | `0x4000` |

当前飞行基线：

```text
源码提交：29215e9
构建开关：-DC_FLIGHT_CONTROL_EN=1
气压：C_FLIGHT_BARO_EN=0（默认关闭）
字节数：24516
SHA256：C4DE867D9C8C0B0937382EABF50DAFD307FD43BDA2994A98DE43473FEAAB07F2
烧录地址：0x4000
```

该镜像已通过主机回归和正常飞行构建检查。**实飞记录属于归档的 `fb28cc3` 镜像**，不是这份新基线。

原 `fb28cc3` 实飞镜像归档：

```text
archive/Robin_24G_Car_flight_control_fb28cc3.bin
24948 字节
B9B98FF5B82105C150BFDD10078B9DD07987EBD3B0F065668E795610A6AF010C
```

其他气压飞行候选和重复的关闭版气压实测件已移到 [归档目录](archive/)，不再位于当前交付目录。

## 校验

在仓库根目录运行：

```powershell
python firmware_delivery\verify_hashes.py
```

完整文件、地址和哈希见 [BIN 清单](INVENTORY.md)。原始启动、遥控和机载固件保留在 [firmware_baseline](../firmware_baseline/)。

## 说明

- 1 Hz 气压测量和定点温补代码保留，默认不启用；
- 正常飞行仍使用 200 Hz 姿态控制、保护逻辑、混控和手动油门；
- 本轮未烧录、未操作实机。
