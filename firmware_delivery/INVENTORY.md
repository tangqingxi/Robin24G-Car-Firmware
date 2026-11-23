# BIN 现状清单（2026-11-22 整理后）

地址为烧录地址；`N/A` 表示原始数据或回归夹具，不直接烧录。SHA256 为完整值。

| 文件 | 用途 | 字节 | SHA256 | 地址 | 保留位置 |
|---|---|---:|---|---|---|
| `data/archive/flight_log_dumps_20261122/flight_log_clean.bin` | 主机回归完整会话原始 dump（非实飞） | 53248 | `DF03276D39786A1C113634751DD938A36317ECA341573E73658F17F0DA87A0D8` | N/A | 原始数据归档 |
| `data/archive/flight_log_dumps_20261122/flight_log_dump.bin` | 坏页/未封口恢复回归 dump | 53248 | `946BB2D6F0E7695CCA6D35FE263B0E985D91F5FA8CCA068D3F86C21E6DABF2F9` | N/A | 原始数据归档 |
| `data/archive/flight_log_dumps_20261122/flight_log_integration.bin` | 真实 FlightLoop 日志集成 dump | 53248 | `2B65BE2CC5D717301097624AEB0C0A602A832E406EF4FB2F14A99633F3A00F77` | N/A | 原始数据归档 |
| `data/archive/flight_log_dumps_20261122/flight_log_locked.bin` | 锁定静置+急停会话 dump | 53248 | `7A69A61C414E2995A60662A87253D5BF790776F0E1580543F1015F4A305D185B` | N/A | 原始数据归档 |
| `data/archive/flight_log_dumps_20261122/flight_log_unlock.bin` | 解锁后失联/超时会话 dump | 53248 | `D7397B87D466F3F01E5724EAA6325C700B35C7EE6FC127446D9E39AB615294BC` | N/A | 原始数据归档 |
| `firmware_baseline/Robin_24G_Bootloader_Flash/Robin_24G_Bootloader_Flash.bin` | 原始旧 bootloader 基线 | 12756 | `46506F44720D2A56C04E6A3E69A54533DC5759F1B2847047359DD9ED13A96D52` | 0x0 | 原始基线 |
| `firmware_baseline/Robin_24G_Car_Flash/Robin_24G_Car_Flash.bin` | 原始车端固件基线 | 34404 | `504042715AF62E84EB8580E35016FF0434992BC0867510C920425424BC9421DF` | 0x4000 | 原始基线 |
| `firmware_baseline/Robin_24G_Remote_Flash/Robin_24G_Remote_Flash.bin` | 原始遥控固件基线 | 19908 | `DDCC427C58DEC67C2BFD25C39BDE0FE0972F8ECCDF21CE2449C77A6D035BAAB4` | 0x0 | 原始基线 |
| `firmware_delivery/archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin` | 历史姿态台架重建件 | 24852 | `54CE488F2F0AB045A924CFF2CFF25772798D0F02D12F841AD88276746D781BD1` | 0x4000 | 归档 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control_b3cb56f.bin` | 拆分前 200Hz 回退件 | 35108 | `F8607D150B56FCDABEDC10F1DE8AE514039CED1CCEB6F500DC5B35A3C8824BF5` | 0x4000 | 归档/回退 |
| `firmware_delivery/archive/Robin_24G_Car_rc_bringup.bin` | 历史 RC 联调件 | 18228 | `476628BD9A3FE679E857104A36956E611ECCE775E327FD8D3CC76235EE152F0F` | 0x4000 | 归档 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400.bin` | 历史 400Hz L400 件 | 25156 | `3F8F62093D97F48E98F8F1B287C05011B4CDCA534CB74E49657F19AD547AA821` | 0x4000 | 归档 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400_keycap.bin` | 历史 400Hz 按键捕获件（由 data 归档迁入） | 27076 | `4571D27456AAD9A86D231243484EB3043B75C58CD882613B46E5FFD1E7D13DDF` | 0x4000 | 归档唯一件 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400_sched.bin` | 历史 400Hz 调度件（由 data 归档迁入） | 26644 | `CAFEAC6C8551889CD7F81225B204408841BA331D878BD4E6D9D681368166BF16` | 0x4000 | 归档唯一件 |
| `firmware_delivery/arm_debug/Robin_24G_Car_flight_arm_debug.bin` | 旧解锁诊断件 | 33876 | `DFDBE4E4C6888388EAC037DF88333A55E796A7C054A80C2C045A2705458A3955` | 0x4000 | 归档诊断 |
| `firmware_delivery/candidates/Robin_24G_Bootloader_no_ota.bin` | 当前无 OTA bootloader | 6844 | `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096` | 0x0 | 交付/当前 |
| `firmware_delivery/candidates/Robin_24G_Car_flight_arm_debug_alignfix.bin` | 对齐修复解锁诊断件 | 30452 | `1FA9323EF35333E201C87A32E4AE6B00F87508C0CC725FFE93BDA10817A1E189` | 0x4000 | 交付诊断 |
| `firmware_delivery/candidates/Robin_24G_Car_flight_control_alignfix.bin` | 已无桨验收回退件 412d6b5 | 24900 | `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087` | 0x4000 | 交付/回退 |
| `firmware_delivery/candidates/Robin_24G_Car_flight_control_log.bin` | 当前 K3 启录日志应用 | 27636 | `5557BF95888010960A600D4FBE697596C9F24401D02AB232204ADA324CF11AEA` | 0x4000 | 交付/当前 |
| `firmware_delivery/candidates/Robin_24G_Car_flight_log_debug.bin` | 当前日志时序诊断版 | 33844 | `04D953C791A4367DBE7D2EC974BD29EB235439C7368961F834254F7A964427F5` | 0x4000 | 交付/诊断 |
| `firmware_delivery/candidates/Robin_24G_Remote_logkey.bin` | K3 录制键和未映射键修复遥控 | 19880 | `C11CD83F5D19AFE5289DE60561A4D338B5290BB64974B407EA9C57596DE96F09` | 0x0 | 交付/当前 |
| `firmware_delivery/daily/Robin_24G_Car_flight_control.bin` | 日常 200Hz 无日志正式件 | 28500 | `AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60` | 0x4000 | 日常 |
| `firmware_delivery/daily/Robin_24G_Car_maint_269a.bin` | 六轴校准维护件 0x269A | 17716 | `F8E59BD29D35C2D8138F9CE60D5202C32907DFB50C7B469EFF60DF123B5C5C38` | 0x4000 | 维护 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Bootloader_no_ota.bin` | 常用烧录目录 bootloader 具名副本 | 6844 | `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096` | 0x0 | 常用烧录目录 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Car_flight_control_alignfix.bin` | 常用烧录目录回退件具名副本 | 24900 | `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087` | 0x4000 | 常用烧录目录 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Car_flight_control_log.bin` | 常用烧录目录当前日志应用具名副本 | 27636 | `5557BF95888010960A600D4FBE697596C9F24401D02AB232204ADA324CF11AEA` | 0x4000 | 常用烧录目录 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Car_flight_log_debug.bin` | 常用烧录目录诊断版具名副本 | 33844 | `04D953C791A4367DBE7D2EC974BD29EB235439C7368961F834254F7A964427F5` | 0x4000 | 常用烧录目录 |
| `tools/regression/fixtures/flight_log_v1_history.bin` | 由历史 CSV 重建的 v1 导出夹具（非实飞原始 dump） | 53248 | `C54EA380D7790352DB310C9757A8F030A1C4F5480FA4F58AC3F22280C0E62B74` | N/A | 回归夹具 |

| `data/archive/flight_log_real_20261123_read.bin` | 实板第一趟 BDT 原始读取（稀疏问题证据） | 53248 | `9453D6FF940FB25057A6FDB4C6684638028BCF2EA394A2E7AE6605F23DB9C57A` | N/A | 原始数据归档 |

| `firmware_delivery/candidates/Robin_24G_Car_flight_log_trace.bin` | 第227条路径诊断版（FLQ/FLD/FLS/FLW/FLX） | 34180 | `404956D5C367FB65718F9CB288939A96F77042F2AA89F94C8BFD2759B015877E` | 0x4000 | 诊断/临时 |

| `data/archive/flight_log_real_20261123_second_read.bin` | 实板第二趟 BDT 原始读取（同样停在第226条） | 53248 | `3425DE69F6C2D9277711B0A030E8C7993AF8819DF51AE75751D222986CB0FF71` | N/A | 原始数据归档 |

## 清理与重复核对

- 删除 `Robin_24G_Car_Flash.bin` 通用构建副本：与当前日志具名副本 SHA256 完全一致。
- 以下为“交付校验件 + 常用烧录目录具名副本”的有意双份，已逐对全 SHA256 核对：
  - `candidates/Robin_24G_Car_flight_control_log.bin` ↔ 烧录目录同名文件
  - `candidates/Robin_24G_Car_flight_log_debug.bin` ↔ 烧录目录同名文件
  - `candidates/Robin_24G_Bootloader_no_ota.bin` ↔ 烧录目录同名文件
  - `candidates/Robin_24G_Car_flight_control_alignfix.bin` ↔ 烧录目录同名文件
- 除以上四对外，仓库内不再有相同 SHA256 的 BIN。
- `data/archive/.../firmware_builds` 的两份唯一历史镜像已迁入 `firmware_delivery/archive/`。

## 数据

根目录 `data/*.json` 已迁到 `data/calibration/`，历史日志/导出文本迁到 `data/logs/`；明细见 `data/MIGRATION_20261122.md`。`data/archive/` 未动。
