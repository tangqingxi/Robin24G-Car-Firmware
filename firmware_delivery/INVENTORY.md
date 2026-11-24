# BIN 清单

以下为仓库中的实际文件。烧录地址仅用于固件。

## 当前交付与烧录副本

| 文件 | 用途 | 字节 | SHA256 | 烧录地址 |
|---|---|---:|---|---|
| `firmware_delivery/candidates/Robin_24G_Car_flight_control_baseline.bin` | 当前实飞基线 | 24516 | `C4DE867D9C8C0B0937382EABF50DAFD307FD43BDA2994A98DE43473FEAAB07F2` | 0x4000 |
| `firmware_delivery/candidates/Robin_24G_Bootloader_no_ota.bin` | 无 OTA bootloader | 6844 | `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096` | 0x0 |
| `firmware_delivery/candidates/Robin_24G_Remote_logkey.bin` | 遥控器恢复镜像 | 19880 | `C11CD83F5D19AFE5289DE60561A4D338B5290BB64974B407EA9C57596DE96F09` | 0x0 |
| `firmware_delivery/daily/Robin_24G_Car_maint_269a.bin` | 六轴校准维护镜像 | 17716 | `F8E59BD29D35C2D8138F9CE60D5202C32907DFB50C7B469EFF60DF123B5C5C38` | 0x4000 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Car_flight_control_baseline.bin` | 当前实飞基线的烧录副本 | 24516 | `C4DE867D9C8C0B0937382EABF50DAFD307FD43BDA2994A98DE43473FEAAB07F2` | 0x4000 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Bootloader_no_ota.bin` | 无 OTA bootloader的烧录副本 | 6844 | `4BA1B269A24779D664360B1F405374975241E86E7D90291CB904EBB41D014096` | 0x0 |
| `firmware_delivery/candidates/Robin_24G_Car_ground_baro_fixed_bdt.bin` | 实板通过的定点温补诊断 | 29812 | `FACF8892CDEF9B2E2244807A80A83A8EE992C6DBF9E57878474536F163C920AF` | 0x4000 |
| `telink_b85m_platform_src1015/project/tlsr_tc32/tc122x/Robin_24G_Car_Flash/Robin_24G_Car_ground_baro_fixed_bdt.bin` | 实板通过的定点温补诊断的烧录副本 | 29812 | `FACF8892CDEF9B2E2244807A80A83A8EE992C6DBF9E57878474536F163C920AF` | 0x4000 |

## 归档固件

| 文件 | 用途 | 字节 | SHA256 | 烧录地址 |
|---|---|---:|---|---|
| `firmware_delivery/archive/Robin_24G_Car_flight_control_alignfix.bin` | 已通过无桨验收的回退件 | 24900 | `008FB301A43782E1F74046C755FC8D942229C6468D68F18057C5E73AF8449087` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_arm_debug_alignfix.bin` | 对齐修复版解锁诊断 | 30452 | `1FA9323EF35333E201C87A32E4AE6B00F87508C0CC725FFE93BDA10817A1E189` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_ground_uart_diag.bin` | 偏航取整修正前地面 UART 对照 | 26164 | `4A8B851CF902EFDBEEDA12EEF46BE3494B2D6BF0200BA4EBF773CD2F6192C3D2` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_ground_uart_yaw_sym.bin` | 偏航取整修正后地面 UART 对照 | 26292 | `A8C7D563416D8EE45288C79442780A1F83180A6F817D27F5D669BE9B128C776F` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control.bin` | 旧 200 Hz 飞控交付件 | 28500 | `AA3D53590D2C366EE203B9840207ABCA4E434E5281C77A72FB9F4C59AFEEAB60` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_arm_debug.bin` | 旧解锁诊断交付件 | 33876 | `DFDBE4E4C6888388EAC037DF88333A55E796A7C054A80C2C045A2705458A3955` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control_b3cb56f.bin` | 拆分前 200 Hz 飞控回退 | 35108 | `F8607D150B56FCDABEDC10F1DE8AE514039CED1CCEB6F500DC5B35A3C8824BF5` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400.bin` | 历史 400 Hz 联调 | 25156 | `3F8F62093D97F48E98F8F1B287C05011B4CDCA534CB74E49657F19AD547AA821` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_rc_bringup.bin` | 历史遥控联调 | 18228 | `476628BD9A3FE679E857104A36956E611ECCE775E327FD8D3CC76235EE152F0F` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_att_bench_dad49ff_rebuilt.bin` | 历史姿态台架重建件 | 24852 | `54CE488F2F0AB045A924CFF2CFF25772798D0F02D12F841AD88276746D781BD1` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400_keycap.bin` | 历史 400 Hz 按键捕获 | 27076 | `4571D27456AAD9A86D231243484EB3043B75C58CD882613B46E5FFD1E7D13DDF` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_rc_loop400_sched.bin` | 历史 400 Hz 调度 | 26644 | `CAFEAC6C8551889CD7F81225B204408841BA331D878BD4E6D9D681368166BF16` | 0x4000 |
| `firmware_baseline/Robin_24G_Car_Flash/Robin_24G_Car_Flash.bin` | 原始机载固件 | 34404 | `504042715AF62E84EB8580E35016FF0434992BC0867510C920425424BC9421DF` | 0x4000 |
| `firmware_baseline/Robin_24G_Bootloader_Flash/Robin_24G_Bootloader_Flash.bin` | 原始启动固件 | 12756 | `46506F44720D2A56C04E6A3E69A54533DC5759F1B2847047359DD9ED13A96D52` | 0x0 |
| `firmware_baseline/Robin_24G_Remote_Flash/Robin_24G_Remote_Flash.bin` | 原始遥控固件 | 19908 | `DDCC427C58DEC67C2BFD25C39BDE0FE0972F8ECCDF21CE2449C77A6D035BAAB4` | 0x0 |
| `firmware_delivery/archive/Robin_24G_Car_ground_baro_bdt.bin` | 气压升降基础诊断 | 26804 | `6BF6D562B93E8EFBB40C141BBE633C153CD78AF2C9180D0DD592CCF8BF600929` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_ground_baro_drift_bdt.bin` | 气压长时间漂移诊断 | 26804 | `1A470B06E3A6A46A6D4CE651F9716ED548F83B408DDF8AE815E01ACEFD8ED898` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_ground_baro_temp_bdt.bin` | 气压和温度双通道诊断 | 27172 | `D7F430B27BFF7470B5AB670A23F4A8A8151559F90A6E95E7E666B1C8D72DDDE8` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_ground_baro_cal_bdt.bin` | 出厂系数和离线温补诊断 | 27252 | `759E132068C330F6117FC953326BE2D9F14217F345C95FF90B74866B12542AEC` | 0x4000 |

## 原始数据和回归夹具

| 文件 | 用途 | 字节 | SHA256 | 烧录地址 |
|---|---|---:|---|---|
| `data/archive/20261123_ordinary_write_before/Read.bin` | 实板 BDT 调试读回 | 53248 | `3A11D8C16B7C3B921A21AFB9EB4F880F85B4B2825780CBE614B9ED52F0223BC3` | — |
| `data/archive/20261123_timing_before/Read.bin` | 实板 BDT 调试读回 | 53248 | `24A28AFC45E251F566FA9CBE66AA9B69582C9CA8F01630066E48DC58EFFA9983` | — |
| `data/archive/20261123_trace_before_light/Read.bin` | 实板 BDT 调试读回 | 53248 | `57DED4C76022C9BB182CCB9FE825136A1DD51EF821B982380AC7D3FED7D15DC0` | — |
| `data/archive/20261123_write_profile_before/Read.bin` | 实板 BDT 调试读回 | 53248 | `6B06D2EBA549D802CDD788F6CBDA6A1FE456C81722A6EB6996394177BB49362E` | — |
| `data/archive/flight_log_dumps_20261122/flight_log_clean.bin` | 完整会话回归数据 | 53248 | `DF03276D39786A1C113634751DD938A36317ECA341573E73658F17F0DA87A0D8` | — |
| `data/archive/flight_log_dumps_20261122/flight_log_dump.bin` | 坏页和未封口恢复回归数据 | 53248 | `946BB2D6F0E7695CCA6D35FE263B0E985D91F5FA8CCA068D3F86C21E6DABF2F9` | — |
| `data/archive/flight_log_dumps_20261122/flight_log_integration.bin` | FlightLoop 日志集成回归数据 | 53248 | `2B65BE2CC5D717301097624AEB0C0A602A832E406EF4FB2F14A99633F3A00F77` | — |
| `data/archive/flight_log_dumps_20261122/flight_log_locked.bin` | 锁定静置和急停回归数据 | 53248 | `7A69A61C414E2995A60662A87253D5BF790776F0E1580543F1015F4A305D185B` | — |
| `data/archive/flight_log_dumps_20261122/flight_log_unlock.bin` | 解锁后停机回归数据 | 53248 | `D7397B87D466F3F01E5724EAA6325C700B35C7EE6FC127446D9E39AB615294BC` | — |
| `data/archive/flight_log_real_20261123_read.bin` | 第一趟实板 BDT 原始数据 | 53248 | `9453D6FF940FB25057A6FDB4C6684638028BCF2EA394A2E7AE6605F23DB9C57A` | — |
| `data/archive/flight_log_real_20261123_second_read.bin` | 第二趟实板 BDT 原始数据 | 53248 | `3425DE69F6C2D9277711B0A030E8C7993AF8819DF51AE75751D222986CB0FF71` | — |
| `tools/regression/fixtures/flight_log_v1_history.bin` | 由历史 CSV 重建的 v1 导出回归夹具 | 53248 | `C54EA380D7790352DB310C9757A8F030A1C4F5480FA4F58AC3F22280C0E62B74` | — |
| `tools/regression/fixtures/baro_real_v4_20261124.bin` | 真实 v4 出厂温补回归夹具 | 4096 | `29ABB7C005F65DBE28D447CF1209CCE00F25B653BD45722E1D42A347C3115F6C` | — |

| `firmware_delivery/archive/Robin_24G_Car_flight_control_fb28cc3.bin` | 原 fb28cc3 实飞镜像归档（实飞记录对应此件） | 24948 | `B9B98FF5B82105C150BFDD10078B9DD07987EBD3B0F065668E795610A6AF010C` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control_baro_candidate.bin` | 已移出当前交付的气压飞行候选 | 27780 | `A7CD8CB36A55546ACC90EA5FEDF084213FBE603EFD72F68FF116596DA9D2ADE4` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control_baro_on_accept.bin` | 已移出当前交付的气压开启验收构建 | 27764 | `0EB8651582F87A62C83EB0B3D71A6689F64A4DD516016D52B689F317F4E870C9` | 0x4000 |
| `firmware_delivery/archive/Robin_24G_Car_flight_control_baro_off_accept.bin` | 已移出当前交付的重复气压关闭构建 | 24516 | `C4DE867D9C8C0B0937382EABF50DAFD307FD43BDA2994A98DE43473FEAAB07F2` | 0x4000 |
