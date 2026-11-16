/********************************************************************************************************
 * @file     app_24g_rf_frame_cfg.h
 *
 * @brief    RF 空中帧 dat[] 扩展长度配置（Car / Remote 共用，改这里即可同步两端）
 *
 * @author   lingzhi.chen
 * @date         08,2026
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/

#ifndef _APP_24G_RF_FRAME_CFG_H_
#define _APP_24G_RF_FRAME_CFG_H_

/* ------------------------------------------------------------------ *
 * 空中帧 dat[] 扩展长度配置（唯一配置点，Car/Remote 两侧共用）
 *
 * 四种命令帧共用旧格式头（10 字节：car_id/remote_id/key/cmd，cmd 固定偏移 9），
 * 各自在帧尾追加独立的 dat[] 扩展，长度由对应宏控制：
 *   RF_CMD_PAIR_DAT_LEN     CMD_PAIR  配对请求扩展参数长度
 *   RF_CMD_ACK_DAT_LEN      CMD_ACK   配对应答扩展参数长度
 *   RF_CMD_DATA_DAT_LEN     CMD_DATA  控制命令扩展参数长度
 *   RF_CMD_DATA_ACK_DAT_LEN CMD_ACK   控制命令应答（双向通讯）扩展参数长度
 *
 * 宏为 0 = 该命令帧走旧协议（10 字节），与 51afa986 之前的固件空中完全兼容；
 * 宏 > 0 = 该命令帧追加 dat[]，客户把应用参数放进 dat 即可扩展命令，
 *          旧设备只读前 10 字节，多余数据自动忽略，仍可兼容。
 *
 * 注意：这些宏属于空中协议，Car/Remote 两端 MUST 使用相同值，
 * 否则配对/通讯会断开。本文件由两端共同引用，天然保证一致。
 * Ceiling: sizeof(帧结构体) + 11 <= PKT_BUFF_SIZE(48)
 *          => 10 + 各宏 + 11 <= 48  =>  各宏 <= 27
 * ------------------------------------------------------------------ */
#ifndef RF_CMD_PAIR_DAT_LEN
// 配对请求帧扩展参数长度：0 = 旧协议（10 字节）。
// OTA 触发统一走 poll → VERSION：未配对/已配对设备都以 3s 低频 poll 查询，
// dongle 在 poll ACK 捎带 VERSION，配对广播无需携带版本/PID。
#define RF_CMD_PAIR_DAT_LEN  2
#endif
#ifndef RF_CMD_ACK_DAT_LEN
// 配对应答帧扩展参数长度：0 = 旧协议（10 字节）。
// 配对 ACK 不再携带 OTA 邀请标志/版本，OTA 入口统一为 poll → VERSION。
#define RF_CMD_ACK_DAT_LEN   2
#endif
#ifndef RF_CMD_DATA_DAT_LEN
// 用户调测产品使用 dat[0]/dat[1] 携带扩展控制数据（如速度档），
// UserRxdFunc.c 的 UserRfRcvBuffer[10]/[11] 消费。OTA 帧按 rf_len>=sizeof(ota_data_t)
// 严格区分，与本扩展不冲突。
#define RF_CMD_DATA_DAT_LEN  10
#endif
#ifndef RF_CMD_DATA_ACK_DAT_LEN
#define RF_CMD_DATA_ACK_DAT_LEN  0
#endif

#endif /* _APP_24G_RF_FRAME_CFG_H_ */
