/********************************************************************************************************
 * @file     AAA_24G_rf_frame.h
 *
 * @brief    This is the header file for KMD SDK
 *
 * @author	 KMD GROUP
 * @date         01,2022
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

/*
 * led_rf_frame.h
 *
 *  Created on: Jan 13, 2014
 *      Author: xuzhen
 */

#ifndef _APP_24G_RF_FRAME_H_
#define _APP_24G_RF_FRAME_H_

// 确保 D24G_OTA_ENABLE 先定义：本文件可能在 app_config.h 之前被包含
// （如 app_24g_rf_common.h 的 include 顺序），否则 OTA 类型/宏不生成。
#include "app_config.h"


enum
{
    PIPE_PARING			= 0x00,//pair pipe 0
    PIPE_MOUSE			= 0x01,//mouse pipe 1
    // PIPE_KEYBOARD		= 0x02,//keyboard pipe 2
    //PIPE_AUDIO			= 0x03,
    //PIPE_TOUCH			= 0x04,
    //PIPE_RC				= 0x05,
    PIPE_ALL   = 0X3F,//pipe all
};

enum
{
    CMD_PAIR = 0xA5,
    CMD_ACK  = 0x5A,
    CMD_DATA = 0x55,
};

typedef struct
{
	u32 did;//device id
}AES_KEY;//aes key struct

typedef struct
{
    u32 dma_len;//dma len

    u8 rf_len; //rf len
    u8 dat[59];
} rf_packet_t;//rf packet struct

// ============================================================================
// 空中帧格式（旧格式 + 可扩展 dat[]）
// 四种命令帧共用旧格式头（10 字节：car_id/remote_id/key/cmd，cmd 固定偏移 9），
// 各自在帧尾追加独立的 dat[] 扩展，长度由对应宏控制：
//   RF_CMD_PAIR_DAT_LEN     CMD_PAIR  配对请求扩展参数长度
//   RF_CMD_ACK_DAT_LEN      CMD_ACK   配对应答扩展参数长度
//   RF_CMD_DATA_DAT_LEN     CMD_DATA  控制命令扩展参数长度
//   RF_CMD_DATA_ACK_DAT_LEN CMD_ACK   控制命令应答（双向通讯）扩展参数长度
// 宏为 0 = 该命令帧走旧协议（10 字节），与 51afa986 之前的固件空中完全兼容；
// 宏 > 0 = 该命令帧追加 dat[]，客户把应用参数放进 dat 即可扩展命令，
//          旧设备只读前 10 字节，多余数据自动忽略，仍可兼容。
// 旧格式 key 为 u8 按键掩码：key==0 表示 keepalive（无按键/全释放），
// 非 0 表示按键事件，与旧固件行为一致。
// （发送/接收长度一律用各结构体的 sizeof() 派生，改宏即自动调整包长。）
// 各扩展宏定义在共用配置文件 toy_common/app_24g_rf_frame_cfg.h，
// Car/Remote 两端引用同一份配置，天然保证空中协议一致。
// Ceiling: sizeof(帧结构体) + 11 <= PKT_BUFF_SIZE(48)
//          => 10 + 各宏 + 11 <= 48  =>  各宏 <= 27
// ============================================================================
// WARNING: THIS FILE MUST STAY BYTE-IDENTICAL WITH THE COUNTERPART:
//   Robin_24G_Car/app_24g_rf_frame.h  <->  Robin_24G_Remote/app_24g_rf_frame.h
#include "app_24g_rf_frame_cfg.h"

typedef struct __attribute__((packed))
{
	u32 car_id;    // Car ID (ACK only)
	u32 remote_id; // Remote ID
	u8  key;       // key byte（旧格式 u8 按键掩码；cmd 保持偏移 9 以兼容旧固件）
	u8  cmd;       // CMD: 0xA5 pair
#if (RF_CMD_PAIR_DAT_LEN > 0)
	u8  dat[RF_CMD_PAIR_DAT_LEN]; // 客户自定义配对扩展参数
#endif
} pair_data_t; // CMD_PAIR 配对请求帧

typedef struct __attribute__((packed))
{
	u32 car_id;    // Car ID
	u32 remote_id; // Remote ID
	u8  key;       // key byte
	u8  cmd;       // CMD: 0x5A ack
#if (RF_CMD_ACK_DAT_LEN > 0)
	u8  dat[RF_CMD_ACK_DAT_LEN]; // 客户自定义配对应答扩展参数
#endif
} pair_ack_data_t; // CMD_ACK 配对应答帧

typedef struct __attribute__((packed))
{
	u32 car_id;    // Car ID (paired)
	u32 remote_id; // Remote ID
	u8  key;       // key byte（旧格式 u8 按键掩码；0 = keepalive）
	u8  cmd;       // CMD: 0x55 data
#if (RF_CMD_DATA_DAT_LEN > 0)
	u8  dat[RF_CMD_DATA_DAT_LEN]; // 客户自定义控制命令扩展参数
#endif
} rf_cmd_frame_t; // CMD_DATA 控制命令帧

typedef struct __attribute__((packed))
{
	u32 car_id;    // Car ID
	u32 remote_id; // Remote ID
	u8  key;       // key byte（echo 被应答命令的 key，0 = keepalive）
	u8  cmd;       // CMD: 0x5A ack
#if (RF_CMD_DATA_ACK_DAT_LEN > 0)
	u8  dat[RF_CMD_DATA_ACK_DAT_LEN]; // 客户自定义命令应答扩展参数
#endif
} rf_cmd_ack_frame_t; // CMD_ACK 控制命令应答帧（双向通讯模式）



// ============================================================================
// 2.4G OTA 帧定义（与参考 slaver / B80B dongle 协议一致）
// 由 D24G_OTA_ENABLE 宏控制，关闭时不编译 OTA 代码
// ============================================================================
#if (D24G_OTA_ENABLE)
#define CMD_OTA_DATA        10      // dongle -> car: OTA data packet (start / data / end)
#define CMD_OTA_ACK         11      // car -> dongle: OTA ACK packet

// Inner OTA sub-commands carried in ota_data_st.cmd
#define CMD_OTA_START       0xFF01  // 发起 OTA
#define CMD_OTA_END         0xFF02  // 结束 OTA
#define CMD_OTA_VERSION     0xFF03  // 版本查询: dongle -> car

// Magic number identifying a valid OTA version-query packet (same as START_UP_FLAG "TLNK")
#define OTA_VERSION_MAGIC   0x544C4E4B

// Version query payload carried in ota_data_st.buf (16 bytes)
typedef struct __attribute__((packed))
{
    u32 magic;        // OTA_VERSION_MAGIC = 0x544C4E4B
    u32 fw_version;   // new fw_version (type<<24 | ver)
    u32 target_did;   // 0 = broadcast, else specific device
    u32 target_pid;  // target product PID (0 = no PID filter)
} ota_version_query_t;

// OTA timeout: 60 seconds with no valid packet -> abort
#define D24G_OTA_TIMEOUT_US  (60 * 1000 * 1000)

// OTA error codes (returned by d24g_ota_write / passed to d24g_ota_resut)
typedef enum
{
    OTA_SUCCESS = 0,
    OTA_DATA_PACKET_SEQ_ERR,
    OTA_PACKET_INVALID,
    OTA_DATA_CRC_ERR,
    OTA_WRITE_FLASH_ERR,
    OTA_DATA_UNCOMPLETE,
    OTA_FLOW_ERR,
    OTA_FW_CHECK_ERR,
    OTA_VERSION_COMPARE_ERR,
    OTA_PDU_LEN_ERR,
    OTA_FIRMWARE_MARK_ERR,
    OTA_FW_SIZE_ERR,
    OTA_DATA_PACKET_TIMEOUT,
    OTA_TIMEOUT,
    OTA_FAIL_DUE_TO_CONNECTION_TERMIANTE,
    OTA_LOGIC_ERR,
} ota_err_e;

// OTA data/ack packet (matches dongle's ota_data_t layout, 27 bytes).
// report_id/opcode/length (USB-channel legacy fields) removed to shorten the
// air frame; PDU type is identified solely by ota_data_st.cmd.
// dongle 额外在 RF offset 27-30/31-34 追加 fw_type(u32) / target_did(u32) 过滤字段，
// 设备端从 raw dat[] 直接读取兼容。
typedef struct __attribute__((packed))
{
    u8  cmd;        // = CMD_OTA_DATA (10) / CMD_OTA_ACK (11)
    u8  seq_no;     // RF frame sequence number, incremented per TX
    u8  pno_no;     // 1 = valid data, 0 = invalid (piggyback only)
    u32 did;        // sender device ID
    u8  dat[20];    // contains ota_data_st (2+16+2 = 20 bytes)
} ota_data_t;

typedef ota_data_t ota_ack_data_t;

// OTA PDU buffer / inner PDU (20 bytes)
// 定义在此头文件（而非 app_public_config.h），供 app_24g_rf.h 引用时可见
typedef struct {
	u8  dat[20];
} ota_buff_t;

typedef struct {
	u16 cmd;   // 0xFF01 START / 0xFF02 END / 0xFF03 VERSION / DATA 时为包序号
	u8  buf[16];
	u16 crc;
} ota_data_st;
#endif // (D24G_OTA_ENABLE)

#endif /* LED_RF_FRAME_H_ */

