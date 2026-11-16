/********************************************************************************************************
 * @file     app_24g_rf_common.h
 *
 * @brief    RF protocol definitions shared between Robin_24G_Car (RX) and
 *           Robin_24G_Remote (TX). Both sides use the same command system
 *           and key bitmask. The RF link-layer state structures
 *           (User/Pair_Relevant_Var_T) are vendor-specific and defined in
 *           each vendor's own header.
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

#ifndef _APP_24G_RF_COMMON_H_
#define _APP_24G_RF_COMMON_H_

#include "types.h"
#include "app_24g_rf_frame.h"
#include "app_config.h"

/* ------------------------------------------------------------------ *
 *  Key bitmask (event field in remote_cmd_t / km_dat payload)
 *  Produced by Remote (hw_tower_key) and consumed by Car (toy_car_ctrl).
 * ------------------------------------------------------------------ */
#define KEY_NONE        0x0000
#define KEY_FORWARD     (1 << 0)  // Forward
#define KEY_BACK        (1 << 1)  // Backward
#define KEY_LEFT        (1 << 2)  // Left
#define KEY_RIGHT       (1 << 3)  // Right
#define KEY_OPEN        (1 << 4)  // Open door
#define KEY_DEMO        (1 << 5)  // Demo
#define KEY_K8          (1 << 6)  // K8
#define KEY_K7          (1 << 7)  // K7

/* ------------------------------------------------------------------ *
 *  Application-layer command (TX -> Car payload format)
 * ------------------------------------------------------------------ */
typedef enum {
	CMD_NONE = 0,      // idle
	CMD_KEEPALIVE,     // keepalive (event = 0)
	CMD_KEY,           // key_event (event = key bitmask, para1/para2 = params)
	CMD_OTA,           // OTA frame (reserved)
} cmd_TypeDef;

typedef struct __attribute__((packed)) {
	u8  type;   // cmd_TypeDef: CMD_KEEPALIVE/CMD_KEY/CMD_OTA
	u16 sqn;    // sequence number, incremented per command
	u16 event;  // key bitmask when CMD_KEY, else 0
	u8  para1;  // event parameter 1 (e.g. speed)
	u8  para2;  // event parameter 2 (e.g. angle)
} remote_cmd_t;

extern remote_cmd_t cmd_data;

/* ------------------------------------------------------------------ *
 *  Pairing access code table (16 preset codes, indexed by group)
 * ------------------------------------------------------------------ */
extern const u32 pair_access_code_array[16];

#endif /* _APP_24G_RF_COMMON_H_ */
