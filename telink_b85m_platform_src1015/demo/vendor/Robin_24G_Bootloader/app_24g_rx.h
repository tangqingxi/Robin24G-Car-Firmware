/********************************************************************************************************
 * @file     APP_24G_RX.h
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

#ifndef _APP_24G_RX_H_
#define _APP_24G_RX_H_
#include "types.h"
#include "app_24g_rf_frame.h"
#include "app_public_config.h"
#include "app_24g_rf_common.h"

/* ------------------------------------------------------------------ *
 *  RF link-layer state (Car/RX variant)
 *  Full member set used by the RX path; the Remote keeps its own
 *  trimmed version in app_24g_rf.h.
 * ------------------------------------------------------------------ */
typedef struct {
	unsigned int  pair_accesscode; // default:0x39517695
	unsigned char pair_success_flag;
	unsigned int  remoteid;        // pairing remote id, 0 means idle
	unsigned int  pair_timer;      // pairing timer tick
} Pair_Relevant_Var_T;

typedef struct {
	unsigned char  rf_rx_buff[PKT_BUFF_SIZE*2];
	rf_packet_t    ack_rf_dat;

	unsigned int   rf_rx_wptr;
	unsigned char  *raw_pkt;
	unsigned short rx_fail;

	unsigned int   dongle_id;

	unsigned char  tx_power_normal;
	unsigned char  host_channel;

	AES_KEY         key_enc; // paired device information
} User_Relevant_Var_T;

extern User_Relevant_Var_T user_var;
extern Pair_Relevant_Var_T pair_var;

void d24_rx_main_loop(void);
void rf_rx_irq_handle(void);
// Forward declarations to avoid implicit declaration warnings
void user_rf_ack_reset(unsigned char pipe, rf_packet_t *ack_rf_data, int rf_length);
void rf_tx_irq_handle(void);

#if (D24G_OTA_ENABLE)
// OTA 只保留 VERSION 查询处理（defined in app_24g.c）
void d24g_ota_write(ota_buff_t *p_usb);
// OTA 命令响应处理（defined in app_24g_rx.c）
u8  rf_ota_data_response(rf_packet_t *p_rf_rx_data);

// OTA state variables (defined in app_24g.c)
extern ota_buff_t ota_buff;
extern u8         ota_buff_valid_flag;
#endif // (D24G_OTA_ENABLE)


#endif
