/********************************************************************************************************
 * @file     AAA_24G_RF.h
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

#ifndef _APP_RF_LINK_LAYER_24_H_
#define _APP_RF_LINK_LAYER_24_H_
#include "types.h"

#define		LL_CHANNEL_SYNC_TH				2
#define		LL_CHANNEL_SEARCH_TH			60
#define		LL_CHANNEL_SEARCH_FLAG			BIT(16)
#define		LL_NEXT_CHANNEL(c)				((c + 6) & 14)


extern u32  ll_chn_mask;
extern u8	device_channel;
extern u8	get_next_channel_with_mask(u32 mask, u8 chn);
extern void irq_device_tx(void);
extern void ll_device_init(void);

typedef void (*task_when_rf_func)(void);
extern task_when_rf_func p_task_when_rf;

extern void rf_drv_private_1m_init(void);
extern void rf_set_pair_access_code(u32 code);
extern void rf_set_data_access_code(u32 code);


extern void rf_set_access_code1(u32 code);
extern u32 rf_get_access_code1();
extern void rf_irq_src_allclr();
extern void irq_handle_private_1m();
extern int rf_trx_state_set(RF_StatusTypeDef rf_status, signed char rf_channel);
extern void rf_acc_len_set(unsigned char len);
void rf_receiving_pipe_enble(u8 channel_mask);

// Add missing macros for RF packet handling
#define GET_RX_PIPE(p)  ((p[4] & 0x07))  // Get RX pipe number from packet
#define GET_RX_RSSI(p)  (p[p[0]+2])      // Get RSSI value from packet

// Add missing function declaration
u8 get_rx_rssi_handle(rf_packet_t *p);
u8 caculate_data_checksum_handle(u8 *pdata, u8 length);
// Add rf_chn extern declaration
extern unsigned char rf_chn[5];
void init_rf_chn_by_device_id(u32 device_id);

void custom_init(void);

#define RF_SEND_DATA_MODE 2
#define RF_SEND_PAIR_MODE 1

#endif

