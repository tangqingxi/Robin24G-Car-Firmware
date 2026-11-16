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
#include "app_config.h"
#include "app_24g_rf_frame.h"
#include "app_24g_rf_common.h"

/* ------------------------------------------------------------------ *
 *  RF link-layer state (Remote/TX variant)
 *  Only the members actually used by the Remote are kept; the Car keeps
 *  its own full version in app_24g_rx.h.
 * ------------------------------------------------------------------ */
typedef struct {
	unsigned int  pair_accesscode; // default:0x39517695
} Pair_Relevant_Var_T;

typedef struct {
	unsigned char  rf_rx_buff[PKT_BUFF_SIZE*2];
	rf_packet_t    ack_rf_dat;        // OTA poll TX buffer (km_3_c_1_data_t)

	unsigned int   dongle_id;
	unsigned char  tx_power_normal;
} User_Relevant_Var_T;

extern User_Relevant_Var_T user_var;
extern Pair_Relevant_Var_T pair_var;

#if (D24G_OTA_ENABLE)
// OTA 只保留 VERSION 查询处理（defined in app_24g.c）
void d24g_ota_write(ota_buff_t *p_usb);
// OTA poll（defined in app_24g.c）
void ota_probe_send_poll(void);
// OTA 命令响应处理（defined in app_24g_rf.c）
u8  rf_ota_data_response(rf_packet_t *p_rf_rx_data);

extern ota_buff_t ota_buff;
extern u8         ota_buff_valid_flag;
// Set to 1 by ota_probe_send_poll() before rf_stx_to_rx (AUTO mode TX).
// TX-done IRQ handler skips its rf_state handling when this is set.
extern u8         ota_auto_tx_pending;
#endif // (D24G_OTA_ENABLE)

#define		LL_CHANNEL_SYNC_TH				2
#define		LL_CHANNEL_SEARCH_TH			60
#define		LL_CHANNEL_SEARCH_FLAG			BIT(16)
#define		LL_NEXT_CHANNEL(c)				((c + 6) & 14)


extern u32  ll_chn_mask;
extern u8	device_channel;
extern u8	get_next_channel_with_mask(u32 mask, u8 chn);
extern void irq_device_tx(void);
extern void irq_device_rx(void);
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

/* ------------------------------------------------------------------ *
 *  TX frame type tags. The application TX path no longer keeps a state
 *  machine: remote_send_cmd() writes the latest command into a buffer and
 *  the RF loop copies it out when sending. These values are kept as the
 *  `mode` argument of app_test_on_tx_packet() (PACKET_LOSS_TEST_ENABLE)
 *  and mirror the legacy RF_SEND_PAIR_MODE / RF_SEND_DATA_MODE values.
 * ------------------------------------------------------------------ */
typedef enum {
    TX_STATE_IDLE = 0,   // (unused as state; kept for compatibility)
    TX_STATE_PAIR = 1,   // pairing frame (rf_stx_to_rx + wait ACK)
    TX_STATE_CMD  = 2,   // control command (5 channels, rf_start_stx)
} tx_state_t;

/**
 * @brief  Queue one command on the paired link (5 channels).
 * @param[in] type    cmd_TypeDef: CMD_KEY (control) / CMD_KEEPALIVE (heartbeat)
 * @param[in] event   u16 key bitmask (KEY_FORWARD/KEY_LEFT/...); for CMD_KEY,
 *                    event=0 means "all keys released" (key-release report)
 * @param[in] p_dat   pointer to additional command payload (copied into dat[]),
 *                    NULL = no payload (dat[] zeroed)
 * @param[in] dat_len number of valid bytes in p_dat; values larger than
 *                    RF_CMD_DATA_DAT_LEN are truncated
 * @return  1 accepted, 0 not paired
 * @note  The latest command overwrites any in-flight one; the RF loop sends
 *        whatever is in tx_cmd_buf next.
 *        旧格式：空中 key 为 u8 按键掩码（取 event 低字节），event=0 => key=0
 *        （keepalive / 全释放）；type 仅用于本地心跳逻辑，不占空中字节。
 *        UI usage: press  -> remote_send_cmd(CMD_KEY, KEY_FORWARD, NULL, 0)
 *                  release-> remote_send_cmd(CMD_KEY, 0, NULL, 0)
 *                  idle   -> remote_send_cmd(CMD_KEEPALIVE, 0, NULL, 0)
 */
u8 remote_send_cmd(u8 type, u16 event, u8 *p_dat, u8 dat_len);

#endif

