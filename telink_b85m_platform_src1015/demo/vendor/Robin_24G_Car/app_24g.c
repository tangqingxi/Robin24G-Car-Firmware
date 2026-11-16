/********************************************************************************************************
 * @file     AAA_24G_APP.c
 *
 * @brief    This is the source file for KMD SDK
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

#include "app_public_config.h"
#include "toy_log.h"
#include "app_24g_rx.h"
#include "app_24g_rf.h"
#include "ota_config.h"
#include "flash_fw_check.h"
#include "sdk_version.h"

#if (FPGA == 1) //The ROM size cannot exceed 32KB; delete unnecessary logs.
	#define TOY_LOG_24G(...)
#else
	#define TOY_LOG_24G(...)   TOY_LOG(__VA_ARGS__)
#endif

_attribute_data_retention_ efuse_dev_info_t efuse_dev_info;

_attribute_data_retention_ u8 device_status = 0; //device status (defined here, referenced by app_24g_rf.c)

#if (D24G_OTA_ENABLE)
// ---------------------------------------------------------------------------
// OTA 只保留 VERSION 查询处理：IRQ 把 VERSION PDU 缓存到 ota_buff，
// 主循环消费后判断可升级则写标记并重启（OTA 数据接收已移到 bootloader）。
// ---------------------------------------------------------------------------
_attribute_data_retention_ ota_buff_t ota_buff;       // most recent valid OTA PDU
_attribute_data_retention_ u8 ota_buff_valid_flag;    // 1 = ota_buff has new data

/**
 * @brief       写 OTA 标记到 flash（0x1C000 扇区），随后重启。
 *              bootloader 上电读到该标记后进入 OTA 模式接收固件。
 * @note        app 检测到可升级版本时调用。
 */
void ota_flag_set(void)
{
    flash_erase_sector(OTA_FLAG_ADDR);
    u32 magic = OTA_FLAG_MAGIC;
    flash_write_page(OTA_FLAG_ADDR, 4, (u8 *)&magic);
    // 把当前 app 版本写入固定地址，bootloader 上报 poll 版本时读取
    u32 ver = FW_VERSION;
    flash_write_page(APP_VERSION_ADDR, 4, (u8 *)&ver);
}

/**
 * @brief       OTA 处理（仅 VERSION 查询）。
 *              收到低版本 VERSION -> 写 OTA 标记 + 重启，由 bootloader 完成接收。
 * @param[in]   p_usb - OTA PDU buffer (ota_buff_t)
 */
void d24g_ota_write(ota_buff_t *p_usb)
{
    ota_data_st *pd = (ota_data_st *)&p_usb->dat[0];

    if (pd->cmd != CMD_OTA_VERSION) {
        return;
    }

    ota_version_query_t *vq = (ota_version_query_t *)pd->buf;
    if (vq->magic != OTA_VERSION_MAGIC) {
        return;
    }
    // PID 过滤：待升级固件产品 PID 与自身不一致时不升级（0=不过滤）。
    if (vq->target_pid != 0 && vq->target_pid != PID) {
        TOY_LOG("---ota: VERSION pid=0x%08x != self 0x%08x, ignore\n",
                vq->target_pid, (u32)PID);
        return;
    }
    // target_did 定向：非 0 且非自身设备 → 忽略
    if (vq->target_did != 0 && vq->target_did != user_var.dongle_id) {
        return;
    }
    if (vq->fw_version == FW_VERSION) {
        TOY_LOG("---ota: VERSION same 0x%04x, no update\n", (u32)FW_VERSION);
    } else {
        // 可升级：写 OTA 标记后重启，bootloader 读标记进入 OTA 模式接收固件
        TOY_LOG("---ota: VERSION new=0x%08x, write flag and reboot\n", vq->fw_version);
        ota_flag_set();
        start_reboot();
    }
}
#endif // (D24G_OTA_ENABLE)



/**
 * @brief       This function 24g user init
 * @return
 * @note
 */
void d24_user_init(void)
{
	// Receiver initialization (Car RX path)
	TOY_LOG_24G("---DEVICE MODE: RX (RECEIVER) - RF_MODE_RX_MULTI\n");

	// Note: rf_drv_private_1m_init() already called in main.c before d24_user_init()
	// So we don't need to call it again here

	/*
	 * RF Access Code Configuration on Power-up (RX - Car side):
	 * - PIPE 0: Pairing channel, uses pair_accesscode for pairing process
	 * - PIPE 1: Data channel, uses dongle_id as data access code
	 * - Enable both PIPE 0 and PIPE 1 (bit mask 0x03) to receive pairing and data packets
	 * - TX default uses PIPE 0 for pairing broadcast
	 */
	rf_set_pair_access_code(pair_var.pair_accesscode);	// Set PIPE 0 access code for pairing
	rf_set_data_access_code(user_var.dongle_id);		// Set PIPE 1 access code (dongle_id)
	rf_rx_acc_code_enable(0x03);	// Enable PIPE 0(bit0) and PIPE 1(bit1) for RX, bit0~bit5 correspond to channel 0~5
	rf_tx_acc_code_select(0);		// TX uses PIPE 0 for pairing broadcast, pipe range: 0~5

	rf_set_power_level_index(user_var.tx_power_normal);

	// Set RF channel and enter RX mode
	rf_set_channel(rf_chn[user_var.host_channel], 0);
	rf_trx_state_set(RF_MODE_RX, rf_chn[user_var.host_channel]);  // Enter RX mode

	TOY_LOG_24G("---RX: Receiver ready on channel %d\n", rf_chn[user_var.host_channel]);
}
