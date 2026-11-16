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
#include "app_battery_check.h"
#include "app_toy/app_layer/toy_log.h"
#include "app_toy/app_layer/app_test.h"
#include "app_toy/app_layer/app_toy.h"
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

//u8 stx_flag = 0; //device status

_attribute_data_retention_ u8 device_status = 0; //device status
_attribute_data_retention_ u32 dongle_id;

#if (D24G_OTA_ENABLE)
// ---------------------------------------------------------------------------
// OTA 只保留 VERSION 查询处理：poll 触发 VERSION 接收，缓存到 ota_buff，
// 主循环消费后判断可升级则写标记并重启（OTA 数据接收已移到 bootloader）。
// ---------------------------------------------------------------------------
_attribute_data_retention_ ota_buff_t ota_buff;       // most recent valid OTA PDU
_attribute_data_retention_ u8 ota_buff_valid_flag;    // 1 = ota_buff has new data

// CRC16 polynomial table (Modbus-style, init 0xffff, poly 0xa001 reflected)
static const unsigned short crc16_poly1[2] = {0x0000, 0xa001};

/**
 * @brief       CRC16 over the given buffer (matches dongle / reference project)
 * @param[in]   pD   - data pointer
 * @param[in]   len  - data length
 * @return      CRC16 value
 */
unsigned short crc16_user(unsigned char *pD, int len)
{
    unsigned short crc = 0xffff;
    int i, j;
    for (j = len; j > 0; j--) {
        unsigned char ds = *pD++;
        for (i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ crc16_poly1[(crc ^ ds) & 1];
            ds = ds >> 1;
        }
    }
    return crc;
}

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
    // target_did 定向：非 0 且非自身设备 → 忽略。
    // 0=广播（car 场景）；非 0=定向（remote 场景：dongle 已知配对广播携带的 did）。
    if (vq->target_did != 0 && vq->target_did != user_var.dongle_id) {
        TOY_LOG("---ota: VERSION target=0x%08x != self 0x%08x, ignore\n",
                vq->target_did, user_var.dongle_id);
        return;
    }
    if (vq->fw_version == FW_VERSION) {
        TOY_LOG("---ota: VERSION same 0x%04x, no update tick=%d\n", (u32)FW_VERSION, clock_time());
    } else {
        // 可升级：写 OTA 标记后重启，bootloader 读标记进入 OTA 模式接收固件
        TOY_LOG("---ota: VERSION new=0x%08x, write flag and reboot tick=%d\n", vq->fw_version, clock_time());
        ota_flag_set();
        start_reboot();
    }
}
#endif // (D24G_OTA_ENABLE)
_attribute_data_retention_ u8 pair_ack_para1 = REMOTE_PAIR_PARA1; // 客户自定义配对扩展参数	0; // pairing ACK dat[0] semantic alias from Car (customer extension)
_attribute_data_retention_ u8 pair_ack_para2 = REMOTE_PAIR_PARA2; // 客户自定义配对扩展参数	0; // pairing ACK dat[1] semantic alias from Car (customer extension)
//u8 dongle_id_valid_f = 0; //use to valid dongle id


_attribute_data_retention_ rf_packet_t rf_pair_buf __attribute__((aligned(4))) =
{
	.dma_len = sizeof(pair_data_t) + 1, // dma_len
	.rf_len  = sizeof(pair_data_t),     // rf_len
};

_attribute_data_retention_ pair_data_t *p_pair_dat = (pair_data_t *)&rf_pair_buf.dat[0]; //point to pair packet

_attribute_data_retention_ rf_packet_t rf_km_buf __attribute__((aligned(4))) =
{
	.dma_len = sizeof(rf_cmd_frame_t) + 1,	// dma_len
	.rf_len  = sizeof(rf_cmd_frame_t),		// rf_len
};
_attribute_data_retention_ rf_cmd_frame_t *p_km_data = (rf_cmd_frame_t *)&rf_km_buf.dat[0]; //point to km data packet

_attribute_data_retention_ u32 wakeup_next_tick = 0;
_attribute_data_retention_ u8 pair_success_flag = 0; //use to assure pair success

// Application-layer TX command buffer: remote_send_cmd() writes the latest
// command here, the RF loop copies it out when actually sending. No queue /
// state machine needed for command dispatch.
_attribute_data_retention_ rf_cmd_frame_t tx_cmd_buf;
_attribute_data_retention_ u8 tx_cmd_pending = 0; // new command waiting to be sent
_attribute_data_retention_ u8 tx_chn_left = 0;    // channels left for current command
_attribute_data_retention_ u32 hb_tick = 0;       // heartbeat timer

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
u8 remote_send_cmd(u8 type, u16 event, u8 *p_dat, u8 dat_len)
{
	if (device_status != STATE_NORMAL) return 0;

	tx_cmd_buf.cmd = CMD_DATA;
	tx_cmd_buf.car_id = dongle_id;              // paired car ID
	tx_cmd_buf.remote_id = user_var.dongle_id;  // remote ID
	// 旧格式 key 为 u8 按键掩码：取 event 低字节；event=0（全释放/keepalive）=> key=0
	tx_cmd_buf.key = (u8)(event & 0xFF);
#if (RF_CMD_DATA_DAT_LEN > 0)
	// tx_cmd_buf is a retention global: always zero dat[] first so heartbeat /
	// key-release frames never leak a previous command's payload on air.
	memset(tx_cmd_buf.dat, 0, RF_CMD_DATA_DAT_LEN);
	if (p_dat && dat_len) {
		u8 n = (dat_len < RF_CMD_DATA_DAT_LEN) ? dat_len : RF_CMD_DATA_DAT_LEN;
		memcpy(tx_cmd_buf.dat, p_dat, n);
		if (dat_len > RF_CMD_DATA_DAT_LEN)
			TOY_LOG_24G("remote_send_cmd: dat_len=%d > RF_CMD_DATA_DAT_LEN=%d, truncated\n",
					dat_len, RF_CMD_DATA_DAT_LEN);
	}
#if (PACKET_LOSS_TEST_ENABLE)
	// 丢包测试签名（event 高字节 0xAC）转发到 dat[0]，
	// 供 app_test_on_tx_packet 区分测试包与真实按键包（需 RF_CMD_DATA_DAT_LEN >= 1）
	if ((event & 0xFF00) == 0xAC00) {
		tx_cmd_buf.dat[0] = 0xAC;
	}
#endif
#endif

	tx_cmd_pending = 1;

	// Only key-control traffic (CMD_KEY, including the key-release report
	// with event=0) refreshes the heartbeat timer, so the periodic keepalive
	// never interleaves with an active key stream; the keepalive itself
	// (CMD_KEEPALIVE) does not touch hb_tick - d24g_rf_loop maintains its own
	// period. Only CMD_KEY with a non-zero event resets the idle timer: a long
	// key-press stays awake, while the idle timeout counts from release.
	if (type == CMD_KEY) {
		hb_tick = clock_time();
		if (event != 0) {
			reset_idle_status();
		}
	}

	return 1;
}



/**
 * @brief       This function deal with rf rx packet
 * @param[in]   p_rf_data	- rf_rx_packet
 * @return
 * @note
 */
u8 rf_rx_process(rf_packet_t *p_rf_data)
{
	if (device_status <= STATE_PAIRING) //Pairing mode
	{
		pair_ack_data_t *pair_ack_dat_ptr = (pair_ack_data_t*)&p_rf_data->dat[0];

		if ((pair_ack_dat_ptr->cmd == CMD_ACK) && (pair_ack_dat_ptr->remote_id == p_pair_dat->remote_id))
		{
#if (RF_CMD_PAIR_DAT_LEN >= 2)
			if((pair_ack_para1 != pair_ack_dat_ptr->dat[0]) || (pair_ack_para2 != pair_ack_dat_ptr->dat[1]))
			{
				return 0;
			}
#endif

        	dongle_id = pair_ack_dat_ptr->car_id; // paired car ID
//#if (RF_CMD_ACK_DAT_LEN >= 1)
//        	pair_ack_para1 = pair_ack_dat_ptr->dat[0]; // customer extension (dat[0] semantic alias)
//#else
//        	pair_ack_para1 = 0;
//#endif
//#if (RF_CMD_ACK_DAT_LEN >= 2)
//        	pair_ack_para2 = pair_ack_dat_ptr->dat[1]; // customer extension (dat[1] semantic alias)
//#else
//        	pair_ack_para2 = 0;
//#endif
        	pair_success_flag = 1;

			TOY_LOG("---pairing success: car_id=0x%08x\n", pair_ack_dat_ptr->car_id);

        	return 1;
    	}
	}

	// Normal mode: no ACK required for data
#if (BIDIR_COMM_ENABLE)
	// Bidirectional-comm mode: normal mode also receives data ACK from Car
	if (device_status == STATE_NORMAL)
	{
		rf_cmd_ack_frame_t *ack_ptr = (rf_cmd_ack_frame_t*)&p_rf_data->dat[0];
		if ((ack_ptr->cmd == CMD_ACK) && (ack_ptr->remote_id == p_km_data->remote_id))
		{
			// Hand the ACK to the app layer (status aggregation / peripherals).
			toy_remote_ack_on_rf_km_dat(ack_ptr, 0);
			return 1;
		}
	}
#endif
    return 0;
}




/**
 * @brief       This function 24g user init
 * @return      
 * @note        
 */
void d24_user_init(void)
{
	// TX side initialization
	TOY_LOG_24G("---DEVICE MODE: TX (TRANSMITTER) - RF_MODE_RX_MULTI\n");

	TOY_LOG_24G("---Device enters pairing mode.\n");
	rf_set_pair_access_code(pair_var.pair_accesscode);  // Set pairing access code first
	rf_rx_acc_code_enable(0x03);  // Enable pipe 0(0x01) + pipe 1(0x02) = 0x03
	rf_tx_acc_code_select(0);  // Default TX uses pipe 0 (for pairing)
	// Debug: Print rf_pair_buf initial values
	TOY_LOG_24G("---TX: rf_pair_buf.dma_len=%d, rf_pair_buf.rf_len=%d\n", rf_pair_buf.dma_len, rf_pair_buf.rf_len);

	p_pair_dat->cmd = CMD_PAIR;
	p_pair_dat->car_id = 0; // unused in pair
	p_pair_dat->remote_id = user_var.dongle_id; // Remote ID
	p_pair_dat->key = 0;
	// 配对广播不再携带版本/PID：OTA 触发统一走 poll → VERSION，
	// dongle 在 poll ACK 捎带 VERSION（PID 过滤），未配对/已配对设备均可触发。
#if (RF_CMD_PAIR_DAT_LEN > 0)
	p_pair_dat->dat[0] = REMOTE_PAIR_PARA1; // 客户自定义配对扩展参数
	p_pair_dat->dat[1] = REMOTE_PAIR_PARA2; // 客户自定义配对扩展参数
#endif

	TOY_LOG_24G("---TX: After init, pair cmd=0x%02x did=0x%08x\n", p_pair_dat->cmd, p_pair_dat->remote_id);

	p_km_data->cmd = CMD_DATA;
	p_km_data->car_id = dongle_id; // Car ID (paired)
	p_km_data->remote_id = user_var.dongle_id; // Remote ID
	p_km_data->key = 0;
#if (RF_CMD_DATA_DAT_LEN > 0)
	memset(p_km_data->dat, 0, RF_CMD_DATA_DAT_LEN); // retention global: zero first
#endif
	TOY_LOG("---TX: NORMAL data did=0x%08x\n", p_km_data->remote_id);

	if (flash_dev_info.dongle_id == 0) //power on or car id not paired
	{
		TOY_LOG_24G("---The device enters the sync mode.\r\n");
		device_status = STATE_SYSNC;
	}
	else
	{
		TOY_LOG_24G("---The device enters the normal mode.\r\n");
		device_status = STATE_NORMAL;
	}

	wakeup_next_tick = clock_time();//syn wakeup_next_tick
	reset_idle_status();//reset idle flag to 0

	// Reset the TX state machine (deep-sleep wakeup also re-enters here)
	tx_cmd_pending = 0;
	tx_chn_left = 0;
	hb_tick = clock_time();
}

////////////////////////////////////////////////////////////////////
_attribute_data_retention_ u8 need_suspend_flag = 0; //suspend flag

/**
 * @brief       This function check whether recieve rf rx packet right or not
 * @return      
 * @note        
 */
void check_rf_complet_status(void)
{
	rf_state = RF_IDLE_STATUS;//rf state set to idle status
	
#if (PACKET_LOSS_TEST_ENABLE)
	app_test_on_tx_attempt();
#endif

#if (BIDIR_COMM_ENABLE)
	// Bidirectional-comm mode: normal-mode data also waits for/checks ACK from
	// Car. If the ACK is received, stop sending the remaining channels of the
	// current command (no need to keep retransmitting on the other 4 channels).
	if (device_status == STATE_NORMAL) {
		if (device_ack_received) {
			tx_cmd_pending = 0;
			tx_chn_left = 0;
		}
		return;
	}
#else
	// In normal mode, data packets do not require ACK
	if (device_status == STATE_NORMAL) {
		return;
	}
#endif

	if (device_ack_received) //if receive ack
	{
	#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE) //use for fast tx rx
		check_rf_fast_setting_flag(); //check rf fast falg
	#endif

#if (PACKET_LOSS_TEST_ENABLE)
		app_test_on_ack_rcvd();
#endif
		
		if (device_status <= STATE_PAIRING)
		{
			if (pair_success_flag) //if pair success flag is 1
			{
				device_status = STATE_NORMAL;//device status change to normal status

				if (flash_dev_info.dongle_id != dongle_id) //new car id
		        {
		            flash_dev_info.dongle_id = dongle_id; //save new car id
		        }

				// Re-init hopping channels based on remote ID after pairing
				init_rf_chn_by_device_id(user_var.dongle_id);
				TOY_LOG("remote_id = 0x%08x\n", user_var.dongle_id);
				TOY_LOG("---REMOTE RF: paired hop rf_chn:%d,%d,%d,%d,%d\n",
					rf_chn[0], rf_chn[1], rf_chn[2], rf_chn[3], rf_chn[4]);
				device_channel = 0;

				connect_ok = 1; //connect ok

#if (PACKET_LOSS_TEST_ENABLE)
				app_test_on_pair_success();
#endif

        		reset_idle_status();

				// Re-arm the TX path and send the first heartbeat
				// immediately (also re-adopts a power-cycled Car).
				tx_cmd_pending = 0;
				tx_chn_left = 0;
				hb_tick = clock_time();
				remote_send_cmd(CMD_KEEPALIVE, 0, NULL, 0);
			}
		}
	}
	else
	{
#if (PACKET_LOSS_TEST_ENABLE)
		app_test_on_ack_miss();
#endif
	}
}

/**
 * @brief       This function deal rf loop
 * @return      
 * @note        
 */
void d24g_rf_loop()//rf state machine loop
{
	//u8 *ptr = (u8 *)&rf_pair_buf; //point pair packet;
	// TOY_LOG("no_ack: %d\n", no_ack);
#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)//rf fast enble
	check_rf_fast_setting_time();//check fast setting time
#endif

	if (rf_state == RF_IDLE_STATUS)//rf status idle
	{
		if (device_status <= STATE_PAIRING)//device status not normal
		{
			_attribute_data_retention_ static u8 last_print_device_status = 0xFF;
			_attribute_data_retention_ static u8 last_print_cmd = 0xFF;
			if (last_print_device_status != device_status || last_print_cmd != p_pair_dat->cmd) {
				TOY_LOG("---TX: Sending pair/sync packet (status=%d, CMD=0x%02x)\n", device_status, p_pair_dat->cmd);
				last_print_device_status = device_status;
				last_print_cmd = p_pair_dat->cmd;
			}

			pair_success_flag = 0; //clear pair_success_flag

			connect_ok = 0; //clear connect_ok

			// Unpaired: send pair frame and wait for the ACK
			// (rf_stx_to_rx -> RF_RX_END / RF_RX_TIMEOUT after D24G_PAIR_TIMER_OUT).
			// OTA 触发统一走 poll → VERSION：未配对状态以 0.5s 低频 poll 查询
			// （配对广播已不携带版本/PID），dongle 在 poll ACK 捎带 VERSION，
			// remote 收到后直接进入 OTA 模式。device_channel=0 固定对码 ch2。
#if (D24G_OTA_ENABLE)
			static u32 ota_poll_pair_tick = 0;
			if (clock_time_exceed(ota_poll_pair_tick, 500000)) {
				ota_poll_pair_tick = clock_time();
				device_channel = 0;
				ota_probe_send_poll();
			} else
#endif
			{
				rf_set_pair_access_code(pair_var.pair_accesscode);
				rf_set_power_level_index(user_var.tx_power_normal);
				rf_stx_to_rx((u8 *)&rf_pair_buf, D24G_PAIR_TIMER_OUT);
			}
		}
		else if (tx_cmd_pending)
		{
			// Paired: send one channel of the latest command. The first
			// channel locks the frame into p_km_data so all 5 channels carry
			// the same event; a newer remote_send_cmd() overwriting the buf
			// mid-burst is picked up on the next 5-channel round.
			if (tx_chn_left == 0) {
				tx_chn_left = 5;
				memcpy(p_km_data, &tx_cmd_buf, sizeof(rf_cmd_frame_t));
			}

#if (PACKET_LOSS_TEST_ENABLE)
			app_test_on_tx_packet(TX_STATE_CMD, p_km_data);
#endif

			rf_set_data_access_code(flash_dev_info.dongle_id);
			rf_tx_acc_code_select(1);
#if (BIDIR_COMM_ENABLE)
			rf_rx_acc_code_enable(0x02);//enable RX pipe1 to receive data ACK
			rf_set_power_level_index(user_var.tx_power_normal);
			rf_stx_to_rx((u8 *)&rf_km_buf, D24G_COMMUNICATION_TIMER_OUT);
#else
			rf_rx_acc_code_enable(0x00);
			rf_set_power_level_index(user_var.tx_power_normal);
			rf_set_channel(rf_chn[device_channel], 0);
			rf_state = RF_TX_START_STATUS;//rf state to TX status
			rf_start_stx((void *)&rf_km_buf.dma_len, clock_time());
#endif

			device_channel = (device_channel + 1) % 5;
			if (--tx_chn_left == 0) {
				tx_cmd_pending = 0;
			}
		}
		else
		{
			// Paired & no pending command: heartbeat keepalive.
			// hb_tick is updated here (remote_send_cmd(CMD_KEEPALIVE) itself
			// does not touch it) so the keepalive keeps a stable period.
#if (D24G_OTA_ENABLE)
			// 低频 OTA 轮询（0.5s）：已配对空闲（仅心跳）时向 dongle 轮询。
			// dongle 在 poll ACK 捎带 VERSION/START，remote 收到后进入 OTA 模式
			// （无需重新配对）。device_channel=0 固定对码 ch2（dongle 监听配对通道）。
			static u32 ota_poll_idle_tick = 0;
			if (clock_time_exceed(ota_poll_idle_tick, 500000)) {
				ota_poll_idle_tick = clock_time();
				device_channel = 0;
				ota_probe_send_poll();
			} else
#endif
			if (clock_time_exceed(hb_tick, TX_HEARTBEAT_INTERVAL_US)) {
				hb_tick = clock_time();
				remote_send_cmd(CMD_KEEPALIVE, 0, NULL, 0);
			}
		}
	}
	else if (rf_state == RF_RX_END_STATUS)//rf status is rf_rx end
	{
        irq_device_rx();//deal rx receive packet
		check_rf_complet_status();//check rf complet 
	}
	else if (rf_state == RF_RX_TIMEOUT_STATUS)//rf status is timeout status
	{
		// Timeout - no ACK received (don't spam logs)
		check_rf_complet_status();//check rf complet
	}
}

/**
 * @brief       This function deal ui_loop
 * @return      
 * @note        
 */
void ui_loop_24g(void)
{
	_attribute_data_retention_ static u32 tick_loop_24g;
	_attribute_data_retention_ static u32 tick_loop_cnt;

	// Heartbeat/keepalive is handled by the TX state machine
	// (d24g_rf_loop RF_IDLE branch -> remote_send_cmd(0)).
	if (clock_time_exceed(tick_loop_24g, 5000))
	{
		if (0 == (tick_loop_cnt%200))
		{
			rc_24m_cal();
		}

		#if MODULE_32K_WATCHDOG_ENABLE
		wd_32k_feed();
		#endif

		tick_loop_cnt ++;
		tick_loop_24g = clock_time();//update time

		idle_status_poll();//poll idle status parameters

		if (connect_ok == 0)//no connect
		{
			adv_count_poll();//adv parameter poll
		}
	}

#if (PACKET_LOSS_TEST_ENABLE)
	app_test_packet_loss_ui_loop();
#endif
}



#define TIMER_ALLOW_ENTER_SUSPEND_MIN	62500  //2600 * CLOCK_SYS_TIMER_CLK_1US
#define TIMER_ENTER_SUSPEND_TICK_1MS	23980  //

/**
 * @brief  休眠准入钩子（weak 默认实现：允许休眠）。
 *         客户在自己的应用文件中定义同名强函数覆盖，返回 0 时系统
 *         本轮不进入休眠，直到应用跑完返回 1 为止。
 * @return  1 = 允许进入休眠，0 = 阻止进入休眠
 */
__attribute__((weak)) u8 allow_sleep(void)
{
	return 1;
}


/**
 * @brief       This function deal 24g pm
 * @return      
 * @note        
 */
void pm_poll()
{
	u32 wakeup_src = PM_WAKEUP_TIMER; //use for set wake src
	u16 interval = 8;//use set time wake time
	need_suspend_flag = 0;//need suspend flag

	if ((rf_state == RF_IDLE_STATUS) && (adc_start_flag == 0))
	{
		if (device_status <= STATE_PAIRING)
		{

		#if D24G_ADV_DEEPSLEEP_TIMEOUT_ENABLE
			if (adv_count >= D24G_ADV_TIMER_OUT)
	   		{
				if (allow_sleep()) // 客户应用未跑完则本轮不进入深睡眠
				{
					TOY_LOG_24G("pm_poll 1:enter deep sleep adv_count=0X%x\n", adv_count);
					reset_adv_status();//reset adv status
		   			enter_deep(ANA_DEEP_SLEEP);//enter deep sleep
	   			}
	   		}
		#endif

		 	//wakeup_src = PM_WAKEUP_TIMER;//set wake src to timer
			//set wake up time
		 	//interval = 8;//set wake up time interval
			need_suspend_flag = 1;//need suspend flag set to 1
		}
		else if (device_status == STATE_NORMAL)
		{
			// Only consider sleeping when no command is in flight
			// (equivalent to the old fifo_km empty check).
			if (!tx_cmd_pending)
			{
				u8 idle_timeout = 1;  // 1 = 未到休眠超时, 0 = 已超时

				if (g_remote_sleep_mode == REMOTE_SLEEP_TIMEOUT_NEVER)
				{
					idle_timeout = 1;  // 永不休眠
				}
				else if (g_remote_sleep_mode == REMOTE_SLEEP_TIMEOUT_30MIN)
				{
					idle_timeout = (idle_count < REMOTE_SLEEP_TIMEOUT_30MIN_S);
				}
				else  // REMOTE_SLEEP_TIMEOUT_CUSTOM
				{
					idle_timeout = (idle_count < g_remote_sleep_timeout_s); //if idle less than sleep timeout
				}

				if (idle_timeout)
				{
					if (user_cfg.report_rate >= 4)
					{
						need_suspend_flag = 1;
					}

					gpio_set_wakeup_enable(0);
				}
				else
				{
					/* idle_count 超时：进入深度睡眠 */
					if (allow_sleep()) // 客户应用未跑完则本轮不进入深睡眠
					{
						TOY_LOG_24G("pm_poll 2:enter deep sleep idle_count=0X%x\n", idle_count);
						reset_idle_status();//reset idle status
						enter_deep(ANA_DEEP_SLEEP);//enter deep sleep

						need_suspend_flag = 1;
					}
				}
			}
		}

		if (need_suspend_flag)//if need suspend
		{
			if (!allow_sleep()) // 客户应用未跑完则本轮不进入浅睡眠
			{
				need_suspend_flag = 0; // 阻止本轮休眠，主循环按正常节奏运行
			}
			else
			{
				PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;
				rf_set_tx_rx_off(); //reset tx rx mode
				hw_sleep_wakeup_set_gpio_suspend(1); //浅睡眠前配置 GPIO 唤醒源
				#if 0
				cpu_sleep_wakeup(SUSPEND_MODE, wake_src, (wakeup_next_tick + interval * CLOCK_SYS_TIMER_CLK_1MS));//enter pm sleep
	            wakeup_next_tick = clock_time();//update wakeup_next_tick
				#else
				wakeup_next_tick = (wakeup_next_tick + interval * TIMER_ENTER_SUSPEND_TICK_1MS); //calculate suspend time
				int suspend_time = (int)(wakeup_next_tick - clock_time());
				if (suspend_time > (interval * TIMER_ENTER_SUSPEND_TICK_1MS))
				{
					//no need enter suspend
					wakeup_next_tick = clock_time();//update wakeup_next_tick
				}
				else if (suspend_time > TIMER_ALLOW_ENTER_SUSPEND_MIN)
				{
					cpu_sleep_wakeup_32k_rc(SUSPEND_MODE, wakeup_src, PM_TICK_STIMER, wakeup_next_tick);
					hw_sleep_wakeup_resume_gpio(1); //浅睡眠唤醒后恢复 GPIO

					#if(DEBUG_MODE==1)
					#if (DEBUG_BUS==DEBUG_IO)
						io_putchar_reinit();
					#endif
					#endif
				}
				else if (suspend_time > 0)
				{
					cpu_sleep_wakeup_32k_rc(SUSPEND_MODE, wakeup_src, PM_TICK_STIMER, clock_time() + TIMER_ALLOW_ENTER_SUSPEND_MIN);
					hw_sleep_wakeup_resume_gpio(1); //浅睡眠唤醒后恢复 GPIO

					#if(DEBUG_MODE==1)
					#if (DEBUG_BUS==DEBUG_IO)
						io_putchar_reinit();
					#endif
					#endif
				}
				#endif
				rf_drv_private_1m_init();//init rf private 1m
			}
		}
	}
}




#if (D24G_OTA_ENABLE)
/**
 * @brief       Send a D24G_OTA_CMD (cmd=10) poll packet to trigger the dongle
 *              to piggyback OTA data (CMD_OTA_START / DATA / END) in its ACK.
 *              未配对/已配对空闲分支与 OTA 模式（3ms 高速轮询）均调用。
 * @note        poll 收发必须在 dongle 监听的配对通道 ch2(2402MHz) 上进行，
 *              本函数内显式固定 device_channel=0（rf_chn[0] 恒为 2），
 *              不依赖调用者设置，避免 OTA 模式下在其它跳频通道发送。
 */
void ota_probe_send_poll(void)
{
    static u32 poll_tick = 0;

    // 正常模式低频 poll（100ms 节流；调用方 d24g_rf_loop 已按 0.5s 节流，
    // 此节流仅作防重入兜底）。poll 只用于查询 VERSION，不 ack 数据。
    if (!clock_time_exceed(poll_tick, 100000)) {
        return;
    }
    poll_tick = clock_time();

    // 固定配对通道 ch2（rf_chn[0]=2=2402MHz）：dongle 固定在该通道监听 poll
    device_channel = 0;

    // Build km_3_c_1_data_t packet (dongle's expected format) in user_var.ack_rf_dat.
    rf_packet_t *tx = &user_var.ack_rf_dat;
    tx->dma_len = 20;  // 1 (rf_len) + 19 (payload)
    tx->rf_len  = 19;
    tx->dat[0] = CMD_OTA_DATA;  // cmd = 10 (D24G_OTA_CMD)
    tx->dat[1] = 0;             // seq_no
    tx->dat[2] = 0;  // pno_no=0：只查询 VERSION，不 ack 数据（停等已移到 bootloader）
    // did (4 bytes) at dat[3..6], little-endian
    tx->dat[3] = (u8)(user_var.dongle_id & 0xff);
    tx->dat[4] = (u8)((user_var.dongle_id >> 8) & 0xff);
    tx->dat[5] = (u8)((user_var.dongle_id >> 16) & 0xff);
    tx->dat[6] = (u8)((user_var.dongle_id >> 24) & 0xff);
    // PID(u32) at dat[7..10], FW_VERSION(u32) at dat[11..14], expect_idx(u16) at dat[15..16]
    memset(&tx->dat[7], 0, 10);
    tx->dat[7]  = (u8)((PID >> 0)  & 0xff);  // PID u32 小端，供 dongle 按产品过滤 OTA
    tx->dat[8]  = (u8)((PID >> 8)  & 0xff);
    tx->dat[9]  = (u8)((PID >> 16) & 0xff);
    tx->dat[10] = (u8)((PID >> 24) & 0xff);
    // fw_version 统一 u32（4 字节 LE），与 USB/dongle 口径一致
    tx->dat[11] = (u8)((FW_VERSION >> 0)  & 0xff);
    tx->dat[12] = (u8)((FW_VERSION >> 8)  & 0xff);
    tx->dat[13] = (u8)((FW_VERSION >> 16) & 0xff);
    tx->dat[14] = (u8)((FW_VERSION >> 24) & 0xff);
    // crc16 at dat[17..18] — over cmd..PID..ver (17 bytes)
    extern unsigned short crc16_user(unsigned char *pD, int len);
    u16 crc = crc16_user(&tx->dat[0], 17);
    tx->dat[17] = (u8)(crc & 0xff);
    tx->dat[18] = (u8)((crc >> 8) & 0xff);

    // 使用 OTA 模式当前信道
    rf_tx_acc_code_select(0);
    rf_set_pair_access_code(pair_var.pair_accesscode);
    // 确保 poll 的 RX 收窗监听 pipe0（pair access code）上的 OTA 帧。
    // 双向通讯（BIDIR_COMM_ENABLE）开启时，数据命令路径会把 RX 配成
    // 只开 pipe1（rf_rx_acc_code_enable(0x02)），若 poll 沿用残留配置
    // 就收不到 pipe0 上捎带的 VERSION，导致 OTA 无法触发。这里显式全开。
    rf_rx_acc_code_enable(0x03);

    // Mark that this TX uses AUTO mode (rf_stx_to_rx). TX-done IRQ handler
    // skips its rf_state handling when this is set.
    ota_auto_tx_pending = 1;

    // poll 收窗 2ms：poll TX(18B) + dongle ACK(36B) 约 0.6ms，加上 dongle
    // 处理/切换延迟，2ms 收窗留足余量。
    rf_stx_to_rx((u8 *)tx, 2000);
}
#endif // (D24G_OTA_ENABLE)

void d24_tx_main_loop(void)
{	
#if (D24G_OTA_ENABLE)
    // 正常/配对模式：消费缓存的 OTA VERSION 查询。版本不同则 d24g_ota_write()
    // 写 OTA 标记并重启，由 bootloader 完成固件接收（无需在此收发 OTA 数据）。
    if (ota_buff_valid_flag) {
        d24g_ota_write(&ota_buff);
        ota_buff_valid_flag = 0;
    }
#endif // (D24G_OTA_ENABLE)

	_attribute_data_retention_ static u32 tick_loop = 0; //tick loop tick init 0
	u16 adv_send_interval = 8000; //adv_send_interval init 8ms

	if (device_status == STATE_NORMAL)
	{
		adv_send_interval = user_cfg.report_rate * 1000;//report time
	}

	if (need_suspend_flag)
	{
		/* The device enters suspend mode, the interval is determined by the suspend wake-up interval time*/
		tick_loop = clock_time();//update tick loop
		ui_loop_24g(); //UI polling and processing
	}
	else if (clock_time_exceed(tick_loop, adv_send_interval))
	{
		/* The device disallow to enter suspend mode, the interval is determined by temp * 1us */
		tick_loop += adv_send_interval*CLOCK_SYS_TIMER_CLK_1US;//update tick loop
		wakeup_next_tick = clock_time();//update wakeup next tick
		ui_loop_24g();
	}
	d24g_rf_loop(); //rf state machine deal

	/* normal mode need to go into deep*/
#if (ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG==0)
	#if DEEP_SLEEP_ENABLE
		pm_poll();//pm deal
	#endif
#endif
}
