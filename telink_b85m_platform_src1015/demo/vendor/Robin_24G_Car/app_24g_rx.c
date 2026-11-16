/********************************************************************************************************
 * @file     APP_24G_RX.c
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
#include "app_24g_rx.h"
#include "app_24g_rf.h"
#include "app_toy/app_layer/app_toy.h"
#include "app_battery_check.h"
#include "toy_log.h"
#include "sdk_version.h"

extern u32 last_rx_tick;

/**
 * @brief   Check whether the configured deepsleep idle timeout is reached.
 * @return  1 = should enter deepsleep, 0 = keep running
 * @note    Mode and timeout come from retention globals, see CAR_SLEEP_TIMEOUT_*
 *          in app_config.h.
 */
static u8 car_sleep_timeout_reached(void)
{
	if (g_car_sleep_mode == CAR_SLEEP_TIMEOUT_NEVER)
	{
		return 0;   // 永不休眠
	}
	else if (g_car_sleep_mode == CAR_SLEEP_TIMEOUT_5MIN)
	{
		return (idle_count >= CAR_SLEEP_TIMEOUT_5MIN_S);
	}
	else if (g_car_sleep_mode == CAR_SLEEP_TIMEOUT_10MIN)
	{
		return (idle_count >= CAR_SLEEP_TIMEOUT_10MIN_S);
	}
	else    // CAR_SLEEP_TIMEOUT_CUSTOM
	{
		return (g_car_sleep_timeout_s != 0) && (idle_count >= g_car_sleep_timeout_s);
	}
}

// RF link-layer shared state globals (Car instance, see toy_common/app_24g_rf_common.h).
_attribute_data_retention_ User_Relevant_Var_T user_var;
_attribute_data_retention_ Pair_Relevant_Var_T pair_var;

unsigned char UserRfRcvBuffer[20];		//RF收到的新数据（用户应用层桥接缓存）
unsigned char RxdNewFlag;	//RF收到新数据的标志

/**
 * @brief       This function poll soft tick
 * @return      
 * @note        
 */
void soft_tick_poll(void)
{
	static unsigned int tick; //tick

	if (clock_time_exceed(tick, 1000)) //1ms
	{
		user_var.rx_fail += ((unsigned int)(clock_time() - tick) / CLOCK_SYS_TIMER_CLK_1MS); //rxfail ++

		tick = clock_time(); //update tick

		if (user_var.rx_fail > 40)
		{
			user_var.rx_fail = 40;
			user_var.host_channel = ((user_var.host_channel+1)%5);//update chn
			rf_set_tx_rx_off();//reset  tx rx mode
			rf_set_channel (rf_chn[user_var.host_channel],0);
			rf_set_rxmode();//rx mode
			//TOY_LOG("---RX: Hopping to channel %d due to RX timeout\n", rf_chn[user_var.host_channel]);
			//PIN_DEBUG_RF_STATE_CHANGE_TOGGLE2;
		}
	}
}


void d24_rx_main_loop(void) {
#if (D24G_OTA_ENABLE)
    // 正常模式：消费缓存的 OTA VERSION 查询。版本不同则 d24g_ota_write()
    // 写 OTA 标记并重启，由 bootloader 完成固件接收（无需在此收发 OTA 数据）。
    if (ota_buff_valid_flag) {
        d24g_ota_write(&ota_buff);
        ota_buff_valid_flag = 0;
    }
#endif // (D24G_OTA_ENABLE)

    // Periodically check watchdog and some basic maintenance tasks
    static u32 tick_loop = 0;
    if (clock_time_exceed(tick_loop, 10000)) {  // Check every 10ms
        tick_loop = clock_time();

        #if MODULE_32K_WATCHDOG_ENABLE
        wd_32k_feed();
        #endif

        // 24M RC 定时校准：每 1s 一次，保持 RF 频率精度（参考 Robin_24G_Demo）
        static u32 rc_cal_cnt = 0;
        if (0 == (rc_cal_cnt % 100)) {
            rc_24m_cal();
        }
        rc_cal_cnt++;

        // Pairing timer timeout check (100ms)
        if (pair_var.remoteid != 0)
        {
            if (clock_time_exceed(pair_var.pair_timer, 100000)) // 100ms timeout
            {
                pair_var.remoteid = 0;
                pair_var.pair_timer = 0;
            }
        }
    }

    // Hold current channel while receiving; hop only if no RX for 40ms
    if (last_rx_tick == 0) {
        last_rx_tick = clock_time();
    }
    if (clock_time_exceed(last_rx_tick, 38000)) {
        user_var.host_channel = (user_var.host_channel + 1) % 5;
        rf_set_tx_rx_off();
        rf_set_channel(rf_chn[user_var.host_channel], 0);
        rf_set_rxmode();
        last_rx_tick = clock_time();
    }

    #if DEEP_SLEEP_ENABLE
        // Enter deepsleep when no remote command for the configured sleep
        // timeout (g_car_sleep_mode, see CAR_SLEEP_TIMEOUT_* in app_config.h).
        // reset_idle_status() is called on each valid CMD_DATA.
        idle_status_poll();
        if (car_sleep_timeout_reached())
        {
            TOY_LOG("---Car idle %d s, enter deepsleep.\n", idle_count);
            reset_idle_status();
            enter_deep(ANA_DEEP_SLEEP);//enter deep sleep
        }
    #endif
}


void user_rf_ack_send(unsigned char pipe, int rf_length)
{
	user_var.ack_rf_dat.dma_len = rf_length+1;
	user_var.ack_rf_dat.rf_len = rf_length;
	//PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;
	rf_tx_acc_code_select(pipe);
	rf_set_tx_rx_off();
	rf_start_stx((void *)&user_var.ack_rf_dat.dma_len, clock_time());
}

/**
 * @brief       This function save device info in fw
 * @param[in]   p	- 
 * @return      
 * @note        
 */
int set_device_id_in_firmware (pair_data_t *p_pair_data)
{
	u32 remote_id = p_pair_data->remote_id;

    if ((user_var.key_enc.did == 0) || (user_var.key_enc.did == remote_id))
    {
		pair_var.pair_success_flag = 1; //set pair_success_flag
		user_var.key_enc.did = remote_id;   // save remote ID

		// Re-init hopping channels after pairing (offset based on remote ID)
		init_rf_chn_by_device_id(remote_id);
		TOY_LOG("remote_id = 0x%08x\n", remote_id);
		TOY_LOG("---Car RF: paired hop rf_chn:%d,%d,%d,%d,%d\n",
			rf_chn[0], rf_chn[1], rf_chn[2], rf_chn[3], rf_chn[4]);

		return 1;
    }

	return 0;
}

/**
 * @brief       This function cheak ram data
 * @param[in]   p	-
 * @return
 * @note
 */
u8 check_from_ram(pair_data_t *p, u8 length)
{
	_attribute_data_retention_ static u32 buf = 0;

	(void)length;
	if (buf != p->remote_id)
	{
		buf = p->remote_id;
		return 0;
	}

	return 1;
}

/**
 * @brief       This function deal rf pair response
 * @param[in]   p_rf_rx_data	- 
 * @return      
 * @note        
 */
unsigned char rf_pair_response(rf_packet_t *p_rf_rx_data)
{
	pair_data_t *p_pair_data = (pair_data_t *)&p_rf_rx_data->dat[0];
	pair_ack_data_t *p_pair_ack_data = (pair_ack_data_t *)&user_var.ack_rf_dat.dat[0];

#if (RF_CMD_PAIR_DAT_LEN >= 2)
	//配对请求校验 Product_VID（2Bytes）：不匹配则不应答，
	//防止漂移车(0x2609)与其它产品遥控器(如 HspdCar 0x2432)误配对
	if ((p_pair_data->dat[0] != CAR_PAIR_ACK_PARA1) ||
	    (p_pair_data->dat[1] != CAR_PAIR_ACK_PARA2))
	{
		return 0;
	}
#endif

	if (pair_var.remoteid == 0)
	{
		// First pair cmd received, record remote id and start timer
		pair_var.remoteid = p_pair_data->remote_id;
		pair_var.pair_timer = clock_time();
	}

	// Only reply pair_ack when remoteid matches
	if (pair_var.remoteid == p_pair_data->remote_id)
	{
		// Refresh timer on each valid pair cmd
		pair_var.pair_timer = clock_time();

		p_pair_ack_data->car_id = user_var.dongle_id; // Car ID
		p_pair_ack_data->remote_id = p_pair_data->remote_id; // Remote ID
		p_pair_ack_data->key = 0;
		p_pair_ack_data->cmd = CMD_ACK;
#if (RF_CMD_ACK_DAT_LEN >= 1)
		memset(p_pair_ack_data->dat, 0, RF_CMD_ACK_DAT_LEN); // retention global: zero first
		p_pair_ack_data->dat[0] = CAR_PAIR_ACK_PARA1; // customer extension (maps to legacy para1)
#endif
#if (RF_CMD_ACK_DAT_LEN >= 2)
		p_pair_ack_data->dat[1] = CAR_PAIR_ACK_PARA2; // customer extension (maps to legacy para2)
#endif

		user_rf_ack_send(PIPE_PARING, sizeof(pair_ack_data_t));
		return 1;
	}

	// remoteid not match, do not reply
	return 0;
}

void user_rf_ack_reset(unsigned char pipe, rf_packet_t *ack_rf_data, int rf_length)
{
	ack_rf_data->dma_len = rf_length+1;
	ack_rf_data->rf_len = rf_length;
	PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;
}




unsigned char save_km_data_to_buffer(unsigned char *rf_rx_pdata, unsigned char *user_pdata)
{
	(void)rf_rx_pdata;
	(void)user_pdata;
	return 1;
}

/**
 * @brief   RF keyboard or mouse data process and ACK
 * @param	p_rf_rx_data: RF received data
 * @param	hw_crc_ok: hardware crc16 check result
 * @return	0:no need ACK, 1:need ACK
 */
unsigned char rf_km_data_response(rf_packet_t *p_rf_rx_data, unsigned char pipe)
{
	(void)pipe;
	rf_cmd_frame_t *km_ptr = (rf_cmd_frame_t*)&p_rf_rx_data->dat[0];

	if (km_ptr->cmd != CMD_DATA) {
		return 0;
	}

	if (user_var.key_enc.did == 0) {
		/* 
		* When a powered-on car receives control commands from an already-paired remote:
		* - Both use the same RF channel 2 and the same data access code
		* - If gid matches local car ID (dongle_id), save remote ID to RAM and switch to paired state
		* - Then re-initialize hopping channels based on the remote ID
		*/
		if(km_ptr->car_id == user_var.dongle_id){
			u32 remote_id = km_ptr->remote_id;

			// Re-init hopping channels after pairing (offset based on remote ID)
			init_rf_chn_by_device_id(remote_id);
			TOY_LOG("remote_id = 0x%08x\n", remote_id);
			TOY_LOG("---Car RF: paired hop rf_chn:%d,%d,%d,%d,%d\n",
				rf_chn[0], rf_chn[1], rf_chn[2], rf_chn[3], rf_chn[4]);

			user_var.key_enc.did = remote_id;	// Save remote ID to RAM
			pair_var.pair_success_flag = 1;	// Mark as paired

			reset_idle_status();//valid command received, refresh idle counter for deepsleep
		}
		return 0;
	}else{
		// Validate remote ID
		if (km_ptr->remote_id != user_var.key_enc.did) {
			return 0;
		}
		// Require RX_ID (gid) to match local car ID when paired.
		if (km_ptr->car_id != user_var.dongle_id) {
			return 0;
		}
	}

#if (BIDIR_COMM_ENABLE)
	// 注意：必须先回复 ACK，再处理接收到的控制命令。
	// 若先处理命令（入队/打印/业务逻辑），处理时间过长会占用 Remote 的
	// ACK 接收窗口，导致 Remote 超时收不到 ACK。
	rf_cmd_ack_frame_t *ack = (rf_cmd_ack_frame_t *)&user_var.ack_rf_dat.dat[0];
	ack->car_id = user_var.dongle_id;
	ack->remote_id = km_ptr->remote_id;
	ack->key = km_ptr->key;   // echo 被应答命令的按键（0 = keepalive）
	ack->cmd = CMD_ACK;
#if (RF_CMD_DATA_ACK_DAT_LEN >= 1)
	memset(ack->dat, 0, RF_CMD_DATA_ACK_DAT_LEN); // retention global: zero first
	ack->dat[0] = low_batt_flg;                  // car status: 0=normal, 1=low battery (maps to legacy para1)
#endif
#if (RF_CMD_DATA_ACK_DAT_LEN >= 2)
	ack->dat[1] = 0;                             // reserved for customer extension (maps to legacy para2)
#endif
	user_rf_ack_send(PIPE_MOUSE, sizeof(rf_cmd_ack_frame_t));
#endif

	// 用户应用层数据桥：把 RF 收到的命令帧拷贝到 UserRfRcvBuffer，置 RxdNewFlag，
	// 由 UserRxdFunc.c 的 RxdFunction_loop() 消费（car_control 等业务处理）。
	for(u8 i=0; i<20; i++)
	{
		UserRfRcvBuffer[i]=0;
	}
	UserRfRcvBuffer[0] = km_ptr->car_id;
	UserRfRcvBuffer[1] = km_ptr->car_id >> 8;
	UserRfRcvBuffer[2] = km_ptr->car_id >> 16;
	UserRfRcvBuffer[3] = km_ptr->car_id >> 24;
	UserRfRcvBuffer[4] = km_ptr->remote_id;
	UserRfRcvBuffer[5] = km_ptr->remote_id >> 8;
	UserRfRcvBuffer[6] = km_ptr->remote_id >> 16;
	UserRfRcvBuffer[7] = km_ptr->remote_id >> 24;
	UserRfRcvBuffer[8] = km_ptr->key;
	UserRfRcvBuffer[9] = km_ptr->cmd;
#if (RF_CMD_DATA_DAT_LEN >= 1)
	UserRfRcvBuffer[10] = km_ptr->dat[0];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 2)
	UserRfRcvBuffer[11] = km_ptr->dat[1];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 3)
	UserRfRcvBuffer[12] = km_ptr->dat[2];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 4)
	UserRfRcvBuffer[13] = km_ptr->dat[3];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 5)
	UserRfRcvBuffer[14] = km_ptr->dat[4];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 6)
	UserRfRcvBuffer[15] = km_ptr->dat[5];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 7)
	UserRfRcvBuffer[16] = km_ptr->dat[6];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 8)
	UserRfRcvBuffer[17] = km_ptr->dat[7];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 9)
	UserRfRcvBuffer[18] = km_ptr->dat[8];
#endif
#if (RF_CMD_DATA_DAT_LEN >= 10)
	UserRfRcvBuffer[19] = km_ptr->dat[9];
#endif

	RxdNewFlag=1;

	// Dispatch the control command to the car control entry.
	toy_car_ctrl_on_rf_km_dat(km_ptr, 0);
	reset_idle_status();//valid command received, refresh idle counter for deepsleep
	return 1;
}

#if (D24G_OTA_ENABLE)
/**
 * @brief   OTA 命令响应处理：缓存 dongle 捎带的 OTA PDU 到 ota_buff，
 *          由主循环 d24g_ota_write() 消费（只处理 VERSION 查询，版本不同则
 *          写标记重启）。实际固件接收已在 bootloader 完成。
 * @param   p_rf_rx_data - RF 接收到的数据包
 * @return  0（OTA 不需要 ACK 帧）
 */
u8 rf_ota_data_response(rf_packet_t *p_rf_rx_data)
{
	if (p_rf_rx_data->dat[0] != CMD_OTA_DATA ||
	    p_rf_rx_data->rf_len < sizeof(ota_data_t)) {
		return 0;
	}

	if (ota_buff_valid_flag == 0) {
		ota_data_t *ota_dat_ptr = (ota_data_t *)&p_rf_rx_data->dat[0];
		memcpy(ota_buff.dat, ota_dat_ptr->dat, 20);
		ota_buff_valid_flag = 1;
	}
	return 0;
}
#endif // (D24G_OTA_ENABLE)




/**
 * @brief       This function handle rf_rx irq
 * @return      
 * @note        
 */
void rf_rx_irq_handle(void)
{
	unsigned char need_ack = 0; //
	unsigned char pipe = 0;// Default to pipe: 0 for pairing|| 1:for mouse_data || 2:for keyboard_data

	user_var.raw_pkt = (u8 *) (user_var.rf_rx_buff + user_var.rf_rx_wptr * PKT_BUFF_SIZE); //get current RX butter address
	user_var.rf_rx_wptr = (user_var.rf_rx_wptr + 1) & 1; //skip to next RX butter
	rf_rx_buffer_set((u8 *)(user_var.rf_rx_buff + user_var.rf_rx_wptr * PKT_BUFF_SIZE), PKT_BUFF_SIZE, 0); //clear next RX butter
	rf_packet_t *p_rf_rx_data = (rf_packet_t *)user_var.raw_pkt; //get RX butter

	/* check data crc16 and data length */
	// 普通帧保留 RF_TPLL_PACKET_LENGTH_OK 严格检查；
	// 仅 OTA 帧（B80B dongle，TC122X RX 时硬件重写 dma_len 导致 LENGTH_OK 误判）
	// 放宽为 CRC + rf_len 范围检查，最小化对正常收包的影响。
	if (RF_TPLL_PACKET_CRC_OK(user_var.raw_pkt) &&
	    (RF_TPLL_PACKET_LENGTH_OK(user_var.raw_pkt)
#if (D24G_OTA_ENABLE)
	     || ((p_rf_rx_data->rf_len > 0) && (p_rf_rx_data->rf_len <= 59))
#endif
	    ))
    { //check right
		pipe = GET_RX_PIPE(user_var.raw_pkt);
		pair_data_t *pair_data_ptr = (pair_data_t*)&p_rf_rx_data->dat[0]; //pair data

#if (D24G_OTA_ENABLE)
		// ---- OTA 数据/命令帧 ----
		// 独立 API 处理：rf_ota_data_response()（与 rf_km_data_response 风格一致）
		if (p_rf_rx_data->dat[0] == CMD_OTA_DATA)
		{
			rf_ota_data_response(p_rf_rx_data);
		}
		// ---- 普通命令帧 ----
		else if (pair_data_ptr->cmd == CMD_DATA)
		{
			pipe = PIPE_MOUSE;
			need_ack = rf_km_data_response(p_rf_rx_data, pipe);//data rsp
		}
		else if (pair_data_ptr->cmd == CMD_PAIR)
		{
			pair_data_t *p_pair_data = (pair_data_t *)&p_rf_rx_data->dat[0];
			u32 remote_id = p_pair_data->remote_id;
			if ((user_var.key_enc.did == 0) || (user_var.key_enc.did == remote_id))
			{
				need_ack = rf_pair_response(p_rf_rx_data);//pair rsp
			}
		}
#else
		// NOTE: do NOT gate CMD_DATA on pipe>0 - GET_RX_PIPE() reads rf_len's
		// low bits, which depends on the payload size (pair_data_t is 10 bytes
		// in the legacy format + optional dat[]), so it cannot identify the
		// RX pipe reliably. The command type (CMD_DATA vs CMD_PAIR) and the
		// access code already distinguish the pipes.
		if (pair_data_ptr->cmd == CMD_DATA)
		{
			pipe = PIPE_MOUSE;
			need_ack = rf_km_data_response(p_rf_rx_data, pipe);//data rsp
		}
		else if (pair_data_ptr->cmd == CMD_PAIR)
		{
			pair_data_t *p_pair_data = (pair_data_t *)&p_rf_rx_data->dat[0];
			u32 remote_id = p_pair_data->remote_id;
			if ((user_var.key_enc.did == 0) || (user_var.key_enc.did == remote_id))
			{
				need_ack = rf_pair_response(p_rf_rx_data);//pair rsp
			}
		}
#endif // (D24G_OTA_ENABLE)
    }

    if (need_ack)
	{ //received data valid
    	user_var.rx_fail = 0;//rx fail 0
    	last_rx_tick = clock_time();//收到有效包刷新，重置 38ms 调频计时（hold channel）
	}

    user_var.raw_pkt[0] = 1; //clear raw_pkt
}


/**
 * @brief       This function handle rf tx handle
 * @return      
 * @note        
 */
void rf_tx_irq_handle(void)
{
	rf_set_tx_rx_off();		//add for save
	rf_set_channel (rf_chn[user_var.host_channel], 0);
	rf_set_rxmode ();//rxmode

}


