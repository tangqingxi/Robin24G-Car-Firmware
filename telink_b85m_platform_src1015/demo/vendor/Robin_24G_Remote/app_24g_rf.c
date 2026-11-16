/********************************************************************************************************
 * @file     AAA_24G_RF.c
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
#include "app_toy/app_layer/toy_log.h"

#define DEVICE_TYPE_INDEX   1 // 1 mouse, 2 keyboard


#define  HOPING_FREQ    0   //hoping freq 

#define  RF_TX_TO_RX_AUTO_MODE_ENABLE  1    //tx to tx auto mode

#define  RF_FAST_TX_SETTING_ENABLE  0    //fast tx enable

#define  RF_FAST_RX_SETTING_ENABLE  0	//fast rx enable

_attribute_data_retention_ unsigned char rf_state = 0;//rf_state parameter
extern u8 device_status;
//_attribute_data_retention_user volatile unsigned int  rf_rx_timeout_us = D24G_PAIR_TIMER_OUT;

// RF link-layer shared state globals (Remote instance, see toy_common/app_24g_rf_common.h).
_attribute_data_retention_ User_Relevant_Var_T user_var;
_attribute_data_retention_ Pair_Relevant_Var_T pair_var;

#if (D24G_OTA_ENABLE)
// Set to 1 by ota_probe_send_poll() before rf_stx_to_rx (AUTO mode TX).
// TX-done IRQ handler skips its rf_state handling when this is set, letting
// the hardware's auto TX→RX transition proceed normally. Cleared to 0 in
// the TX-done IRQ handler.
_attribute_data_retention_ u8 ota_auto_tx_pending = 0;
#endif // (D24G_OTA_ENABLE)

//u32 ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;

_attribute_data_retention_ u8 device_channel = 0;//device frequency channel 

_attribute_data_retention_ unsigned char device_ack_received = 0;//device ack received paremeter
_attribute_data_retention_ u32 last_rx_tick = 0; // last RX time for channel hold

struct app_spi_s * user_api_ptr;
//_attribute_data_retention_user int		device_sync = 0;

//-------------------- rf init------------------
#define FRE_OFFSET 	0
//#define FRE_STEP 	5

// RF hopping channels (device-specific mapping). Filled at runtime in custom_init().
_attribute_data_retention_ unsigned char rf_chn[5] = {0};

// Channels are defined as (MHz - 2400)
#define DEFAULT_RF_CHAN 2 // 2402 MHz
#define MAX_RF_CHAN 40    // 2440 MHz max

// Reserved channels for collision replacement
#define RESERVED_CHAN_0 3   // 2403 MHz
#define RESERVED_CHAN_1 4   // 2404 MHz
#define RESERVED_CHAN_2 37  // 2437 MHz
#define RESERVED_CHAN_3 38  // 2438 MHz

// Selectable channel range: 2405..2436 (32 channels)
#define SELECT_CHAN_BASE 5   // 2405 MHz
#define SELECT_CHAN_CNT  32

#define SELECT_CHAN_LOW_SENSI	24

void init_rf_chn_by_device_id(u32 device_id)
{
	// Channel 0: fixed default (2402 MHz)
	rf_chn[0] = (unsigned char)(FRE_OFFSET + DEFAULT_RF_CHAN);

	// Channels 1..4: each byte of DID & 0x1F, mapped to 2405..2436
	u8 b0 = (u8)((device_id >> 0) & 0xFF);
	u8 b1 = (u8)((device_id >> 8) & 0xFF);
	u8 b2 = (u8)((device_id >> 16) & 0xFF);
	u8 b3 = (u8)((device_id >> 24) & 0xFF);

	u8 ch1 = (u8)(SELECT_CHAN_BASE + (b0 & 0x1F));
	u8 ch2 = (u8)(SELECT_CHAN_BASE + (b1 & 0x1F));
	u8 ch3 = (u8)(SELECT_CHAN_BASE + (b2 & 0x1F));
	u8 ch4 = (u8)(SELECT_CHAN_BASE + (b3 & 0x1F));

	// If any duplication, replace with reserved channels
	if (ch1 == rf_chn[0] || ch1 == SELECT_CHAN_LOW_SENSI ) 
		ch1 = RESERVED_CHAN_0;
	if (ch2 == rf_chn[0] || ch2 == ch1 || ch2 == SELECT_CHAN_LOW_SENSI) 
		ch2 = RESERVED_CHAN_1;
	if (ch3 == rf_chn[0] || ch3 == ch1 || ch3 == ch2 || ch3 == SELECT_CHAN_LOW_SENSI) 
		ch3 = RESERVED_CHAN_2;
	if (ch4 == rf_chn[0] || ch4 == ch1 || ch4 == ch2 || ch4 == ch3 || ch4 == SELECT_CHAN_LOW_SENSI) 
		ch4 = RESERVED_CHAN_3;

	rf_chn[1] = (unsigned char)(FRE_OFFSET + ch1);
	rf_chn[2] = (unsigned char)(FRE_OFFSET + ch2);
	rf_chn[3] = (unsigned char)(FRE_OFFSET + ch3);
	rf_chn[4] = (unsigned char)(FRE_OFFSET + ch4);
}

static void set_unpaired_fixed_channel(void)
{
	// Use fixed 2402 MHz when unpaired
	unsigned char ch = (unsigned char)(FRE_OFFSET + DEFAULT_RF_CHAN);
	for (u8 i = 0; i < 5; i++)
	{
		rf_chn[i] = ch;
	}
}

u8 caculate_data_checksum_handle(u8 *pdata, u8 length)
{
	u8 result = 0;

	for (u8 i=0; i<length; i++)
	{
		result += pdata[i];
	}

	return result;
}

/**
 * @brief       This function get rx rssi value
 * @param[in]   p - rf packet pointer
 * @return      rssi value
 * @note
 */
u8 get_rx_rssi_handle(rf_packet_t *p)
{
	unsigned char *p_rf_rx_data_start = (unsigned char *)&p->dma_len;
	return GET_RX_RSSI(p_rf_rx_data_start);
}


#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)//fast tx or rx enable

volatile u32 tick_rf_fast_setting=0;
volatile int rf_fast_setting_flag=0;//0 init 1:time out  2:enable fast set
#if RF_FAST_RX_SETTING_ENABLE

/**
 * @brief       This function init rf_rx fast settle
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_rx_fast_settle_init()
{
	write_reg8(0x1284,(read_reg8(0x1284)&0x87)|(0x6<<3));
	//write_reg8(0x1284,(read_reg8(0x1284))|0x37);
	//rx timing sequence
	write_reg8(0x12b0,0x00);
	write_reg8(0x12b1,read_reg8(0x12b1)&0xfe);
	write_reg8(0x12b2,0x10);
	write_reg8(0x12b3,read_reg8(0x12b3)&0xfe);
	write_reg8(0x12b4,0x10);
	write_reg8(0x12b5,read_reg8(0x12b5)&0xfe);
	write_reg8(0x12b6,0x32);
	write_reg8(0x12b7,read_reg8(0x12b7)&0xfe);
	write_reg8(0x12b8,0x58);
	write_reg8(0x12b9,read_reg8(0x12b9)&0xfe);
	write_reg8(0x12ba,0x58 );
	write_reg8(0x12bb,read_reg8(0x12bb)&0xfe);
}


/**
 * @brief       This function set_rx_fast_settle en
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_rx_fast_settle_en(void)
{
	write_reg8(0x1229,read_reg8(0x1229)|0x01);
}

/**
 * @brief       This function set fast_settle dis
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_rx_fast_settle_dis(void)
{
	write_reg8(0x1229,read_reg8(0x1229)&0xfe);
}
#endif

#if RF_FAST_TX_SETTING_ENABLE //if rf tx setting enable

/**
 * @brief       This function init rf_tx fast settle
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_tx_fast_settle_init()
{
	write_reg8(0x1284,(read_reg8(0x1284)&0xf8)|0x02);

	//write_reg8(0x1284,(read_reg8(0x1284))|0x02);

	write_reg8(0x12a4,0x00);
	write_reg8(0x12a5,read_reg8(0x12a5)&0xfe);
	write_reg8(0x12a6,0x10);
	write_reg8(0x12a7,read_reg8(0x12a7)&0xfe);
	write_reg8(0x12a8,0x62);
	write_reg8(0x12a9,read_reg8(0x12a9)&0xfe);
	write_reg8(0x12aa,0x64);
	write_reg8(0x12ab,read_reg8(0x12ab)&0xfe);
	write_reg8(0x12ac,0x6a);
	write_reg8(0x12ad,read_reg8(0x12ad)&0xfe);
	write_reg8(0x12ae,0x6a);
	write_reg8(0x12af,read_reg8(0x12af)&0xfe);
	write_reg8(0x12bc,0x62);
	write_reg8(0x12bd,read_reg8(0x12bd)&0xfe);
}


/**
 * @brief       This function set rf_tx fast settle en
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_tx_fast_settle_en(void)
{
	write_reg8(0x1229,read_reg8(0x1229)|0x02);
}

/**
 * @brief       This function set rf_tx fast settle dis
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_tx_fast_settle_dis(void)
{
	write_reg8(0x1229,read_reg8(0x1229)&0xfd);
}
#endif//end if rf tx fast setting enable
#endif//end if rf tx or rx fast setting enable











/**
 * @brief       This function set to init rf drv private 2m
 * @return      
 * @note        
 */
void rf_drv_private_1m_init(void)
{
	rf_mode_init();//init rf mode
	//rf_set_pri_2M_mode();//set pri 2m mode
	//rf_set_pri_250K_mode();//set pri 250k mode
	rf_set_pri_1M_mode();//set pri 1m mode
	//// add end

	write_reg16(0xf06, 0);//rx wait time
	write_reg8(0x0f0c, 0x5c); //rx settle time: Default 150us, minimum 93us(0x5c+1)

	write_reg16(0xf0e, 0);//tx wait time
	write_reg8(0x0f04, 0x70);//tx settle time: Default 150us, minimum 113us(0x70+1)

	write_reg8(0xf10, 0);// wait time on NAK

	// Set the initial RX buffer to user_var's rf_rx_buff array
	rf_rx_buffer_set((u8 *)(user_var.rf_rx_buff), PKT_BUFF_SIZE, 0);

#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)	//if rf fast tx or rx enable
	rf_fast_setting_flag=0;//fast setting flag set 0
	tick_rf_fast_setting=clock_time()|1;//update rf fast setting time
	#if RF_FAST_RX_SETTING_ENABLE	//must fist  call rx fast settle init
		rf_rx_fast_settle_init();//init rx fast settle init first
		rf_rx_fast_settle_dis();//set rf fast settle dis
	#endif

	#if RF_FAST_TX_SETTING_ENABLE	
		rf_tx_fast_settle_init();//init tx fast settle init first
		rf_tx_fast_settle_dis();//set tx fast settle dis
	#endif
#endif

	reg_dma_chn_en |= FLD_DMA_CHN_RF_RX  | FLD_DMA_CHN_RF_TX;//enalbe dma rf rx and dma rf tx
	// Note: rf_rx_buffer_set already called above based on RF_MODE_RX_MULTI
	// No need to set it again here, as it would override the correct buffer

	write_reg8(0x405, (read_reg8(0x405)&0xF8)|5); //access_byte_num[2:0]
	write_reg8(0x420, 38);
	write_reg8(0x402, 0x44); //preamble length 4 bytes

	rf_set_tx_rx_off(); //reset tx rx mode

	rf_rx_acc_code_enable(0x01); //set rf acc_code
	rf_tx_acc_code_select(0); //set rx acc_code chan

	irq_disable();//irq disable
	irq_clr_src();//clear irq src
	rf_irq_src_allclr();//clear all rf irq src
	irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
	rf_irq_disable(FLD_RF_IRQ_ALL);//disable all rf irq
	rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_FIRST_TIMEOUT); //enable tx,rx,rx timeout,first time out irq
	irq_enable();//irq enable
}

#if (RF_TX_TO_RX_AUTO_MODE_ENABLE==0)
/**
 * @brief       This function set rf srx time and time out
 * @param[in]   rx_timeout	- rx time out time
 * @param[in]   tick	- 
 * @return      
 * @note        
 */
_attribute_ram_code_sec_  void my_rf_start_srx  (unsigned int tick,unsigned int rx_timeout)
{
    write_reg8 (0x800f00, 0x80);						// disable srx
	write_reg32 (0x800f28, rx_timeout-1);				// first timeout
	write_reg32(0x800f18, tick);
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg16 (0x800f00, 0x3f86);						// srx
}

/**
 * @brief       This function set single tx mode tx packet addr and tick
 * @param[in]   addr	- packet addr
 * @param[in]   tick	- 
 * @return      
 * @note        
 */
void my_rf_start_stx  (void* addr, unsigned int tick)
{
	write_reg32(0x800f18, tick);
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg8 (0x800f00, 0x85);						// single TX
	write_reg16 (0x800c0c, (unsigned short)((unsigned int)addr));//set packet addr
}
#endif

/**
 * @brief       This function  set single tx to rx mode packet addr and rf rx timeout
 * @param[in]   addr	- the addr of packet need send
 * @param[in]   tick	- 
 * @param[in]   timeout_us	- rf rx time out
 * @return      
 * @note        
 */
void rf_start_stxtorx(void *addr, unsigned int tick, unsigned int timeout_us)
{
    write_reg16(0x800f0a, timeout_us - 1);
    write_reg32(0x800f18, tick);
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	 // Enable cmd_schedule mode
    write_reg16(0x800c0c, (unsigned short)((unsigned int)addr));
    write_reg8(0x800f00, 0x87);	   // single tx2rx
}


/**
 * @brief   set pair access code
 * @param	code
 * @return	none
 */
void rf_set_pair_access_code(u32 code)
{
    write_reg32(0x800408, ((code & 0xffffff00) | 0x71));
    write_reg8(0x80040c, code);
}

/**
 * @brief   set data access code
 * @param	code
 * @return	none
 */
void rf_set_data_access_code(u32 code)
{
	write_reg32(0x800410, ((code & 0xffffff00) | 0x77));
	write_reg8(0x800414, code);
}


/**
 * @brief       This function clr all rf irq src
 * @return      
 * @note        
 */
void rf_irq_src_allclr()
{
    reg_rf_irq_status = 0xffff;
}


/**
 * @brief       This function read payload data
 * @param[in]   payload	- the buff to store rx packet
 * @return      
 * @note        
 */
u8  Read_Payload(u8 *payload)
{
    u8 *rx_pkt;
    u8 ret, i;

    rx_pkt = (u8 *) user_var.rf_rx_buff ;//pointer rx_packet

    rf_rx_buffer_set((u8 *)user_var.rf_rx_buff, PKT_BUFF_SIZE, 0);//set rx buff

    ret = rx_pkt[4] & 0x3f; //get length of rx paylaod

    for (i = 0; i < (ret + 5); i++)
    {
        payload[i] = rx_pkt[i];
    }
    return 1;
}


#if (D24G_OTA_ENABLE)
/**
 * @brief  OTA 命令响应处理：缓存 dongle 捎带的 OTA PDU 到 ota_buff，
 *         由主循环 d24g_ota_write() 消费（只处理 VERSION 查询，版本不同则
 *         写标记重启）。实际固件接收已在 bootloader 完成。
 * @param  p_rf_rx_data - RF 接收到的数据包
 * @return 0（OTA 不需要 ACK 帧）
 */
u8 rf_ota_data_response(rf_packet_t *p_rf_rx_data)
{
	rf_packet_t *p = p_rf_rx_data;

	if (RF_TPLL_PACKET_CRC_OK(user_var.rf_rx_buff) &&
	    (p->rf_len > 0) && (p->rf_len <= 59))
	{
		if ((p->dat[0] == CMD_OTA_DATA) && (p->rf_len >= sizeof(ota_data_t)))
		{
			if (ota_buff_valid_flag == 0) {
				ota_data_t *ota_dat_ptr = (ota_data_t *)&p->dat[0];
				memcpy(ota_buff.dat, ota_dat_ptr->dat, 20);
				ota_buff_valid_flag = 1;
			}
		}
	}

	user_var.rf_rx_buff[0] = 1; // clear raw_pkt
	return 0;
}
#endif // (D24G_OTA_ENABLE)

/**
 * @brief       This function deal 2m private irq
 * @return      
 * @note        
 */
void irq_handle_private_1m()
{
	unsigned short rf_irq_src = rf_irq_src_get();//get irq src

	if (rf_irq_src)
	{
		if (rf_irq_src & FLD_RF_IRQ_RX)//if irq src is rf rx
		{
			// Successfully received ACK - this is good!
			PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;//debug rf state change
			PIN_DEBUG_RF_RX_LEVEL(0);
			rf_irq_clr_src(FLD_RF_IRQ_RX);//clear rf rx src
			rf_state = RF_RX_END_STATUS;//change rf state to rf_rx_end
		}

		if (rf_irq_src & FLD_RF_IRQ_TX)//if irq src is rf tx
		{
			// TX completed
			PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;//debug rf state change
			PIN_DEBUG_RF_TX_LEVEL(0);
			rf_irq_clr_src(FLD_RF_IRQ_TX);//clear rf tx src

			PIN_DEBUG_RF_RX_TIMEOUT_LEVEL(1);//debug rf_rx time out

#if (D24G_OTA_ENABLE)
			// OTA poll 使用 rf_stx_to_rx (AUTO 模式)：TX 完成后硬件自动进入 RX，
			// 仅清标志，保持 rf_state = RF_TX_START_STATUS。若这里提前设
			// RF_IDLE，pm_poll 会在 AUTO RX 收窗期间误判空闲进入 suspend，
			// 关闭 RF 丢失 dongle 的 ACK（VERSION），导致配对后 OTA 触发失败。
			if (ota_auto_tx_pending) {
				ota_auto_tx_pending = 0;
			} else
#endif
			{
				// TX side (transmitter device)
				if (device_status >= STATE_NORMAL) {
				#if (BIDIR_COMM_ENABLE)
					// Bidirectional-comm mode: after TX keep RX to receive the
					// data ACK from Car (same as pairing-mode ACK wait).
					rf_state = RF_RX_START_STATUS;
				#else
					// Normal mode: no ACK expected, return to idle
					rf_state = RF_IDLE_STATUS;
				#endif
				} else {
					// Pairing mode: wait for ACK
					rf_state = RF_RX_START_STATUS;//change rf state to rf_rx_start
				#if (RF_TX_TO_RX_AUTO_MODE_ENABLE==0)
					my_rf_start_srx(clock_time(), rf_rx_timeout_us);//single rx time out set
				#endif
				}
			}
		}

	#if (RF_TX_TO_RX_AUTO_MODE_ENABLE==1)//if tx to rx auto mode enable
		if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT)//if irq src is rf rx time out
		{
			// No ACK received - timeout
			PIN_DEBUG_RF_RX_TIMEOUT_LEVEL(0);//debug rf_rx time out
			rf_irq_clr_src(FLD_RF_IRQ_RX_TIMEOUT);//clear irq rx timeout src
			rf_state = RF_RX_TIMEOUT_STATUS;//change to rf state to rf_rx_timeout status
		}
	#else
		if (rf_irq_src & FLD_RF_IRQ_FIRST_TIMEOUT)
		{
			PIN_DEBUG_RF_RX_TIMEOUT_LEVEL(0);//debug rf_rx time out	
			rf_irq_clr_src(FLD_RF_IRQ_FIRST_TIMEOUT);//clear irq first timeout src
			rf_state = RF_RX_TIMEOUT_STATUS;//change rf_state to rf_rx_timeout status
		}
	#endif

		irq_clr_src(); //clear irq all src
	}
}


/**
 * @brief       This function get next channel
 * @param[in]   chn	- 
 * @param[in]   mask	- 
 * @return      
 * @note        
 */
u8 get_next_channel_with_mask(u32 mask, u8 chn)
{
	// PIN_DEBUG_RF_CHN_NEXT_TOGGLE;
#if (HOPING_FREQ==0)
   	return ((chn+1)%5);//get next chn
#else
	int chn_high = (mask >> 4) & 0x0f;
	if (mask & LL_CHANNEL_SEARCH_FLAG) {
		return LL_NEXT_CHANNEL (chn);
	}
	else if (chn_high != chn) {
		return chn_high;
	}
	else {
		return mask & 0x0f;
	}
#endif
}


/**
 * @brief       This function check rf fasting setting time
 * @return      
 * @note        
 */
void check_rf_fast_setting_time()
{
#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)
	if(clock_time_exceed(tick_rf_fast_setting, 2000000))
	{
		tick_rf_fast_setting=clock_time();
		rf_fast_setting_flag=1;
	}
#endif
}

#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)
/**
 * @brief       This function check rf fasting setting flag
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void check_rf_fast_setting_flag()
{
	if(rf_fast_setting_flag==1)//timeout
	{
		rf_fast_setting_flag=0;//defualt
	}
	else
	{
		rf_fast_setting_flag=2;//enable rf fast setting flag
	}
}
#endif

#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)
/**
 * @brief       This function set rf_tx_rx setting
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_set_tx_rx_setting()
{
	if(rf_fast_setting_flag<2)
	{
	#if RF_FAST_TX_SETTING_ENABLE
		write_reg8(0x0f04, 0x70);//tx settle time: Default 150us, minimum 113us(0x70+1)
		rf_tx_fast_settle_dis();
	#endif
	
	#if RF_FAST_RX_SETTING_ENABLE
		write_reg8(0x0f0c, 0x5c); //rx settle time: Default 150us, minimum 93us(0x5c+1)
		rf_rx_fast_settle_dis();
	#endif
	}
	else //if(rf_ca_cnt==1)
	{
	#if RF_FAST_TX_SETTING_ENABLE
		write_reg8(0x0f04, 70);//tx settle time: Default 150us, minimum 113us(0x70+1)
		rf_tx_fast_settle_en();
	#endif

	#if RF_FAST_RX_SETTING_ENABLE
		write_reg8(0x0f0c, 0x5c); //rx settle time: Default 150us, minimum 93us(0x5c+1)
		rf_rx_fast_settle_en();
	#endif
	}
}
#endif

/**
 * @brief       This function set single tx to rx 
 * @param[in]   p	- packet pointer
 * @param[in]   rx_timeout_us	- rf rx timeout
 * @return      
 * @note        
 */
_attribute_ram_code_sec_ void rf_stx_to_rx(u8 *p, u32 rx_timeout_us)
{
#if (RF_FAST_TX_SETTING_ENABLE||RF_FAST_RX_SETTING_ENABLE)
	rf_set_tx_rx_setting();
#endif

	PIN_DEBUG_RF_STATE_CHANGE_TOGGLE;//debug rf state
	rf_state = RF_TX_START_STATUS;//rf state to TX status
	rf_set_tx_rx_off();//must add
	device_ack_received = 0;//ack receive reset to 0
#if RF_TX_TO_RX_AUTO_MODE_ENABLE
	rf_trx_state_set(RF_MODE_AUTO, rf_chn[device_channel]);
	rf_start_stxtorx(p, ClockTime(), rx_timeout_us);
#else
	rf_set_channel(rf_chn[device_channel], 0);//set RF's channel
	my_rf_start_stx(p, ClockTime());//rf single tx start
#endif
	reg_rf_irq_status = 0xffff;//irq status reset to ffff
}


/**
 * @brief       This function deal with rf rx packet
 * @param[in]   p_rf_data	- rf rx packet
 * @return      
 * @note        
 */
extern u8  rf_rx_process(rf_packet_t *p_pkt);
void irq_device_rx(void)
{
#if (D24G_OTA_ENABLE)
	rf_packet_t *p_rx = (rf_packet_t *)user_var.rf_rx_buff;
#endif
	// 普通帧保留 RF_TPLL_PACKET_LENGTH_OK 严格检查；
	// 仅 OTA 帧（B80B dongle，TC122X RX 时硬件重写 dma_len 导致 LENGTH_OK 误判）
	// 放宽为 CRC + rf_len 范围检查，最小化对正常收包的影响。
	if (RF_TPLL_PACKET_CRC_OK(user_var.rf_rx_buff) &&
	    (RF_TPLL_PACKET_LENGTH_OK(user_var.rf_rx_buff)
#if (D24G_OTA_ENABLE)
	     || ((p_rx->rf_len > 0) && (p_rx->rf_len <= 59))
#endif
	    ))
	{
		// rf_packet_t *p = (rf_packet_t *)user_var.rf_rx_buff;  // Unused, commented out
        // pair_ack_data_t *ack = (pair_ack_data_t *)&p->dat[0];  // Unused, commented out

		// Removed verbose debug output
		if (rf_rx_process((rf_packet_t *)user_var.rf_rx_buff) /*&& ll_chn_tick != p->tick*/)
		{
			// Removed verbose debug output

			device_ack_received = 1;
			last_rx_tick = clock_time();

		#if (HOPING_FREQ==1)
			// hold channel; switch only on RX timeout in main loop
		#endif

		}
#if (D24G_OTA_ENABLE)
		// 非配对 ACK：可能是 dongle 捎带的 OTA VERSION 查询帧
		else if (p_rx->dat[0] == CMD_OTA_DATA)
		{
			rf_ota_data_response(p_rx);
		}
#endif
		// else: ACK received but not for us - ignore silently
	}
	// else: CRC/Length error - ignore silently

	user_var.rf_rx_buff[0] = 1;
}

/**
 * @brief   Convert the access code
 * @param	code
 * @return	result
 */
static inline u32 rf_access_code_24to32 (u32 code)
{
	u32 r = (code & 0x00ffff00)>>8;
	u32 p = code & 0xff;
	u32 t = 0;
	for (int i=0; i<8; i++) {
		t = t << 2;
		t |= ((p & BIT(i)) ? 1 : 2);
		t = t << 2;
		t |= (r & 0x03);
		r = r >>2;
	}
	return t;
}

/**
 * @brief       This function custom init
 * @return      
 * @note        
 */
void custom_init(void)
{
    memset(&user_var, 0, sizeof(user_var));
	memset(&pair_var, 0, sizeof(pair_var));

	// Read chip_id (6 bytes) + toy_type (3 bits at efuse offset 7) from efuse.
	// chip_id is always read from efuse; no validity check / fallback generation.
	if (!efuse_read(EFUSE_CHIP_ID_ADDR, (u8 *)&efuse_dev_info, sizeof(efuse_dev_info_t))) {
		TOY_LOG("efuse_read chip_id failed, reboot\n");
		start_reboot();
	}
	u32 device_id = ((efuse_dev_info.chip_id[2]<<16) | (efuse_dev_info.chip_id[1]<<8) | efuse_dev_info.chip_id[0]);

	// Read access_code_group and normal_tx_power from compiled-in defaults
	u8 access_code_group = g_remote_access_code_group;
	u8 normal_tx_power = g_remote_normal_tx_power;

	pair_var.pair_accesscode = pair_access_code_array[access_code_group];
	
	user_var.dongle_id = rf_access_code_24to32(device_id);  // Temporary dongle ID, will be updated after pairing
	
	// Default to fixed channel (2402) before pairing
	set_unpaired_fixed_channel();
	TOY_LOG("---RF: unpaired fixed rf_chn:%d,%d,%d,%d,%d\n",
			rf_chn[0], rf_chn[1], rf_chn[2], rf_chn[3], rf_chn[4]);

	//efuse_dev_info.debug_enable = 1; //debug

	user_cfg.report_rate = 8; //8ms interval

    TOY_LOG("---dongle_id:0x%04x\n", user_var.dongle_id);

	if (normal_tx_power) // Use the moved normal_tx_power
	{
		user_var.tx_power_normal = (1 + normal_tx_power * 3); 
	} else {
		user_var.tx_power_normal = DEFAULT_NORMAL_TX_POWER; //default 10dBm
	}

}
