/********************************************************************************************************
 * @file     AAA_public_Config.h
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

#ifndef _APP_PUBLIC_CONFIG_H_
#define _APP_PUBLIC_CONFIG_H_

#include "types.h"
#include "app_config.h"
#include "driver.h"
#include "string.h"

#include "app_24g_rf_frame.h"
#include "app_24g_rf.h"

//#include "lark.h"

typedef	struct {
	u32		size;//fifo size
	u16		num;// num
	u8		wptr;
	u8		rptr;
	u8*		p;
}	__attribute__ ((aligned (4))) my_fifo_t;
#define		MYFIFO_INIT(name,size,n)		_attribute_data_retention_ u8 name##_b[size * n]__attribute__((aligned(4)))/*={0}*/;_attribute_data_retention_ my_fifo_t name = {size,n,0,0, name##_b}
/**
 * @brief       This function init fifo
 * @param[in]   f	- pointer to fifo struct
 * @param[in]   n	- fifo num
 * @param[in]   p	- fifo data pointer
 * @param[in]   s	- fifo size
 * @return      
 * @note        
 */
void my_fifo_init (my_fifo_t *f, int s, u8 n, u8 *p);

/**
 * @brief       This function return fifo write pointer
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
u8*  my_fifo_wptr (my_fifo_t *f);

/**
 * @brief       This function wptr ++
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
void my_fifo_next (my_fifo_t *f);
/**
 * @brief       This function push data to fifo f
 * @param[in]   f	- fifo f
 * @param[in]   n	- data len
 * @param[in]   p	- the data pointer need sended
 * @return      
 * @note        
 */
int  my_fifo_push (my_fifo_t *f, u8 *p, int n);

/**
 * @brief       This function rptr ++
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
void my_fifo_pop (my_fifo_t *f);

/**
 * @brief       This function get fifo data
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
u8 * my_fifo_get (my_fifo_t *f);

/**
 * @brief       This function reset fifo
 * @param[in]   f	- fifo need reset
 * @return      
 * @note        
 */
 void my_fifo_reset (my_fifo_t *f);

#define abs(a)   (((a)>0)?((a)):(-(a)))

#define U32_MAX ((u32)0xffffffff)
#define U16_MAX ((u16)0xffff)
#define U8_MAX ((u8)0xff)
#define U31_MAX ((u32)0x7fffffff)
#define U15_MAX ((u16)0x7fff)
#define U7_MAX ((u8)0x7f)

//--------------------------sys clock define--------------------------------

/* Define system clock */
#define CLOCK_SYS_CLOCK_HZ  	24000000

#if(MCU_CORE_B89)
#if(CLOCK_SYS_CLOCK_HZ==12000000)
	#define SYS_CLK  	SYS_CLK_12M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==16000000)
	#define SYS_CLK  	SYS_CLK_16M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==24000000)
	#define SYS_CLK  	SYS_CLK_24M_Crystal
#endif
#else
#if(CLOCK_SYS_CLOCK_HZ==12000000)
	#define SYS_CLK  	SYS_CLK_12M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==16000000)
	#define SYS_CLK  	SYS_CLK_16M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==24000000)
	#define SYS_CLK  	SYS_CLK_24M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==32000000)
	#define SYS_CLK  	SYS_CLK_32M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==48000000)
	#define SYS_CLK  	SYS_CLK_48M_Crystal
#endif
#endif

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};


//-----------------------------DPI SAVE FLG -----------------------------
#define USED_DPI_DEEP_ANA_REG		DEEP_ANA_REG10 	//u8,can save 8 bit dpi info when deep


//-----------------------------DEEP SAVE FLG -----------------------------
#define PAIR_DEEP_ANA_REG			DEEP_ANA_REG2 	//u8,can save 8 bit info when deep

#define FUN_MODE_ANA_FLG	        0X06			//BIT 1,BIT2
#define CLEAR_FUN_MODE_ANA_FLAG		0xF9


//---------------------------security References----------------------------------------
#define RF_TX_FIFO_ALLOW_NUM     	9			//14-5 audio -1 mouse =8


//------------tx power define-------------------
#define DEFAULT_NORMAL_TX_POWER   	RF_POWER_P10p00dBm
#define DEFAULT_EMI_TX_POWER      	RF_POWER_P0p00dBm


#define USER_BTN_MAIN 		0X01

typedef struct
{
    u32 bin_crc;
    u32 fw_version;
} output_dev_info_t;
extern output_dev_info_t output_dev_info;

//--------------------------------------------------
typedef enum
{
    STATE_POWERON = 0,
	STATE_SYSNC,
    STATE_PAIRING,
    STATE_NORMAL,
    STATE_OTA,
} STATE_MODE;

typedef enum
{
	IDLE = 0,
	USB_DEVICE_CONNECT_PC,
	USB_DEVICE_CHECK_PC_SLEEP,
	USB_DEVICE_DISCONECT_PC,//may be pc power off
	USB_DEVICE_UNPLUG,
}USB_DEVICE_STATUS;

typedef struct //The length has to be a multiple of 4!!!
{
    u8  report_rate;
#if EMI_TEST_FUN_ENABLE_AAA
	u8  emi_tx_power;
#endif

} custom_cfg_t;


typedef struct
{
	u8 chip_id[6]; //0~5 -> 12 34 56 00 00 00

	u8 rsv_byte_0; //6: reserved (paring_type etc.)

	union
	{
		struct
		{
			u8 toy_type: 3;
			u8 rsv_bit_1 : 5;
		};
		u8 union_byte_1;
	}; //7
} efuse_dev_info_t;

extern efuse_dev_info_t efuse_dev_info;

extern u8 report_rate;

extern custom_cfg_t user_cfg;
extern u8 pub_key[];
extern u8 private_key[];
extern u8 device_name_len;
extern u8 tbl_advData_aaa[];

extern u8 sensor_type;
extern u8 product_id1;
extern u8 product_id2;
extern u8 product_id3;
extern u8 connect_ok;
extern u8 has_double_click_left;


typedef enum{
	RF_IDLE_STATUS =	0,
    RF_TX_START_STATUS=1,
    RF_TX_END_STATUS=2,
    RF_RX_START_STATUS=3,
    RF_RX_END_STATUS=4,
	RF_RX_TIMEOUT_STATUS=5,
		
}RF_STATUS_USER;
extern unsigned char rf_state;
extern unsigned char device_ack_received;
extern u8 device_channel;
extern volatile unsigned int rf_rx_timeout_us;


typedef enum
{
    ANA_POWER_ON = 0,
    ANA_DEEP_SLEEP = 1,
    ANA_MODE_CHANGE_REBOOT = 2,
    ANA_PAIR_REBOOT = 3,
    ANA_CONN_PARAM_FAIL_REBOOT = 4,
    ANA_CLEAR_FLAG = 5,
    ANA_MULTI_DEVICE_REBOOT = 6,
    ANA_MODE_CHANGE_TO_CHARGING_REBOOT = 7,
    ANA_MODE_CHANGE_TO_USB_REBOOT = 8,
    ANA_BATTERY_LOW_ENTER_DEEPSLEEP = 9,
    ANA_BATTERY_SHUTDOWN_ENTER_DEEPSLEEP = 10,
    ANA_D24G_OTA_ENABLE_REBOOT = 11,
} ana_status_e;




#define MAX_BUTTON_COUNT 6
typedef struct
{
    u8 cnt;
    u8 keycode[MAX_BUTTON_COUNT];
    u16 special_key_press_f;
    u8 press_cnt;
} kb_data_t_aaa;
 extern kb_data_t_aaa key_buf_aaa;

typedef enum
{
    MULTIPIPE_1 = 0,
    MULTIPIPE_1_DOT_5 = 1,
    MULTIPIPE_2_DOT_0 = 2,
    MULTIPIPE_2_DOT_5 = 3,
    MULTIPIPE_3_DOT_0 = 4,
} SENSOR_MULTIPIPE_XY;


#define CONSUMER_BUF_SIZE	4


#define TX_FIFO_NUM   48
#define TX_BUF_SIZE   10

typedef struct
{
    u8 fifo[TX_FIFO_NUM][TX_BUF_SIZE];
    u8 wptr;
    u8 rptr;
	u16 count;

} tx_fifo_t;
 extern tx_fifo_t tx_fifo_aaa;

typedef struct
{
    u8 type;
    u8 len;
    u8 buf[TX_BUF_SIZE - 2];
} tx_packet_t;

typedef struct
{
    u32 dongle_id;
} flash_dev_info_t;
extern flash_dev_info_t flash_dev_info;

extern int		device_sync;

extern u8 deep_flag;
extern u8 pair_flag;
extern u8 ana_reg1_aaa;
extern u8 has_been_paired_flag;


extern u8 d24g_power_on_Pair_flag;

extern u8 suspend_wake_up_enable;

extern u32 row_pins[];
extern u32 col_pins[];

extern u32 btn_tick;
extern u32 power_on_tick;
extern u32 start_tick;

extern u8 active_disconnect_reason;

extern u8 encKey[];
extern u8 my_batVal[1] ;


extern u32 advertise_begin_tick;
extern int binding_master_addr_idx;
extern u8 binding_master_addr[];

extern u8 g_user_payload[sizeof(rf_cmd_frame_t)];

extern u8 has_new_report_aaa;

extern u32 adv_begin_tick;
extern u16 adv_count;

extern u32 idle_tick;
extern u16 idle_count;
//extern u32 loop_cnt;

extern u16 adv_timer_count;
extern u32 adv_led_interval;

extern u8  device_status;
extern u32 wakeup_next_tick;

extern u16 alarm_vol_mv;
extern u16 min_vol_mv;

extern u8 dongle_id_valid_f;



extern u8 xy_multiple_flag;
extern int dev_info_idx;

extern u16 led_pin;

extern u32 power_on_tick;

// Application-layer TX pending flag / channel counter
extern u8 tx_cmd_pending;
extern u8 tx_chn_left;

// Pairing ACK parameters returned by the Car (dat[0]/dat[1] semantic aliases, customer extension)
extern u8 pair_ack_para1;
extern u8 pair_ack_para2;

/**
 * @brief       This function check pair,emi key and set device status to normal
 * @return      
 * @note        
 */
extern void d24_poweron_check_key_handle();

extern void debug_loop_toggle();

extern void user_init_normal();
extern void user_init_deepRetn();

extern void main_loop(void);

/**
 * @brief       This function save dev info to flash
 * @return      
 * @note        
 */
extern bool save_dev_info_flash();
extern int cccWrite(void *p);
extern int cccWrite1(void *p);

u8 push_data_fifo_aaa(u8 *buf, u8 data_type, u8 len);


extern void ble_pair_process_aaa();

/**
 * @brief       This function clear pair flag
 * @return      
 * @note        
 */
extern void clear_fifo();

/**
 * @brief       This function use for reboot chip
 * @param[in]   reason	- reboot reason
 * @return      
 * @note        
 */
extern void user_reboot(u8 reason);

/**
 * @brief       This function set pair and reboot
 * @return      
 * @note        
 */
extern void d24_start_pair();

/**
 * @brief       This function reset idle status parameter
 * @return      
 * @note        
 */
extern void reset_idle_status();

/**
 * @brief       This function poll idle parameters
 * @return      
 * @note        
 */
extern void idle_status_poll();

/**
 * @brief       This function reset idle status parameter
 * @return      
 * @note        
 */
extern void reset_adv_status();

/**
 * @brief       This function poll adv parameters
 * @return      
 * @note        
 */
extern void adv_count_poll();

/**
 * @brief       This function get wheel value
 * @param[in]   wheel_prepare_tick	- 
 * @return      
 * @note        
 */
extern u8 wheel_get_value(u32 wheel_prepare_tick);
extern void sensor_gpio_powerDownConfig();
extern void	rf_power_enable(int en);
extern u8 adaptive_smoother();
extern void battery_check();
extern void battery_hw_init(void);
extern void ble_main_loop_aaa();
extern void ble_user_init_aaa();

extern void ble_notify_data_proc_aaa();




/**
 * @brief       This function set sensor motion gpio as wakeup gpio
 * @param[in]   enable	- 
 * @return      
 * @note        
 */
extern void sensor_set_wakeup_level_suspend(u8 enable);

/**
 * @brief       This function set sensor motion gpio as deepsleep wakeup gpio
 * @param[in]   enable	- 
 * @return      
 * @note        
 */
extern void sensor_set_wakeup_level_deepsleep(u8 enable);
extern void check_rf_fast_setting_time();
extern void check_rf_fast_setting_flag();


/**
 * @brief       This function set wheel gpios as wake up gpios
 * @param[in]   enable	- en
 * @return      
 * @note        
 */
extern void wheel_set_wakeup_level_suspend(u8 enable);

extern void change_mode_aaa(u8 event_new);
extern void check_sensor_dircet(u8 sensor_dir);

extern void d24_tx_main_loop();
extern void d24_user_init();

/**
 * @brief       This function let chip enter deep sleep 
 * @param[in]   reason	- the reason enter deep sleep
 * @return      
 * @note        
 */
extern void enter_deep(ana_status_e reason);

extern void deep_wakeup_proc_aaa();
extern int device_send_packet(u8 *p, int retry, u16 rx_waittime);




/**
 * @brief       This function load info from flash
 * @param[in]   d_addr	- the buff store readed data
 * @param[in]   len	- data len need read
 * @param[in]   s_addr	- the flash addr need read
 * @return      
 * @note        
 */
extern int flash_info_load_aaa(u32 s_addr, u8 *d_addr,  u8 len);
extern void get_24g_data_report_aaa();

extern unsigned int OPTSensor_motion_report(u32 no_overflow);
extern void OPTSensor_Shutdown(void);
extern u8 Draw_a_square_test();


extern void delet_soft_time();
extern void ui_loop();
extern void add_soft_time();
extern void check_softe_time();

/**
 * @brief       This function check pair key
 * @param[in]   check_ms : the pair at least last time	- 
 * @return      
 * @note        
 */

extern void switch_type_poll();

extern void user_batt_check_init();

/**
 * @brief       This function pro battery check
 * @return      
 * @note        
 */
extern void user_batt_check_proc();

extern void aes_ll_decryption(u8 *key, u8 *plaintext, u8 *result);
extern void swapX(const u8 *src, u8 *dst, int len);
extern void aes_user_encryption(u8 *key, u8 *plaintext, u8 *encrypted_data);
extern void aes_user_decryption(u8 *key, u8 *encrypted_data, u8 *decrypted_data);
extern void aes_user_set_key();

extern void  double_click_left_24G_mode();

extern int speckWrite(void *p);

/**
 * @brief       This function use for reboot chip
 * @param[in]   reason	- reboot reason
 * @return      
 * @note        
 */
extern void write_deep_ana0(u8 buf);

extern void irq_handle_usb_timer();
extern void usb_user_init();
extern void usb_update_init();
extern void usb_mode_loop();
extern void usb_update_loop();
extern void usb_init_process();
extern void usb_loop_process();
extern unsigned short crc16_user (unsigned char *pD, int len);
extern void usb_resume_host();
extern void usb_init_interrupt(void);
extern void usb_handle_irq(void) ;
extern void rf_stx_to_rx(u8 *p,u32 rx_timeout_us);

extern void resume_data_fifo_aaa(u8 cnt,u8 rptr);

extern u8 get_usb_data_report_aaa();
void blc_smp_set_simple_multi_mac_en(u8 en);
void notify_rsp_add2buf(u8 *p);

u32 blc_smp_param_getCurStartAddr();

extern u8 run_app_code();
extern void gpio_set_wakeup_enable(u8 enable);

// Runtime configuration globals (initialized from app_config.h defaults).
// Previously lived in app_toy/cfg_layer/cfg_data.c.
extern u16 g_remote_shutdown_voltage_mv;
extern u16 g_remote_low_battery_voltage_mv;
extern remote_sleep_timeout_mode_t g_remote_sleep_mode;
extern u16 g_remote_sleep_timeout_s;
extern u8  g_remote_access_code_group;
extern u8  g_remote_normal_tx_power;

/**
 * @brief  休眠准入钩子（weak 默认实现：允许休眠）。
 *         系统每次进入休眠（浅睡眠/深睡眠）前都会调用本函数确认；
 *         客户在自己的应用文件中定义同名强函数覆盖即可：
 *             u8 allow_sleep(void) {
 *                 return (客户应用已跑完) ? 1 : 0;
 *             }
 * @return  1 = 允许进入休眠，0 = 阻止进入休眠（本轮不睡，下一轮主循环重试）
 * @note    默认返回 1，不覆盖则保持 SDK 原有休眠行为。
 */
u8 allow_sleep(void);

#endif

