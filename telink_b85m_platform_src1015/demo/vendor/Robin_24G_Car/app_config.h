/********************************************************************************************************
 * @file     AAA_app_config_Telink_QFN32.h
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

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

// Firmware type enumeration (previously in cfg_layer/cfg_common.h)
typedef enum {
    FW_TYPE_DEFAULT     = 0b000,  // Default value
    FW_TYPE_S10_REMOTE  = 0b001,  // SOP10 remote
    FW_TYPE_S10_CAR     = 0b010,  // SOP10 car
    FW_TYPE_S16_CAR     = 0b011,  // SOP16 car
} firmware_type_t;


#define Telink_ROBIN_24G_PRJ            0

#define PRJ_NAME   Telink_ROBIN_24G_PRJ

#define	 PKT_BUFF_SIZE						48

// =================================================
// 2.4G OTA 升级使能开关（0=关闭 OTA 代码，1=开启）
// 可通过编译参数 -DD24G_OTA_ENABLE=0 覆盖
// =================================================
#ifndef D24G_OTA_ENABLE
#define D24G_OTA_ENABLE      0
#endif

#define USER_DATA_CMD      EMPTY_CMD
#define USER_DATA_ACK_CMD  EMPTY_ACK_CMD
#define SAVE_DONGLE_ID_TO_PROGRAM_ENABLE		0 //default

#define AES_LEN        16
#define ID_LEN         4


#define DEFAULT_FW_TYPE       		FW_TYPE_S10_CAR

// =================================================
// efuse storage address for chip_id (6 bytes, DID source) + toy_type (byte 7).
// chip_id (DID) and toy_type are always read from efuse on boot.
// =================================================
#define EFUSE_CHIP_ID_ADDR    0x00

// =================================================
// Car RF configuration parameters (defaults, previously read from efuse/flash).
// =================================================
#define CAR_ACCESS_CODE_GROUP 1       // 1-15		//普通遥控车0，漂移车1
#define CAR_NORMAL_TX_POWER   0       // 0=default(10dBm), else maps to (1 + val*3)

// =================================================
// Car battery-check voltage thresholds (defaults).
// 0 或 0xFFFF 表示不启用，可独立配置（BATT_CHECK_ENABLE 控制开关）
// =================================================
#define CAR_SHUTDOWN_VOLTAGE_MV     0xFFFF  // 低电关机电压（mV）
#define CAR_LOW_BATTERY_VOLTAGE_MV  0xFFFF  // 低电提醒电压（mV）

// =================================================
// Build-time default mode selection (for test bins)
// =================================================
// Override with compiler flags, e.g.:
//   -DTOY_S10_CAR_DEFAULT_MODE=TOY_S10_CAR_MODE_2MOTOR_SPRAY
#define TOY_S10_CAR_MODE_DIFF_4PWM_LED  	1 // 4-channel PWM differential drive + LED (GPIO)
#define TOY_S10_CAR_MODE_2MOTOR_SPRAY  		2 // 2-channel PWM forward/backward + 2-channel GPIO left/right + spray (PWM)
#define TOY_S10_CAR_MODE_2MOTOR_LED     	3 // 2-channel PWM forward/backward + 2-channel GPIO left/right + LED (GPIO)

#define TOY_S10_CAR_DEFAULT_MODE        TOY_S10_CAR_MODE_DIFF_4PWM_LED

// =================================================
// Build-time default mode selection (for test bins)
// =================================================
// Override with compiler flags, e.g.:
//   -DTOY_S16_CAR_DEFAULT_MODE=TOY_S16_CAR_MODE_DIFF_SPRAY
#define TOY_S16_CAR_MODE_DIFF_SPRAY      	0 // Differential chassis + spray (W1..W4 selectable) + LED + charging
#define TOY_S16_CAR_MODE_2MOTOR_EXCAVATOR 	1 // Dual-motor chassis + turntable/arm (W1..W4) + LED + charging
#define TOY_S16_CAR_MODE_2MOTOR_SPRAY    	2 // Dual-motor chassis + spray (W1..W4 selectable) + LED + charging

#define TOY_S16_CAR_DEFAULT_MODE         TOY_S16_CAR_MODE_DIFF_SPRAY

//  The SOP10 MCU's SWS GPIO is occupied. Switching 
// to the SOP16 MCU provides more GPIOs, allowing
// for debugging using the SWS and Log GPIOs.
#define BOARD_CAR_MCU_S16         			0

#define TOY_LOG_ENABLE            			0

// =================================================
// Pairing ACK parameters returned to the Remote (customer extension)
// =================================================
#define Product_VID		0x261009					//四轴飞行器遥控器 VID (与 HspdCar 0x2432 区分)
#define CAR_PAIR_ACK_PARA1		(unsigned char)Product_VID   		// 配对 ACK para1，客户可扩展（如固件版本）
#define CAR_PAIR_ACK_PARA2      (unsigned char)(Product_VID>>8)  	// 配对 ACK para2，客户可扩展

// =================================================

#if (FLASH_ENABLE == 1)
#define HW_IS_FLASH     					1 //Flash
#define CFG_PARAM_SAVE_STORAGE_ADDR   		0xF010
#define CFG_2G4_PARAM_SAVE_STORAGE_ADDR   	0xF000
#define CFG_DEFAULT 						0xFF //start save user config 16byte

#define FLASH_BIN_ON                        1
#else
#define HW_IS_FLASH     					0 //efuse
#define CFG_PARAM_SAVE_STORAGE_ADDR  		0x10
#define CFG_2G4_PARAM_SAVE_STORAGE_ADDR   	0x00
#define CFG_DEFAULT 						0x00 //start save user config 16byte

#define FLASH_BIN_ON                        0
#endif



//---------------------------watchdog References-----------------------------------
#define MODULE_32K_WATCHDOG_ENABLE			1
#define WATCHDOG_INIT_TIMEOUT				1500  //ms


#if (PRJ_NAME==Telink_ROBIN_24G_PRJ)
	#define ENTER_SHUWDOWN_WHEN_POWER_IS_CUT_OFF_ENABLE		1 //0:when shutdown battery allow wakeup,1:when shutdown battery disallow wakeup
	//Hardware debugging, The production version needs to be closed
    #define ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG		0

	// Packet loss test with retransmission
	#define PACKET_LOSS_TEST_ENABLE     0

	#define DEEP_SLEEP_ENABLE			0 //default

	/* ============ Car 深睡眠（deepsleep）时长配置 ============
	 * 运行期通过修改 retention 全局变量 g_car_sleep_mode 选择休眠策略：
	 *   CAR_SLEEP_TIMEOUT_NEVER    永不休眠（适合车载供电场景）
	 *   CAR_SLEEP_TIMEOUT_5MIN     遥控器 5 分钟无信号后进入深睡眠
	 *   CAR_SLEEP_TIMEOUT_10MIN    遥控器 10 分钟无信号后进入深睡眠
	 *   CAR_SLEEP_TIMEOUT_CUSTOM   自定义秒数（CAR_SLEEP_TIMEOUT_S，
	 *                              可再通过 g_car_sleep_timeout_s 运行时修改）
	 */
	typedef enum {
		CAR_SLEEP_TIMEOUT_NEVER   = 0,   // 永不休眠
		CAR_SLEEP_TIMEOUT_5MIN    = 1,   // 5 分钟无信号后休眠
		CAR_SLEEP_TIMEOUT_10MIN   = 2,   // 10 分钟无信号后休眠
		CAR_SLEEP_TIMEOUT_CUSTOM  = 3,   // 自定义秒数（CAR_SLEEP_TIMEOUT_S）
	} car_sleep_timeout_mode_t;

	#define CAR_SLEEP_MODE              CAR_SLEEP_TIMEOUT_5MIN  // 默认休眠模式：5min
	#define CAR_SLEEP_TIMEOUT_5MIN_S    300                      // 5 分钟（单位：秒）
	#define CAR_SLEEP_TIMEOUT_10MIN_S   600                      // 10 分钟（单位：秒）
	#define CAR_SLEEP_TIMEOUT_S         300                      // 自定义模式休眠超时（单位：秒），0 = 永不休眠

    #define D24G_ADV_DEEPSLEEP_TIMEOUT_ENABLE     ((!ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG)&&1)

	//Broadcast (paired mode) duration time
    #define D24G_ADV_TIMER_OUT  	60	//unit 1s

	//Connection (normal mode) duration time
    #define D24G_CONNECT_TIME_OUT  	600	//unit 1s

	//#define D24G_PAIR_TIMER_OUT  			380  // for 2Mbps baseband //unit 1us
	#define D24G_PAIR_TIMER_OUT  			760 // for 1Mbps baseband //unit 1us
	//#define D24G_PAIR_TIMER_OUT  			1800 // for 250kbps baseband //unit 1us
	//#define D24G_COMMUNICATION_TIMER_OUT  	280 // for 2Mbps baseband //unit 1us
	#define D24G_COMMUNICATION_TIMER_OUT  	560 // for 1Mbps baseband //unit 1us
	//#define D24G_COMMUNICATION_TIMER_OUT  	880 // for 250kbps baseband //unit 1us

	/* ============ 通讯模式 ============
	 * 0: 远距离模式 - 配对后 remote 单发、car 单收（默认，控制距离尽可能远）
	 * 1: 双向通讯   - 配对后 remote 发命令并收 ACK，car 收命令并回 ACK
	 *                 （remote 可查看小车状态）
	 */
	#define BIDIR_COMM_ENABLE   0

	#define KEY_PRESS_HOLD_CHECK_TIME			1000000  //unit 1us

	// TX transmission timing configuration
	#define TX_HEARTBEAT_INTERVAL_US			20000    // 20ms heartbeat interval in normal mode
	#define TX_KEEPALIVE_INTERVAL_US			8000     // 8ms keepalive interval
	#define MAX_NO_ACK_BEFORE_CHANNEL_SWITCH	3        // Switch channel after 3 no-ACK packets
	#define MAX_NO_ACK_BEFORE_DISCONNECT		1250     // Disconnect threshold

    #if PM_DEEPSLEEP_RETENTION_ENABLE
        #define _attribute_data_retention_user    _attribute_data_retention_
    #else
        #define _attribute_data_retention_user
    #endif

    //-----------------------product function referance---------------------------------------

    // Battery check enable (0/1), customer can configure thresholds in
    // CAR_SHUTDOWN_VOLTAGE_MV / CAR_LOW_BATTERY_VOLTAGE_MV.
    #define BATT_CHECK_ENABLE		0

	#define PIN_BTN_MAIN        		GPIO_PA0
	#define BUTTON_FUN_ENABLE   	((!ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG)&&0)
    #define BLT_APP_LED_ENABLE   	((!ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG)&&1)
		
	//#if (BATT_CHECK_ENABLE)
							//telink device: you must choose one gpio with adc function to output high level(voltage will equal to vbat), then use adc to measure high level voltage
							//use PB7 output high level, then adc measure this high level voltage
		#define ADC_DMA_MODE				1
		#define ADC_NDMA_MODE        		2

		#define ADC_GPIO_SAMPLE      		1
		#define ADC_VBAT_SAMPLE      		2

		//In NDMA mode, only M channel can be used.
		#define NDMA_M_1_CHN_EN 			1
		#define DMA_M_1_CHN_EN  			1

		#define ADC_SAMPLE_GROUP_CNT        16 //Number of adc sample codes per channel.

		#define ADC_MODE                    ADC_NDMA_MODE

		#if (ADC_MODE == ADC_DMA_MODE)
			#define ADC_SAMPLE_CHN_CNT      DMA_M_1_CHN_EN //Number of channels enabled
			#define ADC_M_CHN_SAMPLE_MODE   ADC_VBAT_SAMPLE
		#else
			#define ADC_SAMPLE_CHN_CNT      NDMA_M_1_CHN_EN
			#define ADC_SAMPLE_MODE         ADC_VBAT_SAMPLE
		#endif

		/**
		 * @attention:
		 * For TC321X:
		 * -# The maximum sampling voltage range of PB5-6 and PD0-1 is 0 ~ (vbat*60%).
		 *    e.g., if vbat is supplied with 3.3V, then the sampling range is 0 ~ 1.98V).
		 * -# If external voltage division is required, we recommend using PB5-6 and PD0-1,
		 *    which can prevent the influence of internal resistance and provide more accurate sampling voltage.
		 * -# The maximum sampling voltage range of PB0-4 is 0 ~ 3.3V.
		 */
		#define GPIO_M_CHN_SAMPLE_PIN       ADC_GPIO_PB1


		//#endif
	#endif

//#define DEBUG_GPIO_ENABLE		((ADAPT_ALL_HW_AUTO_DRAW_IN_D24G_MODE_DEBUG)&&1)
#define DEBUG_GPIO_ENABLE       0

#define DEBUG_GPIO_1  GPIO_PA0	
#define DEBUG_GPIO_2  GPIO_PA1
#define DEBUG_GPIO_3  GPIO_PA6	
#define DEBUG_GPIO_4  GPIO_PA7	
#define DEBUG_GPIO_5  GPIO_PB0	
#define DEBUG_GPIO_6  GPIO_PB1
#define DEBUG_GPIO_7  GPIO_PB2

#if DEBUG_GPIO_ENABLE
	#define PIN_DEBUG_RF_TX_LEVEL(x) gpio_write(GPIO_PA0,x)
	#define PIN_DEBUG_RF_RX_LEVEL(x) gpio_write(GPIO_PA1,x)
	#define PIN_DEBUG_RF_RX_TIMEOUT_LEVEL(x) gpio_write(GPIO_PA6,x)
	#define PIN_DEBUG_RF_CHN_NEXT_TOGGLE gpio_toggle(GPIO_PA7)
	#define PIN_DEBUG_RF_STATE_CHANGE_TOGGLE gpio_toggle(GPIO_PB0)
#else
	#define PIN_DEBUG_RF_TX_LEVEL(x) //0
	#define PIN_DEBUG_RF_RX_LEVEL(x) //0
	#define PIN_DEBUG_RF_RX_TIMEOUT_LEVEL(x) //0
	#define PIN_DEBUG_RF_CHN_NEXT_TOGGLE //0
	#define PIN_DEBUG_RF_STATE_CHANGE_TOGGLE //0
#endif //end gpio debug
	
#endif
