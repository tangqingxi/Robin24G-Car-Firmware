/********************************************************************************************************
 * @file     main.c
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
#include "calibration.h"
#include "app_battery_check.h"
#include "lib/include/pm/pm.h"
#include "toy_log.h"
#include "app_24g_rx.h"
#include "app_toy/app_layer/app_toy.h"
#include "app_toy/app_layer/app_emi.h"
#include "common.h"
#include	"UserRxdFunc.h"
#include "UserSpeechFunc.h"
#include "bootloader_ota/boot_copy_data.h"
#include "ota_config.h"
#include "sdk_version.h"
#include "flash_fw_check.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserPwm.h"
#include <UserUart.h>

extern unsigned char sd_adc_rx_done_flag;

//volatile unsigned char sample_cnt=0;
//*************************************************************************
u32 timer0_irq_cnt;
u32 timer1_freq=0;
volatile u8 g_pwmen= 0,g_pwmduty = 80;

/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	//gpio_write(Testing_Pin, 1);
	irq_handle_private_1m();	//RF interrupt handler function, involving RF state changes
	//gpio_write(Testing_Pin, 0);

#if BATT_CHECK_ENABLE
	#if (ADC_MODE == ADC_DMA_MODE)
		if (dma_chn_irq_status_get(FLD_DMA_CHN_SAR_ADC))
		{
			//sample_cnt++;
			sd_adc_rx_done_flag = 1;
			adc_stop_sample_dma();
			dma_chn_irq_status_clr(FLD_DMA_CHN_SAR_ADC);
		}
	#endif
#endif

		if(timer_get_interrupt_status(TMR_STA_TMR1))
		{
			timer_clear_interrupt_status(TMR_STA_TMR1);
			timer1_freq++;
			//Timer1_pwm_set(g_pwmen,g_pwmduty);
			T2000uSInterrupt();

		}

#if(UseSpeechCmd)
#if(UseTm0SoundCmd)
	SpeechCmd_IrqHandler();		//500uS定时器中断服务程序中调用
#endif		//#if(UseTm0SoundCmd)

	//Pwm0IrqHandler();
#endif	//#if(UseSpeechCmd)

	Uart_irq_handler();
}




/**
 * @brief       This function config user need
 * @return      
 * @note        
 */
void user_config()
{
	// efuse 读取（toy_type、did）已移至 custom_init()
}

void main_loop_process()
{
	/*car will rcv cmd and send ack*/
	d24_rx_main_loop();
	car_loop();

	#if (BATT_CHECK_ENABLE)
		user_batt_check_proc();//batt check proc
	#endif
}

void toy_user_init()
{
	TOY_LOG("---init car---\n");
	TOY_LOG("---deep_flag:%d\n", deep_flag);
	TOY_LOG("---fw_type:%d\n", efuse_dev_info.toy_type);

	//toy_init();
	User_hw_init();

	//timer0_set_mode(TIMER_MODE_SYSCLK,0,500 * CLOCK_SYS_CLOCK_1US);
	timer1_set_mode(TIMER_MODE_SYSCLK, 0, 2000 * CLOCK_SYS_CLOCK_1US);
    //timer_start(TIMER0); // 由send_data控制启动
    irq_enable();

	TOY_LOG("---init car done---\n");
}

#if (BOOTLOADER_ENABLE)
volatile int for_keep_ota_symbols = 0;
volatile int ota_use_api(void)
{
	if(for_keep_ota_symbols){
		boot_copy_data(0);
	}
}
#endif

//Adding EMI API to flash.bin increases RAM from 0x1EB3 to 0x220F, causing insufficient RAM.
#if (!FLASH_ENABLE)
volatile int for_keep_symbols = 0;
volatile int emi_use_api(void)
{
	if(for_keep_symbols){
		cpu_wakeup_init(0);
		clock_init(SYS_CLK_24M_Crystal);
		user_read_efuse_value_calib();
		rf_mode_init();
		rf_set_ble_1M_mode();
		rf_set_ble_1M_NO_PN_mode();
		rf_set_ble_2M_mode();
		rf_set_ble_2M_NO_PN_mode();
		rf_set_pri_250K_mode();
		rf_set_pri_500K_mode();
		rf_set_pri_1M_mode();
		rf_set_pri_2M_mode();
		rf_set_channel_singletone(0);
		rf_set_power_level_index_singletone(0);
		rf_access_code_comm(0);

		rf_emi_single_tone(0,0);
		rf_emi_stop();
		rf_emi_tx_continue_setup(0,0,0,0);
		rf_continue_mode_run();

		rf_emi_rx(0,0);
		rf_emi_rx_loop();
		rf_emi_get_rxpkt_cnt();

		rf_emi_tx_burst_setup(0,0,0,0);
		rf_emi_tx_burst_loop(0,0);
		gpio_set_func(0,0);
		gpio_set_output_en(0,0);
		gpio_set_input_en(0,0);
		gpio_write(0,0);
		gpio_read(0);
		gpio_setup_up_down_resistor(0, 0);
	}

	return 1;
}
#endif

/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */

int main (void)   //must on ramcode
{
	PLATFORM_INIT;

    CLOCK_INIT;

	//gpio_shutdown(GPIO_ALL);

	if (deep_flag == ANA_POWER_ON)
	{
		#if MODULE_32K_WATCHDOG_ENABLE
		//Remove the stop 32k watchdog operation in main, otherwise this state cannot be read.
		if (wd_32k_get_status())
		{
			wd_32k_clear_status();
			if (wd_32k_get_status())
			{
				TOY_LOG("32k watchdog error\n");
				start_reboot();
			}
		}
		wd_32k_set_interval_ms(WATCHDOG_INIT_TIMEOUT + 2048);
        #endif

	#if (BOOTLOADER_ENABLE)
		ota_use_api();
	#endif

	#if (!FLASH_ENABLE)
		emi_use_api();
	#endif

		TOY_LOG("Robin_24G_Car FW_VERSION = 0x%02x, PID = 0x%08x\r\n", FW_VERSION, PID);

		user_config();
		custom_init();//custom init

#if (EMI_TEST_FUN_ENABLE)
		emi_poweron_check();//PA0+PB0 pressed at power-on -> enter EMI (never return)
#endif
	}

#if (BATT_CHECK_ENABLE)
	batt_adc_init();//batt adc init
#endif

	toy_user_init();//toy init

	rf_drv_private_1m_init();//rf init

	d24_user_init();//user init

	adv_begin_tick = clock_time()|1;//update adv begin tick
	idle_tick = clock_time()|1;//update idle tick
	
#if(DEBUG_MODE==1)
#if (DEBUG_BUS==DEBUG_IO)
	io_putchar_reinit();
#endif
#endif

	UserPowOnStart();
	timer_start(TIMER1);
	while(1)
	{
		//main_loop_process();
		User_main_loop_process();
	}
}
