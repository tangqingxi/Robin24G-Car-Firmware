/********************************************************************************************************
 * @file     app_emi.c
 *
 * @brief    EMI 测试模块（非正式代码）：PA0+PB0 上电同时按下进入 EMI service，
 *           EMI 中 PA0 短按循环切换测试状态
 *
 * @author   lingzhi.chen
 * @date         08,2026
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
#include "app_emi.h"

#if (EMI_TEST_FUN_ENABLE)

#include "app_emi.h"
#include "lib/include/emi.h"
#include "watchdog.h"
#include "toy_log.h"

#define EMI_KEY_ENTER_PIN1      GPIO_PA0   // 进入 EMI：PA0（与 PB0 同时按下）
#define EMI_KEY_ENTER_PIN2      GPIO_PB0   // 进入 EMI：PB0（与 PA0 同时按下）
#define EMI_KEY_SWITCH_PIN      GPIO_PA0   // EMI 中：PA0 短按切换模式

#define EMI_RF_MODE             RF_MODE_PRIVATE_1M
#define EMI_TX_POWER            RF_POWER_P0p00dBm   //0dBm

#define EMI_PKT_TYPE_PRBS9      0
#define EMI_PKT_TYPE_0XF0       1
#define EMI_PKT_TYPE_0X55       2

#define EMI_PKT_TYPE            EMI_PKT_TYPE_PRBS9

#define EMI_CHN_NUM             3
//2405MHz, 2440MHz, 2478MHz
static const signed char emi_tbl_chn[EMI_CHN_NUM] = {2, 40, 78};

enum {
	EMI_MODE_CARRIER = 0,   //single tone(carrier only)
	EMI_MODE_MODULATED,     //modulated wave(tx continue)
	EMI_MODE_RX,            //rx mode
	EMI_MODE_NUM,
};

//9 states: carrier ch5/40/78, modulated ch5/40/78, rx ch5/40/78
#define EMI_STATE_NUM   (EMI_MODE_NUM * EMI_CHN_NUM)

//current emi state: 0~(EMI_STATE_NUM-1)
static u8 emi_state = 0;

/**
 * @brief       This function init emi key gpio(PA0/PB0 input pull-up, active low)
 */
static void emi_key_gpio_init(void)
{
	gpio_set_func(EMI_KEY_ENTER_PIN1, AS_GPIO);
	gpio_set_output_en(EMI_KEY_ENTER_PIN1, 0);
	gpio_set_input_en(EMI_KEY_ENTER_PIN1, 1);
	gpio_setup_up_down_resistor(EMI_KEY_ENTER_PIN1, PM_PIN_PULLUP_10K);

	gpio_set_func(EMI_KEY_ENTER_PIN2, AS_GPIO);
	gpio_set_output_en(EMI_KEY_ENTER_PIN2, 0);
	gpio_set_input_en(EMI_KEY_ENTER_PIN2, 1);
	gpio_setup_up_down_resistor(EMI_KEY_ENTER_PIN2, PM_PIN_PULLUP_10K);
}

/**
 * @brief       This function run current emi test state
 */
static void emi_state_run(void)
{
	u8 mode = emi_state / EMI_CHN_NUM;

	if (mode == EMI_MODE_MODULATED)
	{
		rf_continue_mode_run();
	}
	else if (mode == EMI_MODE_RX)
	{
		rf_emi_rx_loop();
	}
}

/**
 * @brief       This function print emi state log,
 *              call it only when rf is stopped(print is time-consuming)
 */
static void emi_state_log(void)
{
	u8 mode = emi_state / EMI_CHN_NUM;
	signed char rf_chn = emi_tbl_chn[emi_state % EMI_CHN_NUM];

	TOY_LOG("---EMI state=%d.mode=%d.chn=%d.\n", emi_state + 1, mode, rf_chn);
}

/**
 * @brief       This function enter current emi test state
 */
static void emi_state_enter(void)
{
	u8 mode = emi_state / EMI_CHN_NUM;
	signed char rf_chn = emi_tbl_chn[emi_state % EMI_CHN_NUM];

	if (mode == EMI_MODE_CARRIER)
	{
		rf_emi_single_tone(EMI_TX_POWER, rf_chn);
	}
	else if (mode == EMI_MODE_MODULATED)
	{
		rf_set_tx_rx_off_auto_mode();
		rf_emi_tx_continue_setup(EMI_RF_MODE, EMI_TX_POWER, rf_chn, EMI_PKT_TYPE);
	}
	else //EMI_MODE_RX
	{
		rf_emi_rx(EMI_RF_MODE, rf_chn);
	}
}

/**
 * @brief       This function check switch key(PA0) status,
 *              keep running emi state while checking
 * @return      1: key pressed, 0: key released
 */
_attribute_ram_code_sec_noinline_ u8 emi_switch_key_is_pressed(void)
{
	return !gpio_read(EMI_KEY_SWITCH_PIN);
}

/**
 * @brief       This function check switch key(PA0) press with debounce,
 *              keep running emi state while checking
 * @return      1: short press detected, 0: no short press
 */
static u8 emi_switch_key_scan(void)
{
	u32 tick;

	if (!emi_switch_key_is_pressed())
	{
		return 0;
	}

	//debounce 20ms, keep running emi state
	tick = clock_time();
	while (!clock_time_exceed(tick, 20000))
	{
		emi_state_run();

		if (!emi_switch_key_is_pressed())
		{
			return 0;
		}
	}

	//wait key release, keep running emi state
	while (emi_switch_key_is_pressed())
	{
		emi_state_run();
	}

	return 1;
}

/**
 * @brief       EMI test service loop, never return.
 *              PA0 short press switches the 9 states cyclically.
 */
void emi_serviceloop(void)
{
	emi_state = 0;

	TOY_LOG("---emi_serviceloop.\n");

#if (MODULE_32K_WATCHDOG_ENABLE)
	wd_32k_stop();
#endif

	irq_disable();

	rf_mode_init();
	rf_set_pri_1M_mode();//set pri 1m mode, same as normal work mode
	rf_access_code_comm(0x29417671);
	rf_set_tx_rx_off_auto_mode();

	//PA0/PB0 are both pressed when enter emi, wait for key release first
	while (!gpio_read(EMI_KEY_ENTER_PIN1) || !gpio_read(EMI_KEY_ENTER_PIN2))
	{
	}

	TOY_LOG("---EMI key release.\n");

	emi_state_log();//rf not start yet, print is safe
	emi_state_enter();

	while (1)
	{
		emi_state_run();

		if (emi_switch_key_scan())
		{
			rf_emi_stop();//stop rf first
			emi_state = (emi_state + 1) % EMI_STATE_NUM;//press 10th time back to first state
			emi_state_log();//print log while rf is stopped
			emi_state_enter();//restart rf with new state
		}
	}
}

/**
 * @brief       Power-on EMI check: enter EMI service when PA0 and PB0 are
 *              pressed together at power on. Must be called in the main
 *              power-on flow (never returns if entered).
 */
void emi_poweron_check(void)
{
	emi_key_gpio_init();

	sleep_ms(2);//wait pull-up stable

	TOY_LOG("---emi_poweron_check: PA0=%d PB0=%d.\n",
		gpio_read(EMI_KEY_ENTER_PIN1), gpio_read(EMI_KEY_ENTER_PIN2));

	if (!gpio_read(EMI_KEY_ENTER_PIN1) && !gpio_read(EMI_KEY_ENTER_PIN2))
	{
		TOY_LOG("---enter emi: PA0+PB0 pressed.\n");
		emi_serviceloop();//never return
	}
}

#endif /* EMI_TEST_FUN_ENABLE */
