/********************************************************************************************************
 * @file     app_toy.c
 *
 * @brief    This is the source file for KMD SDK
 *
 * @author   lingzhi.chen
 * @date         04,2026
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

#include "app_toy.h"
#include "app_toy/app_layer/app_test.h"
#include "app_toy/app_layer/toy_log.h"
#include "app_24g_rf_common.h"
#include "random.h"

#if (BIDIR_COMM_ENABLE)
// Car status aggregated from data ACKs (bidirectional-comm mode), for UI/display.
_attribute_data_retention_ u8 car_ack_para1 = 0;  // car status 1 (e.g. battery flag) - dat[0] semantic alias
_attribute_data_retention_ u8 car_ack_para2 = 0;  // car status 2 (reserved) - dat[1] semantic alias

/**
 * @brief   Car ACK handling entry, called from RF RX IRQ (bidirectional-comm mode).
 *          Aggregates car status and drives remote peripherals.
 * @param[in] ack    - data ACK frame from Car (CMD_ACK, dat[0]/dat[1] = status)
 * @param[in] seq_no - RF sequence number
 * @note    Extend here for future needs (status aggregation, display/vibration
 *          control, etc.).
 */
void toy_remote_ack_on_rf_km_dat(const rf_cmd_ack_frame_t *ack, u8 seq_no)
{
	(void)seq_no;

	// Aggregate car status from the ACK payload (dat 长度由 RF_CMD_DATA_ACK_DAT_LEN 控制).
#if (RF_CMD_DATA_ACK_DAT_LEN >= 1)
	car_ack_para1 = ack->dat[0];
#else
	car_ack_para1 = 0;
#endif
#if (RF_CMD_DATA_ACK_DAT_LEN >= 2)
	car_ack_para2 = ack->dat[1];
#else
	car_ack_para2 = 0;
#endif

	// 旧格式：key==0 = keepalive 应答，非 0 = 按键应答
	if (ack->key == 0) {
		TOY_LOG("---car ack: KEEPALIVE dat[0]=%d dat[1]=%d\n", car_ack_para1, car_ack_para2);
	} else {
		TOY_LOG("---car ack: KEY key=0x%02x dat[0]=%d dat[1]=%d\n",
			ack->key, car_ack_para1, car_ack_para2);
		if (ack->key & KEY_FORWARD) TOY_LOG("    +FORWARD\n");
		if (ack->key & KEY_BACK)    TOY_LOG("    +BACK\n");
		if (ack->key & KEY_LEFT)    TOY_LOG("    +LEFT\n");
		if (ack->key & KEY_RIGHT)   TOY_LOG("    +RIGHT\n");
		if (ack->key & KEY_OPEN)    TOY_LOG("    +OPEN\n");
		if (ack->key & KEY_DEMO)    TOY_LOG("    +DEMO\n");
		if (ack->key & KEY_K8)      TOY_LOG("    +K8\n");
		if (ack->key & KEY_K7)      TOY_LOG("    +K7\n");
	}
}
#endif /* BIDIR_COMM_ENABLE */

void toy_init(void)
{
#if (BUTTON_TEST_ENABLE)
	app_test_btn_init();  // PA0 button test: configure pull-up input
#endif
#if (SLEEP_ALLOW_TEST_ENABLE)
	app_test_sleep_allow_init();  // PA0 sleep-allow test: configure pull-up input
#endif
}

int remote_loop(void)
{
#if (BUTTON_TEST_ENABLE)
	app_test_btn_loop();  // PA0 button test: scan + report
#endif
#if (SLEEP_ALLOW_TEST_ENABLE)
	app_test_sleep_allow_loop();  // PA0 sleep-allow test: scan + toggle allow_sleep
#endif
	return 0;
}

/**
 * Wakeup-source callback framework.
 *
 * Design rationale:
 *   - Direct function call (not function pointer, not weak symbol).
 *   - Empty placeholder for now: wakeup sources are not configured yet.
 *   - To configure a wakeup source in the future, follow the original
 *     hw_tower_key_set_wakeup_level_suspend() pattern:
 *
 *       // Low-level wakeup pin (e.g. button to GND):
 *       gpio_intput_pullup_10k(pin);
 *       cpu_set_gpio_wakeup(pin, Level_Low, enable);
 *
 *       // High-level wakeup pin (e.g. signal to VCC):
 *       gpio_intput_pulldown_100k(pin);
 *       cpu_set_gpio_wakeup(pin, Level_High, enable);
 */

void hw_sleep_wakeup_set_gpio_suspend(u8 enable)
{
	(void)enable;
	/* TODO: configure GPIO wakeup sources here when needed.
	 * Pattern reference: original hw_tower_key_set_wakeup_level_suspend()
	 * in deleted hw_tower_key.c (lines 171-204).
	 */
}

void hw_sleep_wakeup_resume_gpio(u8 from_suspend)
{
#if (BUTTON_TEST_ENABLE)
	if (from_suspend) {
		// Suspend wakeup: re-apply the PA0 pad config (input pull-up)
		// without resetting the key state machine.
		app_test_btn_resume();
	} else {
		// Deep-sleep wakeup: all GPIO states are lost, full re-init.
		app_test_btn_init();
	}
#endif
}
