/********************************************************************************************************
 * @file     app_toy.c
 *
 * @brief    This is the source file for KMD SDK
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

#include "app_toy.h"
#include "app_24g_rf_common.h"
#include "toy_log.h"

/* ------------------------------------------------------------------ *
 *  Car control RX queue
 *  RF RX IRQ enqueues via toy_car_ctrl_on_rf_km_dat(), the main loop
 *  dequeues via car_loop(). This keeps peripheral outputs (PWM/motor/...)
 *  out of the ISR; hook them in toy_car_ctrl_handle_cmd().
 * ------------------------------------------------------------------ */
#define TOY_CAR_CTRL_RX_Q_LEN 8

typedef struct {
	rf_cmd_frame_t cmd;   // full command frame
	u8 rf_seq_no;         // RF sequence number
} toy_car_ctrl_rx_item_t;

static volatile toy_car_ctrl_rx_item_t s_rx_q[TOY_CAR_CTRL_RX_Q_LEN];
static volatile u8 s_rx_rptr = 0;   // read pointer (main loop)
static volatile u8 s_rx_wptr = 0;   // write pointer (ISR)
static volatile u32 s_rx_overflow = 0;

/**
 * @brief       Car control entry: receive a remote control command.
 *              Called from RF RX IRQ - only enqueues, processing is done
 *              in the main loop by car_loop().
 * @param[in]   cmd     - control command frame from Remote (CMD_DATA payload)
 * @param[in]   seq_no  - RF sequence number
 */
void toy_car_ctrl_on_rf_km_dat(const rf_cmd_frame_t *cmd, u8 seq_no)
{
	// ISR context: enqueue quickly, process in main loop.
	u8 r = irq_disable();
	u8 next = (u8)((s_rx_wptr + 1) % TOY_CAR_CTRL_RX_Q_LEN);
	if (next == s_rx_rptr) {
		// Drop the oldest to keep the most recent key-state.
		s_rx_rptr = (u8)((s_rx_rptr + 1) % TOY_CAR_CTRL_RX_Q_LEN);
		s_rx_overflow++;
	}
	memcpy(&s_rx_q[s_rx_wptr].cmd, cmd, sizeof(rf_cmd_frame_t)); // byte-wise copy: safe for packed struct
	s_rx_q[s_rx_wptr].rf_seq_no = seq_no;
	s_rx_wptr = next;
	irq_restore(r);
}

/**
 * @brief       Handle one control command in main-loop context.
 * @param[in]   cmd - control command frame
 * @note        No peripherals yet (motor/spray/etc. removed). For now, each distinct
 *              control command is logged with its type/event/params for debugging.
 *              Hook the real outputs (PWM/motor/...) here once peripherals are added.
 */
static void toy_car_ctrl_handle_cmd(const rf_cmd_frame_t *cmd)
{
	// The same command is received once per hop channel (5 hops), so skip
	// duplicates to keep the log readable.
	static u8 last_key;
#if (RF_CMD_DATA_DAT_LEN > 0)
	static u8 last_dat[RF_CMD_DATA_DAT_LEN];
#endif
#if (RF_CMD_DATA_DAT_LEN > 0)
	if (cmd->key == last_key &&
	    memcmp(cmd->dat, last_dat, RF_CMD_DATA_DAT_LEN) == 0) {
		return;
	}
	last_key = cmd->key;
	memcpy(last_dat, cmd->dat, RF_CMD_DATA_DAT_LEN);
#else
	if (cmd->key == last_key) {
		return;
	}
	last_key = cmd->key;
#endif

	// 旧格式：key==0 = keepalive（无按键/全释放），非 0 = 按键事件
	if (cmd->key == 0) {
		TOY_LOG("---Car Ctrl: KEEPALIVE\n");
	} else {
#if (RF_CMD_DATA_DAT_LEN >= 1)
		TOY_LOG("---Car Ctrl: KEY key=0x%02x dat[0]=%d\n", cmd->key, cmd->dat[0]);
#else
		TOY_LOG("---Car Ctrl: KEY key=0x%02x\n", cmd->key);
#endif
		if (cmd->key & KEY_FORWARD) TOY_LOG("    +FORWARD\n");
		if (cmd->key & KEY_BACK)    TOY_LOG("    +BACK\n");
		if (cmd->key & KEY_LEFT)    TOY_LOG("    +LEFT\n");
		if (cmd->key & KEY_RIGHT)   TOY_LOG("    +RIGHT\n");
		if (cmd->key & KEY_OPEN)    TOY_LOG("    +OPEN\n");
		if (cmd->key & KEY_DEMO)    TOY_LOG("    +DEMO\n");
		if (cmd->key & KEY_K8)      TOY_LOG("    +K8\n");
		if (cmd->key & KEY_K7)      TOY_LOG("    +K7\n");
	}
}

/**
 * @brief       Toy application main loop: dequeue and handle each pending
 *              control command. Called from main_loop_process().
 */
void car_loop(void)
{
	while (s_rx_rptr != s_rx_wptr) {
		toy_car_ctrl_rx_item_t item;
		u8 r = irq_disable();
		item = s_rx_q[s_rx_rptr];
		s_rx_rptr = (u8)((s_rx_rptr + 1) % TOY_CAR_CTRL_RX_Q_LEN);
		irq_restore(r);

		toy_car_ctrl_handle_cmd(&item.cmd);
	}
}

/**
 * @brief       Toy application init. Called from main() (toy_user_init).
 *              Empty placeholder: car has no peripherals to init yet.
 */
void toy_init(void)
{
}
