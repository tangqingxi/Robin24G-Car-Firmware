/********************************************************************************************************
 * @file     app_test.c
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

#include "app_test.h"
#include "app_toy.h"
#include "app_toy/app_layer/toy_log.h"
#include "string.h"
#include "app_24g_rf.h"

#if (PACKET_LOSS_TEST_ENABLE)

// 丢包测试用 0xAC 签名区分测试包与真实按键包，签名经 CMD_DATA 帧尾 dat[]
// 传输；因此开启测试时必须给控制命令扩展字段留出空间（RF_CMD_DATA_DAT_LEN >= 1）。
#if (RF_CMD_DATA_DAT_LEN == 0)
#error "PACKET_LOSS_TEST_ENABLE requires RF_CMD_DATA_DAT_LEN >= 1 (0xAC test signature is carried in dat[])"
#endif

#define PACKET_LOSS_TEST_COUNT      10000  // Test 10000 packets

typedef struct {
	u32 test_seq;              // Current test packet sequence number
	u32 total_sent;            // Total packets sent (including retransmissions)
	u32 total_acked;           // Total packets acknowledged
	u32 total_lost;            // Total packets lost (after all retries)
	u32 total_retransmits;     // Total retransmission attempts
	u8  retry_count;           // Current packet retry count
	u8  test_active;           // Test is running
	u8  test_done;             // Test completed
	u32 last_send_tick;        // Last packet send time
	u32 total_send_mouse_cmd;        // Last packet send time
	u32 total_send_mouse_cmd_test;        // Last packet send time
	u32 total_send_mouse_ack_cmd;        // Last packet send time
	u32 total_send_pair_cmd;        // Last packet send time
	u32 total_rf_complete;        // Last packet send time
	u16 last_sent_seq;		// Last packet send time
} packet_loss_test_t;

_attribute_data_retention_user packet_loss_test_t pkt_test = {0};

void app_test_on_mouse_ack_rcvd(void)
{
    if (pkt_test.test_active) {
        pkt_test.total_send_mouse_ack_cmd++; // Count total transmission attempts
    }
}

void app_test_on_tx_attempt(void)
{
    if (pkt_test.test_active) {
		pkt_test.total_sent++; // Count total transmission attempts
	}
}

void app_test_on_ack_rcvd(void)
{
    if (pkt_test.test_active) {
        pkt_test.total_acked++;
    }
}

void app_test_on_pair_success(void)
{
    if (!pkt_test.test_active) {
        memset(&pkt_test, 0, sizeof(pkt_test));
        pkt_test.test_active = 1;
        TOY_LOG("--- 24G Link Test START (Pair Success) ---\n");
    }
}

void app_test_on_reconnect(void)
{
    if (!pkt_test.test_active) {
        memset(&pkt_test, 0, sizeof(pkt_test));
        pkt_test.test_active = 1;
        TOY_LOG("--- 24G Link Test START (Reconnect) ---\n");
    }
}

void app_test_on_ack_miss(void)
{
    if (pkt_test.test_active) {
        pkt_test.total_lost++;
    }
}

void app_test_on_disconnect(void)
{
    pkt_test.test_active = 0; // Stop test on disconnect
}

void app_test_packet_loss_ui_loop(void)
{
    if (pkt_test.test_active) {
		// Generate test packets
		if (pkt_test.test_seq < PACKET_LOSS_TEST_COUNT) {
			// Only send a new test packet when the TX path is idle.
			// The event high byte = 0xAC test signature; the low byte carries
			// the (wrapping) test sequence number.
			if (!tx_cmd_pending) {
				u16 test_event = (u16)(0xAC00 | (pkt_test.test_seq & 0xFF));
				if (remote_send_cmd(CMD_KEY, test_event, NULL, 0)) {
					pkt_test.test_seq++;
				}
			}
		} else {
			// Wait for all packets to be sent and acked (or lost)
			if (!tx_cmd_pending) {
				pkt_test.test_active = 0;
				pkt_test.test_done = 1;
				TOY_LOG("--- 24G Link Test FINISHED ---\n");
				TOY_LOG("Pkts Generated: %d\n", pkt_test.test_seq);
				TOY_LOG("Total Attempts: %d\n", pkt_test.total_sent);
				TOY_LOG("Total Acked:    %d\n", pkt_test.total_acked);
				if(pkt_test.total_sent) TOY_LOG("Success Rate:   %d perent\n", (pkt_test.total_acked * 100) / pkt_test.total_sent);

				TOY_LOG("Total Mouse Cmd test Sent: %d\n", pkt_test.total_send_mouse_cmd_test);
				TOY_LOG("Total Mouse Ack Received: %d\n", pkt_test.total_send_mouse_ack_cmd);
				if(pkt_test.total_send_mouse_cmd_test) TOY_LOG("Mouse Cmd test Ack Rate: %d perent\n", (pkt_test.total_send_mouse_ack_cmd * 100) / pkt_test.total_send_mouse_cmd_test);

				TOY_LOG("Total Mouse Cmd Sent: %d\n", pkt_test.total_send_mouse_cmd);
				if(pkt_test.total_send_mouse_cmd) TOY_LOG("Mouse Cmd Ack Rate: %d perent\n", (pkt_test.total_send_mouse_ack_cmd * 100) / pkt_test.total_send_mouse_cmd);
			}
		}
	}
}

void app_test_on_tx_packet(u8 mode, const void *p_data)
{
    const rf_cmd_frame_t *p_km_data = (const rf_cmd_frame_t *)p_data;
	if(mode == RF_SEND_DATA_MODE){

		if (pkt_test.test_active) {
#if (RF_CMD_DATA_DAT_LEN >= 1)
			// Test packets carry the 0xAC signature in dat[0] (forwarded from
			// event high byte by remote_send_cmd); the seq travels in the legacy
			// u8 key field (see app_test_packet_loss_ui_loop).
			if(p_km_data->dat[0] == 0xAC) {
				u8 test_seq = p_km_data->key;
				if (pkt_test.total_send_mouse_cmd_test > 0 && test_seq != (u8)(pkt_test.last_sent_seq + 1)) {
					TOY_LOG("--- 24G Link Test: Packet Seq Mismatch! Last Sent: %d, Current: %d\n",
							pkt_test.last_sent_seq, test_seq);
				}
				pkt_test.last_sent_seq = test_seq;

				pkt_test.total_send_mouse_cmd_test++; // Count total transmission attempts
			}
#endif
			pkt_test.total_send_mouse_cmd++; // Count total transmission attempts
		}

	}else if (mode == RF_SEND_PAIR_MODE){
		if (pkt_test.test_active) {
			pkt_test.total_send_pair_cmd++; // Count total transmission attempts
		}
	}
}

#endif /* PACKET_LOSS_TEST_ENABLE */

/********************************************************************************************************
 * PA0 button test (guarded by BUTTON_TEST_ENABLE)
 *
 * Hardware: PA0 to GND, internal 10K pull-up input (active low).
 *   - After pairing, a press sends CMD_KEY + KEY_FORWARD.
 *   - While held, the key state is re-armed periodically: the Car clears
 *     its keys on CMD_KEEPALIVE, so the keepalive must never interleave
 *     with a hold. CMD_KEY refreshes hb_tick, keeping the heartbeat
 *     branch of d24g_rf_loop at bay.
 *   - On release, CMD_KEY + event=0 (key release) keeps being reported
 *     for KEY_UP_DELAY_SND_CMD_S seconds, then heartbeat-only mode
 *     (d24g_rf_loop) takes over automatically.
 *   - Deep sleep (enter_deep(ANA_DEEP_SLEEP)) does not use a pin wakeup
 *     source; after waking up, hw_sleep_wakeup_resume_gpio() calls
 *     app_test_btn_init() to re-configure the pin for key scanning.
 *******************************************************************************************************/
#if (BUTTON_TEST_ENABLE)

#define BTN_TEST_PIN                GPIO_PA0
#define BTN_TEST_SCAN_INTERVAL_US   10000    // scan throttle: 10ms
#define BTN_TEST_KEY_EVENT          KEY_FORWARD

typedef enum {
	BTN_TEST_IDLE = 0,   // released, heartbeat mode
	BTN_TEST_PRESSED,    // held, sending KEY_FORWARD
	BTN_TEST_RELEASE,    // just released, sending key-release (0) for 2s
} btn_test_state_t;

_attribute_data_retention_user btn_test_state_t btn_test_state = BTN_TEST_IDLE;
_attribute_data_retention_user u32 btn_test_release_tick = 0;

static void app_test_btn_gpio_init(void)
{
	// PA0: GPIO input with internal 10K pull-up (button to GND => active low)
	gpio_set_func(BTN_TEST_PIN, AS_GPIO);
	gpio_set_input_en(BTN_TEST_PIN, 1);
	gpio_set_output_en(BTN_TEST_PIN, 0);
	gpio_setup_up_down_resistor(BTN_TEST_PIN, PM_PIN_PULLUP_10K);
}

void app_test_btn_init(void)
{
	app_test_btn_gpio_init();
	btn_test_state = BTN_TEST_IDLE;
}

void app_test_btn_resume(void)
{
	// Suspend wakeup: the GPIO pad config may be lost; re-apply it without
	// resetting the key state machine (suspend wakes up every few ms, and
	// resetting it would truncate the 2s key-release report window).
	app_test_btn_gpio_init();
}

void app_test_btn_loop(void)
{
	static u32 last_scan_tick = 0;
	u8 pressed;

	if (!clock_time_exceed(last_scan_tick, BTN_TEST_SCAN_INTERVAL_US)) {
		return;
	}
	last_scan_tick = clock_time();

	pressed = !gpio_read(BTN_TEST_PIN);   // active low

	if (device_status != STATE_NORMAL) {
		btn_test_state = BTN_TEST_IDLE;
		return;
	}

	if (btn_test_state == BTN_TEST_IDLE) {
		if (pressed) {
			btn_test_state = BTN_TEST_PRESSED;
			remote_send_cmd(CMD_KEY, BTN_TEST_KEY_EVENT, NULL, 0);
		}
	} else if (btn_test_state == BTN_TEST_PRESSED) {
		if (!pressed) {
			// Release edge: report the key release, keep repeating it for
			// KEY_UP_DELAY_SND_CMD_S before falling back to heartbeat.
			btn_test_state = BTN_TEST_RELEASE;
			btn_test_release_tick = clock_time();
			remote_send_cmd(CMD_KEY, KEY_NONE, NULL, 0);
		} else if (!tx_cmd_pending) {
			// Re-arm the key state on the link while held (see file header).
			remote_send_cmd(CMD_KEY, BTN_TEST_KEY_EVENT, NULL, 0);
		}
	} else {  // BTN_TEST_RELEASE
		if (pressed) {
			// Re-press during the release window: back to active state.
			btn_test_state = BTN_TEST_PRESSED;
			remote_send_cmd(CMD_KEY, BTN_TEST_KEY_EVENT, NULL, 0);
		} else if (clock_time_exceed(btn_test_release_tick, KEY_UP_DELAY_SND_CMD_S * 1000000UL)) {
			// Release window over: heartbeat-only mode.
			btn_test_state = BTN_TEST_IDLE;
		} else if (!tx_cmd_pending) {
			// Keep reporting the released state for the whole window.
			remote_send_cmd(CMD_KEY, KEY_NONE, NULL, 0);
		}
	}
}

#endif /* BUTTON_TEST_ENABLE */

/********************************************************************************************************
 * 休眠准入测试（guarded by SLEEP_ALLOW_TEST_ENABLE）
 *
 * Hardware: PA0 to GND, internal 10K pull-up input (active low).
 *   - 每次按下 PA0 切换一次休眠准入状态：
 *       按下  -> allow_sleep() 返回 1（允许进入休眠）
 *       再按  -> allow_sleep() 返回 0（禁止进入休眠）
 *   - 与 BUTTON_TEST_ENABLE 互斥（两者共用 PA0）。
 *******************************************************************************************************/
#if (SLEEP_ALLOW_TEST_ENABLE)

#define SLEEP_ALLOW_TEST_PIN                GPIO_PA0
#define SLEEP_ALLOW_TEST_SCAN_INTERVAL_US   10000   // 扫描节流：10ms

// 休眠准入状态：1 = 允许休眠（默认），0 = 禁止休眠
_attribute_data_retention_user u8 s_sleep_allow = 1;

// 覆盖 SDK weak 默认实现：休眠准入由 PA0 测试按键控制
u8 allow_sleep(void)
{
	return s_sleep_allow;
}

static void app_test_sleep_allow_gpio_init(void)
{
	// PA0: GPIO input with internal 10K pull-up (button to GND => active low)
	gpio_set_func(SLEEP_ALLOW_TEST_PIN, AS_GPIO);
	gpio_set_input_en(SLEEP_ALLOW_TEST_PIN, 1);
	gpio_set_output_en(SLEEP_ALLOW_TEST_PIN, 0);
	gpio_setup_up_down_resistor(SLEEP_ALLOW_TEST_PIN, PM_PIN_PULLUP_10K);
}

void app_test_sleep_allow_init(void)
{
	app_test_sleep_allow_gpio_init();
	s_sleep_allow = 1;   // 默认允许休眠
}

void app_test_sleep_allow_loop(void)
{
	static u32 last_scan_tick = 0;
	static u8 last_pressed = 0;   // 上次按键状态（1 = 按下）

	if (!clock_time_exceed(last_scan_tick, SLEEP_ALLOW_TEST_SCAN_INTERVAL_US)) {
		return;
	}
	last_scan_tick = clock_time();

	u8 pressed = !gpio_read(SLEEP_ALLOW_TEST_PIN);   // active low

	if (pressed && !last_pressed) {
		// 按下沿：切换休眠准入状态
		s_sleep_allow = !s_sleep_allow;
		TOY_LOG("---Sleep allow test: allow_sleep=%d\n", s_sleep_allow);
	}
	last_pressed = pressed;
}

#endif /* SLEEP_ALLOW_TEST_ENABLE */
