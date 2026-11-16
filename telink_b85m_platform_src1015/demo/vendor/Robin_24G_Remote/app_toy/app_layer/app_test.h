
/********************************************************************************************************
 * @file     app_test.h
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

/*
 * app_test.h
 *
 *  Created on: Dec 5, 2025
 *      Author: lingzhi.chen
 */

#ifndef _APP_TEST_H_
#define _APP_TEST_H_

#include "app_public_config.h"

/* ================= 测试开关（非正式代码，放在本测试头文件） ================= */
#define PACKET_LOSS_TEST_ENABLE     0   // 包丢失测试：1 使能 app_test.c 中丢包统计
#define BUTTON_TEST_ENABLE          0   // PA0 按键测试：1 使能按键扫描并发送 KEY_FORWARD
#define SLEEP_ALLOW_TEST_ENABLE     0   // 休眠准入测试：1 使能 PA0 切换 allow_sleep
                                        //（与 BUTTON_TEST_ENABLE 互斥，两者共用 PA0）

#if (PACKET_LOSS_TEST_ENABLE)
void app_test_on_mouse_ack_rcvd(void);
void app_test_on_tx_attempt(void);
void app_test_on_ack_rcvd(void);
void app_test_on_pair_success(void);
void app_test_on_reconnect(void);
void app_test_on_ack_miss(void);
void app_test_on_disconnect(void);
void app_test_packet_loss_ui_loop(void);
void app_test_on_tx_packet(u8 mode, const void *p_data);
#endif

/* PA0 button test (guarded by BUTTON_TEST_ENABLE, see app_config.h). */
#if (BUTTON_TEST_ENABLE)
void app_test_btn_init(void);      // full init (cold boot / deep-sleep wakeup)
void app_test_btn_resume(void);    // re-apply GPIO pad config after suspend wakeup
void app_test_btn_loop(void);
#endif

/* PA0 sleep-allow test (guarded by SLEEP_ALLOW_TEST_ENABLE). */
#if (SLEEP_ALLOW_TEST_ENABLE)
void app_test_sleep_allow_init(void);   // full init (GPIO + 默认允许休眠)
void app_test_sleep_allow_loop(void);   // PA0 按下沿切换休眠准入
#endif

#endif // _APP_TEST_H_
