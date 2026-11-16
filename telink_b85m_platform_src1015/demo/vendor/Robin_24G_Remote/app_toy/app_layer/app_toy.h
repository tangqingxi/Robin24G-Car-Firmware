/********************************************************************************************************
 * @file     app_toy.h
 *
 * @brief    This is the header file for KMD SDK
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

#ifndef _APP_TOY_H_
#define _APP_TOY_H_

#include "app_public_config.h"

void toy_init(void);
int  remote_loop(void);

#if (BIDIR_COMM_ENABLE)
/**
 * @brief Car ACK handling entry: called from RF RX IRQ when a data ACK from
 *        the Car is received (bidirectional-comm mode). Aggregates car status
 *        and drives remote peripherals (display/vibration/...); extend here
 *        for future needs.
 * @param[in] ack    - data ACK frame from Car (CMD_ACK, key = echoed key,
 *                     dat[0]/dat[1] = status, length由 RF_CMD_DATA_ACK_DAT_LEN 控制)
 * @param[in] seq_no - RF sequence number
 */
void toy_remote_ack_on_rf_km_dat(const rf_cmd_ack_frame_t *ack, u8 seq_no);
#endif /* BIDIR_COMM_ENABLE */

/**
 * @brief Configure GPIO wakeup sources before entering suspend/deepsleep.
 *        Called from gpio_set_wakeup_enable() and pm_poll() before
 *        cpu_sleep_wakeup_32k_rc(). Reference pattern: original
 *        hw_tower_key_set_wakeup_level_suspend().
 *
 *        Current implementation: empty placeholder. Add GPIO wakeup
 *        source configuration here when needed (e.g. gpio_intput_pullup_10k
 *        + cpu_set_gpio_wakeup(pin, Level_Low, enable)).
 *
 * @param[in] enable  1 = enable wakeup, 0 = disable wakeup
 */
void hw_sleep_wakeup_set_gpio_suspend(u8 enable);

/**
 * @brief Restore GPIO working state after waking up from suspend/deepsleep.
 *        Called from main() (after toy_user_init) and from pm_poll() (after
 *        cpu_sleep_wakeup_32k_rc returns).
 *
 *        Current implementation: empty placeholder.
 *
 * @param[in] from_suspend  1 = resumed from shallow sleep (pm_poll),
 *                          0 = resumed from deepsleep (main re-entry)
 */
void hw_sleep_wakeup_resume_gpio(u8 from_suspend);

#endif /* _APP_TOY_H_ */
