/********************************************************************************************************
 * @file     app_toy.h
 *
 * @brief    This is the header file for KMD SDK
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

#ifndef _APP_TOY_H_
#define _APP_TOY_H_

#include "app_public_config.h"
#include "app_24g_rf_frame.h"

/**
 * @brief Toy application init. Called from main() (toy_user_init).
 *        Empty placeholder: car has no peripherals yet.
 */
void toy_init(void);

/**
 * @brief Toy application main loop. Called from main_loop_process().
 *        Dequeues pending control commands and acts on them
 *        (log only for now).
 */
void car_loop(void);

/**
 * @brief Car control entry, called from RF RX IRQ.
 *        Only enqueues the command; processing happens in car_loop().
 * @param[in] cmd     - control command frame from Remote (CMD_DATA payload)
 * @param[in] seq_no  - RF sequence number
 */
void toy_car_ctrl_on_rf_km_dat(const rf_cmd_frame_t *cmd, u8 seq_no);

#endif /* _APP_TOY_H_ */
