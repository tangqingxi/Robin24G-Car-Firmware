/********************************************************************************************************
 * @file     app_24g_rf_common.c
 *
 * @brief    RF protocol shared instances between Robin_24G_Car (RX) and
 *           Robin_24G_Remote (TX). Defines the shared access-code table and
 *           application-layer command buffer; user_var/pair_var instances
 *           live in each vendor's own app_24g_rf.c / app_24g_rx.c.
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

#include "app_24g_rf_common.h"
#include "app_public_config.h"

/* Pairing access code table — 16 preset codes indexed by access_code_group.
 * Used by both Car (RX) and Remote (TX) to select the pairing pipe 0 code. */
const u32 pair_access_code_array[16] =
{
        0x39517695, //0
        0x59727696, //1
        0x76937697, //2
        0x96b47698, //3
        0xb5c57699, //4
        0xd5d6769a, //5
        0xe317769b, //6
        0x2239769c, //7
        0x3257769c, //8
        0x5176769b, //9
        0x719b769a, //a
        0x92ac7699, //b
        0xb3ad7698, //c
        0xd4b97697, //d
        0xe51e7696, //e
        0x26317695, //f
};

/* Application-layer command buffer. Pushed into fifo_km by the TX side
 * (Remote) and parsed by the RX side (Car). */
_attribute_data_retention_ remote_cmd_t cmd_data;
