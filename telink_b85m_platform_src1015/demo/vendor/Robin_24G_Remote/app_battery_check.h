/********************************************************************************************************
 * @file     AAA_battery_check.h
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

#ifndef _APP_BATTERY_CHECK_H_
#define _APP_BATTERY_CHECK_H_

#define	LOW_BATT_FLG	0x5A


extern unsigned short battery_vol_mv;
extern unsigned char low_batt_flg;
extern unsigned char adc_start_flag;//low batt tick

void batt_adc_init(void);

u16  user_btn_adc_check_proc(u8 chn);

extern void user_battery_power_check(u16 batt_vol_mv);

/**
 * @brief      This function serves to convert to voltage value and temperature value.
 * @param[in]  transfer_mode -enum variable of adc code transfer mode.
 * @param[in]  chn - enum variable of ADC sample channel.
 * @return     adc_result   - adc voltage value or temperature value.
 */
unsigned short adc_get_result(adc_transfer_mode_e transfer_mode, adc_sample_chn_e chn);


#endif /* APP_BATTDET_H_ */
