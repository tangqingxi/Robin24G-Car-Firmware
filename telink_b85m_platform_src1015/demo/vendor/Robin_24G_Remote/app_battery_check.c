/********************************************************************************************************
 * @file     AAA_battery_check.c
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
#include "app_toy/app_layer/toy_log.h"

#if (FPGA == 1) //The ROM size cannot exceed 32KB; delete unnecessary logs.
#undef TOY_LOG
#define TOY_LOG(...)
#endif

#if (ADC_SAMPLE_MODE == ADC_GPIO_SAMPLE)
adc_gpio_cfg_t adc_gpio_cfg_m =
{
        .v_ref       = ADC_VREF_1P2V,
        .pre_scale   = ADC_PRESCALE_1F4,
        .sample_freq = ADC_SAMPLE_FREQ_96K,
        .pin         = GPIO_M_CHN_SAMPLE_PIN,
};
#endif

_attribute_data_retention_ u8  low_batt_flg = 0;
_attribute_data_retention_ u8  adc_start_flag = 0;//low batt tick
_attribute_data_retention_ static u8 batt_first_check = 1;

_attribute_data_retention_ u16 battery_vol_curr;//curr batt
_attribute_data_retention_ u16 battery_vol_mv;//batt vol
_attribute_data_retention_ u32 lowBattDet_tick;//low batt tick

#define BATT_CHECK_NUM 8
_attribute_data_retention_ u16 batt_check_buf[BATT_CHECK_NUM];
_attribute_data_retention_ u8 batt_check_idx;
_attribute_data_retention_ u8 batt_check_cnt;
_attribute_data_retention_ u32 low_batt_shutdown_start_tick;

// 电压阈值是否启用：0 或 0xFFFF 均表示不启用，两个阈值可独立配置
#define BATT_VOLTAGE_ENABLED(v)  ((v) != 0 && (v) != 0xFFFF)

#if BATT_CHECK_ENABLE


volatile unsigned short adc_m_chn_val;
volatile unsigned int adc_data = 0;


#if (ADC_MODE == ADC_DMA_MODE)
unsigned char sd_adc_rx_done_flag = 0;
/**
 * The reason for "ADC_SAMPLE_GROUP_CNT + 2" is as follows:
 * Since the write-back feature of ADC's DMA5 is enabled by default and cannot be disabled, and the length information occupies 1 word,
 * the buffer must be allocated with 1 additional word of space.
 */
unsigned short adc_sample_buffer[(ADC_SAMPLE_GROUP_CNT + 2) * ADC_SAMPLE_CHN_CNT] __attribute__((aligned(4))) = {0};
#elif (ADC_MODE == ADC_NDMA_MODE)
unsigned short adc_sample_buffer[ADC_SAMPLE_GROUP_CNT * ADC_SAMPLE_CHN_CNT] __attribute__((aligned(4))) = {0};
#endif
unsigned short channel_buffers[3][ADC_SAMPLE_GROUP_CNT] __attribute__((aligned(4))) = {{0}, {0}};

unsigned short adc_sort_and_get_average_code(unsigned short *channel_sample_buffer);
unsigned short adc_get_result(adc_transfer_mode_e transfer_mode, adc_sample_chn_e chn);

#if (ADC_MODE == ADC_DMA_MODE)
void adc_code_split_dma(unsigned short *sample_buffer, unsigned int sample_num, unsigned char chn_cnt, unsigned short buffers[chn_cnt][sample_num]);
#endif

void batt_adc_init_set(u8 mode)
{
	#if (ADC_MODE == ADC_NDMA_MODE)
		adc_init(NDMA_M_CHN);
		#if (ADC_SAMPLE_MODE == ADC_GPIO_SAMPLE)
		adc_gpio_sample_init(ADC_M_CHANNEL, adc_gpio_cfg_m);
		#elif (ADC_SAMPLE_MODE == ADC_VBAT_SAMPLE)
		adc_vbat_sample_init(ADC_M_CHANNEL);
		#endif
	#elif (ADC_MODE == ADC_DMA_MODE)
		adc_rx_buff_init((unsigned int *)adc_sample_buffer,ADC_SAMPLE_GROUP_CNT);
		adc_set_irq_mask();
		irq_enable_type(FLD_IRQ_DMA_EN);
		//irq_enable();
		adc_init(DMA_M_CHN);
		#if (ADC_M_CHN_SAMPLE_MODE == ADC_GPIO_SAMPLE)
		adc_gpio_sample_init(ADC_M_CHANNEL, adc_gpio_cfg_m);
		#elif (ADC_M_CHN_SAMPLE_MODE == ADC_VBAT_SAMPLE)
		adc_vbat_sample_init(ADC_M_CHANNEL);
		#endif
	#endif


		lowBattDet_tick = clock_time();
		low_batt_shutdown_start_tick = 0;
}


/**
 * @brief       This function init batt adc init
 * @return      
 * @note        
 */
void batt_adc_init(void)
{
    // Check if battery check is enabled
    if( !BATT_VOLTAGE_ENABLED(g_remote_low_battery_voltage_mv) &&
        !BATT_VOLTAGE_ENABLED(g_remote_shutdown_voltage_mv) )
    {
        return;
    }

    batt_adc_init_set(0);
}



/**
 * @brief This function serves to sort adc sample code and get average value.
 * @param[in]   channel_sample_buffer - This parameter is the first address of the received data buffer, which must be 4 bytes aligned, otherwise the program will enter an exception.
 *              and the actual buffer size defined by the user needs to be not smaller than the sample_num, otherwise there may be an out-of-bounds problem.
 * @return      adc_code_average    - the average value of adc sample code.
 */
unsigned short adc_sort_and_get_average_code(unsigned short *channel_sample_buffer)
{
    int            i, j;
    unsigned short adc_code_average = 0;
    unsigned short temp;
    uint32_t adc_sum = 0;

#if (FPGA == 0)
    for (int i = 0; i < ADC_SAMPLE_GROUP_CNT; i++) {
        if (channel_sample_buffer[i] & BIT(9)) {  //10 bit resolution, BIT(9) is sign bit, 1 means negative voltage in differential_mode
            channel_sample_buffer[i] = 0;
        } else {
            channel_sample_buffer[i] = (channel_sample_buffer[i] & 0x1ff); //BIT(8..0) is valid adc code
        }
    }
#endif

    /**** insert Sort and get average value ******/
    for (i = 1; i < ADC_SAMPLE_GROUP_CNT; i++) {
        if (channel_sample_buffer[i] < channel_sample_buffer[i - 1]) {
            temp                     = channel_sample_buffer[i];
            channel_sample_buffer[i] = channel_sample_buffer[i - 1];
            for (j = i - 1; j >= 0 && channel_sample_buffer[j] > temp; j--) {
                channel_sample_buffer[j + 1] = channel_sample_buffer[j];
            }
            channel_sample_buffer[j + 1] = temp;
        }
    }
    //get average value from raw data(abandon 1/4 small and 1/4 big data)
    for (i = ADC_SAMPLE_GROUP_CNT >> 2; i < (ADC_SAMPLE_GROUP_CNT - (ADC_SAMPLE_GROUP_CNT >> 2)); i++) {
		adc_sum += (uint32_t)channel_sample_buffer[i];
    }
	adc_code_average = adc_sum / (ADC_SAMPLE_GROUP_CNT >> 1);
    return adc_code_average;
}


/**
 * @brief      This function serves to convert to voltage value and temperature value.
 * @param[in]  transfer_mode -enum variable of adc code transfer mode.
 * @param[in]  chn - enum variable of ADC sample channel.
 * @return     adc_result   - adc voltage value or temperature value.
 */
unsigned short adc_get_result(adc_transfer_mode_e transfer_mode, adc_sample_chn_e chn)
{
    unsigned short code_average;
    unsigned short adc_result;

#if (ADC_MODE == ADC_NDMA_MODE)
    unsigned int   cnt = 0;

    //if (transfer_mode == NDMA)
    {
        adc_start_sample_nodma();

        while (cnt < ADC_SAMPLE_GROUP_CNT) {
            int sample_cnt = adc_get_rxfifo_cnt();
            if (sample_cnt > 0) {

                adc_data =  adc_get_raw_code();
                channel_buffers[chn][cnt] = adc_data & 0xffff;
                channel_buffers[chn][cnt+1] = adc_data >> 16;
			// #if (FPGA == 0)
            //     if (channel_buffers[chn][cnt] & BIT(9)) {  //10 bit resolution, BIT(9) is sign bit, 1 means negative voltage in differential_mode
            //         channel_buffers[chn][cnt] = 0;
            //     } else {
            //         channel_buffers[chn][cnt] &= 0x1FF;    //BIT(8..0) is valid adc code
            //     }
			// #endif

                cnt += 2;
            }
        }
    }
#endif

    code_average = adc_sort_and_get_average_code(&channel_buffers[chn][0]);
    return adc_result = adc_calculate_voltage(chn, code_average);
}


#if (ADC_MODE == ADC_DMA_MODE)
/**
 * @brief       This function serves to split the data from all channels in the sample buffer into different channels.
 * @param[in]   sample_buffer - This parameter is the first address of the received data buffer, which must be 4 bytes aligned, otherwise the program will enter an exception.
 *                              and the actual buffer size defined by the user needs to be not smaller than the sample_num, otherwise there may be an out-of-bounds problem.
 * @param[in]   sample_num    - This parameter is used to set the size of the received dma and must be set to a multiple of 4. The maximum value that can be set is 0xFFFFFC.
 * @param[in]   chn_cnt -number of channels used.
 * @param[in]   buffers -This parameter is the first address of ADC sample channel buffers, which must be 4 bytes aligned, otherwise the program will enter an exception.
 * @return      none
 * @note        Since the write-back feature of ADC's DMA5 is enabled by default and cannot be disabled, and the length information occupies 1 word,
 *              the address of sample_buffer needs to be offset by 1 word.
 */
void adc_code_split_dma(unsigned short *sample_buffer, unsigned int sample_num, unsigned char chn_cnt, unsigned short buffers[chn_cnt][sample_num])
{
    unsigned int i, j;
    for (i = 0; i < chn_cnt; i++) {
        for (j = 0; j < sample_num; j++) {
            buffers[i][j] = sample_buffer[j * chn_cnt + i];
        }
    }
}
#endif


/**
 * @brief       This function use for check battery power
 * @return      
 * @note        
 */
void user_battery_power_check(u16 batt_vol_mv)
{
#if (ADC_MODE == ADC_NDMA_MODE)
	adc_stop_sample_nodma();
#elif (ADC_MODE == ADC_DMA_MODE)
	//adc_stop_sample_dma();
#endif
	adc_power_off();

	batt_check_buf[batt_check_idx] = batt_vol_mv;
	batt_check_idx++;
	if (batt_check_idx >= BATT_CHECK_NUM) {
		batt_check_idx = 0;
	}

	if (batt_check_cnt < BATT_CHECK_NUM) {
		batt_check_cnt++;
	}

	if (batt_check_cnt >= BATT_CHECK_NUM)
	{
		u16 min_val = 0xFFFF;
		u16 max_val = 0;
		u32 sum_val = 0;

		for (int i = 0; i < BATT_CHECK_NUM; i++) {
			if (batt_check_buf[i] < min_val) min_val = batt_check_buf[i];
			if (batt_check_buf[i] > max_val) max_val = batt_check_buf[i];
			sum_val += batt_check_buf[i];
		}

		sum_val = sum_val - min_val - max_val;
		battery_vol_curr = sum_val / (BATT_CHECK_NUM - 2);

		TOY_LOG("batt_vol_mv:%d, battery_vol_curr:%d.\n", batt_vol_mv, battery_vol_curr);

		if(BATT_VOLTAGE_ENABLED(g_remote_shutdown_voltage_mv) &&
		   battery_vol_curr <= g_remote_shutdown_voltage_mv)
		{
			TOY_LOG("S10 Remote Low Battery Shutdown! Vbat: %d mV\n", battery_vol_curr);
			enter_deep(ANA_BATTERY_SHUTDOWN_ENTER_DEEPSLEEP);//enter deep because batter shut down
		}else if (BATT_VOLTAGE_ENABLED(g_remote_low_battery_voltage_mv) &&
				  battery_vol_curr <= g_remote_low_battery_voltage_mv){
			low_batt_flg = 1;
		}else{
			low_batt_flg = 0;
		}
	}

}


void user_batt_proc_start(void)
{
    	adc_power_on();

	#if (ADC_MODE == ADC_NDMA_MODE)
        battery_vol_mv = adc_get_result(NDMA, ADC_M_CHANNEL);

        user_battery_power_check(battery_vol_mv);
	#elif (ADC_MODE == ADC_DMA_MODE)
        if (sd_adc_rx_done_flag == 0)
        {
        	lowBattDet_tick = clock_time();//update lowbattdet tick

        	adc_start_flag = 1;

        	adc_start_sample_dma();
        }
	#endif
}

/**
 * @brief       This function pro battery check
 * @return      
 * @note        
 */
void user_batt_check_proc()
{
    u32 check_interval = 500000;

    if(batt_first_check)
    {
        check_interval = 3000000;
    }

    if (clock_time_exceed(lowBattDet_tick, check_interval))
    {
		batt_first_check = 0;
		lowBattDet_tick = clock_time();//update lowbattdet tick
        if(BATT_VOLTAGE_ENABLED(g_remote_low_battery_voltage_mv) ||
           BATT_VOLTAGE_ENABLED(g_remote_shutdown_voltage_mv))
        {
			user_batt_proc_start();
	    }
		
#if (ADC_MODE == ADC_DMA_MODE)
		if (sd_adc_rx_done_flag == 1)
		{
			sd_adc_rx_done_flag = 0;
			adc_start_flag = 0;
			
// #if (FPGA == 0)
// 			for (int i = 1; i < ((ADC_SAMPLE_GROUP_CNT+2) * ADC_SAMPLE_CHN_CNT); i++) {
// 				if (adc_sample_buffer[i] & BIT(9)) {  //10 bit resolution, BIT(9) is sign bit, 1 means negative voltage in differential_mode
// 					adc_sample_buffer[i] = 0;
// 				} else {
// 					adc_sample_buffer[i] = (adc_sample_buffer[i] & 0x1FF); //BIT(8..0) is valid adc code
// 				}
// 			}
// #endif

			adc_code_split_dma((unsigned short *)(adc_sample_buffer+2), ADC_SAMPLE_GROUP_CNT, ADC_SAMPLE_CHN_CNT, channel_buffers);
			battery_vol_mv = adc_get_result(DMA, ADC_M_CHANNEL);
			user_battery_power_check(battery_vol_mv);
		}
#endif
	}
}
#endif
