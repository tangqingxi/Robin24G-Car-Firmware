/********************************************************************************************************
 * @file     AAA_Public.c
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
#include "toy_log.h"

_attribute_data_retention_ flash_dev_info_t flash_dev_info;
_attribute_data_retention_ u8 deep_flag = ANA_POWER_ON;
_attribute_data_retention_ u8 pair_flag = 0;

// Runtime configuration globals (initialized from app_config.h defaults).
// _attribute_data_retention_ keeps them across deepsleep.
_attribute_data_retention_ u8 g_car_access_code_group = CAR_ACCESS_CODE_GROUP;
_attribute_data_retention_ u8 g_car_normal_tx_power   = CAR_NORMAL_TX_POWER;
_attribute_data_retention_ u16 g_car_shutdown_voltage_mv    = CAR_SHUTDOWN_VOLTAGE_MV;
_attribute_data_retention_ u16 g_car_low_battery_voltage_mv = CAR_LOW_BATTERY_VOLTAGE_MV;
_attribute_data_retention_ car_sleep_timeout_mode_t g_car_sleep_mode = CAR_SLEEP_MODE;
_attribute_data_retention_ u16 g_car_sleep_timeout_s        = CAR_SLEEP_TIMEOUT_S; // custom mode, 0 = never sleep



extern void cpu_set_gpio_wakeup(GPIO_PinTypeDef pin, GPIO_LevelTypeDef pol, int en);

	
void my_fifo_init (my_fifo_t *f, int s, u8 n, u8 *p)
{
	f->size = s;//set fifo size to s
	f->num = n;//set fifo num to n
	f->wptr = 0;//set wptr to 0
	f->rptr = 0;//set rptr to 0
	f->p = p;//fifo data pointer to p
}


/**
 * @brief       This function return fifo write pointer
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
u8* my_fifo_wptr (my_fifo_t *f)
{
	if (((f->wptr - f->rptr) & 255) < f->num)
	{
		return f->p + (f->wptr & (f->num-1)) * f->size;
	}
	return 0;
}


/**
 * @brief       This function wptr ++
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
void my_fifo_next (my_fifo_t *f)
{
	f->wptr++;//wptr++
}


/**
 * @brief       This function push data to fifo f
 * @param[in]   f	- fifo f
 * @param[in]   n	- data len
 * @param[in]   p	- the data pointer need sended
 * @return      
 * @note        
 */
int my_fifo_push (my_fifo_t *f, u8 *p, int n)
{
	if (((f->wptr - f->rptr) & 255) >= f->num)//if write wptr overlap num
	{
		return -1;
	}

	if (n >= f->size)//if data len over fifo size
	{
		return -1;
	}
	u8 *pd = f->p + (f->wptr++ & (f->num-1)) * f->size;
	//*pd++ = n & 0xff;
	//*pd++ = (n >> 8) & 0xff;
	memcpy (pd, p, n);//cpy data to fifo
	return 0;
}
	

/**
 * @brief       This function rptr ++
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
void my_fifo_pop (my_fifo_t *f)
{
	if (f->rptr != f->wptr)
	{
		f->rptr++;
	}
}


/**
 * @brief       This function get fifo data
 * @param[in]   f	- fifo
 * @return      
 * @note        
 */
u8 * my_fifo_get (my_fifo_t *f)
{
	if (f->rptr != f->wptr)//if have data
	{
		u8 *p = f->p + (f->rptr & (f->num-1)) * f->size;//pointer to read data
		return p;
	}
	return 0;
}


/**
 * @brief       This function reset fifo
 * @param[in]   f	- fifo need reset
 * @return      
 * @note        
 */
void my_fifo_reset (my_fifo_t *f)
{
	f->rptr = 0;//rptr set to 0
	f->wptr = 0;//wptr set to 0
}


#if TEST_DRAW_A_SQUARE
u8 auto_draw_flag=0;
#endif
//_attribute_data_retention_user  u8 active_disconnect_reason=0;

//_attribute_data_retention_user  u32 power_on_tick = 0;
//_attribute_data_retention_user  u8 device_name_len = 0;
_attribute_data_retention_ u8 connect_ok = 0;

//_attribute_data_retention_user u8 has_been_paired_flag = 0;

//_attribute_data_retention_user u32 loop_cnt;
_attribute_data_retention_ u32 idle_tick;
_attribute_data_retention_ u16 idle_count = 0;
_attribute_data_retention_ u32 adv_begin_tick;
_attribute_data_retention_ u16 adv_count = 0;

//_attribute_data_retention_user u8 has_new_report_aaa;
_attribute_data_retention_ u8 has_new_key_event = 0;


//_attribute_data_retention_user TX_FIFO_AAA_STRUCT tx_fifo_aaa;


_attribute_data_retention_user u8 btn_value = 0;
_attribute_data_retention_user u8 last_btn_value = 0;
_attribute_data_retention_user u8 g_user_payload[sizeof(rf_cmd_frame_t)] = {0};




#if BUTTON_FUN_ENABLE

/**
 * @brief       This function set btn gpio wakeup
 * @param[in]   enable	- enable
 * @return      
 * @note        
 */
void btn_set_wakeup_level_suspend(u8 enable)
{
    if (enable)
    {
        gpio_setup_up_down_resistor(PIN_BTN_MAIN, PM_PIN_PULLDOWN_100K);
    }
    else
    {
        gpio_setup_up_down_resistor(PIN_BTN_MAIN, PM_PIN_PULLUP_1M);
    }

    cpu_set_gpio_wakeup(PIN_BTN_MAIN, 0, enable); 
}

/**
 * @brief       This function get btn status
 * @param[in]   pin	- the pin need get status
 * @return      
 * @note        
 */
static inline u8 button_get_status(u32 pin)
{
	(void)pin;   
    return !gpio_read(PIN_BTN_MAIN);
}
#endif


void test_gpio_init(GPIO_PinTypeDef pin)
{
    gpio_set_func(pin, AS_GPIO);
    gpio_set_output_en(pin, 1);
    gpio_set_input_en(pin, 0);
    gpio_setup_up_down_resistor(pin, PM_PIN_PULLDOWN_100K);
    gpio_write(pin, 0);
}


/**
 * @brief       This function reset idle status parameter
 * @return      
 * @note        
 */
void reset_idle_status()
{
    idle_count = 0;//idle count reset to 0
    //loop_cnt = 0;
    idle_tick = clock_time();//update idle tick
}


/**
 * @brief       This function poll idle parameters
 * @return      
 * @note        
 */
void idle_status_poll()
{
    u8 n = ((u32)(clock_time() - idle_tick)) / CLOCK_SYS_TIMER_CLK_1S;

    idle_tick += n * CLOCK_SYS_TIMER_CLK_1S;//idle_tick update

    idle_count += n;//idle count+n
}

/**
 * @brief       This function reset idle status parameter
 * @return      
 * @note        
 */
void reset_adv_status()
{
    adv_count = 0;//idle count reset to 0
    adv_begin_tick = clock_time();//update idle tick
}

/**
 * @brief       This function poll adv parameters
 * @return      
 * @note        
 */
void adv_count_poll()
{
    u8 n = ((u32)(clock_time() - adv_begin_tick)) / CLOCK_SYS_TIMER_CLK_1S;

    adv_begin_tick += n * CLOCK_SYS_TIMER_CLK_1S;//adv_begin tick update

    adv_count += n;//adv_count +n
}

void gpio_set_wakeup_enable(u8 enable)
{
#if BUTTON_FUN_ENABLE
	btn_set_wakeup_level_suspend(enable);//disable button wakeup
#endif
}


/**
 * @brief       This function let chip enter deep sleep 
 * @param[in]   reason	- the reason enter deep sleep
 * @return      
 * @note        
 */
void enter_deep(ana_status_e reason)
{
#if ENTER_SHUWDOWN_WHEN_POWER_IS_CUT_OFF_ENABLE
	if (reason == ANA_BATTERY_SHUTDOWN_ENTER_DEEPSLEEP)
	{
		gpio_set_wakeup_enable(0);
	}
	else
#endif
	{
		gpio_set_wakeup_enable(1);
	}

    deep_flag = reason;


#if MODULE_32K_WATCHDOG_ENABLE
	//wd_32k_stop();
#endif

    TOY_LOG("---enter_deep:%d.\n", deep_flag);

    cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW8K, PM_WAKEUP_PAD, 0);//enter deep sleep
}



//******************************************************************************
#define cMedian_len  5  			//中值滤波的滤波长度,一般取奇数
#define cMedian_mid  2  			//中值在滤波数组中的位置
//******************************************************************************
//******************************************************************************
_Bool PushRfDatToFIFO(unsigned short *u16p_dat, unsigned short u16DatIn)		//每Cycle采样一次并压入FIFO，RfFIFO_Index==(cMedian_len-1)后才可用
{
	unsigned char i;

	for(i=0; i<(cMedian_len-1); i++)
	{
		u16p_dat[i] = u16p_dat[i+1];
	}

	u16p_dat[cMedian_len-1] = u16DatIn;
	if(u16p_dat[cMedian_len] < (cMedian_len-1))
	{
		u16p_dat[cMedian_len]++;
		return 0;
	}
	else
	{
		if(u16p_dat[cMedian_len] < 0xffff)
		{
			u16p_dat[cMedian_len]++;
		}
		return 1;
	}
}

//******************************************************************************
//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
//******************************************************************************
//******************************************************************************
unsigned short SortingServoAdcData(unsigned short *u16Fifo_p)
{
	unsigned short	SortDatBuf[cMedian_len];   	//中值滤波的数据缓存
	unsigned int Adc_dat, LessData;
	u8 i,j;

	for(i=0;i<cMedian_len;i++)
	{
		Adc_dat=u16Fifo_p[i];
		if(i == 0)                    //缓存的第1个元素,直接放入,不需要排序
		{
			SortDatBuf[0] = Adc_dat;
		}
		else                            	//插入排序算法,按从小到大的顺序排列
		{
			for(j = 0; j < i; j++)
			{
				if( SortDatBuf[j] > Adc_dat) 	// 轮询到的当前元素 > 当前值,则交换它们的值，xd为中间变量存放位置
				{
					LessData = Adc_dat;
					Adc_dat = SortDatBuf[j];
					SortDatBuf[j] = LessData;
				}
			}
			SortDatBuf[i] = Adc_dat; 	//把轮询出较大的数放入缓存的后面.
		}
	}
	return SortDatBuf[cMedian_mid];	    //最终重量取中值滤波数组的中间值
}

//******************************************************************************
//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
//******************************************************************************
#define cS16Median_len  5  			//中值滤波的滤波长度,一般取奇数
#define cS16Median_mid  2  			//中值在滤波数组中的位置
//******************************************************************************
int16_t SortingFiFoData(int16_t *s16Fifo_p)
{
	int16_t	SortDatBuf[cS16Median_len];   	//中值滤波的数据缓存
	int16_t Temp_dat, LessData;
	u8 i,j;

	for(i=0;i<cS16Median_len;i++)
	{
		Temp_dat=s16Fifo_p[i];
		if(i == 0)                    //缓存的第1个元素,直接放入,不需要排序
		{
			SortDatBuf[0] = Temp_dat;
		}
		else                            	//插入排序算法,按从小到大的顺序排列
		{
			for(j = 0; j < i; j++)
			{
				if( SortDatBuf[j] > Temp_dat) 	// 轮询到的当前元素 > 当前值,则交换它们的值，xd为中间变量存放位置
				{
					LessData = Temp_dat;
					Temp_dat = SortDatBuf[j];
					SortDatBuf[j] = LessData;
				}
			}
			SortDatBuf[i] = Temp_dat; 	//把轮询出较大的数放入缓存的后面.
		}
	}
	return SortDatBuf[cS16Median_mid];	    //最终重量取中值滤波数组的中间值
}

//******************************************************************************
//******************************************************************************

