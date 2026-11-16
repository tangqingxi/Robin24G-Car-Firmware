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
#include "app_battery_check.h"
#include "app_toy/app_layer/toy_log.h"
#include "app_toy/app_layer/app_toy.h"

_attribute_data_retention_ custom_cfg_t user_cfg =
{
#if EMI_TEST_FUN_ENABLE_AAA
	.emi_tx_power = DEFAULT_EMI_TX_POWER,
#endif
};
_attribute_data_retention_ flash_dev_info_t flash_dev_info;
_attribute_data_retention_ u8 deep_flag = ANA_POWER_ON;
_attribute_data_retention_ u8 pair_flag = 0;

// Runtime configuration globals (initialized from app_config.h defaults).
// _attribute_data_retention_ keeps them across deepsleep.
_attribute_data_retention_ u16 g_remote_shutdown_voltage_mv    = REMOTE_SHUTDOWN_VOLTAGE_MV;
_attribute_data_retention_ u16 g_remote_low_battery_voltage_mv = REMOTE_LOW_BATTERY_VOLTAGE_MV;
_attribute_data_retention_ remote_sleep_timeout_mode_t g_remote_sleep_mode = REMOTE_SLEEP_MODE;
_attribute_data_retention_ u16 g_remote_sleep_timeout_s        = REMOTE_SLEEP_TIMEOUT_S;
_attribute_data_retention_ u8  g_remote_access_code_group      = REMOTE_ACCESS_CODE_GROUP;
_attribute_data_retention_ u8  g_remote_normal_tx_power        = REMOTE_NORMAL_TX_POWER;

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


//_attribute_data_retention_user TX_FIFO_AAA_STRUCT tx_fifo_aaa;


_attribute_data_retention_user u8 g_user_payload[sizeof(rf_cmd_frame_t)] = {0};



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
	hw_sleep_wakeup_set_gpio_suspend(enable);
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


