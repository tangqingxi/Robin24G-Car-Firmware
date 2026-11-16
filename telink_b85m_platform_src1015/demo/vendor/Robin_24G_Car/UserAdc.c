//*******************************************************************************
/*
 * UserAdc.c
 *
 *  Created on: 2026年10月24日
 *      Author: HuangGuangjian
 */
//*******************************************************************************
//*******************************************************************************
#include "driver.h"
#include "UserRxdFunc.h"
#include "app_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserAdc.h"
#include "app_battery_check.h"
#include "UserPwm.h"

//*******************************************************************************
//*******************************************************************************
#if User_ADC_ENABLE

//*******************************************************************************
//*******************************************************************************
#define UserADC_SAMPLE_GROUP_CNT        8 //Number of adc sample codes per channel.

//unsigned short channel_buffers[3][UserADC_SAMPLE_GROUP_CNT] __attribute__((aligned(4))) = {{0}, {0}};
extern unsigned short channel_buffers[3][UserADC_SAMPLE_GROUP_CNT];
extern volatile unsigned int adc_data;		// = 0;
unsigned short Adc_average_val;
unsigned short Adc_mV_val;

//*******************************************************************************
/**
 * @brief This function serves to sort adc sample code and get average value.
 * @param[in]   channel_sample_buffer - This parameter is the first address of the received data buffer, which must be 4 bytes aligned, otherwise the program will enter an exception.
 *              and the actual buffer size defined by the user needs to be not smaller than the sample_num, otherwise there may be an out-of-bounds problem.
 * @return      adc_code_average    - the average value of adc sample code.
 */
unsigned short UserAdc_sort_and_get_average_code(unsigned short *channel_sample_buffer)
{
    int            i, j;
    unsigned short adc_code_average = 0;
    unsigned short temp;
    uint32_t adc_sum = 0;
#if (FPGA == 0)
    for (int i = 0; i < UserADC_SAMPLE_GROUP_CNT; i++) {
        if (channel_sample_buffer[i] & BIT(9)) {  //10 bit resolution, BIT(9) is sign bit, 1 means negative voltage in differential_mode
            channel_sample_buffer[i] = 0;
        } else {
            channel_sample_buffer[i] = (channel_sample_buffer[i] & 0x1ff); //BIT(8..0) is valid adc code
        }
    }
#endif
    /**** insert Sort and get average value ******/
    for (i = 1; i < UserADC_SAMPLE_GROUP_CNT; i++) {
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
    for (i = UserADC_SAMPLE_GROUP_CNT >> 2; i < (UserADC_SAMPLE_GROUP_CNT - (UserADC_SAMPLE_GROUP_CNT >> 2)); i++) {
        adc_sum += (uint32_t)channel_sample_buffer[i];
    }
    adc_code_average = adc_sum / (UserADC_SAMPLE_GROUP_CNT >> 1);
    return adc_code_average;
}

/**
 * @brief      This function serves to convert to voltage value and temperature value.
 * @param[in]  transfer_mode -enum variable of adc code transfer mode.
 * @param[in]  chn - enum variable of ADC sample channel.
 * @return     adc_result   - adc voltage value or temperature value.
 */
unsigned short UserAdc_get_result(adc_transfer_mode_e transfer_mode, adc_sample_chn_e chn)
{
    unsigned int   cnt = 0;

    if (transfer_mode == NDMA) {
        adc_start_sample_nodma();

        while (cnt < UserADC_SAMPLE_GROUP_CNT) {
            int sample_cnt = adc_get_rxfifo_cnt();
            if (sample_cnt > 0) {

                adc_data =  adc_get_raw_code();
                channel_buffers[chn][cnt]= adc_data & 0xffff;
                channel_buffers[chn][cnt+1]= adc_data >> 16;
                cnt +=2;
            }
        }
    }

    Adc_average_val = UserAdc_sort_and_get_average_code(channel_buffers[chn]);
    Adc_mV_val = adc_calculate_voltage(chn, Adc_average_val);
    return Adc_mV_val;
}


//*******************************************************************************
// 全局变量
//*******************************************************************************
unsigned short w_AD_Val;
unsigned short w_AD_Voltage;
unsigned short w_VbatData;				//ADC采样的电池电压保存值
//*************************************************************************
//*******************************************************************************
// TC122X ADC 兼容层: 单端 GPIO 采样, 返回 B80 13bit raw 语义
//*******************************************************************************
void ADC_GPIOpin_sample_init(GPIO_PinTypeDef pin)
{
	//adc_cfg 必须在 if 外声明 (原代码声明在各分支内, 函数末尾使用会越界) -- 2026-09-06 修正
	adc_gpio_cfg_t adc_cfg = {ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PB2};	//默认 PB2
    if(pin == GPIO_PB0)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PB0};
    }
    else if(pin == GPIO_PB1)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PB1};
    }
    else if(pin == GPIO_PB2)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PB2};
    }
    else if(pin == GPIO_PA4)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PA4};
    }
    else if(pin == GPIO_PA5)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_192K, ADC_GPIO_PA5};
    }


//	if(pin != GPIO_PB2){
//		//Train_Remotor 仅用 PB2, 此处留接口
//	}
	adc_gpio_sample_init(ADC_M_CHANNEL, adc_cfg);
}

//*************************************************************************
//*************************************************************************
void adc_user_init(adc_input_pin_mode_e mode, GPIO_PinTypeDef pin)
{
	if(mode == ADC_VBAT_MODE)
	{
//		adc_init(NDMA_M_CHN);
		adc_vbat_sample_init(ADC_M_CHANNEL);
	}
	else if(mode == ADC_GPIO_MODE)
	{
//        adc_init(NDMA_M_CHN);
        ADC_GPIOpin_sample_init(pin);
	}
	adc_power_on();
	sleep_us(20);                          //通道/Vref 切换稳定时间
}

//*************************************************************************
//*************************************************************************
unsigned short Get_ADC_mV_val(void)		//采样UserADC_SAMPLE_GROUP_CNT次Adc并求平均值并换算为mV值
{
	w_AD_Voltage = UserAdc_get_result(NDMA, ADC_M_CHANNEL);		//adc_sample_and_get_result();
	w_AD_Val = Adc_average_val;
	return w_AD_Voltage;
}

unsigned short Get_ADC_val(void)		//采样UserADC_SAMPLE_GROUP_CNT次Adc并求平均值
{
	w_AD_Voltage = UserAdc_get_result(NDMA, ADC_M_CHANNEL);		//adc_sample_and_get_result();
	w_AD_Val = Adc_average_val;
	return w_AD_Val;
}

//**************************************************************************
//4.2V Li电池供电
//*************************************************************************
//*************************************************************************
#define	LowBatOffThresh 	867		//2.6V
#define cLowBatOffCntSet	6		//300		//3S
#define	LowBatWarnThresh 	1133		//3.4V	(低压报警阈值, 可按需调整)
#define	LowBatSlowThresh 	1000		//3.0V	(低压降速阈值, 可按需调整)

#define cSlowRunTmMax	4000	//低电压慢速行走的最长时间设定为40S
//*********************
unsigned char VddVolFlag;
//#define  LowBatWarnFlag		0x04
//#define  LowBatSlowFlag		0x02
//#define  LowBatOffFlag		0x01
//#define  OneSamGenFlag		0x80		//允许一次采样产生低电压标志
unsigned short BatteryLowCnt;
unsigned short BatLowerCnt;
unsigned short BatLowOffCnt;
unsigned short u16BatOffTmCnt;
//*************************************************************************
#define VbatDectByVdd	1
#define VbatDectByPin	2
#define VbatDectMode	VbatDectByPin	//VbatDectByVdd		//选择使用IC_ VDD或是I/O Pin采样电源电压
//*************************************************************************
#if(VbatDectMode == VbatDectByVdd)
//*************************************************************************
//*************************************************************************
void Bat_adc_Sampling(void)		//for Battery Voltage samples
{
	//设置采样通道
	adc_user_init(ADC_VBAT_MODE, 0);
//	sleep_us(20);
	Get_ADC_mV_val();
	w_VbatData = w_AD_Voltage;
}

//*************************************************************************
#endif	//VbatDectByVdd
//*************************************************************************
//*************************************************************************
#if(VbatDectMode == VbatDectByPin)
//*************************************************************************
#define ADC_VB_Pin	LPDadc_Pin
//*************************************************************************
void Bat_adc_Sampling(void)		//for Battery Voltage samples
{
	//设置采样通道
	adc_user_init(ADC_GPIO_MODE, ADC_VB_Pin);
	Get_ADC_mV_val();
	w_VbatData = w_AD_Voltage;
}

//*************************************************************************
#endif		//VbatDectByPin
//*************************************************************************
void GetBatteryVoltage(void)	//运行一次用时500uS， 500mS loop
{
	Bat_adc_Sampling();		//for Battery Voltage samples

	//检查低电压报警
	if(w_VbatData < LowBatWarnThresh)
	{
		BatteryLowCnt++;
		//if(BatteryLowCnt >= 180)
		if(BatteryLowCnt >= 4)
		{
			//LowBatWarnFlag=1;
			SET_BIT(VddVolFlag, LowBatWarnFlag);
		}
	}
	else
	{
		BatteryLowCnt=0;
	}

	//检查低电压降速
	if(w_VbatData < LowBatSlowThresh)
	{
		BatLowerCnt++;
		//if(BatLowerCnt >= 180)
		if(BatLowerCnt >= 4)
		{
			//LowBatSlowFlag=1;
			SET_BIT(VddVolFlag, LowBatSlowFlag);
		}
	}
	else
	{
		BatLowerCnt=0;
	}

	//检测低电压关机
	if(w_VbatData < LowBatOffThresh)
	{
		BatLowOffCnt++;
		if(BatLowOffCnt >= cLowBatOffCntSet)
		{
			//LowBatOffFlag=1;
			SET_BIT(VddVolFlag, LowBatOffFlag);
		}
	}
	else
	{
		BatLowOffCnt=0;
	}

	//低电压降运行限时
//	if(IS_BIT_SET(VddVolFlag, LowBatSlowFlag) && (IS_BIT_SET(VddVolFlag, LowBatOffFlag)==0))
//	{
//		//慢速行走最长时间限制
//		u16BatOffTmCnt++;
//		if(u16BatOffTmCnt > cSlowRunTmMax)
//		{
//			//LowBatOffFlag=1;
//			SET_BIT(VddVolFlag, LowBatOffFlag);
//		}
//	}
//	else
//	{
//		u16BatOffTmCnt=0;
//	}

	//if(OneSamGenFlag)
	if(IS_BIT_SET(VddVolFlag, OneSamGenFlag))
	{
		//OneSamGenFlag=0;
		CLR_BIT(VddVolFlag, OneSamGenFlag);

		if(w_VbatData < LowBatWarnThresh)
		{
			//LowBatWarnFlag=1;
			SET_BIT(VddVolFlag, LowBatWarnFlag);
			if(w_VbatData < LowBatSlowThresh)
			{
				//LowBatSlowFlag=1;
				SET_BIT(VddVolFlag, LowBatSlowFlag);
				if(w_VbatData < LowBatOffThresh)
				{
					//LowBatOffFlag=1;
					SET_BIT(VddVolFlag, LowBatOffFlag);
				}
			}
		}
	}
}

//*************************************************************************
unsigned short	u16Isen1AdcData;		//for 马达过流
unsigned short	u16Isen2AdcData;		//for 马达过流
//*************************************************************************
//电流采样电阻阻值为50m欧姆
//*************************************************************************
void MC_adc_Sampling(void)		//for motor current samples
{
	//设置采样通道
	adc_user_init(ADC_GPIO_MODE, Isen1_Pin);
	Get_ADC_mV_val();
	u16Isen1AdcData = w_AD_Voltage;
	adc_user_init(ADC_GPIO_MODE, Isen2_Pin);
	Get_ADC_mV_val();
	u16Isen2AdcData = w_AD_Voltage;
}

//*************************************************************************
//*************************************************************************
unsigned char	LargeCurrentFlag=0;
unsigned char	OCC_Dbs_Cnt, OCP_Exit_DbsCnt;

#define c_I_threshold	30		//300mV
//*************************************************************************
void OverCurrentHandler(void)	//10mS loop
{
	MC_adc_Sampling();

	if(LargeCurrentFlag ==0)
	{
		if((u16Isen1AdcData > c_I_threshold) ||(u16Isen2AdcData > c_I_threshold))
		{
			OCC_Dbs_Cnt++;
			if(OCC_Dbs_Cnt > 250)		//
			{
				//SET_BIT(VddVolFlag, VddLow2600Flag);	//受到程序空间限制，暂时用低电压保护替代了过流保护，更换IC后建议完善
				LargeCurrentFlag=1;	//大电流保护
				LargeCurrentProtect();
				OCC_Dbs_Cnt=0;
			}
		}
		else {
			OCC_Dbs_Cnt=0;
		}
	}
	else
	{
		OCC_Dbs_Cnt=0;
		ChkLargeCurrentProtExit();
	}
}

//*************************************************************************
//大电流保护
//大电流保护退出的条件：遥控器的档位为零
//*************************************************************************
void LargeCurrentProtect(void)
{
	if(LargeCurrentFlag)
	{
		LFmotDuty=0;
		LBmotDuty=0;
		RFmotDuty=0;
		RBmotDuty=0;
		FunCtrlFlag=0;
	}
}

void ChkLargeCurrentProtExit(void)
{
	if(LargeCurrentFlag)
	{
//		if(u16RemotorKeyVal == 0)
//		{
//			OCP_Exit_DbsCnt++;
//			if(OCP_Exit_DbsCnt > 30)	//300mS
//			{
//				LargeCurrentFlag = 0;
//			}
//		}
	}
	else
	{
		OCP_Exit_DbsCnt = 0;
	}
}

//*******************************************************************************
#endif		//#if User_ADC_ENABLE
//*******************************************************************************

