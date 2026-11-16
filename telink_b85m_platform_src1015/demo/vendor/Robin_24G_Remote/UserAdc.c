/********************************************************************************************************
 * @file	UserAdc.c
 *
 * @brief	漂移车HX261009 Remotor 单通道电位器 (VR) ADC 实现
 *
 *  - VR_POWER_PIN : 采样前拉高, 采样后拉低省电
  *  - TC122X ADC 9bit, ×16 还原 B80 13bit raw 语义 (中点 0x1000)
 *
 *******************************************************************************************************/
#include "driver.h"
#include "UserTxdFunc.h"
#include "app_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserAdc.h"
#include "app_battery_check.h"

//*******************************************************************************
//*******************************************************************************
#if ADC_ENABLE

//*******************************************************************************
#define ADC_SAMPLE_GROUP_CNT        16 //Number of adc sample codes per channel.

//unsigned short channel_buffers[3][ADC_SAMPLE_GROUP_CNT] __attribute__((aligned(4))) = {{0}, {0}};
extern unsigned short channel_buffers[3][ADC_SAMPLE_GROUP_CNT];
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
unsigned short UserAdc_get_result(adc_transfer_mode_e transfer_mode, adc_sample_chn_e chn)
{
    unsigned int   cnt = 0;

    if (transfer_mode == NDMA) {
        adc_start_sample_nodma();

        while (cnt < ADC_SAMPLE_GROUP_CNT) {
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
	adc_gpio_cfg_t adc_cfg = {ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PB2};	//默认 PB2
    if(pin == GPIO_PB0)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PB0};
    }
    else if(pin == GPIO_PB1)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PB1};
    }
    else if(pin == GPIO_PB2)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PB2};
    }
    else if(pin == GPIO_PA4)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PA4};
    }
    else if(pin == GPIO_PA5)
    {
    	adc_cfg = (adc_gpio_cfg_t){ADC_VREF_1P2V, ADC_PRESCALE_1F4, ADC_SAMPLE_FREQ_96K, ADC_GPIO_PA5};
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
		adc_init(NDMA_M_CHN);
		adc_vbat_sample_init(ADC_M_CHANNEL);
	}
	else if(mode == ADC_GPIO_MODE)
	{
        adc_init(NDMA_M_CHN);
        ADC_GPIOpin_sample_init(pin);
	}
	adc_power_on();
	sleep_us(20);                          //通道/Vref 切换稳定时间
}

//*************************************************************************
//*************************************************************************
unsigned short Get_ADC_mV_val(void)		//采样16次Adc并求平均值并换算为mV值
{
	w_AD_Voltage = UserAdc_get_result(NDMA, ADC_M_CHANNEL);		//adc_sample_and_get_result();
	w_AD_Val = Adc_average_val;
	return w_AD_Voltage;
}

unsigned short Get_ADC_val(void)		//采样16次Adc并求平均值
{
	w_AD_Voltage = UserAdc_get_result(NDMA, ADC_M_CHANNEL);		//adc_sample_and_get_result();
	w_AD_Val = Adc_average_val;
	return w_AD_Val;
}

//**************************************************************************
//3V干电池供电
//*************************************************************************
//*************************************************************************
#define	LowBatOffThresh 	2100		//2.1V
#define cLowBatOffCntSet	300		//3S
#define	LowBatWarnThresh 	2400		//2.4V	(低压报警阈值, 3V干电池, 可按需调整)
#define	LowBatSlowThresh 	2200		//2.2V	(低压降速阈值, 3V干电池, 可按需调整)

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
#define VbatDectMode	VbatDectByVdd		//选择使用IC_ VDD或是I/O Pin采样电源电压
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
//*************************************************************************
void Bat_adc_Sampling(void)		//for Battery Voltage samples
{
	//设置采样通道
	adc_user_init(ADC_GPIO_MODE, ADC_VB_Pin);
	sleep_us(20);
	Get_ADC_mV_val();
	w_VbatData = w_AD_Voltage;
}

//*************************************************************************
#endif		//VbatDectByPin
//*************************************************************************
void GetBatteryVoltage(void)	//运行一次用时280uS
{
	Bat_adc_Sampling();		//for Battery Voltage samples

	//检查低电压报警
	if(w_VbatData < LowBatWarnThresh)
	{
		BatteryLowCnt++;
		if(BatteryLowCnt >= 180)
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
		if(BatLowerCnt >= 180)
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
	if(IS_BIT_SET(VddVolFlag, LowBatSlowFlag) && (IS_BIT_SET(VddVolFlag, LowBatOffFlag)==0))
	{
		//慢速行走最长时间限制
		u16BatOffTmCnt++;
		if(u16BatOffTmCnt > cSlowRunTmMax)
		{
			//LowBatOffFlag=1;
			SET_BIT(VddVolFlag, LowBatOffFlag);
		}
	}
	else
	{
		u16BatOffTmCnt=0;
	}

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

//*******************************************************************************
// VR 电源控制
//*******************************************************************************
void VR_PowerOn(void)
{
	gpio_init_out_High(VR_POWER_PIN);	//VCC_RP (采样时拉高, 闲置时拉低省电)
	//spec: "ADC 采样前先让 IC_PA6 输出高电平, 用于对该电位器供电"
	sleep_us(20);                          //电源 + 电位器稳定时间
}

void VR_PowerOff(void)
{
	gpio_init_out_low(VR_POWER_PIN);	//VCC_RP (采样时拉高, 闲置时拉低省电)
	//spec: "ADC 采样结束后可以让 IC_PA6 输出低电平以节省耗电"
}

//*******************************************************************************
//#define Throttle_Pin     GPIO_PB2   //Throttle（油门）,油门为左摇杆上下移动杆：管控本飞行器的上下移动速度
//#define Rotation_Pin     GPIO_PB1   //
//#define FBmoveAdc_Pin    GPIO_PA4   //前后平移
//#define LRmoveAdc_Pin    GPIO_PA5   //左右平移
//*******************************************************************************
unsigned short	Throttle_AdcData;		//adc Speed sample value, 油门
unsigned short	Rotation_AdcData;		//adc Direction sample value, 方向盘
unsigned short	FBmove_AdcDat;		//前后平移
unsigned short	LRmove_AdcDat;		//左右平移

unsigned short	Throttle_midPoint=0x1000;		//Remotor Throttle middle point value
unsigned short	Rotation_midPoint=0x1000;		//Remotor Rotation middle point value
unsigned short	FBmove_midPoint=0x1000;		//Remotor FBmove middle point value
unsigned short	LRmove_midPoint=0x1000;		//Remotor LRmove middle point value

unsigned short	Throttle_SendData;		//Remotor speed data value for send
unsigned short	Rotation_SendData;		//Remotor rotation data value for send
unsigned short	FBmove_SendData;		//Remotor FBmove data value for send
unsigned short	LRmove_SendData;		//Remotor LRmove data value for send

#define	c_rfdata_min	16		//8192分为512级
//*******************************************************************************
//*******************************************************************************
unsigned short VR_SampleRawPublic(void)
{
	unsigned int u16temper;
	u16temper  = ((Get_ADC_mV_val()*8192)/w_VbatData);		//油门
	if(u16temper > 8191)
	{
		u16temper = 8191;
	}
	return u16temper;
}

unsigned short AdcDataUpdate(unsigned short OldAdcData, unsigned short NewAdcData)
{
	if(OldAdcData > NewAdcData)
	{
		if((OldAdcData - NewAdcData) < c_rfdata_min)
		{
			NewAdcData = OldAdcData;
		}
	}
	else
	{
		if((NewAdcData - OldAdcData) < c_rfdata_min)
		{
			NewAdcData = OldAdcData;
		}
	}
	return NewAdcData;
}

//*******************************************************************************
//*******************************************************************************
// 4次采样 (含电源开关)
//*******************************************************************************
void VR_SampleRaw(void)
{
	//gpio_write(Testing_Pin, 1);
	VR_PowerOn();

	unsigned short u16temper0;
	adc_user_init(ADC_GPIO_MODE, Throttle_Pin);
	u16temper0 = VR_SampleRawPublic();
	PushRfDatToFIFO(ThrottleAdcFIFO, u16temper0);
	u16temper0 = SortingServoAdcData(ThrottleAdcFIFO);	//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
	Throttle_AdcData = AdcDataUpdate(Throttle_AdcData, u16temper0);

	adc_user_init(ADC_GPIO_MODE, Rotation_Pin);
	u16temper0 = VR_SampleRawPublic();
	PushRfDatToFIFO(RotationAdcFIFO, u16temper0);
	u16temper0 = SortingServoAdcData(RotationAdcFIFO);	//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
	Rotation_AdcData = AdcDataUpdate(Rotation_AdcData, u16temper0);

	adc_user_init(ADC_GPIO_MODE, FBmoveAdc_Pin);
	u16temper0 = VR_SampleRawPublic();
	PushRfDatToFIFO(FBmoveAdcFIFO, u16temper0);
	u16temper0 = SortingServoAdcData(FBmoveAdcFIFO);	//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
	FBmove_AdcDat = AdcDataUpdate(FBmove_AdcDat, u16temper0);

	adc_user_init(ADC_GPIO_MODE, LRmoveAdc_Pin);
	u16temper0 = VR_SampleRawPublic();
	PushRfDatToFIFO(LRmoveAdcFIFO, u16temper0);
	u16temper0 = SortingServoAdcData(LRmoveAdcFIFO);	//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值
	LRmove_AdcDat = AdcDataUpdate(LRmove_AdcDat, u16temper0);

	//VR_PowerOff();
	//gpio_write(Testing_Pin, 0);
}

//*******************************************************************************************************
//*******************************************************************************************************
unsigned short AdcMidDataUpdate(unsigned short NewAdcData)
{
	unsigned short u16temper0;

	if(NewAdcData >= RfDataMiddle)
	{
		if((NewAdcData-RfDataMiddle)<0x380)
		{
			u16temper0=NewAdcData;		//RfDataMiddle;
		}
		else
		{
			u16temper0=RfDataMiddle;
		}
	}
	else
	{
		if((RfDataMiddle-NewAdcData)<0x380)
		{
			u16temper0=NewAdcData;		//RfDataMiddle;
		}
		else
		{
			u16temper0=RfDataMiddle;
		}
	}
	return u16temper0;
}

//*******************************************************************************************************
// 规定回中调整范围是1/4，所以方向中点数据是在0x1000基础上加LRmove_AdcDat；
//超过0x200偏差的认为是认为误操作上电的，使用RfDataMiddle为中点值。
//*******************************************************************************************************
void PowerOn_Get_Middle(void)			// Get middle value from adc at power on
{
	VR_SampleRaw();
	VR_SampleRaw();
	VR_SampleRaw();
	VR_SampleRaw();
	VR_SampleRaw();
	VR_SampleRaw();

	Throttle_midPoint = AdcMidDataUpdate(Throttle_AdcData);
	Rotation_midPoint = AdcMidDataUpdate(Rotation_AdcData);
	FBmove_midPoint = AdcMidDataUpdate(FBmove_AdcDat);
	LRmove_midPoint = AdcMidDataUpdate(LRmove_AdcDat);

}

//*******************************************************************************************************
//*******************************************************************************************************
unsigned short AdcDataConvert(unsigned short u16AdcData, unsigned short u16AdcMid,
		unsigned char u8Fmultiply, unsigned char u8FDivide, unsigned char u8Bmultiply, unsigned char u8BDivide)
{
	unsigned short u16temper0;
//	unsigned int u16temper1;

	if(u16AdcData >= u16AdcMid)		//前进
	{
		u16temper0 = u16AdcData - u16AdcMid;
		if(u16temper0 > RfDataMiddle)	u16temper0 = RfDataMiddle;
		if(u16temper0 > MiddleAreaOffset)
		{
			u16temper0 -= MiddleAreaOffset;
			//u16temper1 = u8Fmultiply*u16temper0;
			//u16temper0 = u16temper1/u8FDivide;		//扩大了u8Fmultiply/u8FDivide倍，需要扩大的倍速取决于其电位器的有效动态范围
			u16temper0 += RfDataMiddle;
		}
		else
		{
			u16temper0 = RfDataMiddle;
		}

		if(u16temper0 > RfDataMaxValue)
		{
			u16temper0 = RfDataMaxValue;
		}

	}
	else								//后退
	{
		u16temper0 = u16AdcMid - u16AdcData;
		if(u16temper0 > RfDataMiddle)	u16temper0 = RfDataMiddle;
		if(u16temper0 > MiddleAreaOffset)
		{
			u16temper0 -= MiddleAreaOffset;
			//u16temper1 = u8Bmultiply*u16temper0;
			//u16temper0 = u16temper1/u8BDivide;		//扩大了u8Bmultiply/u8BDivide倍，需要扩大的倍速取决于其电位器的有效动态范围
			u16temper0 = RfDataMiddle-u16temper0;
		}
		else
		{
			u16temper0 = RfDataMiddle;
		}

		if(u16temper0 < RfDataMinValue)
		{
			u16temper0 = RfDataMinValue;
		}

	}

	return u16temper0;		//RfDataMiddle;		//前进数值大于0x1000, 后退数值小于0x1000
}

//*******************************************************************************************************
//方向电位器：VDD=3.23V; 电位器有效调节范围： 3.23V	~	1.64V	~	56mV
//速度电位器：VDD=3.23V; 电位器有效调节范围： 3.23V	~	1.64V	~	500mV
//*******************************************************************************************************
#define cThrottleFmultiply		1
#define cThrottleFdivide		1
#define cThrottleBmultiply		1
#define cThrottleBdivide		1

#define cRotationFmultiply		1
#define cRotationFdivide		1
#define cRotationBmultiply		1
#define cRotationBdivide		1

#define cFBmoveFmultiply		1
#define cFBmoveFdivide			1
#define cFBmoveBmultiply		1
#define cFBmoveBdivide			1

#define cLRmoveFmultiply		1
#define cLRmoveFdivide			1
#define cLRmoveBmultiply		1
#define cLRmoveBdivide			1

//*******************************************************************************************************
// function: to handle direction and speed adc sample data
// 电位器控制范围划分: 中间死区1/10; 正向有效和反向有效区域3/10; 其余3/10作为无效区域
// 定义：上电时操作杆必须处于释放状态，上电时所采样出来的ADC值作为中点值，再在其上增加1/10范围作为中点死区;
// 数值范围: Adc Samples is 0x0000 ~ 0x1fff(绝对中点0x1000)
// 中点值: 0x1000,将原UserCh1/2_midPoint +/- 0x180范围内的值都作为0x1000
// 单边有效范围: 0x0E00
// 输出有效值范围: 0x0200~0x1000~0x1E00;但加入回中校正数据后有效范围变到:0x0000~0x1fff
// 输入:Throttle_AdcData/Rotation_AdcData/FBmove_AdcData/LRmove_AdcData
// 输出:Rotation_SendData/Throttle_SendData/FBmove_SendData/LRmove_SendData
//*******************************************************************************************************
//#define MiddleAreaOffset	0x180
//#define RfDataMaxValue	0x1C00
//#define RfDataMinValue	0x0400
//#define RfDataMiddle	0x1000
//*******************************************************************************************************
void HandleRemoterAdcData(void)
{
	VR_SampleRaw();

	Throttle_SendData = AdcDataConvert(Throttle_AdcData, Throttle_midPoint,
			cThrottleFmultiply, cThrottleFdivide, cThrottleBmultiply, cThrottleBdivide);	//前进数值大于0x1000, 后退数值小于0x1000

	Rotation_SendData = AdcDataConvert(Rotation_AdcData, Rotation_midPoint,
			cRotationFmultiply, cRotationFdivide, cRotationBmultiply, cRotationBdivide);	//前进数值大于0x1000, 后退数值小于0x1000

	FBmove_SendData = AdcDataConvert(FBmove_AdcDat, FBmove_midPoint,
			cFBmoveFmultiply, cFBmoveFdivide, cFBmoveBmultiply, cFBmoveBdivide);	//前进数值大于0x1000, 后退数值小于0x1000

	LRmove_SendData = AdcDataConvert(LRmove_AdcDat, LRmove_midPoint,
			cLRmoveFmultiply, cLRmoveFdivide, cLRmoveBmultiply, cLRmoveBdivide);	//前进数值大于0x1000, 后退数值小于0x1000
}

//******************************************************************************
//#define cMedian_len  5  			//中值滤波的滤波长度,一般取奇数
//#define cMedian_mid  2  			//中值在滤波数组中的位置
//******************************************************************************
unsigned short 	ThrottleAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, ThrottleAdcFIFO[cMedian_len]是压入数量
unsigned short 	RotationAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, RotationAdcFIFO[cMedian_len]是压入数量
unsigned short 	FBmoveAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, FBmoveAdcFIFO[cMedian_len]是压入数量
unsigned short 	LRmoveAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, LRmoveAdcFIFO[cMedian_len]是压入数量
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

//*******************************************************************************
#endif		//#if ADC_ENABLE
//*******************************************************************************
