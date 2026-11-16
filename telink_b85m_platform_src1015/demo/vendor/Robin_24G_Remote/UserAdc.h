/********************************************************************************************************
 * @file	UserAdc.h
 *
 * @brief	漂移车HX261009 Remotor 4通道电位器 ADC 采样
 *
 *******************************************************************************************************/

//*******************************************************************
#ifndef USERADC_H_
#define USERADC_H_
//*******************************************************************
//*******************************************************************

#define ADC_ENABLE     1

//*******************************************************************
//*******************************************************************
//通道				功能			范围				中心值		映射目标
//Throttle			油门			0x400~0x1C00	0x1000		目标相对升降速度		±1m/S
//Rotation			旋转			0x400~0x1C00	0x1000		目标对应偏航角速度 	±30°/S
//FBmove			前后平移		0x400~0x1C00	0x1000		目标俯仰角 ±30°
//LRmove			左右平移		0x400~0x1C00	0x1000		目标滚转角 ±30°
//*******************************************************************
//*******************************************************************
//*******************************************************************
#define SET_BIT(var, bit)    ((var) |=  (bit))
#define CLR_BIT(var, bit)    ((var) &= ~(bit))
#define TOG_BIT(var, bit)    ((var) ^=  (bit))
#define IS_BIT_SET(var, bit) (((var) &  (bit)) != 0)
//Samples:
//SET_BIT(VddVolFlag, LowBatWarnFlag);   // 置位
//CLR_BIT(VddVolFlag, LowBatWarnFlag);   // 清除
//if(IS_BIT_SET(VddVolFlag, LowBatWarnFlag)){} // 判断标志
//**********************************************************************************
//实际电位器装配电气参数方向:
//速度（油门拉）增大 ==> Adc值变大
//旋转向右 ==> 	Adc值变小
//前后平移向后 ==>	Adc值变小
//左右平移向左 ==>	Adc值变小
//**********************************************************************************
//*******************************************************************
#if	ADC_ENABLE
//*******************************************************************
//---------- 13bit 语义 (TC122X 9bit ×16 还原) ----------
#define MiddleAreaOffset	0x200
#define RfDataMaxValue	0x1C00
#define RfDataMinValue	0x0400
#define RfDataMiddle	0x1000
#define c_LimitTop		40	//20%

extern unsigned short	DirectLimitBak;		//速度限值备存，32step,其范围为8~40.


//for ADC
extern unsigned short w_AD_Val;
extern unsigned short w_AD_Voltage;
extern unsigned short w_VbatData;				//ADC采样的电池电压保存值
extern unsigned char VddVolFlag;
#define  LowBatWarnFlag		0x04
#define  LowBatSlowFlag		0x02
#define  LowBatOffFlag		0x01
#define  OneSamGenFlag		0x80		//允许一次采样产生低电压标志


extern unsigned short	Throttle_SendData;		//Remotor speed data value for send
extern unsigned short	Rotation_SendData;		//Remotor rotation data value for send
extern unsigned short	FBmove_SendData;		//Remotor FBmove data value for send
extern unsigned short	LRmove_SendData;		//Remotor LRmove data value for send

#define cMedian_len  5  			//中值滤波的滤波长度,一般取奇数
#define cMedian_mid  2  			//中值在滤波数组中的位置

extern unsigned short 	ThrottleAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, ThrottleAdcFIFO[cMedian_len]是压入数量
extern unsigned short 	RotationAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, RotationAdcFIFO[cMedian_len]是压入数量
extern unsigned short 	FBmoveAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, FBmoveAdcFIFO[cMedian_len]是压入数量
extern unsigned short 	LRmoveAdcFIFO[cMedian_len+1];   	//中值滤波的数据FIFO, LRmoveAdcFIFO[cMedian_len]是压入数量
//*******************************************************************
//---------- API ----------
//*******************************************************************
void GetBatteryVoltage(void);	//运行一次用时280uS
void    VR_PowerOn(void);                 //PA6 拉高, 给电位器供电
void    VR_PowerOff(void);                //PA6 拉低, 省电
void VR_SampleRaw(void);		// 4次采样 (含电源开关)
void PowerOn_Get_Middle(void);			// Get middle value from adc at power on
void HandleRemoterAdcData(void);

_Bool PushRfDatToFIFO(unsigned short *u16p_dat, unsigned short u16DatIn);		//每Cycle采样一次并压入FIFO，RfFIFO_Index==(cMedian_len-1)后才可用
unsigned short SortingServoAdcData(unsigned short *u16Fifo_p);	//把FIFO里的数据搬运到SortDatBuf[]中并由小到大排序并输出中值

//*******************************************************************
#endif		//#if	ADC_ENABLE
//*******************************************************************

//*******************************************************************
#endif /* USERADC_H_ */
//*******************************************************************

