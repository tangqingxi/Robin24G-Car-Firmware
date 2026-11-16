//*******************************************************************
/*
 * UserAdc.h
 *
 *  Created on: 2026年10月24日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USERADC_H_
#define VENDOR_ROBIN_24G_CAR_USERADC_H_
//*******************************************************************
//*******************************************************************

#define User_ADC_ENABLE     1

//*******************************************************************
//*******************************************************************
//通道			功能					范围				阈值
//LPDadc		低电压检测，ADC2		0~4.2V/3		1V
//Isen1			大电流检测1，ADC1	0~4.2V/2		300mV
//Isen2			大电流检测2，ADC0	0~4.2V/2		300mV
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
//*******************************************************************
#if	User_ADC_ENABLE
//*******************************************************************
//---------- 13bit 语义 (TC122X 9bit ×16 还原) ----------
//*******************************************************************
//for ADC
extern unsigned short w_AD_Val;
extern unsigned short w_AD_Voltage;
extern unsigned short w_VbatData;				//ADC采样的电池电压保存值
extern unsigned char VddVolFlag;
#define  LowBatWarnFlag		0x04
#define  LowBatSlowFlag		0x02
#define  LowBatOffFlag		0x01
#define  OneSamGenFlag		0x80		//允许一次采样产生低电压标志


//*******************************************************************
//---------- API ----------
//*******************************************************************
void GetBatteryVoltage(void);	//运行一次用时280uS
void OverCurrentHandler(void);	//10mS loop
void LargeCurrentProtect(void);
void ChkLargeCurrentProtExit(void);

//*******************************************************************
#endif		//#if	User_ADC_ENABLE
//*******************************************************************




//*******************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USERADC_H_ */
//*******************************************************************

