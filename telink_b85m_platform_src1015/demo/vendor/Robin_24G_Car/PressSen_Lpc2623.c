//**********************************************************************************
/*
 * PressSen_Lpc2623.c
 *
 *  Created on: 2026年10月27日
 *      Author: HuangGuangjian
 */
//**********************************************************************************
//**********************************************************************************
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include <UserUart.h>
#include "User_IIC.h"
#include "PressSen_Lpc2623.h"

//**********************************************************************************
//**********************************************************************************
#define i2c_write_reg		i2c_write_reg_s2	//
#define i2c_read_reg		i2c_read_reg_s2		//
#define i2c_read_regs		i2c_read_regs_s2	//


LPC2623_RRE_TEMP_t		LPC2623_CAL_PRE_TEMP;		//数据存放的结构体,
LPC2623_CAL_COFF_t    	LPC2623_CAL_COFF_RE;	    //存放COFF 计算的相关数值,

//**********************************************************************************
//海平面附近（标准大气，15℃）：海拔每升高 1 m，大气压大约下降 12 Pa（≈0.12 hPa）
//**********************************************************************************
/******************************************************************************
* @brief   	LPC2623 根据参数计算相关的系数值
* @param   	参数
*		@arg 	无
* @retval  	无
* @attention	计算校准参数值
*******************************************************************************/
void  LPC2623_COFF_CAL(void)    //计算数据,根据结构体中的数据自动进行计算
 {
	///uint8_t buf[26];  /* 0x20 ~ 0x39 共26字节 */
	///uint8_t *p = buf;

	/// 读取COFF值,
	/// if (i2c_read_regs(REG_CALIB_START, buf, 26))
	if (i2c_read_regs(0x26, &LPC2623_CAL_PRE_TEMP.COEFF_reg[0], 20)) {
		return;
	}

	// -------------------P  COEFF ??------------------------------------------------------------------
    LPC2623_CAL_COFF_RE.C00 = (LPC2623_CAL_PRE_TEMP.COEFF_reg[0] << 12) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[1] << 4) +  ((LPC2623_CAL_PRE_TEMP.COEFF_reg[2] >> 4) & 0x0F);
    if (LPC2623_CAL_COFF_RE.C00 > LPC2623_POW_2_19_MINUS_1)
        LPC2623_CAL_COFF_RE.C00 = LPC2623_CAL_COFF_RE.C00 -LPC2623_POW_2_20;

    LPC2623_CAL_COFF_RE.C10 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[2] & 0x0F) << 16) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[3] << 8) + LPC2623_CAL_PRE_TEMP.COEFF_reg[4];
    if (LPC2623_CAL_COFF_RE.C10 > LPC2623_POW_2_19_MINUS_1)
        LPC2623_CAL_COFF_RE.C10 = LPC2623_CAL_COFF_RE.C10 - LPC2623_POW_2_20;

    LPC2623_CAL_COFF_RE.C01 = (LPC2623_CAL_PRE_TEMP.COEFF_reg[5] << 12) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[6] << 4) +  ((LPC2623_CAL_PRE_TEMP.COEFF_reg[7] >> 4) & 0x0F);
    if (LPC2623_CAL_COFF_RE.C01 > LPC2623_POW_2_19_MINUS_1)
        LPC2623_CAL_COFF_RE.C01 = LPC2623_CAL_COFF_RE.C01 - LPC2623_POW_2_20;

    LPC2623_CAL_COFF_RE.C02 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[7] & 0x0F) << 16) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[8] << 8) + LPC2623_CAL_PRE_TEMP.COEFF_reg[9];
    if (LPC2623_CAL_COFF_RE.C02 > LPC2623_POW_2_19_MINUS_1)
        LPC2623_CAL_COFF_RE.C02 = LPC2623_CAL_COFF_RE.C02 - LPC2623_POW_2_20;

    LPC2623_CAL_COFF_RE.C20 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[10] & 0x7F) << 8) + LPC2623_CAL_PRE_TEMP.COEFF_reg[11];
    if (LPC2623_CAL_COFF_RE.C20 > LPC2623_POW_2_14_MINUS_1)
        LPC2623_CAL_COFF_RE.C20 = LPC2623_CAL_COFF_RE.C20 - LPC2623_POW_2_15;

    LPC2623_CAL_COFF_RE.C30 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[12] & 0x0F) << 8) + LPC2623_CAL_PRE_TEMP.COEFF_reg[13];
    if (LPC2623_CAL_COFF_RE.C30 > LPC2623_POW_2_11_MINUS_1)
        LPC2623_CAL_COFF_RE.C30 = LPC2623_CAL_COFF_RE.C30 - LPC2623_POW_2_12;

    LPC2623_CAL_COFF_RE.C11 = (LPC2623_CAL_PRE_TEMP.COEFF_reg[14] << 9) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[15] << 1) + ((LPC2623_CAL_PRE_TEMP.COEFF_reg[16] & 0x80) >> 7);
    if (LPC2623_CAL_COFF_RE.C11 > LPC2623_POW_2_16_MINUS_1)
        LPC2623_CAL_COFF_RE.C11 = LPC2623_CAL_COFF_RE.C11 - LPC2623_POW_2_17;

    LPC2623_CAL_COFF_RE.C12 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[16] & 0x7F) << 10) + (LPC2623_CAL_PRE_TEMP.COEFF_reg[17] << 2) + ((LPC2623_CAL_PRE_TEMP.COEFF_reg[18] & 0xC0) >> 6);
    if (LPC2623_CAL_COFF_RE.C12 > LPC2623_POW_2_16_MINUS_1)
        LPC2623_CAL_COFF_RE.C12 = LPC2623_CAL_COFF_RE.C12 - LPC2623_POW_2_17;

    LPC2623_CAL_COFF_RE.C21 = ((LPC2623_CAL_PRE_TEMP.COEFF_reg[18] & 0x3F) << 8) + LPC2623_CAL_PRE_TEMP.COEFF_reg[19];
    if (LPC2623_CAL_COFF_RE.C21 > LPC2623_POW_2_13_MINUS_1)
        LPC2623_CAL_COFF_RE.C21 = LPC2623_CAL_COFF_RE.C21 - LPC2623_POW_2_14;

}

/******************************************************************************
* @brief   	LPC2623 初始化，并核对ID
* @param   	参数
*		@arg 	无
* @retval  	器件是否初始化成功
* @attention	初始化器件
*******************************************************************************/
u8 LPC2623_INIT(void)
{
	uint8_t prod_id;

	if (i2c_read_reg(REG_PROD_ID, &prod_id)) {
		 return 1;
	}
	//UartPrintData((u32)prod_id);
	if ((prod_id & 0x0F) != 0x0A) {
		 return 1;	/* 产品ID不正确 */
	}

	//ID 正确 → 执行校准
	LPC2623_COFF_CAL();
	return 0;  // 初始化成功，直接退出
}


/******************************************************************************
* 函数名 :  LPC2623_rateset
* 功能描述: 设置气压传感器的每秒采样率和过采样率
* 输入参数: uint8_t u8OverSmpl  过采样率         Maximal = 128
*           uint8_t u8SmplRate 每秒采样次数(Hz) Maximal = 128
*           uint8_t iSensor     0: Pressure; 1: Temperature
*
*		   如果采样率固定之后，LPC2623_rateset，代码还可以简化。
*******************************************************************************/
void LPC2623_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
	s32  KP;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            KP = 1572864;
            break;
        case 4:
            reg |= 2;
            KP = 3670016;
            break;
        case 8:
            reg |= 3;
            KP = 7864320;
            break;
        case 16:
            KP = 253952;
            reg |= 4;
            break;
        case 32:
            KP = 516096;
            reg |= 5;
            break;
        case 64:
            KP = 1040384;
            reg |= 6;
            break;
        case 128:
            KP = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            KP = 524288;
            break;
    }

    if(iSensor == 0)//PRESSURE_SENSOR
    {
		///LPC2623_CAL_PRE_TEMP.PSR_CFG = reg;
		i2c_write_reg(0x06, reg);
		LPC2623_CAL_COFF_RE.KP = KP;
    }

    if(iSensor == 1)//TEMPERATURE_SENSOR
    {
		///LPC2623_CAL_PRE_TEMP.TMP_CFG = reg;
		i2c_write_reg(0x07, reg|0x80); //Using mems temperature
    }

}


/******************************************************************************
* Function: LPC2623_start_temperature_pressure
* Description: start one measurement for temperature and pressure
* 采样一次温度和气压值
* Input: void
* Output:
* Return: void
* Calls:
* Called By:
*******************************************************************************/
void LPC2623_start_temperature_pressure(void)
{
    i2c_write_reg(0x08, 0x03);
}


/******************************************************************************
* Function: LPC2623_start_continuous
* Description: 持续采样气压和温度值
* Input: void
* Output:
* Return: void
* Calls:
* Called By:
*******************************************************************************/
void LPC2623_start_continuous(uint8_t mode)
{
    i2c_write_reg(0x08, mode+4);
}


/******************************************************************************
* Function: LPC2623_stop
* Description: Pressure sensor Standby Mode，idle mode，sleep mode
* Input:
* Output:
* Return: void
* Calls:
* Called By:
* Noted that:The pressure sensor cannot stop supplying power
* 注意：休眠模式下，气压也不能掉电。如果掉电需要重新初始化
*******************************************************************************/
void LPC2623_stop(void)
{
	i2c_write_reg(0x08, 0);
}


int32_t P_raw;
int32_t T_raw;

int32_t P_raw8;					// current value,
int32_t P_raw8_setting;			// target value,
int32_t P_raw8_offset;			// offset,

#define C_TEST_009	0	//
#if C_TEST_009
int32_t P_raw1;
s32 P_raw32;			// max,
s32 P_raw33;			// diff,
s32 P_raw34;			// diff,
#endif
///s32 P_raw2;
/******************************************************************************
* @brief   	LPC2623 根据参数计算气压值和温度值
* @param   	参数
* @arg 		无
* @retval  	无
* @attention	计算结果
*******************************************************************************/
void LPC2623_calculate(void)
{
	//读取气压温度数值
	i2c_read_regs(0x00, &LPC2623_CAL_PRE_TEMP.PSR_TMP_B[0], 6);

	//----------------------------------------计算压力数值-----------------------------------------
	P_raw	=  (LPC2623_CAL_PRE_TEMP.PSR_TMP_B[0]<<16)|(LPC2623_CAL_PRE_TEMP.PSR_TMP_B[1]<<8)|(LPC2623_CAL_PRE_TEMP.PSR_TMP_B[2]);
	P_raw	=  (P_raw&0x800000) ? (0xFF000000|P_raw) : P_raw;

	T_raw	=  (LPC2623_CAL_PRE_TEMP.PSR_TMP_B[3]<<16)|(LPC2623_CAL_PRE_TEMP.PSR_TMP_B[4]<<8)|(LPC2623_CAL_PRE_TEMP.PSR_TMP_B[5]);
	T_raw	=  (T_raw&0x800000) ? (0xFF000000|T_raw) : T_raw;

	#if C_TEST_009
	P_raw1 = (0xFFFFFFFF - P_raw);	// add by cjlei, 2026-09-23,

	if(P_raw1 > P_raw32){
		P_raw32 = P_raw1;
	}
	if(P_raw32 > P_raw1){
		P_raw33 = P_raw32 - P_raw1;
	}
	if(P_raw33 > P_raw34){
		P_raw34 = P_raw33;
	}
	#endif

	//-----------------------计算气压最终结果------------------------
	#if 1  //
	double  Pres_SCALED 	= 	(double) P_raw		/		LPC2623_CAL_COFF_RE.KP;
	double	Temp_X	 		= 	(double) T_raw 		/ 		1048576;
	double	Temp_SCALED 	=  (8.5 * Temp_X ) / (1 + 8.8 * Temp_X );

	float	PA_CAL 			= 	LPC2623_CAL_COFF_RE.C00
							+ (LPC2623_CAL_COFF_RE.C10 * Pres_SCALED )
							+ (LPC2623_CAL_COFF_RE.C01 * Temp_SCALED )
							+ (LPC2623_CAL_COFF_RE.C20 * Pres_SCALED * Pres_SCALED )
							+ (LPC2623_CAL_COFF_RE.C02 * Temp_SCALED * Temp_SCALED )
							+ (LPC2623_CAL_COFF_RE.C30 * Pres_SCALED * Pres_SCALED * Pres_SCALED )
							+ (LPC2623_CAL_COFF_RE.C11 * Pres_SCALED * Temp_SCALED )
							+ (LPC2623_CAL_COFF_RE.C12 * Pres_SCALED * Temp_SCALED * Temp_SCALED )
							+ (LPC2623_CAL_COFF_RE.C21 * Pres_SCALED * Pres_SCALED * Temp_SCALED );

	LPC2623_CAL_PRE_TEMP.pre_result	=	PA_CAL;
	P_raw8 = PA_CAL;
	#endif

}

/*****************************************************************************
 Function: void LPC2623_soft_reset
 Description: Pressure sensor soft_reset
 Input:
 Output:
 Return: void
 Calls:
 Called By:
 Noted that:After soft reset the pressure sensor need to reinitialize
 注意：软复位之后，建议延时20ms之后，重新初始化，读取校准系数等。
*****************************************************************************/
void LPC2623_soft_reset(void)
{
	i2c_write_reg(0x0D, 0x89);
}

//**********************************************************************************
//**********************************************************************************
void Lpc2623_Init_Seting(void)
{
	LPC2623_INIT(); //读取校准参数，然后进行计算，保存
	LPC2623_rateset(PRESSURE_SENSOR,64, 8);		//Sample Rate 1/64S = 15625uS
	LPC2623_rateset(TEMPERATURE_SENSOR,1, 1);
	LPC2623_start_continuous(CONTINUOUS_PRESSURE);
	WaitMs(1);
}

//*************************************************************************
//*************************************************************************
unsigned char u8PressureVal[3];
int32_t	s32PresValue;
unsigned char u8PresNewFlag;	//每完成一次采样翻转一次，供飞控判断"有新数据"
/******************************************************************************
* @brief   	获取LPC2623 气压值和温度值
* @param   	参数
*******************************************************************************/
void Get_LPC2623_Value(void)
{
	//读取气压温度数值
	i2c_read_regs(0x00, u8PressureVal, 3);

	//----------------------------------------计算压力数值-----------------------------------------
	s32PresValue = (u8PressureVal[0]<<16)|(u8PressureVal[1]<<8)|(u8PressureVal[2]);
	s32PresValue	=  (s32PresValue & 0x800000) ? (0xFF000000|s32PresValue) : s32PresValue;
	s32PresValue = 0xFFFFFFFF - s32PresValue;
	//大约-200/CM, 平时稳定后的误差大约550~600; 也就是3CM误差;

	u8PresNewFlag++;		//通知飞控：气压数据已更新
}

// 诊断用初始化检查：只读产品 ID，不加载校准系数，省 RAM。
u8 LPC2623_CheckId(void)
{
	uint8_t prod_id;

	if (i2c_read_reg(REG_PROD_ID, &prod_id)) {
		return 1;
	}
	if ((prod_id & 0x0F) != 0x0A) {
		return 1;
	}
	return 0;
}

u8 LPC2623_ReadCalibration(u8 block[LPC2623_CALIB_BYTES], u8 *pressure_config)
{
	if (i2c_read_reg(REG_PSR_CFG, pressure_config)) {
		return 1;
	}
	return i2c_read_regs(REG_CALIB_START, block, LPC2623_CALIB_BYTES);
}

// 诊断用温度原始值：TMP_B2..B0，24 位二进制补码，不做取反。
u8 LPC2623_ReadTempRaw(s32 *raw)
{
	u8 d[3];
	s32 v;

	if (i2c_read_regs(0x03, d, 3)) {
		return 1;
	}
	v = ((s32)d[0] << 16) | ((s32)d[1] << 8) | (s32)d[2];
	if (v & 0x800000) {
		v |= (s32)0xFF000000;
	}
	if (raw) {
		*raw = v;
	}
	return 0;
}

// 诊断用带返回值的读取：保留原始计数换算，I2C 失败不覆盖上一次有效值。
u8 LPC2623_ReadRaw(s32 *raw)
{
	u8 d[3];
	s32 v;

	if (i2c_read_regs(0x00, d, 3)) {
		return 1;
	}
	v = ((s32)d[0] << 16) | ((s32)d[1] << 8) | (s32)d[2];
	if (v & 0x800000) {
		v |= (s32)0xFF000000;
	}
	v = (s32)0xFFFFFFFF - v;
	s32PresValue = v;
	u8PresNewFlag++;
	if (raw) {
		*raw = v;
	}
	return 0;
}

//**********************************************************************************
//**********************************************************************************


