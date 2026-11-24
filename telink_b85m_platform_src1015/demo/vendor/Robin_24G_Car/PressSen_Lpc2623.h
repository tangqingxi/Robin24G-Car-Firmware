//**********************************************************************************
/*
 * PressSen_Lpc2623.h
 *
 *  Created on: 2026年10月27日
 *      Author: HuangGuangjian
 */
//**********************************************************************************
//气压计驱动头文件
//**********************************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_PRESSSEN_LPC2623_H_
#define VENDOR_ROBIN_24G_CAR_PRESSSEN_LPC2623_H_
//**********************************************************************************
//**********************************************************************************
// =====================================================================================
//
//	MEAS_CFG: 0x08,
//		meas[2:0] = 0x07 → 连续压力+温度测量
//		meas[2:0] = 0x05 → 仅连续压力
//		meas[2:0] = 0x01 → 单次压力
//
//	CFG_REG: 0x09,
//		cfg & 0x02 = 1 → FIFO 已启用
//		cfg & 0x02 = 0 → FIFO 禁用
//
// -------------------------------------------------------------------------------------

/* 寄存器地址 */
#define REG_PSR_B2       0x00
#define REG_PSR_B1       0x01
#define REG_PSR_B0       0x02
#define REG_TMP_B2       0x03
#define REG_TMP_B1       0x04
#define REG_TMP_B0       0x05
#define REG_PSR_CFG      0x06
#define REG_TEMP_CFG     0x07
#define REG_MEAS_CFG     0x08
#define REG_CFG_REG      0x09
#define REG_INT_STS      0x0A
#define REG_WM_CFG       0x0B
#define REG_FIFO_STS     0x0C
#define REG_RESET        0x0D
#define REG_PROD_ID      0x1D


/* 校准系数寄存器起始地址 */
#define REG_CALIB_START  0x20
#define REG_CALIB_END    0x39


// 来自气压计公司：
// -------------------------------------------------------------------------------------
#define LPC2623   	(0x77<<1)   //IIC address
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

#define  LPC2623_ID_addr	0x1D


#define LPC2623_POW_2_11_MINUS_1    0x7FF
#define LPC2623_POW_2_12            0x1000

#define LPC2623_POW_2_13_MINUS_1    0x1FFF
#define LPC2623_POW_2_14            0x4000

#define LPC2623_POW_2_14_MINUS_1    0x3FFF
#define LPC2623_POW_2_15            0x8000

#define LPC2623_POW_2_16_MINUS_1    0xFFFF
#define LPC2623_POW_2_17            0x20000

#define LPC2623_POW_2_19_MINUS_1    0x7FFFF
#define LPC2623_POW_2_20            0x100000


// -------------------------------------------------------------------------------------
typedef  struct
{
    u8  PSR_TMP_B[6];

//	u8  PSR_CFG;		 //0x06
//	u8  TMP_CFG;         //0x07
//	u8  MEAS_CFG ;       //0x08
//	u8  CFG_REG;         //0x09
//	u8  RESET;			 //0x0D
//	u8  PROD_ID;         //0X1D

	u8  COEFF_reg[20];    //0X26-0X39

	float  pre_result;
	float  temp_result;

}LPC2623_RRE_TEMP_t;

typedef  struct
{
	s32  C00;
	s32  C01;
	s32  C02;

	s32  C10;
	s32  C11;
	s32  C12;
	s16  C20;
	s16  C21;
    s16  C30;

	s32  KP;

	s32 T_raw;
	s32 P_raw;

	///double T_raw_sc;		//masked by cjlei, 2026-09-22,
	///double P_raw_sc;		//masked by cjlei, 2026-09-22,

}LPC2623_CAL_COFF_t;

// -------------------------------------------------------------------------------------
//

extern int32_t P_raw;
extern int32_t T_raw;
extern int32_t P_raw1;

extern int32_t P_raw8;					// current value,
extern int32_t P_raw8_setting;			// target value,
extern int32_t P_raw8_offset;			// offset,

extern int32_t	s32PresValue;
extern unsigned char u8PresNewFlag;	//每完成一次采样 +1，供飞控判断"有新数据"

// -------------------------------------------------------------------------------------


// 计算数据,根据结构体中的数据自动进行计算：
void LPC2623_COFF_CAL(void);

// 气压计初始化：
u8 LPC2623_INIT(void);

// 设置气压传感器的每秒采样率和过采样率：
void LPC2623_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl);

// 采样一次温度和气压值：
void LPC2623_start_temperature_pressure(void);

// 持续采样气压和温度值：
void LPC2623_start_continuous(uint8_t mode);

// 注意：休眠模式下，气压也不能掉电。如果掉电需要重新初始化：
void LPC2623_stop(void);

// 根据参数计算气压值和温度值:
void LPC2623_calculate(void);

void LPC2623_soft_reset(void);

void Lpc2623_Init_Seting(void);

void Get_LPC2623_Value(void);

// 诊断用：只检查产品 ID，不加载校准系数；返回 0 = 通过。
u8 LPC2623_CheckId(void);

// 读取 INT_STS，判断是否已有新的压力样本；1=就绪。
u8 LPC2623_IsPressureReady(void);

// 读取一次三字节原始压力并换算为 s32PresValue；
// 返回 0 = 成功，非 0 = I2C 读取失败（不更新原始值）。
u8 LPC2623_ReadRaw(s32 *raw);
//**********************************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_PRESSSEN_LPC2623_H_ */
//**********************************************************************************

