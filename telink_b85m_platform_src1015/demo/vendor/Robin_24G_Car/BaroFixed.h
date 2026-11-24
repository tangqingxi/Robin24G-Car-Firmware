#ifndef BARO_FIXED_H
#define BARO_FIXED_H

#include <types.h>

// 温补出厂系数：按厂商多项式保存，定点计算在 BaroFixed.c。
typedef struct {
	s32 C00;
	s32 C10;
	s32 C01;
	s32 C20;
	s32 C02;
	s32 C30;
	s32 C11;
	s32 C12;
	s32 C21;
	s32 KP;
	u8 valid;
} BaroFixedCal_t;

// 温补结果标记：bit0 有效，bit1 温度缺失/超时，bit2 校准无效。
#define BARO_FIXED_OK          0x01
#define BARO_FIXED_NO_TEMP     0x02
#define BARO_FIXED_BAD_CAL     0x04

// 独立计算入口：输入原始气压、原始温度，输出 0.1Pa；返回状态位。
u8 BaroFixed_Compensate01Pa(s32 p_raw, s32 t_raw,
							const BaroFixedCal_t *cal, s32 *out);

// 上电读取一次出厂系数；返回 0 成功。
u8 BaroFixed_LoadCalibration(BaroFixedCal_t *cal);
u8 BaroFixed_LoadCalibrationBlock(const u8 *block, u8 pressure_config,
								  BaroFixedCal_t *cal);

#ifdef BARO_BDT_HOST_TEST
void BaroFixed_TestSetV4Cal(BaroFixedCal_t *cal);
#endif

#endif /* BARO_FIXED_H */
