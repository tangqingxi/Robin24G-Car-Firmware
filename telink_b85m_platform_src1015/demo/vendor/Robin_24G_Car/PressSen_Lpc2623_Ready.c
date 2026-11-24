/*
 * LPC2623 数据就绪判断（驱动层，供气压诊断和主机回归共用）。
 * 读取 MEAS_CFG(0x08) 的 bit4(0x10)：置位才表示压力样本可读。
 */
#include "types.h"
#include "PressSen_Lpc2623.h"

// 气压实现在 I2C2 总线上；由 User_IIC.c 提供。
u8 i2c_read_reg_s2(u8 reg, u8 *value);

u8 LPC2623_ReadReady(u8 *bits)
{
	return i2c_read_reg_s2(REG_MEAS_CFG, bits);
}

u8 LPC2623_IsPressureReady(void)
{
	u8 cfg;

	if (LPC2623_ReadReady(&cfg)) {
		return 0;
	}
	return (cfg & LPC2623_PRESSURE_READY) ? 1 : 0;
}

// 温度数据就绪：MEAS_CFG bit5（0x20）。
u8 LPC2623_IsTempReady(void)
{
	u8 cfg;

	if (LPC2623_ReadReady(&cfg)) {
		return 0;
	}
	return (cfg & LPC2623_TEMP_READY) ? 1 : 0;
}
