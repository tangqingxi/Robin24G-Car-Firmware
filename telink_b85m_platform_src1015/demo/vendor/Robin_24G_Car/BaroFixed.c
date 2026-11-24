/*
 * LPC2623 气压温补定点实现（独立文件，后续可替换为浮点版本）。
 * 公式按厂商规格书：
 *   x = PRAW / KP
 *   t = TRAW / 1048576
 *   s = 8.5*t / (1 + 8.8*t)
 *   P = C00 + C10*x + C01*s + C20*x^2 + C02*s^2 + C30*x^3
 *       + C11*x*s + C12*x*s^2 + C21*x^2*s
 * 输出单位为 0.1 Pa，内部 Q16 + long long，避免小压力/负值截断。
 */
#include "BaroFixed.h"
#include "PressSen_Lpc2623.h"

#define Q16 65536

static long long mul_q16(long long a, long long b)
{
	if (a >= 0) {
		return (a * b + (Q16 / 2)) / Q16;
	}
	return -(((-a) * b + (Q16 / 2)) / Q16);
}

static void parse_coeff(const u8 *r, BaroFixedCal_t *c)
{
	s32 v;

	v = ((s32)r[0] << 12) | ((s32)r[1] << 4) | ((s32)r[2] >> 4);
	if (v & 0x80000) v -= 0x100000;
	c->C00 = v;
	v = (((s32)r[2] & 0x0F) << 16) | ((s32)r[3] << 8) | r[4];
	if (v & 0x80000) v -= 0x100000;
	c->C10 = v;
	v = ((s32)r[5] << 12) | ((s32)r[6] << 4) | ((s32)r[7] >> 4);
	if (v & 0x80000) v -= 0x100000;
	c->C01 = v;
	v = (((s32)r[7] & 0x0F) << 16) | ((s32)r[8] << 8) | r[9];
	if (v & 0x80000) v -= 0x100000;
	c->C02 = v;
	v = (((s32)r[10] & 0x7F) << 8) | r[11];
	if (v & 0x4000) v -= 0x8000;
	c->C20 = v;
	v = (((s32)r[12] & 0x0F) << 8) | r[13];
	if (v & 0x800) v -= 0x1000;
	c->C30 = v;
	v = ((s32)r[14] << 9) | ((s32)r[15] << 1) | (r[16] >> 7);
	if (v & 0x10000) v -= 0x20000;
	c->C11 = v;
	v = (((s32)r[16] & 0x7F) << 10) | ((s32)r[17] << 2) | (r[18] >> 6);
	if (v & 0x10000) v -= 0x20000;
	c->C12 = v;
	v = (((s32)r[18] & 0x3F) << 8) | r[19];
	if (v & 0x2000) v -= 0x4000;
	c->C21 = v;
	c->KP = 524288;             // 诊断使用 1Hz 气压采样
	c->valid = 1;
}

u8 BaroFixed_LoadCalibration(BaroFixedCal_t *cal)
{
	u8 block[LPC2623_CALIB_BYTES];
	u8 psr_cfg;

	if (!cal) {
		return 1;
	}
	cal->valid = 0;
	if (LPC2623_ReadCalibration(block, &psr_cfg)) {
		return 1;
	}
	parse_coeff(&block[6], cal);  // 0x20..0x25 不是压力系数
	(void)psr_cfg;
	return cal->valid ? 0 : 1;
}

u8 BaroFixed_Compensate01Pa(s32 p_raw, s32 t_raw,
							const BaroFixedCal_t *cal, s32 *out)
{
	long long x, t, s, num, den, v;
	long long p;

	if (!cal || !cal->valid || cal->KP <= 0) {
		return BARO_FIXED_BAD_CAL;
	}
	if (t_raw == 0) {
		return BARO_FIXED_NO_TEMP;
	}
	{
		s32 ts = t_raw >> 4;             // Q16；TRAW/2^20
		s32 num32 = (17 * ts) / 2;       // 8.5*t
		s32 den32 = Q16 + (44 * ts) / 5; // 1+8.8*t
		u32 un, ud, us;

		if (den32 == 0) {
			return BARO_FIXED_NO_TEMP;
		}
		un = (u32)(num32 < 0 ? -num32 : num32);
		ud = (u32)(den32 < 0 ? -den32 : den32);
		us = (un << 16) / ud;
		x = p_raw / (cal->KP / Q16);
		s = (num32 < 0) != (den32 < 0) ? -(long long)us : (long long)us;
		(void)t; (void)num; (void)den;
	}

	p = (long long)cal->C00 * 10;
	v = (long long)cal->C10 * 10;
	p += mul_q16(v, x);
	v = (long long)cal->C01 * 10;
	p += mul_q16(v, s);
	v = (long long)cal->C20 * 10;
	p += mul_q16(mul_q16(v, x), x);
	v = (long long)cal->C02 * 10;
	p += mul_q16(mul_q16(v, s), s);
	v = (long long)cal->C30 * 10;
	p += mul_q16(mul_q16(mul_q16(v, x), x), x);
	v = (long long)cal->C11 * 10;
	p += mul_q16(mul_q16(v, x), s);
	v = (long long)cal->C12 * 10;
	p += mul_q16(mul_q16(mul_q16(v, x), s), s);
	v = (long long)cal->C21 * 10;
	p += mul_q16(mul_q16(mul_q16(v, x), x), s);

	if (p > 2147483000LL || p < -2147483000LL) {
		return BARO_FIXED_BAD_CAL;
	}
	if (out) {
		*out = (s32)p;
	}
	return BARO_FIXED_OK;
}

#ifdef BARO_BDT_HOST_TEST
void BaroFixed_TestSetV4Cal(BaroFixedCal_t *cal)
{
	cal->C00 = -1348803;
	cal->C10 = 2400222;
	cal->C01 = 2589228;
	cal->C20 = -1313406;
	cal->C02 = -1251557;
	cal->C30 = 238348;
	cal->C11 = -3020093;
	cal->C12 = 889654;
	cal->C21 = 851085;
	cal->KP = 1048576;
	cal->valid = 1;
}
#endif
