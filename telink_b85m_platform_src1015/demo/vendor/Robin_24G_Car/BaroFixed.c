/*
 * LPC2623 气压温补定点实现（独立文件，后续可替换为浮点版本）。
 * 公式按厂商规格书：
 *   x = PRAW / KP
 *   t = TRAW / 1048576
 *   s = 8.5*t / (1 + 8.8*t)
 *   P = C00 + C10*x + C01*s + C20*x^2 + C02*s^2 + C30*x^3
 *       + C11*x*s + C12*x*s^2 + C21*x^2*s
 * 中间保留 Q24 小数，最后再舍入为 0.1 Pa。
 */
#include "BaroFixed.h"
#include "PressSen_Lpc2623.h"

#define Q24 16777216LL
#define RAW_MIN (-8388608)
#define RAW_MAX 8388607

static long long round_q24(long long value)
{
	return value >= 0 ? (value + Q24 / 2) / Q24
					  : -((-value + Q24 / 2) / Q24);
}

// 每项只做 16 位乘法，保留完整的 32×32 位乘积。
static unsigned long long mul_u32(u32 a, u32 b)
{
	u32 a_low = a & 0xFFFFu, a_high = a >> 16;
	u32 b_low = b & 0xFFFFu, b_high = b >> 16;

	return (unsigned long long)(a_low * b_low)
		+ ((unsigned long long)(a_high * b_low) << 16)
		+ ((unsigned long long)(a_low * b_high) << 16)
		+ ((unsigned long long)(a_high * b_high) << 32);
}

// TC32 SDK 的 __muldi3 只有 32 位参数，不能用于本模块的宽乘法。
static long long mul_wide(long long value, s32 factor)
{
	unsigned long long magnitude = (unsigned long long)(value < 0 ? -value : value);
	u32 multiplier = (u32)(factor < 0 ? -factor : factor);
	unsigned long long result = mul_u32((u32)magnitude, multiplier)
		+ (mul_u32((u32)(magnitude >> 32), multiplier) << 32);

	return (value < 0) != (factor < 0) ? -(long long)result : (long long)result;
}

// 先分离整数和小数部分，避免两个放大后的量直接相乘溢出。
static long long mul_q24(long long value, s32 factor)
{
	return mul_wide(value / Q24, factor) + round_q24(mul_wide(value % Q24, factor));
}

// 用余数逐位求小数，TC32 不必链接 64 位除法库。
static u8 ratio_q24(s32 numerator, s32 denominator, s32 *out)
{
	u32 divisor, magnitude, whole, remainder, fraction = 0;
	u8 bit;

	if (denominator == 0) return 1;
	divisor = (u32)(denominator < 0 ? -denominator : denominator);
	magnitude = (u32)(numerator < 0 ? -numerator : numerator);
	whole = magnitude / divisor;
	if (whole >= 128u) return 1;
	remainder = magnitude - whole * divisor;
	for (bit = 0; bit < 24; bit++) {
		remainder <<= 1;
		fraction <<= 1;
		if (remainder >= divisor) {
			remainder -= divisor;
			fraction |= 1u;
		}
	}
	magnitude = (whole << 24) + fraction + ((remainder << 1) >= divisor);
	if (magnitude > 0x7FFFFFFFu) return 1;
	*out = (numerator < 0) != (denominator < 0) ? -(s32)magnitude : (s32)magnitude;
	return 0;
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
	return BaroFixed_LoadCalibrationBlock(block, psr_cfg, cal);
}

u8 BaroFixed_LoadCalibrationBlock(const u8 *block, u8 pressure_config,
								  BaroFixedCal_t *cal)
{
	static const s32 scales[8] = {
		524288, 1572864, 3670016, 7864320,
		253952, 516096, 1040384, 2088960
	};
	u8 idx = pressure_config & 0x07;
	u8 i, any_set = 0, any_clear = 0;

	if (!cal) return 1;
	cal->valid = 0;
	if (!block || idx >= sizeof(scales) / sizeof(scales[0])) return 1;
	for (i = 6; i < LPC2623_CALIB_BYTES; i++) {
		any_set |= block[i];
		any_clear |= (u8)~block[i];
	}
	if (!any_set || !any_clear) return 1;
	parse_coeff(&block[6], cal);  // 前六字节不参与压力系数解析。
	cal->KP = scales[idx];
	cal->valid = 1;
	return 0;
}

u8 BaroFixed_Compensate01Pa(s32 p_raw, s32 t_raw,
							const BaroFixedCal_t *cal, s32 *out)
{
	s32 x, s;
	long long p0, p1, p2, result;

	if (!cal || !cal->valid || cal->KP <= 0) {
		return BARO_FIXED_BAD_CAL;
	}
	if (t_raw == 0 || t_raw < RAW_MIN || t_raw > RAW_MAX) {
		return BARO_FIXED_NO_TEMP;
	}
	// 驱动为兼容旧记录保存了取反后的值，在有符号域还原。
	if (p_raw < RAW_MIN || p_raw > RAW_MAX || ratio_q24(~p_raw, cal->KP, &x)) {
		return BARO_FIXED_BAD_CAL;
	}
	// 直接整理有理式，保留温度计数的全部位。
	if (ratio_q24(85 * t_raw, 10 * 1048576 + 88 * t_raw, &s)) {
		return BARO_FIXED_NO_TEMP;
	}
	// 按压力幂次合并同类项。系数来自规格书限定的位宽，x 和 s 均为有符号 Q24。
	p0 = cal->C00 * Q24 + mul_q24(cal->C01 * Q24 + mul_q24(cal->C02 * Q24, s), s);
	p1 = cal->C10 * Q24 + mul_q24(cal->C11 * Q24 + mul_q24(cal->C12 * Q24, s), s);
	p2 = cal->C20 * Q24 + mul_q24(cal->C21 * Q24, s);
	result = p0 + mul_q24(p1 + mul_q24(p2 + mul_q24(cal->C30 * Q24, x), x), x);
	result = (result / Q24) * 10 + round_q24((result % Q24) * 10);
	if (result < (-2147483647LL - 1) || result > 2147483647LL) {
		return BARO_FIXED_BAD_CAL;
	}
	if (out) {
		*out = (s32)result;
	}
	return BARO_FIXED_OK;
}

#ifdef BARO_BDT_HOST_TEST
void BaroFixed_TestSetV4Cal(BaroFixedCal_t *cal)
{
	cal->C00 = 67542;
	cal->C10 = -77141;
	cal->C01 = 74959;
	cal->C02 = -66612;
	cal->C20 = -4532;
	cal->C30 = -313;
	cal->C11 = -13873;
	cal->C12 = 5716;
	cal->C21 = -594;
	cal->KP = 7864320;
	cal->valid = 1;
}
#endif