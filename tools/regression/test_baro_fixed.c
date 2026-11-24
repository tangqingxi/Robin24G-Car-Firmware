/* 定点温补边界回归：缺温度、坏校准、符号及可表示范围。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "BaroFixed.h"
#include "PressSen_Lpc2623.h"

u8 LPC2623_ReadCalibration(u8 block[LPC2623_CALIB_BYTES], u8 *cfg)
{
	(void)block;
	(void)cfg;
	return 1;
}

int main(void)
{
	u8 block[LPC2623_CALIB_BYTES] = {0};
	BaroFixedCal_t cal;
	s32 out = 123;

	if (!BaroFixed_LoadCalibrationBlock(block, 0x03, &cal)
		|| BaroFixed_Compensate01Pa(1390000, 93000, &cal, &out) != BARO_FIXED_BAD_CAL
		|| out != 123) {
		printf("FAIL empty calibration\n");
		return 1;
	}
	memset(block, 0xFF, sizeof(block));
	if (!BaroFixed_LoadCalibrationBlock(block, 0x03, &cal)) {
		printf("FAIL erased calibration\n");
		return 1;
	}

	// 20 位 C00=-1、C10=1，其他系数为零。
	memset(block, 0, sizeof(block));
	block[6] = 0xFF;
	block[7] = 0xFF;
	block[8] = 0xF0;
	block[10] = 1;
	{
		u8 lr = BaroFixed_LoadCalibrationBlock(block, 0x03, &cal);
		printf("dbg load=%u valid=%u\n", lr, cal.valid);
		if (lr) {
			printf("FAIL pressure configuration\n");
			return 1;
		}
	}
	if (BaroFixed_Compensate01Pa(1390000, 0, &cal, &out) != BARO_FIXED_NO_TEMP
		|| BaroFixed_Compensate01Pa(1390000, -119156, &cal, &out) != BARO_FIXED_NO_TEMP
		|| BaroFixed_Compensate01Pa(8388608, 93000, &cal, &out) != BARO_FIXED_BAD_CAL
		|| out != 123) {
		printf("FAIL sample bounds or temperature\n");
		return 1;
	}
	if (BaroFixed_Compensate01Pa(26214, 93000, &cal, &out) != BARO_FIXED_OK
		|| out != -10) {
		printf("FAIL negative rounding out=%d\n", out);
		return 1;
	}
	printf("PASS calibration validity, sensor bounds and negative rounding\n");
	return 0;
}
