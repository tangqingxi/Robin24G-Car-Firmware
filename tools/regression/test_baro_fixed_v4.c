/* 用实际 v4 Read.bin 的出厂系数和原始值，输出 C 定点 0.1Pa 结果。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "BaroFixed.h"
#include "PressSen_Lpc2623.h"

u8 LPC2623_ReadCalibration(u8 block[LPC2623_CALIB_BYTES], u8 *cfg)
{
	(void)block; (void)cfg;
	return 1;
}

int main(void)
{
	FILE *fp = fopen("v4_actual.bin", "rb");
	static u8 d[4096];
	BaroFixedCal_t cal;
	int n, i;

	if (!fp) {
		printf("FAIL open v4_actual.bin\n");
		return 1;
	}
	n = (int)fread(d, 1, sizeof(d), fp);
	fclose(fp);
	if (n < 800 || memcmp(d, "BARO", 4) || d[4] != 4) {
		printf("FAIL v4 header\n");
		return 1;
	}
	if (BaroFixed_LoadCalibrationBlock(&d[48], d[42], &cal)) {
		printf("FAIL factory calibration\n");
		return 1;
	}
	printf("cal C00=%d C10=%d KP=%d cfg=0x%02X\n",
		   cal.C00, cal.C10, cal.KP, d[42]);
	fp = fopen("baro_v4_c.csv", "w");
	if (!fp) return 1;
	fprintf(fp, "session_ms,raw_pressure,raw_temperature,pressure_01pa\n");
	for (i = 0; i < 150; i++) {
		int po = (d[674 + i / 8] >> (i % 8)) & 1;
		int to = (d[693 + i / 8] >> (i % 8)) & 1;
		if (po && to) {
			s32 pr = (s16)(d[74 + i * 2] | (d[75 + i * 2] << 8));
			s32 tr = (s16)(d[374 + i * 2] | (d[375 + i * 2] << 8));
			s32 f;
			pr += (s32)(d[28] | (d[29] << 8) | (d[30] << 16) | (d[31] << 24));
			tr += (s32)(d[32] | (d[33] << 8) | (d[34] << 16) | (d[35] << 24));
			if (BaroFixed_Compensate01Pa(pr, tr, &cal, &f) == BARO_FIXED_OK) {
				fprintf(fp, "%d,%d,%d,%d\n", i * 1000, pr, tr, f);
			}
		}
	}
	fclose(fp);
	printf("PASS generated C fixed reference rows\n");
	return 0;
}
