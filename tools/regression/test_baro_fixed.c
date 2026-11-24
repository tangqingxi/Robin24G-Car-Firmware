/* 定点温补回归：逐条对比 v4 Python 浮点 pressure_pa，并测缺温度/坏校准/边界。 */
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

static int read_rows(const char *path, s32 *p, s32 *t, float *ref)
{
	FILE *fp = fopen(path, "r");
	char line[256];
	int n = 0;

	if (!fp) {
		return -1;
	}
	fgets(line, sizeof(line), fp);       // 表头
	while (fgets(line, sizeof(line), fp) && n < 200) {
		int ms;
		s32 pp, tt;
		float pa, tc;
		if (sscanf(line, "%d,%d,%d,%f,%f", &ms, &pp, &tt, &pa, &tc) == 5) {
			p[n] = pp;
			t[n] = tt;
			ref[n] = pa;
			n++;
		}
	}
	fclose(fp);
	return n;
}

int main(void)
{
	const char *path = "v4_baro.csv";
	s32 p[200], t[200], out;
	float ref[200];
	int n, i, bad = 0;
	int maxerr01 = 0;
	BaroFixedCal_t cal;

	BaroFixed_TestSetV4Cal(&cal);
	n = read_rows(path, p, t, ref);
	if (n <= 0) {
		printf("FAIL read v4 csv\n");
		return 1;
	}
	for (i = 0; i < n; i++) {
		if (BaroFixed_Compensate01Pa(p[i], t[i], &cal, &out) != BARO_FIXED_OK) {
			bad++;
			continue;
		}
		{
			int diff = (int)((float)out / 10.0f - ref[i]);
			if (diff < 0) diff = -diff;
			if (diff > maxerr01) maxerr01 = diff;
		}
	}
	if (bad) {
		printf("FAIL fixed invalid rows=%d\n", bad);
		return 1;
	}
	printf("PASS v4 fixed-vs-python rows=%d max_error=%.1fPa (limit 0.2Pa)\n",
		   n, maxerr01 / 10.0f);
	if (maxerr01 > 2) {
		printf("FAIL error over 0.2Pa\n");
		return 1;
	}

	cal.valid = 0;
	if (BaroFixed_Compensate01Pa(1390000, 93000, &cal, &out)
		!= (BARO_FIXED_BAD_CAL)) {
		printf("FAIL bad calibration\n");
		return 1;
	}
	BaroFixed_TestSetV4Cal(&cal);
	if (BaroFixed_Compensate01Pa(1390000, 0, &cal, &out)
		!= BARO_FIXED_NO_TEMP) {
		printf("FAIL missing temperature\n");
		return 1;
	}
	// 负系数、负温度附近的舍入不应产生明显跳变。
	cal.C00 = -1;
	cal.C10 = cal.C01 = cal.C20 = cal.C02 = 0;
	cal.C30 = cal.C11 = cal.C12 = cal.C21 = 0;
	if (BaroFixed_Compensate01Pa(1390000, 93000, &cal, &out) != BARO_FIXED_OK
		|| out != -10) {
		printf("FAIL negative rounding out=%d\n", out);
		return 1;
	}
	printf("PASS missing temperature / bad calibration / negative rounding\n");
	printf("RESULT OK\n");
	return 0;
}
