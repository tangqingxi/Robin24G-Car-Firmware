/* IMU 测量边界：100ms 更新、105ms 保持、回绕、读失败和恢复。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "ImuMeasure.h"

static u32 now;
static u8 fail;
static short ax = 100, ay = 8, az = 2100;

u32 clock_time(void) { return now; }
u8 t3488_read_6axis(short *a, short *g)
{
	if (fail) return 1;
	a[0] = ax; a[1] = ay; a[2] = az;
	g[0] = g[1] = g[2] = 0;
	return 0;
}
s32 Euler_GyroToQ10(s32 raw, s32 bias_q6, s32 *rem)
{
	(void)bias_q6; (void)rem;
	return raw;
}

static int expect_lpf_change(ImuMeasure *m, ImuSample *s, s32 old, int change)
{
	ImuMeasure_Update(m, s);
	if (!s->valid) {
		printf("FAIL sample invalid\n");
		return 1;
	}
	if (change ? (s->acc_mg[0] == old) : (s->acc_mg[0] != old)) {
		printf("FAIL lpf change=%d old=%d now=%d\n",
			   change, old, s->acc_mg[0]);
		return 1;
	}
	return 0;
}

int main(void)
{
	ImuMeasure m;
	ImuSample s;
	s32 old;

	ImuMeasure_Init(&m);
	now = 0;
	ax = 100;
	fail = 0;
	ImuMeasure_Update(&m, &s);
	if (!s.valid || s.dt_ticks != 0) {
		printf("FAIL first sample\n");
		return 1;
	}

	now = 100u * 24000u;
	ax = 300;
	old = s.acc_mg[0];
	if (expect_lpf_change(&m, &s, old, 1)) return 1;
	if (s.dt_ticks != 100u * 24000u) {
		printf("FAIL 100ms dt=%u\n", s.dt_ticks);
		return 1;
	}

	old = s.acc_mg[0];
	now += 105u * 24000u;
	ax = 1000;
	if (expect_lpf_change(&m, &s, old, 0)) return 1;
	if (s.dt_ticks != 105u * 24000u) {
		printf("FAIL 105ms dt=%u\n", s.dt_ticks);
		return 1;
	}
	printf("PASS 100ms updates, 105ms keeps lpf\n");

	old = s.acc_mg[0];
	now += 5u * 24000u;
	ax = 1200;
	if (expect_lpf_change(&m, &s, old, 1)) return 1;
	printf("PASS recovery next sample updates lpf\n");

	// 回绕：首拍在接近 32 位顶部，下一次有效样本跨回绕。
	ImuMeasure_Init(&m);
	now = 0xFFFFF000u;
	ax = 500;
	ImuMeasure_Update(&m, &s);
	now += 100u * 24000u;
	ax = 700;
	ImuMeasure_Update(&m, &s);
	if (!s.valid || s.dt_ticks != 100u * 24000u) {
		printf("FAIL wrap dt=%u valid=%u\n", s.dt_ticks, s.valid);
		return 1;
	}
	printf("PASS tick wrap dt\n");

	// 读失败：失败拍不更新 tick/低通，恢复后 dt 从上一有效样本计算。
	old = s.acc_mg[0];
	fail = 1;
	now += 5u * 24000u;
	ImuMeasure_Update(&m, &s);
	if (s.valid || s.acc_mg[0] != old) {
		printf("FAIL failure update\n");
		return 1;
	}
	fail = 0;
	now += 5u * 24000u;
	ImuMeasure_Update(&m, &s);
	if (!s.valid || s.dt_ticks != 10u * 24000u) {
		printf("FAIL failure recovery dt=%u\n", s.dt_ticks);
		return 1;
	}
	printf("PASS read failure and recovery\n");
	printf("RESULT OK\n");
	return 0;
}
