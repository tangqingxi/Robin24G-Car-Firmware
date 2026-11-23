/* 偏航通道向零截断回归：等幅正负输入应得到等幅反号 PWM 差动。 */
#include <stdio.h>
#include <stdlib.h>
#include "RateControl.h"
#include "MotorMixer.h"

static int fail;
static void expect(int got, int want, const char *what)
{
	if (got != want) { printf("FAIL %s got=%d want=%d\n", what, got, want); fail = 1; }
}

static int yaw_diff(const MotorMixerOutput *o)
{
	return (int)o->left + (int)o->right - (int)o->front - (int)o->rear;
}

int main(void)
{
	RateControl rc;
	int tgt[3], gyro[3] = {0, 0, 0};
	MotorMixerOutput pos, neg, ref;

	RateControl_Init(&rc);
	tgt[0] = tgt[1] = 0;
	tgt[2] = 1;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], 2, "yaw +1 P=2");
	tgt[2] = -1;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], -2, "yaw -1 P=2");
	tgt[2] = 7;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], 14, "yaw +7 P=2");
	tgt[2] = -7;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], -14, "yaw -7 P=2");

	/* 未饱和区：±误差输出等幅反号，并匹配 P=2.0 的向零截断。 */
	{
		int e;
		for (e = 1; e <= 120; e++) {
			int want = (e * 512) >> 8;
			tgt[2] = e;
			RateControl_Update(&rc, tgt, gyro);
			expect(rc.corr[2], want, "yaw +err P2");
			tgt[2] = -e;
			RateControl_Update(&rc, tgt, gyro);
			expect(rc.corr[2], -want, "yaw -err P2");
		}
	}

	/* target/gyro 到限幅边界，err=8192，乘法和 corr 存储不能溢出。 */
	tgt[2] = 4096;
	gyro[2] = -4096;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], 4096, "yaw +max saturates");
	tgt[2] = -4096;
	gyro[2] = 4096;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[2], -4096, "yaw -max saturates");
	gyro[2] = 0;

	/* 横滚/俯仰仍为 P=0.25。 */
	tgt[0] = 7;
	tgt[1] = -7;
	tgt[2] = 0;
	gyro[0] = gyro[1] = gyro[2] = 0;
	RateControl_Update(&rc, tgt, gyro);
	expect(rc.corr[0], 1, "roll +7 P0.25");
	expect(rc.corr[1], -2, "pitch -7 P0.25");

	MotorMixer_Mix(190, 0, 0, 1, &pos);
	MotorMixer_Mix(190, 0, 0, -1, &neg);
	expect(yaw_diff(&pos), 0, "small +yaw diff");
	expect(yaw_diff(&neg), 0, "small -yaw diff");

	MotorMixer_Mix(190, 0, 0, 35, &pos);
	MotorMixer_Mix(190, 0, 0, -35, &neg);
	expect(yaw_diff(&pos), 4, "+35 yaw diff");
	expect(yaw_diff(&neg), -4, "-35 yaw diff");

	MotorMixer_Mix(340, 0, 0, 4096, &pos);
	MotorMixer_Mix(340, 0, 0, -4096, &neg);
	expect(yaw_diff(&pos), 240, "limited +yaw diff");
	expect(yaw_diff(&neg), -240, "limited -yaw diff");
	expect(pos.limited && neg.limited, 1, "yaw limit flag");

	MotorMixer_Mix(190, 100, 0, 0, &pos);
	expect(pos.front, 190, "roll front");
	expect(pos.rear, 190, "roll rear");
	expect(pos.left, 194, "roll left");
	expect(pos.right, 186, "roll right");
	MotorMixer_Mix(190, 0, 100, 0, &pos);
	expect(pos.front, 193, "pitch front");
	expect(pos.rear, 187, "pitch rear");
	expect(pos.left, 190, "pitch left");
	expect(pos.right, 190, "pitch right");
	(void)ref;

	if (fail) return 1;
	printf("PASS yaw symmetric truncation, limit and roll/pitch unchanged\n");
	printf("RESULT OK\n");
	return 0;
}
