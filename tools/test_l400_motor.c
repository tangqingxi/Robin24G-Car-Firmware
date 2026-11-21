/*
 * L400 电机混控与急停解锁回归（主机侧）
 *
 * 两项：
 *   1. 机头抬高（corr_pitch<0）时，限幅前后都必须 后电机 > 前电机；
 *   2. 急停键按住期间四路保持 0 且禁止解锁；松开后必须重新长按解锁键。
 *
 * 编译（在 tools 目录）：
 *   gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
 *       test_l400_motor.c -o ..\data\test_l400_motor.exe
 */

#include <stdio.h>
#include <string.h>

#include "L400Motor.h"

static int failed;

static void check(int cond, const char *msg)
{
	if (!cond) {
		printf("FAIL %s\n", msg);
		failed = 1;
	}
}

// 机头抬高：corr_pitch<0，后电机指令必须大于前电机；小信号和限幅后都成立
static void test_pitch_sign(void)
{
	L400MMix small_mix, large_mix, nose_down_mix;

	L400M_Mix(190, 0, -2048, 0, &small_mix);
	check(small_mix.rear > small_mix.front, "nose-up small: rear > front");
	check(small_mix.front >= L400_PWM_MIN_RUN
		  && small_mix.rear <= L400_PWM_MAX, "nose-up small in range");

	// 超过差动限幅：仍要保持 后 > 前，验证限幅前后符号一致
	L400M_Mix(190, 0, -8000, 0, &large_mix);
	check(large_mix.rear > large_mix.front, "nose-up limited: rear > front");

	// 反向：机头低头 corr_pitch>0，前电机应大于后电机
	L400M_Mix(190, 0, 2048, 0, &nose_down_mix);
	check(nose_down_mix.front > nose_down_mix.rear,
		  "nose-down: front > rear");

	printf("PASS pitch sign small=%u/%u limited=%u/%u\n",
		   small_mix.front, small_mix.rear,
		   large_mix.front, large_mix.rear);
}

// 急停按住：四路 0 且禁止解锁；松开后必须松开再长按解锁键
static void test_estop_hold_and_rearm(void)
{
	L400MArm arm;
	L400MMix mix = {100, 200, 300, 400};
	unsigned short out[4];
	const unsigned int hold = 1000;
	unsigned char stop;

	L400M_ArmInit(&arm);
	check(!arm.armed, "boot locked");

	// 先松开解锁键，再新按下并长按 1s
	L400M_ArmStep(&arm, 0, 0, 0, 0, 1, 1, 1, hold);
	L400M_ArmStep(&arm, 10, 1, 0, 0, 1, 1, 1, hold);
	L400M_ArmStep(&arm, 10 + hold - 1, 1, 0, 0, 1, 1, 1, hold);
	check(!arm.armed, "arm before hold");
	L400M_ArmStep(&arm, 10 + hold, 1, 0, 0, 1, 1, 1, hold);
	check(arm.armed, "arm after hold");

	L400M_Output(&arm, &mix, out);
	check(out[0] == 100 && out[3] == 400, "armed output mix");

	// 按下“速度转换”急停
	stop = L400M_ArmStep(&arm, 3000, 1, 1, 0, 1, 1, 1, hold);
	check(stop == L400_LOCK_MANUAL, "estop reason");
	check(!arm.armed, "estop locks");
	L400M_Output(&arm, &mix, out);
	check(out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 0,
		  "estop outputs zero");

	// 急停键继续按住，同时一直按住解锁键：必须保持锁定、禁止解锁
	{
		unsigned int t = 3010;
		int i;

		for (i = 0; i < 20; i++, t += hold) {
			L400M_ArmStep(&arm, t, 1, 1, 0, 1, 1, 1, hold);
			check(!arm.armed, "estop held blocks unlock");
			L400M_Output(&arm, &mix, out);
			check(out[0] == 0 && out[3] == 0, "estop held outputs zero");
		}
	}

	// 急停松开但解锁键仍按住：仍禁止解锁
	L400M_ArmStep(&arm, 40000, 1, 0, 0, 1, 1, 1, hold);
	check(!arm.armed, "estop release with arm held still locked");

	// 松开解锁键，再重新长按：必须先松开才能重新解锁
	L400M_ArmStep(&arm, 40010, 0, 0, 0, 1, 1, 1, hold);
	check(!arm.armed, "arm released stays locked");
	L400M_ArmStep(&arm, 40020, 1, 0, 0, 1, 1, 1, hold);
	L400M_ArmStep(&arm, 40020 + hold - 1, 1, 0, 0, 1, 1, 1, hold);
	check(!arm.armed, "re-arm before hold");
	L400M_ArmStep(&arm, 40020 + hold, 1, 0, 0, 1, 1, 1, hold);
	check(arm.armed, "re-arm after hold");

	printf("PASS estop held blocks unlock and re-hold re-arms\n");
}

int main(void)
{
	test_pitch_sign();
	test_estop_hold_and_rearm();
	if (failed) {
		printf("RESULT FAIL\n");
		return 1;
	}
	printf("RESULT OK\n");
	return 0;
}
