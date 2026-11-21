/*
 * L400 电机混控与 0001/0002 短事件回归（主机侧）
 *
 * 覆盖：
 *   1. 机头抬高（corr_pitch<0）时，限幅前后都必须 后电机 > 前电机；
 *   2. 实测 200ms 的 0002 脉冲：重复包只解锁一次；无效摇杆包丢弃，
 *      之后同一脉冲的重复包不能补解锁，必须重新发新 0002；
 *   3. 0001 立即急停且优先；按住 0001 期间 0002 无效；急停/失联后
 *      必须收到新的 0002 才能重新解锁。
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

static unsigned char event_step(L400MArm *a, unsigned short key,
								unsigned char packet_valid)
{
	// 除 packet_valid 外，其余解锁条件全部满足
	return L400M_EventStep(a, key, packet_valid, 1, 1, 1, 1, 0);
}

// 200ms 脉冲、重复包、无效摇杆包
static void test_unlock_pulse_and_invalid_packet(void)
{
	L400MArm arm;
	L400MMix mix = {100, 200, 300, 400};
	unsigned short out[4];
	int i;

	L400M_ArmInit(&arm);
	check(!arm.armed, "boot locked");

	// 200ms 脉冲按 5ms 一包模拟，共 40 包；只有第一个沿解锁
	for (i = 0; i < 40; i++) {
		event_step(&arm, L400_EVENT_UNLOCK, 1);
		check(arm.armed, "200ms pulse unlocks");
	}
	L400M_Output(&arm, &mix, out);
	check(out[0] == 100 && out[3] == 400, "armed output mix");

	// 释放 0002，保持解锁状态
	event_step(&arm, 0, 1);
	check(arm.armed, "release does not relock");

	// 先造一次失联锁定，然后无效摇杆包带新 0002：应消费事件但不解锁
	L400M_ArmLock(&arm, L400_LOCK_LINK);
	event_step(&arm, 0, 0);                  // 先让按键回 0，形成新事件条件
	check(!arm.armed, "link loss locked");
	event_step(&arm, L400_EVENT_UNLOCK, 0);  // 无效包：丢弃
	check(!arm.armed, "invalid packet discards unlock");
	for (i = 0; i < 39; i++) {
		event_step(&arm, L400_EVENT_UNLOCK, 1); // 同一脉冲的后续有效重复包
	}
	check(!arm.armed, "duplicate of discarded pulse does not unlock");

	// 松开后重新发一个新的 0002 事件才解锁
	event_step(&arm, 0, 1);
	event_step(&arm, L400_EVENT_UNLOCK, 1);
	check(arm.armed, "new 0002 after invalid pulse unlocks");

	printf("PASS 200ms pulse, duplicates and invalid stick packet\n");
}

// 0001 急停优先、按住期间禁止 0002、失联/急停后必须新事件
static void test_estop_priority_and_rearm(void)
{
	L400MArm arm;
	unsigned char stop;

	L400M_ArmInit(&arm);
	event_step(&arm, L400_EVENT_UNLOCK, 1);
	check(arm.armed, "arm before estop");

	// 0001 新事件：立即锁定
	stop = event_step(&arm, L400_EVENT_ESTOP, 1);
	check(stop == L400_LOCK_MANUAL, "estop reason");
	check(!arm.armed, "estop locks");

	// 0001 按住期间，新出现的 0002（key=0003）不能解锁
	event_step(&arm, 0, 1);
	event_step(&arm, L400_EVENT_ESTOP, 1);
	event_step(&arm, L400_EVENT_ESTOP | L400_EVENT_UNLOCK, 1);
	check(!arm.armed, "0002 while 0001 held blocked");
	event_step(&arm, L400_EVENT_ESTOP | L400_EVENT_UNLOCK, 1);
	check(!arm.armed, "held 0003 duplicate blocked");

	// 松开 0001 但仍保留 0002 的旧沿：不算新 0002
	event_step(&arm, L400_EVENT_UNLOCK, 1);
	check(!arm.armed, "old 0002 after estop does not unlock");

	// 松开到 0，再发新的 0002
	event_step(&arm, 0, 1);
	event_step(&arm, L400_EVENT_UNLOCK, 1);
	check(arm.armed, "new 0002 after estop unlocks");

	// 失联锁定；若 0002 在失联前已按住，同一个脉冲不能重新解锁
	L400M_ArmLock(&arm, L400_LOCK_LINK);
	event_step(&arm, L400_EVENT_UNLOCK, 1);  // key_prev 仍为 0002，无新沿
	check(!arm.armed, "link loss requires new 0002");
	event_step(&arm, 0, 1);
	event_step(&arm, L400_EVENT_UNLOCK, 1);
	check(arm.armed, "new 0002 after link loss unlocks");

	// 0001 即使落在无效摇杆包上，也必须立即急停
	stop = event_step(&arm, L400_EVENT_ESTOP, 0);
	check(stop == L400_LOCK_MANUAL, "invalid-packet estop reason");
	check(!arm.armed, "invalid-packet estop locks");

	printf("PASS estop priority and new-event rearm\n");
}

int main(void)
{
	test_pitch_sign();
	test_unlock_pulse_and_invalid_packet();
	test_estop_priority_and_rearm();
	if (failed) {
		printf("RESULT FAIL\n");
		return 1;
	}
	printf("RESULT OK\n");
	return 0;
}
