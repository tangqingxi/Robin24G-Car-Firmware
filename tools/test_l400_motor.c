/*
 * L400 拆分后的混控/事件回归（主机侧）
 *
 * 覆盖：
 *   1. 机头抬高（corr_pitch<0）时，限幅前后都必须 后 > 前；
 *   2. 正偏航时 左+右 > 前+后，负偏航相反；限幅后同样成立；
 *   3. 200ms 的 0002 脉冲只解锁一次；重复包/无效摇杆包不能补解锁；
 *   4. 0001 立即锁定并清四路，0001 优先；急停/失联后必须新 0002。
 *
 * 编译（在 tools 目录，gcc 可用）：
 *   gcc -std=c99 -O2 -fno-builtin -I..\telink_b85m_platform_src1015\common ^
 *       -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
 *       test_l400_motor.c ..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\MotorMixer.c ^
 *       ..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\FlightSafety.c ^
 *       -o ..\data\test_l400_motor.exe
 */

#include <stdio.h>
#include <string.h>

#include "MotorMixer.h"
#include "FlightSafety.h"
#include "UserPwm.h"

// 主机侧 PWM 桩：FlightSafety 只通过这两个符号写硬件。
unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;

void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static int failed;

static void check(int cond, const char *msg)
{
	if (!cond) {
		printf("FAIL %s\n", msg);
		failed = 1;
	}
}

static void test_pitch_sign(void)
{
	MotorMixerOutput small_mix, large_mix, nose_down_mix;

	MotorMixer_Mix(190, 0, -2048, 0, &small_mix);
	check(small_mix.rear > small_mix.front, "nose-up small: rear > front");
	check(small_mix.front >= MOTOR_PWM_MIN_RUN
		  && small_mix.rear <= MOTOR_PWM_MAX, "nose-up small in range");

	MotorMixer_Mix(190, 0, -8000, 0, &large_mix);
	check(large_mix.rear > large_mix.front, "nose-up limited: rear > front");

	MotorMixer_Mix(190, 0, 2048, 0, &nose_down_mix);
	check(nose_down_mix.front > nose_down_mix.rear,
		  "nose-down: front > rear");

	printf("PASS pitch sign small=%u/%u limited=%u/%u\n",
		   small_mix.front, small_mix.rear,
		   large_mix.front, large_mix.rear);
}

static void test_yaw_sign(void)
{
	MotorMixerOutput pos, neg, pos_lim, neg_lim;

	MotorMixer_Mix(190, 0, 0, 2048, &pos);
	check(pos.left + pos.right > pos.front + pos.rear,
		  "positive yaw: left+right > front+rear");
	MotorMixer_Mix(190, 0, 0, -2048, &neg);
	check(neg.left + neg.right < neg.front + neg.rear,
		  "negative yaw: left+right < front+rear");

	MotorMixer_Mix(190, 0, 0, 8000, &pos_lim);
	check(pos_lim.left + pos_lim.right > pos_lim.front + pos_lim.rear,
		  "positive yaw limited: left+right > front+rear");
	MotorMixer_Mix(190, 0, 0, -8000, &neg_lim);
	check(neg_lim.left + neg_lim.right < neg_lim.front + neg_lim.rear,
		  "negative yaw limited: left+right < front+rear");

	printf("PASS yaw sign +%u/%u -%u/%u lim +%u/%u -%u/%u\n",
		   pos.left + pos.right, pos.front + pos.rear,
		   neg.left + neg.right, neg.front + neg.rear,
		   pos_lim.left + pos_lim.right, pos_lim.front + pos_lim.rear,
		   neg_lim.left + neg_lim.right, neg_lim.front + neg_lim.rear);
}

static unsigned char key_event(FlightSafety *fs, unsigned short key,
							   unsigned char packet_valid)
{
	unsigned char stop = FlightSafety_OnKey(fs, key, packet_valid,
											1, 1, 1, 1, 0);

	if (stop) {
		FlightSafety_Lock(fs, stop);
	}
	return stop;
}

static void test_unlock_pulse_and_invalid_packet(void)
{
	FlightSafety fs;
	MotorMixerOutput mix = {100, 200, 300, 400};
	int i;

	FlightSafety_Init(&fs);
	FlightSafety_InitPwm(&fs);
	FlightSafety_NoteAttitude(&fs);
	check(!fs.armed, "boot locked");

	for (i = 0; i < 40; i++) {
		key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
		check(fs.armed, "200ms pulse unlocks");
	}
	FlightSafety_Apply(&fs, &mix);
	check(LFmotDuty == 100 && LBmotDuty == 200
		  && RBmotDuty == 300 && RFmotDuty == 400, "armed output mix");

	key_event(&fs, 0, 1);
	check(fs.armed, "release does not relock");

	FlightSafety_Lock(&fs, FLIGHT_LOCK_LINK);
	key_event(&fs, 0, 0);
	check(!fs.armed, "link loss locked");
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 0);
	check(!fs.armed, "invalid packet discards unlock");
	for (i = 0; i < 39; i++) {
		key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	}
	check(!fs.armed, "duplicate of discarded pulse does not unlock");

	key_event(&fs, 0, 1);
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(fs.armed, "new 0002 after invalid pulse unlocks");

	printf("PASS 200ms pulse, duplicates and invalid stick packet\n");
}

static void test_estop_priority_and_rearm(void)
{
	FlightSafety fs;
	unsigned char stop;

	FlightSafety_Init(&fs);
	FlightSafety_InitPwm(&fs);
	FlightSafety_NoteAttitude(&fs);
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(fs.armed, "arm before estop");

	stop = key_event(&fs, FLIGHT_EVENT_ESTOP, 1);
	check(stop == FLIGHT_LOCK_MANUAL, "estop reason");
	check(!fs.armed, "estop locks");
	check(LFmotDuty == 0 && LBmotDuty == 0
		  && RBmotDuty == 0 && RFmotDuty == 0, "estop zeroes pwm");

	key_event(&fs, 0, 1);
	key_event(&fs, FLIGHT_EVENT_ESTOP, 1);
	key_event(&fs, FLIGHT_EVENT_ESTOP | FLIGHT_EVENT_UNLOCK, 1);
	check(!fs.armed, "0002 while 0001 held blocked");
	key_event(&fs, FLIGHT_EVENT_ESTOP | FLIGHT_EVENT_UNLOCK, 1);
	check(!fs.armed, "held 0003 duplicate blocked");

	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(!fs.armed, "old 0002 after estop does not unlock");

	key_event(&fs, 0, 1);
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(fs.armed, "new 0002 after estop unlocks");

	FlightSafety_Lock(&fs, FLIGHT_LOCK_LINK);
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(!fs.armed, "link loss requires new 0002");
	key_event(&fs, 0, 1);
	key_event(&fs, FLIGHT_EVENT_UNLOCK, 1);
	check(fs.armed, "new 0002 after link loss unlocks");

	stop = key_event(&fs, FLIGHT_EVENT_ESTOP, 0);
	check(stop == FLIGHT_LOCK_MANUAL, "invalid-packet estop reason");
	check(!fs.armed, "invalid-packet estop locks");

	printf("PASS estop priority and new-event rearm\n");
}

int main(void)
{
	test_pitch_sign();
	test_yaw_sign();
	test_unlock_pulse_and_invalid_packet();
	test_estop_priority_and_rearm();
	if (failed) {
		printf("RESULT FAIL\n");
		return 1;
	}
	printf("RESULT OK\n");
	return 0;
}
