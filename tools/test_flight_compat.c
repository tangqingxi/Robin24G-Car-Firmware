/*
 * b3cb56f 旧版 L400Motor.h 参考逻辑 vs 当前拆分模块的主机对照。
 *
 * 只比较纯行为：油门换算、四路混控、正负差动、限幅、事件沿和故障后重新解锁。
 * 旧版公式逐行取自 b3cb56f:.../L400Motor.h，不链接旧固件。
 */
#include <stdio.h>
#include <string.h>

#include "MotorMixer.h"
#include "FlightSafety.h"
#include "RcInput.h"
#include "UserPwm.h"

unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static int failures;
static int checks;

static void check_eq(int a, int b, const char *what)
{
	checks++;
	if (a != b) {
		printf("FAIL %s: current=%d old=%d\n", what, a, b);
		failures++;
	}
}

// ---- b3cb56f 旧版参考实现（常量/公式保持一致）----
#define OLD_STICK_MAX          307
#define OLD_THR_MID            190
#define OLD_THR_SPAN           140
#define OLD_THR_MIN            60
#define OLD_THR_MAX            340
#define OLD_PWM_MIN_RUN        45
#define OLD_PWM_MAX            400
#define OLD_ARM_THR_MAX        (-250)
#define OLD_ARM_STICK_MAX      45
#define OLD_DIFF_GAIN_Q16      2880
#define OLD_YAW_DIFF_GAIN_Q16  1920
#define OLD_DIFF_LIMIT         160
#define OLD_EVENT_ESTOP        0x0001
#define OLD_EVENT_UNLOCK       0x0002
#define OLD_LOCK_NONE          0
#define OLD_LOCK_BOOT          1
#define OLD_LOCK_LINK          2
#define OLD_LOCK_MANUAL        5

typedef struct {
	unsigned short front, rear, left, right;
} OldMix;

typedef struct {
	unsigned char armed;
	unsigned char reason;
	unsigned short key_prev;
} OldArm;

static int old_clamp(int v, int lim)
{
	if (v > lim) return lim;
	if (v < -lim) return -lim;
	return v;
}

static int old_thrust(int fb)
{
	int t = OLD_THR_MID + (old_clamp(fb, OLD_STICK_MAX) * OLD_THR_SPAN) / OLD_STICK_MAX;

	t = old_clamp(t, OLD_THR_MAX);
	if (t < OLD_THR_MIN) t = OLD_THR_MIN;
	return t;
}

static void old_mix(int thrust, int roll, int pitch, int yaw, OldMix *out)
{
	int dr = (roll * OLD_DIFF_GAIN_Q16) >> 16;
	int dp = (pitch * OLD_DIFF_GAIN_Q16) >> 16;
	int dy = (yaw * OLD_YAW_DIFF_GAIN_Q16) >> 16;
	int mf, mr, ml, mm, lo, hi, scale, room;

	dr = old_clamp(dr, OLD_DIFF_LIMIT);
	dp = old_clamp(dp, OLD_DIFF_LIMIT);
	dy = old_clamp(dy, OLD_DIFF_LIMIT);

	mf = thrust + dp - dy;
	mr = thrust - dp - dy;
	ml = thrust + dr + dy;
	mm = thrust - dr + dy;

	lo = mf;
	if (mr < lo) lo = mr;
	if (ml < lo) lo = ml;
	if (mm < lo) lo = mm;
	hi = mf;
	if (mr > hi) hi = mr;
	if (ml > hi) hi = ml;
	if (mm > hi) hi = mm;

	scale = 256;
	if (hi > OLD_PWM_MAX && hi > thrust) {
		room = ((OLD_PWM_MAX - thrust) << 8) / (hi - thrust);
		if (room < scale) scale = room;
	}
	if (lo < OLD_PWM_MIN_RUN && lo < thrust) {
		room = ((thrust - OLD_PWM_MIN_RUN) << 8) / (thrust - lo);
		if (room < scale) scale = room;
	}
	if (scale < 0) scale = 0;
	if (scale < 256) {
		dr = (dr * scale) >> 8;
		dp = (dp * scale) >> 8;
		dy = (dy * scale) >> 8;
		mf = thrust + dp - dy;
		mr = thrust - dp - dy;
		ml = thrust + dr + dy;
		mm = thrust - dr + dy;
	}

	mf = old_clamp(mf, OLD_PWM_MAX);
	mr = old_clamp(mr, OLD_PWM_MAX);
	ml = old_clamp(ml, OLD_PWM_MAX);
	mm = old_clamp(mm, OLD_PWM_MAX);
	if (mf < OLD_PWM_MIN_RUN) mf = OLD_PWM_MIN_RUN;
	if (mr < OLD_PWM_MIN_RUN) mr = OLD_PWM_MIN_RUN;
	if (ml < OLD_PWM_MIN_RUN) ml = OLD_PWM_MIN_RUN;
	if (mm < OLD_PWM_MIN_RUN) mm = OLD_PWM_MIN_RUN;

	out->front = (unsigned short)mf;
	out->rear = (unsigned short)mr;
	out->left = (unsigned short)ml;
	out->right = (unsigned short)mm;
}

static unsigned char old_event(OldArm *a, unsigned short key,
							   unsigned char pv, unsigned char low,
							   unsigned char center, unsigned char att,
							   unsigned char link)
{
	unsigned char ne = (key & OLD_EVENT_ESTOP) && !(a->key_prev & OLD_EVENT_ESTOP);
	unsigned char nu = (key & OLD_EVENT_UNLOCK) && !(a->key_prev & OLD_EVENT_UNLOCK);

	a->key_prev = key;
	if (ne) {
		a->armed = 0;
		a->reason = OLD_LOCK_MANUAL;
		return OLD_LOCK_MANUAL;
	}
	if (nu && pv && !(key & OLD_EVENT_ESTOP)
		&& low && center && att && link) {
		a->armed = 1;
		a->reason = OLD_LOCK_NONE;
	}
	return 0;
}

static void compare_one(int thrust, int roll, int pitch, int yaw,
						const char *tag)
{
	MotorMixerOutput cur;
	OldMix old;

	MotorMixer_Mix(thrust, roll, pitch, yaw, &cur);
	old_mix(thrust, roll, pitch, yaw, &old);
	check_eq(cur.front, old.front, tag);
	check_eq(cur.rear, old.rear, tag);
	check_eq(cur.left, old.left, tag);
	check_eq(cur.right, old.right, tag);
}

static void test_mixer(void)
{
	static const int fb[] = {-400, -307, -250, 0, 190, 307, 400};
	static const int axis[] = {0, -2048, 2048, -8000, 8000};
	unsigned int i, j;

	for (i = 0; i < sizeof(fb) / sizeof(fb[0]); i++) {
		check_eq(MotorMixer_Thrust(fb[i]), old_thrust(fb[i]), "thrust");
	}
	for (i = 0; i < 5; i++) {
		compare_one(190, 0, axis[i], 0, "pitch");
		compare_one(190, 0, 0, axis[i], "yaw");
		compare_one(190, axis[i], 0, 0, "roll");
	}
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			compare_one(190, axis[i], axis[j], axis[4 - i], "combined");
		}
	}
	compare_one(60, 8000, -8000, 8000, "low-throttle-limit");
	compare_one(340, -8000, 8000, -8000, "high-throttle-limit");
}

static void compare_event(FlightSafety *fs, OldArm *old,
						  unsigned short key, unsigned char pv,
						  unsigned char low, unsigned char center,
						  unsigned char att, unsigned char link)
{
	unsigned char cs = FlightSafety_OnKey(fs, key, pv, low, center, att, link);
	unsigned char os = old_event(old, key, pv, low, center, att, link);

	if (cs) {
		FlightSafety_Lock(fs, cs);
	}
	if (os) {
		old->armed = 0;
		old->reason = os;
	}
	check_eq(cs, os, "event stop");
	check_eq(fs->armed, old->armed, "event armed");
	check_eq(fs->reason, old->reason, "event reason");
	if (!fs->armed) {
		check_eq(LFmotDuty | LBmotDuty | RBmotDuty | RFmotDuty, 0, "locked pwm zero");
	}
}

static void test_events(void)
{
	FlightSafety fs;
	OldArm old;

	FlightSafety_Init(&fs);
	FlightSafety_InitPwm(&fs);
	FlightSafety_NoteAttitude(&fs);
	old.armed = 0;
	old.reason = OLD_LOCK_BOOT;
	old.key_prev = 0;

	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 2, 1, 1, 1, 1, 1);     // 新 0002 解锁
	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 1, 1, 1, 1, 1, 1);     // 0001 急停
	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 2, 1, 1, 1, 1, 1);     // 急停后新 0002
	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 2, 0, 1, 1, 1, 1);     // 无效包拒绝
	compare_event(&fs, &old, 2, 1, 1, 1, 1, 1);     // 同一脉冲重复包不能补解锁
	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 2, 1, 1, 1, 1, 1);

	FlightSafety_Lock(&fs, FLIGHT_LOCK_LINK);
	old.armed = 0;
	old.reason = OLD_LOCK_LINK;
	compare_event(&fs, &old, 0, 1, 1, 1, 1, 1);
	compare_event(&fs, &old, 2, 1, 1, 1, 1, 1);     // 链路故障后新 0002

	compare_event(&fs, &old, 1, 0, 1, 1, 1, 1);     // 无效包也能急停
	compare_event(&fs, &old, 0, 0, 1, 1, 1, 1);
	compare_event(&fs, &old, 3, 0, 1, 1, 1, 1);     // 0001+0002 同包
}

static void test_rc_mapping(void)
{
	static const unsigned short raws[] = {0x0400, 0x0800, 0x1000, 0x1800, 0x1C00};
	RcInput rc;
	unsigned int i;

	RcInput_Init(&rc);
	for (i = 0; i < sizeof(raws) / sizeof(raws[0]); i++) {
		RcInputEvent ev = RcInput_OnPacket(&rc, raws[i], raws[4 - i],
										   raws[i], raws[4 - i], 0, 100);
		check_eq(ev.cmd[0], (int)(raws[i] - 0x1000) / 10, "rc throttle");
		check_eq(ev.cmd[3], -((int)(raws[4 - i] - 0x1000) / 10), "rc roll");
	}
}

int main(void)
{
	test_mixer();
	test_events();
	test_rc_mapping();
	printf("checks=%d failures=%d\n", checks, failures);
	if (failures) {
		printf("RESULT FAIL\n");
		return 1;
	}
	printf("PASS b3cb56f vs current: thrust/mixer/limits/events/rc mapping identical\n");
	printf("RESULT OK\n");
	return 0;
}
