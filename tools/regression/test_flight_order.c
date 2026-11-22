/*
 * 飞控调用顺序回归（主机侧，链接真实 FlightLoop_OnRcPacket /
 * FlightLoop_PreSampleSafety / FlightSafety / RcInput）。
 *
 * 覆盖：
 *   1. 新 0001 事件先触发四路清零，同包不再做目标姿态换算；
 *   2. 失联/指令超时的采样前安全门确实调用 PWM 清零。
 */
#include <stdio.h>
#include <string.h>

#include "FlightLoop.h"
#include "AttitudeControl.h"
#include "RateControl.h"

unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;

static int seq;
static int zero_writes;
static int zero_seq = -1;
static int target_calls;

void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void)
{
	if (LFmotDuty == 0 && LBmotDuty == 0
		&& RBmotDuty == 0 && RFmotDuty == 0) {
		zero_writes++;
		zero_seq = seq++;
	}
}

void AttitudeControl_Init(AttitudeControl *ac) { (void)ac; }
void AttitudeControl_OnStick(AttitudeControl *ac, const int cmd[4])
{
	(void)ac; (void)cmd;
	target_calls++;
	seq++;
}
void AttitudeControl_Inhibit(AttitudeControl *ac) { (void)ac; }
void AttitudeControl_ClearOutputs(AttitudeControl *ac) { (void)ac; }
void AttitudeControl_Update(AttitudeControl *ac, const euler_q_t *att)
{ (void)ac; (void)att; }

void RateControl_Init(RateControl *rc) { (void)rc; }
void RateControl_ClearOutputs(RateControl *rc) { (void)rc; }
void RateControl_Update(RateControl *rc, const int target[3], const int gyro[3])
{ (void)rc; (void)target; (void)gyro; }

// FlightLoop 其他路径的链接桩；顺序测试不进入这些函数。
u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 0; return 1; }
u8 t3488_read_6axis(short *a, short *g) { (void)a; (void)g; return 1; }
void Euler_Reset(euler_q_t *m) { (void)m; }
u8 Euler_Update(euler_q_t *m, const s32 a[3], const s32 g[3], s32 dt, u32 t)
{ (void)m; (void)a; (void)g; (void)dt; (void)t; return 0; }
u8 Euler_InitFromAccel(euler_q_t *m, const s32 a[3], u32 t)
{ (void)m; (void)a; (void)t; return 0; }
u8 Euler_IsStill(const s32 a[3], const s32 g[3], const s32 prev[3], u8 ok)
{ (void)a; (void)g; (void)prev; (void)ok; return 0; }
s32 Euler_GyroToQ10(s32 raw, s32 bias, s32 *rem)
{ (void)bias; if (rem) *rem = 0; return raw; }

unsigned int clock_time(void) { return 1000; }

void FlightLoop_OrderTestReset(void);
void FlightLoop_OrderTestSetRc(unsigned char link, unsigned char cmd_ok);

int main(void)
{
	FlightLoop_OrderTestReset();
	seq = 0;
	zero_writes = 0;
	zero_seq = -1;
	target_calls = 0;

	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x1000, 0x1000, 0x0001);
	if (zero_writes < 1 || zero_seq < 0) {
		printf("FAIL estop did not zero pwm\n");
		return 1;
	}
	if (target_calls != 0) {
		printf("FAIL target conversion ran in estop packet\n");
		return 1;
	}
	printf("PASS 0001 zero pwm before target conversion (seq=%d)\n", zero_seq);

	FlightLoop_OrderTestReset();
	FlightLoop_OrderTestSetRc(0, 1);
	zero_writes = 0;
	zero_seq = -1;
	FlightLoop_PreSampleSafety();
	if (zero_writes < 1) {
		printf("FAIL link loss presample did not zero pwm\n");
		return 1;
	}
	printf("PASS link-loss presample zero pwm (seq=%d)\n", zero_seq);

	FlightLoop_OrderTestReset();
	FlightLoop_OrderTestSetRc(1, 0);
	zero_writes = 0;
	zero_seq = -1;
	FlightLoop_PreSampleSafety();
	if (zero_writes < 1) {
		printf("FAIL command timeout presample did not zero pwm\n");
		return 1;
	}
	printf("PASS command-timeout presample zero pwm (seq=%d)\n", zero_seq);
	printf("RESULT OK\n");
	return 0;
}
