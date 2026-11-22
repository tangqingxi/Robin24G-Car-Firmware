/*
 * FlightLoop 真实启动路径回归（主机侧）。
 *
 * 输入静置加速度 -10 / 0 / -1010 mg（由校准逆算的原始计数送入
 * t3488_read_6axis），连续满足 500ms 后姿态应就绪；再送合法低油门
 * 0002 包，验证 FlightSafety 能解锁。
 *
 * 这个用例专门防止 still_sum 被写成无符号后，负加速度求平均导致
 * Euler_InitFromAccel 失败、永远不就绪。
 */
#include <stdio.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "UserEuler.h"

// PWM 桩
unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

// 传感器桩：固定返回对应 -10/0/-1010 mg 的原始计数
static unsigned char read_ok = 0;
u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 3; return 0; }
u8 t3488_read_6axis(short *a, short *g)
{
	a[0] = -22;
	a[1] = 8;
	a[2] = 2100;
	g[0] = 0;
	g[1] = 0;
	g[2] = 0;
	read_ok = 1;
	return 0;
}

static u32 g_now = 1000000;
u32 clock_time(void) { return g_now; }

unsigned char FlightLoop_StartupTestReady(void);
unsigned char FlightLoop_StartupTestArmed(void);
unsigned int FlightLoop_StartupTestStillN(void);
unsigned int FlightLoop_StartupTestStillTicks(void);
int FlightLoop_StartupTestLpf(int i);

int main(void)
{
	int i;
	int ready = 0;

	FlightLoop_Init();
	for (i = 0; i < 140; i++) {
		g_now += FLIGHT_PERIOD_TICKS;
		FlightLoop_Process();
		if (FlightLoop_StartupTestReady()) {
			ready = 1;
			break;
		}
	}
	if (!read_ok) {
		printf("FAIL sensor stub not called\n");
		return 1;
	}
	if (!ready) {
		printf("FAIL startup not ready i=%d n=%u ticks=%u lpf=%d/%d/%d\n",
			   i, FlightLoop_StartupTestStillN(), FlightLoop_StartupTestStillTicks(),
			   FlightLoop_StartupTestLpf(0), FlightLoop_StartupTestLpf(1),
			   FlightLoop_StartupTestLpf(2));
		return 1;
	}
	printf("PASS startup ready at sample %d\n", i);

	// 合法低油门包：FB=0x0400（-307）、其余中位、0002 解锁事件
	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0002);
	if (!FlightLoop_StartupTestArmed()) {
		printf("FAIL valid 0002 did not unlock after startup\n");
		return 1;
	}
	printf("PASS valid 0002 unlocks after startup\n");
	printf("RESULT OK\n");
	return 0;
}
