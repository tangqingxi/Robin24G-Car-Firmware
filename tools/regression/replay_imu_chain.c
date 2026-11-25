/* 整链 IMU 回放：静止启动、动态输入、读失败、100/105ms、回绕、急停和失联。 */
#include <stdio.h>
#include <string.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "UserEuler.h"

unsigned short LFmotDuty, LBmotDuty, RBmotDuty, RFmotDuty;
static char diag_line[160];
u8 UartTrySend(u8 *d, u8 n)
{
	if (n >= sizeof(diag_line)) return 0;
	memcpy(diag_line, d, n);
	diag_line[n] = 0;
	return 1;
}
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static u32 g_now = 0xFFFFF000u;
static int g_tick;
static int g_fail;
static short ax, ay, az, gx, gy, gz;
static u8 read_ok;

u32 clock_time(void) { return g_now; }
u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 3; return 0; }
u8 t3488_read_6axis(short *a, short *g)
{
	if (g_fail) return 1;
	a[0] = ax; a[1] = ay; a[2] = az;
	g[0] = gx; g[1] = gy; g[2] = gz;
	read_ok = 1;
	return 0;
}

unsigned char FlightLoop_StartupTestReady(void);
unsigned char FlightLoop_StartupTestArmed(void);
int FlightLoop_StartupTestLpf(int i);
unsigned char FlightLoop_StartupTestLockReason(void);
unsigned char FlightLoop_StartupTestAttUpdated(void);
void FlightLoop_StartupTestGyro(int out[3]);
void FlightLoop_StartupTestQ(int out[4]);

int main(void)
{
	int i, q[4], gr[3], armed = 0, unlocked = 0, stopped = 0, lost = 0;
	unsigned short pwm[4];
	u16 gap, missed;

	FlightLoop_Init();
	for (i = 0; i < 600; i++) {
		g_tick = i;
		g_fail = (i >= 220 && i <= 222);
		if (i < 160) {
			ax = -22; ay = 8; az = 2100;
			gx = gy = gz = 0;
		} else {
			ax = (short)(-22 + i % 17);
			ay = (short)(8 + i % 11);
			az = (short)(2100 + i % 23);
			gx = (short)(i % 7);
			gy = (short)(i % 5);
			gz = (short)(i % 3);
		}
		if (i == 260) g_now += 95u * 24000u;   // 加上正常 5ms = 100ms
		if (i == 320) g_now += 100u * 24000u;  // 105ms
		if (i >= 160 && i < 450) {
			FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000,
								  i == 160 ? 0x0002 : 0);
		}
		if (i == 420) {
			FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0001);
		}
		g_now += FLIGHT_PERIOD_TICKS;
		FlightLoop_Process();
		FlightLoop_GetDiagStatus((u8 *)&armed, pwm, &gap, &missed);
		FlightLoop_StartupTestGyro(gr);
		FlightLoop_StartupTestQ(q);
		printf("%d,%d,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			   i, armed, FlightLoop_StartupTestLockReason(),
			   pwm[0], pwm[1], pwm[2], pwm[3],
			   FlightLoop_StartupTestLpf(0), FlightLoop_StartupTestLpf(1),
			   FlightLoop_StartupTestLpf(2), gr[0], gr[1], gr[2],
			   q[0], q[1], q[2], q[3]);

		if (!FlightLoop_StartupTestReady() && i > 400) {
			printf("FAIL attitude not ready at %d\n", i);
			return 1;
		}
		if (i == 160 && !FlightLoop_StartupTestReady()) {
			printf("FAIL attitude not ready before unlock\n");
			return 1;
		}
		if (i == 180) {
			if (!armed || (pwm[0] == 0 && pwm[1] == 0 &&
						   pwm[2] == 0 && pwm[3] == 0)) {
				printf("FAIL unlock/nonzero pwm\n");
				return 1;
			}
			unlocked = 1;
		}
		if (i == 430) {
			if (armed || pwm[0] || pwm[1] || pwm[2] || pwm[3]) {
				printf("FAIL estop\n");
				return 1;
			}
			stopped = 1;
		}
		if (i == 599) {
			if (armed) {
				printf("FAIL link loss\n");
				return 1;
			}
			lost = 1;
		}
	}
	if (!read_ok || !unlocked || !stopped || !lost) {
		printf("FAIL event missing read=%u unlock=%d estop=%d loss=%d\n",
			   read_ok, unlocked, stopped, lost);
		return 1;
	}
	printf("PASS chain replay ticks=600 unlock/estop/loss\n");
	return 0;
}
