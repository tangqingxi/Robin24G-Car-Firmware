/* 整链 IMU 回放：静止启动、动态输入、读失败、100/105 ms、回绕、急停、指令超时和链路超时。 */
#include <stdio.h>
#include <string.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "UserEuler.h"
#include "FlightSafety.h"

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
int FlightLoop_StartupTestLpf(int i);
unsigned char FlightLoop_StartupTestAttUpdated(void);
unsigned char FlightLoop_StartupTestLockReason(void);
void FlightLoop_StartupTestGyro(int out[3]);
void FlightLoop_StartupTestQ(int out[4]);

static int all_zero(const unsigned short p[4])
{
	return !p[0] && !p[1] && !p[2] && !p[3];
}

int main(void)
{
	int i, q[4], gr[3];
	u8 armed = 0;
	unsigned short pwm[4];
	u16 gap, missed;
	int got_unlock = 0, got_estop = 0, got_cmd = 0, got_link = 0;

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
		if (i == 270) g_now += 95u * 24000u;   // 正常 5ms，合起来正好 100ms
		if (i == 330) g_now += 100u * 24000u;  // 105ms

		// 160 首次解锁；240 读失败后重新解锁；300 急停后再次解锁。
		if ((i >= 160 && i < 220) || (i >= 240 && i < 340)) {
			unsigned short key = (i == 160 || i == 240 || i == 300 || i == 310 || i == 320) ? 0x0002 : 0;
			FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, key);
		}
		if (i == 260) {
			FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0001);
		}
		if (i >= 340 && i < 420) {
			// 只发心跳，保持链路但不再刷新有效摇杆命令。
			FlightLoop_OnRcPacket(0, 0, 0, 0, 0);
		}

		g_now += FLIGHT_PERIOD_TICKS;
		FlightLoop_Process();
		FlightLoop_GetDiagStatus(&armed, pwm, &gap, &missed);
		FlightLoop_StartupTestGyro(gr);
		FlightLoop_StartupTestQ(q);

		// 每个控制拍一条定长 CSV，PASS/失败信息只写 stderr。
		printf("%d,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			   i, armed, FlightLoop_StartupTestAttUpdated(),
			   FlightLoop_StartupTestLockReason(),
			   pwm[0], pwm[1], pwm[2], pwm[3],
			   FlightLoop_StartupTestLpf(0), FlightLoop_StartupTestLpf(1),
			   FlightLoop_StartupTestLpf(2), gr[0], gr[1], gr[2],
			   q[0], q[1], q[2], q[3]);

		if (i == 160 && !FlightLoop_StartupTestReady()) {
			fprintf(stderr, "FAIL attitude not ready\n");
			return 1;
		}
		if (i == 180) {
			if (!armed || all_zero(pwm)) {
				fprintf(stderr, "FAIL unlock or zero pwm\n");
				return 1;
			}
			got_unlock = 1;
		}
		if (i == 222) {
			if (FlightLoop_StartupTestAttUpdated()
				|| FlightLoop_StartupTestLockReason() != FLIGHT_LOCK_ATT
				|| !all_zero(pwm)) {
				fprintf(stderr, "FAIL read-failure att/lock/pwm\n");
				return 1;
			}
		}
		if (i == 224) {
			if (!FlightLoop_StartupTestAttUpdated()) {
				fprintf(stderr, "FAIL attitude did not recover\n");
				return 1;
			}
		}
		if (i == 250 && (!armed || all_zero(pwm))) {
			fprintf(stderr, "FAIL re-unlock before estop\n");
			return 1;
		}
		if (i == 260) {
			if (armed || !all_zero(pwm)
				|| FlightLoop_StartupTestLockReason() != FLIGHT_LOCK_MANUAL) {
				fprintf(stderr, "FAIL estop pwm/lock\n");
				return 1;
			}
			got_estop = 1;
		}
		if (i == 400) {
			if (FlightLoop_StartupTestLockReason() != FLIGHT_LOCK_CMD
				|| armed || !all_zero(pwm)) {
				fprintf(stderr, "FAIL command timeout reason=%u\n",
						FlightLoop_StartupTestLockReason());
				return 1;
			}
			got_cmd = 1;
		}
		if (i == 540) {
			if (FlightLoop_StartupTestLockReason() != FLIGHT_LOCK_LINK
				|| armed || !all_zero(pwm)) {
				fprintf(stderr, "FAIL link timeout reason=%u\n",
						FlightLoop_StartupTestLockReason());
				return 1;
			}
			got_link = 1;
		}
	}
	if (!read_ok || !got_unlock || !got_estop || !got_cmd || !got_link) {
		fprintf(stderr, "FAIL missing events read=%u unlock=%d estop=%d cmd=%d link=%d\n",
				read_ok, got_unlock, got_estop, got_cmd, got_link);
		return 1;
	}
	fprintf(stderr, "PASS chain replay ticks=600 unlock/estop/read-recovery/cmd/link\n");
	return 0;
}
