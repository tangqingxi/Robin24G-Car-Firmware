/*
 * 诊断启动路径回归：链接真实 FlightLoop + FlightArmDebug，确认
 * OnStillSample / OnStartupResult 已接回判稳路径，ADI 统计不为零。
 */
#include <stdio.h>
#include <string.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "FlightArmDebug.h"
#include "UserUart.h"

unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static char lines[64][UART_DATA_LEN];
static int line_count;

u8 UartTrySend(u8 *dat, u8 len)
{
	if (line_count < 64) {
		int n = len < UART_DATA_LEN - 1 ? len : UART_DATA_LEN - 1;
		memcpy(lines[line_count], dat, n);
		lines[line_count][n] = 0;
		line_count++;
	}
	return 1;
}

u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 3; return 0; }
u8 t3488_read_6axis(short *a, short *g)
{
	a[0] = -22;
	a[1] = 8;
	a[2] = 2100;
	g[0] = g[1] = g[2] = 0;
	return 0;
}

static u32 g_now;
u32 clock_time(void) { return g_now; }

unsigned char FlightLoop_StartupTestReady(void);
unsigned char FlightLoop_StartupTestArmed(void);
void FlightLoop_Init(void);
void FlightLoop_Process(void);
void FlightLoop_OnRcPacket(unsigned short thr, unsigned short rot,
						   unsigned short fb, unsigned short lr,
						   unsigned short key);

static const char *find_prefix(const char *prefix)
{
	int i;

	for (i = 0; i < line_count; i++) {
		if (strncmp(lines[i], prefix, strlen(prefix)) == 0) {
			return lines[i];
		}
	}
	return NULL;
}

int main(void)
{
	int i;
	int ready = 0;
	const char *adi;
	const char *adb;

	FlightLoop_Init();
	FlightArmDebug_Process();
	for (i = 0; i < 260; i++) {
		g_now += FLIGHT_PERIOD_TICKS;
		FlightLoop_Process();
		FlightArmDebug_Process();
		if (!ready && FlightLoop_StartupTestReady()) {
			ready = 1;
		}
	}
	if (!ready) {
		printf("FAIL startup not ready\n");
		return 1;
	}
	adb = find_prefix("ADB ");
	adi = find_prefix("ADI ");
	if (!adb || !adi) {
		printf("FAIL no ADB/ADI (ADB=%d ADI=%d lines=%d)\n",
			   adb != NULL, adi != NULL, line_count);
		return 1;
	}
	if (!strstr(adi, "sm=") || strstr(adi, "sm=0 ")) {
		printf("FAIL still max not recorded: %s", adi);
		return 1;
	}
	printf("PASS diagnostic startup ADB/ADI, %s", adi);
	printf("RESULT OK\n");
	return 0;
}
