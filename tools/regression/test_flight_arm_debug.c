/*
 * 解锁诊断固件文本/队列回归（主机侧，链接真实 FlightArmDebug.c）。
 * 只验证诊断协议、长度和 UART 忙重试，不替代上板电机测试。
 */
#include <stdio.h>
#include <string.h>

#include "FlightArmDebug.h"
#include "FlightConfig.h"
#include "UserUart.h"

#define MAX_LINES 32
static char lines[MAX_LINES][UART_DATA_LEN];
static int line_count;
static int busy_once;
static int uart_calls;

unsigned int FlightLog_Dropped(void) { return 0; }
unsigned int FlightLog_QueueCount(void) { return 2; }
unsigned int FlightLog_Written(void) { return 0; }
unsigned int FlightLog_QueueDrops(void) { return 0; }
unsigned int FlightLog_FlashPeakUs(void) { return 0; }

u8 UartTrySend(u8 *dat, u8 len)
{
	if (busy_once) {
		busy_once = 0;
		return 0;
	}
	if (line_count < MAX_LINES) {
		int n = len < UART_DATA_LEN - 1 ? len : UART_DATA_LEN - 1;
		memcpy(lines[line_count], dat, n);
		lines[line_count][n] = 0;
		line_count++;
	}
	uart_calls++;
	return 1;
}

void FlightSafety_GetPwm(const void *s, unsigned short out[4])
{
	(void)s;
	out[0] = 64;
	out[1] = 90;
	out[2] = 60;
	out[3] = 50;
}

static unsigned int g_now;
unsigned int clock_time(void) { return g_now; }

static int has_prefix(const char *p)
{
	int i;

	for (i = 0; i < line_count; i++) {
		if (strncmp(lines[i], p, strlen(p)) == 0) {
			return 1;
		}
	}
	return 0;
}

static int has_substr(const char *needle)
{
	int i;

	for (i = 0; i < line_count; i++) {
		if (strstr(lines[i], needle) != NULL) {
			return 1;
		}
	}
	return 0;
}

static int all_lengths_ok(void)
{
	int i;

	for (i = 0; i < line_count; i++) {
		if (strlen(lines[i]) > 159) {
			return 0;
		}
	}
	return 1;
}

int main(void)
{
	int cmd_bad[4] = {-180, 0, 0, 0};
	int cmd_ok[4] = {-307, 0, 0, 0};
	int acc[3] = {-10, 0, -1010};
	int gyro[3] = {0, 0, 0};
	int prev[3] = {-10, 0, -1010};
	unsigned short pwm[4] = {64, 90, 60, 50};

	FlightArmDebug_Init();
	FlightArmDebug_OnInit(1, 0, 0, 0, 0);
	FlightArmDebug_Process();
	FlightArmDebug_OnInit(0, 3, 500, 1, 1);
	FlightArmDebug_Process();

	g_now = 2 * CLOCK_SYS_CLOCK_HZ;
	FlightArmDebug_OnPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0002);
	FlightArmDebug_OnStatus(500, 1, 1);
	FlightArmDebug_Process();

	busy_once = 1;
	FlightArmDebug_OnUnlockAttempt(0x0002, 0x0400, cmd_bad, 1, 0, 1, 1, 1, acc, gyro);
	FlightArmDebug_Process();       // 第一次 UART 忙，保留
	FlightArmDebug_Process();       // 重试
	FlightArmDebug_OnUnlockAttempt(0x0000, 0, cmd_bad, 0, 0, 0, 0, 0, acc, gyro);
	FlightArmDebug_OnUnlockAttempt(0x0002, 0x0400, cmd_ok, 1, 1, 1, 1, 1, acc, gyro);
	FlightArmDebug_Process();

	FlightArmDebug_OnControl(1, 1, 1, 0, pwm);
	FlightArmDebug_Process();
	FlightArmDebug_OnControl(1, 0, 0, 2, (unsigned short[4]){0, 0, 0, 0});
	FlightArmDebug_Process();
	FlightArmDebug_OnEstop();
	FlightArmDebug_Process();

	/* 真实错过一次：同一拍先无预算、整拍结束仍未写，下一拍结算 bs/br。 */
	g_now = 100u * CLOCK_SYS_CLOCK_HZ;
	FlightArmDebug_OnLogGate(0, 2, 0);
	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnLogGate(0, 2, 0);
	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnLogGate(0, 2, 0);
	if (FlightArmDebug_LogBudgetSkips() < 2
		|| FlightArmDebug_LogBudgetMaxRun() < 2) {
		printf("FAIL gate skip not counted\n");
		return 1;
	}

	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnLogGate(1, 0, 0);        // 结算上一拍
	unsigned int budget_before = FlightArmDebug_LogBudgetSkips();

	/* 同一拍先无预算、后段实际写成功：这一拍不能算错过。 */
	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnLogGate(0, 2, 0);
	FlightArmDebug_OnLogGate(1, 2, 1);
	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnLogGate(1, 0, 0);
	if (FlightArmDebug_LogBudgetSkips() != budget_before) {
		printf("FAIL successful same-tick write counted as miss before=%u after=%u max=%u\n",
			   budget_before, FlightArmDebug_LogBudgetSkips(),
			   FlightArmDebug_LogBudgetMaxRun());
		return 1;
	}

	/* bm 只记同一控制拍第一次预算检查，后续查询不能拉低。 */
	g_now += FLIGHT_PERIOD_TICKS;
	FlightArmDebug_OnBudgetTiming(2400);
	FlightArmDebug_OnBudgetTiming(1000);
	if (FlightArmDebug_BudgetBelow() != 1
		|| FlightArmDebug_BudgetMinUs() != 2400) {
		printf("FAIL budget timing not deduped: below=%u min=%u\n",
			   FlightArmDebug_BudgetBelow(), FlightArmDebug_BudgetMinUs());
		return 1;
	}

	/* 启动擦除耗时必须单独输出 pm。 */
	g_now += CLOCK_SYS_CLOCK_HZ;
	FlightArmDebug_OnLogPrepareUs(12345);
	FlightArmDebug_Process();
	if (!has_prefix("ADT ") || !has_substr(" pm=12345")) {
		printf("FAIL prepare time not reported in ADT\n");
		return 1;
	}

	if (!has_prefix("ADB ")) { printf("FAIL no ADB\n"); return 1; }
	if (!has_prefix("ADS ")) { printf("FAIL no ADS\n"); return 1; }
	if (!has_prefix("ADU ")) { printf("FAIL no ADU\n"); return 1; }
	if (!has_prefix("ADO ")) { printf("FAIL no ADO\n"); return 1; }
	if (!has_prefix("ADL ")) { printf("FAIL no ADL\n"); return 1; }
	if (!has_prefix("ADE ")) { printf("FAIL no ADE\n"); return 1; }
	if (!all_lengths_ok()) { printf("FAIL line >160\n"); return 1; }
	if (uart_calls < 1) { printf("FAIL uart not called\n"); return 1; }
	printf("PASS arm debug %d lines, max<=159, UART busy retried\n", line_count);
	printf("RESULT OK\n");
	return 0;
}
