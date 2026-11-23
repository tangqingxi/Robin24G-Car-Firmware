/*
 * 真实 FlightLoop + FlightLog 集成：只在主循环调用 FlightLoop_Process 和
 * FlightLog_PollCommand，不手动调用 FlightLog_OnSample/OnEvent。
 * 启动、稳定、外部 BDT 命令、采样、急停后写 dump 给 Python 导出。
 */
#include <stdio.h>
#include <string.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "FlightLog.h"

unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static unsigned char flash_mem[0x20000];

void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	memcpy(buf, &flash_mem[addr], len);
}

void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	memcpy(&flash_mem[addr], buf, len);
}

void flash_erase_sector(unsigned int addr)
{
	memset(&flash_mem[addr], 0xFF, 4096);
}

u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 3; return 0; }
u8 t3488_read_6axis(short *a, short *g)
{
	a[0] = -22; a[1] = 8; a[2] = 2100;
	g[0] = g[1] = g[2] = 0;
	return 0;
}

static unsigned int g_now;
u32 clock_time(void) { return g_now; }

int main(void)
{
	unsigned int i;
	int ready = 0;

	memset(flash_mem, 0xFF, sizeof(flash_mem));
	FlightLoop_Init();
	for (i = 0; i < 140; i++) {
		g_now += FLIGHT_PERIOD_TICKS;
		FlightLoop_Process();
		if (FlightLoop_StartupTestReady()) {
			ready = 1;
		}
	}
	if (!ready) {
		printf("FAIL startup not ready\n");
		return 1;
	}

	// 先给一包合法低油门，建立链路；再短按“演示”键 0x0004。
	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0000);
	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, FLIGHT_LOG_KEY_START);
	for (i = 0; i < 400; i++) {
		g_now += FLIGHT_PERIOD_TICKS;
		if ((i % 20) == 0) {
			FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0000);
		}
		FlightLoop_Process();
		if (FlightLog_State() == FLIGHT_LOG_PREP && FlightLoop_CanEraseLog()) {
			FlightLog_BeginPrepare();
		}
		if (FlightLoop_CanWriteLog()) {
			FlightLog_Service(1);
		}
	}
	if (!FlightLog_Active()) {
		printf("FAIL real log not active\n");
		return 1;
	}

	// 0001：FlightLoop 先清 PWM，再写事件并 Stop。
	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x1000, 0x1000, 0x0001);
	FlightLog_Service(1);
	for (i = 0; i < 12 && FlightLog_State() == FLIGHT_LOG_ST_CLOSING; i++) {
		FlightLog_Service(1);
	}
	if (FlightLog_Active()) {
		printf("FAIL log not stopped by estop\n");
		return 1;
	}
	{
		FILE *f = fopen("flight_log_integration.bin", "wb");
		if (!f) {
			printf("FAIL dump open\n");
			return 1;
		}
		fwrite(&flash_mem[FLIGHT_LOG_BASE], 1,
			   FLIGHT_LOG_END - FLIGHT_LOG_BASE, f);
		fclose(f);
	}
	printf("PASS real FlightLoop log dump\n");
	return 0;
}
