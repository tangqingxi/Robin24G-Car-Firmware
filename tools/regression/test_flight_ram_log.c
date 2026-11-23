/* RAM 飞行缓存回归：K3 开始/结束、解锁无擦写、回绕顺序、锁定落盘可导出。 */
#include <stdio.h>
#include <string.h>
#include "FlightLog.h"
#include "FlightConfig.h"

static unsigned char flash_mem[0x20000];
static unsigned int now, writes, erases;
u32 clock_time(void) { return now; }
void flash_read_page(unsigned int a, unsigned int n, unsigned char *b) { memcpy(b, &flash_mem[a], n); }
void flash_write_page(unsigned int a, unsigned int n, unsigned char *b) { writes++; memcpy(&flash_mem[a], b, n); }
void flash_erase_sector(unsigned int a) { erases++; memset(&flash_mem[a], 0xFF, 4096); }

static unsigned short crc16(const unsigned char *p, int n)
{
	unsigned short c = 0xFFFF;
	int i, b;
	while (n--) {
		c ^= (unsigned short)(*p++) << 8;
		for (b = 0; b < 8; b++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
	}
	return c;
}
static unsigned short rd16(const unsigned char *p) { return (unsigned short)(p[0] | p[1] << 8); }
static unsigned int rd32(const unsigned char *p) { return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24; }

static void mk_sample(FlightLogSample *s, int i)
{
	memset(s, 0, sizeof(*s));
	s->flags = FLIGHT_LOG_F_ARMED;
	s->yaw_stick = (short)i;
	s->yaw_target = (short)(i * 2);
	s->yaw_corr = (short)(i * 3);
	s->gyro[2] = (short)(i * 4);
	s->thrust = (unsigned short)(100 + i);
	s->pwm[0] = s->pwm[1] = s->pwm[2] = s->pwm[3] = 200;
}

int main(void)
{
	FlightLogSample s;
	unsigned int i, k, n, samples = 0;
	int last_t = -1;

	memset(flash_mem, 0xFF, sizeof(flash_mem));
	FlightLog_Init();
	now = 1000000;
	FlightLog_OnKey(FLIGHT_LOG_KEY_START, 1, 0, 0);  // 第一次 K3：空中/锁定都允许
	if (FlightLog_State() != FLIGHT_LOG_REC) { printf("FAIL start state %u\n", FlightLog_State()); return 2; }
	if (writes || erases) { printf("FAIL start touched flash\n"); return 3; }

	for (i = 0; i < 60; i++) {
		mk_sample(&s, i);
		for (k = 0; k < 20; k++) {
			now += FLIGHT_PERIOD_TICKS;
			FlightLog_OnControlTick();
			FlightLog_OnSample(&s);
			FlightLog_Service(1);
		}
	}
	if (writes || erases) { printf("FAIL flash during armed %u/%u\n", writes, erases); return 4; }

	FlightLog_OnKey(0, 1, 0, 0);                     // 松开 K3
	FlightLog_OnKey(FLIGHT_LOG_KEY_START, 1, 0, 0);  // 第二次 K3：冻结
	if (FlightLog_State() != FLIGHT_LOG_ST_CLOSING) { printf("FAIL freeze state %u\n", FlightLog_State()); return 5; }
	if (writes || erases) { printf("FAIL freeze touched flash\n"); return 6; }

	for (i = 0; i < 200 && FlightLog_State() != FLIGHT_LOG_ST_END; i++) {
		now += FLIGHT_PERIOD_TICKS;
		FlightLog_OnControlTick();
		FlightLog_Service(1);
	}
	if (FlightLog_State() != FLIGHT_LOG_ST_END) { printf("FAIL flush state\n"); return 7; }
	if (!writes || !erases) { printf("FAIL no flash after landing\n"); return 8; }

	n = rd16(&flash_mem[FLIGHT_LOG_BASE + 10]);
	if (!n || n > 60) { printf("FAIL page count %u\n", n); return 9; }
	for (i = 0; i < n; i++) {
		unsigned int off = FLIGHT_LOG_BASE + 256 + (i / 4) * 256 + (i % 4) * 56;
		unsigned char *r = &flash_mem[off];
		unsigned short flags;
		int type;
		if (rd16(&r[54]) != crc16(r, 54)) { printf("FAIL crc rec %u\n", i); return 10; }
		flags = rd16(&r[52]);
		type = flags >> 12;
		if (type == 0) {
			int t = (int)rd32(&r[0]);
			samples++;
			if (t <= last_t) { printf("FAIL time order %d %d\n", last_t, t); return 11; }
			last_t = t;
		}
	}
	if (samples != 34) { printf("FAIL samples=%u\n", samples); return 12; }
	{
		FILE *f = fopen("ram_log_host.bin", "wb");
		if (!f) return 13;
		fwrite(&flash_mem[FLIGHT_LOG_BASE], 1, FLIGHT_LOG_END - FLIGHT_LOG_BASE, f);
		fclose(f);
	}
	printf("PASS RAM log samples=%u writes=%u erases=%u window=3.4s\n",
		   samples, writes, erases);
	printf("RESULT OK\n");
	return 0;
}
