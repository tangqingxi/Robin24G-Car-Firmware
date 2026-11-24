/* 定点温补地面记录：100×1s，三路对齐，锁定后保存。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "BaroFixedBdt.h"
#include "PressSen_Lpc2623.h"

static unsigned int now;
static unsigned int erase_count;
static unsigned int write_count;
static unsigned int t_ready_left;
static unsigned int p_fail_left;
static unsigned int t_fail_left;
static unsigned char armed;
static unsigned char flash_mem[4096];

u32 BaroFixedBdt_TestClock(void) { return now; }

u8 LPC2623_CheckId(void) { return 0; }
u8 LPC2623_IsPressureReady(void) { return p_fail_left ? 0 : 1; }
u8 LPC2623_IsTempReady(void)
{
	if (t_ready_left) { t_ready_left--; return 0; }
	return 1;
}
u8 LPC2623_ReadCalibration(u8 block[LPC2623_CALIB_BYTES], u8 *cfg)
{
	(void)block; (void)cfg;
	return 1;
}
void LPC2623_rateset(unsigned char a, unsigned char b, unsigned char c)
{ (void)a; (void)b; (void)c; }
void LPC2623_start_continuous(unsigned char m) { (void)m; }
u8 LPC2623_ReadRaw(s32 *raw)
{
	if (p_fail_left) { p_fail_left--; return 1; }
	if (raw) *raw = 1390000 + (s32)(now / 24000u / 1000u) * 10;
	return 0;
}
u8 LPC2623_ReadTempRaw(s32 *raw)
{
	if (t_fail_left) { t_fail_left--; return 1; }
	if (raw) *raw = 93000 - (s32)(now / 24000u / 1000u);
	return 0;
}
void FlightLoop_ResetDiagStats(void) {}
void FlightLoop_GetDiagStatus(u8 *a, u16 pwm[4], u16 *mg, u16 *mt)
{
	if (a) *a = armed;
	if (pwm) pwm[0] = pwm[1] = pwm[2] = pwm[3] = 0;
	if (mg) *mg = 11;
	if (mt) *mt = 1;
}
void BaroFixedBdt_TestFlashErase(unsigned long a)
{
	(void)a; erase_count++; memset(flash_mem, 0xFF, sizeof(flash_mem));
}
void BaroFixedBdt_TestFlashWrite(unsigned long a, unsigned long n, u8 *d)
{
	write_count++;
	if (a >= BARO_FIXED_SECTOR && a - BARO_FIXED_SECTOR + n <= sizeof(flash_mem)) {
		memcpy(flash_mem + (a - BARO_FIXED_SECTOR), d, n);
	}
}
void BaroFixedBdt_TestFlashRead(unsigned long a, unsigned long n, u8 *d)
{
	(void)a; memcpy(d, flash_mem, n);
}

static void write_file(const char *name)
{
	FILE *fp = fopen(name, "wb");
	if (!fp || fwrite(flash_mem, 1, sizeof(flash_mem), fp) != sizeof(flash_mem)) {
		printf("FAIL write %s\n", name);
		if (fp) fclose(fp);
		return;
	}
	fclose(fp);
}

static void run(void)
{
	unsigned int i;
	BaroFixedCal_t cal;
	BaroFixed_TestSetV4Cal(&cal);
	BaroFixedBdt_TestCal(&cal);
	BaroFixedBdt_TestReset();
	BaroFixedBdt_TestCal(&cal);
	now = 0;
	BaroFixedBdt_Process();
	now = 5000u * 24000u;
	BaroFixedBdt_Process();
	BaroFixedBdt_Process();
	for (i = 1; i < BARO_FIXED_SLOTS; i++) {
		now += 1000u * 24000u;
		BaroFixedBdt_Process();
	}
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	run();
	write_file("baro_fixed_read.bin");
	if (BaroFixedBdt_GetState() != BARO_FIXED_ST_DONE
		|| BaroFixedBdt_GetPressureCount() != BARO_FIXED_SLOTS
		|| BaroFixedBdt_GetTempCount() != BARO_FIXED_SLOTS
		|| BaroFixedBdt_GetFixedCount() != BARO_FIXED_SLOTS) {
		printf("FAIL complete state=%u p=%u t=%u f=%u\n",
			   BaroFixedBdt_GetState(), BaroFixedBdt_GetPressureCount(),
			   BaroFixedBdt_GetTempCount(), BaroFixedBdt_GetFixedCount());
		return 1;
	}
	printf("PASS fixed complete p=90 t=90 f=90\n");

	// 缺温度：该槽温补无效，但气压槽保留。
	BaroFixedBdt_TestReset();
	t_ready_left = 1;
	{
		BaroFixedCal_t cal;
		BaroFixed_TestSetV4Cal(&cal);
		BaroFixedBdt_TestCal(&cal);
	}
	now = 0; BaroFixedBdt_Process();
	now = 5000u * 24000u; BaroFixedBdt_Process(); BaroFixedBdt_Process();
	for (unsigned int i = 1; i < BARO_FIXED_SLOTS; i++) {
		now += 1000u * 24000u; BaroFixedBdt_Process();
	}
	if (BaroFixedBdt_GetState() != BARO_FIXED_ST_DONE
		|| BaroFixedBdt_GetPressureCount() != BARO_FIXED_SLOTS
		|| BaroFixedBdt_GetFixedCount() != BARO_FIXED_SLOTS - 1) {
		printf("FAIL missing t p=%u f=%u\n",
			   BaroFixedBdt_GetPressureCount(), BaroFixedBdt_GetFixedCount());
		return 1;
	}
	printf("PASS missing temperature leaves fixed slot empty\n");

	// 解锁取消。
	BaroFixedBdt_TestReset();
	{
		BaroFixedCal_t cal;
		BaroFixed_TestSetV4Cal(&cal);
		BaroFixedBdt_TestCal(&cal);
	}
	now = 0; BaroFixedBdt_Process();
	now = 5000u * 24000u; BaroFixedBdt_Process(); BaroFixedBdt_Process();
	armed = 1; now += 1000u * 24000u; BaroFixedBdt_Process(); armed = 0;
	while (BaroFixedBdt_GetState() == BARO_FIXED_ST_SAMPLING) {
		now += 1000u * 24000u; BaroFixedBdt_Process();
	}
	if (BaroFixedBdt_GetState() != BARO_FIXED_ST_CANCELED) {
		printf("FAIL cancel state=%u\n", BaroFixedBdt_GetState());
		return 1;
	}
	printf("PASS unlock cancels save\n");
	printf("RESULT OK\n");
	return 0;
}
