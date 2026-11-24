/* 定点温补地面记录：90×1s，三路对齐，锁定后保存。 */
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
static unsigned char pressure_config;
static unsigned char continuous_started;
static unsigned char factory_cal[LPC2623_CALIB_BYTES];
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
	if (pressure_config != 0x03 || !continuous_started) return 1;
	memcpy(block, factory_cal, sizeof(factory_cal));
	*cfg = pressure_config;
	return 0;
}
void LPC2623_rateset(unsigned char a, unsigned char b, unsigned char c)
{ if (a == PRESSURE_SENSOR && b == 1 && c == 8) pressure_config = 0x03; }
void LPC2623_start_continuous(unsigned char m) { continuous_started = m == CONTINUOUS_P_AND_T; }
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

static void begin(void)
{
	pressure_config = 0;
	continuous_started = 0;
	now = 0;
	BaroFixedBdt_Process();
	now = 5000u * 24000u;
	BaroFixedBdt_Process();
	BaroFixedBdt_Process();
}

static void run(void)
{
	unsigned int i;

	begin();
	for (i = 1; i < BARO_FIXED_SLOTS; i++) {
		now += 1000u * 24000u;
		BaroFixedBdt_Process();
	}
}

int main(void)
{
	FILE *fp;
	size_t count;

	setvbuf(stdout, NULL, _IONBF, 0);
	fp = fopen("v4_actual.bin", "rb");
	if (!fp) return 1;
	count = fseek(fp, 48, SEEK_SET) ? 0 : fread(factory_cal, 1, sizeof(factory_cal), fp);
	fclose(fp);
	if (count != sizeof(factory_cal)) return 1;
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
	run();
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
	erase_count = 0;
	write_count = 0;
	begin();
	armed = 1; now += 1000u * 24000u; BaroFixedBdt_Process(); armed = 0;
	while (BaroFixedBdt_GetState() == BARO_FIXED_ST_SAMPLING) {
		now += 1000u * 24000u; BaroFixedBdt_Process();
	}
	if (BaroFixedBdt_GetState() != BARO_FIXED_ST_CANCELED || erase_count || write_count) {
		printf("FAIL cancel state=%u\n", BaroFixedBdt_GetState());
		return 1;
	}
	printf("PASS unlock cancels save\n");
	printf("RESULT OK\n");
	return 0;
}
