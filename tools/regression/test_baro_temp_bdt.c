/* 气压+温度双通道 BDT 回归：150×1s、独立基准、通道就绪、写后导出。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "BaroTempBdt.h"

static unsigned int now;
static unsigned int erase_count;
static unsigned int write_count;
static unsigned int p_fail_left;
static unsigned int t_fail_left;
static unsigned int p_ready_left;
static unsigned int t_ready_left;
static unsigned char armed;
static unsigned char flash_mem[4096];

u32 BaroTempBdt_TestClock(void) { return now; }

u8 LPC2623_CheckId(void) { return 0; }
u8 LPC2623_IsPressureReady(void)
{
	if (p_ready_left) {
		p_ready_left--;
		return 0;
	}
	return 1;
}
u8 LPC2623_IsTempReady(void)
{
	if (t_ready_left) {
		t_ready_left--;
		return 0;
	}
	return 1;
}
void LPC2623_rateset(unsigned char sensor, unsigned char rate, unsigned char osr)
{
	(void)sensor; (void)rate; (void)osr;
}
void LPC2623_start_continuous(unsigned char mode) { (void)mode; }
u8 LPC2623_ReadRaw(s32 *raw)
{
	if (p_fail_left) {
		p_fail_left--;
		return 1;
	}
	if (raw) *raw = 40000 + ((s32)(now / 24000u) - 5000) / 1000;
	return 0;
}
u8 LPC2623_ReadTempRaw(s32 *raw)
{
	if (t_fail_left) {
		t_fail_left--;
		return 1;
	}
	if (raw) *raw = 50000 + ((s32)(now / 24000u) - 5000) / 1000;
	return 0;
}

void FlightLoop_ResetDiagStats(void) {}
void FlightLoop_GetDiagStatus(u8 *a, u16 pwm[4], u16 *max_gap, u16 *missed)
{
	if (a) *a = armed;
	if (pwm) pwm[0] = pwm[1] = pwm[2] = pwm[3] = 0;
	if (max_gap) *max_gap = 23;
	if (missed) *missed = 4;
}

void BaroTempBdt_TestFlashErase(unsigned long addr)
{
	(void)addr;
	erase_count++;
	memset(flash_mem, 0xFF, sizeof(flash_mem));
}
void BaroTempBdt_TestFlashWrite(unsigned long addr, unsigned long len, u8 *data)
{
	write_count++;
	if (addr >= BARO_TEMP_SECTOR
		&& addr - BARO_TEMP_SECTOR + len <= sizeof(flash_mem)) {
		memcpy(flash_mem + (addr - BARO_TEMP_SECTOR), data, len);
	}
}
void BaroTempBdt_TestFlashRead(unsigned long addr, unsigned long len, u8 *data)
{
	(void)addr;
	memcpy(data, flash_mem, len);
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
	now = 0;
	BaroTempBdt_Process();           // 启动气压+温度
	now += 5000u * 24000u;
	BaroTempBdt_Process();           // 进入采样
	BaroTempBdt_Process();           // 槽 0
}

static void run(void)
{
	unsigned int i;

	begin();
	for (i = 1; i < BARO_TEMP_SLOTS; i++) {
		now += 1000u * 24000u;
		BaroTempBdt_Process();
	}
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	BaroTempBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	run();
	write_file("baro_temp_read.bin");
	if (BaroTempBdt_GetState() != BARO_TEMP_ST_DONE
		|| BaroTempBdt_GetPressureCount() != 150
		|| BaroTempBdt_GetTempCount() != 150) {
		printf("FAIL complete state=%u p=%u t=%u\n",
			   BaroTempBdt_GetState(), BaroTempBdt_GetPressureCount(),
			   BaroTempBdt_GetTempCount());
		return 1;
	}
	printf("PASS complete p=150 t=150 erase=%u writes=%u\n",
		   erase_count, write_count);

	// 温度通道某一槽未就绪：该槽温度留空，气压仍有效。
	BaroTempBdt_TestReset();
	t_ready_left = 1;
	run();
	write_file("baro_temp_missing.bin");
	if (BaroTempBdt_GetState() != BARO_TEMP_ST_DONE
		|| BaroTempBdt_GetPressureCount() != 150
		|| BaroTempBdt_GetTempCount() != 149) {
		printf("FAIL missing state=%u p=%u t=%u\n",
			   BaroTempBdt_GetState(), BaroTempBdt_GetPressureCount(),
			   BaroTempBdt_GetTempCount());
		return 1;
	}
	printf("PASS one temperature slot empty\n");

	// 解锁取消保存。
	BaroTempBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	begin();
	armed = 1;
	now += 1000u * 24000u;
	BaroTempBdt_Process();
	armed = 0;
	while (BaroTempBdt_GetState() == BARO_TEMP_ST_SAMPLING) {
		now += 1000u * 24000u;
		BaroTempBdt_Process();
	}
	if (BaroTempBdt_GetState() != BARO_TEMP_ST_CANCELED
		|| erase_count || write_count) {
		printf("FAIL cancel state=%u erase=%u write=%u\n",
			   BaroTempBdt_GetState(), erase_count, write_count);
		return 1;
	}
	printf("PASS unlock cancels save\n");
	printf("RESULT OK\n");
	return 0;
}
