/* 气压 BDT 回归：初始化提前、就绪后设基准、漏槽不重复、tick 回绕。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "BaroBdt.h"

static unsigned int now;
static unsigned int erase_count;
static unsigned int write_count;
static unsigned int read_fail_left;
static unsigned int ready_fail_left;
static unsigned char armed;
static unsigned int flash_used;
static unsigned char flash_mem[4096];

#if BARO_BDT_SAMPLE_MS == 500
#define READ_FILE       "baro_read_drift.bin"
#define INITIAL_FILE    "baro_read_initial_drift.bin"
#define MISS_FILE       "baro_read_miss_drift.bin"
#else
#define READ_FILE       "baro_read.bin"
#define INITIAL_FILE    "baro_read_initial.bin"
#define MISS_FILE       "baro_read_miss.bin"
#endif

u32 BaroBdt_TestClock(void) { return now; }

u8 LPC2623_CheckId(void) { return 0; }
u8 LPC2623_IsPressureReady(void)
{
	if (ready_fail_left) {
		ready_fail_left--;
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
	if (read_fail_left) {
		read_fail_left--;
		return 1;
	}
	if (raw) {
		*raw = 40000 + ((s32)(now / 24000u) - 5000) / BARO_BDT_SAMPLE_MS;
	}
	return 0;
}

void FlightLoop_ResetDiagStats(void) {}
void FlightLoop_GetDiagStatus(u8 *a, u16 pwm[4], u16 *max_gap, u16 *missed)
{
	if (a) *a = armed;
	if (pwm) pwm[0] = pwm[1] = pwm[2] = pwm[3] = 0;
	if (max_gap) *max_gap = 17;
	if (missed) *missed = 2;
}

void BaroBdt_TestFlashErase(unsigned long addr)
{
	(void)addr;
	erase_count++;
	memset(flash_mem, 0xFF, sizeof(flash_mem));
}
void BaroBdt_TestFlashWrite(unsigned long addr, unsigned long len, u8 *data)
{
	write_count++;
	flash_used += (unsigned int)len;
	if (addr >= BARO_BDT_SECTOR
		&& addr - BARO_BDT_SECTOR + len <= sizeof(flash_mem)) {
		memcpy(flash_mem + (addr - BARO_BDT_SECTOR), data, len);
	}
}
void BaroBdt_TestFlashRead(unsigned long addr, unsigned long len, u8 *data)
{
	(void)addr;
	memcpy(data, flash_mem, len);
}

static void write_flash_file(const char *name)
{
	FILE *fp = fopen(name, "wb");
	if (!fp || fwrite(flash_mem, 1, sizeof(flash_mem), fp) != sizeof(flash_mem)) {
		printf("FAIL write %s\n", name);
		if (fp) fclose(fp);
		return;
	}
	fclose(fp);
}

static unsigned int initial_tick(void)
{
#if BARO_BDT_SAMPLE_MS == 500
	return 0xFFFF0000u;               // 跨越 32 位 tick 回绕
#else
	return 0;
#endif
}

static void begin(void)
{
	now = initial_tick();
	BaroBdt_Process();                // 立即启动气压计并等待 5 秒
	now += 5000u * 24000u;
	BaroBdt_Process();                // 进入采样
	BaroBdt_Process();                // 槽 0
}

static void run_samples(void)
{
	unsigned int i;

	begin();
	for (i = 1; i < BARO_BDT_MAX_SAMPLES; i++) {
		now += BARO_BDT_SAMPLE_MS * 24000u;
		BaroBdt_Process();
		if (i < 3) {
		}
	}
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	flash_used = 0;
	run_samples();
	write_flash_file(READ_FILE);
	if (erase_count != 1 || write_count == 0) {
		printf("FAIL complete flash erase=%u write=%u\n", erase_count, write_count);
		return 1;
	}
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE
		|| BaroBdt_GetCount() != BARO_BDT_MAX_SAMPLES) {
		printf("FAIL complete state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS complete %u samples, erase=%u writes=%u bytes=%u\n",
		   BARO_BDT_MAX_SAMPLES, erase_count, write_count, flash_used);

	// 首读未就绪：首槽留空，后续从真实样本建立基准。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	ready_fail_left = 1;
	run_samples();
	write_flash_file(INITIAL_FILE);
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE
		|| BaroBdt_GetCount() != BARO_BDT_MAX_SAMPLES - 1) {
		printf("FAIL initial state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS first not-ready slot skipped, count=%u\n",
		   BaroBdt_GetCount());

	// 漏槽：槽 1 留空，后续槽只写一次。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	{
		unsigned int t0 = initial_tick();

		now = t0;
		BaroBdt_Process();
		now = t0 + 5000u * 24000u;
		BaroBdt_Process();
		BaroBdt_Process();            // 槽 0
		now += 2u * BARO_BDT_SAMPLE_MS * 24000u;
		BaroBdt_Process();            // 槽 2，槽 1 漏掉
		for (now += BARO_BDT_SAMPLE_MS * 24000u;
			 now < t0 + 5000u * 24000u
				   + 300u * BARO_BDT_SAMPLE_MS * 24000u;
			 now += BARO_BDT_SAMPLE_MS * 24000u) {
			BaroBdt_Process();
		}
	}
	write_flash_file(MISS_FILE);
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE
		|| BaroBdt_GetCount() != BARO_BDT_MAX_SAMPLES - 1) {
		printf("FAIL miss state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS missed slot stays empty, count=%u\n", BaroBdt_GetCount());

	// 采样期间解锁：取消保存。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	begin();
	armed = 1;
	now += BARO_BDT_SAMPLE_MS * 24000u;
	BaroBdt_Process();
	armed = 0;
	while (BaroBdt_GetState() == BARO_BDT_ST_SAMPLING) {
		now += BARO_BDT_SAMPLE_MS * 24000u;
		BaroBdt_Process();
	}
	if (BaroBdt_GetState() != BARO_BDT_ST_CANCELED
		|| erase_count || write_count) {
		printf("FAIL cancel state=%u erase=%u write=%u\n",
			   BaroBdt_GetState(), erase_count, write_count);
		return 1;
	}
	printf("PASS unlock cancels save\n");

	// I2C 失败只影响对应槽。
	BaroBdt_TestReset();
	read_fail_left = 3;
	run_samples();
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE
		|| BaroBdt_GetCount() != BARO_BDT_MAX_SAMPLES - 3) {
		printf("FAIL i2c state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS I2C failure counted, count=%u\n", BaroBdt_GetCount());
	printf("RESULT OK\n");
	return 0;
}
