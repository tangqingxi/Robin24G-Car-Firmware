/* 气压 BDT 诊断回归：初始化提前、就绪后设基准、漏槽不重复、锁定才保存。 */
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
		*raw = 40000 + (s32)((now / 24000u - 5000u) / 100u);
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

static void start_sampling(void)
{
	now = 0;
	BaroBdt_Process();                 // 立即启动气压计并等 5 秒
	now = 5000u * 24000u;
	BaroBdt_Process();                 // 进入采样
	BaroBdt_Process();                 // 第 0 槽
}

static void run_samples(void)
{
	unsigned int i;

	start_sampling();
	for (i = 1; i < 300; i++) {
		now += 100u * 24000u;
		BaroBdt_Process();
		if (i == 150 && (erase_count || write_count)) {
			printf("FAIL flash before sampling end\n");
			return;
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
	write_flash_file("baro_read.bin");
	if (erase_count != 1 || write_count == 0) {
		printf("FAIL complete flash erase=%u write=%u\n", erase_count, write_count);
		return 1;
	}
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE || BaroBdt_GetCount() != 300) {
		printf("FAIL complete state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS complete 300 samples, erase=%u writes=%u bytes=%u\n",
		   erase_count, write_count, flash_used);

	// 首读未就绪：第 0 槽留空，后续从第 1 槽开始，基准必须来自真实样本。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	ready_fail_left = 1;
	run_samples();
	write_flash_file("baro_read_initial.bin");
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE || BaroBdt_GetCount() != 299) {
		printf("FAIL initial state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS first not-ready slot skipped, count=299\n");

	// 漏掉 100ms 槽：槽 1 留空，后续槽只写一次，总数 299。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	now = 0;
	BaroBdt_Process();
	now = 5000u * 24000u;
	BaroBdt_Process();
	BaroBdt_Process();                 // 槽 0
	now += 200u * 24000u;              // 跳到槽 2，槽 1 漏掉
	BaroBdt_Process();
	for (now += 100u * 24000u; now < 5000u * 24000u + 300u * 100u * 24000u;
		 now += 100u * 24000u) {
		BaroBdt_Process();
	}
	write_flash_file("baro_read_miss.bin");
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE || BaroBdt_GetCount() != 299) {
		printf("FAIL miss state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS missed slot stays empty, count=299\n");

	// 采样期间中途解锁：结束后必须取消，不能擦写。
	BaroBdt_TestReset();
	erase_count = 0;
	write_count = 0;
	now = 0;
	BaroBdt_Process();
	now = 5000u * 24000u;
	BaroBdt_Process();
	BaroBdt_Process();
	armed = 1;
	now += 100u * 24000u;
	BaroBdt_Process();
	armed = 0;
	while (BaroBdt_GetState() == BARO_BDT_ST_SAMPLING) {
		now += 100u * 24000u;
		BaroBdt_Process();
	}
	if (BaroBdt_GetState() != BARO_BDT_ST_CANCELED
		|| erase_count || write_count) {
		printf("FAIL cancel state=%u erase=%u write=%u\n",
			   BaroBdt_GetState(), erase_count, write_count);
		return 1;
	}
	printf("PASS unlock cancels save\n");

	// I2C 失败只影响对应槽，不阻止完整记录。
	BaroBdt_TestReset();
	read_fail_left = 3;
	run_samples();
	if (BaroBdt_GetState() != BARO_BDT_ST_DONE || BaroBdt_GetCount() != 297) {
		printf("FAIL i2c state=%u count=%u\n",
			   BaroBdt_GetState(), BaroBdt_GetCount());
		return 1;
	}
	printf("PASS I2C failure counted, count=297\n");
	printf("RESULT OK\n");
	return 0;
}
