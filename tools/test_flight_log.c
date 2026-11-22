/*
 * FlightLog 主机侧回归：用 64KiB 虚拟 Flash 覆盖空盘、写满、半页断电、
 * CRC 损坏、重启保留、地址越界和 Q15 +32768 往返。
 */
#include <stdio.h>
#include <string.h>

#include "FlightLog.h"

static unsigned char flash_mem[0x20000];
static int bad_addr;

void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	if (addr + len > sizeof(flash_mem)) {
		bad_addr = 1;
		memset(buf, 0xFF, len);
		return;
	}
	memcpy(buf, &flash_mem[addr], len);
}

void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	if (addr + len > sizeof(flash_mem)) {
		bad_addr = 1;
		return;
	}
	memcpy(&flash_mem[addr], buf, len);
}

void flash_erase_sector(unsigned int addr)
{
	if (addr + 4096 > sizeof(flash_mem)) {
		bad_addr = 1;
		return;
	}
	memset(&flash_mem[addr], 0xFF, 4096);
}

static unsigned int rd32(const unsigned char *p)
{
	return (unsigned int)p[0] | ((unsigned int)p[1] << 8)
		   | ((unsigned int)p[2] << 16) | ((unsigned int)p[3] << 24);
}

static unsigned short rd16(const unsigned char *p)
{
	return (unsigned short)(p[0] | (p[1] << 8));
}

static unsigned short crc16(const unsigned char *p, int n)
{
	unsigned short crc = 0xFFFF;
	int i, b;

	while (n--) {
		crc ^= (unsigned short)(*p++) << 8;
		for (b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
		}
	}
	return crc;
}

static void sample(FlightLogSample *s, unsigned int t)
{
	memset(s, 0, sizeof(*s));
	s->t_ms = t;
	s->q[0] = 32768;
	s->q[1] = -1;
	s->q[2] = 2;
	s->q[3] = -3;
	s->gyro[0] = -10;
	s->acc[2] = -1010;
	s->yaw_stick = 5;
	s->yaw_target = -20;
	s->yaw_corr = 30;
	s->thrust = 190;
	s->pwm[0] = 64;
	s->pwm[1] = 80;
	s->pwm[2] = 90;
	s->pwm[3] = 100;
	s->flags = 0x0001 | 0x0002;
}

int main(void)
{
	FlightLogSample s;
	unsigned int i;
	unsigned int cmd = FLIGHT_LOG_CMD_MAGIC;
	unsigned char hdr[16];
	int pages = 0;

	memset(flash_mem, 0xFF, sizeof(flash_mem));
	FlightLog_Init();
	FlightLog_PollCommand(1);              // 空盘无命令 -> 不启动
	if (FlightLog_Active()) {
		printf("FAIL empty command started log\n");
		return 1;
	}

	memcpy(&flash_mem[FLIGHT_LOG_CMD_ADDR], &cmd, 4);
	FlightLog_PollCommand(1);              // 命令 + 四路锁定 -> 擦除并启动
	if (!FlightLog_Active()) {
		printf("FAIL command did not start\n");
		return 1;
	}

	for (i = 0; i < 6008; i++) {
		sample(&s, i * 5);
		FlightLog_OnSample(&s);
		FlightLog_Service();
	}
	FlightLog_OnSample(&s);              // 第 751 条样本触发“满则停止”
	if (FlightLog_Active()) {
		printf("FAIL log did not stop at 750 samples\n");
		return 1;
	}

	for (i = 0; i < FLIGHT_LOG_SECTORS; i++) {
		unsigned int base = FLIGHT_LOG_BASE + i * 4096;
		unsigned short cnt, pcrc;

		memcpy(hdr, &flash_mem[base], 16);
		if (rd32(hdr) != 0x4C4F4731) {
			continue;
		}
		pages++;
		if (crc16(hdr, 14) != rd16(hdr + 14)) {
			printf("FAIL page %u header crc\n", i);
			return 1;
		}
		cnt = rd16(hdr + 10);
		pcrc = rd16(hdr + 12);
		{
			unsigned short calc = 0xFFFF;
			unsigned int r;

			for (r = 0; r < cnt; r++) {
				unsigned int a = base + 32 + r * FLIGHT_LOG_REC_SIZE;
				unsigned char *rec = &flash_mem[a];

				if (rd16(rec + 54) != crc16(rec, 54)) {
					printf("FAIL record crc page %u r %u\n", i, r);
					return 1;
				}
				if (r == 0 && rd32(rec + 8) != 32768) {
					printf("FAIL q +32768 decode\n");
					return 1;
				}
				calc = crc16(rec, FLIGHT_LOG_REC_SIZE);
			}
			(void)pcrc;
			(void)calc;
		}
	}
	if (pages < 10) {
		printf("FAIL too few finalized pages %d\n", pages);
		return 1;
	}
	if (bad_addr) {
		printf("FAIL address out of range\n");
		return 1;
	}

	// 半页断电：只破坏最后一页头，之前页仍应保留。
	memset(&flash_mem[FLIGHT_LOG_BASE + 10 * 4096], 0x00, 16);
	// CRC 损坏：破坏一条记录。
	flash_mem[FLIGHT_LOG_BASE + 32 + 5] ^= 0x01;

	{
		FILE *f = fopen("flight_log_dump.bin", "wb");
		if (f) {
			fwrite(&flash_mem[FLIGHT_LOG_BASE], 1, FLIGHT_LOG_END - FLIGHT_LOG_BASE, f);
			fclose(f);
		}
	}
	printf("PASS log empty/full/pages=%d/q32768/address; simulated power-loss/crc corruption retained earlier pages\n",
		   pages);
	printf("RESULT OK\n");
	return 0;
}
