/* 主机回归：真实 FlightLog_BenchStep 状态机写入两个扇区，核对 24 条/CRC。 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "FlightLog.h"
#include "FlightConfig.h"

static unsigned char flash_mem[0x20000];
static unsigned int now;
static char lines[8][96];
static int nlines;

u32 clock_time(void) { now += 100; return now; }
void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf) { memcpy(buf, &flash_mem[addr], len); }
static void flash_write_stub(unsigned int addr, unsigned int len, unsigned char *buf)
{
	memcpy(&flash_mem[addr], buf, len);
}
void (*flash_write_page)(unsigned int, unsigned int, unsigned char *) = flash_write_stub;
void flash_erase_sector(unsigned int addr) { memset(&flash_mem[addr], 0xFF, 4096); }
void UartDbgSend(unsigned char *dat, unsigned char len)
{
	if (nlines < 8) {
		int n = len < 95 ? len : 95;
		memcpy(lines[nlines], dat, n);
		lines[nlines][n] = 0;
		nlines++;
	}
}

static unsigned short crc16_update(unsigned short crc, const unsigned char *p, unsigned int n)
{
	unsigned int i;

	while (n--) {
		crc ^= (unsigned short)(*p++) << 8;
		for (i = 0; i < 8; i++) {
			crc = (crc & 0x8000) ? (unsigned short)((crc << 1) ^ 0x1021)
								 : (unsigned short)(crc << 1);
		}
	}
	return crc;
}

static int check_sector(unsigned int sec, unsigned short session)
{
	unsigned int base = FLIGHT_LOG_BASE + sec * 4096u;
	unsigned short hdr[8];
	unsigned short pcrc = 0xFFFF;
	unsigned int i;

	memcpy(hdr, &flash_mem[base], 16);
	if (hdr[0] != 0x4731 || hdr[1] != 0x4C4F || hdr[2] != 2
		|| hdr[3] != session || hdr[4] != sec || hdr[5] != 24) {
		return 0;
	}
	if (crc16_update(0xFFFF, &flash_mem[base], 14) != hdr[7]) return 0;
	for (i = 0; i < 24; i++) {
		unsigned int off = base + 256u + (i / 4u) * 256u + (i % 4u) * 56u;
		unsigned char *r = &flash_mem[off];
		unsigned short crc;

		memcpy(&crc, &r[54], 2);
		if (crc != crc16_update(0xFFFF, r, 54)) return 0;
		pcrc = crc16_update(pcrc, r, 56);
	}
	if (pcrc != hdr[6]) return 0;
	return 1;
}

int main(void)
{
	unsigned int steps = 0;

	memset(flash_mem, 0xFF, sizeof(flash_mem));
	FlightLog_BenchStart();
	while (steps++ < 100) {
		now += FLIGHT_PERIOD_TICKS;
		FlightLog_BenchStep(now + FLIGHT_PERIOD_TICKS);
	}
	if (!check_sector(10, 0xBA00u) || !check_sector(11, 0xBB00u)) {
		printf("FAIL sector/record/page CRC\n");
		return 1;
	}
	{
		FILE *f = fopen("flash_ab_host.bin", "wb");
		if (!f) { printf("FAIL host bin\n"); return 1; }
		fwrite(&flash_mem[FLIGHT_LOG_BASE], 1, 13u * 4096u, f);
		fclose(f);
	}

	if (nlines != 3) {
		printf("FAIL summary lines=%d\n", nlines);
		return 1;
	}
	{
		int a = 0, b = 0, i;

		for (i = 0; i < nlines; i++) {
			if (strstr(lines[i], "FBA n=24 w=24")) a = 1;
			if (strstr(lines[i], "FBB n=24 w=6")) b = 1;
		}
		if (!a || !b) {
			printf("FAIL summary lines=%d\n", nlines);
			return 1;
		}
	}
	printf("PASS flash A/B host: A=24 B=24 CRC OK\n");
	printf("RESULT OK\n");
	return 0;
}
