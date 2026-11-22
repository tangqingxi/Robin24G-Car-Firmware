/*
 * 用真实 flash_fw_check 和 Flash 桩验证：
 * CRC 覆盖 fw_size-4，末尾 4 字节单独读，块<=256，所有读取在应用区内。
 */
#include <stdio.h>
#include <string.h>
#include "flash_fw_check.h"

#define APP_BASE 0x4000u
#define APP_MAX  0xB000u
static unsigned char flash_mem[0x20000];
static unsigned int read_count;
static int read_bad;
static int max_read;

static const unsigned long tbl[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	if (addr < APP_BASE || addr + len > APP_BASE + max_read || len > 256) {
		read_bad = 1;
	}
	read_count++;
	if (addr + len > sizeof(flash_mem)) {
		read_bad = 1;
		memset(buf, 0, len);
		return;
	}
	memcpy(buf, &flash_mem[addr], len);
}

unsigned long crc32_half_cal(unsigned long crc, unsigned char *input,
							 unsigned long *table, int len);

static unsigned int image_crc(const unsigned char *p, unsigned int n)
{
	unsigned char nib[512];
	unsigned long crc = 0xFFFFFFFF;
	unsigned int off = 0;

	while (off < n) {
		unsigned int k = n - off > 256 ? 256 : n - off;
		unsigned int i;

		for (i = 0; i < k; i++) {
			nib[i * 2] = p[off + i] & 0x0F;
			nib[i * 2 + 1] = p[off + i] >> 4;
		}
		crc = crc32_half_cal(crc, nib, (unsigned long *)tbl, (int)(k << 1));
		off += k;
	}
	return (unsigned int)crc;
}

static void make_image(unsigned int total, unsigned int seed)
{
	unsigned int i;
	unsigned int crc;

	memset(&flash_mem[APP_BASE], 0, total);
	for (i = 0; i < total - 4; i++) {
		flash_mem[APP_BASE + i] = (unsigned char)(i * 37 + seed);
	}
	memcpy(&flash_mem[APP_BASE + 0x18], &total, 4);
	crc = image_crc(&flash_mem[APP_BASE], total - 4);
	memcpy(&flash_mem[APP_BASE + total - 4], &crc, 4);
}

static int check_size(unsigned int total)
{
	unsigned int i;

	make_image(total, 11);
	max_read = total;
	read_count = 0;
	read_bad = 0;
	if (flash_fw_check(0xffffffff, APP_BASE) != 0) {
		printf("FAIL valid size 0x%X\n", total);
		return 1;
	}
	if (read_bad) {
		printf("FAIL read outside app for size 0x%X\n", total);
		return 1;
	}
	// 坏 CRC
	flash_mem[APP_BASE + 17] ^= 1;
	if (flash_fw_check(0xffffffff, APP_BASE) == 0) {
		printf("FAIL bad crc accepted 0x%X\n", total);
		return 1;
	}
	(void)i;
	return 0;
}

int main(void)
{
	unsigned int sizes[] = {0x100, 0x104, 0x108, APP_MAX};
	unsigned int i;
	unsigned int total = 0x10;
	FILE *f;

	for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		if (check_size(sizes[i])) {
			return 1;
		}
	}

	// 越界长度：header 写 0xB004，函数必须直接拒绝且不读数据区。
	memset(flash_mem, 0, sizeof(flash_mem));
	total = APP_MAX + 4;
	memcpy(&flash_mem[APP_BASE + 0x18], &total, 4);
	max_read = APP_MAX;
	read_count = 0;
	read_bad = 0;
	if (flash_fw_check(0xffffffff, APP_BASE) == 0) {
		printf("FAIL oversize accepted\n");
		return 1;
	}
	if (read_bad) {
		printf("FAIL oversize caused out-of-range read\n");
		return 1;
	}

	// 真实日志应用镜像：整包 CRC 必须通过。
	f = fopen("..\\firmware_delivery\\candidates\\Robin_24G_Car_flight_control_log.bin", "rb");
	if (f) {
		size_t got;
		memset(flash_mem, 0, sizeof(flash_mem));
		got = fread(&flash_mem[APP_BASE], 1, sizeof(flash_mem) - APP_BASE, f);
		fclose(f);
		memcpy(&total, &flash_mem[APP_BASE + 0x18], 4);
		max_read = total;
		read_count = 0;
		read_bad = 0;
		if (flash_fw_check(0xffffffff, APP_BASE) != 0 || read_bad) {
			printf("FAIL real app crc size=%u got=%zu\n", total, got);
			return 1;
		}
		printf("PASS real app crc size=%u reads=%u\n", total, read_count);
	}

	printf("PASS flash_fw_check sizes 0x100/0x104/0x108/0xB000, bad crc, oversize, app bounds, reads<=256\n");
	printf("RESULT OK\n");
	return 0;
}
