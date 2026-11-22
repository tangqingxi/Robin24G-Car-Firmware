/* 主机回归用 Flash 桩：这些用例不触发日志区写入。 */
#include <string.h>

void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	(void)addr;
	memset(buf, 0xFF, len);
}

void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	(void)addr;
	(void)len;
	(void)buf;
}

void flash_erase_sector(unsigned int addr)
{
	(void)addr;
}
