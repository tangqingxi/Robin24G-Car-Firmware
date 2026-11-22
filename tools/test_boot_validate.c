/*
 * 无 OTA bootloader 启动校验：合法、坏 CRC、越界长度、坏启动标记。
 */
#include <stdio.h>
#include "boot_validate.h"

int main(void)
{
	if (!boot_validate_header(START_UP_FLAG, 24756, 1)) {
		printf("FAIL valid image rejected\n");
		return 1;
	}
	if (boot_validate_header(START_UP_FLAG, 24756, 0)) {
		printf("FAIL bad crc accepted\n");
		return 1;
	}
	if (boot_validate_header(START_UP_FLAG, BOOT_APP_MAX_SIZE + 1, 1)) {
		printf("FAIL oversized accepted\n");
		return 1;
	}
	if (boot_validate_header(0x12345678, 24756, 1)) {
		printf("FAIL bad startup flag accepted\n");
		return 1;
	}
	if (boot_validate_header(START_UP_FLAG, 0x10, 1)) {
		printf("FAIL undersized accepted\n");
		return 1;
	}
	printf("PASS boot valid/bad-crc/oversize/flag/size checks\n");
	printf("RESULT OK\n");
	return 0;
}
