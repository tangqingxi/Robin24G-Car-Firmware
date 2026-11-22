#ifndef BOOT_VALIDATE_H
#define BOOT_VALIDATE_H

#include "ota_config.h"

// app 区 0x4000..0xEFFF；最大 bin 0xB000 字节。
#define BOOT_APP_MAX_SIZE  (0xF000 - FW_ADDRESS)

// 启动标记、长度、边界和 CRC 四项全通过才允许跳转。
static inline unsigned char boot_validate_header(unsigned int startup_flag,
												 unsigned int fw_size,
												 unsigned char crc_ok)
{
	if (startup_flag != START_UP_FLAG) {
		return 0;
	}
	if (fw_size < 0x20 || fw_size > BOOT_APP_MAX_SIZE) {
		return 0;
	}
	return crc_ok ? 1 : 0;
}

#endif /* BOOT_VALIDATE_H */
