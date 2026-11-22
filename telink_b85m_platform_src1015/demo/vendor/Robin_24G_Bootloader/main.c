/********************************************************************************************************
 * @file     main.c
 *
 * @brief    This is the source file for KMD SDK
 *
 * @author	 KMD GROUP
 * @date         01,2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/

#include "app_public_config.h"
#include "toy_log.h"
#include "calibration.h"
#include "lib/include/pm/pm.h"

#include "watchdog.h"

#include "common.h"
#include "bootloader_ota/boot_copy_data.h"
#include "ota_config.h"
#include "sdk_version.h"
#include "flash_fw_check.h"
#include "boot_validate.h"


//volatile unsigned char sample_cnt=0;


/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	// OTA 已停用：bootloader 不再处理 2.4G OTA 收发中断。
}

#if (BOOTLOADER_ENABLE)
// 只校验 app 启动标记、长度、边界和 CRC；无效时停在安全循环，等待 BDT 恢复。
static u8 boot_app_is_valid(void)
{
	u32 startup_flag = 0;
	u32 fw_size = 0;

	flash_read_page(FW_ADDRESS + 0x8, 4, (u8 *)&startup_flag);
	flash_read_page(FW_ADDRESS + 0x18, 4, (u8 *)&fw_size);
	if (startup_flag != START_UP_FLAG) {
		return 0;
	}
	if (!boot_validate_header(startup_flag, fw_size, 1)) {
		return 0;                       // 先挡住坏标记/越界长度，避免 CRC 越界读
	}
	return boot_validate_header(startup_flag, fw_size,
								(flash_fw_check(0xffffffff, FW_ADDRESS) == 0));
}
#endif


/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */

int main (void)   //must on ramcode
{
	PLATFORM_INIT;

    CLOCK_INIT;

	//gpio_shutdown(GPIO_ALL);

#if (MODULE_32K_WATCHDOG_ENABLE)
	// app 软复位(start_reboot)后 32k 看门狗状态保留并继续运行，
	// bootloader 若不处理会在 OTA 接收阶段被看门狗反复复位。
	if (wd_32k_get_status())
	{
		wd_32k_clear_status();
	}
	wd_32k_set_interval_ms(WATCHDOG_INIT_TIMEOUT + 2048);
#endif

#if(DEBUG_MODE==1)
#if (DEBUG_BUS==DEBUG_IO)
	io_putchar_reinit();
#endif
#endif

	TOY_LOG("Robin_24G_Bootloader start up\n");
	TOY_LOG("Robin_24G_Bootloader FW_VERSION = 0x%02x, PID = 0x%08x\n", FW_VERSION, PID);

#if (BOOTLOADER_ENABLE)
#if (MODULE_32K_WATCHDOG_ENABLE)
	// app 整包 CRC 校验（flash_fw_check）耗时较长，先关看门狗防止误复位。
	wd_32k_stop();
#endif
	if (!boot_app_is_valid()) {
		// 无效 app 不跳转、不进 OTA；停在安全循环，等待 BDT 恢复。
		while (1) {
			wd_32k_clear_status();
		}
	}
#if (MODULE_32K_WATCHDOG_ENABLE)
	wd_32k_set_interval_ms(WATCHDOG_INIT_TIMEOUT + 2048);  // 重新使能看门狗
#endif

	// 启动标记、长度、边界、CRC 全部通过后才跳转 app。
	boot_copy_data(FW_ADDRESS);
#endif

	while(1)
	{		
		start_reboot();
	}
}
