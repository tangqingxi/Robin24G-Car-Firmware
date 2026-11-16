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


//volatile unsigned char sample_cnt=0;


/**
 * @brief		This function serves to handle the interrupt of MCU
 * @param[in] 	none
 * @return 		none
 */
_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	irq_handle_private_1m();  // RF IRQ（OTA 收发）
}

#if (BOOTLOADER_ENABLE)
// OTA 流程已移到 bootloader（见 ota_boot.c）。
// boot_need_ota()：标记存在 OR app 区无有效固件 OR OTA 区有完整固件 -> 需要 OTA
// boot_ota_entry()：OTA 区有完整固件则重新搬运重启，否则进入 OTA 模式接收固件（不返回）
extern void boot_ota_entry(void);
extern u8   boot_need_ota(void);
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
	// boot_need_ota() 内含 app/OTA 两区整包 CRC 校验（flash_fw_check），耗时较长，
	// 先关看门狗防止校验期间被误复位；判断完立即重新打开。
	wd_32k_stop();
#endif
	u8 need_ota = boot_need_ota();
#if (MODULE_32K_WATCHDOG_ENABLE)
	wd_32k_set_interval_ms(WATCHDOG_INIT_TIMEOUT + 2048);  // 重新使能看门狗
#endif
	if (need_ota) {
		// 搬运中断恢复（app 无效 + OTA 区完整 -> 重新搬运重启）或进 OTA 接收（不返回）
		boot_ota_entry();
	}

	// app 区有有效固件：直接跳转运行 app
	boot_copy_data(FW_ADDRESS);
#endif

	while(1)
	{		
		start_reboot();
	}
}
