/********************************************************************************************************
 * @file    boot_copy_data.c
 *
 * @brief   This is the source file for Telink MCU
 *
 * @author  Driver Group
 * @date    2022
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
 *
 *******************************************************************************************************/
#include "boot_copy_data.h"
#include "spi_i.h"
#include "timer.h"
#include "flash.h"
#include "lib/include/clock.h"
#include "lib/include/sys.h"
#include "analog.h"

#if (BOOTLOADER_ENABLE)
/**
 * @brief		This function serves to set flash write command, use for boot.
 * @param[in]	cmd	- set command.
 * @return		none.
 */
_attribute_boot_  __attribute__((used)) static void send_flash_cmd(unsigned char cmd)
{
	mspi_high();
	CLOCK_DLY_10_CYC;
	mspi_low();
	mspi_write(cmd);
	mspi_wait();
}

/**
 * @brief		This function serves to send flash address, use for boot.
 * @param[in]	addr	- the flash address.
 * @return		none.
 */
_attribute_boot_  __attribute__((used)) static void send_flash_addr(unsigned int addr)
{
	mspi_write((unsigned char)(addr>>16));
	mspi_wait();
	mspi_write((unsigned char)(addr>>8));
	mspi_wait();
	mspi_write((unsigned char)(addr));
	mspi_wait();
}

/**
 * @brief		This function serves to read flash data , use for boot.
 * @param[in]	addr	- the flash address.
 * @param[in]	len		- the read byte length.
 * @param[in]	buf		- the read data buffer.
 * @return		none.
 */
_attribute_boot_  __attribute__((used)) void flash_read(unsigned long addr, unsigned long len, unsigned char *buf)
{
	send_flash_cmd(FLASH_READ_CMD);
	send_flash_addr(addr);

	mspi_write(0x00);			/* to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);		/* auto mode */
	mspi_wait();
	for(int i = 0; i < len; ++i)
	{
		*buf++ = mspi_get();
		mspi_wait();
	}
	mspi_high();
}

/**
 * @brief		This function serves to read flash data, use for boot.
 * @param[in]	addr	- the flash address of firmware.
 * @return		none.
 */

volatile unsigned short ram_code_bsize=0;

_attribute_boot_  __attribute__((used)) void boot_copy_data(unsigned int addr)
{

	flash_read(addr+0x0c, 2, (unsigned char *)(&ram_code_bsize));
	ram_code_bsize <<= 4;
	//flash_read(addr, ram_code_bsize, (unsigned char *)buff_rec);
	flash_read(addr, ram_code_bsize, (unsigned char *)0x840000);
	//debug_addr = (read_reg32(0x608)&0xff000000 )|0x10000;
	write_reg32(0x608,0x10000);

	write_reg8(0x602,read_reg8(0x602)&(~BIT(2)));
	write_reg8(0x602,read_reg8(0x602)|(BIT(7)));

	//write_reg8(0x602, 0x88);
	while(1);

}
#endif
