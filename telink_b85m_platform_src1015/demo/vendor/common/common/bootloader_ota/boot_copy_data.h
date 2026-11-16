/********************************************************************************************************
 * @file    boot_copy_data.h
 *
 * @brief   This is the header file for Telink MCU
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
#ifndef _BOOTLOADER_
#define _BOOTLOADER_

#define	BOOT_SECTION_EN		(1 && BOOTLOADER_ENABLE)

#if(BOOT_SECTION_EN)
#define _attribute_boot_	__attribute__((section(".boot")))
#else
#define _attribute_boot_
#endif

/**
 * @brief		This function serves to read flash data.
 * @param[in]	addr	- the flash address of firmware.
 * @return		none.
 */
_attribute_boot_  __attribute__((used)) void boot_copy_data(unsigned int addr);


#endif /* _BOOTLOADER_ */
