/********************************************************************************************************
 * @file     ota_config.h
 *
 * @brief    This is the header file for OTA SDK
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

#ifndef _OTA_CONFIG_H_
#define _OTA_CONFIG_H_

#define	OTA_SIZE							48 //unit K
#define OTA_ADDRESS							0x10000
#define FW_ADDRESS							0x4000

// OTA 标记区（4K 扇区 @ 0x1C000，位于 OTA 区之后的空闲区）。
// app 检测到可升级版本时写标记并重启；bootloader 读标记进入 OTA 模式。
#define OTA_FLAG_ADDR       0x1C000
#define OTA_FLAG_MAGIC      0x4F544131u   // "1ATO"

// app 跳转 bootloader 前写入的版本号地址（OTA_FLAG 同扇区，magic@+0，version@+4）。
// bootloader 上报 poll 版本时读取：有 app 则为 app 版本，无 app（未写入/0xFF）为 0。
#define APP_VERSION_ADDR    (OTA_FLAG_ADDR + 0x4)

// bootloader 等待 START 超时：进入 OTA 模式后，若超过该时长仍未收到
// CMD_OTA_START（dongle 失联/取消/串行排队过久），判定失败清标记重启回 app，
// 避免设备永久卡死在 bootloader 的 poll 循环。
// 240s < 16MHz u32 tick 回绕周期(~268s)，可直接用 clock_time_exceed 比较。
#define D24G_OTA_WAIT_START_TIMEOUT_US  (240 * 1000 * 1000)

#define START_UP_FLAG						(0x544c4e4b)

#endif