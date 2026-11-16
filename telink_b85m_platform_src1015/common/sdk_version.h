/********************************************************************************************************
 * @file    sdk_version.h
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
#pragma once

#define For_TXD		1
#define For_RXD		2
#define RF_Type		For_RXD	//For_TXD		//

#if(RF_Type == For_TXD)

#define FW_VERSION						0x0001
/* PID (u32), used to mark the product id, spliced into the SDK_VERSION string as "0x00000001" */
#define PID								0x01000001

#else

#define FW_VERSION						0x0002
/* PID (u32), used to mark the product id, spliced into the SDK_VERSION string as "0x00000001" */
#define PID								0x01000002

#endif	//#if(RF_Type == For_TXD)

/*
 * It is only used by SDK, and the customer is not allowed to modify it
 */
#if(MCU_CORE_B80)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B80_SDK_VERSION_NUM				V3.5.1

#define SDK_VERSION_NUM					B80_SDK_VERSION_NUM
#elif(MCU_CORE_B80B)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B80B_SDK_VERSION_NUM			V3.5.1
#define SDK_VERSION_NUM					B80B_SDK_VERSION_NUM
#elif(MCU_CORE_B85)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B85_SDK_VERSION_NUM				V3.5.1
#define SDK_VERSION_NUM					B85_SDK_VERSION_NUM
#elif(MCU_CORE_B87)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B87_SDK_VERSION_NUM				V3.5.1
#define SDK_VERSION_NUM					B87_SDK_VERSION_NUM
#elif(MCU_CORE_B89)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B89_SDK_VERSION_NUM				V3.5.1
#define SDK_VERSION_NUM					B89_SDK_VERSION_NUM
#elif(MCU_CORE_TC321X)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define TC321X_SDK_VERSION_NUM			V3.5.1
#define SDK_VERSION_NUM					TC321X_SDK_VERSION_NUM
#elif(MCU_CORE_TC1211)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define TC1211_SDK_VERSION_NUM			V3.5.1
#define SDK_VERSION_NUM					TC1211_SDK_VERSION_NUM
#elif(MCU_CORE_TC122X)
/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$tc_platform_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define TC122X_SDK_VERSION_NUM				V3.5.1
#define SDK_VERSION_NUM					TC122X_SDK_VERSION_NUM
#endif


/*
 * Stringify PID (u32), e.g. PID=0x00000001 -> "0x00000001"
 */
#define	STR_HELPER(num)					#num
#define	STR(num)						STR_HELPER(num)

#define	SDK_VERSION1(sdk_version_num,fw_version_num)	"$$$tc_platform_sdk_"#sdk_version_num"_"STR(PID)"_"#fw_version_num"$$$"
#define	SDK_VERSION(sdk_version_num,fw_version_num)	     SDK_VERSION1(sdk_version_num,fw_version_num)


