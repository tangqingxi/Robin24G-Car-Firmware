/********************************************************************************************************
 * @file     toy_log.h
 *
 * @brief    This is the header file for KMD SDK
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

/*
 * toy_log.h
 *
 *  Created on: Dec 5, 2025
 *      Author: lingzhi.chen
 */

#ifndef _TOY_LOG_H_
#define _TOY_LOG_H_

#include "driver.h"
#include "app_config.h"

#ifndef TOY_LOG_ENABLE
#define TOY_LOG_ENABLE      0
#endif

#if TOY_LOG_ENABLE
    #define TOY_LOG(...)		printf(__VA_ARGS__)
#else
    #define TOY_LOG(...)
#endif

#endif /* _TOY_LOG_H_ */
