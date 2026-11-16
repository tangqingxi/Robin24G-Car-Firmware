/********************************************************************************************************
 * @file     app_emi.h
 *
 * @brief    EMI 测试模块头文件（非正式代码）
 *
 * @author   lingzhi.chen
 * @date         08,2026
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

#ifndef _APP_EMI_H_
#define _APP_EMI_H_

/* EMI 测试开关：1 使能（上电 PA0+PB0 同时按下进入 EMI 测试，永不返回） */
#define EMI_TEST_FUN_ENABLE     0

#if (EMI_TEST_FUN_ENABLE)

/**
 * @brief   上电检测 EMI 按键：PA0 和 PB0 同时按下进入 EMI service（死循环，
 *          永不返回）。须在 main 上电流程中调用。
 * @return
 * @note    再次上电不按按键即可退出 EMI 模式
 */
void emi_poweron_check(void);

#endif /* EMI_TEST_FUN_ENABLE */

#endif /* _APP_EMI_H_ */
