/***************************************************************************************
 *  File: AttBench.h
 *
 *  台架姿态估计入口：T-3488 直读 → FRD 映射与校准 → 低通 → Mahony → 串口文本
 ****************************************************************************************/
#ifndef __ATT_BENCH_H__
#define __ATT_BENCH_H__

#include "AttConfig.h"

// 主循环调用：内部按 C_ATT_SAMPLE_HZ 节拍采样，约 C_ATT_TX_HZ 输出姿态文本
void AttBench_Process(void);

#endif // __ATT_BENCH_H__
