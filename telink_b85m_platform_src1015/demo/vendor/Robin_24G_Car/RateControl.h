#ifndef RATE_CONTROL_H
#define RATE_CONTROL_H

#include "FlightConfig.h"

// 三轴角速度内环状态；I/D 状态只在对应增益非零时编译，节省 RAM。
typedef struct __attribute__((aligned(4))) {
	int corr[3];              // 三轴纠偏输出，Q10；横滚/俯仰/偏航
#if (FLIGHT_KI_Q16 != 0)
	int integ[3];             // 三轴误差积分状态（未乘 dt，按 200Hz 整定）
#endif
#if (FLIGHT_KD_Q8 != 0)
	int prev_err[3];          // 上一拍误差，用于 D 项差分
	unsigned char have_prev;  // 0=还没有上一拍误差，第一拍 D 项置 0
#endif
} RateControl;

void RateControl_Init(RateControl *rc);
void RateControl_Update(RateControl *rc, const int target[3],
						const int gyro[3]);
void RateControl_ClearOutputs(RateControl *rc);

#endif /* RATE_CONTROL_H */
