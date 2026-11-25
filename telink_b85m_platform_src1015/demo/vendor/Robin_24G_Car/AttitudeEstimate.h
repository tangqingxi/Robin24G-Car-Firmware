#ifndef ATTITUDE_ESTIMATE_H
#define ATTITUDE_ESTIMATE_H

#include "types.h"
#include "UserEuler.h"

// 姿态估计状态：四元数、静止初始化窗口、上一帧加速度和积分时间余数。
typedef struct __attribute__((aligned(4))) {
	euler_q_t att;
	s32 prev_acc[3];
	s32 still_sum[3];
	u32 still_ticks;
	u32 still_n;
	u32 dt_rem;
	u8 still_prev_ok;
} AttitudeEstimate;

void AttitudeEstimate_Init(AttitudeEstimate *a);
void AttitudeEstimate_Reset(AttitudeEstimate *a);

/*
 * 消费一次有效 IMU 样本：
 *   acc_mg[3]   加速度，单位 mg
 *   gyro_q10[3] 角速度，单位 Q10 rad/s
 *   tick        读取成功后的 32 位系统 tick
 *   dt_ticks    与上一有效样本的间隔，单位系统 tick
 * 返回 1 表示本拍姿态更新成功；返回 0 包括初始化拍和未达到更新条件。
 * 初始化成功的那一拍 IsReady 会变为 1，但本拍 Update 仍返回 0。
 */
u8 AttitudeEstimate_Update(AttitudeEstimate *a, const s32 acc_mg[3],
						   const s32 gyro_q10[3], u32 tick, u32 dt_ticks);

u8 AttitudeEstimate_IsReady(const AttitudeEstimate *a);
const euler_q_t *AttitudeEstimate_GetState(const AttitudeEstimate *a);
u32 AttitudeEstimate_GetStillN(const AttitudeEstimate *a);
u32 AttitudeEstimate_GetStillTicks(const AttitudeEstimate *a);

#endif /* ATTITUDE_ESTIMATE_H */
