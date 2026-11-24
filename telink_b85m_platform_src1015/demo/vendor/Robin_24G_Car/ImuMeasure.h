#ifndef IMU_MEASURE_H
#define IMU_MEASURE_H

#include <types.h>

// 一次有效六轴读数的结果；加速度单位 mg，陀螺 Q10 rad/s。
typedef struct {
	s32 acc_mg[3];
	s32 gyro_q10[3];
	s16 raw_acc[3];
	u32 tick;
	u32 dt_ticks;
	u8 valid;
} ImuSample;

// 测量模块内部状态，保留陀螺余数、加速度低通和上一拍时间。
typedef struct {
	s32 gyro_rem[3];
	s32 lpf[3];
	s32 last_raw[3];
	u32 lpf_alpha;
	u32 lpf_alpha_dt_us;
	u32 last_tick;
	u8 lpf_ok;
	u8 have_tick;
} ImuMeasure;

void ImuMeasure_Init(ImuMeasure *m);
// 读取、映射、校准并更新低通；返回 0 表示本次有效。
u8 ImuMeasure_Update(ImuMeasure *m, ImuSample *out);

#endif /* IMU_MEASURE_H */
