#ifndef IMU_MEASURE_H
#define IMU_MEASURE_H

#include <types.h>

// 一次有效六轴读数的结果；加速度单位 mg，陀螺 Q10 rad/s。
typedef struct __attribute__((aligned(4))) {
	s32 acc_mg[3];
	s32 gyro_q10[3];
	u32 tick;
	u32 dt_ticks;
	s16 raw_acc[3];
	u8 valid;
} ImuSample;

// 测量模块内部状态，保留陀螺余数、加速度低通和上一拍时间。
typedef struct __attribute__((aligned(4))) {
	s32 gyro_rem[3];
	s32 lpf[3];
	s32 last_raw[3];
	u32 lpf_alpha;
	u32 lpf_alpha_dt_us;
	u32 last_tick;
	u8 lpf_ok;
	u8 have_tick;
} ImuMeasure;

// 编译期核对：传给姿态代码的三轴数组和 32 位时间字段必须 4 字节对齐。
typedef char imu_acc_align[(__builtin_offsetof(ImuSample, acc_mg) & 3u) ? -1 : 1];
typedef char imu_gyro_align[(__builtin_offsetof(ImuSample, gyro_q10) & 3u) ? -1 : 1];
typedef char imu_tick_align[(__builtin_offsetof(ImuSample, tick) & 3u) ? -1 : 1];
typedef char imu_dt_align[(__builtin_offsetof(ImuSample, dt_ticks) & 3u) ? -1 : 1];
typedef char imu_lpf_align[(__builtin_offsetof(ImuMeasure, lpf) & 3u) ? -1 : 1];
typedef char imu_gyro_rem_align[(__builtin_offsetof(ImuMeasure, gyro_rem) & 3u) ? -1 : 1];
typedef char imu_last_tick_align[(__builtin_offsetof(ImuMeasure, last_tick) & 3u) ? -1 : 1];

void ImuMeasure_Init(ImuMeasure *m);
// 读取、映射、校准并更新低通；返回 0 表示本次有效。
u8 ImuMeasure_Update(ImuMeasure *m, ImuSample *out);

#endif /* IMU_MEASURE_H */
