#ifndef FLIGHT_TIMING_H
#define FLIGHT_TIMING_H

#include "types.h"

// 仅供无桨耗时诊断：所有时间以 24 MHz 系统 tick 记录，读数时再换算为 us。
typedef struct __attribute__((aligned(4))) {
	u32 magic;
	u32 imu_sum_ticks;
	u32 imu_max_ticks;
	u32 imu_count;
	u32 ctrl_max_ticks;
	u32 remain_min_ticks;
	u32 missed_ticks;
	u32 seq;
	u32 imu_start;
	u32 ctrl_start;
} FlightTiming;

extern u32 clock_time(void);

static inline void FlightTiming_Init(FlightTiming *t)
{
	t->magic = 0x54494D47u; // "TIMG"
	t->imu_sum_ticks = 0;
	t->imu_max_ticks = 0;
	t->imu_count = 0;
	t->ctrl_max_ticks = 0;
	t->remain_min_ticks = 0xFFFFFFFFu;
	t->missed_ticks = 0;
	t->seq = 0;
	t->imu_start = 0;
	t->ctrl_start = 0;
}

static inline void FlightTiming_ImuBegin(FlightTiming *t)
{
	t->imu_start = clock_time();
}

static inline void FlightTiming_ImuEnd(FlightTiming *t)
{
	u32 d = clock_time() - t->imu_start;

	t->imu_sum_ticks += d;
	if (d > t->imu_max_ticks) {
		t->imu_max_ticks = d;
	}
	t->imu_count++;
}

static inline void FlightTiming_CtrlBegin(FlightTiming *t, u32 now)
{
	t->ctrl_start = now;
}

static inline void FlightTiming_CtrlEnd(FlightTiming *t, u32 next_tick)
{
	u32 d = clock_time() - t->ctrl_start;
	s32 remain = (s32)(next_tick - clock_time());

	if (d > t->ctrl_max_ticks) {
		t->ctrl_max_ticks = d;
	}
	if (remain < 0) {
		remain = 0;
	}
	if ((u32)remain < t->remain_min_ticks) {
		t->remain_min_ticks = (u32)remain;
	}
	t->seq++;
}

#endif /* FLIGHT_TIMING_H */
