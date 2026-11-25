#include "AttitudeEstimate.h"
#include "FlightConfig.h"

// 静止窗口和长间隔阈值沿用原 FlightLoop 参数。
static void still_reset(AttitudeEstimate *a)
{
	a->still_ticks = 0;
	a->still_n = 0;
	a->still_sum[0] = 0;
	a->still_sum[1] = 0;
	a->still_sum[2] = 0;
}

static void startup_step(AttitudeEstimate *a, const s32 acc_mg[3],
						 const s32 gyr_q10[3], u32 tick, u32 dt_ticks)
{
	u8 still = Euler_IsStill(acc_mg, gyr_q10, a->prev_acc,
							 a->still_prev_ok);
	int i;

	if (!still) {
		still_reset(a);
	} else {
		a->still_sum[0] += acc_mg[0];
		a->still_sum[1] += acc_mg[1];
		a->still_sum[2] += acc_mg[2];
		a->still_n++;
		a->still_ticks += dt_ticks;

		if (a->still_ticks >= FLIGHT_INIT_STILL_TICKS && a->still_n > 0) {
			s32 avg[3];

			// 平均前先转有符号，避免负加速度被当成无符号。
			avg[0] = a->still_sum[0] / (s32)a->still_n;
			avg[1] = a->still_sum[1] / (s32)a->still_n;
			avg[2] = a->still_sum[2] / (s32)a->still_n;
			if (!Euler_InitFromAccel(&a->att, avg,
									 tick / FLIGHT_TICKS_PER_MS)) {
				still_reset(a);
			}
		}
	}
	for (i = 0; i < 3; i++) {
		a->prev_acc[i] = acc_mg[i];
	}
	a->still_prev_ok = 1;
}

void AttitudeEstimate_Init(AttitudeEstimate *a)
{
	AttitudeEstimate_Reset(a);
}

void AttitudeEstimate_Reset(AttitudeEstimate *a)
{
	int i;

	Euler_Reset(&a->att);
	a->dt_rem = 0;
	a->still_prev_ok = 0;
	for (i = 0; i < 3; i++) {
		a->prev_acc[i] = 0;
	}
	still_reset(a);
}

u8 AttitudeEstimate_Update(AttitudeEstimate *a, const s32 acc_mg[3],
						   const s32 gyro_q10[3], u32 tick, u32 dt_ticks)
{
	u32 dt_100us = 0;

	// 超过 100 ms 的长间隔：清积分端点、静止窗口和本拍积分时间。
	if (dt_ticks > (u32)C_ATT_MAX_GAP_100US * FLIGHT_TICKS_PER_100US) {
		a->dt_rem = 0;
		a->att.have_prev = 0;
		still_reset(a);
		dt_ticks = 0;
	}
	if (dt_ticks) {
		a->dt_rem += dt_ticks;
		dt_100us = a->dt_rem / FLIGHT_TICKS_PER_100US;
		a->dt_rem -= dt_100us * FLIGHT_TICKS_PER_100US;
	}

	// 初始化成功的那一拍只置 ready，不报告本拍更新成功。
	if (!a->att.ready) {
		startup_step(a, acc_mg, gyro_q10, tick, dt_ticks);
		return 0;
	}
	if (dt_100us > 0) {
		if (Euler_Update(&a->att, acc_mg, gyro_q10, (s32)dt_100us,
						 tick / FLIGHT_TICKS_PER_MS)) {
			return 1;
		}
	}
	return 0;
}

u8 AttitudeEstimate_IsReady(const AttitudeEstimate *a)
{
	return a->att.ready;
}

const euler_q_t *AttitudeEstimate_GetState(const AttitudeEstimate *a)
{
	return &a->att;
}

u32 AttitudeEstimate_GetStillN(const AttitudeEstimate *a)
{
	return a->still_n;
}

u32 AttitudeEstimate_GetStillTicks(const AttitudeEstimate *a)
{
	return a->still_ticks;
}
