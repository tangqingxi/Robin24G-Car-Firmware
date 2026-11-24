#include "ImuMeasure.h"
#include "FlightConfig.h"
#include "UserEuler.h"
#include "my_t3488.h"

// 保留原飞控使用的轴向、零偏和加速度尺度；迁移时数值不变。
static const s32 s_acc_off_lsb[3] = { -3, -8, -29 };
static const s32 s_acc_scale_q14[3] = { 16436, 16429, 16357 };
static const s32 s_gyro_bias_q6[3] = { -93, -232, -381 };

void ImuMeasure_Init(ImuMeasure *m)
{
	u8 i;

	for (i = 0; i < 3; i++) {
		m->gyro_rem[i] = 0;
		m->lpf[i] = 0;
		m->last_raw[i] = 0;
	}
	m->lpf_alpha = 0;
	m->lpf_alpha_dt_us = 0;
	m->last_tick = 0;
	m->lpf_ok = 0;
	m->have_tick = 0;
}

u8 ImuMeasure_Update(ImuMeasure *m, ImuSample *out)
{
	short raw_a[3], raw_g[3];
	s32 acc_frd[3], acc_mg[3], gyr_q10[3];
	u32 tick;
	u32 dt_ticks = 0;
	u8 i;

	out->valid = 0;
	if (t3488_read_6axis(raw_a, raw_g)) {
		for (i = 0; i < 3; i++) {
			out->raw_acc[i] = (s16)m->last_raw[i];
		}
		return 1;
	}

	tick = clock_time();
	for (i = 0; i < 3; i++) {
		out->raw_acc[i] = raw_a[i];
		m->last_raw[i] = raw_a[i];
	}

	acc_frd[0] = raw_a[0];
	acc_frd[1] = -raw_a[1];
	acc_frd[2] = -raw_a[2];
	gyr_q10[0] = Euler_GyroToQ10(raw_g[0], s_gyro_bias_q6[0],
								 &m->gyro_rem[0]);
	gyr_q10[1] = Euler_GyroToQ10(-raw_g[1], s_gyro_bias_q6[1],
								 &m->gyro_rem[1]);
	gyr_q10[2] = Euler_GyroToQ10(-raw_g[2], s_gyro_bias_q6[2],
								 &m->gyro_rem[2]);
	for (i = 0; i < 3; i++) {
		s32 lsb = (acc_frd[i] - s_acc_off_lsb[i])
				  * s_acc_scale_q14[i] >> 14;

		acc_mg[i] = (lsb * 125) >> 8;
	}

	if (!m->have_tick) {
		m->last_tick = tick;
		m->have_tick = 1;
		dt_ticks = 0;
	} else {
		dt_ticks = tick - m->last_tick;
		m->last_tick = tick;
	}

	if (!m->lpf_ok) {
		m->lpf[0] = acc_mg[0];
		m->lpf[1] = acc_mg[1];
		m->lpf[2] = acc_mg[2];
		m->lpf_ok = 1;
	} else if (C_ACC_LPF_EN && dt_ticks > 0) {
		u32 dt_us = (dt_ticks + FLIGHT_TICKS_PER_US / 2)
					/ FLIGHT_TICKS_PER_US;
		s32 alpha;

		if (dt_us + 50 < m->lpf_alpha_dt_us
			|| dt_us > m->lpf_alpha_dt_us + 50) {
			m->lpf_alpha = (u32)((dt_us * 32768u)
								 / (C_ACC_LPF_TAU_US + dt_us));
			m->lpf_alpha_dt_us = dt_us;
		}
		alpha = (s32)m->lpf_alpha;
		for (i = 0; i < 3; i++) {
			m->lpf[i] += (alpha * (acc_mg[i] - m->lpf[i])) >> 15;
		}
	} else if (!C_ACC_LPF_EN) {
		m->lpf[0] = acc_mg[0];
		m->lpf[1] = acc_mg[1];
		m->lpf[2] = acc_mg[2];
	}

	for (i = 0; i < 3; i++) {
		out->acc_mg[i] = m->lpf[i];
		out->gyro_q10[i] = gyr_q10[i];
	}
	out->tick = tick;
	out->dt_ticks = dt_ticks;
	out->valid = 1;
	return 0;
}
