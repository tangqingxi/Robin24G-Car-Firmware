#include "AttitudeControl.h"
#include "FlightConfig.h"

static int att_clamp(int v, int lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

static int att_sin_q15(int ang_01)
{
	int r, r2, r3, r5;

	r = (ang_01 * 5857) >> 10;
	r2 = (r * r) >> 15;
	r3 = (r2 * r) >> 15;
	r5 = (r3 * r2) >> 15;
	return r - r3 / 6 + r5 / 120;
}

static int att_cos_q15(int ang_01)
{
	int r, r2, r4, r6;

	r = (ang_01 * 5857) >> 10;
	r2 = (r * r) >> 15;
	r4 = (r2 * r2) >> 15;
	r6 = (r4 * r2) >> 15;
	return ATT_Q15_ONE - r2 / 2 + r4 / 24 - r6 / 720;
}

void AttitudeControl_Init(AttitudeControl *ac)
{
	int i;

	ac->tgt_att[0] = 0;
	ac->tgt_att[1] = 0;
	ac->tgt_up[0] = 0;
	ac->tgt_up[1] = 0;
	ac->tgt_up[2] = 0;
	ac->tgt_yaw_rate = 0;
	ac->stick_last[0] = 0;
	ac->stick_last[1] = 0;
	ac->stick_yaw_last = 0;
	ac->valid = 0;
	for (i = 0; i < 3; i++) {
		ac->tgt_rate[i] = 0;
	}
}

// 有效收包时换算：摇杆→目标姿态角、目标向上比力、偏航目标角速度。
// 目标方向仍只在有效杆量变化时重算。
void AttitudeControl_OnStick(AttitudeControl *ac, const int cmd[4])
{
	int roll_cmd = cmd[3];
	int pitch_cmd = cmd[2];
	int yaw_cmd = cmd[1];
	unsigned char att_changed = !ac->valid
		|| roll_cmd != ac->stick_last[0]
		|| pitch_cmd != ac->stick_last[1];
	unsigned char yaw_changed = !ac->valid
		|| yaw_cmd != ac->stick_yaw_last;

	if (!att_changed && !yaw_changed) {
		return;
	}

	if (att_changed) {
		int roll, pitch, sr, cr, sp, cp;

		roll = att_clamp((roll_cmd * FLIGHT_MAX_TILT_DEG100) / FLIGHT_STICK_MAX,
						 FLIGHT_MAX_TILT_DEG100);
		pitch = att_clamp(-(pitch_cmd * FLIGHT_MAX_TILT_DEG100) / FLIGHT_STICK_MAX,
						  FLIGHT_MAX_TILT_DEG100);
		ac->tgt_att[0] = roll;
		ac->tgt_att[1] = pitch;

		sr = att_sin_q15(roll);
		cr = att_cos_q15(roll);
		sp = att_sin_q15(pitch);
		cp = att_cos_q15(pitch);
		ac->tgt_up[0] = sp;
		ac->tgt_up[1] = -((sr * cp) >> 15);
		ac->tgt_up[2] = -((cr * cp) >> 15);
		ac->stick_last[0] = roll_cmd;
		ac->stick_last[1] = pitch_cmd;
	}

	if (yaw_changed) {
		ac->tgt_yaw_rate = att_clamp(
			(yaw_cmd * FLIGHT_YAW_RATE_MAX_Q10) / FLIGHT_STICK_MAX,
			FLIGHT_YAW_RATE_MAX_Q10);
		ac->stick_yaw_last = yaw_cmd;
	}
	ac->valid = 1;
}

// 横滚/俯仰外环：目标向上方向 × 实际向上方向，输出目标角速度。
// 偏航目标角速度直接来自遥控映射。
void AttitudeControl_Update(AttitudeControl *ac, const euler_q_t *att)
{
	int q0 = att->q[0];
	int q1 = att->q[1];
	int q2 = att->q[2];
	int q3 = att->q[3];
	int ax, ay, az, ex, ey, tgt;

	ax = ((q0 * q2) >> 14) - ((q1 * q3) >> 14);
	ay = -(((q0 * q1) >> 14) + ((q2 * q3) >> 14));
	az = (((q1 * q1) >> 14) + ((q2 * q2) >> 14)) - ATT_Q15_ONE;

	ex = ((ac->tgt_up[1] * az) >> 15) - ((ac->tgt_up[2] * ay) >> 15);
	ey = ((ac->tgt_up[2] * ax) >> 15) - ((ac->tgt_up[0] * az) >> 15);

	tgt = (ex * FLIGHT_ATT_KP_Q8) >> 13;
	ac->tgt_rate[0] = att_clamp(tgt, FLIGHT_RATE_LIMIT_Q10);
	tgt = (ey * FLIGHT_ATT_KP_Q8) >> 13;
	ac->tgt_rate[1] = att_clamp(tgt, FLIGHT_RATE_LIMIT_Q10);
	ac->tgt_rate[2] = ac->tgt_yaw_rate;
}

void AttitudeControl_ClearOutputs(AttitudeControl *ac)
{
	int i;

	for (i = 0; i < 3; i++) {
		ac->tgt_rate[i] = 0;
	}
}

void AttitudeControl_Inhibit(AttitudeControl *ac)
{
	ac->tgt_att[0] = 0;
	ac->tgt_att[1] = 0;
	ac->tgt_up[0] = 0;
	ac->tgt_up[1] = 0;
	ac->tgt_up[2] = 0;
	ac->tgt_yaw_rate = 0;
	ac->valid = 0;
	AttitudeControl_ClearOutputs(ac);
}
