#include "MotorMixer.h"

// 对称限幅，避免中间量越界。
static int motor_clamp(int v, int lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

// 手动油门 FB(±307) → 推力 PWM(60..340)；中心 190。
int MotorMixer_Thrust(int fb)
{
	int thrust = MOTOR_THR_MID
				 + (motor_clamp(fb, MOTOR_STICK_MAX) * MOTOR_THR_SPAN) / MOTOR_STICK_MAX;

	thrust = motor_clamp(thrust, MOTOR_THR_MAX);
	if (thrust < MOTOR_THR_MIN) {
		thrust = MOTOR_THR_MIN;
	}
	return thrust;
}

/*
 * 200Hz 混控：thrust 已由油门换算，corr 为三轴 Q10 纠偏。
 * 俯仰约定：机头抬高时 corr_pitch<0（控制环要低头），必须前减后加，
 * 即 front=thr+dp、rear=thr-dp；这样 rear > front。
 * 偏航约定：从机体正上方看前/后 CW、左/右 CCW；正偏航 dy>0 时
 * 左/右加速、前/后减速，所以 dy 只在这里翻一次符号：
 * 前/后 -dy，左/右 +dy。
 * 限幅/统一缩放前后只使用同一个 dp、dy。
 */
void MotorMixer_Mix(int thrust, int corr_roll, int corr_pitch,
					int corr_yaw, MotorMixerOutput *out)
{
	int dr = (corr_roll * MOTOR_DIFF_GAIN_Q16) >> 16;
	int dp = (corr_pitch * MOTOR_DIFF_GAIN_Q16) >> 16;
	int dy = (corr_yaw * MOTOR_YAW_DIFF_GAIN_Q16) >> 16;
	int mf, mr, ml, mm, lo, hi, scale, room;

	dr = motor_clamp(dr, MOTOR_DIFF_LIMIT);
	dp = motor_clamp(dp, MOTOR_DIFF_LIMIT);
	dy = motor_clamp(dy, MOTOR_DIFF_LIMIT);

	mf = thrust + dp - dy;
	mr = thrust - dp - dy;
	ml = thrust + dr + dy;
	mm = thrust - dr + dy;

	lo = mf;
	if (mr < lo) lo = mr;
	if (ml < lo) lo = ml;
	if (mm < lo) lo = mm;
	hi = mf;
	if (mr > hi) hi = mr;
	if (ml > hi) hi = ml;
	if (mm > hi) hi = mm;

	scale = 256;
	if (hi > MOTOR_PWM_MAX && hi > thrust) {
		room = ((MOTOR_PWM_MAX - thrust) << 8) / (hi - thrust);
		if (room < scale) scale = room;
	}
	if (lo < MOTOR_PWM_MIN_RUN && lo < thrust) {
		room = ((thrust - MOTOR_PWM_MIN_RUN) << 8) / (thrust - lo);
		if (room < scale) scale = room;
	}
	if (scale < 0) scale = 0;
	if (scale < 256) {
		dr = (dr * scale) >> 8;
		dp = (dp * scale) >> 8;
		dy = (dy * scale) >> 8;
		mf = thrust + dp - dy;
		mr = thrust - dp - dy;
		ml = thrust + dr + dy;
		mm = thrust - dr + dy;
	}

	mf = motor_clamp(mf, MOTOR_PWM_MAX);
	mr = motor_clamp(mr, MOTOR_PWM_MAX);
	ml = motor_clamp(ml, MOTOR_PWM_MAX);
	mm = motor_clamp(mm, MOTOR_PWM_MAX);
	if (mf < MOTOR_PWM_MIN_RUN) mf = MOTOR_PWM_MIN_RUN;
	if (mr < MOTOR_PWM_MIN_RUN) mr = MOTOR_PWM_MIN_RUN;
	if (ml < MOTOR_PWM_MIN_RUN) ml = MOTOR_PWM_MIN_RUN;
	if (mm < MOTOR_PWM_MIN_RUN) mm = MOTOR_PWM_MIN_RUN;

	out->limited = (scale < 256) ? 1 : 0;
	out->front = (unsigned short)mf;
	out->rear = (unsigned short)mr;
	out->left = (unsigned short)ml;
	out->right = (unsigned short)mm;
}
