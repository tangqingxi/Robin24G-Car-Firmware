/*
 * AttitudeControl.c
 *
 * 横滚/俯仰姿态外环 + 偏航角速度给定。
 *
 * 输入输出单位与坐标：
 *   - 遥控杆量 cmd[4]：thr, yaw, pitch, roll，范围约 ±307；
 *   - 目标横滚/俯仰 tgt_att：0.01°，右滚/抬头为正；
 *   - 目标向上比力 tgt_up[3]：Q15，机体 FRD 表达；
 *   - 输出 tgt_rate[3]：Q10 rad/s，依次为横滚/俯仰/偏航，送入 RateControl。
 *
 * 设计取舍：
 *   - 外环只做横滚/俯仰的姿态保持，偏航不闭环姿态角，只把摇杆映射成
 *     目标偏航角速度；
 *   - 目标向上方向不是每拍重算：只有杆量变化时才重算，避免遥控抖动和
 *     定点三角函数量化噪声直接进入角速度目标；
 *   - 三角运算使用 Q15 多项式，不使用浮点，适合没有软浮点的 TC32。
 *
 * 坐标符号：横滚绕 +X（右滚正），俯仰绕 +Y（抬头正），偏航绕 +Z（俯视顺时针正）。
 */
#include "AttitudeControl.h"
#include "FlightConfig.h"

// 对称限幅：用于目标角速度、角度和杆量，避免异常值进入下一级。
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

// Q15 正弦近似，输入单位 0.01°，输出 Q15。
// 小角度用 5 阶 Maclaurin：sin(x)≈x-x^3/6+x^5/120；角度先换成弧度 Q15。
// 这里的目标倾角只有几十度，5 阶误差远小于 0.1°，够外环使用。
static int att_sin_q15(int ang_01)
{
	int r, r2, r3, r5;

	r = (ang_01 * 5857) >> 10;
	r2 = (r * r) >> 15;
	r3 = (r2 * r) >> 15;
	r5 = (r3 * r2) >> 15;
	return r - r3 / 6 + r5 / 120;
}

// Q15 余弦近似，输入单位 0.01°，输出 Q15；同样基于弧度 Q15 的 6 阶展开。
static int att_cos_q15(int ang_01)
{
	int r, r2, r4, r6;

	r = (ang_01 * 5857) >> 10;
	r2 = (r * r) >> 15;
	r4 = (r2 * r2) >> 15;
	r6 = (r4 * r2) >> 15;
	return ATT_Q15_ONE - r2 / 2 + r4 / 24 - r6 / 720;
}

// 外环初始化：所有目标归零，valid=0 表示还没有有效杆量，第一次 OnStick 强制重算。
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
//
// 杆量到姿态角的映射：
//   roll  =  roll_cmd/307 × MAX_TILT，右滚为正；
//   pitch = -pitch_cmd/307 × MAX_TILT，遥控“推杆”对应低头；
// 反向很关键：如果这里符号错，外环会把飞机往相反方向拉。
// 目标向上比力由 R_y(pitch)·R_x(roll)·(0,0,-1) 得到：
//   tgt_up = (sin(pitch), -sin(roll)cos(pitch), -cos(roll)cos(pitch))
// 它就是“目标姿态下，加速度计应当看到的机体向上方向”。
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
//
// 实际向上方向（机体 FRD）由四元数旋转矩阵第三列取反得到：
//   ax = 2(q0·q2 - q1·q3)
//   ay = -2(q0·q1 + q2·q3)
//   az = 2(q1² + q2²) - 1
// 目标向上方向与它的叉积 (ex,ey) 近似表示“需要绕 X/Y 转多少才能对齐”。
// 叉积用 Q15 乘法，再乘外环 Kp（Q8）得到 Q10 角速度：
//   目标角速度 = 误差 × Kp，最后限幅到 FLIGHT_RATE_LIMIT_Q10。
//
// 这里只取叉积的 X/Y 分量：横滚/俯仰外环不产生偏航修正；
// 偏航通道完全由摇杆目标角速度决定，避免没有磁力计时偏航角漂移
// 反过来污染横滚/俯仰。
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

// 只清三轴目标角速度，不破坏目标姿态/杆量缓存。
// 失联或超时恢复后，只要杆量没变，外环仍可沿用原目标方向。
void AttitudeControl_ClearOutputs(AttitudeControl *ac)
{
	int i;

	for (i = 0; i < 3; i++) {
		ac->tgt_rate[i] = 0;
	}
}

// 安全门要求彻底禁止外环：清目标姿态、目标向上方向、偏航目标、杆量缓存和输出。
// valid=0 后，下一次有效 OnStick 会当作全新杆量重新计算，不会把旧目标带回来。
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
