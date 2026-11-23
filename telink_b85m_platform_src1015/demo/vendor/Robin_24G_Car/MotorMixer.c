/*
 * MotorMixer.c
 *
 * 手动油门换算 + 十字(+)四电机混控。
 *
 * 机架与电机编号：
 *   front（前）、rear（后）、left（左）、right（右）分别对应四个电机。
 *   横滚绕 +X：右侧下沉为正，需要左侧加速、右侧减速；
 *   俯仰绕 +Y：抬头为正，需要前侧减速、后侧加速；
 *   偏航绕 +Z：俯视顺时针为正，需要左/右加速、前/后减速。
 *
 * 输入：
 *   thrust      : MotorMixer_Thrust 算出的基础推力 PWM（约 60..340）
 *   corr_roll   : 横滚纠偏 Q10，正=希望右滚
 *   corr_pitch  : 俯仰纠偏 Q10，正=希望抬头
 *   corr_yaw    : 偏航纠偏 Q10，正=希望俯视顺时针
 * 输出：
 *   四路 PWM + limited 标志；PWM 已限在 [MOTOR_PWM_MIN_RUN, MOTOR_PWM_MAX]。
 *
 * 关键设计：
 *   - 先算四路“期望值”，再统一缩放，保证混控方向不被逐路限幅破坏；
 *   - 只在接近 PWM 上限或下滑到最小运行值时缩放，缩放对所有差动项用同一
 *     比例，因此横滚/俯仰/偏航的相对比例不变；
 *   - 缩放后仍逐路做最终限幅，防止整数舍入落到范围外；
 *   - 最终下限 MOTOR_PWM_MIN_RUN 是“电机能持续转”的最小值，不是 0；
 *     真正停机的 0 由 FlightSafety 在安全门/急停时直接写，不经过混控。
 */
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
// `fb` 是前后通道（俯仰/油门）杆量；中位 190 对应悬停附近的基础推力。
// 映射先线性放大到 ±140，再夹到 60..340：
//   - 低于 60 的推力没有意义，直接抬到 60；
//   - 高于 340 的推力留给混控差动余量，避免一起飞就顶到 PWM 上限。
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
// 四路混控主函数。调用前 `thrust` 已由 MotorMixer_Thrust 换算；
// corr_* 为内环 Q10 纠偏，允许正负。
//
// 混控矩阵（每行对应一个电机）：
//   front = thrust + dp - dy
//   rear  = thrust - dp - dy
//   left  = thrust + dr + dy
//   right = thrust - dr + dy
// 其中 dr/dp/dy 是 Q10 纠偏经 Q16 增益换算后的 PWM 差动。
// 俯仰的符号容易看反：抬头为正时 dp>0，前减后加，所以 front 用 +dp、
// rear 用 -dp；如果这里写反，飞控会正反馈发散。
void MotorMixer_Mix(int thrust, int corr_roll, int corr_pitch,
					int corr_yaw, MotorMixerOutput *out)
{
	// Q10 纠偏 × Q16 增益 → Q16 中间量，右移 16 得 PWM 差动。
	// 横滚/俯仰共用 2880，偏航用 1920；后者较小，是因为偏航力臂和
	// 反扭矩效率更高，过大的 dy 会让偏航一打就掉高。
	int dr = (corr_roll * MOTOR_DIFF_GAIN_Q16) >> 16;
	int dp = (corr_pitch * MOTOR_DIFF_GAIN_Q16) >> 16;
	int dy = (corr_yaw * MOTOR_YAW_DIFF_GAIN_Q16) >> 16;
	int mf, mr, ml, mm, lo, hi, scale, room;

	// 先限制单项差动，避免某一轴纠偏瞬间过大把总推力彻底吃掉。
	dr = motor_clamp(dr, MOTOR_DIFF_LIMIT);
	dp = motor_clamp(dp, MOTOR_DIFF_LIMIT);
	dy = motor_clamp(dy, MOTOR_DIFF_LIMIT);

	// 未限幅的四路期望值；先保留原始值，后面用 lo/hi 判断是否需要整体缩放。
	mf = thrust + dp - dy;
	mr = thrust - dp - dy;
	ml = thrust + dr + dy;
	mm = thrust - dr + dy;

	// 找四路中的最小值/最大值，决定是否触碰上限或最小运行值。
	lo = mf;
	if (mr < lo) lo = mr;
	if (ml < lo) lo = ml;
	if (mm < lo) lo = mm;
	hi = mf;
	if (mr > hi) hi = mr;
	if (ml > hi) hi = ml;
	if (mm > hi) hi = mm;

	// 统一缩放：
	//   1) 上限：最高一路超过 MOTOR_PWM_MAX 时，按“基础推力到上限的余量”
	//      求一个 0..256 的比例，把差动项一起缩小，保证最高路正好落到上限；
	//   2) 下限：最低一路低于 MOTOR_PWM_MIN_RUN 时，按“基础推力到最小运行值
	//      的余量”缩放，避免为了修正姿态把某个电机压到停转；
	//   3) 两个条件同时存在时取更小的 scale，先满足最紧的一侧。
	// 只缩放 dr/dp/dy，不缩放 thrust，所以总推力意图保持不变。
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
		// 缩放后重新计算四路；使用同一 scale，保持三轴差动比例。
		dr = (dr * scale) >> 8;
		dp = (dp * scale) >> 8;
		dy = (dy * scale) >> 8;
		mf = thrust + dp - dy;
		mr = thrust - dp - dy;
		ml = thrust + dr + dy;
		mm = thrust - dr + dy;
	}

	// 缩放后仍逐路夹紧：整数舍入可能让结果比上限多 1 个 LSB。
	// 这里的下限是“能持续转”的最小 PWM，不是 0；安全门停机时由
	// FlightSafety 直接写 0，绕过本函数。
	mf = motor_clamp(mf, MOTOR_PWM_MAX);
	mr = motor_clamp(mr, MOTOR_PWM_MAX);
	ml = motor_clamp(ml, MOTOR_PWM_MAX);
	mm = motor_clamp(mm, MOTOR_PWM_MAX);
	if (mf < MOTOR_PWM_MIN_RUN) mf = MOTOR_PWM_MIN_RUN;
	if (mr < MOTOR_PWM_MIN_RUN) mr = MOTOR_PWM_MIN_RUN;
	if (ml < MOTOR_PWM_MIN_RUN) ml = MOTOR_PWM_MIN_RUN;
	if (mm < MOTOR_PWM_MIN_RUN) mm = MOTOR_PWM_MIN_RUN;

	// limited 只表示“本拍是否发生统一缩放”，用于日志/诊断；
	// 最终逐路夹紧不算 limited，因为那只是整数舍入保护。
	out->limited = (scale < 256) ? 1 : 0;
	out->front = (unsigned short)mf;
	out->rear = (unsigned short)mr;
	out->left = (unsigned short)ml;
	out->right = (unsigned short)mm;
}
