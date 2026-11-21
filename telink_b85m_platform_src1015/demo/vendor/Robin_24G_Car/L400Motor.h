#ifndef L400_MOTOR_H
#define L400_MOTOR_H

/*
 * L400 手动油门、十字四电机混控和解锁状态机的纯逻辑部分。
 * 不依赖硬件，UserRxdFunc.c 和主机回归共用；所有注释中文。
 */

// 油门/混控常量。油门中心约悬停，满行程约 ±140 PWM。
#define L400_STICK_MAX          307
#define L400_THR_MID            190
#define L400_THR_SPAN           140
#define L400_THR_MIN            60
#define L400_THR_MAX            340
#define L400_PWM_MIN_RUN        45
#define L400_PWM_MAX            400
#define L400_ARM_THR_MAX        (-250)   // 油门低位才允许解锁
#define L400_ARM_STICK_MAX      45       // 横滚/俯仰/偏航回中范围

// 三轴纠偏 Q10(±4096) → PWM 差动；横滚/俯仰满量程约 180，限 160；
// 偏航满量程约 120。系数已包含十字 + 型相对 X 型的 1.414 补偿。
#define L400_DIFF_GAIN_Q16      2880
#define L400_YAW_DIFF_GAIN_Q16  1920
#define L400_DIFF_LIMIT         160

// 锁定原因（L400Q 的 x 字段）
#define L400_LOCK_NONE          0
#define L400_LOCK_BOOT          1
#define L400_LOCK_LINK          2
#define L400_LOCK_CMD           3
#define L400_LOCK_ATT           4
#define L400_LOCK_MANUAL        5
#define L400_LOCK_DIAG          6

typedef struct {
	unsigned short front;
	unsigned short rear;
	unsigned short left;
	unsigned short right;
} L400MMix;

typedef struct {
	unsigned char armed;
	unsigned char require_release;  // 故障/急停后必须先松开解锁键
	unsigned char arm_prev;
	unsigned char estop_prev;
	unsigned char attf_prev;
	unsigned char hold_active;
	unsigned int  hold_tick;
	unsigned char reason;           // L400_LOCK_*
} L400MArm;

static int l400m_clamp(int v, int lim)
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
static int L400M_Thrust(int fb)
{
	int thrust = L400_THR_MID
				 + (l400m_clamp(fb, L400_STICK_MAX) * L400_THR_SPAN) / L400_STICK_MAX;

	thrust = l400m_clamp(thrust, L400_THR_MAX);
	if (thrust < L400_THR_MIN) {
		thrust = L400_THR_MIN;
	}
	return thrust;
}

/*
 * 200Hz 混控：thrust 已由油门换算，corr 为三轴 Q10 纠偏。
 * 俯仰约定：机头抬高时 corr_pitch<0（控制环要低头），必须前减后加，
 * 即 front=thr+dp、rear=thr-dp；这样 rear > front。
 * 限幅只作用于同一个 dp，混控前后符号一致。
 */
static void L400M_Mix(int thrust, int corr_roll, int corr_pitch,
					  int corr_yaw, L400MMix *mix)
{
	int dr = (corr_roll * L400_DIFF_GAIN_Q16) >> 16;
	int dp = (corr_pitch * L400_DIFF_GAIN_Q16) >> 16;
	int dy = (corr_yaw * L400_YAW_DIFF_GAIN_Q16) >> 16;
	int mf, mr, ml, mm, lo, hi, scale, room;

	dr = l400m_clamp(dr, L400_DIFF_LIMIT);
	dp = l400m_clamp(dp, L400_DIFF_LIMIT);
	dy = l400m_clamp(dy, L400_DIFF_LIMIT);

	// 机头抬高(corr_pitch<0) → dp<0 → 前=thr+dp 减小、后=thr-dp 增大
	mf = thrust + dp + dy;
	mr = thrust - dp + dy;
	ml = thrust + dr - dy;
	mm = thrust - dr - dy;

	// 任一路越界时三轴差动统一缩放，保持平均推力不变
	lo = mf;
	if (mr < lo) lo = mr;
	if (ml < lo) lo = ml;
	if (mm < lo) lo = mm;
	hi = mf;
	if (mr > hi) hi = mr;
	if (ml > hi) hi = ml;
	if (mm > hi) hi = mm;

	scale = 256;
	if (hi > L400_PWM_MAX && hi > thrust) {
		room = ((L400_PWM_MAX - thrust) << 8) / (hi - thrust);
		if (room < scale) scale = room;
	}
	if (lo < L400_PWM_MIN_RUN && lo < thrust) {
		room = ((thrust - L400_PWM_MIN_RUN) << 8) / (thrust - lo);
		if (room < scale) scale = room;
	}
	if (scale < 0) scale = 0;
	if (scale < 256) {
		dr = (dr * scale) >> 8;
		dp = (dp * scale) >> 8;
		dy = (dy * scale) >> 8;
		mf = thrust + dp + dy;
		mr = thrust - dp + dy;
		ml = thrust + dr - dy;
		mm = thrust - dr - dy;
	}

	mf = l400m_clamp(mf, L400_PWM_MAX);
	mr = l400m_clamp(mr, L400_PWM_MAX);
	ml = l400m_clamp(ml, L400_PWM_MAX);
	mm = l400m_clamp(mm, L400_PWM_MAX);
	if (mf < L400_PWM_MIN_RUN) mf = L400_PWM_MIN_RUN;
	if (mr < L400_PWM_MIN_RUN) mr = L400_PWM_MIN_RUN;
	if (ml < L400_PWM_MIN_RUN) ml = L400_PWM_MIN_RUN;
	if (mm < L400_PWM_MIN_RUN) mm = L400_PWM_MIN_RUN;

	mix->front = (unsigned short)mf;
	mix->rear = (unsigned short)mr;
	mix->left = (unsigned short)ml;
	mix->right = (unsigned short)mm;
}

static void L400M_ArmInit(L400MArm *a)
{
	a->armed = 0;
	a->require_release = 1;
	a->arm_prev = 1;              // 上电先要求松开解锁键
	a->estop_prev = 0;
	a->attf_prev = 0;
	a->hold_active = 0;
	a->hold_tick = 0;
	a->reason = L400_LOCK_BOOT;
}

static void L400M_ArmLock(L400MArm *a, unsigned char reason)
{
	a->armed = 0;
	a->hold_active = 0;
	a->require_release = 1;
	if (reason) {
		a->reason = reason;
	}
}

/*
 * 解锁/急停状态机。
 * 返回非 0 表示本拍需要立即清四路（值为锁定原因），调用方再执行硬件清零。
 * 急停键或姿态注入键按住期间：禁止解锁、清四路并保持锁定；
 * 松开后还要求先松开解锁键，再重新长按 hold_ticks。
 */
static unsigned char L400M_ArmStep(L400MArm *a, unsigned int now,
								   unsigned char arm_key,
								   unsigned char estop_key,
								   unsigned char attf_key,
								   unsigned char valid,
								   unsigned char thr_low,
								   unsigned char sticks_center,
								   unsigned int hold_ticks)
{
	unsigned char stop = 0;

	if (a->armed) {
		if (estop_key && !a->estop_prev) {
			L400M_ArmLock(a, L400_LOCK_MANUAL);
			stop = L400_LOCK_MANUAL;
		} else if (attf_key && !a->attf_prev) {
			L400M_ArmLock(a, L400_LOCK_ATT);
			stop = L400_LOCK_ATT;
		}
		a->estop_prev = estop_key;
		a->attf_prev = attf_key;
		a->arm_prev = arm_key;
		return stop;
	}

	a->estop_prev = estop_key;
	a->attf_prev = attf_key;

	// 急停/姿态注入键按住：无论解锁键怎么按都不允许解锁
	if (estop_key || attf_key) {
		a->require_release = 1;
		a->arm_prev = arm_key;
		a->hold_active = 0;
		return 0;
	}

	// 锁定后必须先看到解锁键松开
	if (a->require_release) {
		if (arm_key) {
			a->arm_prev = 1;
			a->hold_active = 0;
			return 0;
		}
		a->require_release = 0;
		a->arm_prev = 0;
	}

	if (!arm_key) {
		a->arm_prev = 0;
		a->hold_active = 0;
		return 0;
	}

	if (!a->arm_prev) {
		// 只在新按下沿开始计时；中途失效不会顺延
		if (valid && thr_low && sticks_center) {
			a->hold_active = 1;
			a->hold_tick = now;
		} else {
			a->hold_active = 0;
		}
	} else if (!valid || !thr_low || !sticks_center) {
		a->hold_active = 0;
	}

	if (a->hold_active
		&& (unsigned int)(now - a->hold_tick) >= hold_ticks
		&& valid && thr_low && sticks_center) {
		a->armed = 1;
		a->reason = L400_LOCK_NONE;
		a->hold_active = 0;
		a->require_release = 0;
	}
	a->arm_prev = 1;
	return 0;
}

// 未解锁时强制四路 0；解锁时才输出混控结果。
static void L400M_Output(const L400MArm *a, const L400MMix *mix,
						 unsigned short out[4])
{
	if (!a->armed) {
		out[0] = 0;
		out[1] = 0;
		out[2] = 0;
		out[3] = 0;
	} else {
		out[0] = mix->front;
		out[1] = mix->rear;
		out[2] = mix->left;
		out[3] = mix->right;
	}
}

#endif /* L400_MOTOR_H */
