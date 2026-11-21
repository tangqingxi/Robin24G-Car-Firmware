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

// 实测短事件：0002=解锁，0001=急停；每次 0→1 沿只处理一次
#define L400_EVENT_ESTOP        0x0001
#define L400_EVENT_UNLOCK       0x0002

typedef struct {
	unsigned char armed;
	unsigned char reason;           // L400_LOCK_*
	unsigned short key_prev;        // 上一包按键字，用于事件沿检测
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
	a->reason = L400_LOCK_BOOT;
	a->key_prev = 0;
}

static void L400M_ArmLock(L400MArm *a, unsigned char reason)
{
	a->armed = 0;
	if (reason) {
		a->reason = reason;
	}
}

/*
 * 每个 RF 包调用一次（包括无效摇杆/全零心跳包）。
 * 0001=急停，收到新事件立即锁定；0002=解锁，只在新事件且该包有效、
 * 油门低位、三轴回中、姿态就绪、链路有效且非诊断时才解锁。
 * 条件不满足的 0002 事件也会被消费，后续重复包不会补解锁。
 */
static unsigned char L400M_EventStep(L400MArm *a, unsigned short key,
									 unsigned char packet_valid,
									 unsigned char thr_low,
									 unsigned char sticks_center,
									 unsigned char att_ready,
									 unsigned char link_ok,
									 unsigned char diag_enabled)
{
	unsigned char new_estop = (key & L400_EVENT_ESTOP)
							  && !(a->key_prev & L400_EVENT_ESTOP);
	unsigned char new_unlock = (key & L400_EVENT_UNLOCK)
							   && !(a->key_prev & L400_EVENT_UNLOCK);

	a->key_prev = key;

	// 急停优先：即使同包同时出现解锁位也不处理解锁
	if (new_estop) {
		L400M_ArmLock(a, L400_LOCK_MANUAL);
		return L400_LOCK_MANUAL;
	}

	if (new_unlock) {
		if (packet_valid && !(key & L400_EVENT_ESTOP)
			&& thr_low && sticks_center && att_ready && link_ok
			&& !diag_enabled) {
			a->armed = 1;
			a->reason = L400_LOCK_NONE;
		}
		// 条件不满足：事件已消费，不锁定也不延迟解锁
	}
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
