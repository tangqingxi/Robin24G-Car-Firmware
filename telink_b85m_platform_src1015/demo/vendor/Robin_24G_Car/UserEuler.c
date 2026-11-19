//********************************************************************
/*
 * UserEuler.c
 *
 *  定点四元数姿态估计（Mahony 互补滤波）
 *    - 陀螺仪相邻角速度梯形积分推进四元数；
 *    - 加速度计只做比例纠偏，模长明显偏离 1g 时降低/暂停，模长过小时只跳过纠偏；
 *    - 上电由一段静止样本的平均加速度定横滚/俯仰（允许斜放、倒放），偏航置 0；
 *    - 每次更新后归一化。
 *
 *  全部 32 位整数实现（工具链没有 libgcc/软浮点，不能用 float）。
 *  配置全部来自 UserEuler.h，本文件不再重复定义阈值。
 */
//********************************************************************
#include "UserEuler.h"
//********************************************************************

// 32 位无符号整数平方根（向下取整）
static att_u32 euler_isqrt(att_u32 x)
{
	att_u32 res = 0;
	att_u32 bit = 1u << 30;

	while (bit > x) {
		bit >>= 2;
	}
	while (bit) {
		if (x >= res + bit) {
			x -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

// atan2 近似：输入 Q15 的 y、x，输出 0.01 度，范围 (-18000, 18000]
//   atan(z) ≈ 45°·z + z(1-z)(14.02° + 3.80°·z)，z ∈ [0,1]，误差 < 0.15°
static att_i32 euler_atan2_deg100(att_i32 y, att_i32 x)
{
	att_i32 ax = (x < 0) ? -x : x;
	att_i32 ay = (y < 0) ? -y : y;
	att_i32 z, deg, t1, t2, corr;
	att_u8 swap = 0;

	if (ax == 0 && ay == 0) {
		return 0;
	}
	if (ay > ax) {              // 保证 z = 小/大 ∈ [0,1]
		att_i32 tmp = ax;
		ax = ay;
		ay = tmp;
		swap = 1;
	}

	z = (att_i32)(((att_u32)ay << 15) / (att_u32)ax);       // Q15

	deg = (45000 * z) >> 15;                                // 45°·z（0.001°）
	t1  = (z * (ATT_Q15_ONE - z)) >> 15;                    // z(1-z)，Q15
	t2  = 14020 + ((3800 * z) >> 15);                       // 14.02° + 3.80°·z（0.001°）
	corr = (t1 * t2) >> 15;
	deg += corr;

	if (swap) {
		deg = 90000 - deg;                                  // 90° - atan(1/z)
	}
	if (x < 0) {
		deg = 180000 - deg;
	}
	if (y < 0) {
		deg = -deg;
	}
	return deg / 10;                                        // 0.001° → 0.01°
}

// 对称舍入的右移：正负都按四舍五入，避免负值向下取整带来的系统性偏置
static att_i32 euler_round_shift(att_i32 v, att_u8 shift)
{
	att_i32 half = (att_i32)1 << (shift - 1);

	if (v >= 0) {
		return (v + half) >> shift;
	}
	return -(((-v) + half) >> shift);
}

// 带余数的平均：avg = sum/2，余数累积到下一拍，长期没有取整偏置
//   （直接用 >>1 或四舍五入都会让"0/-1 交替"这类输入产生固定方向漂移）
static att_i32 euler_avg2(att_i32 *rem, att_i32 sum)
{
	att_i32 avg = sum >> 1;         // 向下取整
	att_i32 r = sum & 1;            // 0 或 1（负数同样是 1）

	*rem += r;
	if (*rem >= 2) {
		avg += 1;
		*rem -= 2;
	}
	return avg;
}

// 四元数归一化
static att_u8 euler_normalize(att_i32 q[4])
{
	att_u32 n2;
	att_u32 n;

	n2 = (att_u32)q[0] * (att_u32)q[0] + (att_u32)q[1] * (att_u32)q[1]
	   + (att_u32)q[2] * (att_u32)q[2] + (att_u32)q[3] * (att_u32)q[3];
	n = euler_isqrt(n2);
	if (n == 0) {
		return 0;
	}
	q[0] = (att_i32)(((att_i32)q[0] * ATT_Q15_ONE) / (att_i32)n);
	q[1] = (att_i32)(((att_i32)q[1] * ATT_Q15_ONE) / (att_i32)n);
	q[2] = (att_i32)(((att_i32)q[2] * ATT_Q15_ONE) / (att_i32)n);
	q[3] = (att_i32)(((att_i32)q[3] * ATT_Q15_ONE) / (att_i32)n);
	return 1;
}

//********************************************************************
// 对外接口
//********************************************************************
att_i32 Euler_GyroToQ10(att_i32 raw_lsb, att_i32 bias_q6, att_i32 *rem)
{
	// 原始计数先放大到 1/64 LSB，与实测零偏同单位相减，精度不会被整数 LSB 截掉
	att_i32 delta_q6 = (raw_lsb << 6) - bias_q6;
	att_i32 total;
	att_i32 q10;

	// 1 LSB = 1024/16.4 × π/180 ≈ 1.08966 (Q10)；
	// 换到 1/64 LSB 就是 1.08966/64 ≈ 278.95/16384，取 279/16384（误差 0.02%）
	total = delta_q6 * 279 + *rem;

	// 余数补偿：只取整数部分，小数留到下一拍，零速噪声不会被"总是朝零"的
	// 移位吃掉，静置偏航因此不会缓慢累积
	q10 = total / 16384;              // 朝零截断
	*rem = total - q10 * 16384;       // 余数在 (-16384, 16384)

	return q10;
}

void Euler_Reset(euler_q_t *m)
{
	m->q[0] = ATT_Q15_ONE;          // 单位四元数
	m->q[1] = 0;
	m->q[2] = 0;
	m->q[3] = 0;
	m->gyro_prev[0] = 0;
	m->gyro_prev[1] = 0;
	m->gyro_prev[2] = 0;
	m->dq_rem[0] = 0;
	m->dq_rem[1] = 0;
	m->dq_rem[2] = 0;
	m->dq_rem[3] = 0;
	m->trap_rem[0] = 0;
	m->trap_rem[1] = 0;
	m->trap_rem[2] = 0;
	m->have_prev = 0;
	m->ready = 0;
	m->t_ms = 0;
}

att_u8 Euler_InitFromAccel(euler_q_t *m, const att_i32 acc_mg[3], att_u32 t_ms)
{
	att_i32 ax, ay, az, n2, n;
	att_i32 sin_t, cos_t, sin_p, cos_p;
	att_i32 cth, sth, cph, sph;

	n2 = acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2];
	n = (att_i32)euler_isqrt((att_u32)n2);
	if (n < 300) {                  // 小于 0.3g：读数不可信
		return 0;
	}

	// 单位化（Q15）
	ax = (att_i32)(((att_i32)acc_mg[0] * ATT_Q15_ONE) / n);
	ay = (att_i32)(((att_i32)acc_mg[1] * ATT_Q15_ONE) / n);
	az = (att_i32)(((att_i32)acc_mg[2] * ATT_Q15_ONE) / n);

	// a = (sinθ, -sinφcosθ, -cosφcosθ)：θ 为俯仰、φ 为横滚
	sin_t = ax;                                                 // sinθ
	cos_t = (att_i32)euler_isqrt((att_u32)(ay * ay + az * az)); // cosθ
	if (cos_t < 1) {
		cos_t = 1;
	}
	sin_p = (att_i32)(((att_i32)(-ay) * ATT_Q15_ONE) / cos_t);  // sinφ
	cos_p = (att_i32)(((att_i32)(-az) * ATT_Q15_ONE) / cos_t);  // cosφ

	// 半角：cos(θ/2)=sqrt((1+cosθ)/2)，sin(θ/2)=sinθ/(2cos(θ/2))
	// 小角度时 1-cosθ 会相消放大误差，所以 sin 半角用 sin/2cos 计算
	cth = (att_i32)euler_isqrt((att_u32)((ATT_Q15_ONE + cos_t) * 16384));
	if (cth < 1) {
		cth = 1;
	}
	sth = (att_i32)(((att_i32)sin_t * 16384) / cth);

	cph = (att_i32)euler_isqrt((att_u32)((ATT_Q15_ONE + cos_p) * 16384));
	if (cph >= 16) {
		sph = (att_i32)(((att_i32)sin_p * 16384) / cph);
	} else {
		// 精确倒放（φ≈±180°）时 cos(φ/2)≈0，除法退化，
		// 改用 sqrt((1-cosφ)/2)；符号取 sinφ 的符号（sinφ≈0 时取正）
		sph = (att_i32)euler_isqrt((att_u32)((ATT_Q15_ONE - cos_p) * 16384));
		if (sin_p < 0) {
			sph = -sph;
		}
	}

	// q = qy(θ) ⊗ qx(φ)，偏航取 0
	m->q[0] = (att_i32)(((att_i32)cth * cph) >> 15);
	m->q[1] = (att_i32)(((att_i32)cth * sph) >> 15);
	m->q[2] = (att_i32)(((att_i32)sth * cph) >> 15);
	m->q[3] = (att_i32)(-(((att_i32)sth * sph) >> 15));

	if (!euler_normalize(m->q)) {
		return 0;
	}

	m->gyro_prev[0] = 0;
	m->gyro_prev[1] = 0;
	m->gyro_prev[2] = 0;
	m->dq_rem[0] = 0;
	m->dq_rem[1] = 0;
	m->dq_rem[2] = 0;
	m->dq_rem[3] = 0;
	m->trap_rem[0] = 0;
	m->trap_rem[1] = 0;
	m->trap_rem[2] = 0;
	m->have_prev = 0;
	m->ready = 1;
	m->t_ms = t_ms;
	return 1;
}

att_u8 Euler_Update(euler_q_t *m, const att_i32 acc_mg[3], const att_i32 gyro_q10[3],
                    att_i32 dt_100us, att_u32 t_ms)
{
	att_i32 q0, q1, q2, q3;
	att_i32 vx, vy, vz;             // 预测的"上"方向（机体 FRD，Q15）
	att_i32 ex = 0, ey = 0, ez = 0; // 叉积误差（Q15）
	att_i32 trust, kp = 0;
	att_i32 wx, wy, wz;             // 合成角速度（Q10 rad/s）
	att_i32 t0, t1, t2, t3;         // q⊗ω（Q24）
	att_u32 n2;
	att_i32 n;

	if (!m->ready || dt_100us <= 0) {
		return 0;
	}

	q0 = m->q[0];
	q1 = m->q[1];
	q2 = m->q[2];
	q3 = m->q[3];

	// 预测静止比力方向：v = R^T × (0,0,-1)，Q15
	vx = (att_i32)((((att_i32)q0 * q2 - (att_i32)q1 * q3)) >> 14);
	vy = (att_i32)(-((((att_i32)q0 * q1 + (att_i32)q2 * q3))) >> 14);
	vz = (att_i32)((((att_i32)q1 * q1 + (att_i32)q2 * q2)) >> 14) - ATT_Q15_ONE;

	// 加速度模长；太小（自由落体/剧烈运动）时只跳过纠偏，陀螺继续积分
	n2 = (att_u32)(acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2]);
	n = (att_i32)euler_isqrt(n2);

	// 先按模长算纠偏权重：权重为 0（自由落体/剧烈运动）时跳过归一化与叉积，
	// 只保留陀螺积分，省掉每拍 3 次整数除法
	trust = 0;
	if (n >= C_ATT_TRUST_FULL_LO_MG && n <= C_ATT_TRUST_FULL_HI_MG) {
		trust = ATT_Q15_ONE;
	} else if (n > C_ATT_TRUST_MIN_MG && n < C_ATT_TRUST_FULL_LO_MG) {
		trust = (att_i32)(((n - C_ATT_TRUST_MIN_MG) * ATT_Q15_ONE)
		                  / (C_ATT_TRUST_FULL_LO_MG - C_ATT_TRUST_MIN_MG));
	} else if (n > C_ATT_TRUST_FULL_HI_MG && n < C_ATT_TRUST_MAX_MG) {
		trust = (att_i32)(((C_ATT_TRUST_MAX_MG - n) * ATT_Q15_ONE)
		                  / (C_ATT_TRUST_MAX_MG - C_ATT_TRUST_FULL_HI_MG));
	}
	if (trust > 0) {
		att_i32 ax, ay, az;

		ax = (att_i32)(((att_i32)acc_mg[0] * ATT_Q15_ONE) / n);
		ay = (att_i32)(((att_i32)acc_mg[1] * ATT_Q15_ONE) / n);
		az = (att_i32)(((att_i32)acc_mg[2] * ATT_Q15_ONE) / n);

		// 误差 = 实测上方向 × 预测上方向
		ex = (att_i32)((((att_i32)ay * vz - (att_i32)az * vy)) >> 15);
		ey = (att_i32)((((att_i32)az * vx - (att_i32)ax * vz)) >> 15);
		ez = (att_i32)((((att_i32)ax * vy - (att_i32)ay * vx)) >> 15);
		kp = (att_i32)(((att_i32)C_MAHONY_KP_Q15 * trust) >> 15);
	}

	// 梯形积分：相邻两拍角速度取平均
	if (m->have_prev) {
		// 相邻两拍取平均：余数累积，避免"0/-1 交替"这类输入的固定漂移
		wx = euler_avg2(&m->trap_rem[0], m->gyro_prev[0] + gyro_q10[0]);
		wy = euler_avg2(&m->trap_rem[1], m->gyro_prev[1] + gyro_q10[1]);
		wz = euler_avg2(&m->trap_rem[2], m->gyro_prev[2] + gyro_q10[2]);
	} else {
		wx = gyro_q10[0];
		wy = gyro_q10[1];
		wz = gyro_q10[2];
		m->have_prev = 1;
	}
	m->gyro_prev[0] = gyro_q10[0];
	m->gyro_prev[1] = gyro_q10[1];
	m->gyro_prev[2] = gyro_q10[2];

	// 叠加比例纠正（Q15·Q15 → Q30 → Q10）
	wx += euler_round_shift(kp * ex, 20);
	wy += euler_round_shift(kp * ey, 20);
	wz += euler_round_shift(kp * ez, 20);

	// q̇ = 0.5 × q ⊗ ω（Q24），用更新前的 q
	// q 与 ω 相乘后要除以 2：同样用对称舍入，否则负角速度一侧会多积一点，
	// 方波/往复输入 60 秒后会累积成可见的偏航漂移
	t0 = -euler_round_shift((att_i32)q1 * wx, 1) - euler_round_shift((att_i32)q2 * wy, 1)
	     - euler_round_shift((att_i32)q3 * wz, 1);
	t1 =  euler_round_shift((att_i32)q0 * wx, 1) + euler_round_shift((att_i32)q2 * wz, 1)
	     - euler_round_shift((att_i32)q3 * wy, 1);
	t2 =  euler_round_shift((att_i32)q0 * wy, 1) - euler_round_shift((att_i32)q1 * wz, 1)
	     + euler_round_shift((att_i32)q3 * wx, 1);
	t3 =  euler_round_shift((att_i32)q0 * wz, 1) + euler_round_shift((att_i32)q1 * wy, 1)
	     - euler_round_shift((att_i32)q2 * wx, 1);

	// Δq = t × dt / 1.024e7（dt 单位 0.1ms）
	// 整数除法会截断，余数累计到下一拍，避免缓慢旋转系统性偏慢
	{
		att_i32 t[4];
		att_i32 qn[4];
		att_i32 k;

		t[0] = t0; t[1] = t1; t[2] = t2; t[3] = t3;
		qn[0] = q0; qn[1] = q1; qn[2] = q2; qn[3] = q3;
		for (k = 0; k < 4; k++) {
			// t 是 Q24，先对称舍入到 Q14 再乘 dt（余数由 dq_rem 补偿）
			att_i32 num = euler_round_shift(t[k], 10) * dt_100us;
			att_i32 dq = num / 10000;
			att_i32 rem = num % 10000;

			m->dq_rem[k] += rem;
			if (m->dq_rem[k] >= 10000) {
				dq += 1;
				m->dq_rem[k] -= 10000;
			} else if (m->dq_rem[k] <= -10000) {
				dq -= 1;
				m->dq_rem[k] += 10000;
			}
			m->q[k] = qn[k] + dq;
		}
	}

	if (!euler_normalize(m->q)) {
		return 0;
	}

	m->t_ms = t_ms;
	return 1;
}

att_u8 Euler_IsStill(const att_i32 acc_mg[3], const att_i32 gyr_q10[3],
                     const att_i32 prev_acc[3], att_u8 have_prev)
{
	att_i32 n2;
	att_i32 lo = (att_i32)C_ATT_INIT_ACC_LO_MG;
	att_i32 hi = (att_i32)C_ATT_INIT_ACC_HI_MG;
	int i;

	n2 = acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2];
	if (n2 < lo * lo || n2 > hi * hi) {
		return 0;                       // 模长不像静止
	}
	for (i = 0; i < 3; i++) {
		if (gyr_q10[i] > C_ATT_INIT_GYRO_MAX_Q10
		 || gyr_q10[i] < -C_ATT_INIT_GYRO_MAX_Q10) {
			return 0;                   // 还在转/晃
		}
		if (have_prev) {
			att_i32 d = acc_mg[i] - prev_acc[i];
			if (d > C_ATT_INIT_ACC_DEV_MG || d < -C_ATT_INIT_ACC_DEV_MG) {
				return 0;               // 相邻样本跳变
			}
		}
	}
	return 1;
}

void Euler_GetEulerDeg100(const euler_q_t *m, att_i32 *roll, att_i32 *pitch, att_i32 *yaw)
{
	att_i32 q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];
	att_i32 r00, r10, r20, r21, r22, root;

	// R（机体 FRD → 水平系），Q15
	r00 = ATT_Q15_ONE - (att_i32)((((att_i32)q2 * q2 + (att_i32)q3 * q3)) >> 14);
	r10 = (att_i32)((((att_i32)q1 * q2 + (att_i32)q0 * q3)) >> 14);
	r20 = (att_i32)((((att_i32)q1 * q3 - (att_i32)q0 * q2)) >> 14);
	r21 = (att_i32)((((att_i32)q2 * q3 + (att_i32)q0 * q1)) >> 14);
	r22 = ATT_Q15_ONE - (att_i32)((((att_i32)q1 * q1 + (att_i32)q2 * q2)) >> 14);

	if (roll) {
		*roll = euler_atan2_deg100(r21, r22);
	}
	if (pitch) {
		att_i32 r20c = r20;

		if (r20c > ATT_Q15_ONE) {           // 量化误差可能略超 1，先限幅
			r20c = ATT_Q15_ONE;
		} else if (r20c < -ATT_Q15_ONE) {
			r20c = -ATT_Q15_ONE;
		}
		root = (att_i32)euler_isqrt((att_u32)((att_i32)ATT_Q15_ONE * (att_i32)ATT_Q15_ONE
		                                     - r20c * r20c));
		*pitch = euler_atan2_deg100(-r20c, root);
	}
	if (yaw) {
		*yaw = euler_atan2_deg100(r10, r00);
	}
}
