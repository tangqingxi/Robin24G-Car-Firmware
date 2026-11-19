/***************************************************************************************
 *  File: AttMahony.c
 *
 *  Mahony 互补滤波（定点）：陀螺仪梯形积分 + 加速度计慢速纠正
 *  全部使用 32 位整数：四元数 Q15、角速度 Q10 rad/s、加速度 mg。
 *  纯算法模块，可在 PC 上单独编译做单元测试。
 ****************************************************************************************/
#include "AttMahony.h"

// Mahony 参数（与 AttConfig.h 对应；这里给默认值，便于本文件单独编译测试）
#ifndef C_MAHONY_KP_Q15
#define C_MAHONY_KP_Q15         16384   // 0.5
#endif
#ifndef C_ATT_TRUST_MIN_MG
#define C_ATT_TRUST_MIN_MG      700     // 0.70g
#endif
#ifndef C_ATT_TRUST_FULL_LO_MG
#define C_ATT_TRUST_FULL_LO_MG  850     // 0.85g 起完全信任
#endif
#ifndef C_ATT_TRUST_FULL_HI_MG
#define C_ATT_TRUST_FULL_HI_MG  1150    // 1.15g 以内完全信任
#endif
#ifndef C_ATT_TRUST_MAX_MG
#define C_ATT_TRUST_MAX_MG      1300    // 1.30g 以上完全暂停
#endif

// -------------------------------------------------------------------------------------
// 工具：整数平方根与 atan2 近似（无浮点）
// -------------------------------------------------------------------------------------

// 32 位无符号整数平方根（结果向下取整）
static att_u32 att_isqrt(att_u32 x)
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

// atan2 近似：输入 Q15 的 y、x，输出单位 0.01 度，范围 (-18000, 18000]
// 用 atan(z) ≈ 45°·z + z(1-z)(14.02° + 3.80°·z)（z ∈ [0,1]，误差 < 0.15°）
static att_i32 att_atan2_deg100(att_i32 y, att_i32 x)
{
	att_i32 ax = (x < 0) ? -x : x;
	att_i32 ay = (y < 0) ? -y : y;
	att_i32 z, deg, t1, t2, corr;
	att_u8 swap = 0;

	if (ax == 0 && ay == 0) {
		return 0;
	}
	if (ay > ax) {          // 保证 z = 小/大 ∈ [0,1]
		att_i32 tmp = ax;
		ax = ay;
		ay = tmp;
		swap = 1;
	}

	z = (att_i32)(((att_u32)ay << 15) / (att_u32)ax);       // Q15

	deg = (45000 * z) >> 15;                                // 45°·z，单位 0.001°
	t1  = (z * (ATT_Q15_ONE - z)) >> 15;                    // z(1-z)，Q15
	t2  = 14020 + ((3800 * z) >> 15);                       // 14.02° + 3.80°·z，单位 0.001°
	corr = (t1 * t2) >> 15;                                 // 修正项，单位 0.001°
	deg += corr;

	if (swap) {
		deg = 90000 - deg;                                  // atan(v) → 90° - atan(1/v)
	}
	if (x < 0) {
		deg = 180000 - deg;
	}
	if (y < 0) {
		deg = -deg;
	}
	return deg / 10;                                        // 0.001° → 0.01°
}

// 归一化四元数（Q15）
static att_u8 att_normalize(att_i32 q[4])
{
	att_u32 n2 = 0;
	att_u32 n;

	n2 = (att_u32)q[0] * (att_u32)q[0] + (att_u32)q[1] * (att_u32)q[1]
	   + (att_u32)q[2] * (att_u32)q[2] + (att_u32)q[3] * (att_u32)q[3];
	n = att_isqrt(n2);
	if (n == 0) {
		return 0;
	}
	q[0] = (att_i32)(((att_i32)q[0] * ATT_Q15_ONE) / (att_i32)n);
	q[1] = (att_i32)(((att_i32)q[1] * ATT_Q15_ONE) / (att_i32)n);
	q[2] = (att_i32)(((att_i32)q[2] * ATT_Q15_ONE) / (att_i32)n);
	q[3] = (att_i32)(((att_i32)q[3] * ATT_Q15_ONE) / (att_i32)n);
	return 1;
}

// -------------------------------------------------------------------------------------
// 对外接口
// -------------------------------------------------------------------------------------
void AttMahony_Reset(att_mahony_t *m)
{
	m->q[0] = ATT_Q15_ONE;      // 单位四元数
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
	m->have_prev = 0;
	m->ready = 0;
	m->t_ms = 0;
}

att_u8 AttMahony_InitFromAccel(att_mahony_t *m, const att_i32 acc_mg[3], att_u32 t_ms)
{
	att_i32 ax, ay, az, n2, n;
	att_i32 sin_t, cos_t, sin_p, cos_p;
	att_i32 cth, sth, cph, sph;

	n2 = acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2];
	n = (att_i32)att_isqrt((att_u32)n2);
	if (n < 300) {              // 小于 0.3g：读数不可信
		return 0;
	}

	// 单位化（Q15）
	ax = (att_i32)(((att_i32)acc_mg[0] * ATT_Q15_ONE) / n);
	ay = (att_i32)(((att_i32)acc_mg[1] * ATT_Q15_ONE) / n);
	az = (att_i32)(((att_i32)acc_mg[2] * ATT_Q15_ONE) / n);

	// 由 a = (sinθ, -sinφcosθ, -cosφcosθ) 求俯仰 θ 与横滚 φ
	sin_t = ax;                                                 // sinθ = ax
	cos_t = (att_i32)att_isqrt((att_u32)(ay * ay + az * az));   // cosθ，Q15
	if (cos_t < 1) {
		cos_t = 1;
	}
	sin_p = (att_i32)(((att_i32)(-ay) * ATT_Q15_ONE) / cos_t);  // sinφ
	cos_p = (att_i32)(((att_i32)(-az) * ATT_Q15_ONE) / cos_t);  // cosφ

	// 半角：cos(θ/2)=sqrt((1+cosθ)/2)，sin(θ/2)=sinθ/(2cos(θ/2))
	// 小角度时 1-cosθ 会相消放大误差，所以 sin 半角用 sin/2cos 计算
	cth = (att_i32)att_isqrt((att_u32)((ATT_Q15_ONE + cos_t) * 16384));
	cph = (att_i32)att_isqrt((att_u32)((ATT_Q15_ONE + cos_p) * 16384));
	if (cth < 1) cth = 1;
	if (cph < 1) cph = 1;
	sth = (att_i32)(((att_i32)sin_t * 16384) / cth);      // sinθ/2cos(θ/2)
	sph = (att_i32)(((att_i32)sin_p * 16384) / cph);      // sinφ/2cos(φ/2)

	// q = qy(θ) ⊗ qx(φ)，偏航取 0
	m->q[0] = (att_i32)(((att_i32)cth * cph) >> 15);
	m->q[1] = (att_i32)(((att_i32)cth * sph) >> 15);
	m->q[2] = (att_i32)(((att_i32)sth * cph) >> 15);
	m->q[3] = (att_i32)(-(((att_i32)sth * sph) >> 15));

	if (!att_normalize(m->q)) {
		return 0;
	}

	m->gyro_prev[0] = 0;
	m->gyro_prev[1] = 0;
	m->gyro_prev[2] = 0;
	m->have_prev = 0;
	m->ready = 1;
	m->t_ms = t_ms;
	return 1;
}

att_u8 AttMahony_Update(att_mahony_t *m, const att_i32 acc_mg[3], const att_i32 gyro_q10[3],
                        att_i32 dt_100us, att_u32 t_ms)
{
	att_i32 q0, q1, q2, q3;
	att_i32 vx, vy, vz;             // 预测的"上"方向（机体 FRD，Q15）
	att_i32 ax, ay, az, n2, n;
	att_i32 ex, ey, ez;             // 叉积误差（Q15）
	att_i32 trust, kp;
	att_i32 wx, wy, wz;             // 合成角速度（Q10 rad/s）
	att_i32 t0, t1, t2, t3;         // q⊗ω（Q24）

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

	// 加速度归一化（Q15）
	n2 = acc_mg[0] * acc_mg[0] + acc_mg[1] * acc_mg[1] + acc_mg[2] * acc_mg[2];
	n = (att_i32)att_isqrt((att_u32)n2);
	if (n < 50) {
		return 0;
	}
	ax = (att_i32)(((att_i32)acc_mg[0] * ATT_Q15_ONE) / n);
	ay = (att_i32)(((att_i32)acc_mg[1] * ATT_Q15_ONE) / n);
	az = (att_i32)(((att_i32)acc_mg[2] * ATT_Q15_ONE) / n);

	// 误差 = 实测上方向 × 预测上方向（Q15）
	ex = (att_i32)((((att_i32)ay * vz - (att_i32)az * vy)) >> 15);
	ey = (att_i32)((((att_i32)az * vx - (att_i32)ax * vz)) >> 15);
	ez = (att_i32)((((att_i32)ax * vy - (att_i32)ay * vx)) >> 15);

	// 模长明显偏离 1g 时降低/暂停纠偏（线性过渡）
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
	kp = (att_i32)(((att_i32)C_MAHONY_KP_Q15 * trust) >> 15);

	// 梯形积分：相邻两拍角速度取平均
	if (m->have_prev) {
		wx = (m->gyro_prev[0] + gyro_q10[0]) >> 1;
		wy = (m->gyro_prev[1] + gyro_q10[1]) >> 1;
		wz = (m->gyro_prev[2] + gyro_q10[2]) >> 1;
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
	wx += (att_i32)(((att_i32)kp * ex) >> 20);
	wy += (att_i32)(((att_i32)kp * ey) >> 20);
	wz += (att_i32)(((att_i32)kp * ez) >> 20);

	// q̇ = 0.5 × q ⊗ ω（Q24），用更新前的 q
	t0 = -(((att_i32)q1 * wx) >> 1) - (((att_i32)q2 * wy) >> 1) - (((att_i32)q3 * wz) >> 1);
	t1 =  (((att_i32)q0 * wx) >> 1) + (((att_i32)q2 * wz) >> 1) - (((att_i32)q3 * wy) >> 1);
	t2 =  (((att_i32)q0 * wy) >> 1) - (((att_i32)q1 * wz) >> 1) + (((att_i32)q3 * wx) >> 1);
	t3 =  (((att_i32)q0 * wz) >> 1) + (((att_i32)q1 * wy) >> 1) - (((att_i32)q2 * wx) >> 1);

	// Δq = t × dt / 1.024e7（dt 单位 0.1ms）
	// 整数除法会截断，余数累计到下一拍，否则缓慢旋转会系统性偏慢
	{
		att_i32 t[4];
		att_i32 qn[4];
		att_i32 k;

		t[0] = t0; t[1] = t1; t[2] = t2; t[3] = t3;
		qn[0] = q0; qn[1] = q1; qn[2] = q2; qn[3] = q3;
		for (k = 0; k < 4; k++) {
			att_i32 num = (t[k] >> 10) * dt_100us;
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

	if (!att_normalize(m->q)) {
		return 0;
	}

	m->t_ms = t_ms;
	return 1;
}

void AttMahony_GetEulerDeg100(const att_mahony_t *m, att_i32 *roll, att_i32 *pitch, att_i32 *yaw)
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
		*roll = att_atan2_deg100(r21, r22);
	}
	if (pitch) {
		att_i32 r20c = r20;

		if (r20c > ATT_Q15_ONE) {          // 量化误差可能略超 1，先限幅
			r20c = ATT_Q15_ONE;
		} else if (r20c < -ATT_Q15_ONE) {
			r20c = -ATT_Q15_ONE;
		}
		// pitch = atan2(-r20, sqrt(1 - r20²))
		root = (att_i32)att_isqrt((att_u32)((att_i32)ATT_Q15_ONE * (att_i32)ATT_Q15_ONE
		                                    - r20c * r20c));
		*pitch = att_atan2_deg100(-r20c, root);
	}
	if (yaw) {
		*yaw = att_atan2_deg100(r10, r00);
	}
}
