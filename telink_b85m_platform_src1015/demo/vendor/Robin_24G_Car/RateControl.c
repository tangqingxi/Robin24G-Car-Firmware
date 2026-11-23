/*
 * RateControl.c
 *
 * 三轴角速度内环（PID，默认只有 P）。
 *
 * 数据流：
 *   AttitudeControl 输出 target[3]（Q10 rad/s）
 *   − 陀螺实测 gyro[3]（Q10 rad/s）
 *   = err（Q10）
 *   → P/I/D 组合成 corr[3]（Q10，约 ±FLIGHT_CORR_LIMIT_Q10）
 *   → MotorMixer 把 corr 换算成四路 PWM 差动。
 *
 * 符号约定：target 和 gyro 都用同一套机体系角速度正方向，
 * 因此 err 为正表示“需要产生正方向角加速度”，混控器再按
 * 横滚/俯仰/偏航的桨叶几何分配差动。
 *
 * 定点比例：
 *   - err、target、gyro 都是 Q10；
 *   - FLIGHT_KP_Q8 是 Q8 增益，err×Kp>>8 仍为 Q10；
 *   - I 项把 err 直接累加（未乘 dt），FLIGHT_KI_Q16 已按 200Hz 整定；
 *   - D 项用相邻两拍误差差分，未除 dt，同样按 200Hz 整定。
 *
 * 默认 FLIGHT_KI_Q16=0、FLIGHT_KD_Q8=0，I/D 状态不参与编译；
 * 这样不会因为未使用积分器而多占 RAM，也不会出现隐藏 windup。
 */
#include "RateControl.h"

// 有符号向零截断：先取绝对值右移，再恢复符号。
// 只用于偏航 P，保证 +e 和 -e 得到等幅结果；其他轴保持原换算。
static int rate_yaw_shift(int v, int shift)
{
	int a = (v < 0) ? -v : v;

	a >>= shift;
	return (v < 0) ? -a : a;
}

// 对称限幅：积分器、纠偏输出共用，避免异常输入把后级推到饱和。
static int rate_clamp(int v, int lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

// 内环初始化：纠偏归零；只有编译了 I/D 项时才清对应状态。
// prev_err 清空并令 have_prev=0，保证第一个 D 拍不拿“上电前的误差”做差分。
void RateControl_Init(RateControl *rc)
{
	int i;

	for (i = 0; i < 3; i++) {
		rc->corr[i] = 0;
#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] = 0;
#endif
#if (FLIGHT_KD_Q8 != 0)
		rc->prev_err[i] = 0;
#endif
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 0;
#endif
}

// 内环：误差=目标角速度−实测角速度；P=64/256，I/D=0 时对应状态不编译。
//
// 每轴独立计算：
//   1) err = target - gyro；
//   2) P 项 = err × 轴增益 >> 8；偏航用 FLIGHT_YAW_KP_Q8=512，其余=64；
//   3) 若启用 I：integ += err，先限幅再乘 Ki；
//   4) 若启用 D：d_err = err - prev_err，乘 Kd；
//   5) corr 限幅后交给混控。
//
// 积分器直接累加误差而不是 err×dt：本工程固定 200Hz，Ki 已按该周期整定；
// 如果以后改控制频率，必须同步重算 Ki/Kd，否则等效增益会变。
// 积分限幅在乘 Ki 之前，目的是限制积分状态而不是限制最终输出。
void RateControl_Update(RateControl *rc, const int target[3],
						const int gyro[3])
{
	int i;

	for (i = 0; i < 3; i++) {
		int err = target[i] - gyro[i];
		// 偏航独立 P=0.5（Q8=128）；横滚/俯仰仍为 P=0.25（Q8=64）。
		int corr = (i == 2)
				   ? rate_yaw_shift(err * FLIGHT_YAW_KP_Q8, 8)
				   : ((err * FLIGHT_KP_Q8) >> 8);

#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] += err;
		rc->integ[i] = rate_clamp(rc->integ[i], FLIGHT_RATE_INT_MAX);
		corr += (rc->integ[i] * FLIGHT_KI_Q16) >> 16;
#endif
#if (FLIGHT_KD_Q8 != 0)
		{
			int d_err = rc->have_prev ? (err - rc->prev_err[i]) : 0;

			rc->prev_err[i] = err;
			corr += (d_err * FLIGHT_KD_Q8) >> 8;
		}
#endif
		rc->corr[i] = rate_clamp(corr, FLIGHT_CORR_LIMIT_Q10);
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 1;
#endif
}

// 清内环历史：输出、积分、微分前一拍误差全部归零。
// 失联/急停后调用，防止重新解锁时接着使用旧积分或旧误差差分。
void RateControl_ClearOutputs(RateControl *rc)
{
	int i;

	for (i = 0; i < 3; i++) {
		rc->corr[i] = 0;
#if (FLIGHT_KI_Q16 != 0)
		rc->integ[i] = 0;
#endif
#if (FLIGHT_KD_Q8 != 0)
		rc->prev_err[i] = 0;
#endif
	}
#if (FLIGHT_KD_Q8 != 0)
	rc->have_prev = 0;
#endif
}
