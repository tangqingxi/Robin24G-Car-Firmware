#include "FlightLoop.h"
#include "FlightConfig.h"
#include "RcInput.h"
#include "AttitudeControl.h"
#include "RateControl.h"
#include "FlightSafety.h"
#include "MotorMixer.h"
#include "GroundDiag.h"
#include "UserEuler.h"
#include "my_t3488.h"

// 当前实飞基线：200 Hz / 5 ms 调度 FlightLoop_Process。
// 每拍顺序：RcInput 超时 → PreSampleSafety 先清四路 PWM
// → IMU 读取 → 姿态 → 外环/内环 → 故障判定 → 混控 → FlightSafety 输出。
// 正常镜像无板载日志；GroundDiag 仅由 C_GROUND_UART_DIAG=1 的构建启用。
// 与台架固件一致的校准常数；迁移控制链时保持数值不变。
static const s32 s_fc_acc_off_lsb[3] = { -3, -8, -29 };
static const s32 s_fc_acc_scale_q14[3] = { 16436, 16429, 16357 };
static const s32 s_fc_gyro_bias_q6[3] = { -93, -232, -381 };

// TC32 使用 -fpack-struct；这里逐个把跨模块指针/32 位成员对齐到 4 字节，
// 不能只给 s_fl 整体加 aligned(4)（基址对齐不能修复成员偏移）。
typedef struct __attribute__((aligned(4))) {
	RcInput rc __attribute__((aligned(4)));
	AttitudeControl attc __attribute__((aligned(4)));
	RateControl ratec __attribute__((aligned(4)));
	FlightSafety safety __attribute__((aligned(4)));

	euler_q_t att __attribute__((aligned(4)));
	s32 lpf[3] __attribute__((aligned(4)));
	s32 gyro_rem[3] __attribute__((aligned(4)));
	s32 rate[3] __attribute__((aligned(4)));
	s32 prev_acc[3] __attribute__((aligned(4)));
	s32 still_sum[3] __attribute__((aligned(4)));
	u32 still_ticks __attribute__((aligned(4)));
	u32 still_n __attribute__((aligned(4)));
	u32 last_ts __attribute__((aligned(4)));
	u32 dt_rem __attribute__((aligned(4)));
	u32 next_tick __attribute__((aligned(4)));
	u32 retry_tick __attribute__((aligned(4)));
	u32 lpf_alpha __attribute__((aligned(4)));
	u32 lpf_alpha_dt_us __attribute__((aligned(4)));
	u8 lpf_ok;
	u8 have_ts;
	u8 still_prev_ok;
	u8 inited;
} FlightLoopState;

static FlightLoopState s_fl;

// 编译期锁住指针接口的 4 字节地址；若以后增删成员导致偏移变化，构建直接失败。
#define FLIGHT_ALIGN4(expr) ((expr) & 3u)
typedef char flight_align_rc[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, rc)) ? -1 : 1];
typedef char flight_align_attc[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, attc)) ? -1 : 1];
typedef char flight_align_ratec[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, ratec)) ? -1 : 1];
typedef char flight_align_att[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, att)) ? -1 : 1];
typedef char flight_align_lpf[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, lpf)) ? -1 : 1];
typedef char flight_align_gyro_rem[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, gyro_rem)) ? -1 : 1];
typedef char flight_align_rate[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, rate)) ? -1 : 1];
typedef char flight_align_prev_acc[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, prev_acc)) ? -1 : 1];
typedef char flight_align_still_sum[FLIGHT_ALIGN4(__builtin_offsetof(FlightLoopState, still_sum)) ? -1 : 1];
typedef char flight_align_rc_cmd[FLIGHT_ALIGN4(__builtin_offsetof(RcInput, cmd)) ? -1 : 1];
typedef char flight_align_rc_pkt[FLIGHT_ALIGN4(__builtin_offsetof(RcInput, pkt_tick)) ? -1 : 1];
typedef char flight_align_event_cmd[FLIGHT_ALIGN4(__builtin_offsetof(RcInputEvent, cmd)) ? -1 : 1];
typedef char flight_align_euler_q[FLIGHT_ALIGN4(__builtin_offsetof(euler_q_t, q)) ? -1 : 1];
typedef char flight_align_euler_t[FLIGHT_ALIGN4(__builtin_offsetof(euler_q_t, t_ms)) ? -1 : 1];
typedef char flight_align_att_tgt[FLIGHT_ALIGN4(__builtin_offsetof(AttitudeControl, tgt_rate)) ? -1 : 1];
typedef char flight_align_rate_corr[FLIGHT_ALIGN4(__builtin_offsetof(RateControl, corr)) ? -1 : 1];
typedef char flight_align_mix_front[(__builtin_offsetof(MotorMixerOutput, front) & 1u) ? -1 : 1];

static void flight_still_reset(void)
{
	s_fl.still_ticks = 0;
	s_fl.still_n = 0;
	s_fl.still_sum[0] = 0;
	s_fl.still_sum[1] = 0;
	s_fl.still_sum[2] = 0;
}

static void flight_state_reset(void)
{
	int i;

	RcInput_Init(&s_fl.rc);
	AttitudeControl_Init(&s_fl.attc);
	RateControl_Init(&s_fl.ratec);
	FlightSafety_Init(&s_fl.safety);

	Euler_Reset(&s_fl.att);
	s_fl.have_ts = 0;
	s_fl.last_ts = 0;
	s_fl.dt_rem = 0;
	s_fl.lpf_ok = 0;
	s_fl.lpf_alpha = 0;
	s_fl.lpf_alpha_dt_us = 0;
	s_fl.still_prev_ok = 0;
	for (i = 0; i < 3; i++) {
		s_fl.gyro_rem[i] = 0;
		s_fl.lpf[i] = 0;
		s_fl.rate[i] = 0;
		s_fl.prev_acc[i] = 0;
	}
	flight_still_reset();
}

// 传感器初始化成功后一次性建立模块状态；失败 1s 后重试。
void FlightLoop_Init(void)
{
	u32 now = clock_time();
	u8 rb = 0;
	u8 st;

	if (s_fl.inited) {
		return;
	}
	if ((s32)(now - s_fl.retry_tick) < 0) {
		return;
	}
	s_fl.retry_tick = now + CLOCK_SYS_CLOCK_HZ;

	st = t3488_bench_init(C_GYRO_LPF_SEL, &rb);
	if (st) {
		(void)rb;
		return;
	}

	flight_state_reset();
	FlightSafety_InitPwm(&s_fl.safety);

	now = clock_time();
	s_fl.next_tick = now + FLIGHT_PERIOD_TICKS;
	s_fl.inited = 1;
}

// 启动阶段：连续静止 500ms 后取平均加速度定姿；移动/间断就重开窗口。
static void flight_startup(const s32 acc_mg[3], const s32 gyr_q10[3],
						   u32 dt_ticks, u32 ts)
{
	u8 still = Euler_IsStill(acc_mg, gyr_q10, s_fl.prev_acc, s_fl.still_prev_ok);
	int i;

	if (!still) {
		flight_still_reset();
	} else {
		s_fl.still_sum[0] += acc_mg[0];
		s_fl.still_sum[1] += acc_mg[1];
		s_fl.still_sum[2] += acc_mg[2];
		s_fl.still_n++;
		s_fl.still_ticks += dt_ticks;

		if (s_fl.still_ticks >= FLIGHT_INIT_STILL_TICKS && s_fl.still_n > 0) {
			s32 avg[3];

			// 静置和必须有符号；先把样本计数转 s32 再除，避免负数被当无符号。
			avg[0] = s_fl.still_sum[0] / (s32)s_fl.still_n;
			avg[1] = s_fl.still_sum[1] / (s32)s_fl.still_n;
			avg[2] = s_fl.still_sum[2] / (s32)s_fl.still_n;
			if (!Euler_InitFromAccel(&s_fl.att, avg,
									 ts / FLIGHT_TICKS_PER_MS)) {
				flight_still_reset();
			}
		}
	}
	for (i = 0; i < 3; i++) {
		s_fl.prev_acc[i] = acc_mg[i];
	}
	s_fl.still_prev_ok = 1;
}

// 收包路径：先消费 0001/0002，再更新同包目标；0001 在本包立即清四路。
void FlightLoop_OnRcPacket(unsigned short thr, unsigned short rot,
						   unsigned short fb, unsigned short lr,
						   unsigned short key)
{
	RcInputEvent ev;
	u32 now = clock_time();
	u8 thr_low, center, stop;

	if (!s_fl.inited) {
		return;
	}
	ev = RcInput_OnPacket(&s_fl.rc, thr, rot, fb, lr, key, now);

	thr_low = (ev.cmd[0] <= MOTOR_ARM_THR_MAX) ? 1 : 0;
	center = (ev.cmd[1] >= -MOTOR_ARM_STICK_MAX
			  && ev.cmd[1] <= MOTOR_ARM_STICK_MAX
			  && ev.cmd[2] >= -MOTOR_ARM_STICK_MAX
			  && ev.cmd[2] <= MOTOR_ARM_STICK_MAX
			  && ev.cmd[3] >= -MOTOR_ARM_STICK_MAX
			  && ev.cmd[3] <= MOTOR_ARM_STICK_MAX) ? 1 : 0;
	stop = FlightSafety_OnKey(&s_fl.safety, ev.key, ev.packet_valid,
							  thr_low, center, s_fl.att.ready,
							  s_fl.rc.link);
	if (stop) {
		// 急停优先：先锁存并写四路 0 PWM，不再做同包目标换算。
		FlightSafety_Lock(&s_fl.safety, stop);
		return;
	}
	if (ev.packet_valid) {
		AttitudeControl_OnStick(&s_fl.attc, ev.cmd);
	}
}

// 采样前安全门：失联/指令超时先把四路写 0，之后才允许读 IMU。
void FlightLoop_PreSampleSafety(void)
{
	if (!s_fl.rc.link) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_LINK);
	} else if (!s_fl.rc.cmd_ok) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_CMD);
	}
}

void FlightLoop_Process(void)
{
	short raw_a[3], raw_g[3];
	s32 acc_frd[3], acc_mg[3], gyr_q10[3];
	u32 now, planned;
	u32 dt_ticks = 0;
	s32 dt_100us = 0;
	u8 att_updated = 0;
	u8 ctrl_ok = 0;
	u8 imu_ok = 1;
	u8 mixer_limited = 0;
	int i;

	if (!s_fl.inited) {
		FlightLoop_Init();
		return;
	}

	now = clock_time();
	if ((s32)(now - s_fl.next_tick) < 0) {
		return;
	}

	planned = s_fl.next_tick;
	s_fl.next_tick = planned + FLIGHT_PERIOD_TICKS;
	if ((u32)(now - planned) >= FLIGHT_PERIOD_TICKS) {
		// 已错过至少一拍：从当前时刻重新排下一拍，不追发补拍。
		s_fl.next_tick = now + FLIGHT_PERIOD_TICKS;
	}

	RcInput_UpdateTimeouts(&s_fl.rc, now);
	// 必须在下面的 t3488_read_6axis/Euler 之前完成四路清零。
	FlightLoop_PreSampleSafety();

	{
		u8 read_fail = t3488_read_6axis(raw_a, raw_g);

		if (read_fail) {
			imu_ok = 0;
			AttitudeControl_ClearOutputs(&s_fl.attc);
			RateControl_ClearOutputs(&s_fl.ratec);
			FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_ATT);
		} else {
			u32 t_read1 = clock_time();

			acc_frd[0] = raw_a[0];
			acc_frd[1] = -raw_a[1];
			acc_frd[2] = -raw_a[2];
			gyr_q10[0] = Euler_GyroToQ10(raw_g[0], s_fc_gyro_bias_q6[0],
										 &s_fl.gyro_rem[0]);
			gyr_q10[1] = Euler_GyroToQ10(-raw_g[1], s_fc_gyro_bias_q6[1],
										 &s_fl.gyro_rem[1]);
			gyr_q10[2] = Euler_GyroToQ10(-raw_g[2], s_fc_gyro_bias_q6[2],
										 &s_fl.gyro_rem[2]);
			for (i = 0; i < 3; i++) {
				s32 lsb = (acc_frd[i] - s_fc_acc_off_lsb[i])
						  * s_fc_acc_scale_q14[i] >> 14;

				acc_mg[i] = (lsb * 125) >> 8;
				s_fl.rate[i] = gyr_q10[i];
			}

			if (!s_fl.have_ts) {
				s_fl.last_ts = t_read1;
				s_fl.have_ts = 1;
				dt_ticks = 0;
			} else {
				dt_ticks = t_read1 - s_fl.last_ts;
				s_fl.last_ts = t_read1;
			}
			if (dt_ticks > (u32)C_ATT_MAX_GAP_100US * FLIGHT_TICKS_PER_100US) {
				s_fl.dt_rem = 0;
				s_fl.att.have_prev = 0;
				flight_still_reset();
				dt_ticks = 0;
			}
			if (dt_ticks) {
				s_fl.dt_rem += dt_ticks;
				dt_100us = (s32)(s_fl.dt_rem / FLIGHT_TICKS_PER_100US);
				s_fl.dt_rem -= (u32)dt_100us * FLIGHT_TICKS_PER_100US;
			}

			if (!s_fl.lpf_ok) {
				s_fl.lpf[0] = acc_mg[0];
				s_fl.lpf[1] = acc_mg[1];
				s_fl.lpf[2] = acc_mg[2];
				s_fl.lpf_ok = 1;
			} else if (C_ACC_LPF_EN && dt_ticks > 0) {
				u32 dt_us = (dt_ticks + FLIGHT_TICKS_PER_US / 2)
							/ FLIGHT_TICKS_PER_US;
				s32 alpha;

				if (dt_us + 50 < s_fl.lpf_alpha_dt_us
					|| dt_us > s_fl.lpf_alpha_dt_us + 50) {
					s_fl.lpf_alpha = (u32)((dt_us * 32768u)
										   / (C_ACC_LPF_TAU_US + dt_us));
					s_fl.lpf_alpha_dt_us = dt_us;
				}
				alpha = (s32)s_fl.lpf_alpha;
				for (i = 0; i < 3; i++) {
					s_fl.lpf[i] += (alpha * (acc_mg[i] - s_fl.lpf[i])) >> 15;
				}
			} else if (!C_ACC_LPF_EN) {
				s_fl.lpf[0] = acc_mg[0];
				s_fl.lpf[1] = acc_mg[1];
				s_fl.lpf[2] = acc_mg[2];
			}

			if (!s_fl.att.ready) {
				flight_startup(s_fl.lpf, gyr_q10, dt_ticks, t_read1);
			} else if (dt_100us > 0) {
				if (Euler_Update(&s_fl.att, s_fl.lpf, gyr_q10, dt_100us,
								 t_read1 / FLIGHT_TICKS_PER_MS)) {
					FlightSafety_NoteAttitude(&s_fl.safety);
					att_updated = 1;
				}
			}
		}
	}

	if (att_updated && s_fl.rc.link && s_fl.rc.cmd_ok) {
		AttitudeControl_Update(&s_fl.attc, &s_fl.att);
		RateControl_Update(&s_fl.ratec, s_fl.attc.tgt_rate, s_fl.rate);
		ctrl_ok = 1;
	} else {
		AttitudeControl_ClearOutputs(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
	}

	FlightSafety_UpdateFaults(&s_fl.safety, s_fl.rc.link, s_fl.rc.cmd_ok,
							  att_updated, ctrl_ok);
	{
		MotorMixerOutput mix;

		MotorMixer_Mix(MotorMixer_Thrust(s_fl.rc.cmd[0]),
					   s_fl.ratec.corr[0], s_fl.ratec.corr[1],
					   s_fl.ratec.corr[2], &mix);
		mixer_limited = mix.limited;
		FlightSafety_Apply(&s_fl.safety, &mix);
	}

#if C_GROUND_UART_DIAG
	{
		unsigned short dpwm[4];

		FlightSafety_GetPwm(&s_fl.safety, dpwm);
		GroundDiag_OnControl(s_fl.rc.cmd[0], s_fl.rc.cmd[1],
							 s_fl.attc.tgt_rate,
							 s_fl.rate, s_fl.ratec.corr, dpwm,
							 raw_a, s_fl.att.q, imu_ok,
							 s_fl.safety.armed, mixer_limited);
	}
#endif
}

#ifdef FLIGHT_STARTUP_TEST
// 仅主机启动路径回归：读取真实 FlightLoop 内部状态。
unsigned char FlightLoop_StartupTestReady(void)
{
	return s_fl.att.ready;
}

unsigned char FlightLoop_StartupTestArmed(void)
{
	return s_fl.safety.armed;
}

unsigned int FlightLoop_StartupTestStillN(void) { return s_fl.still_n; }
unsigned int FlightLoop_StartupTestStillTicks(void) { return s_fl.still_ticks; }
int FlightLoop_StartupTestLpf(int i) { return s_fl.lpf[i]; }
void FlightLoop_StartupTestPwm(unsigned short out[4])
{
	FlightSafety_GetPwm(&s_fl.safety, out);
}
#endif

#ifdef FLIGHT_ORDER_TEST
// 仅主机顺序回归使用：建立最小可运行状态，不启动 IMU/PWM 采样。
void FlightLoop_OrderTestReset(void)
{
	RcInput_Init(&s_fl.rc);
	AttitudeControl_Init(&s_fl.attc);
	RateControl_Init(&s_fl.ratec);
	FlightSafety_Init(&s_fl.safety);
	FlightSafety_InitPwm(&s_fl.safety);
	s_fl.att.ready = 1;
	s_fl.rc.link = 1;
	s_fl.rc.cmd_ok = 1;
	s_fl.inited = 1;
}

void FlightLoop_OrderTestSetRc(unsigned char link, unsigned char cmd_ok)
{
	s_fl.rc.link = link;
	s_fl.rc.cmd_ok = cmd_ok;
}
#endif
