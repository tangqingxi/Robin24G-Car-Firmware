#include "FlightLoop.h"
#include "FlightConfig.h"
#include "RcInput.h"
#include "AttitudeControl.h"
#include "RateControl.h"
#include "FlightSafety.h"
#include "MotorMixer.h"
#include "UserEuler.h"
#include "my_t3488.h"

// 与台架固件一致的校准常数；迁移控制链时保持数值不变。
static const s32 s_fc_acc_off_lsb[3] = { -3, -8, -29 };
static const s32 s_fc_acc_scale_q14[3] = { 16436, 16429, 16357 };
static const s32 s_fc_gyro_bias_q6[3] = { -93, -232, -381 };

typedef struct {
	RcInput rc;
	AttitudeControl attc;
	RateControl ratec;
	FlightSafety safety;

	euler_q_t att;
	s32 lpf[3];
	s32 gyro_rem[3];
	s32 rate[3];
	s32 prev_acc[3];
	s32 still_sum[3];
	u32 still_ticks;
	u32 still_n;
	u32 last_ts;
	u32 dt_rem;
	u32 next_tick;
	u32 retry_tick;
	u32 lpf_alpha;
	u32 lpf_alpha_dt_us;
	u8 lpf_ok;
	u8 have_ts;
	u8 still_prev_ok;
	u8 inited;
} FlightLoopState;

static FlightLoopState s_fl;

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

	if (s_fl.inited) {
		return;
	}
	if ((s32)(now - s_fl.retry_tick) < 0) {
		return;
	}
	s_fl.retry_tick = now + CLOCK_SYS_CLOCK_HZ;

	if (t3488_bench_init(C_GYRO_LPF_SEL, &rb)) {
		return;
	}

	flight_state_reset();
#if (!C_L400_KEY_CAPTURE)
	FlightSafety_InitPwm(&s_fl.safety);
#endif

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

			// 静置和必须有符号；先把样本计数转 s32 再除，避免负数被当无符号
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

	if (!s_fl.inited) {
		return;
	}
	ev = RcInput_OnPacket(&s_fl.rc, thr, rot, fb, lr, key, now);

#if (!C_L400_KEY_CAPTURE)
	{
		u8 thr_low = (ev.cmd[0] <= MOTOR_ARM_THR_MAX) ? 1 : 0;
		u8 center = (ev.cmd[1] >= -MOTOR_ARM_STICK_MAX
					 && ev.cmd[1] <= MOTOR_ARM_STICK_MAX
					 && ev.cmd[2] >= -MOTOR_ARM_STICK_MAX
					 && ev.cmd[2] <= MOTOR_ARM_STICK_MAX
					 && ev.cmd[3] >= -MOTOR_ARM_STICK_MAX
					 && ev.cmd[3] <= MOTOR_ARM_STICK_MAX) ? 1 : 0;
		u8 stop = FlightSafety_OnKey(&s_fl.safety, ev.key, ev.packet_valid,
									 thr_low, center, s_fl.att.ready,
									 s_fl.rc.link);

		if (stop) {
			// 急停优先：先写零 PWM，不再做同包目标换算
			FlightSafety_Lock(&s_fl.safety, stop);
		} else if (ev.packet_valid) {
			AttitudeControl_OnStick(&s_fl.attc, ev.cmd);
		}
	}
#endif
}

// 采样前安全门：失联/指令超时先把四路写 0，之后才允许读 IMU。
void FlightLoop_PreSampleSafety(void)
{
	if (!s_fl.rc.link) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
#if (!C_L400_KEY_CAPTURE)
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_LINK);
#endif
	} else if (!s_fl.rc.cmd_ok) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
#if (!C_L400_KEY_CAPTURE)
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_CMD);
#endif
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
		// 已错过至少一拍：从当前时刻重新排下一拍，不追发补拍
		s_fl.next_tick = now + FLIGHT_PERIOD_TICKS;
	}

	RcInput_UpdateTimeouts(&s_fl.rc, now);
	// 必须在下面的 t3488_read_6axis/Euler 之前完成四路清零
	FlightLoop_PreSampleSafety();

	if (t3488_read_6axis(raw_a, raw_g)) {
		AttitudeControl_ClearOutputs(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
#if (!C_L400_KEY_CAPTURE)
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_ATT);
#endif
	} else {
		u32 t_read1;

		t_read1 = clock_time();
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

	if (att_updated && s_fl.rc.link && s_fl.rc.cmd_ok) {
		AttitudeControl_Update(&s_fl.attc, &s_fl.att);
		RateControl_Update(&s_fl.ratec, s_fl.attc.tgt_rate, s_fl.rate);
		ctrl_ok = 1;
	} else {
		AttitudeControl_ClearOutputs(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
	}

#if (!C_L400_KEY_CAPTURE)
	FlightSafety_UpdateFaults(&s_fl.safety, s_fl.rc.link, s_fl.rc.cmd_ok,
							  att_updated, ctrl_ok);
	{
		MotorMixerOutput mix;

		MotorMixer_Mix(MotorMixer_Thrust(s_fl.rc.cmd[0]),
					   s_fl.ratec.corr[0], s_fl.ratec.corr[1],
					   s_fl.ratec.corr[2], &mix);
		FlightSafety_Apply(&s_fl.safety, &mix);
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
