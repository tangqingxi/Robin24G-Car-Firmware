#include "FlightLoop.h"
#include "FlightConfig.h"
#include "RcInput.h"
#include "AttitudeControl.h"
#include "RateControl.h"
#include "FlightSafety.h"
#include "FlightTelemetry.h"
#include "MotorMixer.h"
#include "UserEuler.h"
#include "my_t3488.h"

// 与台架固件一致的校准常数；L400 迁移时保持数值不变。
static const s32 s_fc_acc_off_lsb[3] = { -3, -8, -29 };
static const s32 s_fc_acc_scale_q14[3] = { 16436, 16429, 16357 };
static const s32 s_fc_gyro_bias_q6[3] = { -93, -232, -381 };

typedef struct {
	RcInput rc;
	AttitudeControl attc;
	RateControl ratec;
	FlightSafety safety;
	FlightTelemetry tele;

	euler_q_t att;
	s32 lpf[3];
	s32 gyro_rem[3];
	s32 rate[3];
	s32 prev_acc[3];
	u32 still_sum[3];
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

static void flight_state_reset(u32 now)
{
	int i;

	RcInput_Init(&s_fl.rc);
	AttitudeControl_Init(&s_fl.attc);
	RateControl_Init(&s_fl.ratec);
	FlightSafety_Init(&s_fl.safety);
	FlightTelemetry_Init(&s_fl.tele, now);

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

	flight_state_reset(now);
#if (!C_L400_KEY_CAPTURE)
	FlightSafety_InitPwm(&s_fl.safety);
#endif

	now = clock_time();
	s_fl.next_tick = now + FLIGHT_PERIOD_TICKS;
	s_fl.tele.tx_tick = now + FLIGHT_REPORT_TICKS;
	s_fl.tele.stat_tick = now + CLOCK_SYS_CLOCK_HZ;
	s_fl.tele.prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	s_fl.inited = 1;
}

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

			avg[0] = (s32)(s_fl.still_sum[0] / s_fl.still_n);
			avg[1] = (s32)(s_fl.still_sum[1] / s_fl.still_n);
			avg[2] = (s32)(s_fl.still_sum[2] / s_fl.still_n);
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

static void flight_build_snapshot(FlightTelemetrySnapshot *s, u32 now)
{
	RcInputFrame rcf;

	RcInput_Get(&s_fl.rc, &rcf, now);
	s->now = now;
	s->att_ready = s_fl.att.ready;
	s->link = rcf.link;
	s->cmd_ok = rcf.cmd_ok;
	s->ctrl_ok = 0;
	s->q[0] = s_fl.att.q[0];
	s->q[1] = s_fl.att.q[1];
	s->q[2] = s_fl.att.q[2];
	s->q[3] = s_fl.att.q[3];
	s->rate[0] = s_fl.rate[0];
	s->rate[1] = s_fl.rate[1];
	s->rate[2] = s_fl.rate[2];
	s->corr[0] = s_fl.ratec.corr[0];
	s->corr[1] = s_fl.ratec.corr[1];
	s->corr[2] = s_fl.ratec.corr[2];
	s->tgt_att[0] = s_fl.attc.tgt_att[0];
	s->tgt_att[1] = s_fl.attc.tgt_att[1];
	s->tgt_rate[0] = s_fl.attc.tgt_rate[0];
	s->tgt_rate[1] = s_fl.attc.tgt_rate[1];
	s->tgt_rate[2] = s_fl.attc.tgt_rate[2];
	s->armed = s_fl.safety.armed;
	s->reason = s_fl.safety.reason;
	FlightSafety_GetPwm(&s_fl.safety, s->pwm);
	s->link_age_ms = (u16)rcf.link_age_ms;
	s->pkt_age_ms = (u16)rcf.pkt_age_ms;
	s->gap_ms = (u16)rcf.gap_ms;
	s->bad = (u16)rcf.bad;
}

// 收包路径：先更新 RC 状态，再消费 0001/0002 事件；急停在同包立即清 PWM。
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
	if (ev.packet_valid) {
		// A/B 按键沿只在有效摇杆包上识别，保持原台架入口行为
		FlightTelemetry_OnKey(&s_fl.tele, ev.key, now);
		if (s_fl.tele.diag_enabled) {
			FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_DIAG);
		}
		AttitudeControl_OnStick(&s_fl.attc, ev.cmd);
	}

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
									 s_fl.rc.link, s_fl.tele.diag_enabled);
		if (stop) {
			FlightSafety_Lock(&s_fl.safety, stop);
		}
	}
#endif
}

void FlightLoop_Process(void)
{
	short raw_a[3], raw_g[3];
	s32 acc_frd[3], acc_mg[3], gyr_q10[3];
	u32 now, planned, t_sample0, t_exec1, t_prep1 = 0, t_eul1 = 0;
	u32 end, total, exec_us, ctrl_total;
	u32 dt_ticks = 0;
	s32 dt_100us = 0;
	u8 att_updated = 0;
	u8 prep_timed = 0;
	u8 eul_timed = 0;
	u8 ctrl_ok = 0;
	FlightTelemetrySnapshot snap;
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
	{
		u32 late = now - planned;

		if (late >= FLIGHT_PERIOD_TICKS) {
			FlightTelemetry_OnSkip(&s_fl.tele, late / FLIGHT_PERIOD_TICKS);
			s_fl.next_tick = now + FLIGHT_PERIOD_TICKS;
		}
	}

	RcInput_UpdateTimeouts(&s_fl.rc, now);
	if (!s_fl.rc.link) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
	} else if (!s_fl.rc.cmd_ok) {
		AttitudeControl_Inhibit(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
	}

	// A/B 状态机在采样前推进，保证 A/B 段的控制使能当拍生效
	flight_build_snapshot(&snap, now);
	FlightTelemetry_DiagStep(&s_fl.tele, &snap);

	t_sample0 = clock_time();
	FlightTelemetry_OnLate(&s_fl.tele,
						   (t_sample0 - planned) / FLIGHT_TICKS_PER_US);
	FlightTelemetry_OnTick(&s_fl.tele);

	if (t3488_read_6axis(raw_a, raw_g)) {
		FlightTelemetry_OnFail(&s_fl.tele);
		t_prep1 = clock_time();
		prep_timed = 1;
		AttitudeControl_ClearOutputs(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
#if (!C_L400_KEY_CAPTURE)
		FlightSafety_Lock(&s_fl.safety, FLIGHT_LOCK_ATT);
#endif
	} else {
		u32 t_read1;

		FlightTelemetry_OnSample(&s_fl.tele);
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
			t_prep1 = clock_time();
			prep_timed = 1;
		} else if (dt_100us > 0) {
			t_prep1 = clock_time();
			prep_timed = 1;
			if (Euler_Update(&s_fl.att, s_fl.lpf, gyr_q10, dt_100us,
							 t_read1 / FLIGHT_TICKS_PER_MS)) {
				FlightTelemetry_OnEuler(&s_fl.tele);
				FlightSafety_NoteAttitude(&s_fl.safety);
				att_updated = 1;
			}
			t_eul1 = clock_time();
			eul_timed = 1;
		} else {
			t_prep1 = clock_time();
			prep_timed = 1;
		}
	}

	if (prep_timed) {
		FlightTelemetry_OnPrep(&s_fl.tele,
							   (t_prep1 - t_sample0) / FLIGHT_TICKS_PER_US);
	}
	if (eul_timed) {
		FlightTelemetry_OnEul(&s_fl.tele,
							  (t_eul1 - t_prep1) / FLIGHT_TICKS_PER_US);
	}

	if (att_updated && s_fl.rc.link && s_fl.rc.cmd_ok) {
		if (FlightTelemetry_CtrlEnable(&s_fl.tele)) {
			AttitudeControl_Update(&s_fl.attc, &s_fl.att);
			RateControl_Update(&s_fl.ratec, s_fl.attc.tgt_rate, s_fl.rate);
			ctrl_ok = 1;
		} else {
			AttitudeControl_ClearOutputs(&s_fl.attc);
			RateControl_ClearOutputs(&s_fl.ratec);
		}
	} else {
		AttitudeControl_ClearOutputs(&s_fl.attc);
		RateControl_ClearOutputs(&s_fl.ratec);
	}

#if (!C_L400_KEY_CAPTURE)
	FlightSafety_UpdateFaults(&s_fl.safety, s_fl.rc.link, s_fl.rc.cmd_ok,
							  att_updated, ctrl_ok,
							  s_fl.tele.diag_enabled);
	{
		MotorMixerOutput mix;

		MotorMixer_Mix(MotorMixer_Thrust(s_fl.rc.cmd[0]),
					   s_fl.ratec.corr[0], s_fl.ratec.corr[1],
					   s_fl.ratec.corr[2], &mix);
		FlightSafety_Apply(&s_fl.safety, &mix);
	}
#endif

	t_exec1 = clock_time();
	exec_us = (t_exec1 - t_sample0) / FLIGHT_TICKS_PER_US;
	FlightTelemetry_OnExec(&s_fl.tele, exec_us);
	ctrl_total = t_exec1 - planned;
	FlightTelemetry_OnCtrl(&s_fl.tele, ctrl_total);

	snap.now = t_exec1;
	snap.att_ready = s_fl.att.ready;
	snap.link = s_fl.rc.link;
	snap.cmd_ok = s_fl.rc.cmd_ok;
	snap.ctrl_ok = ctrl_ok;
	snap.q[0] = s_fl.att.q[0];
	snap.q[1] = s_fl.att.q[1];
	snap.q[2] = s_fl.att.q[2];
	snap.q[3] = s_fl.att.q[3];
	for (i = 0; i < 3; i++) {
		snap.rate[i] = s_fl.rate[i];
		snap.corr[i] = s_fl.ratec.corr[i];
		snap.tgt_rate[i] = s_fl.attc.tgt_rate[i];
	}
	snap.tgt_att[0] = s_fl.attc.tgt_att[0];
	snap.tgt_att[1] = s_fl.attc.tgt_att[1];
	snap.armed = s_fl.safety.armed;
	snap.reason = s_fl.safety.reason;
	FlightSafety_GetPwm(&s_fl.safety, snap.pwm);
	{
		RcInputFrame rcf;

		RcInput_Get(&s_fl.rc, &rcf, t_exec1);
		snap.link_age_ms = (u16)rcf.link_age_ms;
		snap.pkt_age_ms = (u16)rcf.pkt_age_ms;
		snap.gap_ms = (u16)rcf.gap_ms;
		snap.bad = (u16)rcf.bad;
	}
	FlightTelemetry_Update(&s_fl.tele, &snap);
	if (FlightTelemetry_WindowReset(&s_fl.tele)) {
		RcInput_ClearGap(&s_fl.rc);
	}

	end = clock_time();
	total = end - planned;
	if (total > FLIGHT_PERIOD_TICKS) {
		FlightTelemetry_OnMiss(&s_fl.tele);
	}
}

void FlightLoop_RecordRx(unsigned int ticks)
{
	if (!s_fl.inited) return;
	FlightTelemetry_OnRx(&s_fl.tele, ticks / FLIGHT_TICKS_PER_US);
}

void FlightLoop_RecordRfLoop(unsigned int ticks)
{
	if (!s_fl.inited) return;
	FlightTelemetry_RecordRfLoop(&s_fl.tele, ticks);
}

unsigned char FlightLoop_DiagActive(void)
{
	return s_fl.inited ? FlightTelemetry_DiagActive(&s_fl.tele) : 0;
}
