#include "FlightSafety.h"
#include "UserPwm.h"

// 四路实际输出：前=LF/PWM1，后=LB/PWM3，左=RB/PWM0，右=RF/PWM2。
static void safety_write(unsigned short front, unsigned short rear,
						 unsigned short left, unsigned short right)
{
	LFmotDuty = front;
	LBmotDuty = rear;
	RBmotDuty = left;
	RFmotDuty = right;
	Update4MotPwmDuty();
}

void FlightSafety_Init(FlightSafety *s)
{
	s->armed = 0;
	s->reason = FLIGHT_LOCK_BOOT;
	s->key_prev = 0;
	s->att_ever_ready = 0;
	s->pwm_ready = 0;
}

// 正常飞控启动 PWM 并保持四路 0；抓包模式不调用本函数。
void FlightSafety_InitPwm(FlightSafety *s)
{
	if (!s->pwm_ready) {
		User_pwm_init_func();
		s->pwm_ready = 1;
	}
	FlightSafety_Lock(s, FLIGHT_LOCK_BOOT);
}

void FlightSafety_Lock(FlightSafety *s, unsigned char reason)
{
	s->armed = 0;
	if (reason) {
		s->reason = reason;
	}
	if (s->pwm_ready) {
		safety_write(0, 0, 0, 0);
	}
}

void FlightSafety_NoteAttitude(FlightSafety *s)
{
	s->att_ever_ready = 1;
}

/*
 * 每个 RF 包调用一次。0001 收到新事件立即要求锁定；0002 只在
 * 该包有效、油门低位、三轴回中、姿态就绪、链路有效且未诊断时解锁。
 * 条件不满足的 0002 也消费事件沿，后续重复包不能补解锁。
 * 返回非 0 表示调用方必须立即执行 FlightSafety_Lock()。
 */
unsigned char FlightSafety_OnKey(FlightSafety *s, unsigned short key,
								 unsigned char packet_valid,
								 unsigned char thr_low,
								 unsigned char sticks_center,
								 unsigned char att_ready,
								 unsigned char link_ok,
								 unsigned char diag_enabled)
{
	unsigned char new_estop = (key & FLIGHT_EVENT_ESTOP)
							  && !(s->key_prev & FLIGHT_EVENT_ESTOP);
	unsigned char new_unlock = (key & FLIGHT_EVENT_UNLOCK)
							   && !(s->key_prev & FLIGHT_EVENT_UNLOCK);

	s->key_prev = key;

	// 急停优先：同包同时出现解锁位也不处理解锁
	if (new_estop) {
		s->armed = 0;
		s->reason = FLIGHT_LOCK_MANUAL;
		return FLIGHT_LOCK_MANUAL;
	}

	if (new_unlock) {
		if (packet_valid && !(key & FLIGHT_EVENT_ESTOP)
			&& thr_low && sticks_center && att_ready && link_ok
			&& !diag_enabled) {
			s->armed = 1;
			s->reason = FLIGHT_LOCK_NONE;
		}
		// 条件不满足：消费事件，不延迟解锁
	}
	return 0;
}

// 失联/指令超时/姿态未更新/控制失败/诊断段统一锁定。
void FlightSafety_UpdateFaults(FlightSafety *s, unsigned char link_ok,
							   unsigned char cmd_ok, unsigned char att_updated,
							   unsigned char ctrl_ok, unsigned char diag_enabled)
{
	if (!link_ok) {
		FlightSafety_Lock(s, FLIGHT_LOCK_LINK);
	} else if (!cmd_ok) {
		FlightSafety_Lock(s, FLIGHT_LOCK_CMD);
	} else if (!att_updated) {
		FlightSafety_Lock(s, s->att_ever_ready
						  ? FLIGHT_LOCK_ATT : FLIGHT_LOCK_BOOT);
	} else if (!ctrl_ok) {
		FlightSafety_Lock(s, FLIGHT_LOCK_ATT);
	} else if (diag_enabled) {
		FlightSafety_Lock(s, FLIGHT_LOCK_DIAG);
	}
}

void FlightSafety_Apply(FlightSafety *s, const MotorMixerOutput *out)
{
	if (!s->pwm_ready) {
		return;
	}
	if (!s->armed) {
		safety_write(0, 0, 0, 0);
		return;
	}
	safety_write(out->front, out->rear, out->left, out->right);
}

void FlightSafety_GetPwm(const FlightSafety *s, unsigned short out[4])
{
	(void)s;
	out[0] = LFmotDuty;
	out[1] = LBmotDuty;
	out[2] = RBmotDuty;
	out[3] = RFmotDuty;
}
