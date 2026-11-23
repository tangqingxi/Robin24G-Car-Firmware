#ifndef FLIGHT_SAFETY_H
#define FLIGHT_SAFETY_H

#include "MotorMixer.h"

// 实测短事件：0002=解锁，0001=急停；每次 0→1 沿只处理一次。
#define FLIGHT_EVENT_ESTOP        0x0001
#define FLIGHT_EVENT_UNLOCK       0x0002

// 当前基线的解锁条件（FlightSafety_OnKey）：
//   收到有效包、按键为 0002 上升沿、无 0001 急停、油门低、
//   俯仰/横滚/偏航杆回中、姿态已就绪且遥控链路有效。
// 安全顺序：急停或失联先由 FlightSafety_Lock 清 armed 并立即写四路 0 PWM，
// 再进入后续控制/混控；上锁状态 Apply 也强制四路 0。

// 锁定原因（历史遥测帧的 x 字段曾使用；本版仅作内部状态）
#define FLIGHT_LOCK_NONE          0
#define FLIGHT_LOCK_BOOT          1
#define FLIGHT_LOCK_LINK          2
#define FLIGHT_LOCK_CMD           3
#define FLIGHT_LOCK_ATT           4
#define FLIGHT_LOCK_MANUAL        5

typedef struct __attribute__((aligned(4))) {
	unsigned char armed;
	unsigned char reason;
	unsigned short key_prev;
	unsigned char att_ever_ready;
	unsigned char pwm_ready;
} FlightSafety;

void FlightSafety_Init(FlightSafety *s);
void FlightSafety_InitPwm(FlightSafety *s);
void FlightSafety_Lock(FlightSafety *s, unsigned char reason);
unsigned char FlightSafety_OnKey(FlightSafety *s, unsigned short key,
								 unsigned char packet_valid,
								 unsigned char thr_low,
								 unsigned char sticks_center,
								 unsigned char att_ready,
								 unsigned char link_ok);
void FlightSafety_NoteAttitude(FlightSafety *s);
void FlightSafety_UpdateFaults(FlightSafety *s, unsigned char link_ok,
							   unsigned char cmd_ok, unsigned char att_updated,
							   unsigned char ctrl_ok);
void FlightSafety_Apply(FlightSafety *s, const MotorMixerOutput *out);
void FlightSafety_GetPwm(const FlightSafety *s, unsigned short out[4]);

#endif /* FLIGHT_SAFETY_H */
