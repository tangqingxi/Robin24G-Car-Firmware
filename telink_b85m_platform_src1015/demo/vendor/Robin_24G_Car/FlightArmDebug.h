#ifndef FLIGHT_ARM_DEBUG_H
#define FLIGHT_ARM_DEBUG_H

#include <types.h>

// 专用解锁诊断默认关闭；正式镜像编译后不产生任何调试代码/UART 文本。
#ifndef C_FLIGHT_ARM_DEBUG
#define C_FLIGHT_ARM_DEBUG 0
#endif

#if C_FLIGHT_ARM_DEBUG
void FlightArmDebug_Init(void);
void FlightArmDebug_OnInit(unsigned char st, unsigned int still_ms,
						   unsigned char att_ready, unsigned char pwm_ready);
void FlightArmDebug_OnPacket(unsigned short thr, unsigned short rot,
							 unsigned short fb, unsigned short lr,
							 unsigned short key);
void FlightArmDebug_OnUnlockAttempt(unsigned short key,
									unsigned int raw_thr,
									const int cmd[4],
									unsigned char packet_valid,
									unsigned char thr_low,
									unsigned char center,
									unsigned char att_ready,
									unsigned char link_ok);
void FlightArmDebug_OnEstop(void);
void FlightArmDebug_OnStatus(unsigned int still_ms, unsigned char att_ready,
							 unsigned char pwm_ready);
void FlightArmDebug_OnControl(unsigned char att_updated, unsigned char ctrl_ok,
							  unsigned char armed, unsigned char reason,
							  const unsigned short pwm[4]);
void FlightArmDebug_Process(void);
#else
#define FlightArmDebug_Init() ((void)0)
#define FlightArmDebug_OnInit(st, still, att, pwm) ((void)0)
#define FlightArmDebug_OnPacket(thr, rot, fb, lr, key) ((void)0)
#define FlightArmDebug_OnUnlockAttempt(key, rt, cmd, pv, low, ctr, att, lk) ((void)0)
#define FlightArmDebug_OnEstop() ((void)0)
#define FlightArmDebug_OnStatus(still, att, pwm) ((void)0)
#define FlightArmDebug_OnControl(up, ctrl, arm, reason, pwm) ((void)0)
#define FlightArmDebug_Process() ((void)0)
#endif

#endif /* FLIGHT_ARM_DEBUG_H */
