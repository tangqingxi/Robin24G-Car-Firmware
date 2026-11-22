#ifndef FLIGHT_ARM_DEBUG_H
#define FLIGHT_ARM_DEBUG_H

#include <types.h>

// 专用解锁诊断默认关闭；正式镜像编译后不产生任何调试代码/UART 文本。
#ifndef C_FLIGHT_ARM_DEBUG
#define C_FLIGHT_ARM_DEBUG 0
#endif

#if C_FLIGHT_ARM_DEBUG
void FlightArmDebug_Init(void);
void FlightArmDebug_OnInit(unsigned char st, unsigned char lpf_rb,
						   unsigned int still_ms, unsigned char att_ready,
						   unsigned char pwm_ready);
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
									unsigned char link_ok,
									const int acc[3], const int gyro[3]);
void FlightArmDebug_OnEstop(void);
void FlightArmDebug_OnRead(unsigned char ok);
void FlightArmDebug_OnStillSample(const int acc[3], const int gyro[3],
								  const int prev[3], unsigned char have_prev,
								  unsigned int dt_ticks, unsigned char still);
void FlightArmDebug_OnGapBreak(void);
void FlightArmDebug_OnStartupResult(unsigned char ok);
void FlightArmDebug_OnStatus(unsigned int still_ms, unsigned char att_ready,
							 unsigned char pwm_ready);
void FlightArmDebug_OnControl(unsigned char att_updated, unsigned char ctrl_ok,
							  unsigned char armed, unsigned char reason,
							  const unsigned short pwm[4]);
void FlightArmDebug_Process(void);
#else
#define FlightArmDebug_Init() ((void)0)
#define FlightArmDebug_OnInit(st, rb, still, att, pwm) ((void)0)
#define FlightArmDebug_OnPacket(thr, rot, fb, lr, key) ((void)0)
#define FlightArmDebug_OnUnlockAttempt(key, rt, cmd, pv, low, ctr, att, lk, a, g) ((void)0)
#define FlightArmDebug_OnEstop() ((void)0)
#define FlightArmDebug_OnRead(ok) ((void)0)
#define FlightArmDebug_OnStillSample(a, g, p, h, d, s) ((void)0)
#define FlightArmDebug_OnGapBreak() ((void)0)
#define FlightArmDebug_OnStartupResult(ok) ((void)0)
#define FlightArmDebug_OnStatus(still, att, pwm) ((void)0)
#define FlightArmDebug_OnControl(up, ctrl, arm, reason, pwm) ((void)0)
#define FlightArmDebug_Process() ((void)0)
#endif

#endif /* FLIGHT_ARM_DEBUG_H */
