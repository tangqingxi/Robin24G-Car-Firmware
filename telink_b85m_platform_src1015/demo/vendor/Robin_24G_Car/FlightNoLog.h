#ifndef FLIGHT_NO_LOG_H
#define FLIGHT_NO_LOG_H

/* 地面诊断/正常飞控构建已移除板载日志：保留最小类型和空接口，
 * 使主循环和飞控逻辑不再链接 FlightLog/FlightArmDebug。 */

#include <types.h>

#define FLIGHT_LOG_KEY_START 0x0004
#define FLIGHT_LOG_IDLE 0
#define FLIGHT_LOG_PREP 1
#define FLIGHT_LOG_REC 2
#define FLIGHT_LOG_ST_END 3
#define FLIGHT_LOG_REJECT 4
#define FLIGHT_LOG_ST_CLOSING 5

#define FLIGHT_LOG_F_IMU_OK 0x0001
#define FLIGHT_LOG_F_ATT_READY 0x0002
#define FLIGHT_LOG_F_ATT_UPD 0x0004
#define FLIGHT_LOG_F_ARMED 0x0008
#define FLIGHT_LOG_F_LOCKED 0x0080
#define FLIGHT_LOG_F_ESTOP 0x0020
#define FLIGHT_LOG_F_LINKLOSS 0x0040
#define FLIGHT_LOG_F_LIMITED 0x0010

typedef struct {
	u32 t_ms;
	s16 q[4];
	s16 gyro[3];
	s16 acc[3];
	s16 yaw_stick;
	s16 yaw_target;
	s16 yaw_corr;
	u16 thrust;
	u16 pwm[4];
	u16 flags;
} FlightLogSample;

#define FlightLog_OnKey(...) ((void)0)
#define FlightLog_BeginPrepare() ((void)0)
#define FlightLog_OnControlTick() ((void)0)
#define FlightLog_OnSample(...) ((void)0)
#define FlightLog_OnEvent(...) ((void)0)
#define FlightLog_OnLinkDown() ((void)0)
#define FlightLog_Service(...) ((void)0)
#define FlightLog_BenchStart() ((void)0)
#define FlightLog_BenchStep(...) ((void)0)
#define FlightLog_State() (FLIGHT_LOG_IDLE)
#define FlightLog_Active() 0
#define FlightLog_Dropped() 0
#define FlightLog_QueueCount() 0
#define FlightLog_Written() 0
#define FlightLog_PhysicalWrites() 0

#define FlightArmDebug_Init() ((void)0)
#define FlightArmDebug_OnInit(...) ((void)0)
#define FlightArmDebug_Process() ((void)0)
#define FlightArmDebug_OnPacket(...) ((void)0)
#define FlightArmDebug_OnUnlockAttempt(...) ((void)0)
#define FlightArmDebug_OnStatus(...) ((void)0)
#define FlightArmDebug_OnControl(...) ((void)0)
#define FlightArmDebug_OnEstop() ((void)0)
#define FlightArmDebug_OnRead(...) ((void)0)
#define FlightArmDebug_OnGapBreak() ((void)0)
#define FlightArmDebug_OnStillSample(...) ((void)0)
#define FlightArmDebug_OnStartupResult(...) ((void)0)
#define FlightArmDebug_OnControlTiming(...) ((void)0)
#define FlightArmDebug_OnBudgetTiming(...) ((void)0)
#define FlightArmDebug_OnLogServiceUs(...) ((void)0)
#define FlightArmDebug_OnLogPrepareUs(...) ((void)0)
#define FlightArmDebug_OnLogCloseUs(...) ((void)0)
#define FlightArmDebug_OnLogOverrun() ((void)0)
#define FlightArmDebug_OnLogWriteResult(...) ((void)0)
#define FlightArmDebug_OnLogGate(...) ((void)0)
#define FlightArmDebug_OnLogState(...) ((void)0)

#endif
