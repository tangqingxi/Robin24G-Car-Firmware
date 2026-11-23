#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <types.h>

// 日志区：0x10000..0x1CFFF，共 13 个 4KiB 扇区。
#define FLIGHT_LOG_BASE        0x10000u
#define FLIGHT_LOG_END         0x1D000u
#define FLIGHT_LOG_SECTORS     13u
#define FLIGHT_LOG_SECTOR_SIZE 0x1000u

#define FLIGHT_LOG_VERSION     2u
#define FLIGHT_LOG_PAGE_HDR    256u
#define FLIGHT_LOG_REC_PER_PHYS 4u
#define FLIGHT_LOG_PHYS_PER_SEC 15u
#define FLIGHT_LOG_REC_PER_SEC 60u
#define FLIGHT_LOG_REC_SIZE    56u
#define FLIGHT_LOG_MAX_RECORDS 750u          // 25Hz 约 30 秒

// flags 位定义
#define FLIGHT_LOG_F_IMU_OK    0x0001
#define FLIGHT_LOG_F_ATT_READY 0x0002
#define FLIGHT_LOG_F_ATT_UPD   0x0004
#define FLIGHT_LOG_F_ARMED     0x0008
#define FLIGHT_LOG_F_LIMITED   0x0010
#define FLIGHT_LOG_F_ESTOP     0x0020
#define FLIGHT_LOG_F_LINKLOSS  0x0040
#define FLIGHT_LOG_F_LOCKED    0x0080

typedef struct {
	unsigned int t_ms;
	s32 q[4];
	s16 gyro[3];
	s16 acc[3];
	s16 yaw_stick;
	s16 yaw_target;
	s16 yaw_corr;
	unsigned short thrust;
	unsigned short pwm[4];
	unsigned short flags;
} FlightLogSample;

void FlightLog_Init(void);
#define FLIGHT_LOG_KEY_START 0x0004
#define FLIGHT_LOG_IDLE 0
#define FLIGHT_LOG_PREP 1
#define FLIGHT_LOG_REC 2
#define FLIGHT_LOG_ST_END 3
#define FLIGHT_LOG_REJECT 4
#define FLIGHT_LOG_ST_CLOSING 5

void FlightLog_OnKey(unsigned short key, unsigned char packet_valid,
					 unsigned char can_start, unsigned char blocked);
void FlightLog_BeginPrepare(void);
unsigned char FlightLog_State(void);
void FlightLog_OnSample(const FlightLogSample *sample);
void FlightLog_Service(unsigned char budget_ok);
void FlightLog_OnEvent(unsigned char event, const FlightLogSample *sample);
void FlightLog_OnControlTick(void);
void FlightLog_OnLinkDown(void);
#define FLIGHT_LOG_TYPE_SHIFT 12u
#define FLIGHT_LOG_TYPE_EVENT 1u
#define FLIGHT_LOG_TYPE_SUMMARY 2u
#define FLIGHT_LOG_TYPE_BASELINE 3u
#define FLIGHT_LOG_TYPE_TAIL 4u
void FlightLog_Stop(void);
#if C_FLIGHT_ARM_DEBUG
void FlightLog_BenchRun(unsigned int next_tick);
#endif
unsigned char FlightLog_Active(void);
unsigned int FlightLog_Dropped(void);
#if C_FLIGHT_ARM_DEBUG
unsigned int FlightLog_QueueCount(void);
unsigned int FlightLog_Written(void);
unsigned int FlightLog_PhysicalWrites(void);
unsigned int FlightLog_QueueDrops(void);
unsigned int FlightLog_FlashPeakUs(void);
unsigned int FlightLog_PackPeakUs(void);
unsigned int FlightLog_RecCrcPeakUs(void);
unsigned int FlightLog_PageCrcPeakUs(void);
#endif

#endif /* FLIGHT_LOG_H */
