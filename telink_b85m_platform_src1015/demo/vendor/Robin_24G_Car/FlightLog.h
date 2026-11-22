#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <types.h>

// 日志区：0x10000..0x1CFFF，共 13 个 4KiB 扇区。
#define FLIGHT_LOG_BASE        0x10000u
#define FLIGHT_LOG_END         0x1D000u
#define FLIGHT_LOG_SECTORS     13u
#define FLIGHT_LOG_SECTOR_SIZE 0x1000u
#define FLIGHT_LOG_CMD_ADDR    0x1D000u
#define FLIGHT_LOG_CMD_MAGIC   0x31474F4Cu   // "LOG1"
#define FLIGHT_LOG_VERSION     1u
#define FLIGHT_LOG_PAGE_HDR    32u
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
void FlightLog_PollCommand(unsigned char locked);
void FlightLog_OnSample(const FlightLogSample *sample);
void FlightLog_OnEvent(unsigned char event, const FlightLogSample *sample);
void FlightLog_Stop(void);
unsigned char FlightLog_Active(void);
unsigned int FlightLog_Dropped(void);

#endif /* FLIGHT_LOG_H */
