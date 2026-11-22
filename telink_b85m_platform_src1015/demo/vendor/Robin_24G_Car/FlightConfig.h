#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include <types.h>
#ifndef FLIGHT_HOST_TEST
#include "app_public_config.h"
#endif
#include "UserEuler.h"

// 抓包镜像只记录原始按键，不初始化电机 PWM；正常镜像默认 0。
#ifndef C_L400_KEY_CAPTURE
#define C_L400_KEY_CAPTURE 0
#endif

// 200Hz 飞控公共常量。所有时间仍以 tick 计，毫秒超时只在边界换算。
#define FLIGHT_LOOP_HZ             200
#define FLIGHT_PERIOD_TICKS        (CLOCK_SYS_CLOCK_HZ / FLIGHT_LOOP_HZ)
#define FLIGHT_TICKS_PER_MS        (CLOCK_SYS_CLOCK_HZ / 1000)
#define FLIGHT_TICKS_PER_100US     (CLOCK_SYS_CLOCK_HZ / 10000)
#define FLIGHT_TICKS_PER_US        (CLOCK_SYS_CLOCK_HZ / 1000000)

// 遥控超时保持原毫秒数，不随控制周期缩放。
#define FLIGHT_CMD_TIMEOUT_MS      200
#define FLIGHT_CMD_TIMEOUT_TICKS   ((u32)FLIGHT_CMD_TIMEOUT_MS * FLIGHT_TICKS_PER_MS)
#define FLIGHT_LINK_TIMEOUT_MS     500
#define FLIGHT_LINK_TIMEOUT_TICKS  ((u32)FLIGHT_LINK_TIMEOUT_MS * FLIGHT_TICKS_PER_MS)

#define FLIGHT_STICK_MAX           307
#define FLIGHT_MAX_TILT_DEG100     3000
#define FLIGHT_YAW_RATE_MAX_Q10    3072

// 外环 P 与限幅；内环沿用 P=64/256，I=D=0。
#define FLIGHT_ATT_KP_Q8           1500
#define FLIGHT_RATE_LIMIT_Q10      4096
#define FLIGHT_KP_Q8               64
#define FLIGHT_KI_Q16              0
#define FLIGHT_KD_Q8               0
#define FLIGHT_RATE_INT_MAX        1000000
#define FLIGHT_CORR_LIMIT_Q10      4096

#define FLIGHT_INIT_STILL_TICKS    ((u32)C_ATT_INIT_STILL_US * FLIGHT_TICKS_PER_US)

#endif /* FLIGHT_CONFIG_H */
