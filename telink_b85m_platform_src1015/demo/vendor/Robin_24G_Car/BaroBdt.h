#ifndef BARO_BDT_H
#define BARO_BDT_H

#include <types.h>

// 地面气压 BDT 诊断：静置 5 秒后以 10Hz 采 30 秒，只写 RAM；
// 结束后仅在锁定且四路 PWM 全零时写 0x10000 的单个 4KB 扇区。
#define BARO_BDT_MAX_SAMPLES   300
#define BARO_BDT_START_MS      5000
#ifndef BARO_BDT_SAMPLE_MS
#define BARO_BDT_SAMPLE_MS     100
#endif
#define BARO_BDT_DURATION_MS   (BARO_BDT_MAX_SAMPLES * BARO_BDT_SAMPLE_MS)
#define BARO_BDT_RECORD_VERSION 2
#define BARO_BDT_SECTOR        0x10000ul

#define BARO_BDT_ST_WAIT       0
#define BARO_BDT_ST_SAMPLING   1
#define BARO_BDT_ST_DONE       2
#define BARO_BDT_ST_CANCELED   3
#define BARO_BDT_ST_ERROR      4

void BaroBdt_Process(void);
unsigned char BaroBdt_GetState(void);
unsigned short BaroBdt_GetCount(void);

#ifdef BARO_BDT_HOST_TEST
void BaroBdt_TestReset(void);
u32 BaroBdt_TestClock(void);
#endif

#endif /* BARO_BDT_H */
