#ifndef BARO_FIXED_BDT_H
#define BARO_FIXED_BDT_H

#include <types.h>
#include "BaroFixed.h"

// 定点温补地面诊断：100 槽 × 1s，记录原始气压、实际参与温补的温度、板端 0.1Pa 结果。
#define BARO_FIXED_SLOTS        90
#define BARO_FIXED_SAMPLE_MS    1000
#define BARO_FIXED_START_MS     5000
#define BARO_FIXED_DURATION_MS  (BARO_FIXED_SLOTS * BARO_FIXED_SAMPLE_MS)
#define BARO_FIXED_RECORD_VER   5
#define BARO_FIXED_SECTOR       0x10000ul
#define BARO_FIXED_TEMP_TIMEOUT_MS 2500

#define BARO_FIXED_ST_WAIT      0
#define BARO_FIXED_ST_SAMPLING  1
#define BARO_FIXED_ST_DONE      2
#define BARO_FIXED_ST_CANCELED  3
#define BARO_FIXED_ST_ERROR     4

void BaroFixedBdt_Process(void);
unsigned char BaroFixedBdt_GetState(void);
unsigned short BaroFixedBdt_GetPressureCount(void);
unsigned short BaroFixedBdt_GetTempCount(void);
unsigned short BaroFixedBdt_GetFixedCount(void);

#ifdef BARO_BDT_HOST_TEST
void BaroFixedBdt_TestReset(void);
u32 BaroFixedBdt_TestClock(void);
void BaroFixedBdt_TestCal(BaroFixedCal_t *cal);
void BaroFixedBdt_TestFlashErase(unsigned long addr);
void BaroFixedBdt_TestFlashWrite(unsigned long addr, unsigned long len, u8 *data);
void BaroFixedBdt_TestFlashRead(unsigned long addr, unsigned long len, u8 *data);
#endif

#endif /* BARO_FIXED_BDT_H */
