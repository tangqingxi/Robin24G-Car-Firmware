#ifndef BARO_TEMP_BDT_H
#define BARO_TEMP_BDT_H

#include <types.h>

// 气压+芯片温度 BDT 诊断：上电启动，5 秒后每秒一组，150 槽约 150 秒。
#define BARO_TEMP_SLOTS        150
#define BARO_TEMP_SAMPLE_MS    1000
#define BARO_TEMP_START_MS     5000
#define BARO_TEMP_DURATION_MS  (BARO_TEMP_SLOTS * BARO_TEMP_SAMPLE_MS)
#define BARO_TEMP_RECORD_VER   3
#define BARO_TEMP_SECTOR       0x10000ul

#define BARO_TEMP_ST_WAIT      0
#define BARO_TEMP_ST_SAMPLING  1
#define BARO_TEMP_ST_DONE      2
#define BARO_TEMP_ST_CANCELED  3
#define BARO_TEMP_ST_ERROR     4

void BaroTempBdt_Process(void);
unsigned char BaroTempBdt_GetState(void);
unsigned short BaroTempBdt_GetPressureCount(void);
unsigned short BaroTempBdt_GetTempCount(void);

#ifdef BARO_BDT_HOST_TEST
void BaroTempBdt_TestReset(void);
u32 BaroTempBdt_TestClock(void);
void BaroTempBdt_TestFlashErase(unsigned long addr);
void BaroTempBdt_TestFlashWrite(unsigned long addr, unsigned long len, u8 *data);
void BaroTempBdt_TestFlashRead(unsigned long addr, unsigned long len, u8 *data);
#endif

#endif /* BARO_TEMP_BDT_H */
