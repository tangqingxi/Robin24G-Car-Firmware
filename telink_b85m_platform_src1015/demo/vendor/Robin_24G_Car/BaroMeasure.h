#ifndef BARO_MEASURE_H
#define BARO_MEASURE_H

#include <types.h>

typedef struct __attribute__((aligned(4))) {
	s32 pressure_01pa;
	u32 tick;
	u32 sequence;
} BaroMeasureSample;

void BaroMeasure_Init(u32 now);
u8 BaroMeasure_Service(u32 now, u32 next_control_tick);
u8 BaroMeasure_GetSample(u32 now, BaroMeasureSample *out);

#endif
