/* 验证气压测量预算、逐拍读取、样本时效和无传感器时的飞控启动。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "BaroMeasure.h"
#include "FlightConfig.h"
#include "PressSen_Lpc2623.h"

#ifdef BARO_MEASURE_FLIGHT_REPLAY
static u8 id_failure = 1;
#else
static u8 id_failure;
#endif
static u8 configured, continuous, ready, pressure_failure;
static unsigned int transactions;

u8 LPC2623_CheckId(void) { return id_failure; }
void LPC2623_rateset(u8 sensor, u8 rate, u8 oversampling)
{
	if (sensor == PRESSURE_SENSOR && rate == 1 && oversampling == 8) {
		configured = 1;
	}
}
void LPC2623_start_continuous(u8 mode) { continuous = mode == CONTINUOUS_P_AND_T; }
u8 LPC2623_ReadCalibration(u8 block[LPC2623_CALIB_BYTES], u8 *cfg)
{
	assert(configured && continuous);
	memset(block, 0, LPC2623_CALIB_BYTES);
	// 仅设 C00=100000 Pa，便于独立核对单位与样本发布。
	block[6] = 0x18;
	block[7] = 0x6A;
	*cfg = 3;
	return 0;
}
u8 LPC2623_ReadReady(u8 *bits) { transactions++; *bits = ready; return 0; }
u8 LPC2623_ReadRaw(s32 *value) { transactions++; *value = 1390000; return pressure_failure; }
u8 LPC2623_ReadTempRaw(s32 *value) { transactions++; *value = 93000; return 0; }

#ifndef BARO_MEASURE_FLIGHT_REPLAY
int main(void)
{
	const u32 period = FLIGHT_PERIOD_TICKS;
	const u32 start = 0xFF000000u;
	u32 now = start + 5u * CLOCK_SYS_CLOCK_HZ;
	BaroMeasureSample sample;

	BaroMeasure_Init(start);
	ready = LPC2623_PRESSURE_READY | LPC2623_TEMP_READY;
	assert(!BaroMeasure_Service(now - 1, now + period));
	assert(!BaroMeasure_Service(now, now + 999u * FLIGHT_TICKS_PER_US));
	assert(transactions == 0);
	assert(BaroMeasure_Service(now, now + period) && transactions == 1);
	now += period;
	assert(BaroMeasure_Service(now, now + period) && transactions == 2);
	now += period;
	assert(BaroMeasure_Service(now, now + period) && transactions == 3);
	assert(!BaroMeasure_GetSample(now, &sample));
	now += period;
	assert(BaroMeasure_Service(now, now + period) && transactions == 3);
	assert(BaroMeasure_GetSample(now, &sample));
	assert(sample.pressure_01pa == 1000000 && sample.sequence == 1 && sample.tick == now - period);
	assert(!BaroMeasure_Service(now + period, now + 2 * period));

	// 最新压力读取失败时保留上一条有效样本，温度超时后不发布新样本。
	ready = LPC2623_PRESSURE_READY;
	pressure_failure = 1;
	now += CLOCK_SYS_CLOCK_HZ;
	assert(BaroMeasure_Service(now, now + period) && transactions == 4);
	now += period;
	assert(BaroMeasure_Service(now, now + period) && transactions == 5);
	assert(BaroMeasure_GetSample(now, &sample) && sample.sequence == 1);
	pressure_failure = 0;
	now += 2 * CLOCK_SYS_CLOCK_HZ;
	assert(!BaroMeasure_GetSample(now, &sample));
	assert(BaroMeasure_Service(now, now + period));
	assert(BaroMeasure_Service(now + period, now + 2 * period));
	assert(BaroMeasure_Service(now + 2 * period, now + 3 * period));
	assert(!BaroMeasure_GetSample(now + 2 * period, &sample));

	id_failure = 1;
	BaroMeasure_Init(now);
	assert(!BaroMeasure_Service(now + 5 * CLOCK_SYS_CLOCK_HZ, now + 6 * CLOCK_SYS_CLOCK_HZ));
	printf("PASS baro measurement: split budget, tick wrap, stale temperature and missing sensor\n");
	return 0;
}
#endif
