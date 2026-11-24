#include "BaroMeasure.h"
#include "BaroFixed.h"
#include "FlightConfig.h"
#include "PressSen_Lpc2623.h"

// 先沿用已验证的 1 Hz，并为每次总线访问或定点计算预留独立的时间窗口。
#define BARO_PERIOD_TICKS CLOCK_SYS_CLOCK_HZ
#define BARO_START_TICKS (5u * CLOCK_SYS_CLOCK_HZ)
#define BARO_MAX_AGE_TICKS (2500u * FLIGHT_TICKS_PER_MS)
#define BARO_STEP_BUDGET_TICKS (1000u * FLIGHT_TICKS_PER_US)

typedef enum {
	BARO_WAIT,
	BARO_READ_TEMP,
	BARO_READ_PRESSURE,
	BARO_COMPENSATE
} BaroMeasurePhase;

static BaroFixedCal_t s_cal;
static BaroMeasureSample s_sample;
static u32 s_next_poll;
static u32 s_temp_tick;
static u32 s_pressure_tick;
static s32 s_temp;
static s32 s_pressure;
static BaroMeasurePhase s_phase;
static u8 s_have_temp;
static u8 s_pressure_ready;

void BaroMeasure_Init(u32 now)
{
	s_cal.valid = 0;
	s_sample.sequence = 0;
	s_phase = BARO_WAIT;
	s_have_temp = 0;
	if (LPC2623_CheckId()) {
		return;
	}
	LPC2623_rateset(PRESSURE_SENSOR, 1, 8);
	LPC2623_rateset(TEMPERATURE_SENSOR, 1, 1);
	LPC2623_start_continuous(CONTINUOUS_P_AND_T);
	// 初始化在第一控制拍之前完成；读不到有效校准时保留手动飞行。
	BaroFixed_LoadCalibration(&s_cal);
	s_next_poll = now + BARO_START_TICKS;
}

// 每次最多推进一步。采集和温补分别执行，测量模块不触碰姿态和油门。
u8 BaroMeasure_Service(u32 now, u32 next_control_tick)
{
	u8 ready;
	s32 result;

	if (!s_cal.valid || (s32)(next_control_tick - now) < (s32)BARO_STEP_BUDGET_TICKS) {
		return 0;
	}

	switch (s_phase) {
	case BARO_WAIT:
		if ((s32)(now - s_next_poll) < 0) {
			return 0;
		}
		s_next_poll = now + BARO_PERIOD_TICKS;
		if (LPC2623_ReadReady(&ready) == 0) {
			s_pressure_ready = (ready & LPC2623_PRESSURE_READY) != 0;
			if (ready & LPC2623_TEMP_READY) {
				s_phase = BARO_READ_TEMP;
			} else if (s_pressure_ready) {
				s_phase = BARO_READ_PRESSURE;
			}
		}
		return 1;
	case BARO_READ_TEMP:
		if (LPC2623_ReadTempRaw(&s_temp) == 0) {
			s_temp_tick = now;
			s_have_temp = 1;
		}
		s_phase = s_pressure_ready ? BARO_READ_PRESSURE : BARO_WAIT;
		return 1;
	case BARO_READ_PRESSURE:
		if (LPC2623_ReadRaw(&s_pressure) == 0) {
			s_pressure_tick = now;
			s_phase = BARO_COMPENSATE;
		} else {
			s_phase = BARO_WAIT;
		}
		return 1;
	case BARO_COMPENSATE:
		s_phase = BARO_WAIT;
		if (s_have_temp && (u32)(now - s_temp_tick) <= BARO_MAX_AGE_TICKS
			&& (u32)(now - s_pressure_tick) <= BARO_PERIOD_TICKS
			&& BaroFixed_Compensate01Pa(s_pressure, s_temp, &s_cal, &result) == BARO_FIXED_OK) {
			s_sample.pressure_01pa = result;
			s_sample.tick = s_pressure_tick;
			if (++s_sample.sequence == 0) s_sample.sequence = 1;
		}
		return 1;
	}
	return 0;
}

u8 BaroMeasure_GetSample(u32 now, BaroMeasureSample *out)
{
	if (!s_sample.sequence || (u32)(now - s_sample.tick) > BARO_MAX_AGE_TICKS) {
		return 0;
	}
	*out = s_sample;
	return 1;
}
