/*
 * 气压原始值、实际参与温补的温度、定点补偿结果三路对齐记录。
 * 采样期间只写 RAM；锁定且 PWM 全零后才写 0x10000 扇区。
 */
#include "BaroFixedBdt.h"
#include "FlightConfig.h"
#include "FlightLoop.h"
#include "PressSen_Lpc2623.h"
#include "driver.h"

#define BF_SLOTS       BARO_FIXED_SLOTS
#define BF_BYTES       (BF_SLOTS * 2)
#define BF_BITS        ((BF_SLOTS + 7) / 8)
#define BF_HDR         56

#ifdef BARO_BDT_HOST_TEST
#define BF_CLOCK()        BaroFixedBdt_TestClock()
#define BF_FLASH_ERASE(a) BaroFixedBdt_TestFlashErase(a)
#define BF_FLASH_WRITE(a,l,d) BaroFixedBdt_TestFlashWrite(a,l,d)
#define BF_FLASH_READ(a,l,d) BaroFixedBdt_TestFlashRead(a,l,d)
#else
#define BF_CLOCK()        clock_time()
#define BF_FLASH_ERASE(a) flash_erase_sector(a)
#define BF_FLASH_WRITE(a,l,d) flash_write_page(a,l,d)
#define BF_FLASH_READ(a,l,d) flash_read_page(a,l,d)
#endif

static BaroFixedCal_t s_cal;
static s16 s_p_delta[BF_SLOTS];
static s16 s_t_delta[BF_SLOTS];
static s16 s_f_delta[BF_SLOTS];
static u8 s_p_valid[BF_BITS];
static u8 s_t_valid[BF_BITS];
static u8 s_f_valid[BF_BITS];
static u8 s_header[BF_HDR];
static u8 s_started;
static u8 s_state;
static u8 s_seen_armed;
static u8 s_init_failed;
static u8 s_have_pbase;
static u8 s_have_tbase;
static u8 s_have_fbase;
static u8 s_have_temp;
static s32 s_pbase;
static s32 s_tbase;
static s32 s_fbase;
static s32 s_last_temp;
static u32 s_last_temp_tick;
static u16 s_p_count;
static u16 s_t_count;
static u16 s_f_count;
static u16 s_i2c_fail;
static u16 s_dropped;
static u16 s_max_read_us;
static u16 s_max_comp_us;
static u16 s_max_gap_ms;
static u16 s_missed_ticks;
static u16 s_slot;
static u32 s_boot_tick;
static u32 s_next_tick;
static u32 s_sample_start_tick;

static void put16(u8 *p, u16 v)
{
	p[0] = (u8)v;
	p[1] = (u8)(v >> 8);
}

static void put32(u8 *p, u32 v)
{
	p[0] = (u8)v;
	p[1] = (u8)(v >> 8);
	p[2] = (u8)(v >> 16);
	p[3] = (u8)(v >> 24);
}

static u16 crc16_update(u16 crc, u8 d)
{
	u8 i;

	crc ^= (u16)d << 8;
	for (i = 0; i < 8; i++) {
		crc = (crc & 0x8000) ? (u16)((crc << 1) ^ 0x1021)
							 : (u16)(crc << 1);
	}
	return crc;
}

static u16 record_crc(void)
{
	u16 crc = 0xFFFF;
	u16 i;
	const u8 *p;

	for (i = 0; i < BF_HDR; i++) {
		crc = crc16_update(crc, s_header[i]);
	}
	p = (const u8 *)s_p_delta;
	for (i = 0; i < BF_BYTES; i++) crc = crc16_update(crc, p[i]);
	p = (const u8 *)s_t_delta;
	for (i = 0; i < BF_BYTES; i++) crc = crc16_update(crc, p[i]);
	p = (const u8 *)s_f_delta;
	for (i = 0; i < BF_BYTES; i++) crc = crc16_update(crc, p[i]);
	for (i = 0; i < BF_BITS; i++) {
		crc = crc16_update(crc, s_p_valid[i]);
		crc = crc16_update(crc, s_t_valid[i]);
		crc = crc16_update(crc, s_f_valid[i]);
	}
	return crc;
}

static void make_header(void)
{
	u16 i;

	for (i = 0; i < sizeof(s_header); i++) s_header[i] = 0;
	s_header[0] = 'B'; s_header[1] = 'A'; s_header[2] = 'R'; s_header[3] = 'O';
	s_header[4] = BARO_FIXED_RECORD_VER;
	s_header[5] = s_init_failed ? 1 : 0;
	put16(&s_header[6], s_p_count);
	put32(&s_header[8], s_sample_start_tick / FLIGHT_TICKS_PER_MS);
	put16(&s_header[12], s_i2c_fail);
	put16(&s_header[14], s_dropped);
	put16(&s_header[16], s_max_read_us);
	put16(&s_header[18], s_max_gap_ms);
	put16(&s_header[20], s_missed_ticks);
	put16(&s_header[22], 0);
	put16(&s_header[24], BF_SLOTS);
	put16(&s_header[26], BARO_FIXED_SAMPLE_MS);
	put32(&s_header[28], (u32)s_pbase);
	put32(&s_header[32], (u32)s_tbase);
	put16(&s_header[36], BF_BYTES);
	put16(&s_header[38], BF_BYTES);
	put16(&s_header[40], s_f_count);
	put32(&s_header[42], (u32)s_fbase);
	put16(&s_header[46], BF_BYTES);
	put16(&s_header[48], s_max_comp_us);
}

static u8 pressure_init(void)
{
	if (LPC2623_CheckId()) {
		s_init_failed = 1;
		s_i2c_fail++;
		return 1;
	}
	LPC2623_rateset(PRESSURE_SENSOR, 1, 8);
	LPC2623_rateset(TEMPERATURE_SENSOR, 1, 1);
	LPC2623_start_continuous(CONTINUOUS_P_AND_T);
	// 先完成气压配置和连续测量，再读 PSR_CFG/出厂系数。
	if (BaroFixed_LoadCalibration(&s_cal)) {
		s_init_failed = 1;
	}
	return 0;
}

static void write_record(void)
{
	u16 crc;
	u8 verified[32];

	make_header();
	crc = record_crc();
	put16(&s_header[22], crc);
	BF_FLASH_ERASE(BARO_FIXED_SECTOR);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR, sizeof(s_header), s_header);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR, BF_BYTES, (u8 *)s_p_delta);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR + BF_BYTES, BF_BYTES,
				   (u8 *)s_t_delta);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR + 2 * BF_BYTES, BF_BYTES,
				   (u8 *)s_f_delta);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR + 3 * BF_BYTES, BF_BITS,
				   s_p_valid);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR + 3 * BF_BYTES + BF_BITS,
				   BF_BITS, s_t_valid);
	BF_FLASH_WRITE(BARO_FIXED_SECTOR + BF_HDR + 3 * BF_BYTES + 2 * BF_BITS,
				   BF_BITS, s_f_valid);
	BF_FLASH_READ(BARO_FIXED_SECTOR, sizeof(verified), verified);
	if (verified[0] != 'B' || verified[1] != 'A'
		|| verified[2] != 'R' || verified[3] != 'O'
		|| verified[4] != BARO_FIXED_RECORD_VER
		|| verified[6] != (u8)s_p_count
		|| verified[7] != (u8)(s_p_count >> 8)) {
		s_state = BARO_FIXED_ST_ERROR;
		return;
	}
	s_state = BARO_FIXED_ST_DONE;
}

static void finish(void)
{
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};

	FlightLoop_GetDiagStatus(&armed, pwm, &s_max_gap_ms, &s_missed_ticks);
	if (s_seen_armed || armed || (pwm[0] | pwm[1] | pwm[2] | pwm[3])) {
		s_state = BARO_FIXED_ST_CANCELED;
		return;
	}
	if (s_p_count == 0 && s_t_count == 0 && s_f_count == 0) {
		s_state = BARO_FIXED_ST_ERROR;
		return;
	}
	write_record();
}

void BaroFixedBdt_Process(void)
{
	u32 now, read0, read_ticks, comp0, comp_ticks = 0;
	s32 p_raw, t_raw, f01;
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};
	u16 gap_ms;
	u16 slot;
	u8 temp_fresh = 0;
	u8 temp_used = 0;

	if (!s_started) {
		pressure_init();
		s_started = 1;
		s_boot_tick = BF_CLOCK();
		s_next_tick = s_boot_tick
					  + (u32)BARO_FIXED_START_MS * FLIGHT_TICKS_PER_MS;
		s_state = BARO_FIXED_ST_WAIT;
		return;
	}
	now = BF_CLOCK();
	FlightLoop_GetDiagStatus(&armed, pwm, &gap_ms, &s_missed_ticks);
	if (gap_ms > s_max_gap_ms) s_max_gap_ms = gap_ms;
	if (armed) s_seen_armed = 1;

	if (s_state == BARO_FIXED_ST_WAIT) {
		if ((s32)(now - s_next_tick) < 0) return;
		// 传感器已配置并静置，读取实际过采样档和出厂校准。
		if (!s_init_failed && BaroFixed_LoadCalibration(&s_cal)) {
			s_init_failed = 1;
			s_i2c_fail++;
		}
		FlightLoop_ResetDiagStats();
		s_sample_start_tick = BF_CLOCK();
		s_slot = 0;
		s_state = BARO_FIXED_ST_SAMPLING;
		return;
	}
	if (s_state != BARO_FIXED_ST_SAMPLING) return;
	if ((u32)(now - s_sample_start_tick) >=
		(u32)BARO_FIXED_DURATION_MS * FLIGHT_TICKS_PER_MS) {
		finish();
		return;
	}
	slot = (u16)(((u32)(now - s_sample_start_tick) / FLIGHT_TICKS_PER_MS)
				 / BARO_FIXED_SAMPLE_MS);
	if (slot >= BF_SLOTS) { finish(); return; }
	if (slot < s_slot) return;
	if (slot > s_slot) s_dropped += (u16)(slot - s_slot);
	s_slot = slot;

	read0 = BF_CLOCK();
	if (LPC2623_IsTempReady()) {
		if (LPC2623_ReadTempRaw(&t_raw) == 0) {
			if (!s_have_tbase) { s_tbase = t_raw; s_have_tbase = 1; }
			if (t_raw - s_tbase >= -32768 && t_raw - s_tbase <= 32767) {
				s_t_delta[slot] = (s16)(t_raw - s_tbase);
				s_t_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
				s_t_count++;
			}
			s_last_temp = t_raw;
			s_last_temp_tick = now;
			s_have_temp = 1;
			temp_fresh = 1;
		} else {
			s_i2c_fail++;
		}
	}

	if (LPC2623_IsPressureReady()) {
		if (LPC2623_ReadRaw(&p_raw) == 0) {
			if (!s_have_pbase) { s_pbase = p_raw; s_have_pbase = 1; }
			if (p_raw - s_pbase >= -32768 && p_raw - s_pbase <= 32767) {
				s_p_delta[slot] = (s16)(p_raw - s_pbase);
				s_p_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
				s_p_count++;
			}
			if (s_have_temp
				&& (u32)(now - s_last_temp_tick) / FLIGHT_TICKS_PER_MS
				   <= BARO_FIXED_TEMP_TIMEOUT_MS) {
				u8 status;
				u32 us;

				comp0 = BF_CLOCK();
				status = BaroFixed_Compensate01Pa(p_raw, s_last_temp, &s_cal, &f01);
				comp_ticks = (u32)(BF_CLOCK() - comp0);
				us = comp_ticks / FLIGHT_TICKS_PER_US;
				if (us > s_max_comp_us) s_max_comp_us = (u16)(us > 60000u ? 60000u : us);
				if (status == BARO_FIXED_OK) {
					if (!s_have_fbase) { s_fbase = f01; s_have_fbase = 1; }
					if (f01 - s_fbase >= -32768 && f01 - s_fbase <= 32767) {
						s_f_delta[slot] = (s16)(f01 - s_fbase);
						s_f_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
						s_f_count++;
					}
				}
				temp_used = 1;
			}
		} else {
			s_i2c_fail++;
		}
	}

	// 温度没有刷新但参与了本次补偿：把实际使用的温度写入本槽，便于对齐。
	if (temp_used && !temp_fresh && s_have_temp) {
		if (s_last_temp - s_tbase >= -32768 && s_last_temp - s_tbase <= 32767) {
			s_t_delta[slot] = (s16)(s_last_temp - s_tbase);
			s_t_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
			s_t_count++;
		}
	}

	// 读取及样本整理耗时扣除上面单独统计的温补耗时。
	read_ticks = (u32)(BF_CLOCK() - read0) - comp_ticks;
	if (read_ticks / FLIGHT_TICKS_PER_US > 60000u) {
		s_max_read_us = 60000u;
	} else {
		u16 us = (u16)(read_ticks / FLIGHT_TICKS_PER_US);
		if (us > s_max_read_us) s_max_read_us = us;
	}

	s_slot = (u16)(slot + 1u);
	if (s_slot >= BF_SLOTS) finish();
}

unsigned char BaroFixedBdt_GetState(void) { return s_state; }
unsigned short BaroFixedBdt_GetPressureCount(void) { return s_p_count; }
unsigned short BaroFixedBdt_GetTempCount(void) { return s_t_count; }
unsigned short BaroFixedBdt_GetFixedCount(void) { return s_f_count; }

#ifdef BARO_BDT_HOST_TEST
void BaroFixedBdt_TestReset(void)
{
	u16 i;
	s_started = 0; s_state = 0; s_seen_armed = 0; s_init_failed = 0;
	s_have_pbase = 0; s_have_tbase = 0; s_have_fbase = 0; s_have_temp = 0;
	s_pbase = 0; s_tbase = 0; s_fbase = 0; s_last_temp = 0;
	s_last_temp_tick = 0; s_p_count = 0; s_t_count = 0; s_f_count = 0;
	s_i2c_fail = 0; s_dropped = 0; s_max_read_us = 0; s_max_comp_us = 0;
	s_max_gap_ms = 0; s_missed_ticks = 0; s_slot = 0; s_boot_tick = 0;
	s_next_tick = 0; s_sample_start_tick = 0;
	s_cal.valid = 0;
	for (i = 0; i < BF_SLOTS; i++) {
		s_p_delta[i] = 0; s_t_delta[i] = 0; s_f_delta[i] = 0;
	}
	for (i = 0; i < BF_BITS; i++) {
		s_p_valid[i] = 0; s_t_valid[i] = 0; s_f_valid[i] = 0;
	}
}
#endif
