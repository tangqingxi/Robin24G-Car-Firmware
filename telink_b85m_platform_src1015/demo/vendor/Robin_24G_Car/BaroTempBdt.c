#include "BaroTempBdt.h"
#include "FlightConfig.h"
#include "FlightLoop.h"
#include "PressSen_Lpc2623.h"
#include "driver.h"

#define BT_SLOTS       BARO_TEMP_SLOTS
#define BT_P_BYTES     (BT_SLOTS * 2)
#define BT_T_BYTES     (BT_SLOTS * 2)
#define BT_BIT_BYTES   ((BT_SLOTS + 7) / 8)
#define BT_HDR_BYTES   48

#ifdef BARO_BDT_HOST_TEST
#define BT_CLOCK()        BaroTempBdt_TestClock()
#define BT_FLASH_ERASE(a) BaroTempBdt_TestFlashErase(a)
#define BT_FLASH_WRITE(a,l,d) BaroTempBdt_TestFlashWrite(a,l,d)
#define BT_FLASH_READ(a,l,d) BaroTempBdt_TestFlashRead(a,l,d)
#else
#define BT_CLOCK()        clock_time()
#define BT_FLASH_ERASE(a) flash_erase_sector(a)
#define BT_FLASH_WRITE(a,l,d) flash_write_page(a,l,d)
#define BT_FLASH_READ(a,l,d) flash_read_page(a,l,d)
#endif

static s16 s_p_delta[BT_SLOTS];
static s16 s_t_delta[BT_SLOTS];
static u8 s_p_valid[BT_BIT_BYTES];
static u8 s_t_valid[BT_BIT_BYTES];
static u8 s_header[BT_HDR_BYTES];
static u8 s_started;
static u8 s_state;
static u8 s_seen_armed;
static u8 s_init_failed;
static u8 s_have_pbase;
static u8 s_have_tbase;
static s32 s_pbase;
static s32 s_tbase;
static u16 s_p_count;
static u16 s_t_count;
static u16 s_i2c_fail;
static u16 s_dropped;
static u16 s_max_read_us;
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

	for (i = 0; i < BT_HDR_BYTES; i++) {
		crc = crc16_update(crc, s_header[i]);
	}
	p = (const u8 *)s_p_delta;
	for (i = 0; i < BT_P_BYTES; i++) {
		crc = crc16_update(crc, p[i]);
	}
	p = (const u8 *)s_t_delta;
	for (i = 0; i < BT_T_BYTES; i++) {
		crc = crc16_update(crc, p[i]);
	}
	for (i = 0; i < BT_BIT_BYTES; i++) {
		crc = crc16_update(crc, s_p_valid[i]);
		crc = crc16_update(crc, s_t_valid[i]);
	}
	return crc;
}

static void make_header(void)
{
	u16 i;

	for (i = 0; i < sizeof(s_header); i++) {
		s_header[i] = 0;
	}
	s_header[0] = 'B';
	s_header[1] = 'A';
	s_header[2] = 'R';
	s_header[3] = 'O';
	s_header[4] = BARO_TEMP_RECORD_VER;
	s_header[5] = s_init_failed ? 1 : 0;
	put16(&s_header[6], s_p_count);
	put32(&s_header[8], s_sample_start_tick / FLIGHT_TICKS_PER_MS);
	put16(&s_header[12], s_i2c_fail);
	put16(&s_header[14], s_dropped);
	put16(&s_header[16], s_max_read_us);
	put16(&s_header[18], s_max_gap_ms);
	put16(&s_header[20], s_missed_ticks);
	put16(&s_header[22], 0);
	put16(&s_header[24], BT_SLOTS);
	put16(&s_header[26], BARO_TEMP_SAMPLE_MS);
	put32(&s_header[28], (u32)s_pbase);
	put32(&s_header[32], (u32)s_tbase);
	put16(&s_header[36], BT_P_BYTES);
	put16(&s_header[38], BT_T_BYTES);
	put16(&s_header[40], s_t_count);
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
	return 0;
}

static void write_record(void)
{
	u16 crc;
	u8 verified[32];

	make_header();
	crc = record_crc();
	put16(&s_header[22], crc);

	BT_FLASH_ERASE(BARO_TEMP_SECTOR);
	BT_FLASH_WRITE(BARO_TEMP_SECTOR, sizeof(s_header), s_header);
	BT_FLASH_WRITE(BARO_TEMP_SECTOR + BT_HDR_BYTES, BT_P_BYTES,
				   (u8 *)s_p_delta);
	BT_FLASH_WRITE(BARO_TEMP_SECTOR + BT_HDR_BYTES + BT_P_BYTES,
				   BT_T_BYTES, (u8 *)s_t_delta);
	BT_FLASH_WRITE(BARO_TEMP_SECTOR + BT_HDR_BYTES + BT_P_BYTES + BT_T_BYTES,
				   BT_BIT_BYTES, s_p_valid);
	BT_FLASH_WRITE(BARO_TEMP_SECTOR + BT_HDR_BYTES + BT_P_BYTES + BT_T_BYTES
				   + BT_BIT_BYTES, BT_BIT_BYTES, s_t_valid);

	BT_FLASH_READ(BARO_TEMP_SECTOR, sizeof(verified), verified);
	if (verified[0] != 'B' || verified[1] != 'A'
		|| verified[2] != 'R' || verified[3] != 'O'
		|| verified[4] != BARO_TEMP_RECORD_VER
		|| verified[6] != (u8)s_p_count
		|| verified[7] != (u8)(s_p_count >> 8)) {
		s_state = BARO_TEMP_ST_ERROR;
		return;
	}
	s_state = BARO_TEMP_ST_DONE;
}

static void finish(void)
{
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};

	FlightLoop_GetDiagStatus(&armed, pwm, &s_max_gap_ms, &s_missed_ticks);
	if (s_seen_armed || armed || (pwm[0] | pwm[1] | pwm[2] | pwm[3])) {
		s_state = BARO_TEMP_ST_CANCELED;
		return;
	}
	if (s_p_count == 0 && s_t_count == 0) {
		s_state = BARO_TEMP_ST_ERROR;
		return;
	}
	write_record();
}

void BaroTempBdt_Process(void)
{
	u32 now;
	u32 read0, read1;
	s32 raw;
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};
	u16 gap_ms;
	u16 slot;

	if (!s_started) {
		pressure_init();               // 静置等待前启动
		s_started = 1;
		s_boot_tick = BT_CLOCK();
		s_next_tick = s_boot_tick
					  + (u32)BARO_TEMP_START_MS * FLIGHT_TICKS_PER_MS;
		s_state = BARO_TEMP_ST_WAIT;
		return;
	}

	now = BT_CLOCK();
	FlightLoop_GetDiagStatus(&armed, pwm, &gap_ms, &s_missed_ticks);
	if (gap_ms > s_max_gap_ms) {
		s_max_gap_ms = gap_ms;
	}
	if (armed) {
		s_seen_armed = 1;
	}

	if (s_state == BARO_TEMP_ST_WAIT) {
		if ((s32)(now - s_next_tick) < 0) {
			return;
		}
		FlightLoop_ResetDiagStats();
		s_sample_start_tick = now;
		s_slot = 0;
		s_state = BARO_TEMP_ST_SAMPLING;
		return;
	}
	if (s_state != BARO_TEMP_ST_SAMPLING) {
		return;
	}
	if ((u32)(now - s_sample_start_tick) >=
		(u32)BARO_TEMP_DURATION_MS * FLIGHT_TICKS_PER_MS) {
		finish();
		return;
	}

	slot = (u16)(((u32)(now - s_sample_start_tick) / FLIGHT_TICKS_PER_MS)
				 / BARO_TEMP_SAMPLE_MS);
	if (slot >= BT_SLOTS) {
		finish();
		return;
	}
	if (slot < s_slot) {
		return;
	}
	if (slot > s_slot) {
		s_dropped += (u16)(slot - s_slot);
	}
	s_slot = slot;

	// 两个通道各自检查就绪；任一未就绪只留空该通道。
	read0 = BT_CLOCK();
	if (LPC2623_IsPressureReady()) {
		if (LPC2623_ReadRaw(&raw) == 0) {
			if (!s_have_pbase) {
				s_pbase = raw;
				s_have_pbase = 1;
			}
			if (raw - s_pbase >= -32768 && raw - s_pbase <= 32767) {
				s_p_delta[slot] = (s16)(raw - s_pbase);
				s_p_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
				s_p_count++;
			} else {
				s_i2c_fail++;
			}
		} else {
			s_i2c_fail++;
		}
	}
	if (LPC2623_IsTempReady()) {
		if (LPC2623_ReadTempRaw(&raw) == 0) {
			if (!s_have_tbase) {
				s_tbase = raw;
				s_have_tbase = 1;
			}
			if (raw - s_tbase >= -32768 && raw - s_tbase <= 32767) {
				s_t_delta[slot] = (s16)(raw - s_tbase);
				s_t_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
				s_t_count++;
			} else {
				s_i2c_fail++;
			}
		} else {
			s_i2c_fail++;
		}
	}
	read1 = BT_CLOCK();
	if ((u32)(read1 - read0) / FLIGHT_TICKS_PER_US > 60000u) {
		s_max_read_us = 60000u;
	} else {
		u16 us = (u16)(((u32)(read1 - read0)) / FLIGHT_TICKS_PER_US);
		if (us > s_max_read_us) {
			s_max_read_us = us;
		}
	}

	s_slot = (u16)(slot + 1u);
	if (s_slot >= BT_SLOTS) {
		finish();
	}
}

unsigned char BaroTempBdt_GetState(void) { return s_state; }
unsigned short BaroTempBdt_GetPressureCount(void) { return s_p_count; }
unsigned short BaroTempBdt_GetTempCount(void) { return s_t_count; }

#ifdef BARO_BDT_HOST_TEST
void BaroTempBdt_TestReset(void)
{
	u16 i;

	s_started = 0;
	s_state = 0;
	s_seen_armed = 0;
	s_init_failed = 0;
	s_have_pbase = 0;
	s_have_tbase = 0;
	s_pbase = 0;
	s_tbase = 0;
	s_p_count = 0;
	s_t_count = 0;
	s_i2c_fail = 0;
	s_dropped = 0;
	s_max_read_us = 0;
	s_max_gap_ms = 0;
	s_missed_ticks = 0;
	s_slot = 0;
	s_boot_tick = 0;
	s_next_tick = 0;
	s_sample_start_tick = 0;
	for (i = 0; i < BT_SLOTS; i++) {
		s_p_delta[i] = 0;
		s_t_delta[i] = 0;
	}
	for (i = 0; i < BT_BIT_BYTES; i++) {
		s_p_valid[i] = 0;
		s_t_valid[i] = 0;
	}
}
#endif
