#include "BaroBdt.h"
#include "FlightConfig.h"
#include "FlightLoop.h"
#include "PressSen_Lpc2623.h"
#include "driver.h"

#ifdef BARO_BDT_HOST_TEST
u32 BaroBdt_TestClock(void);
void BaroBdt_TestFlashErase(unsigned long addr);
void BaroBdt_TestFlashWrite(unsigned long addr, unsigned long len, u8 *data);
void BaroBdt_TestFlashRead(unsigned long addr, unsigned long len, u8 *data);
#define BARO_BDT_CLOCK() BaroBdt_TestClock()
#define BARO_FLASH_ERASE(a) BaroBdt_TestFlashErase(a)
#define BARO_FLASH_WRITE(a,l,d) BaroBdt_TestFlashWrite(a,l,d)
#define BARO_FLASH_READ(a,l,d) BaroBdt_TestFlashRead(a,l,d)
#else
#define BARO_BDT_CLOCK() clock_time()
#define BARO_FLASH_ERASE(a) flash_erase_sector(a)
#define BARO_FLASH_WRITE(a,l,d) flash_write_page(a,l,d)
#define BARO_FLASH_READ(a,l,d) flash_read_page(a,l,d)
#endif

// 300 个 100ms 槽位：raw 600B + 有效位图 38B。
// 失败槽位保留 invalid，导出时按槽位还原会话时间，避免丢掉原始计数。
#define BARO_BDT_SLOTS        BARO_BDT_MAX_SAMPLES
#define BARO_BDT_RAW_BYTES    (BARO_BDT_SLOTS * 2)
#define BARO_BDT_BIT_BYTES    ((BARO_BDT_SLOTS + 7) / 8)
#define BARO_BDT_HEADER_BYTES 40

static s16 s_raw[BARO_BDT_SLOTS];
static u8 s_valid[BARO_BDT_BIT_BYTES];
static u8 s_header[BARO_BDT_HEADER_BYTES];
static u8 s_started;
static u8 s_state;
static u8 s_seen_armed;
static u8 s_init_failed;
static u16 s_count;
static u16 s_i2c_fail;
static u16 s_dropped;
static u16 s_max_read_us;
static u16 s_max_gap_ms;
static u16 s_missed_ticks;
static u16 s_slot;
static s32 s_base;
static u8 s_have_base;
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
	const u8 *p = (const u8 *)s_raw;

	for (i = 0; i < BARO_BDT_HEADER_BYTES; i++) {
		crc = crc16_update(crc, s_header[i]);
	}
	for (i = 0; i < BARO_BDT_RAW_BYTES; i++) {
		crc = crc16_update(crc, p[i]);
	}
	for (i = 0; i < BARO_BDT_BIT_BYTES; i++) {
		crc = crc16_update(crc, s_valid[i]);
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
	s_header[4] = BARO_BDT_RECORD_VERSION;   // 记录版本
	s_header[5] = s_init_failed ? 1 : 0;     // bit0：气压初始化失败
	put16(&s_header[6], s_count);            // 有效样本数
	put32(&s_header[8], s_sample_start_tick / FLIGHT_TICKS_PER_MS);
	put16(&s_header[12], s_i2c_fail);
	put16(&s_header[14], s_dropped);
	put16(&s_header[16], s_max_read_us);
	put16(&s_header[18], s_max_gap_ms);
	put16(&s_header[20], s_missed_ticks);
	put16(&s_header[22], 0);                  // 先填头，CRC 随后计算
	put16(&s_header[24], BARO_BDT_RAW_BYTES); // delta 区字节数
	put16(&s_header[26], 0x0001);             // bit0：完整记录
	put16(&s_header[28], BARO_BDT_SLOTS);     // 槽位数
	put32(&s_header[30], (u32)s_base);        // 32 位原始压力基准
	put16(&s_header[34], BARO_BDT_SAMPLE_MS); // 采样间隔 ms
}

static u8 pressure_init(void)
{
	if (LPC2623_CheckId()) {
		s_init_failed = 1;
		s_i2c_fail++;
		return 1;
	}
	LPC2623_rateset(PRESSURE_SENSOR, 64, 8);
	LPC2623_rateset(TEMPERATURE_SENSOR, 1, 1);
	LPC2623_start_continuous(CONTINUOUS_PRESSURE);
	return 0;
}

static void write_record(void)
{
	u16 crc;
	u16 i;
	u8 verified[32];

	make_header();
	crc = record_crc();
	put16(&s_header[22], crc);

	BARO_FLASH_ERASE(BARO_BDT_SECTOR);

	// 原始计数区：600B；最后不足一页时直接按实际长度写。
	for (i = 0; i < BARO_BDT_RAW_BYTES; i += 256u) {
		u16 n = BARO_BDT_RAW_BYTES - i;

		if (n > 256u) {
			n = 256u;
		}
		BARO_FLASH_WRITE(BARO_BDT_SECTOR + BARO_BDT_HEADER_BYTES + i,
						 (unsigned long)n, (u8 *)s_raw + i);
	}
	BARO_FLASH_WRITE(BARO_BDT_SECTOR + BARO_BDT_HEADER_BYTES + BARO_BDT_RAW_BYTES,
					 BARO_BDT_BIT_BYTES, s_valid);
	BARO_FLASH_WRITE(BARO_BDT_SECTOR, sizeof(s_header), s_header);

	BARO_FLASH_READ(BARO_BDT_SECTOR, sizeof(verified), verified);
	if (verified[0] != 'B' || verified[1] != 'A'
		|| verified[2] != 'R' || verified[3] != 'O'
		|| verified[4] != BARO_BDT_RECORD_VERSION
		|| verified[6] != (u8)s_count
		|| verified[7] != (u8)(s_count >> 8)) {
		s_state = BARO_BDT_ST_ERROR;
		return;
	}
	s_state = BARO_BDT_ST_DONE;
}

static void finish(void)
{
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};

	FlightLoop_GetDiagStatus(&armed, pwm, &s_max_gap_ms, &s_missed_ticks);
	if (s_seen_armed || armed || (pwm[0] | pwm[1] | pwm[2] | pwm[3])) {
		s_state = BARO_BDT_ST_CANCELED;
		return;
	}
	if (s_count == 0) {
		s_state = BARO_BDT_ST_ERROR;
		return;
	}
	write_record();
}

void BaroBdt_Process(void)
{
	u32 now;
	u32 read0, read1;
	s32 raw;
	u8 armed = 0;
	u16 pwm[4] = {0, 0, 0, 0};
	u16 gap_ms;
	u16 slot;

	if (!s_started) {
		// 气压计先在静置等待前启动，等待期间让转换结果稳定下来。
		pressure_init();
		s_started = 1;
		s_boot_tick = BARO_BDT_CLOCK();
		s_next_tick = s_boot_tick
					  + (u32)BARO_BDT_START_MS * FLIGHT_TICKS_PER_MS;
		s_state = BARO_BDT_ST_WAIT;
		return;
	}

	now = BARO_BDT_CLOCK();

	FlightLoop_GetDiagStatus(&armed, pwm, &gap_ms, &s_missed_ticks);
	if (gap_ms > s_max_gap_ms) {
		s_max_gap_ms = gap_ms;
	}
	if (armed) {
		s_seen_armed = 1;
	}

	if (s_state == BARO_BDT_ST_WAIT) {
		if ((s32)(now - s_next_tick) < 0) {
			return;
		}
		FlightLoop_ResetDiagStats();
		s_sample_start_tick = now;
		s_slot = 0;
		s_state = BARO_BDT_ST_SAMPLING;
		return;                         // 下一轮按槽位读取
	}

	if (s_state != BARO_BDT_ST_SAMPLING) {
		return;
	}

	if ((u32)(now - s_sample_start_tick) >=
		(u32)BARO_BDT_DURATION_MS * FLIGHT_TICKS_PER_MS) {
		finish();
		return;
	}

	slot = (u16)(((u32)(now - s_sample_start_tick) / FLIGHT_TICKS_PER_MS)
				 / BARO_BDT_SAMPLE_MS);
	if (slot >= BARO_BDT_SLOTS) {
		finish();
		return;
	}
	if (slot < s_slot) {
		return;                         // 该槽已处理，绝不重复写
	}
	if (slot > s_slot) {
		s_dropped += (u16)(slot - s_slot);  // 漏掉的槽保持无效
	}
	s_slot = slot;

	// 只有数据就绪才读取并设首样本基准；初始寄存器值不进入记录。
	if (!LPC2623_IsPressureReady()) {
		s_dropped++;
		s_slot = (u16)(slot + 1u);
		return;
	}

	read0 = BARO_BDT_CLOCK();
	if (LPC2623_ReadRaw(&raw) == 0) {
		s32 delta;

		if (!s_have_base) {
			s_base = raw;
			s_have_base = 1;
		}
		delta = raw - s_base;
		if (delta < -32768 || delta > 32767) {
			s_i2c_fail++;              // 本实验量程内不应发生，坏了也不写假值
		} else {
			read1 = BARO_BDT_CLOCK();
			if ((u32)(read1 - read0) / FLIGHT_TICKS_PER_US > 60000u) {
				s_max_read_us = 60000u;
			} else {
				u16 us = (u16)(((u32)(read1 - read0))
							   / FLIGHT_TICKS_PER_US);
				if (us > s_max_read_us) {
					s_max_read_us = us;
				}
			}
			s_raw[slot] = (s16)delta;
			s_valid[slot / 8u] |= (u8)(1u << (slot % 8u));
			s_count++;
		}
	} else {
		s_i2c_fail++;
	}

	s_slot = (u16)(slot + 1u);
	if (s_slot >= BARO_BDT_SLOTS) {
		finish();
	}
}

unsigned char BaroBdt_GetState(void)
{
	return s_state;
}

unsigned short BaroBdt_GetCount(void)
{
	return s_count;
}

#ifdef BARO_BDT_HOST_TEST
// 仅主机回归：清空状态，便于同一进程测试完成/取消两条路径。
void BaroBdt_TestReset(void)
{
	u16 i;

	s_started = 0;
	s_state = 0;
	s_seen_armed = 0;
	s_init_failed = 0;
	s_count = 0;
	s_i2c_fail = 0;
	s_dropped = 0;
	s_max_read_us = 0;
	s_max_gap_ms = 0;
	s_missed_ticks = 0;
	s_slot = 0;
	s_base = 0;
	s_have_base = 0;
	s_boot_tick = 0;
	s_next_tick = 0;
	s_sample_start_tick = 0;
	for (i = 0; i < BARO_BDT_SLOTS; i++) {
		s_raw[i] = 0;
	}
	for (i = 0; i < BARO_BDT_BIT_BYTES; i++) {
		s_valid[i] = 0;
	}
}
#endif
