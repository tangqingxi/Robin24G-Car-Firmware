#include <string.h>
#include "FlightLog.h"
#include "FlightConfig.h"
#include "FlightArmDebug.h"
#if C_FLIGHT_ARM_DEBUG
#include "printf.h"
#include "UserUart.h"
#endif

#ifdef FLIGHT_LOG_HOST_TEST
void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf);
void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf);
void flash_erase_sector(unsigned int addr);
#else
#include "flash.h"
#endif

#define FLIGHT_LOG_MAGIC       0x4C4F4731u
#define FLIGHT_LOG_HDR_CRC_OFF 14u

typedef struct {
	u32 page_base;
	u16 page_seq;
	u16 page_count;
	u16 page_crc;
	u16 record_seq;
	u16 session;
	u16 session_serial;
	u8 active;
	u8 page_open;
	u8 state;
	u8 pending;
	u8 last_result;
	u8 need_release;
	u8 link_seen;
	u8 event_pending;
	u8 event_code;
	u8 closing;
	u8 summary_pending;
	u8 tail_pending;
	u8 close_pending;
	u8 baseline_written;
	u8 frozen;
	u8 need_erase;
	u8 erase_index;
	u8 ram_started;
	u16 key_prev;
	u32 tick;
	u32 dropped;
	u32 data_records;
	u32 start_ms;
	u32 end_ms;
	u32 last_ctrl_tick;
	u32 session_us;
	u32 max_gap_ticks;
	u32 base_max_gap_ticks;
	u32 base_late_count;
	u32 base_skip_count;
	u32 late_count;
	u32 skip_count;
	u32 flash_peak_ticks;
	u32 prep_ticks;
#if C_FLIGHT_ARM_DEBUG
	u32 written;
	u32 physical_writes;
	u32 queue_drops;
	u32 pack_peak_ticks;
	u32 rec_crc_peak_ticks;
	u32 page_crc_peak_ticks;
#endif
	u8 end_reason;
	FlightLogSample q[2];
	u8 qhead;
	u8 qtail;
	u8 qcount;
	FlightLogSample event_sample;
} FlightLogState;

static FlightLogState s_log;

// CRC-16/CCITT 查表：与逐位算法输出完全一致，只减少每字节 8 次移位。
static const u16 crc16_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
	0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
	0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
	0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
	0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
	0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
	0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
	0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
	0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
	0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
	0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
	0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
	0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
	0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
	0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
	0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
	0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
	0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
	0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
	0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
	0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
	0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
	0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static u16 crc16_update(u16 crc, const u8 *p, u32 n)
{
	while (n--) {
		crc = (u16)((crc << 8) ^ crc16_table[((crc >> 8) ^ *p++) & 0xFF]);
	}
	return crc;
}
static void put16(u8 *p, u16 v)
{
	p[0] = (u8)(v & 0xFF);
	p[1] = (u8)(v >> 8);
}

static void put32(u8 *p, u32 v)
{
	p[0] = (u8)(v & 0xFF);
	p[1] = (u8)((v >> 8) & 0xFF);
	p[2] = (u8)((v >> 16) & 0xFF);
	p[3] = (u8)((v >> 24) & 0xFF);
}

static void put_s32(u8 *p, s32 v) { put32(p, (u32)v); }
static void put_s16(u8 *p, s16 v) { put16(p, (u16)v); }

static u32 now_ms(void) { return clock_time() / FLIGHT_TICKS_PER_MS; }

static u32 ticks_to_us(u32 ticks)
{
	return ticks / (CLOCK_SYS_CLOCK_HZ / 1000000u);
}

static void close_page(void)
{
	u8 hdr[16];

	if (!s_log.page_open || !s_log.page_count) {
		return;
	}
	put32(&hdr[0], FLIGHT_LOG_MAGIC);
	put16(&hdr[4], FLIGHT_LOG_VERSION);
	put16(&hdr[6], s_log.session);
	put16(&hdr[8], s_log.page_seq);
	put16(&hdr[10], s_log.page_count);
	put16(&hdr[12], s_log.page_crc);
	put16(&hdr[14], crc16_update(0xFFFF, hdr, 14));
	flash_write_page(s_log.page_base, 16, hdr);
#if C_FLIGHT_ARM_DEBUG
	s_log.physical_writes++;
#endif
	s_log.page_open = 0;
}

static void open_page(u16 seq)
{
	s_log.page_base = FLIGHT_LOG_BASE + (u32)seq * FLIGHT_LOG_SECTOR_SIZE;
	s_log.page_seq = seq;
	s_log.page_count = 0;
	s_log.page_crc = 0xFFFF;
	s_log.page_open = 1;
}

static void erase_log(void)
{
	u32 i;

	for (i = 0; i < FLIGHT_LOG_SECTORS; i++) {
		flash_erase_sector(FLIGHT_LOG_BASE + i * FLIGHT_LOG_SECTOR_SIZE);
	}
}

// 飞行阶段只写 RAM：10 Hz、定长、循环覆盖；落地锁定后再批量落盘。
#define FLIGHT_RAM_HZ          10u
#define FLIGHT_RAM_SECONDS     3u
#define FLIGHT_RAM_N           (FLIGHT_RAM_HZ * FLIGHT_RAM_SECONDS + 4u)
#define FLIGHT_RAM_DIV         (FLIGHT_LOOP_HZ / FLIGHT_RAM_HZ)

typedef struct {
	u16 t10;
	s16 yaw_stick;
	s16 yaw_target;
	s16 gyro_z;
	s16 yaw_corr;
	u16 thrust;
	u16 pwm[4];
	u8 flags;
} FlightRamRec;

static FlightRamRec s_ram[FLIGHT_RAM_N];
static u16 s_ram_head;
static u16 s_ram_count;
static u8 write_blob(const u8 *rec, u8 is_data)
{
	u32 phys, slot, addr;
	u32 t0, dt;

	if (!s_log.page_open) {
		if (s_log.page_seq + 1 >= FLIGHT_LOG_SECTORS) {
			return 0;
		}
		open_page((u16)(s_log.page_seq + 1));
	}
	if (s_log.page_count >= FLIGHT_LOG_REC_PER_SEC) {
		close_page();                  // 封口单独一次物理编程
		return 0;
	}
	phys = s_log.page_count / FLIGHT_LOG_REC_PER_PHYS;
	slot = s_log.page_count % FLIGHT_LOG_REC_PER_PHYS;
	addr = s_log.page_base + FLIGHT_LOG_PAGE_HDR
		   + phys * 256u + slot * FLIGHT_LOG_REC_SIZE;
	t0 = clock_time();
	flash_write_page(addr, FLIGHT_LOG_REC_SIZE, (u8 *)rec);
	dt = (u32)(clock_time() - t0);
	if (dt > s_log.flash_peak_ticks) {
		s_log.flash_peak_ticks = dt;
	}
	if (s_log.closing) {
		if (dt > s_log.max_gap_ticks) {
			s_log.max_gap_ticks = dt;
		}
		if (dt > FLIGHT_PERIOD_TICKS) {
			s_log.late_count++;
		}
	}
#if C_FLIGHT_ARM_DEBUG
	{
		u32 pc0 = clock_time();
		u32 pc_dt;

		s_log.page_crc = crc16_update(s_log.page_crc, rec, FLIGHT_LOG_REC_SIZE);
		pc_dt = (u32)(clock_time() - pc0);
		if (pc_dt > s_log.page_crc_peak_ticks) {
			s_log.page_crc_peak_ticks = pc_dt;
		}
	}
#else
	s_log.page_crc = crc16_update(s_log.page_crc, rec, FLIGHT_LOG_REC_SIZE);
#endif
	s_log.page_count++;
	s_log.record_seq++;
	if (is_data) {
		s_log.data_records++;
	}
#if C_FLIGHT_ARM_DEBUG
	s_log.written++;
	s_log.physical_writes++;
#endif
	return 1;
}
static u8 write_meta(u8 type)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];
	u16 flags = (u16)(type << FLIGHT_LOG_TYPE_SHIFT);

	memset(rec, 0, sizeof(rec));
	put32(&rec[0], s_log.session_us / 1000u);
	put16(&rec[4], s_log.session);
	put16(&rec[6], (u16)s_log.record_seq);
	put32(&rec[8], 0);
	put32(&rec[12], s_log.session_us / 1000u);
	put32(&rec[16], ticks_to_us(s_log.max_gap_ticks));
	put16(&rec[20], (u16)s_log.late_count);
	put16(&rec[22], (u16)s_log.skip_count);
	put16(&rec[24], (u16)ticks_to_us(s_log.flash_peak_ticks));
	put16(&rec[26], (u16)(ticks_to_us(s_log.prep_ticks) / 1000u));
	put16(&rec[28], (u16)s_log.data_records);
	put16(&rec[30], (u16)s_log.dropped);
	rec[32] = s_log.end_reason;
	put16(&rec[52], flags);
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
	return write_blob(rec, 0);
}

static void write_baseline(void)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];
	u16 flags = (u16)(FLIGHT_LOG_TYPE_BASELINE << FLIGHT_LOG_TYPE_SHIFT);

	memset(rec, 0, sizeof(rec));
	put32(&rec[0], 0);
	put16(&rec[4], s_log.session);
	put16(&rec[6], (u16)s_log.record_seq);
	put32(&rec[16], ticks_to_us(s_log.base_max_gap_ticks));
	put16(&rec[20], (u16)s_log.base_late_count);
	put16(&rec[22], (u16)s_log.base_skip_count);
	put16(&rec[52], flags);
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
	write_blob(rec, 0);
}

static void finish_log(u8 reason)
{
	if (!s_log.active) {
		return;
	}
	s_log.end_reason = reason;
	s_log.end_ms = s_log.session_us / 1000u;
	s_log.active = 0;
	s_log.closing = 1;
	s_log.qcount = 0;
	s_log.summary_pending = 1;
	s_log.tail_pending = 0;
	s_log.close_pending = 0;
	s_log.state = FLIGHT_LOG_ST_CLOSING;
}

static u8 write_record(const u8 *rec, u8 is_data)
{
	if (!s_log.active) {
		return 0;
	}
	return write_blob(rec, is_data);
}

static void serialize(const FlightLogSample *s, u8 *rec, u8 type)
{
	u16 flags = s->flags;
#if C_FLIGHT_ARM_DEBUG
	u32 t0, t1, dt;
#endif

	if (type) {
		flags = (u16)((type << FLIGHT_LOG_TYPE_SHIFT) | flags);
	}
#if C_FLIGHT_ARM_DEBUG
	t0 = clock_time();
#endif
	put32(&rec[0], s->t_ms);
	put16(&rec[4], s_log.session);
	put16(&rec[6], s_log.record_seq);
	put_s32(&rec[8], s->q[0]);
	put_s32(&rec[12], s->q[1]);
	put_s32(&rec[16], s->q[2]);
	put_s32(&rec[20], s->q[3]);
	put_s16(&rec[24], s->gyro[0]);
	put_s16(&rec[26], s->gyro[1]);
	put_s16(&rec[28], s->gyro[2]);
	put_s16(&rec[30], s->acc[0]);
	put_s16(&rec[32], s->acc[1]);
	put_s16(&rec[34], s->acc[2]);
	put_s16(&rec[36], s->yaw_stick);
	put_s16(&rec[38], s->yaw_target);
	put_s16(&rec[40], s->yaw_corr);
	put16(&rec[42], s->thrust);
	put16(&rec[44], s->pwm[0]);
	put16(&rec[46], s->pwm[1]);
	put16(&rec[48], s->pwm[2]);
	put16(&rec[50], s->pwm[3]);
	put16(&rec[52], flags);
#if C_FLIGHT_ARM_DEBUG
	t1 = clock_time();
	dt = (u32)(t1 - t0);
	if (dt > s_log.pack_peak_ticks) {
		s_log.pack_peak_ticks = dt;
	}
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
	dt = (u32)(clock_time() - t1);
	if (dt > s_log.rec_crc_peak_ticks) {
		s_log.rec_crc_peak_ticks = dt;
	}
#else
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
#endif
}

static void start_log(void)
{
	s_log.session_serial++;
	if (!s_log.session_serial) {
		s_log.session_serial = 1;
	}
	s_log.session = s_log.session_serial;
	s_log.record_seq = 0;
	s_log.data_records = 0;
	s_log.tick = 0;
	s_log.dropped = 0;
	s_log.qhead = 0;
	s_log.qtail = 0;
	s_log.qcount = 0;
	s_log.event_pending = 0;
	s_log.max_gap_ticks = 0;
	s_log.late_count = 0;
	s_log.skip_count = 0;
	s_log.flash_peak_ticks = 0;
	s_log.end_reason = 0;
	s_log.session_us = 0;
	s_log.active = 1;
	s_log.closing = 0;
	s_log.summary_pending = 0;
	s_log.tail_pending = 0;
	s_log.close_pending = 0;
	s_log.state = FLIGHT_LOG_REC;
	s_log.pending = 0;
	s_log.last_result = FLIGHT_LOG_REC;
	s_log.start_ms = 0;
	s_log.last_ctrl_tick = clock_time();
	open_page(0);
	write_baseline();              // 启录前基线先落盘，后续 BDT 一并读回
}

void FlightLog_Init(void)
{
	memset(&s_log, 0, sizeof(s_log));
	s_log.page_base = FLIGHT_LOG_BASE;
	s_log.state = FLIGHT_LOG_IDLE;
	s_log.last_result = FLIGHT_LOG_IDLE;
}

// 手动开始：只登记会话并清空 RAM，不碰 Flash；旧趟数据继续保持。
static void ram_log_start(void)
{
	s_log.session_serial++;
	if (!s_log.session_serial) {
		s_log.session_serial = 1;
	}
	s_log.session = s_log.session_serial;
	s_log.active = 1;
	s_log.closing = 0;
	s_log.frozen = 0;
	s_log.need_erase = 0;
	s_log.erase_index = 0;
	s_log.ram_started = 1;
	s_log.state = FLIGHT_LOG_REC;
	s_log.record_seq = 0;
	s_log.data_records = 0;
	s_log.dropped = 0;
	s_log.session_us = 0;
	s_log.last_ctrl_tick = clock_time();
	s_log.tick = 0;
	s_log.max_gap_ticks = 0;
	s_log.late_count = 0;
	s_log.skip_count = 0;
	s_log.flash_peak_ticks = 0;
	s_log.event_pending = 0;
	s_log.summary_pending = 0;
	s_log.tail_pending = 0;
	s_log.close_pending = 0;
	s_log.page_open = 0;
	s_ram_head = 0;
	s_ram_count = 0;
}

// 第二次 K3/急停/失联：冻结采样；真正擦写必须等落地且 PWM 为零。
static void ram_log_freeze(void)
{
	if (!s_log.active && !s_log.ram_started) {
		return;
	}
	s_log.active = 0;
	s_log.ram_started = 0;
	s_log.frozen = 1;
	s_log.closing = 1;
	s_log.need_erase = 1;
	s_log.erase_index = 0;
	s_log.summary_pending = 0;
	s_log.end_reason = 7;              // 手动第二次 K3 冻结
	s_log.end_ms = s_log.session_us / 1000u;
	s_log.state = FLIGHT_LOG_ST_CLOSING;
}
void FlightLog_OnKey(unsigned short key, unsigned char packet_valid,
					 unsigned char can_start, unsigned char blocked)
{
	u8 new_key = (key & FLIGHT_LOG_KEY_START)
				 && !(s_log.key_prev & FLIGHT_LOG_KEY_START);

	s_log.key_prev = key;
#ifndef FLIGHT_LOG_LEGACY_WRITE_TEST
	if (!new_key || !packet_valid) {
		return;
	}
	if (s_log.state == FLIGHT_LOG_ST_CLOSING) {
		s_log.last_result = FLIGHT_LOG_REJECT;
		return;
	}
	if (s_log.active) {
		ram_log_freeze();              // 第二次短按：只冻结，不擦写
		return;
	}
	if (s_log.state == FLIGHT_LOG_IDLE || s_log.state == FLIGHT_LOG_ST_END) {
		ram_log_start();               // 第一次短按：只开始采集
		return;
	}
	s_log.last_result = FLIGHT_LOG_REJECT;
	return;
#else
	if (packet_valid) {
		s_log.link_seen = 1;
		if (!(key & FLIGHT_LOG_KEY_START)) {
			s_log.need_release = 0;
		}
	}
	if (!new_key) {
		return;
	}
	if (s_log.state == FLIGHT_LOG_PREP || s_log.state == FLIGHT_LOG_REC
		|| s_log.state == FLIGHT_LOG_ST_CLOSING || s_log.active) {
		s_log.last_result = FLIGHT_LOG_REJECT;
		return;
	}
	if (s_log.need_release) {
		s_log.last_result = FLIGHT_LOG_REJECT;
		return;
	}
	if (!packet_valid || blocked || !can_start) {
		s_log.state = FLIGHT_LOG_REJECT;
		s_log.last_result = FLIGHT_LOG_REJECT;
		return;
	}
	s_log.pending = 1;
	s_log.state = FLIGHT_LOG_PREP;
#endif
}

void FlightLog_OnLinkDown(void)
{
	// 首次上电从未建立链路不算“重连”；只有链路建立后再丢失才要求先见 0x0004=0。
	if (s_log.link_seen) {
		s_log.need_release = 1;
	}
}

void FlightLog_BeginPrepare(void)
{
	u32 t0;

	if (s_log.state != FLIGHT_LOG_PREP || !s_log.pending) {
		return;
	}
	s_log.pending = 0;
	t0 = clock_time();
	erase_log();
	start_log();
	s_log.prep_ticks = (u32)(clock_time() - t0);
}

unsigned char FlightLog_State(void) { return s_log.state; }

void FlightLog_OnControlTick(void)
{
	u32 now = clock_time();
	u32 gap;

	if (!s_log.last_ctrl_tick) {
		s_log.last_ctrl_tick = now;
		return;
	}
	gap = (u32)(now - s_log.last_ctrl_tick);
	s_log.last_ctrl_tick = now;
	if (s_log.active || s_log.closing) {
		s_log.session_us += ticks_to_us(gap);
		if (gap > s_log.max_gap_ticks) {
			s_log.max_gap_ticks = gap;
		}
		if (gap > FLIGHT_PERIOD_TICKS) {
			s_log.late_count++;
		}
		if (gap > (FLIGHT_PERIOD_TICKS + FLIGHT_PERIOD_TICKS / 2)) {
			s_log.skip_count++;
		}
	} else {
		if (gap > s_log.base_max_gap_ticks) {
			s_log.base_max_gap_ticks = gap;
		}
		if (gap > FLIGHT_PERIOD_TICKS) {
			s_log.base_late_count++;
		}
		if (gap > (FLIGHT_PERIOD_TICKS + FLIGHT_PERIOD_TICKS / 2)) {
			s_log.base_skip_count++;
		}
	}
}

void FlightLog_OnSample(const FlightLogSample *sample)
{
#ifdef FLIGHT_LOG_LEGACY_WRITE_TEST
	FlightLogSample s;

	if (!s_log.active || !sample || s_log.event_pending) {
		return;
	}
	s_log.tick++;
	if ((s_log.tick & 7u) != 0u) {
		return;
	}
	if (s_log.qcount >= 2) {
		s_log.dropped++;
		return;
	}
	s = *sample;
	s.t_ms = s_log.session_us / 1000u;
	s_log.q[s_log.qtail] = s;
	s_log.qtail = (u8)((s_log.qtail + 1) & 1);
	s_log.qcount++;
#else
	FlightRamRec *r;
	u16 tail;

	if (!s_log.active || !sample || s_log.event_pending) {
		return;
	}
	// 只有真正解锁后才采样；K3 准备阶段和落地锁定阶段不产生样本。
	if (!(sample->flags & FLIGHT_LOG_F_ARMED)) {
		return;
	}
	s_log.tick++;
	if ((s_log.tick % FLIGHT_RAM_DIV) != 0u) {
		return;
	}
	if (s_ram_count >= FLIGHT_RAM_N) {
		s_ram_head = (u16)((s_ram_head + 1u) % FLIGHT_RAM_N);
		s_ram_count--;
		s_log.dropped++;
	}
	tail = (u16)((s_ram_head + s_ram_count) % FLIGHT_RAM_N);
	r = &s_ram[tail];
	r->t10 = (u16)(s_log.session_us / 10000u);
	r->yaw_stick = sample->yaw_stick;
	r->yaw_target = sample->yaw_target;
	r->gyro_z = sample->gyro[2];
	r->yaw_corr = sample->yaw_corr;
	r->thrust = sample->thrust;
	r->pwm[0] = sample->pwm[0];
	r->pwm[1] = sample->pwm[1];
	r->pwm[2] = sample->pwm[2];
	r->pwm[3] = sample->pwm[3];
	r->flags = (u8)((sample->flags & FLIGHT_LOG_F_LIMITED) ? 1u : 0u);
	s_ram_count++;
#endif
}
static u8 flush_ram_one(void)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];
	FlightLogSample s;
	const FlightRamRec *r;

	if (!s_ram_count) {
		return 0;
	}
	r = &s_ram[s_ram_head];
	memset(&s, 0, sizeof(s));
	s.t_ms = (u32)r->t10 * 10u;
	s.yaw_stick = r->yaw_stick;
	s.yaw_target = r->yaw_target;
	s.gyro[2] = r->gyro_z;
	s.yaw_corr = r->yaw_corr;
	s.thrust = r->thrust;
	s.pwm[0] = r->pwm[0];
	s.pwm[1] = r->pwm[1];
	s.pwm[2] = r->pwm[2];
	s.pwm[3] = r->pwm[3];
	if (r->flags & 1u) {
		s.flags |= FLIGHT_LOG_F_LIMITED;
	}
	s.flags |= FLIGHT_LOG_F_LOCKED;
	serialize(&s, rec, 0);
	if (!write_blob(rec, 1)) {
		return 0;
	}
	s_ram_head = (u16)((s_ram_head + 1u) % FLIGHT_RAM_N);
	s_ram_count--;
	return 1;
}
void FlightLog_Service(unsigned char budget_ok)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];

	if (!s_log.active && !s_log.closing) {
		return;
	}
	if (!budget_ok) {
		return;                        // 距下一控制拍余量不足，本轮不发起编程
	}

#ifndef FLIGHT_LOG_LEGACY_WRITE_TEST
	// 冻结后先在主循环里逐扇区擦除，再写 baseline；每轮最多一次物理操作。
	if (s_log.need_erase == 1) {
		flash_erase_sector(FLIGHT_LOG_BASE + (u32)s_log.erase_index * FLIGHT_LOG_SECTOR_SIZE);
		s_log.erase_index++;
		if (s_log.erase_index >= FLIGHT_LOG_SECTORS) {
			s_log.need_erase = 2;
		}
		return;
	}
	if (s_log.need_erase == 2) {
		open_page(0);
		write_baseline();
		s_log.need_erase = 0;
		return;
	}
	// 飞行阶段只写 RAM；只有冻结/停机后才批量落盘。
	if (s_ram_count && s_log.frozen) {
		flush_ram_one();
		return;
	}
	if (s_log.active && !s_log.event_pending && !s_log.closing) {
		return;
	}
	if (s_log.frozen && !s_log.event_pending && !s_log.summary_pending
		&& !s_log.tail_pending && !s_log.close_pending) {
		s_log.summary_pending = 1;     // 手动第二次 K3：无停机事件也正常收尾
		return;
	}
#endif
	if (s_log.summary_pending) {
		if (write_meta(FLIGHT_LOG_TYPE_SUMMARY)) {
			s_log.summary_pending = 0;
			s_log.tail_pending = 1;
		}
		return;                        // 本次最多一次物理编程
	}
	if (s_log.tail_pending) {
		if (write_meta(FLIGHT_LOG_TYPE_TAIL)) {
			s_log.tail_pending = 0;
			s_log.close_pending = 1;
		}
		return;
	}
	if (s_log.close_pending) {
		close_page();
		s_log.close_pending = 0;
		s_log.closing = 0;
		s_log.state = FLIGHT_LOG_ST_END;
		return;
	}
	if (s_log.event_pending) {
		serialize(&s_log.event_sample, rec, FLIGHT_LOG_TYPE_EVENT);
		if (write_record(rec, 1)) {
			s_log.event_pending = 0;
			finish_log(s_log.event_code);
		}
		return;
	}
	if (!s_log.qcount) {
		return;
	}
	serialize(&s_log.q[s_log.qhead], rec, 0);
	if (write_record(rec, 1)) {
		s_log.qhead = (u8)((s_log.qhead + 1) & 1);
		s_log.qcount--;
		if (s_log.data_records >= FLIGHT_LOG_MAX_RECORDS) {
			finish_log(5);
		}
	}
}
void FlightLog_OnEvent(unsigned char event, const FlightLogSample *sample)
{
	if (!s_log.active || !sample) {
		return;
	}
	s_log.event_sample = *sample;
	s_log.event_sample.t_ms = s_log.session_us / 1000u;  // 事件用当前会话时间，避免回跳
	s_log.event_sample.flags |= (u16)((u16)event << 8);
	s_log.event_code = event;
	s_log.event_pending = 1;
	s_log.dropped += s_log.qcount;
	s_log.qcount = 0;
#ifndef FLIGHT_LOG_LEGACY_WRITE_TEST
	ram_log_freeze();                  // 急停/失联自动冻结，落地后分批落盘
	s_log.event_pending = 1;
#endif
}

#if C_FLIGHT_ARM_DEBUG
// 诊断专用：把一条 56 字节记录打包成现有格式，末尾写 CRC。
static void bench_make_record(u8 *rec, u16 session, u16 seq, u32 t_ms)
{
	memset(rec, 0, FLIGHT_LOG_REC_SIZE);
	put32(&rec[0], t_ms);
	put16(&rec[4], session);
	put16(&rec[6], seq);
	put16(&rec[52], 0);
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
}

static void bench_write_header(u32 base, u16 seq, u16 session, u16 count,
							   u16 page_crc, u32 total_us, u32 max_us, u32 over)
{
	u8 hdr[32];

	put32(&hdr[0], FLIGHT_LOG_MAGIC);
	put16(&hdr[4], FLIGHT_LOG_VERSION);
	put16(&hdr[6], session);
	put16(&hdr[8], seq);
	put16(&hdr[10], count);
	put16(&hdr[12], page_crc);
	put16(&hdr[14], crc16_update(0xFFFF, hdr, 14));

	// 16..31 为诊断保留区：不改标准头，导出器仍只读前 16 字节。
	put32(&hdr[16], 0x41425453u);       // "STBA"
	put32(&hdr[20], total_us);
	put32(&hdr[24], max_us);
	put16(&hdr[28], (u16)over);
	put16(&hdr[30], crc16_update(0xFFFF, &hdr[16], 14));
	flash_write_page(base, 32, hdr);
}
static char *bench_u32(char *p, u32 v)
{
	char tmp[10];
	int n = 0;

	do {
		tmp[n++] = (char)('0' + v % 10);
		v /= 10;
	} while (v && n < 10);
	while (n) {
		*p++ = tmp[--n];
	}
	return p;
}

static void bench_line(const char *tag, u32 records, u32 writes,
					   u32 total_us, u32 max_us, u32 over)
{
	char buf[72];
	char *p = buf;

	while (*tag) {
		*p++ = *tag++;
	}
	*p++ = ' ';
	*p++ = 'n';
	*p++ = '=';
	p = bench_u32(p, records);
	*p++ = ' ';
	*p++ = 'w';
	*p++ = '=';
	p = bench_u32(p, writes);
	*p++ = ' ';
	*p++ = 't';
	*p++ = '=';
	p = bench_u32(p, total_us);
	*p++ = ' ';
	*p++ = 'm';
	*p++ = 'x';
	*p++ = '=';
	p = bench_u32(p, max_us);
	*p++ = ' ';
	*p++ = 'o';
	*p++ = 'v';
	*p++ = '=';
	p = bench_u32(p, over);
	*p++ = '\n';
	UartDbgSend((unsigned char *)buf, (unsigned char)(p - buf));
}

// K3 只启动实验；之后每个控制拍最多推进一次物理操作。
// 阶段顺序：擦 A、擦 B、A 组24条、A页头、B组6次、B页头。
enum {
	BENCH_IDLE = 0,
	BENCH_ERASE_A,
	BENCH_ERASE_B,
	BENCH_A,
	BENCH_A_HDR,
	BENCH_B,
	BENCH_B_HDR
};

typedef struct {
	u8 active;
	u8 phase;
	u8 idx;
	u16 page_crc;
	u32 base_a;
	u32 base_b;
	u32 sum;
	u32 max;
	u32 over;
	u32 writes;
	u32 records;
	u32 erase_a_us;
	u32 erase_b_us;
} BenchState;

static BenchState b;

void FlightLog_BenchStart(void)
{
	memset(&b, 0, sizeof(b));
	b.active = 1;
	b.phase = BENCH_ERASE_A;
	b.base_a = FLIGHT_LOG_BASE + 10u * FLIGHT_LOG_SECTOR_SIZE;
	b.base_b = FLIGHT_LOG_BASE + 11u * FLIGHT_LOG_SECTOR_SIZE;
	b.page_crc = 0xFFFF;
}

u8 FlightLog_BenchStep(u32 next_tick)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];
	u8 page[FLIGHT_LOG_REC_SIZE * 4];
	u32 t0, t_end, dt, addr;
	u16 seq;

	if (!b.active) {
		return 0;
	}

	switch (b.phase) {
	case BENCH_ERASE_A:
		t0 = clock_time();
		flash_erase_sector(b.base_a);
		b.erase_a_us = (u32)(clock_time() - t0)
					   / (CLOCK_SYS_CLOCK_HZ / 1000000u);
		b.phase = BENCH_ERASE_B;
		break;

	case BENCH_ERASE_B:
		t0 = clock_time();
		flash_erase_sector(b.base_b);
		b.erase_b_us = (u32)(clock_time() - t0)
					   / (CLOCK_SYS_CLOCK_HZ / 1000000u);
		{
			char ebuf[48];
			char *ep = ebuf;

			*ep++ = 'F'; *ep++ = 'B'; *ep++ = 'E';
			*ep++ = ' '; *ep++ = 'e'; *ep++ = 'a'; *ep++ = '=';
			ep = bench_u32(ep, b.erase_a_us);
			*ep++ = ' '; *ep++ = 'e'; *ep++ = 'b'; *ep++ = '=';
			ep = bench_u32(ep, b.erase_b_us);
			*ep++ = '\n';
			UartDbgSend((unsigned char *)ebuf, (unsigned char)(ep - ebuf));
		}
		b.phase = BENCH_A;
		break;

	case BENCH_A:
		t0 = clock_time();
		addr = b.base_a + FLIGHT_LOG_PAGE_HDR
			   + (b.idx / 4u) * 256u + (b.idx % 4u) * FLIGHT_LOG_REC_SIZE;
		bench_make_record(rec, 0xBA00u, (u16)(b.idx + 1u), b.idx * 40u);
		b.page_crc = crc16_update(b.page_crc, rec, FLIGHT_LOG_REC_SIZE);
		flash_write_page(addr, FLIGHT_LOG_REC_SIZE, rec);
		t_end = clock_time();
		dt = (u32)(t_end - t0);
		b.sum += dt;
		if (dt > b.max) b.max = dt;
		if ((s32)(t_end - next_tick) > 0) b.over++;
		b.writes++;
		b.records++;
		b.idx++;
		if (b.idx == 24) {
			bench_line("FBA", b.records, b.writes,
					   b.sum / (CLOCK_SYS_CLOCK_HZ / 1000000u),
					   b.max / (CLOCK_SYS_CLOCK_HZ / 1000000u), b.over);
			b.phase = BENCH_A_HDR;
		}
		break;

	case BENCH_A_HDR:
		bench_write_header(b.base_a, 10, 0xBA00u, 24, b.page_crc,
						   b.sum / (CLOCK_SYS_CLOCK_HZ / 1000000u),
						   b.max / (CLOCK_SYS_CLOCK_HZ / 1000000u), b.over);
		b.sum = 0;
		b.max = 0;
		b.over = 0;
		b.writes = 0;
		b.records = 0;
		b.idx = 0;
		b.page_crc = 0xFFFF;
		b.phase = BENCH_B;
		break;

	case BENCH_B:
		t0 = clock_time();
		for (seq = 0; seq < 4; seq++) {
			u16 n = (u16)(b.idx * 4u + seq + 1u);
			bench_make_record(&page[seq * FLIGHT_LOG_REC_SIZE],
							  0xBB00u, n, n * 40u);
			b.page_crc = crc16_update(b.page_crc,
									  &page[seq * FLIGHT_LOG_REC_SIZE],
									  FLIGHT_LOG_REC_SIZE);
		}
		addr = b.base_b + FLIGHT_LOG_PAGE_HDR + b.idx * 256u;
		flash_write_page(addr, FLIGHT_LOG_REC_SIZE * 4u, page);
		t_end = clock_time();
		dt = (u32)(t_end - t0);
		b.sum += dt;
		if (dt > b.max) b.max = dt;
		if ((s32)(t_end - next_tick) > 0) b.over++;
		b.writes++;
		b.records += 4;
		b.idx++;
		if (b.idx == 6) {
			bench_line("FBB", b.records, b.writes,
					   b.sum / (CLOCK_SYS_CLOCK_HZ / 1000000u),
					   b.max / (CLOCK_SYS_CLOCK_HZ / 1000000u), b.over);
			b.phase = BENCH_B_HDR;
		}
		break;

	case BENCH_B_HDR:
		bench_write_header(b.base_b, 11, 0xBB00u, 24, b.page_crc,
						   b.sum / (CLOCK_SYS_CLOCK_HZ / 1000000u),
						   b.max / (CLOCK_SYS_CLOCK_HZ / 1000000u), b.over);
		b.active = 0;
		b.phase = BENCH_IDLE;
		break;
	}
	return 1;
}
#endif

void FlightLog_Stop(void)
{
	if (s_log.active) {
		s_log.dropped += s_log.qcount;
		finish_log(6);
	}
}

unsigned char FlightLog_Active(void) { return s_log.active; }
unsigned int FlightLog_Dropped(void) { return s_log.dropped; }
#if C_FLIGHT_ARM_DEBUG
unsigned int FlightLog_QueueCount(void) { return s_log.qcount; }
unsigned int FlightLog_Written(void) { return s_log.written; }
unsigned int FlightLog_PhysicalWrites(void) { return s_log.physical_writes; }
unsigned int FlightLog_QueueDrops(void) { return s_log.queue_drops; }
unsigned int FlightLog_FlashPeakUs(void) { return ticks_to_us(s_log.flash_peak_ticks); }
unsigned int FlightLog_PackPeakUs(void) { return ticks_to_us(s_log.pack_peak_ticks); }
unsigned int FlightLog_RecCrcPeakUs(void) { return ticks_to_us(s_log.rec_crc_peak_ticks); }
unsigned int FlightLog_PageCrcPeakUs(void) { return ticks_to_us(s_log.page_crc_peak_ticks); }
#endif
