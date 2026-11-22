#include <string.h>
#include "FlightLog.h"
#include "FlightConfig.h"
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
	u16 key_prev;
	u32 tick;
	u32 dropped;
	u32 data_records;
	u32 start_ms;
	u32 end_ms;
	u32 last_ctrl_tick;
	u32 max_gap_ticks;
	u32 late_count;
	u32 skip_count;
	u32 flash_peak_ticks;
	u32 prep_ticks;
	u8 end_reason;
	FlightLogSample q[2];
	u8 qhead;
	u8 qtail;
	u8 qcount;
	FlightLogSample event_sample;
} FlightLogState;

static FlightLogState s_log;

static u16 crc16_update(u16 crc, const u8 *p, u32 n)
{
	u32 i;

	while (n--) {
		crc ^= (u16)(*p++) << 8;
		for (i = 0; i < 8; i++) {
			crc = (crc & 0x8000) ? (u16)((crc << 1) ^ 0x1021)
								 : (u16)(crc << 1);
		}
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

static void write_summary(void)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];
	u16 flags = (u16)(FLIGHT_LOG_TYPE_SUMMARY << FLIGHT_LOG_TYPE_SHIFT);

	if (!s_log.page_open) {
		return;
	}
	if (s_log.page_count >= FLIGHT_LOG_REC_PER_SEC) {
		close_page();
		if (s_log.page_seq + 1 >= FLIGHT_LOG_SECTORS) {
			return;
		}
		open_page((u16)(s_log.page_seq + 1));
	}
	memset(rec, 0, sizeof(rec));
	put32(&rec[0], now_ms());
	put16(&rec[4], s_log.session);
	put16(&rec[6], (u16)s_log.record_seq);
	put32(&rec[8], s_log.start_ms);
	put32(&rec[12], now_ms());
	put32(&rec[16], ticks_to_us(s_log.max_gap_ticks));
	put16(&rec[20], (u16)s_log.late_count);
	put16(&rec[22], (u16)s_log.skip_count);
	put16(&rec[24], (u16)ticks_to_us(s_log.flash_peak_ticks));
	put16(&rec[26], (u16)s_log.prep_ticks / FLIGHT_TICKS_PER_MS);
	put16(&rec[28], (u16)s_log.data_records);
	put16(&rec[30], (u16)s_log.dropped);
	rec[32] = s_log.end_reason;
	put16(&rec[52], flags);
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
	{
		u32 phys = s_log.page_count / FLIGHT_LOG_REC_PER_PHYS;
		u32 slot = s_log.page_count % FLIGHT_LOG_REC_PER_PHYS;
		u32 addr = s_log.page_base + FLIGHT_LOG_PAGE_HDR
				   + phys * 256u + slot * FLIGHT_LOG_REC_SIZE;

		flash_write_page(addr, FLIGHT_LOG_REC_SIZE, rec);
	}
	s_log.page_crc = crc16_update(s_log.page_crc, rec, FLIGHT_LOG_REC_SIZE);
	s_log.page_count++;
	s_log.record_seq++;
}

static void finish_log(u8 reason)
{
	if (!s_log.active) {
		return;
	}
	s_log.end_reason = reason;
	s_log.end_ms = now_ms();
	write_summary();
	close_page();
	s_log.active = 0;
	s_log.state = FLIGHT_LOG_ST_END;
	s_log.qcount = 0;
}

static void write_record(const u8 *rec, u8 is_data)
{
	u32 phys, slot, addr;

	if (!s_log.active) {
		return;
	}
	if (s_log.page_count >= FLIGHT_LOG_REC_PER_SEC) {
		close_page();
		if (s_log.page_seq + 1 >= FLIGHT_LOG_SECTORS) {
			finish_log(5);
			return;
		}
		open_page((u16)(s_log.page_seq + 1));
	}
	phys = s_log.page_count / FLIGHT_LOG_REC_PER_PHYS;
	slot = s_log.page_count % FLIGHT_LOG_REC_PER_PHYS;
	addr = s_log.page_base + FLIGHT_LOG_PAGE_HDR
		   + phys * 256u + slot * FLIGHT_LOG_REC_SIZE;
	{
		u32 t0 = clock_time();
		u32 dt;

		flash_write_page(addr, FLIGHT_LOG_REC_SIZE, (u8 *)rec);
		dt = (u32)(clock_time() - t0);
		if (dt > s_log.flash_peak_ticks) {
			s_log.flash_peak_ticks = dt;
		}
	}
	s_log.page_crc = crc16_update(s_log.page_crc, rec, FLIGHT_LOG_REC_SIZE);
	s_log.page_count++;
	s_log.record_seq++;
	if (is_data) {
		s_log.data_records++;
	}
}

static void serialize(const FlightLogSample *s, u8 *rec, u8 type)
{
	u16 flags = s->flags;

	if (type) {
		flags = (u16)((type << FLIGHT_LOG_TYPE_SHIFT) | flags);
	}
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
	put16(&rec[54], crc16_update(0xFFFF, rec, 54));
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
	s_log.active = 1;
	s_log.state = FLIGHT_LOG_REC;
	s_log.pending = 0;
	s_log.last_result = FLIGHT_LOG_REC;
	s_log.start_ms = now_ms();
	s_log.last_ctrl_tick = clock_time();
	open_page(0);
}

void FlightLog_Init(void)
{
	memset(&s_log, 0, sizeof(s_log));
	s_log.page_base = FLIGHT_LOG_BASE;
	s_log.state = FLIGHT_LOG_IDLE;
	s_log.last_result = FLIGHT_LOG_IDLE;
}

void FlightLog_OnKey(unsigned short key, unsigned char packet_valid,
					 unsigned char can_start, unsigned char blocked)
{
	u8 new_key = (key & FLIGHT_LOG_KEY_START)
				 && !(s_log.key_prev & FLIGHT_LOG_KEY_START);

	s_log.key_prev = key;
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
		|| s_log.active) {
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
	s_log.prep_ticks = (u32)(clock_time() - t0);
	start_log();
}

unsigned char FlightLog_State(void) { return s_log.state; }

void FlightLog_OnControlTick(void)
{
	u32 now;

	if (!s_log.active) {
		return;
	}
	now = clock_time();
	if (s_log.last_ctrl_tick) {
		u32 gap = (u32)(now - s_log.last_ctrl_tick);

		if (gap > s_log.max_gap_ticks) {
			s_log.max_gap_ticks = gap;
		}
		if (gap > FLIGHT_PERIOD_TICKS) {
			s_log.late_count++;
		}
		if (gap > (FLIGHT_PERIOD_TICKS + FLIGHT_PERIOD_TICKS / 2)) {
			s_log.skip_count++;
		}
	}
	s_log.last_ctrl_tick = now;
}

void FlightLog_OnSample(const FlightLogSample *sample)
{
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
	s_log.q[s_log.qtail] = *sample;
	s_log.qtail = (u8)((s_log.qtail + 1) & 1);
	s_log.qcount++;
}

void FlightLog_Service(void)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];

	if (!s_log.active) {
		return;
	}
	if (s_log.event_pending) {
		serialize(&s_log.event_sample, rec, FLIGHT_LOG_TYPE_EVENT);
		write_record(rec, 1);
		s_log.event_pending = 0;
		finish_log(s_log.event_code);
		return;
	}
	if (!s_log.qcount) {
		return;
	}
	serialize(&s_log.q[s_log.qhead], rec, 0);
	s_log.qhead = (u8)((s_log.qhead + 1) & 1);
	s_log.qcount--;
	write_record(rec, 1);
	if (s_log.data_records >= FLIGHT_LOG_MAX_RECORDS) {
		finish_log(5);
	}
}

void FlightLog_OnEvent(unsigned char event, const FlightLogSample *sample)
{
	if (!s_log.active || !sample) {
		return;
	}
	s_log.event_sample = *sample;
	s_log.event_sample.flags |= (u16)((u16)event << 8);
	s_log.event_code = event;
	s_log.event_pending = 1;
	s_log.dropped += s_log.qcount;
	s_log.qcount = 0;
}

void FlightLog_Stop(void)
{
	if (s_log.active) {
		s_log.dropped += s_log.qcount;
		finish_log(6);
	}
}

unsigned char FlightLog_Active(void) { return s_log.active; }
unsigned int FlightLog_Dropped(void) { return s_log.dropped; }
