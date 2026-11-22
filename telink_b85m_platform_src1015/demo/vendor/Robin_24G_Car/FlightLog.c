#include "FlightLog.h"
#ifdef FLIGHT_LOG_HOST_TEST
void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf);
void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf);
void flash_erase_sector(unsigned int addr);
#else
#include "flash.h"
#endif

#define FLIGHT_LOG_MAGIC 0x4C4F4731u   // 页头 "1GOL"
#define FLIGHT_LOG_HDR_CRC_OFF 14u

typedef struct {
	u32 page_base;
	u16 page_seq;
	u16 page_count;
	u16 page_crc;
	u16 record_seq;
	u16 session;
	u8 active;
	u8 page_open;
	u32 tick;
	u32 dropped;
	FlightLogSample q[4];
	u8 qhead;
	u8 qtail;
	u8 qcount;
	u8 event_pending;
	u16 event_code;
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

static void put_s32(u8 *p, s32 v)
{
	put32(p, (u32)v);
}

static void put_s16(u8 *p, s16 v)
{
	put16(p, (u16)v);
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

static void write_record(const u8 *rec)
{
	u32 addr;

	if (!s_log.active) {
		return;
	}
	if (s_log.record_seq >= FLIGHT_LOG_MAX_RECORDS) {
		close_page();
		s_log.active = 0;
		s_log.dropped++;
		return;
	}
	if (s_log.page_count >= (FLIGHT_LOG_SECTOR_SIZE - FLIGHT_LOG_PAGE_HDR)
							 / FLIGHT_LOG_REC_SIZE) {
		close_page();
		if (s_log.page_seq + 1 >= FLIGHT_LOG_SECTORS) {
			s_log.active = 0;
			s_log.dropped++;
			return;
		}
		open_page((u16)(s_log.page_seq + 1));
	}
	addr = s_log.page_base + FLIGHT_LOG_PAGE_HDR
		   + (u32)s_log.page_count * FLIGHT_LOG_REC_SIZE;
	flash_write_page(addr, FLIGHT_LOG_REC_SIZE, (u8 *)rec);
	s_log.page_crc = crc16_update(s_log.page_crc, rec, FLIGHT_LOG_REC_SIZE);
	s_log.page_count++;
	s_log.record_seq++;
}

static void erase_log(void)
{
	u32 i;

	for (i = 0; i < FLIGHT_LOG_SECTORS; i++) {
		flash_erase_sector(FLIGHT_LOG_BASE + i * FLIGHT_LOG_SECTOR_SIZE);
	}
}

static void start_log(u16 session)
{
	erase_log();
	s_log.session = session;
	s_log.record_seq = 0;
	s_log.tick = 0;
	s_log.dropped = 0;
	s_log.qhead = 0;
	s_log.qtail = 0;
	s_log.qcount = 0;
	s_log.event_pending = 0;
	s_log.active = 1;
	open_page(0);
}

void FlightLog_Init(void)
{
	s_log.active = 0;
	s_log.page_open = 0;
	s_log.page_base = FLIGHT_LOG_BASE;
	s_log.session = 0;
	s_log.record_seq = 0;
	s_log.tick = 0;
	s_log.dropped = 0;
	s_log.qhead = 0;
	s_log.qtail = 0;
	s_log.qcount = 0;
	s_log.event_pending = 0;
}

void FlightLog_PollCommand(unsigned char locked)
{
	u32 cmd = 0;

	flash_read_page(FLIGHT_LOG_CMD_ADDR, 4, (u8 *)&cmd);
	if (cmd != FLIGHT_LOG_CMD_MAGIC || !locked) {
		return;
	}
	// 外部命令只在四路锁定、PWM 已清零时执行擦除。
	start_log(1);
	flash_erase_sector(FLIGHT_LOG_CMD_ADDR);
}

static void serialize(const FlightLogSample *s, u8 *rec)
{
	u16 flags = s->flags;

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

void FlightLog_OnSample(const FlightLogSample *sample)
{
	if (!s_log.active || !sample || s_log.event_pending) {
		return;
	}
	s_log.tick++;
	if ((s_log.tick & 7u) != 0u) {
		return;
	}
	if (s_log.qcount >= 4) {
		s_log.dropped++;
		return;
	}
	s_log.q[s_log.qtail] = *sample;
	s_log.qtail = (u8)((s_log.qtail + 1) & 3);
	s_log.qcount++;
}

// 服务路径每次最多写一条 56B 记录；控制拍只做快照入队，不碰 Flash。
void FlightLog_Service(void)
{
	u8 rec[FLIGHT_LOG_REC_SIZE];

	if (!s_log.active) {
		return;
	}
	if (s_log.event_pending) {
		serialize(&s_log.event_sample, rec);
		write_record(rec);
		s_log.event_pending = 0;
		close_page();
		s_log.active = 0;
		return;
	}
	if (!s_log.qcount) {
		return;
	}
	serialize(&s_log.q[s_log.qhead], rec);
	s_log.qhead = (u8)((s_log.qhead + 1) & 3);
	s_log.qcount--;
	write_record(rec);
}

void FlightLog_OnEvent(unsigned char event, const FlightLogSample *sample)
{
	if (!s_log.active || !sample) {
		return;
	}
	s_log.event_sample = *sample;
	s_log.event_sample.flags |= (u16)((u16)event << 8);
	s_log.event_pending = 1;
	s_log.qcount = 0;               // 停机事件优先，丢弃尚未服务的普通样本
}

void FlightLog_Stop(void)
{
	if (s_log.active) {
		close_page();
		s_log.active = 0;
	}
}

unsigned char FlightLog_Active(void) { return s_log.active; }
unsigned int FlightLog_Dropped(void) { return s_log.dropped; }
