#include "FlightArmDebug.h"
#include "FlightConfig.h"
#include "MotorMixer.h"
#include "FlightSafety.h"
#include "UserUart.h"

#if C_FLIGHT_ARM_DEBUG

#define ARM_DBG_QDEPTH 4
#define ARM_DBG_LINE UART_DATA_LEN

typedef struct {
	unsigned char inited;
	unsigned char boot_reported;
	unsigned char last_boot_st;
	unsigned char last_boot_att;
	unsigned char last_boot_pwm;

	unsigned char st;
	unsigned int still_ms;
	unsigned char att_ready;
	unsigned char pwm_ready;

	unsigned int rx;
	unsigned int valid;
	unsigned int heartbeat;
	unsigned char got_e2;
	unsigned short last_key;

	unsigned short attempt_prev;
	unsigned int attempt;
	unsigned int current_attempt;
	unsigned char accepted_pending;
	unsigned char first_output_done;
	unsigned char was_armed;
	unsigned char last_reason;

	char q[ARM_DBG_QDEPTH][ARM_DBG_LINE];
	unsigned char qlen[ARM_DBG_QDEPTH];
	unsigned char qhead;
	unsigned char qtail;
	unsigned char qcount;
	unsigned int qdrop;

	unsigned int last_summary;
} ArmDebug;

static ArmDebug d;

static char *p_str(char *p, const char *s)
{
	while (*s) {
		*p++ = *s++;
	}
	return p;
}

static char *p_u32(char *p, unsigned int v)
{
	char tmp[10];
	unsigned char n = 0;

	if (!v) {
		*p++ = '0';
		return p;
	}
	while (v) {
		tmp[n++] = (char)('0' + v % 10);
		v /= 10;
	}
	while (n) {
		*p++ = tmp[--n];
	}
	return p;
}

static char *p_s32(char *p, int v)
{
	if (v < 0) {
		*p++ = '-';
		return p_u32(p, (unsigned int)(-v));
	}
	return p_u32(p, (unsigned int)v);
}

static char *p_hex(char *p, unsigned int v, unsigned char digits)
{
	static const char hex[] = "0123456789ABCDEF";
	int i;

	for (i = digits - 1; i >= 0; i--) {
		*p++ = hex[(v >> (i * 4)) & 0x0F];
	}
	return p;
}

static void q_line(const char *buf, unsigned char len)
{
	if (len > UART_DATA_LEN - 1) {
		len = UART_DATA_LEN - 1;
	}
	if (d.qcount >= ARM_DBG_QDEPTH) {
		// 队列满时丢最旧事件，保留最新故障现场
		d.qhead = (unsigned char)((d.qhead + 1) % ARM_DBG_QDEPTH);
		d.qcount--;
		d.qdrop++;
	}
	{
		unsigned char i;
		unsigned char idx = d.qtail;

		for (i = 0; i < len; i++) {
			d.q[idx][i] = buf[i];
		}
		d.qlen[idx] = len;
		d.qtail = (unsigned char)((d.qtail + 1) % ARM_DBG_QDEPTH);
		d.qcount++;
	}
}

static unsigned int now_ms(void)
{
	return clock_time() / FLIGHT_TICKS_PER_MS;
}

static const char *why_text(unsigned char why)
{
	switch (why) {
	case 1: return "pkt";
	case 2: return "estop";
	case 3: return "thr";
	case 4: return "roll";
	case 5: return "pitch";
	case 6: return "yaw";
	case 7: return "att";
	case 8: return "link";
	default: return "none";
	}
}

void FlightArmDebug_Init(void)
{
	unsigned int i;

	if (d.inited) {
		return;
	}
	for (i = 0; i < sizeof(d); i++) {
		((unsigned char *)&d)[i] = 0;
	}
	d.inited = 1;
}

void FlightArmDebug_OnInit(unsigned char st, unsigned int still_ms,
						   unsigned char att_ready, unsigned char pwm_ready)
{
	char buf[UART_DATA_LEN];
	char *p = buf;

	d.st = st;
	d.still_ms = still_ms;
	d.att_ready = att_ready;
	d.pwm_ready = pwm_ready;
	if (d.boot_reported && d.last_boot_st == st
		&& d.last_boot_att == att_ready && d.last_boot_pwm == pwm_ready) {
		return;
	}
	d.boot_reported = 1;
	d.last_boot_st = st;
	d.last_boot_att = att_ready;
	d.last_boot_pwm = pwm_ready;

	p = p_str(p, "ADB t=");
	p = p_u32(p, now_ms());
	p = p_str(p, " st=");
	p = p_u32(p, st);
	p = p_str(p, " still=");
	p = p_u32(p, still_ms);
	p = p_str(p, " att=");
	p = p_u32(p, att_ready);
	p = p_str(p, " pwm=");
	p = p_u32(p, pwm_ready);
	*p++ = '\n';
	q_line(buf, (unsigned char)(p - buf));
}

void FlightArmDebug_OnPacket(unsigned short thr, unsigned short rot,
							 unsigned short fb, unsigned short lr,
							 unsigned short key)
{
	unsigned char heartbeat = (thr == 0 && rot == 0 && fb == 0 && lr == 0);
	unsigned char valid = !heartbeat
		&& thr >= 0x0400 && thr <= 0x1C00
		&& rot >= 0x0400 && rot <= 0x1C00
		&& fb >= 0x0400 && fb <= 0x1C00
		&& lr >= 0x0400 && lr <= 0x1C00;

	d.rx++;
	if (heartbeat) {
		d.heartbeat++;
	} else if (valid) {
		d.valid++;
	}
	if (key & 0x0002) {
		d.got_e2 = 1;
	}
	d.last_key = key;
}

void FlightArmDebug_OnUnlockAttempt(unsigned short key,
									unsigned int raw_thr,
									const int cmd[4],
									unsigned char packet_valid,
									unsigned char thr_low,
									unsigned char center,
									unsigned char att_ready,
									unsigned char link_ok)
{
	unsigned char new_e2 = (key & 0x0002)
						   && !(d.attempt_prev & 0x0002);
	unsigned char why = 0;
	unsigned char accepted = 0;
	char buf[UART_DATA_LEN];
	char *p = buf;

	d.attempt_prev = key;
	if (!new_e2) {
		return;
	}

	d.attempt++;
	d.current_attempt = d.attempt;
	if (!packet_valid) {
		why = 1;
	} else if (key & 0x0001) {
		why = 2;
	} else if (!thr_low) {
		why = 3;
	} else if (cmd[3] < -MOTOR_ARM_STICK_MAX
			   || cmd[3] > MOTOR_ARM_STICK_MAX) {
		why = 4;
	} else if (cmd[2] < -MOTOR_ARM_STICK_MAX
			   || cmd[2] > MOTOR_ARM_STICK_MAX) {
		why = 5;
	} else if (cmd[1] < -MOTOR_ARM_STICK_MAX
			   || cmd[1] > MOTOR_ARM_STICK_MAX) {
		why = 6;
	} else if (!att_ready) {
		why = 7;
	} else if (!link_ok) {
		why = 8;
	} else {
		accepted = 1;
	}
	if (!center && !why) {
		why = 4;
	}

	p = p_str(p, "ADU t=");
	p = p_u32(p, now_ms());
	p = p_str(p, " n=");
	p = p_u32(p, d.attempt);
	p = p_str(p, " rt=");
	p = p_hex(p, raw_thr, 4);
	p = p_str(p, " cmd=");
	p = p_s32(p, cmd[0]); *p++ = '/';
	p = p_s32(p, cmd[1]); *p++ = '/';
	p = p_s32(p, cmd[2]); *p++ = '/';
	p = p_s32(p, cmd[3]);
	p = p_str(p, " pkt="); p = p_u32(p, packet_valid);
	p = p_str(p, " low="); p = p_u32(p, thr_low);
	p = p_str(p, " ctr="); p = p_u32(p, center);
	p = p_str(p, " att="); p = p_u32(p, att_ready);
	p = p_str(p, " lk="); p = p_u32(p, link_ok);
	p = p_str(p, " ok="); p = p_u32(p, accepted);
	p = p_str(p, " why=");
	p = p_str(p, why_text(why));
	*p++ = '\n';
	q_line(buf, (unsigned char)(p - buf));

	if (accepted) {
		d.accepted_pending = 1;
		d.first_output_done = 0;
		d.was_armed = 1;
	}
}

void FlightArmDebug_OnEstop(void)
{
	char buf[UART_DATA_LEN];
	char *p = buf;
	unsigned short pwm[4];

	FlightSafety_GetPwm(NULL, pwm);
	p = p_str(p, "ADE t=");
	p = p_u32(p, now_ms());
	p = p_str(p, " n=");
	p = p_u32(p, d.current_attempt);
	p = p_str(p, " x=5 pwm=");
	p = p_hex(p, pwm[0], 3); *p++ = '/';
	p = p_hex(p, pwm[1], 3); *p++ = '/';
	p = p_hex(p, pwm[2], 3); *p++ = '/';
	p = p_hex(p, pwm[3], 3);
	*p++ = '\n';
	q_line(buf, (unsigned char)(p - buf));
	d.accepted_pending = 0;
	d.was_armed = 0;
}

void FlightArmDebug_OnStatus(unsigned int still_ms, unsigned char att_ready,
							 unsigned char pwm_ready)
{
	d.still_ms = still_ms;
	d.att_ready = att_ready;
	d.pwm_ready = pwm_ready;
}

void FlightArmDebug_OnControl(unsigned char att_updated, unsigned char ctrl_ok,
							  unsigned char armed, unsigned char reason,
							  const unsigned short pwm[4])
{
	char buf[UART_DATA_LEN];
	char *p = buf;

	if (d.accepted_pending && !d.first_output_done) {
		d.first_output_done = 1;
		d.accepted_pending = 0;
		p = p_str(p, "ADO t=");
		p = p_u32(p, now_ms());
		p = p_str(p, " n=");
		p = p_u32(p, d.current_attempt);
		p = p_str(p, " up="); p = p_u32(p, att_updated);
		p = p_str(p, " ctrl="); p = p_u32(p, ctrl_ok);
		p = p_str(p, " arm="); p = p_u32(p, armed);
		p = p_str(p, " x="); p = p_u32(p, reason);
		p = p_str(p, " pwm=");
		p = p_hex(p, pwm[0], 3); *p++ = '/';
		p = p_hex(p, pwm[1], 3); *p++ = '/';
		p = p_hex(p, pwm[2], 3); *p++ = '/';
		p = p_hex(p, pwm[3], 3);
		*p++ = '\n';
		q_line(buf, (unsigned char)(p - buf));
	}

	if (d.was_armed && !armed && d.current_attempt) {
		p = buf;
		p = p_str(p, "ADL t=");
		p = p_u32(p, now_ms());
		p = p_str(p, " n=");
		p = p_u32(p, d.current_attempt);
		p = p_str(p, " x="); p = p_u32(p, reason);
		p = p_str(p, " pwm=");
		p = p_hex(p, pwm[0], 3); *p++ = '/';
		p = p_hex(p, pwm[1], 3); *p++ = '/';
		p = p_hex(p, pwm[2], 3); *p++ = '/';
		p = p_hex(p, pwm[3], 3);
		*p++ = '\n';
		q_line(buf, (unsigned char)(p - buf));
	}
	d.was_armed = armed;
	d.last_reason = reason;
}

void FlightArmDebug_Process(void)
{
	unsigned int now = clock_time();
	char buf[UART_DATA_LEN];
	char *p;

	if ((unsigned int)(now - d.last_summary) >= CLOCK_SYS_CLOCK_HZ) {
		d.last_summary = now;
		p = buf;
		p = p_str(p, "ADS t=");
		p = p_u32(p, now_ms());
		p = p_str(p, " st="); p = p_u32(p, d.st);
		p = p_str(p, " still="); p = p_u32(p, d.still_ms);
		p = p_str(p, " att="); p = p_u32(p, d.att_ready);
		p = p_str(p, " pwm="); p = p_u32(p, d.pwm_ready);
		p = p_str(p, " rx="); p = p_u32(p, d.rx);
		p = p_str(p, " vld="); p = p_u32(p, d.valid);
		p = p_str(p, " hb="); p = p_u32(p, d.heartbeat);
		p = p_str(p, " e2="); p = p_u32(p, d.got_e2);
		p = p_str(p, " n="); p = p_u32(p, d.attempt);
		p = p_str(p, " arm="); p = p_u32(p, d.was_armed);
		p = p_str(p, " x="); p = p_u32(p, d.last_reason);
		p = p_str(p, " drop="); p = p_u32(p, d.qdrop);
		*p++ = '\n';
		q_line(buf, (unsigned char)(p - buf));
	}

	while (d.qcount) {
		unsigned char idx = d.qhead;

		if (!UartTrySend((unsigned char *)d.q[idx], d.qlen[idx])) {
			return;                 // 串口忙：保留队列，下一主循环再发
		}
		d.qhead = (unsigned char)((d.qhead + 1) % ARM_DBG_QDEPTH);
		d.qcount--;
	}
}

#endif /* C_FLIGHT_ARM_DEBUG */
