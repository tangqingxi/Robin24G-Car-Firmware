/*
 * 地面 UART 诊断：10 Hz 文本，复用非阻塞 UART，发送忙时丢帧不影响控制。
 * 只用于台架确认偏航方向和纠偏方向，不参与飞控。
 */
#include <string.h>
#include "GroundDiag.h"
#include "FlightConfig.h"
#include "UserUart.h"

static char s_line[UART_DATA_LEN];
static unsigned char s_have;
static unsigned int s_last_ms;
static int s_stick;
static int s_tgt[3];
static int s_gyro[3];
static int s_corr[3];
static unsigned short s_pwm[4];
static short s_acc_min[3];
static short s_acc_max[3];
static int s_q[4];
static unsigned char s_acc_seen;
static unsigned char s_armed;
static unsigned char s_limited;

static char *d_put_s32(char *p, int v)
{
	unsigned int u;
	char tmp[12];
	int n = 0;

	if (v < 0) {
		*p++ = '-';
		u = (unsigned int)(-v);
	} else {
		u = (unsigned int)v;
	}
	do {
		tmp[n++] = (char)('0' + u % 10u);
		u /= 10u;
	} while (u && n < 12);
	while (n) {
		*p++ = tmp[--n];
	}
	return p;
}

static char *d_put_tag(char *p, const char *tag)
{
	while (*tag) {
		*p++ = *tag++;
	}
	return p;
}

void GroundDiag_Init(void)
{
	s_have = 0;
	s_last_ms = 0;
}

void GroundDiag_OnControl(int yaw_stick, const int tgt[3], const int gyro[3],
						  const int corr[3], const unsigned short pwm[4],
						  const short raw_acc[3], const int q[4],
						  unsigned char armed, unsigned char limited)
{
	int i;

	s_stick = yaw_stick;
	memcpy(s_tgt, tgt, sizeof(s_tgt));
	memcpy(s_gyro, gyro, sizeof(s_gyro));
	memcpy(s_corr, corr, sizeof(s_corr));
	memcpy(s_pwm, pwm, sizeof(s_pwm));
	memcpy(s_q, q, sizeof(s_q));
	for (i = 0; i < 3; i++) {
		if (!s_acc_seen) {
			s_acc_min[i] = raw_acc[i];
			s_acc_max[i] = raw_acc[i];
		} else {
			if (raw_acc[i] < s_acc_min[i]) s_acc_min[i] = raw_acc[i];
			if (raw_acc[i] > s_acc_max[i]) s_acc_max[i] = raw_acc[i];
		}
	}
	s_acc_seen = 1;
	s_armed = armed;
	s_limited = limited;
	s_have = 1;
}

void GroundDiag_Process(void)
{
	char *p;
	unsigned int now = clock_time() / FLIGHT_TICKS_PER_MS;
	int i;

	if (!s_have || (unsigned int)(now - s_last_ms) < 100u) {
		return;
	}
	s_last_ms = now;
	p = s_line;
	p = d_put_tag(p, "YAW y=");
	p = d_put_s32(p, s_stick);
	p = d_put_tag(p, " tr=");
	for (i = 0; i < 3; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, s_tgt[i]);
	}
	p = d_put_tag(p, " gy=");
	for (i = 0; i < 3; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, s_gyro[i]);
	}
	p = d_put_tag(p, " co=");
	for (i = 0; i < 3; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, s_corr[i]);
	}
	p = d_put_tag(p, " p=");
	for (i = 0; i < 4; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, s_pwm[i]);
	}
	p = d_put_tag(p, " q=");
	for (i = 0; i < 4; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, s_q[i]);
	}
	p = d_put_tag(p, " j=");
	for (i = 0; i < 3; i++) {
		if (i) *p++ = ',';
		p = d_put_s32(p, (int)s_acc_max[i] - (int)s_acc_min[i]);
	}
	p = d_put_tag(p, " a=");
	*p++ = (char)('0' + (s_armed ? 1 : 0));
	p = d_put_tag(p, " l=");
	*p++ = (char)('0' + (s_limited ? 1 : 0));
	*p++ = '\n';
	(void)UartTrySend((unsigned char *)s_line, (unsigned char)(p - s_line));
	s_acc_seen = 0;                    // 下一个 100 ms 重新统计原始加速度峰峰值
}
