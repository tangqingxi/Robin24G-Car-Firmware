#include "FlightTelemetry.h"
#include <string.h>
#include "UserUart.h"
#include "UserRxdFunc.h"

#define FLIGHT_TX_MARGIN_US        100
#define FLIGHT_OBS_COST_DEFAULT_US 100
#define FLIGHT_STAT_COST_DEFAULT_US 300

extern volatile unsigned char g_rf_irq_meas_en;
extern volatile unsigned int g_rf_irq_sum_ticks;
extern volatile unsigned int g_rf_irq_max_ticks;
extern volatile unsigned int g_rf_irq_cnt;

static char *t_put_str(char *p, const char *s)
{
	while (*s) {
		*p++ = *s++;
	}
	return p;
}

static char *t_put_u32(char *p, u32 v)
{
	char tmp[10];
	u8 n = 0;

	if (v == 0) {
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

static char *t_put_hex(char *p, u32 v, u8 digits)
{
	static const char hex[] = "0123456789ABCDEF";
	int i;

	for (i = digits - 1; i >= 0; i--) {
		*p++ = hex[(v >> (i * 4)) & 0x0F];
	}
	return p;
}

static char *t_put_q15(char *p, s32 v)
{
	u32 raw;

	if (v > 32767) v = 32767;
	if (v < -32768) v = -32768;
	raw = (u32)v & 0xFFFFu;
	return t_put_hex(p, raw, 4);
}

static u32 t_cap(u32 v, u32 max)
{
	return v > max ? max : v;
}

static u32 t_ticks_to_us(u32 ticks)
{
	return (ticks + FLIGHT_TICKS_PER_US / 2) / FLIGHT_TICKS_PER_US;
}

static void t_add(u32 *sum, u32 *max, u32 us)
{
	*sum += us;
	if (us > *max) {
		*max = us;
	}
}

void FlightTelemetry_Init(FlightTelemetry *t, u32 now)
{
	int i;

	for (i = 0; i < (int)(sizeof(*t) / sizeof(u32)); i++) {
		((u32 *)t)[i] = 0;
	}
	t->tx_tick = now + FLIGHT_REPORT_TICKS;
	t->stat_tick = now + CLOCK_SYS_CLOCK_HZ;
	t->prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	L400Diag_Init(&t->diag, CLOCK_SYS_CLOCK_HZ * 2, CLOCK_SYS_CLOCK_HZ * 10);
	L400DiagTx_Init(&t->diag_tx);
	t->diag_ctrl_enable = 1;

	// 首次发送的保守预算
	t->obs_cost_us = FLIGHT_OBS_COST_DEFAULT_US;
	t->stat_cost_us = FLIGHT_STAT_COST_DEFAULT_US;
}

void FlightTelemetry_OnSample(FlightTelemetry *t) { t->sample++; }
void FlightTelemetry_OnFail(FlightTelemetry *t) { t->fail++; }
void FlightTelemetry_OnEuler(FlightTelemetry *t) { t->euler_ok++; }
void FlightTelemetry_OnTick(FlightTelemetry *t) { t->tick_cnt++; }
void FlightTelemetry_OnMiss(FlightTelemetry *t) { t->miss++; }
void FlightTelemetry_OnSkip(FlightTelemetry *t, u32 count) { t->skip += count; }
void FlightTelemetry_OnLate(FlightTelemetry *t, u32 us) { t_add(&t->late_sum, &t->late_max, us); }
void FlightTelemetry_OnExec(FlightTelemetry *t, u32 us) { t_add(&t->exec_sum, &t->exec_max, us); }
void FlightTelemetry_OnRx(FlightTelemetry *t, u32 us) { t_add(&t->rx_sum, &t->rx_max, us); t->rx_cnt++; }
void FlightTelemetry_OnGap(FlightTelemetry *t, u32 ticks)
{
	if (ticks > t->gap_max_tick) t->gap_max_tick = ticks;
}
void FlightTelemetry_OnPrep(FlightTelemetry *t, u32 us) { t_add(&t->prep_sum, &t->prep_max, us); t->prep_cnt++; }
void FlightTelemetry_OnEul(FlightTelemetry *t, u32 us) { t_add(&t->eul_sum, &t->eul_max, us); t->eul_cnt++; }
void FlightTelemetry_OnCtrl(FlightTelemetry *t, u32 ticks)
{
	u32 us = ticks / FLIGHT_TICKS_PER_US;

	if (us > t->ctrl_max_us) t->ctrl_max_us = us;
	if (us > t->ctrl_peak_us) t->ctrl_peak_us = us;
	if (ticks > FLIGHT_PERIOD_TICKS) t->ctrl_miss++;
}

u8 FlightTelemetry_DiagActive(const FlightTelemetry *t) { return t->diag_active; }
u8 FlightTelemetry_DiagEnabled(const FlightTelemetry *t) { return t->diag_enabled; }
u8 FlightTelemetry_CtrlEnable(const FlightTelemetry *t) { return t->diag_ctrl_enable; }

void FlightTelemetry_RecordRfLoop(FlightTelemetry *t, u32 ticks)
{
	if (!t->diag_active) return;
	t->diag_rf_sum_ticks += ticks;
	if (ticks > t->diag_rf_max_ticks) t->diag_rf_max_ticks = ticks;
	t->diag_rf_cnt++;
}

// A/B 特技键入口；键沿只起一次，条件在状态机 2s 有效后判断。
void FlightTelemetry_OnKey(FlightTelemetry *t, u16 key, u32 now)
{
	if ((key & RfSendKey_StuntShow)
		&& !(t->diag_key_prev & RfSendKey_StuntShow)
		&& !t->diag_active && t->diag_tx.count == 0) {
		t->diag_enabled = 1;
		L400Diag_Init(&t->diag, CLOCK_SYS_CLOCK_HZ * 2, CLOCK_SYS_CLOCK_HZ * 10);
		L400DiagTx_Init(&t->diag_tx);
		t->diag_ctrl_enable = 1;
		t->diag_have_a = 0;
		t->diag_rf_sum_ticks = 0;
		t->diag_rf_max_ticks = 0;
		t->diag_rf_cnt = 0;
		g_rf_irq_meas_en = 0;
	}
	t->diag_key_prev = key;
}

static void t_reset_window(FlightTelemetry *t)
{
	t->ctrl_max_us = 0;
	t->gap_max_tick = 0;
	t->late_sum = 0;
	t->late_max = 0;
	t->exec_sum = 0;
	t->exec_max = 0;
	t->rx_sum = 0;
	t->rx_max = 0;
	t->tx_sum = 0;
	t->tx_max = 0;
	t->tick_cnt = 0;
	t->rx_cnt = 0;
	t->tx_cnt = 0;
	t->window_reset = 1;
}

static void t_reset_diag(FlightTelemetry *t)
{
	t->miss = 0;
	t->ctrl_miss = 0;
	t->skip = 0;
	t->ctrl_max_us = 0;
	t->late_sum = 0;
	t->late_max = 0;
	t->exec_sum = 0;
	t->exec_max = 0;
	t->prep_sum = 0;
	t->prep_max = 0;
	t->prep_cnt = 0;
	t->eul_sum = 0;
	t->eul_max = 0;
	t->eul_cnt = 0;
	t->rx_sum = 0;
	t->rx_max = 0;
	t->tx_sum = 0;
	t->tx_max = 0;
	t->defer_budget = 0;
	t->defer_busy = 0;
	t->tx_diag_due = 0;
	t->tick_cnt = 0;
	t->rx_cnt = 0;
	t->tx_cnt = 0;
	t->gap_max_tick = 0;
	t->diag_rf_sum_ticks = 0;
	t->diag_rf_max_ticks = 0;
	t->diag_rf_cnt = 0;
	t->window_reset = 1;
	g_rf_irq_sum_ticks = 0;
	g_rf_irq_max_ticks = 0;
	g_rf_irq_cnt = 0;
}

static void t_take_diag(FlightTelemetry *t, FlightDiagResult *d)
{
	u32 n = t->tick_cnt;

	d->late_avg = n ? t->late_sum / n : 0;
	d->late_max = t->late_max;
	d->exec_avg = n ? t->exec_sum / n : 0;
	d->exec_max = t->exec_max;
	d->cmiss = t->ctrl_miss;
	d->skip = t->skip;
	d->rfirq_avg = g_rf_irq_cnt
		? t_ticks_to_us(g_rf_irq_sum_ticks / g_rf_irq_cnt) : 0;
	d->rfirq_max = t_ticks_to_us(g_rf_irq_max_ticks);
	d->rf_avg = t->diag_rf_cnt
		? t_ticks_to_us(t->diag_rf_sum_ticks / t->diag_rf_cnt) : 0;
	d->rf_max = t_ticks_to_us(t->diag_rf_max_ticks);
	d->app_avg = t->rx_cnt ? t->rx_sum / t->rx_cnt : 0;
	d->app_max = t->rx_max;
}

static void t_queue_seg(FlightTelemetry *t, char seg, const FlightDiagResult *d)
{
	char buf[L400_DIAG_LINE_LEN];
	char *q = buf;

	q = t_put_str(q, "L400D ");
	*q++ = seg;
	q = t_put_str(q, " late=");
	q = t_put_u32(q, d->late_avg);
	*q++ = '/';
	q = t_put_u32(q, d->late_max);
	q = t_put_str(q, " exec=");
	q = t_put_u32(q, d->exec_avg);
	*q++ = '/';
	q = t_put_u32(q, d->exec_max);
	q = t_put_str(q, " cmiss=");
	q = t_put_u32(q, d->cmiss);
	q = t_put_str(q, " skip=");
	q = t_put_u32(q, d->skip);
	q = t_put_str(q, " rfirq=");
	q = t_put_u32(q, d->rfirq_avg);
	*q++ = '/';
	q = t_put_u32(q, d->rfirq_max);
	q = t_put_str(q, " rf=");
	q = t_put_u32(q, d->rf_avg);
	*q++ = '/';
	q = t_put_u32(q, d->rf_max);
	q = t_put_str(q, " app=");
	q = t_put_u32(q, d->app_avg);
	*q++ = '/';
	q = t_put_u32(q, d->app_max);
	*q++ = '\n';
	L400DiagTx_Enqueue(&t->diag_tx, buf, (u8)(q - buf));
}

static void t_queue_abort(FlightTelemetry *t)
{
	char buf[L400_DIAG_LINE_LEN];
	char *q = buf;

	q = t_put_str(q, "L400D ABORT seg=");
	*q++ = t->diag.abort_seg ? 'B' : 'A';
	q = t_put_str(q, " reason=");
	q = t_put_str(q, t->diag.abort_reason == L400_DIAG_REASON_CMD ? "cmd" : "link");
	q = t_put_str(q, " haveA=");
	q = t_put_u32(q, t->diag_have_a);
	*q++ = '\n';
	L400DiagTx_Enqueue(&t->diag_tx, buf, (u8)(q - buf));
}

static u8 t_send_diag_line(const char *s, u8 len)
{
	return UartTrySend((unsigned char *)s, len);
}

static void t_diag_flush(FlightTelemetry *t, u32 now)
{
	L400DiagTx_Flush(&t->diag_tx, t_send_diag_line);
	if (t->diag_active && t->diag_tx.count == 0
		&& (t->diag.state == L400_DIAG_STATE_DONE
			|| t->diag.state == L400_DIAG_STATE_ABORT)) {
		t->diag_active = 0;
		t->diag_enabled = 0;
		t->tx_tick = now + FLIGHT_REPORT_TICKS;
		t->stat_tick = now + CLOCK_SYS_CLOCK_HZ;
		t->prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	}
}

static void t_diag_poll(FlightTelemetry *t, const FlightTelemetrySnapshot *s)
{
	u8 valid = (s->att_ready && s->link && s->cmd_ok) ? 1 : 0;
	u8 reason = !s->link ? L400_DIAG_REASON_LINK
		: (!s->cmd_ok ? L400_DIAG_REASON_CMD : L400_DIAG_REASON_NONE);
	unsigned int ev;

	if (!t->diag_enabled
		|| (t->diag.state != L400_DIAG_STATE_WAIT
			&& t->diag.state != L400_DIAG_STATE_A
			&& t->diag.state != L400_DIAG_STATE_B)) {
		return;
	}
	if (!t->diag_active && !t->diag.valid_seen && !valid) {
		return;
	}
	ev = L400Diag_Poll(&t->diag, s->now, valid, reason);

	if (ev & L400_DIAG_EV_START_A) {
		t->diag_active = 1;
		t->diag_ctrl_enable = 0;
		t->diag_have_a = 0;
		t_reset_diag(t);
		g_rf_irq_meas_en = 1;
	}
	if (ev & L400_DIAG_EV_A_DONE) {
		t_take_diag(t, &t->diag_a);
		t->diag_have_a = 1;
		t->diag_ctrl_enable = 1;
		t_reset_diag(t);
	}
	if (ev & L400_DIAG_EV_B_DONE) {
		t_take_diag(t, &t->diag_b);
		g_rf_irq_meas_en = 0;
		t_queue_seg(t, 'A', &t->diag_a);
		t_queue_seg(t, 'B', &t->diag_b);
		t->diag_ctrl_enable = 1;
		t_reset_diag(t);
	}
	if (ev & L400_DIAG_EV_ABORT) {
		g_rf_irq_meas_en = 0;
		if (t->diag_have_a) {
			t_queue_seg(t, 'A', &t->diag_a);
		}
		t_queue_abort(t);
		t->diag_ctrl_enable = 1;
		t_reset_diag(t);
	}
}

static u8 t_send_obs(FlightTelemetry *t, const FlightTelemetrySnapshot *s)
{
	char buf[UART_DATA_LEN];
	char *q = buf;
	u8 flags = (u8)((s->att_ready ? 0x01 : 0)
					| (s->link ? 0x02 : 0)
					| (s->cmd_ok ? 0x04 : 0)
					| (s->ctrl_ok ? 0x08 : 0));

	q = t_put_str(q, "FCQ f=");
	q = t_put_hex(q, flags, 1);
	q = t_put_str(q, " a=");
	q = t_put_hex(q, t_cap(s->link_age_ms, 0xFFFF), 4);
	q = t_put_str(q, " v=");
	q = t_put_hex(q, t_cap(s->pkt_age_ms, 0xFFFF), 4);
	q = t_put_str(q, " k=");
	q = t_put_hex(q, t_cap(s->gap_ms, 0xFFFF), 4);
	q = t_put_str(q, " q=");
	q = t_put_q15(q, s->q[0]); *q++ = '/';
	q = t_put_q15(q, s->q[1]); *q++ = '/';
	q = t_put_q15(q, s->q[2]); *q++ = '/';
	q = t_put_q15(q, s->q[3]);
	q = t_put_str(q, " r=");
	q = t_put_hex(q, (u32)s->rate[0] & 0xFFFFF, 5); *q++ = '/';
	q = t_put_hex(q, (u32)s->rate[1] & 0xFFFFF, 5); *q++ = '/';
	q = t_put_hex(q, (u32)s->rate[2] & 0xFFFFF, 5);
	q = t_put_str(q, " c=");
	q = t_put_hex(q, (u32)s->corr[0] & 0xFFFF, 4); *q++ = '/';
	q = t_put_hex(q, (u32)s->corr[1] & 0xFFFF, 4); *q++ = '/';
	q = t_put_hex(q, (u32)s->corr[2] & 0xFFFF, 4);
	q = t_put_str(q, " m=");
	q = t_put_hex(q, s->armed, 1);
	q = t_put_str(q, " x=");
	q = t_put_hex(q, s->reason, 1);
	q = t_put_str(q, " pwm=");
	q = t_put_hex(q, s->pwm[0], 3); *q++ = '/';
	q = t_put_hex(q, s->pwm[1], 3); *q++ = '/';
	q = t_put_hex(q, s->pwm[2], 3); *q++ = '/';
	q = t_put_hex(q, s->pwm[3], 3); *q++ = '\n';

	if (UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		t->tx_tick = s->now + FLIGHT_REPORT_TICKS;
		t->window_reset = 1;
		return 1;
	}
	return 0;
}

static u8 t_send_stats(FlightTelemetry *t, u32 now, const FlightTelemetrySnapshot *s)
{
	char buf[UART_DATA_LEN];
	char *q = buf;
	u32 n = t->tick_cnt;
	u32 late = n ? t->late_sum / n : 0;
	u32 exec = n ? t->exec_sum / n : 0;
	u32 rx = t->rx_cnt ? t->rx_sum / t->rx_cnt : 0;
	u32 tx = t->tx_cnt ? t->tx_sum / t->tx_cnt : 0;

	q = t_put_str(q, "FCS bad=");
	q = t_put_hex(q, t_cap(s->bad, 0xFFFFu), 4);
	q = t_put_str(q, " n=");
	q = t_put_hex(q, t_cap(t->sample, 0xFFFFFFu), 6);
	q = t_put_str(q, " f=");
	q = t_put_hex(q, t->fail, 4);
	q = t_put_str(q, " eul=");
	q = t_put_hex(q, t->euler_ok, 6);
	q = t_put_str(q, " miss=");
	q = t_put_hex(q, t->miss, 6);
	q = t_put_str(q, " cmiss=");
	q = t_put_hex(q, t->ctrl_miss, 6);
	q = t_put_str(q, " skip=");
	q = t_put_hex(q, t->skip, 6);
	q = t_put_str(q, " txd=");
	q = t_put_hex(q, t->txd, 6);
	q = t_put_str(q, " ctl=");
	q = t_put_hex(q, t->ctrl_max_us, 4); *q++ = '/';
	q = t_put_hex(q, t->ctrl_peak_us, 4);
	q = t_put_str(q, " late=");
	q = t_put_hex(q, late, 4); *q++ = '/';
	q = t_put_hex(q, t->late_max, 4);
	q = t_put_str(q, " exec=");
	q = t_put_hex(q, exec, 4); *q++ = '/';
	q = t_put_hex(q, t->exec_max, 4);
	q = t_put_str(q, " rx=");
	q = t_put_hex(q, rx, 4); *q++ = '/';
	q = t_put_hex(q, t->rx_max, 4);
	q = t_put_str(q, " tx=");
	q = t_put_hex(q, tx, 4); *q++ = '/';
	q = t_put_hex(q, t->tx_max, 4);
	*q++ = '\n';

	if (UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		t->stat_tick = now + CLOCK_SYS_CLOCK_HZ;
		t->tx_tick = now + FLIGHT_REPORT_TICKS;
		t_reset_window(t);
		return 1;
	}
	return 0;
}

static void t_send_txdiag(FlightTelemetry *t, u32 now)
{
	char buf[48];
	char *q = buf;

	if (!t->tx_diag_due || (s32)(now - t->tx_diag_tick) < 0) return;
	q = t_put_str(q, "L400T budget=");
	q = t_put_u32(q, t->defer_budget);
	q = t_put_str(q, " busy=");
	q = t_put_u32(q, t->defer_busy);
	*q++ = '\n';
	if (UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		t->tx_diag_due = 0;
		t->tx_diag_tick = now + CLOCK_SYS_CLOCK_HZ;
	}
}

static void t_send_prep(FlightTelemetry *t, u32 now)
{
	char buf[72];
	char *q = buf;
	u32 prep = t->prep_cnt ? t->prep_sum / t->prep_cnt : 0;
	u32 eul = t->eul_cnt ? t->eul_sum / t->eul_cnt : 0;

	if ((s32)(now - t->prep_tick) < 0) return;
	q = t_put_str(q, "L400P prep=");
	q = t_put_u32(q, prep); *q++ = '/';
	q = t_put_u32(q, t->prep_max);
	q = t_put_str(q, " eul=");
	q = t_put_u32(q, eul); *q++ = '/';
	q = t_put_u32(q, t->eul_max);
	q = t_put_str(q, " n=");
	q = t_put_u32(q, t->prep_cnt);
	q = t_put_str(q, " e=");
	q = t_put_u32(q, t->eul_cnt);
	*q++ = '\n';
	if (UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		t->prep_sum = 0;
		t->prep_max = 0;
		t->prep_cnt = 0;
		t->eul_sum = 0;
		t->eul_max = 0;
		t->eul_cnt = 0;
		t->prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	}
}

static void t_note_defer(FlightTelemetry *t, u8 *flag, u8 busy)
{
	if (!*flag) {
		*flag = 1;
		t->txd++;
		if (busy) t->defer_busy++;
		else t->defer_budget++;
		t->tx_diag_due = 1;
	}
}

void FlightTelemetry_DiagStep(FlightTelemetry *t, const FlightTelemetrySnapshot *s)
{
	t_diag_poll(t, s);
	if (t->diag_tx.count) {
		t_diag_flush(t, s->now);
	}
}

void FlightTelemetry_Update(FlightTelemetry *t, const FlightTelemetrySnapshot *s)
{
	u32 now = s->now;
	s32 remain;

	if (t->diag_active) return;

	remain = (s32)(t->tx_tick - now);
	{
		u8 attempted = 0;
		u8 stat_due = ((s32)(now - t->stat_tick) >= 0);
		u8 main_due = ((s32)(now - t->tx_tick) >= 0);
		u8 stat_ok = (remain >= (s32)((t->stat_cost_us + FLIGHT_TX_MARGIN_US)
									 * FLIGHT_TICKS_PER_US));
		u8 obs_ok = (remain >= (s32)((t->obs_cost_us + FLIGHT_TX_MARGIN_US)
									 * FLIGHT_TICKS_PER_US));

		if (stat_due && stat_ok) {
			// 长统计行优先；统计行被串口占用时只记 busy，不抢发短行
			attempted = 1;
			if (t_send_stats(t, now, s)) {
				t->stat_deferred = 0;
				t->tx_deferred = 0;
			} else {
				t_note_defer(t, &t->stat_deferred, 1);
			}
		} else if (stat_due) {
			// 统计行放不下：短观测到点且放得下时先发观测，统计延期
			t_note_defer(t, &t->stat_deferred, 0);
			if (main_due && obs_ok) {
				attempted = 1;
				if (t_send_obs(t, s)) {
					t->tx_deferred = 0;
				} else {
					t_note_defer(t, &t->tx_deferred, 1);
				}
			}
		} else if (main_due) {
			if (obs_ok) {
				attempted = 1;
				if (t_send_obs(t, s)) {
					t->tx_deferred = 0;
				} else {
					t_note_defer(t, &t->tx_deferred, 1);
				}
			} else {
				t_note_defer(t, &t->tx_deferred, 0);
			}
		}

		if (!attempted) {
			if (t->tx_diag_due) {
				t_send_txdiag(t, now);
			}
			t_send_prep(t, now);
		}
	}
}
u8 FlightTelemetry_WindowReset(FlightTelemetry *t)
{
	u8 reset = t->window_reset;

	t->window_reset = 0;
	return reset;
}
