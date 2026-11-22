#ifndef FLIGHT_TELEMETRY_H
#define FLIGHT_TELEMETRY_H

#include "FlightConfig.h"
#include "L400Diag.h"

typedef struct {
	u32 late_avg, late_max;
	u32 exec_avg, exec_max;
	u32 cmiss, skip;
	u32 rfirq_avg, rfirq_max;
	u32 rf_avg, rf_max;
	u32 app_avg, app_max;
} FlightDiagResult;

// 本拍控制/姿态快照；遥测只读，不反向修改控制状态。
typedef struct {
	u32 now;
	u8 att_ready;
	u8 link;
	u8 cmd_ok;
	u8 ctrl_ok;
	s32 q[4];
	s32 rate[3];
	s32 corr[3];
	s32 tgt_att[2];
	s32 tgt_rate[3];
	u8 armed;
	u8 reason;
	u16 pwm[4];
	u16 link_age_ms;
	u16 pkt_age_ms;
	u16 gap_ms;
	u16 bad;
} FlightTelemetrySnapshot;

typedef struct {
	u32 sample;          // 成功采样累计
	u32 fail;            // 读失败
	u32 euler_ok;        // 姿态更新成功
	u32 miss;
	u32 ctrl_miss;
	u32 skip;
	u32 txd;
	u32 ctrl_max_us;
	u32 ctrl_peak_us;

	u32 late_sum, late_max;
	u32 exec_sum, exec_max;
	u32 prep_sum, prep_max, prep_cnt;
	u32 eul_sum, eul_max, eul_cnt;
	u32 rx_sum, rx_max, rx_cnt;
	u32 tx_sum, tx_max, tx_cnt;
	u32 tick_cnt;
	u32 gap_max_tick;

	u32 tx_tick;
	u32 stat_tick;
	u8 stat_deferred;
	u8 tx_deferred;
	u32 obs_cost_us;
	u32 stat_cost_us;
	u32 defer_budget;
	u32 defer_busy;
	u8 tx_diag_due;
	u32 tx_diag_tick;
	u32 prep_tick;

	L400Diag diag;
	L400DiagTx diag_tx;
	u8 diag_enabled;
	u8 diag_active;
	u8 diag_ctrl_enable;
	u8 diag_have_a;
	u16 diag_key_prev;
	FlightDiagResult diag_a;
	FlightDiagResult diag_b;
	u32 diag_rf_sum_ticks;
	u32 diag_rf_max_ticks;
	u32 diag_rf_cnt;
	u8 window_reset;
} FlightTelemetry;

void FlightTelemetry_Init(FlightTelemetry *t, u32 now);
void FlightTelemetry_OnKey(FlightTelemetry *t, u16 key, u32 now);
void FlightTelemetry_DiagStep(FlightTelemetry *t, const FlightTelemetrySnapshot *s);
void FlightTelemetry_Update(FlightTelemetry *t, const FlightTelemetrySnapshot *s);
void FlightTelemetry_OnSample(FlightTelemetry *t);
void FlightTelemetry_OnFail(FlightTelemetry *t);
void FlightTelemetry_OnEuler(FlightTelemetry *t);
void FlightTelemetry_OnLate(FlightTelemetry *t, u32 us);
void FlightTelemetry_OnExec(FlightTelemetry *t, u32 us);
void FlightTelemetry_OnPrep(FlightTelemetry *t, u32 us);
void FlightTelemetry_OnEul(FlightTelemetry *t, u32 us);
void FlightTelemetry_OnRx(FlightTelemetry *t, u32 us);
void FlightTelemetry_OnGap(FlightTelemetry *t, u32 ticks);
void FlightTelemetry_OnTick(FlightTelemetry *t);
void FlightTelemetry_OnCtrl(FlightTelemetry *t, u32 ticks);
void FlightTelemetry_OnMiss(FlightTelemetry *t);
void FlightTelemetry_OnSkip(FlightTelemetry *t, u32 count);

u8 FlightTelemetry_DiagActive(const FlightTelemetry *t);
u8 FlightTelemetry_CtrlEnable(const FlightTelemetry *t);
void FlightTelemetry_RecordRfLoop(FlightTelemetry *t, u32 ticks);
u8 FlightTelemetry_WindowReset(FlightTelemetry *t);

#endif /* FLIGHT_TELEMETRY_H */
