#ifndef L400_DIAG_H
#define L400_DIAG_H

/*
 * L400 A/B 分段诊断的纯逻辑状态机与 UART 待发队列。
 *
 * 这个头文件只依赖标准整数，不碰硬件，方便在主机上回归：
 *   - 连续有效时间用“连续有效起始 tick”与当前 tick 的差判断；
 *   - A/B 各段用段起始 tick 与当前 tick 的差判断；
 *   - 掉链中止时保留 A 段已完成快照和中止原因；
 *   - 汇总行先入队，再用逐行回调发送，忙时保留。
 *
 * 实际统计采集由 UserRxdFunc.c 在事件点完成；本模块只负责时序、
 * 阶段标志和待发文本队列。
 */

#define L400_DIAG_STATE_WAIT    0
#define L400_DIAG_STATE_A       1
#define L400_DIAG_STATE_B       2
#define L400_DIAG_STATE_DONE    3
#define L400_DIAG_STATE_ABORT   4

#define L400_DIAG_EV_START_A    0x01
#define L400_DIAG_EV_A_DONE     0x02
#define L400_DIAG_EV_B_DONE     0x04
#define L400_DIAG_EV_ABORT      0x08

#define L400_DIAG_REASON_NONE   0
#define L400_DIAG_REASON_LINK   1
#define L400_DIAG_REASON_CMD    2

#define L400_DIAG_LINE_LEN      128
#define L400_DIAG_QUEUE_LEN     2

typedef struct {
	unsigned char state;
	unsigned char valid_seen;
	unsigned char a_done;
	unsigned char b_done;
	unsigned char abort_reason;
	unsigned char abort_seg;        // 0=A, 1=B
	unsigned int settle_ticks;
	unsigned int seg_ticks;
	unsigned int valid_since;       // 连续有效状态的起始 tick
	unsigned int seg_since;         // 当前 A/B 段的起始 tick
} L400Diag;

typedef struct {
	char line[L400_DIAG_QUEUE_LEN][L400_DIAG_LINE_LEN];
	unsigned char len[L400_DIAG_QUEUE_LEN];
	unsigned char head;
	unsigned char count;
} L400DiagTx;

static void L400Diag_Init(L400Diag *d, unsigned int settle_ticks,
						  unsigned int seg_ticks)
{
	d->state = L400_DIAG_STATE_WAIT;
	d->valid_seen = 0;
	d->a_done = 0;
	d->b_done = 0;
	d->abort_reason = L400_DIAG_REASON_NONE;
	d->abort_seg = 0;
	d->settle_ticks = settle_ticks;
	d->seg_ticks = seg_ticks;
	d->valid_since = 0;
	d->seg_since = 0;
}

/*
 * 每个 200Hz 控制拍调用一次（等待且无有效状态时上层会跳过）。
 * valid: 姿态就绪 && 链路有效 && 指令有效。
 * reason: valid=0 时传 LINK/CMD，用于中止原因。
 * 返回 L400_DIAG_EV_* 位掩码。
 */
static unsigned int L400Diag_Poll(L400Diag *d, unsigned int now,
								  unsigned char valid, unsigned char reason)
{
	unsigned int ev = 0;

	if (d->state == L400_DIAG_STATE_WAIT) {
		if (!valid) {
			d->valid_seen = 0;
			return 0;
		}
		if (!d->valid_seen) {
			d->valid_seen = 1;
			d->valid_since = now;
			return 0;
		}
		if ((unsigned int)(now - d->valid_since) < d->settle_ticks) {
			return 0;
		}
		d->state = L400_DIAG_STATE_A;
		d->seg_since = now;
		return L400_DIAG_EV_START_A;
	}

	if (d->state != L400_DIAG_STATE_A && d->state != L400_DIAG_STATE_B) {
		return 0;
	}

	// A/B 段内失效：立即中止；已完成阶段由调用方保留
	if (!valid) {
		d->abort_reason = reason ? reason : L400_DIAG_REASON_LINK;
		d->abort_seg = (d->state == L400_DIAG_STATE_A) ? 0 : 1;
		d->state = L400_DIAG_STATE_ABORT;
		return L400_DIAG_EV_ABORT;
	}

	if ((unsigned int)(now - d->seg_since) < d->seg_ticks) {
		return 0;
	}

	if (d->state == L400_DIAG_STATE_A) {
		d->a_done = 1;
		d->state = L400_DIAG_STATE_B;
		d->seg_since = now;
		ev |= L400_DIAG_EV_A_DONE;
	} else {
		d->b_done = 1;
		d->state = L400_DIAG_STATE_DONE;
		ev |= L400_DIAG_EV_B_DONE;
	}
	return ev;
}

static void L400DiagTx_Init(L400DiagTx *tx)
{
	tx->head = 0;
	tx->count = 0;
}

// 汇总行先入队；最多两行（A/B，或 A+中止原因）
static unsigned char L400DiagTx_Enqueue(L400DiagTx *tx, const char *s,
										unsigned char len)
{
	unsigned char idx, i;

	if (tx->count >= L400_DIAG_QUEUE_LEN) {
		return 0;
	}
	if (len >= L400_DIAG_LINE_LEN) {
		len = L400_DIAG_LINE_LEN - 1;
	}
	idx = (unsigned char)((tx->head + tx->count) % L400_DIAG_QUEUE_LEN);
	for (i = 0; i < len; i++) {
		tx->line[idx][i] = s[i];
	}
	tx->line[idx][len] = 0;
	tx->len[idx] = len;
	tx->count++;
	return 1;
}

// 逐行尝试发送：回调返回 0 表示串口忙，保留当前行等下一拍
static void L400DiagTx_Flush(L400DiagTx *tx,
							 unsigned char (*send)(const char *s, unsigned char len))
{
	while (tx->count) {
		unsigned char idx = tx->head;

		if (!send(tx->line[idx], tx->len[idx])) {
			return;
		}
		tx->head = (unsigned char)((tx->head + 1) % L400_DIAG_QUEUE_LEN);
		tx->count--;
	}
}

#endif /* L400_DIAG_H */
