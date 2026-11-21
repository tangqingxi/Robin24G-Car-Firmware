/*
 * L400 A/B 诊断状态机与待发队列回归（主机侧，不依赖 TC32 硬件）
 *
 * 覆盖：
 *   1. 连续有效满 2s 才进入 A；
 *   2. A/B 各 10s；
 *   3. 中途失效后重新计 2s；
 *   4. A/B 掉链中止保留已完成阶段和中止原因；
 *   5. UART 忙时汇总行保留，忙过后 A/B 两行都能发出。
 *
 * 编译（在 tools 目录）：
 *   gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
 *       test_l400_diag.c -o test_l400_diag.exe
 */

#include <stdio.h>
#include <string.h>

#include "L400Diag.h"

#define SETTLE  400u        /* 2s @200Hz */
#define SEG     2000u       /* 10s @200Hz */

static int failed;

static void check(int cond, const char *msg)
{
	if (!cond) {
		printf("FAIL %s\n", msg);
		failed = 1;
	}
}

static void test_timing(void)
{
	L400Diag d;

	L400Diag_Init(&d, SETTLE, SEG);
	check(L400Diag_Poll(&d, 0, 1, 0) == 0, "first valid should not start");
	check(L400Diag_Poll(&d, 399, 1, 0) == 0, "should wait full 2s");
	check((L400Diag_Poll(&d, 400, 1, 0) & L400_DIAG_EV_START_A) != 0,
		  "2s continuous valid should start A");
	check(d.state == L400_DIAG_STATE_A, "state should be A");

	check(L400Diag_Poll(&d, 2399, 1, 0) == 0, "A should run full 10s");
	check((L400Diag_Poll(&d, 2400, 1, 0) & L400_DIAG_EV_A_DONE) != 0,
		  "A should finish after 10s");
	check(d.state == L400_DIAG_STATE_B, "state should be B");
	check(L400Diag_Poll(&d, 4399, 1, 0) == 0, "B should run full 10s");
	check((L400Diag_Poll(&d, 4400, 1, 0) & L400_DIAG_EV_B_DONE) != 0,
		  "B should finish after 10s");
	check(d.state == L400_DIAG_STATE_DONE, "state should be DONE");
	check(d.a_done && d.b_done, "both segments done");
	printf("PASS timing 2s/10s/10s\n");
}

static void test_invalid_resets_timer(void)
{
	L400Diag d;

	L400Diag_Init(&d, SETTLE, SEG);
	L400Diag_Poll(&d, 0, 1, 0);
	L400Diag_Poll(&d, 250, 0, L400_DIAG_REASON_LINK);
	L400Diag_Poll(&d, 300, 1, 0);            /* 重新从 300 开始计 */
	check(L400Diag_Poll(&d, 699, 1, 0) == 0, "timer should restart at 300");
	check((L400Diag_Poll(&d, 700, 1, 0) & L400_DIAG_EV_START_A) != 0,
		  "2s from restart should start A");
	printf("PASS invalid resets valid timer\n");
}

static void test_abort(void)
{
	L400Diag d;

	/* A 段中止：没有已完成阶段 */
	L400Diag_Init(&d, SETTLE, SEG);
	L400Diag_Poll(&d, 0, 1, 0);
	check((L400Diag_Poll(&d, 400, 1, 0) & L400_DIAG_EV_START_A) != 0,
		  "abort case should start A");
	check((L400Diag_Poll(&d, 500, 0, L400_DIAG_REASON_LINK)
		   & L400_DIAG_EV_ABORT) != 0, "A abort event");
	check(d.abort_reason == L400_DIAG_REASON_LINK, "A abort reason link");
	check(d.abort_seg == 0 && !d.a_done, "A abort has no completed A");

	/* B 段中止：A 已完成，保留 A 标志和 B 中止原因 */
	L400Diag_Init(&d, SETTLE, SEG);
	L400Diag_Poll(&d, 0, 1, 0);
	L400Diag_Poll(&d, 400, 1, 0);
	L400Diag_Poll(&d, 2400, 1, 0);
	check(d.state == L400_DIAG_STATE_B && d.a_done, "A done before B abort");
	check((L400Diag_Poll(&d, 2500, 0, L400_DIAG_REASON_CMD)
		   & L400_DIAG_EV_ABORT) != 0, "B abort event");
	check(d.abort_reason == L400_DIAG_REASON_CMD, "B abort reason cmd");
	check(d.abort_seg == 1 && d.a_done && !d.b_done,
		  "B abort keeps completed A");
	printf("PASS abort keeps stage/reason\n");
}

/* 待发队列测试：busy_calls 次发送返回忙，之后成功 */
static int busy_calls;
static int sent_count;
static char sent[2][L400_DIAG_LINE_LEN];

static unsigned char fake_send(const char *s, unsigned char len)
{
	if (busy_calls > 0) {
		busy_calls--;
		return 0;
	}
	if (sent_count < 2) {
		memcpy(sent[sent_count], s, len);
		sent[sent_count][len] = 0;
	}
	sent_count++;
	return 1;
}

static void test_tx_busy(void)
{
	L400DiagTx tx;

	L400DiagTx_Init(&tx);
	check(L400DiagTx_Enqueue(&tx, "A", 1) == 1, "enqueue A");
	check(L400DiagTx_Enqueue(&tx, "B", 1) == 1, "enqueue B");

	busy_calls = 1;
	L400DiagTx_Flush(&tx, fake_send);
	check(tx.count == 2 && sent_count == 0, "busy keeps both lines");

	busy_calls = 0;
	L400DiagTx_Flush(&tx, fake_send);
	check(tx.count == 0, "queue drains after busy");
	check(sent_count == 2, "both lines sent");
	check(strcmp(sent[0], "A") == 0 && strcmp(sent[1], "B") == 0,
		  "A/B order preserved");
	printf("PASS UART busy retains and later sends A/B\n");
}

int main(void)
{
	test_timing();
	test_invalid_resets_timer();
	test_abort();
	test_tx_busy();
	return failed ? 1 : 0;
}
