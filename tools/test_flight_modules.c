/*
 * 飞控拆分模块回归：RC 映射/超时、外环目标、内环 P。
 * 纯主机测试，不链接任何硬件驱动。
 */
#include <stdio.h>
#include <string.h>

#include "RcInput.h"
#include "AttitudeControl.h"
#include "RateControl.h"

static int failed;

static void check(int cond, const char *msg)
{
	if (!cond) {
		printf("FAIL %s\n", msg);
		failed = 1;
	}
}

static void test_rc_mapping_and_timeouts(void)
{
	RcInput rc;
	RcInputEvent ev;
	RcInputFrame f;
	unsigned int now = 100000;
	unsigned int t;

	RcInput_Init(&rc);
	ev = RcInput_OnPacket(&rc, 0x1000, 0x1000, 0x0400, 0x1000, 0, now);
	check(ev.packet_valid, "center raw packet valid");
	check(rc.cmd[0] == -307, "FB low maps throttle -307");
	check(rc.cmd[1] == 0 && rc.cmd[2] == 0 && rc.cmd[3] == 0,
		  "center yaw/pitch/roll");

	ev = RcInput_OnPacket(&rc, 0x2000, 0x1000, 0x1000, 0x1000, 0, now + 10);
	check(!ev.packet_valid, "out-of-range packet rejected");
	check(rc.bad == 1, "bad counter");

	ev = RcInput_OnPacket(&rc, 0, 0, 0, 0, 0, now + 20);
	check(ev.heartbeat && !ev.packet_valid, "all-zero heartbeat");
	check(rc.cmd_ok, "heartbeat does not invalidate command");

	t = now + 20 + FLIGHT_CMD_TIMEOUT_TICKS + 1;
	RcInput_UpdateTimeouts(&rc, t);
	check(!rc.cmd_ok, "200ms command timeout");
	check(rc.link, "500ms link still valid");
	RcInput_Get(&rc, &f, t);
	check(f.pkt_age_ms >= FLIGHT_CMD_TIMEOUT_MS, "packet age");

	t += FLIGHT_LINK_TIMEOUT_TICKS + 1;
	RcInput_UpdateTimeouts(&rc, t);
	check(!rc.link, "500ms link timeout");

	printf("PASS rc mapping/heartbeat/timeouts\n");
}

static void test_attitude_targets(void)
{
	AttitudeControl ac;
	euler_q_t q;
	int cmd[4] = {0, 307, 0, 0};
	int i;

	AttitudeControl_Init(&ac);
	memset(&q, 0, sizeof(q));
	q.q[0] = 32767;
	AttitudeControl_OnStick(&ac, cmd);
	AttitudeControl_Update(&ac, &q);
	for (i = 0; i < 2; i++) {
		check(ac.tgt_rate[i] == 0, "level roll/pitch rate zero");
	}
	check(ac.tgt_rate[2] == FLIGHT_YAW_RATE_MAX_Q10,
		  "yaw target passthrough");

	cmd[1] = 0;
	cmd[3] = 307;
	AttitudeControl_OnStick(&ac, cmd);
	AttitudeControl_Update(&ac, &q);
	check(ac.tgt_rate[0] != 0 || ac.tgt_rate[1] != 0,
		  "roll stick produces outer rate");

	printf("PASS attitude targets/yaw passthrough\n");
}

static void test_rate_inner(void)
{
	RateControl rc;
	int target[3] = {100, -100, 0};
	int gyro[3] = {0, 0, 0};
	int i;

	RateControl_Init(&rc);
	RateControl_Update(&rc, target, gyro);
	for (i = 0; i < 3; i++) {
		check(rc.corr[i] == (target[i] * FLIGHT_KP_Q8) >> 8,
			  "inner P=64/256");
	}
	RateControl_ClearOutputs(&rc);
	check(rc.corr[0] == 0 && rc.corr[1] == 0 && rc.corr[2] == 0,
		  "inner reset");

	printf("PASS rate inner P and reset\n");
}

int main(void)
{
	test_rc_mapping_and_timeouts();
	test_attitude_targets();
	test_rate_inner();
	if (failed) {
		printf("RESULT FAIL\n");
		return 1;
	}
	printf("RESULT OK\n");
	return 0;
}
