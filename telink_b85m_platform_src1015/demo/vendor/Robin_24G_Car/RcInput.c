#include "RcInput.h"
#include "FlightConfig.h"

#define RF_DATA_MIN 0x0400
#define RF_DATA_MAX 0x1C00
#define RF_DATA_MID 0x1000

// 与旧 RxdDataConvert 相同的中心/比例换算，先校验再调用。
static int rc_convert(unsigned short raw)
{
	int v = (int)raw - RF_DATA_MID;
	return v / 10;
}

void RcInput_Init(RcInput *rc)
{
	int i;

	for (i = 0; i < 4; i++) {
		rc->cmd[i] = 0;
	}
	rc->key = 0;
	rc->cmd_ok = 0;
	rc->link = 0;
	rc->have_pkt = 0;
	rc->have_link = 0;
	rc->pkt_tick = 0;
	rc->link_tick = 0;
}

/*
 * 一包解码完成后的入口。任何包先刷新链路和 key；随后才判断
 * 全零心跳/原始值越界/有效摇杆。key 事件由 FlightSafety 在包路径消费，
 * 所以这里即使 packet_valid=0 也返回 key。
 */
RcInputEvent RcInput_OnPacket(RcInput *rc, unsigned short thr,
							  unsigned short rot, unsigned short fb,
							  unsigned short lr, unsigned short key,
							  unsigned int now)
{
	RcInputEvent ev;

	ev.packet_valid = 0;
	ev.heartbeat = 0;
	ev.key = key;
	ev.raw[0] = thr;
	ev.raw[1] = rot;
	ev.raw[2] = fb;
	ev.raw[3] = lr;
	ev.cmd[0] = rc->cmd[0];
	ev.cmd[1] = rc->cmd[1];
	ev.cmd[2] = rc->cmd[2];
	ev.cmd[3] = rc->cmd[3];

	rc->link = 1;
	rc->link_tick = now;
	rc->have_link = 1;
	rc->key = key;

	// 全零心跳：只更新链路/按键，不动摇杆指令
	if (thr == 0 && rot == 0 && fb == 0 && lr == 0) {
		ev.heartbeat = 1;
		return ev;
	}

	// 原始值越界：包丢弃，但事件仍被消费
	if (thr < RF_DATA_MIN || thr > RF_DATA_MAX
		|| rot < RF_DATA_MIN || rot > RF_DATA_MAX
		|| fb < RF_DATA_MIN || fb > RF_DATA_MAX
		|| lr < RF_DATA_MIN || lr > RF_DATA_MAX) {
		return ev;
	}

	// 实机确认映射：FB→油门、LR→偏航、Throttle→俯仰、Rotation→横滚(-1)
	rc->cmd[0] = rc_convert(fb);
	rc->cmd[1] = rc_convert(lr);
	rc->cmd[2] = rc_convert(thr);
	rc->cmd[3] = -rc_convert(rot);

	rc->pkt_tick = now;
	rc->have_pkt = 1;
	rc->cmd_ok = 1;

	ev.packet_valid = 1;
	ev.cmd[0] = rc->cmd[0];
	ev.cmd[1] = rc->cmd[1];
	ev.cmd[2] = rc->cmd[2];
	ev.cmd[3] = rc->cmd[3];
	return ev;
}

// 两种超时保持原毫秒数：有效摇杆 200ms、链路 500ms。
void RcInput_UpdateTimeouts(RcInput *rc, unsigned int now)
{
	if (rc->have_link
		&& (unsigned int)(now - rc->link_tick) > FLIGHT_LINK_TIMEOUT_TICKS) {
		rc->link = 0;
	}
	if (rc->have_pkt
		&& (unsigned int)(now - rc->pkt_tick) > FLIGHT_CMD_TIMEOUT_TICKS) {
		rc->cmd_ok = 0;
	}
}
