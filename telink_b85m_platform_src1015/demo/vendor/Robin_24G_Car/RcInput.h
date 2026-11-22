#ifndef RC_INPUT_H
#define RC_INPUT_H

// 一包遥控数据的提交结果。packet_valid=0 表示心跳/原始值越界，
// 但 key 仍已消费，急停事件不会因摇杆无效而丢失。
typedef struct {
	unsigned char packet_valid;
	unsigned char heartbeat;
	unsigned short key;
	unsigned short raw[4];      // throttle/rotation/fb/lr 原始值
	int cmd[4];                 // 油门/偏航/俯仰/横滚，±307
} RcInputEvent;

typedef struct {
	int cmd[4];
	unsigned short key;
	unsigned char cmd_ok;
	unsigned char link;
	unsigned char have_pkt;
	unsigned char have_link;
	unsigned int pkt_tick;
	unsigned int link_tick;
	unsigned int gap_max_tick;
	unsigned int bad;
} RcInput;

typedef struct {
	int cmd[4];
	unsigned short key;
	unsigned char cmd_ok;
	unsigned char link;
	unsigned int link_age_ms;
	unsigned int pkt_age_ms;
	unsigned int gap_ms;
	unsigned int bad;
} RcInputFrame;

void RcInput_Init(RcInput *rc);
RcInputEvent RcInput_OnPacket(RcInput *rc, unsigned short thr,
							  unsigned short rot, unsigned short fb,
							  unsigned short lr, unsigned short key,
							  unsigned int now);
void RcInput_UpdateTimeouts(RcInput *rc, unsigned int now);
void RcInput_Get(const RcInput *rc, RcInputFrame *out, unsigned int now);
void RcInput_ClearGap(RcInput *rc);

#endif /* RC_INPUT_H */
