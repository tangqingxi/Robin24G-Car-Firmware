#ifndef RC_INPUT_H
#define RC_INPUT_H

// 一包遥控数据的提交结果。packet_valid=0 表示心跳/原始值越界，
// 但 key 仍已消费，急停事件不会因摇杆无效而丢失。
// 事件按值返回；cmd 偏移 12，类型基址显式对齐到 4 字节。
typedef struct __attribute__((aligned(4))) {
	unsigned char packet_valid;
	unsigned char heartbeat;
	unsigned short key;
	unsigned short raw[4];      // throttle/rotation/fb/lr 原始值
	int cmd[4];                 // 油门/偏航/俯仰/横滚，±307
} RcInputEvent;

// 两个 32 位时间戳放最前，避免 -fpack-struct 把 pkt_tick/link_tick 挤到奇数偏移。
typedef struct __attribute__((aligned(4))) {
	unsigned int pkt_tick;
	unsigned int link_tick;
	int cmd[4];
	unsigned short key;
	unsigned char cmd_ok;
	unsigned char link;
	unsigned char have_pkt;
	unsigned char have_link;
} RcInput;

void RcInput_Init(RcInput *rc);
RcInputEvent RcInput_OnPacket(RcInput *rc, unsigned short thr,
							  unsigned short rot, unsigned short fb,
							  unsigned short lr, unsigned short key,
							  unsigned int now);
void RcInput_UpdateTimeouts(RcInput *rc, unsigned int now);

#endif /* RC_INPUT_H */
