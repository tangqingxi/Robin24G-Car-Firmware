#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "UserEuler.h"

// 外环状态；成员顺序保证 4 字节对齐，便于 TC32 整块访问。
typedef struct __attribute__((aligned(4))) {
	int tgt_att[2];       // 横滚/俯仰目标，0.01°，右滚/抬头为正
	int tgt_up[3];        // 目标向上比力，Q15，机体 FRD 表达
	int tgt_yaw_rate;     // 偏航目标角速度，Q10 rad/s，俯视顺时针为正
	int tgt_rate[3];      // 外环输出三轴目标角速度，Q10 rad/s
	int stick_last[2];    // 上一次横滚/俯仰杆量，用于“杆量没变不重算”
	int stick_yaw_last;   // 上一次偏航杆量
	unsigned char valid;  // 1=已收到过有效杆量；0=需要强制重算
} AttitudeControl;

void AttitudeControl_Init(AttitudeControl *ac);
void AttitudeControl_OnStick(AttitudeControl *ac, const int cmd[4]);
void AttitudeControl_Update(AttitudeControl *ac, const euler_q_t *att);
void AttitudeControl_ClearOutputs(AttitudeControl *ac);
void AttitudeControl_Inhibit(AttitudeControl *ac);

#endif /* ATTITUDE_CONTROL_H */
