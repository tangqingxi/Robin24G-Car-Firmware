#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "UserEuler.h"

typedef struct {
	int tgt_att[2];       // 横滚/俯仰目标，0.01°
	int tgt_up[3];        // 目标向上比力 Q15
	int tgt_yaw_rate;     // 偏航目标角速度 Q10
	int tgt_rate[3];      // 外环输出三轴目标角速度 Q10
	int stick_last[2];
	int stick_yaw_last;
	unsigned char valid;
} AttitudeControl;

void AttitudeControl_Init(AttitudeControl *ac);
void AttitudeControl_OnStick(AttitudeControl *ac, const int cmd[4]);
void AttitudeControl_Update(AttitudeControl *ac, const euler_q_t *att);
void AttitudeControl_ClearOutputs(AttitudeControl *ac);
void AttitudeControl_Inhibit(AttitudeControl *ac);

#endif /* ATTITUDE_CONTROL_H */
