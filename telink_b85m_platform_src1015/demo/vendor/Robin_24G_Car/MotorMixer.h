#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

// 手动油门与十字(+)四电机混控常量。
// 杆量 307 ≈ 满舵；推力中位 190，向上最多 340、向下最少 60。
// 340 的上限故意低于 PWM 上限 400，给横滚/俯仰/偏航差动留余量。
#define MOTOR_STICK_MAX          307
#define MOTOR_THR_MID            190
#define MOTOR_THR_SPAN           140
#define MOTOR_THR_MIN            60
#define MOTOR_THR_MAX            340
// 混控输出的绝对上下限；45 是电机能持续运行的最小 PWM，0 由安全门直接写。
#define MOTOR_PWM_MIN_RUN        45
#define MOTOR_PWM_MAX            400
// 解锁判定：油门必须低于 -250，且俯仰/横滚/偏航杆都在 ±45 内。
#define MOTOR_ARM_THR_MAX        (-250)
#define MOTOR_ARM_STICK_MAX      45

// Q10 纠偏为 4096 时，横滚、俯仰、偏航限幅前的差动分别为 180、150、120。
// 手测力臂前后约 48 毫米、左右约 40 毫米，俯仰系数按 2880 × 40 ÷ 48 补偿。
#define MOTOR_ROLL_DIFF_GAIN_Q16   2880
#define MOTOR_PITCH_DIFF_GAIN_Q16  2400
#define MOTOR_YAW_DIFF_GAIN_Q16    1920
#define MOTOR_DIFF_LIMIT           160

typedef struct __attribute__((aligned(2))) {
	unsigned short front;
	unsigned short rear;
	unsigned short left;
	unsigned short right;
	unsigned char limited;       // 1=本拍发生统一缩放限幅
} MotorMixerOutput;

int MotorMixer_Thrust(int fb);
void MotorMixer_Mix(int thrust, int corr_roll, int corr_pitch,
					int corr_yaw, MotorMixerOutput *out);

#endif /* MOTOR_MIXER_H */
