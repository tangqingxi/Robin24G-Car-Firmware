#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

// 手动油门与十字(+)四电机混控常量。
#define MOTOR_STICK_MAX          307
#define MOTOR_THR_MID            190
#define MOTOR_THR_SPAN           140
#define MOTOR_THR_MIN            60
#define MOTOR_THR_MAX            340
#define MOTOR_PWM_MIN_RUN        45
#define MOTOR_PWM_MAX            400
#define MOTOR_ARM_THR_MAX        (-250)
#define MOTOR_ARM_STICK_MAX      45

// 纠偏 Q10(±4096) → PWM 差动；横滚/俯仰满量程约 180，限 160；
// 偏航满量程约 120。系数已包含十字 + 型相对 X 型的 1.414 补偿。
#define MOTOR_DIFF_GAIN_Q16      2880
#define MOTOR_YAW_DIFF_GAIN_Q16  1920
#define MOTOR_DIFF_LIMIT         160

typedef struct {
	unsigned short front;
	unsigned short rear;
	unsigned short left;
	unsigned short right;
} MotorMixerOutput;

int MotorMixer_Thrust(int fb);
void MotorMixer_Mix(int thrust, int corr_roll, int corr_pitch,
					int corr_yaw, MotorMixerOutput *out);

#endif /* MOTOR_MIXER_H */
