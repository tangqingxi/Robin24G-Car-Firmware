//********************************************************************
/*
 * UserEuler.h
 *
 *  Created on: 2026年10月4日
 *      Author: HuangGuangjian
 */
//********************************************************************
//********************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USEREULER_H_
#define VENDOR_ROBIN_24G_CAR_USEREULER_H_
//********************************************************************
//********************************************************************
extern int16_t accel[3], gyro[3];
extern int16_t g_gyro_offset_x;
extern int16_t g_gyro_offset_y;
extern int16_t g_gyro_offset_z;
extern volatile int32_t g_roll;   // 横滚角 (绕X轴)
extern volatile int32_t g_pitch;   // 俯仰角 (绕Y轴)
extern volatile int32_t g_yaw;   // 偏航角 (绕Z轴)，仅积分，会漂移

//ConvertT3488ParaToS16() 输出的 s16 量（原始值/欧拉角），供漂移辅助等模块使用
extern int16_t s16AccX, s16AccY, s16AccZ, s16GyroX, s16GyroY, s16GyroZ;
extern int16_t s16Roll, s16Pitch, s16Yaw;		//单位0.01度
extern void ConvertT3488ParaToS16(void);


//********************************************************************
//********************************************************************
//void update_euler_10ms(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
//                       uint16_t gx_raw, uint16_t gy_raw, uint16_t gz_raw);		//每 10ms 调用一次
void update_euler_10ms(int16_t *accel, int16_t *gyro);		//每 10ms 调用一次（兼容旧调用）

//********************************************************************
//带周期参数的姿态解算（飞控用）
//	u8DtMs: 调用周期，单位 mS（飞控推荐 5 或 6）
//	陀螺积分与互补滤波系数都随 u8DtMs 自动换算，因此改周期无需改增益
//********************************************************************
void update_euler_dt(int16_t *accel, int16_t *gyro, unsigned char u8DtMs);
void gyro_calibrate(uint16_t *gx_buf, uint16_t *gy_buf, uint16_t *gz_buf, int n);		//上电后静止时调用，采集 N 次求平均作为零偏
void get_euler_angle(int32_t *roll, int32_t *pitch, int32_t *yaw);
void get_GyroSensor_offset(void);		//获取Gyro传感器零偏
void usart1_data_report(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 roll,s16 pitch,s16 yaw);
void usart1_report_EulerDat(void);

//********************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USEREULER_H_ */
//********************************************************************

