//*******************************************************************
//*******************************************************************
/*
 * UserEuler.c
 *
 *  Created on: 2026年10月3日
 *      Author: HuangGuangjian
 */
//*******************************************************************
#include "types.h"
#include "UserUart.h"
#include "UserEuler.h"
#include "User_IIC.h"
#include "UserFilter.h"
#include "app_public_config.h"

//*******************************************************************
/* ===================== 配置区 ===================== */

#define ANGLE_UNIT      1000        // 角度单位：0.001度 (1000 = 1.000°)
#define DT_MS           10          // 更新周期 10ms

//*******************************************************************
// 陀螺仪积分系数（定点，随调用周期自适应）
// 2000dps 档位，16.4 LSB/°/s → 每 LSB 的角速度 = 1/16.4 °/s
// 每 1mS 的角度增量 = (1/16.4) * 0.001 ° = 0.000060976 ° = 0.060976 (0.001度单位)
// 0.060976 × 4096 = 249.75 → 取 250，故：
//		角度增量(0.001°) = (gyro_LSB × dt_mS × 250) >> 12
// 校验：10mS、1 LSB → (1×10×250)>>12 = 0.6104 (0.001°)，真值 0.60976，误差 0.1%
//*******************************************************************
#define GYRO_INTEG_NUM_4096	250
#define GYRO_INTEG_SHIFT	12

//*******************************************************************
// 互补滤波：用"加速度计信任时间常数"描述，随调用周期自动换算 α
//		α = τ / (τ + dt)
// τ 越大 → 越信任陀螺仪（振动大时取大值）；τ 越小 → 越信任加速度计
// ★ 2026/11/07：400 → 1500。台架 200Hz 数据实测：桨流振动被整流成 ax/ay
//		上的假直流(±0.5~0.8°等效倾角、约 3S 周期)，τ=400mS 时几乎原样
//		灌进角度估计 → 角度环跟着假角度反复修正 → 真机"脉冲式"摇晃。
//		τ=1.5S 后 3S 周期的摆动衰减到 ~1/4，静态收敛时间 2~3S（水平校准
//		本来就会静置，可接受）。若个别机架振动更大可再加到 2000~3000。
//*******************************************************************
#define ACC_TRUST_TAU_MS	1600		//1500

//*******************************************************************
// 加计 1g 模值门控（2026/11/07 新增）
//	|ax,ay,az| 合模值偏离 1g 超过 ACC_MAG_GATE 时，本拍完全不信加计
//	（纯陀螺）：起飞/降落瞬间的线性加速度、大振动都会把 acc 角度拉偏，
//	门控把它们挡在估计器外。±16g 量程下 1g = 2048 LSB（T3488 实测）。
//*******************************************************************
#define ACC_1G_LSB			2048
#define ACC_MAG_GATE		410		// 0.2g

//*******************************************************************
// 陀螺仪零偏 (上电静止校准后写入，原始值)
//*******************************************************************
int16_t accel[3], gyro[3];
int16_t g_gyro_offset_x = 0;
int16_t g_gyro_offset_y = 0;
int16_t g_gyro_offset_z = 0;

/* ===================== 查表 atan2 ===================== */

// atan(t) 查找表，t ∈ [0, 1]，对应角度 [0, 45°]
// 表大小 65，分辨率约 1/64 ≈ 1.56%
// 输出单位：0.001度 (即 45000 = 45.000°)
//*******************************************************************
const uint16_t ATAN_LUT[65] = {
      0,  1000,  2000,  3000,  4000,  5000,  6000,  7000,
   8000,  8900,  9800, 10700, 11600, 12500, 13400, 14200,
  15000, 15800, 16600, 17400, 18100, 18900, 19600, 20300,
  21000, 21700, 22400, 23000, 23700, 24300, 24900, 25500,
  26100, 26700, 27200, 27800, 28300, 28800, 29300, 29800,
  30300, 30800, 31200, 31700, 32100, 32500, 32900, 33300,
  33700, 34100, 34500, 34900, 35200, 35600, 35900, 36300,
  36600, 36900, 37200, 37500, 37800, 38100, 38400, 38700,
  39000
};

//*******************************************************************
//*******************************************************************
// 整数 atan2，输入 y, x（任意 int16_t 范围）
// 返回角度，单位 0.001度，范围 [-180000, 180000]
//*******************************************************************
int32_t int_atan2(int32_t y, int32_t x)
{
    if (x == 0) {
        if (y > 0) return 90000;
        if (y < 0) return -90000;
        return 0;
    }

    int32_t abs_y = (y < 0) ? -y : y;
    int32_t abs_x = (x < 0) ? -x : x;
    int32_t result;

    if (abs_y <= abs_x) {
        // |y/x| ≤ 1，查表索引 = |y| * 64 / |x|
        int32_t idx = (abs_y << 6) / abs_x;
        if (idx > 64) idx = 64;
        result = ATAN_LUT[idx];
    } else {
        // |y/x| > 1，用 90° 减去 atan(|x/y|)
        int32_t idx = (abs_x << 6) / abs_y;
        if (idx > 64) idx = 64;
        result = 90000 - ATAN_LUT[idx];
    }

    // 象限还原
    if (x < 0) {
        if (y < 0) return result - 180000;
        return 180000 - result;
    } else {
        if (y < 0) return -result;
        return result;
    }
}

//*******************************************************************
//注意：上面的 result 单位是度，乘以 ANGLE_UNIT 是为了统一成 0.001度单位。
//如果追求极致速度，可以统一使用“度”作为中间单位，最后再乘以 ANGLE_UNIT。
//*******************************************************************


//*******************************************************************
/* ===================== 姿态解算 ===================== */
//*******************************************************************
// 全局欧拉角，单位 0.001度
//*******************************************************************
volatile int32_t g_roll  = 0;   // 横滚角 (绕X轴)
volatile int32_t g_pitch = 0;   // 俯仰角 (绕Y轴)
volatile int32_t g_yaw   = 0;   // 偏航角 (绕Z轴)，仅积分，会漂移

//*******************************************************************
// 带周期参数的姿态解算（飞控用：5mS / 6mS / 10mS 均可）
// 输入：加速度原始值 int16_t，陀螺仪原始值 int16_t
// 安装方向：X向前，Y向左，Z向上
//	u8DtMs: 距上次调用的间隔，单位 mS
//*******************************************************************
void update_euler_dt(int16_t *accel, int16_t *gyro, unsigned char u8DtMs)
{
	int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
	int32_t s32AlphaQ10, s32OneMinusAlphaQ10;
	int32_t s32DtMs = (int32_t)u8DtMs;

	ax_raw = accel[0];
	ay_raw = accel[1];
	az_raw = accel[2];
	gx_raw = gyro[0];
	gy_raw = gyro[1];
	gz_raw = gyro[2];

	if(s32DtMs < 1)  s32DtMs = 1;
	if(s32DtMs > 50) s32DtMs = 50;

    // ---------- 0. 按周期换算互补滤波系数 ----------
    // α = τ / (τ + dt)   (Q10)
	s32AlphaQ10 = ((int32_t)ACC_TRUST_TAU_MS << 10) / ((int32_t)ACC_TRUST_TAU_MS + s32DtMs);
	s32OneMinusAlphaQ10 = 1024 - s32AlphaQ10;

    // ---------- 1. 陀螺仪：去零偏 + 积分 ----------
    int16_t gx = (int16_t)gx_raw - g_gyro_offset_x;
    int16_t gy = (int16_t)gy_raw - g_gyro_offset_y;
    int16_t gz = (int16_t)gz_raw - g_gyro_offset_z;

    // 积分得到角度变化 (定点乘法 + 累加)
    // 坐标系：X向前，Y向左，Z向上
    // Roll  绕 X 轴 → 由 gyro_x 积分
    // Pitch 绕 Y 轴 → 由 gyro_y 积分
    // Yaw   绕 Z 轴 → 由 gyro_z 积分
    // 角度增量(0.001°) = (gyro_LSB × dt_mS × 250) >> 12
    int32_t roll_gyro  = g_roll  + (int32_t)(((int32_t)gx * s32DtMs * GYRO_INTEG_NUM_4096) >> GYRO_INTEG_SHIFT);
    int32_t pitch_gyro = g_pitch + (int32_t)(((int32_t)gy * s32DtMs * GYRO_INTEG_NUM_4096) >> GYRO_INTEG_SHIFT);
    int32_t yaw_gyro   = g_yaw   + (int32_t)(((int32_t)gz * s32DtMs * GYRO_INTEG_NUM_4096) >> GYRO_INTEG_SHIFT);

    // ---------- 2. 加速度计：计算静态角度 ----------
    // 由重力分量估算 Roll 和 Pitch（只在静态/低动态时准确）
    // Roll ≈ atan2(ay, az)   （绕 X 轴倾斜，Y/Z 平面重力分量）
    // Pitch ≈ atan2(-ax, az)
    int32_t roll_acc  = int_atan2(ay_raw, az_raw);
    int32_t pitch_acc = int_atan2(-ax_raw, az_raw);

    // ---------- 2.5 加计模值门控（见文件头 ACC_MAG_GATE 说明） ----------
    //	模值偏离 1g 超过 0.2g → 本拍 α=1（纯陀螺），完全不信加计。
    //	注意：桨流振动的"整流直流"不改变模值，门控挡不住它，靠大 τ 摊薄；
    //	门控挡的是线性加速度类的大扰动（起降瞬间、机动、外部冲击）。
    {
        s32 axg = ax_raw, ayg = ay_raw, azg = az_raw;
        // 先钳到 ±4g 再平方，防 16g 满量程时 int32 溢出（超 4g 必被门控挡掉）
        if(axg >  8192) axg =  8192; else if(axg < -8192) axg = -8192;
        if(ayg >  8192) ayg =  8192; else if(ayg < -8192) ayg = -8192;
        if(azg >  8192) azg =  8192; else if(azg < -8192) azg = -8192;
        s32 acc_mag2 = axg*axg + ayg*ayg + azg*azg;
        s32 mag2_lo = (ACC_1G_LSB - ACC_MAG_GATE) * (ACC_1G_LSB - ACC_MAG_GATE);
        s32 mag2_hi = (ACC_1G_LSB + ACC_MAG_GATE) * (ACC_1G_LSB + ACC_MAG_GATE);
        if((acc_mag2 < mag2_lo) || (acc_mag2 > mag2_hi))
        {
            s32AlphaQ10 = 1024;				// 本拍纯陀螺
            s32OneMinusAlphaQ10 = 0;
        }
    }

    // ---------- 3. 互补滤波融合 ----------
    // g = (ALPHA * gyro + (1-ALPHA) * acc) / 1024
    g_roll  = ((int32_t)s32AlphaQ10 * roll_gyro
             + (int32_t)s32OneMinusAlphaQ10 * roll_acc) >> 10;

    g_pitch = ((int32_t)s32AlphaQ10 * pitch_gyro
             + (int32_t)s32OneMinusAlphaQ10 * pitch_acc) >> 10;

    // Yaw 无加速度计修正，只能靠积分，会缓慢漂移
    g_yaw = yaw_gyro;

    // ---------- 4. 角度归一化到 -180° ~ +180° ----------
    // 单位 0.001度，所以边界是 180000
    if (g_roll >  180000) g_roll  -= 360000;
    if (g_roll < -180000) g_roll  += 360000;
    if (g_pitch >  180000) g_pitch -= 360000;
    if (g_pitch < -180000) g_pitch += 360000;
    if (g_yaw >  180000) g_yaw   -= 360000;
    if (g_yaw < -180000) g_yaw   += 360000;
}

//*******************************************************************
// 兼容旧调用：固定按 10mS 周期解算
//*******************************************************************
void update_euler_10ms(int16_t *accel, int16_t *gyro)
{
	update_euler_dt(accel, gyro, DT_MS);
}

//*******************************************************************
/* ===================== 零偏校准 ===================== */
//*******************************************************************
// 上电后静止时调用，采集 N 次求平均作为零偏
//*******************************************************************
void gyro_calibrate(uint16_t *gx_buf, uint16_t *gy_buf, uint16_t *gz_buf, int n)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < n; i++) {
        sum_x += (int16_t)gx_buf[i];
        sum_y += (int16_t)gy_buf[i];
        sum_z += (int16_t)gz_buf[i];
    }
    g_gyro_offset_x = (int16_t)(sum_x / n);
    g_gyro_offset_y = (int16_t)(sum_y / n);
    g_gyro_offset_z = (int16_t)(sum_z / n);
}

//*******************************************************************
/**
 * @brief 获取Gyro传感器零偏
 */
//*******************************************************************
void get_GyroSensor_offset(void)
{
    int32_t sum_accel[3] = {0};
    int32_t sum_gyro[3] = {0};
    uint16_t i;
//    int32_t accel_roll_avg, accel_pitch_avg;
    t3488_data_fifo_Prepare();		//将数据压满FIFO

    // 采集256组数据求平均（降低噪声影响）
    for (i = 0; i < 256; i++) {
        //t3488_gyro_read_all(accel, gyro);
    	t3488_data_filter_handler(accel, gyro);
        sum_accel[0] += accel[0];
        sum_accel[1] += accel[1];
        sum_accel[2] += accel[2];
        sum_gyro[0] += gyro[0];
        sum_gyro[1] += gyro[1];
        sum_gyro[2] += gyro[2];
        sleep_us(1250);			//800Hz 采集时间
    }

    // 计算平均值
    int32_t accel_avg[3];
    int32_t gyro_avg[3];
    for (i = 0; i < 3; i++) {
        accel_avg[i] = sum_accel[i] /256;
        gyro_avg[i]  = sum_gyro[i]  /256;
    }

    g_gyro_offset_x = (int16_t)gyro_avg[0];
    g_gyro_offset_y = (int16_t)gyro_avg[1];
    g_gyro_offset_z = (int16_t)gyro_avg[2];

}

//*******************************************************************
/* ===================== 获取接口 ===================== */
//*******************************************************************
void get_euler_angle(int32_t *roll, int32_t *pitch, int32_t *yaw)
{
    *roll  = g_roll;
    *pitch = g_pitch;
    *yaw   = g_yaw;
}

//*******************************************************************
//*******************************************************************
//关键说明
//1. 精度取舍
//GYRO_INTEG_FACTOR = 6 意味着每 LSB 在 10ms 内产生 0.006° 的角度变化。真实值是 0.006098°，取整误差约 1.6%，长时间积分会有累积偏差。
//如果精度不够，可以：将角度单位改为 0.0001度（即 10000 = 1.000°），则系数为 60（真实值 60.98），误差降到 1.6% 以下但累加精度更高；
//或者用 int64_t 做内部累加，每 N 次循环再舍入一次。
//
//2. 加速度计公式的坐标系适配
//当前代码中：
//roll_acc = int_atan2(ay, az) —- 假设 Roll 是绕 X 轴旋转，重力在 Y/Z 平面的分量决定倾角；
//pitch_acc = int_atan2(-ax, az) —- 这是简化公式，在小角度和 Roll 较小时可用。
//如果你实测发现 Roll 和 Pitch 正负号反了，只需在 int_atan2 的参数前加负号即可。务必用实际静止倾斜测试校准符号。
//
//3. 零偏必须校准
//Yaw 角完全依赖陀螺仪 Z 轴积分。如果 g_gyro_offset_z 不准，Yaw 会以恒定速率漂移。上电静止采集 200~500 次 求平均是比较常规的做法。
//
//4. 除法优化
//int_atan2 中的 (abs_y << 6) / abs_x 是整数除法，某些 MCU 上较慢。如果担心速度，可以：
//改用乘法逆元近似，但会引入额外误差；
//或减小查表分辨率（如 32 点表），降低对除法精度的要求；
//多数 Cortex-M0+ 在几十 MHz 下，10ms 内完成这些运算绰绰有余，不必过度优化。
//
//5. T-3488 数据手册确认项
//以下参数需要你用 T-3488 实际数据手册确认：
//2000dps 档位的灵敏度（是否也是 16.4 LSB/°/s？）
//6g 档位加速度计的 LSB/g（本代码中加速度计只用于 atan2 的分子分母，比值消除了量纲影响，所以加速度计灵敏度不影响角度计算，只要三轴一致即可）
//原始数据格式：是否以 32768 为零点？还是以 0 为零点？这影响 (int16_t) 强转的符号处理。
//*******************************************************************
int16_t s16AccX, s16AccY, s16AccZ, s16GyroX, s16GyroY, s16GyroZ, s16Roll, s16Pitch, s16Yaw;
//*******************************************************************
void ConvertT3488ParaToS16(void)
{
	int32_t temp;
	s16AccX = accel[0];
	s16AccY = accel[1];
	s16AccZ = accel[2];
	s16GyroX = gyro[0];
	s16GyroY = gyro[1];
	s16GyroZ = gyro[2];
	//分辨率0.001 => 0.01
	temp = g_roll/10;
	s16Roll = (int16_t)temp;
	temp = g_pitch/10;
	s16Pitch = (int16_t)temp;
	temp = g_yaw/10;
	s16Yaw = (int16_t)temp;
}

//*******************************************************************
/**
  * @brief 发送数据至匿名上位机 V2.6
  * @param fun 功能字 (0xA0~0xAF)
  * @param data 数据缓冲区指针
  * @param len 数据长度
  */
//*******************************************************************
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    uint8_t sum = 0;

    send_buf[0] = 0x88;      // 帧头
    send_buf[1] = fun;       // 功能字
    send_buf[2] = len;       // 数据长度

    for (i = 0; i < len; i++) {
        send_buf[3 + i] = data[i];
        sum += data[i];      // 累加 DATA
    }

    // 校验 SUM = 0x88 + FUN + LEN + DATA
    sum += 0x88 + fun + len;
    send_buf[3 + len] = sum;

    // 通过串口发送 send_buf
    user_uart_send(send_buf, (len+4));

}

//*******************************************************************
//*******************************************************************
void usart1_data_report(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 roll,s16 pitch,s16 yaw)
{
	uint8_t tbuf[28];

	memset(tbuf, 0, 28);
	tbuf[0]=(accx>>8)&0XFF;
	tbuf[1]=accx&0XFF;
	tbuf[2]=(accy>>8)&0XFF;
	tbuf[3]=accy&0XFF;
	tbuf[4]=(accz>>8)&0XFF;
	tbuf[5]=accz&0XFF;
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	/// tbuf[12]~[16] = magxyz, ///
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	/// tbuf[24]~[27] = 0x00, ///
	usart1_niming_report(0xAF,tbuf,28);	// 0xAF: 飞控显示帧,

}

//*******************************************************************
//*******************************************************************
void usart1_report_EulerDat(void)
{
	uint8_t tbuf[28];

	//ConvertT3488ParaToS16();
	memset(tbuf, 0, 28);
	tbuf[0]=(s16AccX>>8)&0XFF;
	tbuf[1]=s16AccX&0XFF;
	tbuf[2]=(s16AccY>>8)&0XFF;
	tbuf[3]=s16AccY&0XFF;
	tbuf[4]=(s16AccZ>>8)&0XFF;
	tbuf[5]=s16AccZ&0XFF;
	tbuf[6]=(s16GyroX>>8)&0XFF;
	tbuf[7]=s16GyroX&0XFF;
	tbuf[8]=(s16GyroY>>8)&0XFF;
	tbuf[9]=s16GyroY&0XFF;
	tbuf[10]=(s16GyroZ>>8)&0XFF;
	tbuf[11]=s16GyroZ&0XFF;
	/// tbuf[12]~[16] = magxyz, ///
	tbuf[18]=(s16Roll>>8)&0XFF;
	tbuf[19]=s16Roll&0XFF;
	tbuf[20]=(s16Pitch>>8)&0XFF;
	tbuf[21]=s16Pitch&0XFF;
	tbuf[22]=(s16Yaw>>8)&0XFF;
	tbuf[23]=s16Yaw&0XFF;
	/// tbuf[24]~[27] = 0x00, ///
	usart1_niming_report(0xAF,tbuf,28);	// 0xAF: 飞控显示帧,

}

//*******************************************************************
//*******************************************************************

