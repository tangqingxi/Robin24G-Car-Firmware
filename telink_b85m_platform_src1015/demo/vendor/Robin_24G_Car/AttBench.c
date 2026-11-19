/***************************************************************************************
 *  File: AttBench.c
 *
 *  第一版台架姿态估计（定点，无浮点）：
 *      T-3488 直读（12 字节连续读）
 *      → 传感器坐标映射到机体 FRD
 *      → 加速度计六面校准（FRD 零偏/比例）+ 一阶低通
 *      → 陀螺仪扣 FRD 零偏（片内 UI 低通已在芯片内完成）→ Q10 rad/s
 *      → Mahony 四元数互补滤波（Q15）
 *      → 串口文本（约 14Hz 姿态 + 1Hz 统计）
 *
 *  本路径不调用旧欧拉角解算、控制器、混控、遥控连接零偏采集、气压高度估计。
 *  电机输出保持停转（不调用任何 PWM 更新）。
 ****************************************************************************************/
#include "types.h"
#include "app_public_config.h"
#include "driver.h"          // clock_time()
#include "AttConfig.h"
#include "AttBench.h"
#include "AttMahony.h"
#include "my_t3488.h"
#include "UserUart.h"

// =====================================================================================
// 实测参数（来源：data/imu_axis_map_frd.json、data/accel_cal.json、data/gyro_bias_frd.json）
//   float 版本无法链接（工具链无 libgcc），这里换算成整数：
//     加速度零偏：LSB（四舍五入），比例：Q14（1.0 = 16384）
//     陀螺零偏：LSB（四舍五入）
// =====================================================================================
// 1) 轴映射：v_frd = R × v_sensor
//      X_frd = +X_sensor，Y_frd = -Y_sensor，Z_frd = -Z_sensor（行列式 +1）
static const att_i32 s_axis_map[3][3] = {
	{  1,  0,  0 },
	{  0, -1,  0 },
	{  0,  0, -1 },
};

// 2) 加速度校准（机体 FRD）
//      传感器实测：X -2.786/1.003180，Y +8.284/1.002753，Z +28.542/0.998359
//      FRD：offset = R×offset_sensor，scale 只重排
static const att_i32 s_acc_off_lsb[3]   = { -3, -8, -29 };
static const att_i32 s_acc_scale_q14[3] = { 16436, 16429, 16357 };

// 3) 陀螺零偏（机体 FRD，LSB）：-1.1552 / -2.982 / -1.3231
static const att_i32 s_gyro_bias_lsb[3] = { -1, -3, -1 };

// 换算常数
#define ATT_MG_PER_LSB        1000        // 1g = 2048 LSB → mg = lsb*1000/2048
#define ATT_LSB_PER_G         2048
#define ATT_GYRO_Q10_PER_LSB  10896       // 1 LSB = 1.0896 (Q10 rad/s) → ×10896/10000
#define ATT_LPF_TAU_US        5305        // 30Hz 一阶低通时间常数：1/(2π·30)

// =====================================================================================
// 内部状态
// =====================================================================================
#define ATT_TICKS_PER_US   (CLOCK_SYS_CLOCK_HZ / 1000000u)
#define ATT_TICKS_PER_MS   (CLOCK_SYS_CLOCK_HZ / 1000u)
#define ATT_TICKS_PER_100US (CLOCK_SYS_CLOCK_HZ / 10000u)
#define ATT_SAMPLE_TICKS   (CLOCK_SYS_CLOCK_HZ / C_ATT_SAMPLE_HZ)
#define ATT_TX_TICKS       (CLOCK_SYS_CLOCK_HZ / C_ATT_TX_HZ)
#define ATT_STAT_TICKS     (CLOCK_SYS_CLOCK_HZ)            // 1s 一条统计

static att_u8   s_inited;              // 传感器初始化是否完成
static att_u32  s_init_next_tick;      // 初始化失败后的重试时刻
static att_u32  s_next_tick;           // 下一次采样时刻
static att_u32  s_last_tick;           // 上一有效样本的时间戳（tick）
static att_u8   s_have_last;           // 是否已有上一有效样本

static att_mahony_t s_att;
static att_i32  s_acc_lpf_mg[3];       // 加速度一阶低通状态（mg，FRD）
static att_u8   s_lpf_valid;
static att_i32  s_init_acc_mg[3];      // 上电静止样本累加
static att_u16  s_init_cnt;

static att_u32  s_last_tx_tick;        // 上次姿态文本
static att_u32  s_last_stat_tick;      // 上次统计文本

// 统计
static struct {
	att_u32 valid;        // 有效样本数
	att_u32 fail;         // I2C 读失败数
	att_u32 gap;          // 长间断次数
	att_u16 dt_min_us;    // 相邻有效样本间隔
	att_u16 dt_max_us;
	att_u16 rd_min_us;    // 单次 I2C 读取耗时
	att_u16 rd_max_us;
	att_u32 rd_sum_us;
	att_u16 up_min_us;    // 预处理 + 姿态更新耗时
	att_u16 up_max_us;
	att_u32 up_sum_us;
} s_stat;

// =====================================================================================
// 小工具：手工格式化（不用 printf / 浮点）
// =====================================================================================
static char *att_put_str(char *p, const char *s)
{
	while (*s) *p++ = *s++;
	return p;
}

static char *att_put_u32(char *p, att_u32 v)
{
	char tmp[10];
	att_u8 n = 0;

	if (v == 0) {
		*p++ = '0';
		return p;
	}
	while (v && n < sizeof(tmp)) {
		tmp[n++] = (char)('0' + (v % 10));
		v /= 10;
	}
	while (n) *p++ = tmp[--n];
	return p;
}

// 输出带符号定点数：v 的单位是 0.01，例如 1234 → +12.34
static char *att_put_deg100(char *p, att_i32 v)
{
	att_u32 a;

	if (v < 0) {
		*p++ = '-';
		a = (att_u32)(-v);
	} else {
		*p++ = '+';
		a = (att_u32)v;
	}
	p = att_put_u32(p, a / 100);
	*p++ = '.';
	*p++ = (char)('0' + ((a % 100) / 10));
	*p++ = (char)('0' + (a % 10));
	return p;
}

static void att_send(char *p, att_u8 len)
{
	// 忙时跳过本次输出，不等待
	(void)UartTrySend((unsigned char *)p, len);
}

// =====================================================================================
// 初始化：传感器 + 片内陀螺低通（写后回读）
// =====================================================================================
static void AttBench_SensorInit(void)
{
	att_u32 now = clock_time();
	att_u8 lpf_rb = 0;
	att_u8 st;

	if ((att_i32)(now - s_init_next_tick) < 0) {
		return;                     // 还没到重试时刻
	}
	s_init_next_tick = now + CLOCK_SYS_CLOCK_HZ;   // 失败后 1s 再试

	st = t3488_bench_init(C_GYRO_LPF_SEL, &lpf_rb);
	if (st) {
		char *p = (char *)T_txdata_user.data;
		char *q = p;
		q = att_put_str(q, "ATT init fail st=");
		q = att_put_u32(q, st);
		q = att_put_str(q, " lpf_rb=");
		q = att_put_u32(q, lpf_rb);
		*q++ = '\n';
		att_send(p, (att_u8)(q - p));
		return;
	}

	{
		char *p = (char *)T_txdata_user.data;
		char *q = p;
		q = att_put_str(q, "ATT init ok: T3488 16g/2000dps 800Hz, gyro_lpf_sel=");
		q = att_put_u32(q, C_GYRO_LPF_SEL);
		q = att_put_str(q, " (=ODR/16, ~");
		q = att_put_u32(q, (att_u32)C_GYRO_LPF_HZ_NOMINAL);
		q = att_put_str(q, "Hz), readback=");
		q = att_put_u32(q, lpf_rb);
		*q++ = '\n';
		att_send(p, (att_u8)(q - p));
	}

	// 复位状态
	AttMahony_Reset(&s_att);
	s_lpf_valid = 0;
	s_init_cnt = 0;
	s_init_acc_mg[0] = s_init_acc_mg[1] = s_init_acc_mg[2] = 0;
	s_have_last = 0;
	s_stat.valid = s_stat.fail = s_stat.gap = 0;
	s_stat.dt_min_us = 0xFFFF; s_stat.dt_max_us = 0;
	s_stat.rd_min_us = 0xFFFF; s_stat.rd_max_us = 0; s_stat.rd_sum_us = 0;
	s_stat.up_min_us = 0xFFFF; s_stat.up_max_us = 0; s_stat.up_sum_us = 0;

	s_next_tick = clock_time() + ATT_SAMPLE_TICKS;
	s_inited = 1;
}

// =====================================================================================
// 每个采样拍的处理
// =====================================================================================
void AttBench_Process(void)
{
	att_u32 now;
	int16_t acc_raw[3], gyro_raw[3];
	att_i32 acc_frd[3], gyr_frd[3];
	att_i32 acc_mg[3];
	att_i32 dt_100us;
	att_u8 st;
	att_u32 t_read0, t_read1, t_up0, t_up1;
	int i;

	if (!s_inited) {
		AttBench_SensorInit();
		return;
	}

	now = clock_time();
	if ((att_i32)(now - s_next_tick) < 0) {
		return;                     // 还没到下一拍
	}
	s_next_tick += ATT_SAMPLE_TICKS;
	if ((att_i32)(clock_time() - s_next_tick) > 0) {
		s_next_tick = clock_time() + ATT_SAMPLE_TICKS;   // 落后太多就重排
	}

	// ---- 1) I2C 连续读 12 字节（SREG_DATA 0x03）----
	t_read0 = clock_time();
	st = t3488_read_6axis(acc_raw, gyro_raw);
	t_read1 = clock_time();

	{
		att_u16 us = (att_u16)((t_read1 - t_read0) / ATT_TICKS_PER_US);
		if (us < s_stat.rd_min_us) s_stat.rd_min_us = us;
		if (us > s_stat.rd_max_us) s_stat.rd_max_us = us;
		s_stat.rd_sum_us += us;
	}

	if (st) {
		// 读失败：不发布零值、不更新估计，只累加错误计数
		s_stat.fail++;
		return;
	}

	// ---- 2) 预处理 + 姿态更新（计时）----
	t_up0 = clock_time();

	// 映射到机体 FRD（加速度与陀螺共用同一张表）
	for (i = 0; i < 3; i++) {
		acc_frd[i] = s_axis_map[i][0] * (att_i32)acc_raw[0]
		           + s_axis_map[i][1] * (att_i32)acc_raw[1]
		           + s_axis_map[i][2] * (att_i32)acc_raw[2];
		gyr_frd[i] = s_axis_map[i][0] * (att_i32)gyro_raw[0]
		           + s_axis_map[i][1] * (att_i32)gyro_raw[1]
		           + s_axis_map[i][2] * (att_i32)gyro_raw[2];
	}

	// ---- 3) 时间戳与 Δt（回绕由无符号相减处理）----
	if (!s_have_last) {
		s_last_tick = t_read1;
		s_have_last = 1;
		dt_100us = 0;                        // 首拍只建立时间基准
	} else {
		att_u32 dt_ticks = t_read1 - s_last_tick;
		s_last_tick = t_read1;
		dt_100us = (att_i32)(dt_ticks / ATT_TICKS_PER_100US);
		if (dt_100us > C_ATT_MAX_GAP_100US) {
			s_stat.gap++;
			dt_100us = 0;
			s_att.have_prev = 0;             // 间断后不用旧角速度做梯形
		}
	}
	if (dt_100us > 0) {
		att_u16 dt_us = (att_u16)(dt_100us * 100);
		if (dt_us < s_stat.dt_min_us) s_stat.dt_min_us = dt_us;
		if (dt_us > s_stat.dt_max_us) s_stat.dt_max_us = dt_us;
	}

	// ---- 4) 加速度校准（FRD 零偏/比例）→ mg ----
	for (i = 0; i < 3; i++) {
		att_i32 lsb = (acc_frd[i] - s_acc_off_lsb[i]) * s_acc_scale_q14[i] >> 14;
		acc_mg[i] = lsb * ATT_MG_PER_LSB / ATT_LSB_PER_G;
	}

	// ---- 5) 加速度一阶低通（FRD 校准之后）----
	if (!s_lpf_valid) {
		s_acc_lpf_mg[0] = acc_mg[0];
		s_acc_lpf_mg[1] = acc_mg[1];
		s_acc_lpf_mg[2] = acc_mg[2];
		s_lpf_valid = 1;
	} else if (C_ACC_LPF_EN && dt_100us > 0) {
		att_u32 dt_us = (att_u32)dt_100us * 100;
		att_i32 alpha = (att_i32)((dt_us * 32768) / (ATT_LPF_TAU_US + dt_us));   // Q15
		for (i = 0; i < 3; i++) {
			s_acc_lpf_mg[i] += (alpha * (acc_mg[i] - s_acc_lpf_mg[i])) >> 15;
		}
	} else if (!C_ACC_LPF_EN) {
		s_acc_lpf_mg[0] = acc_mg[0];
		s_acc_lpf_mg[1] = acc_mg[1];
		s_acc_lpf_mg[2] = acc_mg[2];
	}
	// 其余情况（dt 为 0）：保持原状态

	// ---- 6) 陀螺仪：扣 FRD 零偏 → Q10 rad/s ----
	for (i = 0; i < 3; i++) {
		att_i32 lsb = gyr_frd[i] - s_gyro_bias_lsb[i];
		gyr_frd[i] = (lsb * ATT_GYRO_Q10_PER_LSB) / 10000;
	}

	// ---- 7) 上电静止样本：平均加速度定向 ----
	if (!s_att.ready) {
		s_init_acc_mg[0] += s_acc_lpf_mg[0];
		s_init_acc_mg[1] += s_acc_lpf_mg[1];
		s_init_acc_mg[2] += s_acc_lpf_mg[2];
		s_init_cnt++;
		if (s_init_cnt >= C_ATT_INIT_SAMPLES) {
			att_i32 avg[3];
			avg[0] = s_init_acc_mg[0] / (att_i32)s_init_cnt;
			avg[1] = s_init_acc_mg[1] / (att_i32)s_init_cnt;
			avg[2] = s_init_acc_mg[2] / (att_i32)s_init_cnt;
			AttMahony_InitFromAccel(&s_att, avg, t_read1 / ATT_TICKS_PER_MS);
		}
	} else if (dt_100us > 0) {
		AttMahony_Update(&s_att, s_acc_lpf_mg, gyr_frd, dt_100us,
		                 t_read1 / ATT_TICKS_PER_MS);
	}

	t_up1 = clock_time();
	{
		att_u16 us = (att_u16)((t_up1 - t_up0) / ATT_TICKS_PER_US);
		if (us < s_stat.up_min_us) s_stat.up_min_us = us;
		if (us > s_stat.up_max_us) s_stat.up_max_us = us;
		s_stat.up_sum_us += us;
	}
	s_stat.valid++;

	// ---- 8) 串口输出（在估计更新之外）----
	now = clock_time();
	if (s_att.ready && (att_u32)(now - s_last_tx_tick) >= ATT_TX_TICKS) {
		char *p = (char *)T_txdata_user.data;
		char *q = p;
		att_i32 roll, pitch, yaw;

		s_last_tx_tick = now;
		AttMahony_GetEulerDeg100(&s_att, &roll, &pitch, &yaw);

		q = att_put_str(q, "ATT t=");
		q = att_put_u32(q, now / ATT_TICKS_PER_MS);
		q = att_put_str(q, " rdy=");
		q = att_put_u32(q, s_att.ready);
		q = att_put_str(q, " r=");
		q = att_put_deg100(q, roll);
		q = att_put_str(q, " p=");
		q = att_put_deg100(q, pitch);
		q = att_put_str(q, " y=");
		q = att_put_deg100(q, yaw);
		*q++ = '\n';
		att_send(p, (att_u8)(q - p));
	}

	if ((att_u32)(now - s_last_stat_tick) >= ATT_STAT_TICKS) {
		char *p = (char *)T_txdata_user.data;
		char *q = p;

		s_last_stat_tick = now;
		q = att_put_str(q, "STA n=");
		q = att_put_u32(q, s_stat.valid);
		q = att_put_str(q, " f=");
		q = att_put_u32(q, s_stat.fail);
		q = att_put_str(q, " g=");
		q = att_put_u32(q, s_stat.gap);
		q = att_put_str(q, " dt=");
		q = att_put_u32(q, s_stat.dt_min_us == 0xFFFF ? 0 : s_stat.dt_min_us);
		*q++ = '/';
		q = att_put_u32(q, s_stat.dt_max_us);
		q = att_put_str(q, " rd=");
		q = att_put_u32(q, s_stat.valid ? (s_stat.rd_sum_us / s_stat.valid) : 0);
		*q++ = '/';
		q = att_put_u32(q, s_stat.rd_max_us);
		q = att_put_str(q, " up=");
		q = att_put_u32(q, s_stat.valid ? (s_stat.up_sum_us / s_stat.valid) : 0);
		*q++ = '/';
		q = att_put_u32(q, s_stat.up_max_us);
		*q++ = '\n';
		att_send(p, (att_u8)(q - p));
	}
}
