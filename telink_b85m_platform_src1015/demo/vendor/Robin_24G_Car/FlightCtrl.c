//**********************************************************************************
/*
 * FlightCtrl.c
 *
 *	四轴飞行器 —— 飞行姿态控制程序（机体部分）
 *
 *  Created on: 2026年10月28日
 *      Author: HuangGuangjian
 *
 *	调用方式：由主循环每 FC_CTRL_DT_MS (默认5mS) 调用一次 FlightCtrl_Tick()
 *	内部流程：
 *		① 读陀螺仪（IIC + 中值滤波）
 *		② 姿态解算（update_euler_dt，含可变周期）
 *		③ 摇杆 → 目标角度/角速度/升降速度
 *		④ 角度环 → 角速度环 → 混控；高度PID(单环 v19) → 基础油门
 *		⑤ 一键起降状态机 + 失联/低压/翻机保护
 *		⑥ 输出 4 路马达 PWM
 */
//**********************************************************************************
#include "types.h"
#include "string.h"
#include "app_public_config.h"
#include "app_battery_check.h"
#include "UserAdc.h"
#include "FlightCtrl.h"
#include "UserEuler.h"
#include "UserFilter.h"
#include "my_t3488.h"
#include "User_IIC.h"
#include "UserPwm.h"
#include "UserUart.h"
#include "driver.h"					// ★v25: flash_read_page/flash_write_page（黑匣子）
#include "UserRxdFunc.h"
#include "PressSen_Lpc2623.h"
//**********************************************************************************
//**********************************************************************************
//==================================================================================
// 内部类型定义
//==================================================================================
typedef struct
{
	s16		kp;			// 比例增益 Q10
	s16		ki;			// 积分增益 Q10
	s16		kd;			// 微分增益 Q10
	s32		integ;		// 积分累加器（Q10，即输出单位<<10）
	s32		i_min;		// 积分下限
	s32		i_max;		// 积分上限
	s16		out_min;	// 输出下限
	s16		out_max;	// 输出上限
	s32		prev_meas;	// 上一次测量值（求微分）
	s32		dterm;		// 微分项（一阶低通后）
	u8		d_shift;	// 微分低通系数（>>n）
} fc_pid_t;

//==================================================================================
// 状态机
//==================================================================================
#define FC_ST_DISARM		0		// 上锁：马达停转
#define FC_ST_TAKEOFF		1		// 一键起飞爬升中
#define FC_ST_FLY			2		// 飞行中（定高）
#define FC_ST_LANDING		3		// 降落中

//==================================================================================
// 静态变量
//==================================================================================
static fc_pid_t		pid_roll_a, pid_pitch_a;	// 角度环（滚转/俯仰）
static fc_pid_t		pid_roll,   pid_pitch;		// 角速度环（滚转/俯仰）
static fc_pid_t		pid_yaw;					// 偏航角速度环
static fc_pid_t		pid_vz;						// 升降速度环（积分项=悬停油门）

static u8			fc_state;
static s32			pres_ref;				// 高度零点(气压原始值)
static s32			alt_est_x10;			// 当前高度估计(0.1cm)，融合输出
static s32			alt_target_x10;			// 目标高度(0.1cm)
static s32			vz_est_x10;				// 升降速度估计(0.1cm/S)，融合输出
static s32			vz_cmd_x10;				// 目标升降速度(0.1cm/S)
static s32			alt_hist[FC_VZ_BUF_NUM];// 高度滑动窗口（求升降速度）
static u8			alt_hist_idx;
static u8			alt_hist_cnt;
static u8			pres_last_flag;

//---- ★v19 气压 5 点中值滤波 FIFO（用户建议：同其六轴 PushRfDatToFIFO 写法；
//		初始化时一次性填满；取代旧的野点剔除+一阶低通）----
static s32			pres_fifo[FC_PRES_MED_N];
static u8			pres_fifo_idx;

//---- 2026/11/04 气压+加计互补估计（见 FlightCtrl.h 7.4，治"脉冲爬升"）----
static s32			alt_baro_x10;			// 气压慢通道高度(0.1cm，LPF后)
static s32			alt_fast_x10;			// 惯性快通道高度(0.1cm，由 alt_fast_acc/64 得到)
static s32			alt_fast_acc;			// 快通道高度累加器(0.1cm×64)，1 气压拍 = 1/64 S
										// ★ 必须用累加器：原来直接 alt+=(vz>>6)，
										//   负数右移是向下取整，|vz|<64 时每拍固定 -1，
										//   等于给高度加了个 -6.4cm/S 的单向棘轮
static s32			alt_corr_q;				// 气压对快通道高度的慢修正(Q8，0.1cm×256)
static s32			vz_baro_x10;			// 气压慢通道升降速度(0.1cm/S)
static s32			vz_acc_q;				// 加计垂向速度积分器(0.1cm/S×mS)
static s32			vz_corr_q;				// 气压对加计积分速度的慢修正(Q8，0.1cm/S×256)
static s32			alt_err_dbg;			// 高度误差镜像(0.1cm)，只给调试看
static s32			thrust_lpf;				// 慢速推力(τ≈0.3S)，判断"推力是否正在变化"

static s32			roll_trim_01;			// 滚转配平(0.01°)
static s32			pitch_trim_01;			// 俯仰配平(0.01°)
static s32			trim_acc_roll, trim_acc_pitch;

static u16			land_timer;				// 降落计时(×FC_CTRL_DT_MS)
static u16			takeoff_timer;
static s32			thrust_slew;			// 基础油门斜率限制状态（上锁时归 0）
static s32			yaw_lock_01;			// 无头模式：锁定机头方向(0.01°)

//---- 油门杆 → 目标高度（用户新定义）----
static s32			alt_tgt_acc;			// 目标高度增量累加器(0.1cm×mS)
static u16			ground_timer;			// 下降受阻计时(×FC_CTRL_DT_MS)
static s32			ground_alt_ref;			// 检测窗口起点的高度估计(0.1cm)
static u16			desc_req_timer;			// 「刚下过降指令」的余温计时(×FC_CTRL_DT_MS)

//---- 离地/触地检测（2026/11/03 新增，防桨流污染气压导致高度环失控）----
static u8			fc_airborne;			// =1 已离地（地面阶段高度环不参与）
static s32			a_vert_raw;				// 本拍垂向加速度原始值(0.1cm/S²，未扣零偏)
static s32			a_vert_bias;			// 垂向加速度零偏学习值(0.1cm/S²，见 FlightCtrl.h 7.5)
static u8			bias_seeded;			// =1 零偏已用首个样本播种（见 ②.5）
static s32			a_vert_lpf;				// 垂向加速度低通值【已扣零偏】(0.1cm/S²，静止≈0)
static u16			liftoff_cnt;			// 离地确认计数（a_vert 判据）
static u16			baro_liftoff_cnt;		// ★v28 离地确认计数（气压正速度判据，OR 关系）
static u16			tdcd_cnt;				// 触地确认计数
static s32			tdcd_alt_ref;			// 触地判定窗口起点高度(0.1cm)
static u8			fc_disarm_reason;		// 停桨原因代码（见 FlightCtrl.h 对外接口）
static s16			dis_alt_x10;			// 停桨瞬间高度估计
static s16			dis_tilt_01;			// 停桨瞬间倾角
static u8			dis_rec;				// 停桨现场已记录
static s16			pres_alt_dbg;			// 本拍未滤波高度换算值（调试）
static u16			air_timer;				// 离地后计时(×FC_CTRL_DT_MS)
static u16			lift_quiet_cnt;			// 离地静默窗口剩余拍数（高度环暂不参与）
static s16			thrust_last;			// 上一拍的基础推力（悬停油门学习用）
static s16			liftoff_thrust;			// ★v27 确认离地当拍的实测推力（静默窗地板基准）
static u16			ground_ramp;			// 地面阶段推力斜坡/计时(×FC_CTRL_DT_MS)
static u8			onek_prev;				// 上一拍一键起降键状态（边沿检测）

//---- 悬停漂移自学习（FlightCtrl.h 7.7，2026/11/07 新增）----
static s16			htrim_roll_01;			// 自学习配平 roll(0.01°)：叠加在校准配平之上
static s16			htrim_pitch_01;			// 自学习配平 pitch(0.01°)
static s32			ax_htrim_lpf;			// 机体水平加速度 x 低通(0.1cm/S²，meas 域)
static s32			ay_htrim_lpf;			// 机体水平加速度 y 低通(0.1cm/S²，meas 域)
static u16			htrim_cnt;				// 学习节拍计数(×FC_CTRL_DT_MS)

//---- 水平速度估计 + 漂移阻尼（FlightCtrl.h 7.10，2026/11/09 v16 新增）----
static s32			vx_q;					// 水平速度积分器 x(0.1cm/S×mS，含泄漏)
static s32			vy_q;					// 水平速度积分器 y(0.1cm/S×mS，含泄漏)
static s16			hvel_vx;				// 水平速度估计 x(0.1cm/S，meas 域)
static s16			hvel_vy;				// 水平速度估计 y(0.1cm/S，meas 域)
static s32			hx_out_roll;			// 阻尼器输出的反倾角 roll(0.01°，叠加进目标)
static s32			hx_out_pitch;			// 阻尼器输出的反倾角 pitch(0.01°)
#if (FC_VZ_MANUAL_EN)
static u16			man_land_cnt;			// ★v17: 手动模式专用落地判定计数(×FC_CTRL_DT_MS)
#endif

#if (FC_UART_DBG_EN)
//---- 7.9 高速调试采样（每拍刷新，节拍末打包经 UART 发出，协议见 FlightCtrl.h 7.9）----
static s16			dbg_pitch_meas;			// 姿态估计 pitch(0.01°)
static s16			dbg_rate_y;				// pitch 角速度测量(0.1°/S)
static s16			dbg_vx;					// 水平速度估计 x(0.1cm/S) ★v16
static s16			dbg_vy;					// 水平速度估计 y(0.1cm/S) ★v16
static s16			dbg_diff_p;				// 权限收缩后 pitch 差动(PWM)
static s16			dbg_roll_meas;			// 姿态估计 roll(0.01°) ★v15
static u8			dbg_seq;				// 包序号
#endif

//---- 摇杆组合：解锁 / 水平校准 ----
static u8			fc_unlocked;			// =1 已解锁（允许起飞）
static u8			thr_start_ready;		// =1 解锁后已见全部摇杆释放，允许推油门起飞
static u16			combo_hold;				// 组合保持计时(×FC_CTRL_DT_MS)
static u8			combo_type;				// 0:无 1:解锁组合 2:校准组合
static u8			combo_done;				// 本次组合已生效（防止松手前重复触发）
//	水平校准（非阻塞采样，不能用 UserEuler 的 get_GyroSensor_offset()，那个会阻塞 320mS）
static u8			cal_state;				// 0:空闲 1:采样中 2:完成
static u16			cal_cnt;				// 已采样次数
static s32			cal_sum_x, cal_sum_y, cal_sum_z;
//---- 特技表演：顺时针盘旋 360° ----
static u8			stunt_active;			// =1 盘旋进行中
static u16			stunt_timer;			// 盘旋计时(×FC_CTRL_DT_MS)
static s32			stunt_yaw_acc;			// 已转过的角度累计(0.01°，取绝对值)
static s32			stunt_yaw_prev;			// 上一拍的 s16Yaw

// 调试观察量
u8	FlightCtrl_State;
u8	FlightCtrl_Unlocked;
u8	FlightCtrl_CalState;
u8	FlightCtrl_Stunt;
u16	FlightCtrl_DescBlock;
u8	FlightCtrl_Airborne;
s16	FlightCtrl_AVert;
s16	FlightCtrl_AccBiasX10;		// 垂向加速度零偏学习值（BDT 观察：解锁前应收敛稳定）
s16	FlightCtrl_OutRoll;
s16	FlightCtrl_OutPitch;
s16	FlightCtrl_OutYaw;
s16	FlightCtrl_Thrust;
s16	FlightCtrl_AltX10;
s16	FlightCtrl_VzX10;
s16	FlightCtrl_VzBaroX10;		// 气压慢通道升降速度（与 VzX10 对比看融合效果）
s16	FlightCtrl_HTrimR01;		// 悬停漂移自学习配平 roll(0.01°)
s16	FlightCtrl_HTrimP01;		// 悬停漂移自学习配平 pitch(0.01°)
s16	FlightCtrl_VzAccX10;		// 加计快通道速度（与 VzX10 差值 = 气压纠漂量）
s16	FlightCtrl_VzCmdX10;		// 升降速度指令
s16	FlightCtrl_AltErrX10;		// 高度误差（已过死区）
s32	FlightCtrl_AltTargetX10;
s32	FlightCtrl_PresRaw;
s16	FlightCtrl_PresAltX10;
u16	FlightCtrl_PresCnt;
u16	FlightCtrl_PresDrop;
u8	FlightCtrl_DisarmReason;
s16	FlightCtrl_DisarmAltX10;
s16	FlightCtrl_DisarmTilt;
//---- ★v23 解锁诊断镜（排查 v20~v22 无法解锁，BDT 直接观察）----
u16	FlightCtrl_TickCnt;			// 每节拍 +1：不动 = 5mS 节拍死了（主循环被挂起）
u8	FlightCtrl_ComboType;		// 摇杆组合实时值 0无 1解锁 2校准（按住组合时应=1）
u16	FlightCtrl_ComboHold;		// 组合保持计数(×5mS)，到 100(0.5S) 即解锁；卡在 0~99 = 组合判定抖动
u16	FlightCtrl_NoRxdPeak;		// 开机以来 u32NoRxdCnt 峰值：>100 = 期间发生过"1S 无包"假失联

u8	FlightCtrl_BBState;			// ★v26 黑匣子状态 0就绪 1录制 2读出 3满 4区域准备
u16	FlightCtrl_BBRecs;			// ★v26 已录制条数（×20mS = 录制时长）
u32	FlightCtrl_BBWRAddr;		// ★v26 写指针（诊断：是否随飞行增长）
u8	FlightCtrl_BBDiag;			// ★v26 区域准备自检位标志（见 FlightCtrl.h）
u8	FlightCtrl_BBChk0;			// 准备前 0x10000 首字节
u8	FlightCtrl_BBChk1;			// 擦除后 0x10000 首字节
u8	FlightCtrl_BBChk2;			// 写头后 0x10000 首字节
u8	FlightCtrl_BBChk3;			// 写头后 0x10000 第 4 字节
u8	FlightCtrl_BBFlashCap;		// flash 容量码

// 气压计驱动里的原始值（PressSen_Lpc2623.c）
extern int32_t	s32PresValue;

// 气压计新数据标志（在 PressSen_Lpc2623.c 中置位）
extern u8 u8PresNewFlag;
// 未收到遥控数据的计时（×10mS，在 UserRxdFunc.c 中累加）
extern unsigned int u32NoRxdCnt;

//==================================================================================
// 工具函数
//==================================================================================
static s32 fc_clamp(s32 v, s32 lo, s32 hi)
{
	if(v > hi) v = hi;
	if(v < lo) v = lo;
	return v;
}

//**********************************************************************************
// PID 计算：err = 目标 - 反馈；meas = 反馈（用于微分项，避免设定值突变引起微分冲击）
// 输出单位由增益决定（角度环输出 0.1°/S，其它环输出 PWM）
//**********************************************************************************
static s16 fc_pid_calc(fc_pid_t *p, s32 err, s32 meas)
{
	s32 p_term, d_term, out, integ_new;

	p_term = err * (s32)p->kp;							// Q10

	integ_new = p->integ + err * (s32)p->ki;			// Q10
	if(integ_new > p->i_max) integ_new = p->i_max;
	if(integ_new < p->i_min) integ_new = p->i_min;

	d_term = 0;
	if(p->kd)
	{
		s32 d_raw = meas - p->prev_meas;
		p->dterm += (d_raw - p->dterm) >> p->d_shift;	// 一阶低通
		d_term = p->dterm * (s32)p->kd;					// Q10
	}
	p->prev_meas = meas;

	out = (p_term + integ_new - d_term) >> 10;
	if(out > p->out_max) out = p->out_max;
	if(out < p->out_min) out = p->out_min;

	// 抗积分饱和：输出已饱和且误差继续同向时，冻结积分
	if( ((out >= p->out_max) && (err > 0)) || ((out <= p->out_min) && (err < 0)) )
	{
		// 不更新积分
	}
	else
	{
		p->integ = integ_new;
	}
	return (s16)out;
}

//**********************************************************************************
// 装载一组 PID 参数
//**********************************************************************************
static void fc_pid_load(fc_pid_t *p, s16 kp, s16 ki, s16 kd, s32 i_min, s32 i_max, s16 out_min, s16 out_max, u8 d_shift)
{
	p->kp = kp;
	p->ki = ki;
	p->kd = kd;
	p->i_min = i_min;
	p->i_max = i_max;
	p->integ = 0;
	p->out_min = out_min;
	p->out_max = out_max;
	p->prev_meas = 0;
	p->dterm = 0;
	p->d_shift = d_shift;
}

//**********************************************************************************
// 陀螺仪原始值(LSB, 16.4LSB/(°/S)) → 角速度(0.1°/S)
//		1 LSB = 1/16.4 °/S = 0.60976 (0.1°/S)  →  (625 >> 10) = 0.6104
//**********************************************************************************
static s32 fc_gyro_to_rate01(s16 raw, s16 offset, s8 sign)
{
	s32 v = (s32)raw - (s32)offset;
	return ((sign * v) * 625) >> 10;
}

//**********************************************************************************
// 加速度原始值(LSB, 2048 LSB/g @±16g) → 比力(0.1cm/S²)
//		1 LSB = 981/2048 cm/S² = 4.7899 (0.1cm/S²)  →  (2452 >> 9) = 4.7891
//**********************************************************************************
static s32 fc_acc_to_01(s16 raw)
{
	return ((s32)raw * 2452) >> 9;
}

//**********************************************************************************
// sin(角度) —— 角度单位 0.01°，返回 Q15
// 采用 sin(x) ≈ x - x³/6，输入限幅 ±60°，误差 <1.5%
//**********************************************************************************
static s32 fc_sin_q15(s32 ang_01)
{
	s32 r, r2, r3;

	ang_01 = fc_clamp(ang_01, -6000, 6000);
	r  = (ang_01 * 5857) >> 10;					// 弧度 Q15 (0.01° → rad: ×5.7196e-5×32768)
	r2 = (r * r) >> 15;							// Q15
	r3 = (r2 * r) >> 15;						// Q15
	return (r - r3/6);
}

#if (FC_HEADLESS_EN)
static s32 fc_cos_q15(s32 ang_01)
{
	return fc_sin_q15(9000 - ang_01);
}
#endif

//**********************************************************************************
// cos(roll)*cos(pitch) 近似：cos ≈ 1 - (φ²+θ²)/2（Q15）
//		(roll_01² + pitch_01²) × 4.99e-4
//**********************************************************************************
static s32 fc_cos_tilt_q15(s32 roll_01, s32 pitch_01)
{
	s32 r2 = roll_01*roll_01 + pitch_01*pitch_01;
	s32 d  = ((r2 >> 10) * 523) >> 10;
	s32 c  = 32768 - d;
	if(c < 0) c = 0;
	return c;
}

//**********************************************************************************
// ★v19 气压 5 点中值滤波（用户建议：与其六轴 PushRfDatToFIFO 同款思路）
//	FIFO 在 FlightCtrl_Init 时一次性填满（正式使用前采样 5 次），此后每个
//	气压拍推入新样本、取 5 个样本的中位数输出。
//**********************************************************************************
static s32 fc_pres_median5(void)
{
	s32 t[FC_PRES_MED_N];
	s32 i, j, key;

	for(i = 0; i < FC_PRES_MED_N; i++)	t[i] = pres_fifo[i];
	for(i = 1; i < FC_PRES_MED_N; i++)			// 插入排序（5 元素，开销可忽略）
	{
		key = t[i];
		j = i - 1;
		while((j >= 0) && (t[j] > key))
		{
			t[j+1] = t[j];
			j--;
		}
		t[j+1] = key;
	}
	return t[FC_PRES_MED_N / 2];				// 中位数
}

//==================================================================================
// 气压 + 加速度 → 高度 / 升降速度（2026/11/04 改为互补融合，见 FlightCtrl.h 7.4）
//==================================================================================
//**********************************************************************************
// 由最新气压采样刷新高度与升降速度
//	慢通道（气压）：alt_baro = (s32PresValue-pres_ref)×10/FC_PRES_PER_CM，LPF 后
//		再窗口差分 + 低通得 vz_baro —— 无漂移，但滞后且被桨流污染（推力↑→表观↓）
//	快通道（加计）：vz_acc_q 每 5mS 积分 a_vert_lpf，推力脉冲骗不了它
//	融合：vz_est = vz_acc + vz_corr（气压速度慢慢纠正加计零偏）
//		      alt_est = alt_fast + alt_corr（气压高度慢慢纠正惯性积分漂移）
//**********************************************************************************
static void fc_altitude_update(void)
{
	s32 d, vz_raw, new_alt, med_raw;
#if (FC_VZ_ACC_FUSE_EN)
	s32 vz_acc, err;
	u8  corr_open;
	static u16 corr_cnt;					// ★v16: 推力连续稳定的拍数（纠漂防抖）
#endif

	// 调试镜像：本拍未滤波的换算值（与 FlightCtrl_AltX10 对比即可看出
	// 气压噪声/跳点有多大；FlightCtrl_PresRaw 则是最底层原始计数）
	pres_alt_dbg = (s16)fc_clamp(((s32)(s32PresValue - pres_ref) * 10) / (s32)FC_PRES_PER_CM,
								 -32768, 32767);
	if(FlightCtrl_PresCnt < 60000) FlightCtrl_PresCnt++;

	// ---- ★v19 5 点中值滤波（取代野点剔除 + 一阶低通，见 FlightCtrl.h 7.1③/7.2）----
	//	FIFO 已在 FlightCtrl_Init 一次性填满（用户要求：正式使用前采样 5 次）。
	//	中值对 I2C 坏点/桨流毛刺天然免疫：5 个样本混入 1~2 个坏点不影响输出；
	//	对缓变信号中值≈均值 → 兼具平滑效果且无低通的相位滞后。
	pres_fifo[pres_fifo_idx] = s32PresValue;		// 推入新样本（循环 FIFO）
	pres_fifo_idx++;
	if(pres_fifo_idx >= FC_PRES_MED_N) pres_fifo_idx = 0;
	med_raw = fc_pres_median5();					// 中位数（原始计数域）

	new_alt = ((med_raw - pres_ref) * 10) / (s32)FC_PRES_PER_CM;

	// 毛刺统计（仅观察，不再丢样本）：中值输出与上拍差 > 8cm 说明气压读数在乱跳
	if(((new_alt - alt_baro_x10) > FC_PRES_GLITCH) ||
	   ((alt_baro_x10 - new_alt) > FC_PRES_GLITCH))
	{
		if(FlightCtrl_PresDrop < 60000) FlightCtrl_PresDrop++;
	}

	// ---- 气压慢通道（中值输出直接作高度，无低通滞后） ----
	alt_baro_x10 = new_alt;

	alt_hist[alt_hist_idx] = alt_baro_x10;
	alt_hist_idx++;
	if(alt_hist_idx >= FC_VZ_BUF_NUM) alt_hist_idx = 0;
	if(alt_hist_cnt < FC_VZ_BUF_NUM) alt_hist_cnt++;

	if(alt_hist_cnt >= FC_VZ_BUF_NUM)
	{
		// 窗口时间 = FC_VZ_BUF_NUM × FC_PRES_SAMPLE_US (uS)
		// vz(0.1cm/S) = Δalt(0.1cm) × 1000000 / 窗口时间(uS)
		d = alt_baro_x10 - alt_hist[alt_hist_idx];		// 最老的一个数据
		vz_raw = (d * (s32)1000000L) / ((s32)FC_VZ_BUF_NUM * (s32)FC_PRES_SAMPLE_US);
		vz_baro_x10 += (vz_raw - vz_baro_x10) >> FC_VZ_LPF_SHIFT;
	}

#if (FC_VZ_ACC_FUSE_EN)
	// ---- 互补融合 ----
	vz_acc = vz_acc_q / 1000;						// 0.1cm/S×mS → 0.1cm/S
	vz_acc = fc_clamp(vz_acc, -FC_VZ_ACC_LIMIT, FC_VZ_ACC_LIMIT);

	// ★ 纠漂门控（2026/11/04 第三轮）：推力一变，桨流把气压"抬/压"的量立刻跟着变，
	//   此刻由气压算出来的升降速度 100% 是假信号（台架实测：马达一起转，
	//   气压就阶跃 -9cm，等效 -4~-18cm/S 的假下降）。门关着时完全不给气压，
	//   只信加计积分 —— 这是切断"推力↑→假下降→加推力"正反馈的唯一位置。
	// ★ v16 防抖：推力须【连续】稳定 FC_VZ_CORR_DEBOUNCE 拍(2S)才真正开门——
	//   即用户要求的"起飞成功、气压稳定后才引入气压"；起飞/机动段纯加计快通道。
	corr_open = (abs((s32)thrust_last - thrust_lpf) <= FC_VZ_CORR_GATE) ? 1 : 0;
	if(corr_open)
	{
		if(corr_cnt < 60000) corr_cnt++;
	}
	else
	{
		corr_cnt = 0;
	}
	if(corr_cnt < (u16)FC_VZ_CORR_DEBOUNCE) corr_open = 0;

	// 速度：以加计积分（快通道）为主，气压只做极慢纠漂
	//	★ 用 Q8 累加器：若直接 vz_corr += err>>8，err 小于 256 时每拍都截成 0，
	//	  小误差永远修不动（原来 >>4 也是同一类问题，只是没那么明显）。
	if(corr_open)
	{
		err = vz_baro_x10 - (vz_acc + (vz_corr_q >> 8));
		vz_corr_q += (err * 256) >> FC_VZ_CORR_SHIFT;
		vz_corr_q = fc_clamp(vz_corr_q, -(FC_VZ_CORR_LIMIT << 8), (FC_VZ_CORR_LIMIT << 8));
	}
	vz_est_x10 = vz_acc + (vz_corr_q >> 8);
	vz_est_x10 = fc_clamp(vz_est_x10, -FC_VZ_EST_LIMIT, FC_VZ_EST_LIMIT);

	// 高度：融合速度积分（每气压拍 = 1/64 S），气压高度只做极慢纠漂
	alt_fast_acc += vz_est_x10;
	alt_fast_acc = fc_clamp(alt_fast_acc, -(20000L*64), (20000L*64));
	alt_fast_x10 = alt_fast_acc / 64;				// 除法=向零截断，无正负棘轮

	if(corr_open)
	{
		err = alt_baro_x10 - (alt_fast_x10 + (alt_corr_q >> 8));
		alt_corr_q += (err * 256) >> FC_ALT_CORR_SHIFT;
		alt_corr_q = fc_clamp(alt_corr_q, -(FC_ALT_CORR_LIMIT << 8), (FC_ALT_CORR_LIMIT << 8));
	}
	alt_est_x10 = alt_fast_x10 + (alt_corr_q >> 8);
#else
	vz_est_x10 = vz_baro_x10;
	alt_est_x10 = alt_baro_x10;
#endif
}

//**********************************************************************************
// 垂向加速度（世界系，去除重力），单位 0.1cm/S²
//		a_z_world = -ax·sin(pitch) + ay·sin(roll) + az·cos(roll)·cos(pitch)
//		静止时该值恒为 0（与姿态无关），因此可直接用作阻尼/加速度反馈
//**********************************************************************************
static s32 fc_vert_accel_01(void)
{
	s32 ax, ay, az, sp, sr, cp, ct;
	s32 a_vert;

	ax = fc_acc_to_01((s16)(FC_ACC_SIGN_X  * (s32)s16AccX));
	ay = fc_acc_to_01((s16)(FC_ACC_SIGN_Y  * (s32)s16AccY));
	az = fc_acc_to_01((s16)(FC_ACC_SIGN_Z  * (s32)s16AccZ));

	sp = fc_sin_q15((s32)s16Pitch);			// sin(pitch)
	sr = fc_sin_q15((s32)s16Roll);			// sin(roll)
	cp = fc_cos_tilt_q15((s32)s16Roll, (s32)s16Pitch);

	ct = ((az * cp) >> 15);
	a_vert = ct - ((ax * sp) >> 15) + ((ay * sr) >> 15) - 9810;		// 减重力 981cm/S²
	return a_vert;
}

//==================================================================================
// 对外接口
//==================================================================================
//**********************************************************************************
// 装载全部 PID 参数、清零状态
//**********************************************************************************
static void fc_pid_init_all(void)
{
	// 角度环：输入 0.01°，输出 0.1°/S（pitch 外环独立放软，见 FlightCtrl.h 7.8）
	fc_pid_load(&pid_roll_a,  FC_ANG_KP_ROLL_Q10, FC_ANG_KI_ROLL_Q10, FC_ANG_KD_ROLL_Q10,
				-FC_ANG_I_LIMIT_ROLL, FC_ANG_I_LIMIT_ROLL, -FC_ANG_RATE_LIMIT, FC_ANG_RATE_LIMIT, 3);
	// ★ v32: pitch 外环 Ki/Kd/I_LIMIT 独立（原共用 roll 的 FC_ANG_KI_Q10 等无后缀参数）
	fc_pid_load(&pid_pitch_a, FC_ANG_KP_PITCH_Q10, FC_ANG_KI_PITCH_Q10, FC_ANG_KD_PITCH_Q10,
				-FC_ANG_I_LIMIT_PITCH, FC_ANG_I_LIMIT_PITCH, -FC_ANG_RATE_LIMIT, FC_ANG_RATE_LIMIT, 3);

	// 角速度环：输入 0.1°/S，输出 PWM（pitch 内环独立整定：降 Kp/加大 D/放宽 I 限幅）
	fc_pid_load(&pid_roll,  FC_RATE_KP_Q10, FC_RATE_KI_Q10, FC_RATE_KD_Q10,
				-FC_RATE_I_LIMIT, FC_RATE_I_LIMIT, -FC_RATE_OUT_LIMIT, FC_RATE_OUT_LIMIT, 3);
	fc_pid_load(&pid_pitch, FC_RATE_KP_PITCH_Q10, FC_RATE_KI_PITCH_Q10, FC_RATE_KD_PITCH_Q10,
				-FC_RATE_I_LIMIT_PITCH, FC_RATE_I_LIMIT_PITCH, -FC_RATE_OUT_LIMIT, FC_RATE_OUT_LIMIT, 3);

	// 偏航角速度环
	fc_pid_load(&pid_yaw,   FC_YAW_KP_Q10, FC_YAW_KI_Q10, FC_YAW_KD_Q10,
				-FC_YAW_I_LIMIT, FC_YAW_I_LIMIT, -FC_YAW_OUT_LIMIT, FC_YAW_OUT_LIMIT, 3);

	// 升降速度环：积分项初值 = 悬停油门（积分项即"自适应悬停油门"）
	fc_pid_load(&pid_vz,    FC_VZ_KP_Q10, FC_VZ_KI_Q10, FC_VZ_KD_Q10,
				FC_VZ_I_MIN, FC_VZ_I_MAX, FC_VZ_OUT_MIN, FC_VZ_OUT_MAX, FC_ACC_LPF_SHIFT);
	pid_vz.integ = ((s32)FC_PWM_HOVER) << 10;
}

//**********************************************************************************
// 上电初始化
//**********************************************************************************
void FlightCtrl_Init(void)
{
	fc_state = FC_ST_DISARM;
	fc_pid_init_all();
	BlackBox_Init();			// ★v26：黑匣子进入"区域准备"态（上电自检/条件擦除/写头）

	pres_ref = s32PresValue;
	alt_est_x10 = 0;
	alt_target_x10 = 0;
	vz_est_x10 = 0;
	vz_cmd_x10 = 0;
	alt_hist_idx = 0;
	alt_hist_cnt = 0;
	memset(alt_hist, 0, sizeof(alt_hist));
	pres_last_flag = 0;
	// 互补估计通道（见 FlightCtrl.h 7.4）
	alt_baro_x10 = 0;
	alt_fast_x10 = 0;
	alt_fast_acc = 0;
	alt_corr_q = 0;
	vz_baro_x10 = 0;
	vz_acc_q = 0;
	vz_corr_q = 0;

	roll_trim_01 = 0;
	pitch_trim_01 = 0;
	trim_acc_roll = 0;
	trim_acc_pitch = 0;
	htrim_roll_01 = 0;
	htrim_pitch_01 = 0;
	ax_htrim_lpf = 0;
	ay_htrim_lpf = 0;
	htrim_cnt = 0;
	vx_q = 0;					// ★v16: 水平速度估计/阻尼输出清零
	vy_q = 0;
	hvel_vx = 0;
	hvel_vy = 0;
	hx_out_roll = 0;
	hx_out_pitch = 0;
#if (FC_VZ_MANUAL_EN)
	man_land_cnt = 0;			// ★v17: 手动模式落地判定计数清零
#endif

	land_timer = 0;
	takeoff_timer = 0;
	ground_timer = 0;
	ground_alt_ref = 0;
	desc_req_timer = 0;
	alt_tgt_acc = 0;
	onek_prev = (FunCtrlFlag & CarOneKRun) ? 1 : 0;	// 防止上电时标志已置位被当成按键边沿
	yaw_lock_01 = 0;

	//---- 离地/触地检测 ----
	fc_airborne = 0;
	a_vert_raw = 0;
	a_vert_bias = 0;			// 上电/复位后重新播种（见 ②.5），τ≈1.3S 收敛
	bias_seeded = 0;
	a_vert_lpf = 0;
	liftoff_cnt = 0;
	baro_liftoff_cnt = 0;
	tdcd_cnt = 0;
	tdcd_alt_ref = 0;
	ground_ramp = 0;
	// ★v19 气压中值 FIFO 一次性填满（用户要求：正式使用前初始化采样 5 次填满）
	{
		u8 k;
		for(k = 0; k < FC_PRES_MED_N; k++) pres_fifo[k] = s32PresValue;
	}
	pres_fifo_idx = 0;
	air_timer = 0;
	lift_quiet_cnt = 0;
	thrust_last = FC_PWM_HOVER;
	liftoff_thrust = FC_PWM_HOVER;
	thrust_lpf = FC_PWM_HOVER;
	alt_err_dbg = 0;
	//---- 气压健康度 / 停桨现场 ----
	pres_alt_dbg = 0;
	FlightCtrl_PresCnt = 0;
	FlightCtrl_PresDrop = 0;
	fc_disarm_reason = 0;
	dis_alt_x10 = 0;
	dis_tilt_01 = 0;
	dis_rec = 0;

	//---- 解锁 / 校准 / 特技 ----
	fc_unlocked = 0;			// ★上电后处于飞行锁定状态，必须先摇杆解锁
	thr_start_ready = 0;		// 上电未解锁，推油门起飞先禁止
	combo_hold = 0;
	combo_type = 0;
	combo_done = 0;
	cal_state = 0;
	cal_cnt = 0;
	cal_sum_x = 0; cal_sum_y = 0; cal_sum_z = 0;
	stunt_active = 0;
	stunt_timer = 0;
	stunt_yaw_acc = 0;
	stunt_yaw_prev = 0;

	FlightCtrl_State = 0;
	FlightCtrl_Unlocked = 0;
	FlightCtrl_CalState = 0;
	FlightCtrl_Stunt = 0;
	FlightCtrl_DescBlock = 0;
	FlightCtrl_Airborne = 0;
	FlightCtrl_AVert = 0;
	FlightCtrl_OutRoll = 0;
	FlightCtrl_OutPitch = 0;
	FlightCtrl_OutYaw = 0;
	FlightCtrl_Thrust = 0;
	FlightCtrl_AltX10 = 0;
	FlightCtrl_VzX10 = 0;
	FlightCtrl_VzBaroX10 = 0;
	FlightCtrl_VzAccX10 = 0;
	FlightCtrl_VzCmdX10 = 0;
	FlightCtrl_AltErrX10 = 0;
	FlightCtrl_AltTargetX10 = 0;
}

//**********************************************************************************
// 复位（不改动高度零点）
//**********************************************************************************
void FlightCtrl_Reset(void)
{
	fc_state = FC_ST_DISARM;
	alt_target_x10 = 0;			// 复位后目标高度从 0 起步
	alt_tgt_acc = 0;
	ground_timer = 0;
	ground_alt_ref = 0;
	desc_req_timer = 0;
	thr_start_ready = 0;		// 重新对频/复位后需重新松杆+推油门
	//---- 离地/触地检测 ----
	fc_airborne = 0;
	a_vert_raw = 0;				// 零偏 a_vert_bias 保留不Reset：落地上锁后会继续学习刷新
	a_vert_lpf = 0;
	liftoff_cnt = 0;
	baro_liftoff_cnt = 0;
	tdcd_cnt = 0;
	tdcd_alt_ref = 0;
	ground_ramp = 0;
	air_timer = 0;
	lift_quiet_cnt = 0;
	thrust_last = FC_PWM_HOVER;
	liftoff_thrust = FC_PWM_HOVER;
	thrust_lpf = FC_PWM_HOVER;
	alt_err_dbg = 0;
	//---- 停桨现场 ----
	fc_disarm_reason = 0;
	dis_alt_x10 = 0;
	dis_tilt_01 = 0;
	dis_rec = 0;
#if (FC_VZ_MANUAL_EN)
	man_land_cnt = 0;
#endif
	fc_pid_init_all();
	alt_hist_idx = 0;
	alt_hist_cnt = 0;
	vz_est_x10 = 0;
	alt_baro_x10 = 0;
	alt_fast_x10 = 0;
	alt_fast_acc = 0;
	alt_corr_q = 0;
	vz_baro_x10 = 0;
	vz_acc_q = 0;
	vz_corr_q = 0;
	land_timer = 0;
	takeoff_timer = 0;
	stunt_active = 0;
	stunt_timer = 0;
	stunt_yaw_acc = 0;
}

//**********************************************************************************
// 以当前气压值为高度零点
//**********************************************************************************
void FlightCtrl_SetAltRef(void)
{
	pres_ref = s32PresValue;
	alt_est_x10 = 0;
	alt_target_x10 = 0;
	vz_est_x10 = 0;
	alt_hist_idx = 0;
	alt_hist_cnt = 0;
	memset(alt_hist, 0, sizeof(alt_hist));
	// ★v19：pres_fifo 不重置——FIFO 存的是气压【绝对计数】，与 pres_ref 无关，
	//	重清零点只改变换算偏移，中值滤波输出自动跟随，无需重新填满。
	// 互补通道一并归零（加计积分器/慢修正/快慢通道，见 FlightCtrl.h 7.4）
	alt_baro_x10 = 0;
	alt_fast_x10 = 0;
	alt_fast_acc = 0;
	alt_corr_q = 0;
	vz_baro_x10 = 0;
	vz_acc_q = 0;
	vz_corr_q = 0;
}

//==================================================================================
// 主控制节拍
//==================================================================================
//**********************************************************************************
// 四个摇杆是否全部处于释放（回中）状态
//	用于一键降落的条件判断：飞行中只有当所有摇杆都松手才允许一键降落
//**********************************************************************************
u8 FlightCtrl_AllStickReleased(void)
{
	if(abs((s32)s16_Throttle) > FC_THR_DEADBAND)	return 0;
	if(abs((s32)s16_Rotation) > 20)					return 0;
	if(abs((s32)s16_FBmove)   > 20)					return 0;
	if(abs((s32)s16_LRmove)   > 20)					return 0;
	return 1;
}

//**********************************************************************************
// 摇杆组合检测
//	返回：0 = 无组合，1 = 解锁组合，2 = 水平校准组合
//
//	解锁（双摇杆内下角）：  左 下+左(Thr -12, Rot +307)，右 下+右(FB -307, LR +307)
//	校准（双摇杆外下角）：  左 下+右(Thr -12, Rot -307)，右 下+左(FB -307, LR -307)
//**********************************************************************************
static u8 fc_stick_combo(void)
{
	u8 thr_down = (s16_Throttle <= -(s16)FC_END_THR) ? 1 : 0;	// 油门拉到底
	u8 fb_down  = (s16_FBmove   <= -(s16)FC_END_FB)  ? 1 : 0;	// 前后拉到底
	if(!thr_down || !fb_down)	return 0;						// 两个摇杆都必须下拉到底
	if(abs((s32)s16_Rotation) < FC_END_ROT)	return 0;
	if(abs((s32)s16_LRmove)   < FC_END_LR)  return 0;

	if(s16_Rotation > 0)		// 左摇杆向左 → 旋转 = +307
	{
		if(s16_LRmove > 0)		// 右摇杆向右 → 左右 = +307
		{
			return 1;			// 解锁组合
		}
	}
	else						// 左摇杆向右 → 旋转 = -307
	{
		if(s16_LRmove < 0)		// 右摇杆向左 → 左右 = -307
		{
			return 2;			// 水平校准组合
		}
	}
	return 0;
}

//**********************************************************************************
// 水平校准：把飞机静置在水平面上，重新标定陀螺零偏 + 清零配平与姿态积分
//	用飞控自己的 5mS 节拍分次采样（非阻塞）。
//	不能调用 UserEuler.c 的 get_GyroSensor_offset()：它内部 sleep_us(1250)×256
//	会阻塞主循环约 320mS，飞行中调用必然摔机。
//**********************************************************************************
static void fc_horiz_cal_start(void)
{
	cal_state  = 1;
	cal_cnt    = 0;
	cal_sum_x  = 0;
	cal_sum_y  = 0;
	cal_sum_z  = 0;
}

static void fc_horiz_cal_tick(void)
{
	if(cal_state != 1) return;

	cal_sum_x += (s32)s16GyroX;
	cal_sum_y += (s32)s16GyroY;
	cal_sum_z += (s32)s16GyroZ;
	cal_cnt++;

	if(cal_cnt >= FC_CAL_SAMPLE_NUM)
	{
		// 陀螺零偏 = 静止期间的平均输出。
		// 注意：s16GyroX/Y/Z 是【原始值】（ConvertT3488ParaToS16 直接拷贝 gyro[]），
		// 零偏是在 update_euler_dt() 内部才减掉的，所以这里直接取平均，不能再叠加旧零偏。
		g_gyro_offset_x = (s16)(cal_sum_x / (s32)FC_CAL_SAMPLE_NUM);
		g_gyro_offset_y = (s16)(cal_sum_y / (s32)FC_CAL_SAMPLE_NUM);
		g_gyro_offset_z = (s16)(cal_sum_z / (s32)FC_CAL_SAMPLE_NUM);

		// ★ 2026/11/07 关键修复：配平 = 校准时刻的姿态读数，而不是 0！
		//	台架数据（2026/11/07）证明传感器板相对机架有恒定安装倾角/加计零偏：
		//	静置水平面时 s16Roll=-1.14°、s16Pitch=-1.59°（加计反推 -1.25°/-1.79°，
		//	估计器与加计一致，说明滤波器没坏，倾角是"真实存在"的）。
		//	旧代码把配平清零 + 姿态清零 → 解锁后加计修正(τ)很快把估计值拉回
		//	-1.14°/-1.59°，角度环为了把"估计值"压到 0，会把【机架】反向压斜
		//	1.14°/1.59° → 推力矢量恒定偏斜 → 起飞后恒定向左后漂移爬升，
		//	与真机观测的"向左后上方"完全吻合。
		//	现在把校准时刻的读数记入配平：飞行中角度环把姿态保持在"校准面姿态"
		//	→ 传感器板保持其安装倾角 → 机架（桨平面）真正水平 → 无恒定漂移。
		//	（无论倾角来自机械安装还是加计零偏，此法都成立：只要校准面是水平的。）
		roll_trim_01  = fc_clamp(FC_ROLL_SIGN  * (s32)s16Roll,  -800, 800);
		pitch_trim_01 = fc_clamp(FC_PITCH_SIGN * (s32)s16Pitch, -800, 800);
		trim_acc_roll  = 0;
		trim_acc_pitch = 0;

		// 姿态积分：只清 yaw（它无加计锚定会漂移）。
		//	roll/pitch 不再清零——加了配平后它们本来就该停在安装倾角上，
		//	清零反而制造一个 2~3S 的反向瞬态。
		g_yaw = 0;

		// 以当前气压重新设定高度零点（飞机就在地面）
		FlightCtrl_SetAltRef();
		alt_target_x10 = 0;
		vz_est_x10 = 0;

		fc_pid_init_all();
		cal_state = 2;			// 完成
	}
}

//==================================================================================
//**********************************************************************************
// 起飞条件检查
//**********************************************************************************
static u8 fc_can_arm(void)
{
#if (FC_UNLOCK_EN)
	if(fc_unlocked == 0)		return 0;		// 未解锁（上电后必须先摇杆解锁）
#endif
	if(link_ok_flag == 0)			return 0;		// RF 未连接
	if(VddVolFlag & LowBatOffFlag)	return 0;		// 低压关机
	if(abs((s32)s16Roll)  > FC_ARM_TILT_LIMIT)	return 0;	// 机身没放平
	if(abs((s32)s16Pitch) > FC_ARM_TILT_LIMIT)	return 0;
	return 1;
}

//**********************************************************************************
// 「已落住」检测 ★ v19 按用户判定重写
//
// 判据：【拉杆要求下降（want_down），但连续 FC_GROUND_TMOUT(3S) 评估高度
//	几乎没变（窗口内降幅 < FC_GROUND_MIN_DROP）】→ 已落住。
//
// ★ 删除旧判据的"目标高度低于实际 6cm"条件——目标高度允许为负（低于起飞点），
//	目标低只代表要求的高度低，不代表着地；是否"要求下降"只看杆（desc_req）。
//	用「窗口内实际下降量」而不是瞬时速度，是为了躲开气压噪声（±3cm），
//	3S 窗口的差分比 125mS 窗口的差分稳得多。
//	安全性：杆指令最小下降 2cm/S（死区 1 的下一档），3S 降 6cm ≫ 3cm 门槛，
//	正常下降不会被误判；真落住时降幅≈0，3S 内必然触发。
//
// 返回 1 = 判定已落住（调用方应上锁、关马达）
//**********************************************************************************
static u8 fc_ground_detect(u8 want_down)
{
	if(want_down == 0)
	{
		ground_timer = 0;
		ground_alt_ref = alt_est_x10;			// 随时准备开新窗口
		return 0;
	}

	if(ground_timer == 0)
	{
		ground_alt_ref = alt_est_x10;			// 窗口起点
	}

	ground_timer++;
	if(ground_timer < FC_GROUND_TMOUT)	return 0;

	if((ground_alt_ref - (s32)alt_est_x10) < FC_GROUND_MIN_DROP)
	{
		// 一整个 3S 窗口内几乎没降下去 → 落住了
		ground_timer = 0;
		return 1;
	}

	// 仍在正常下降：滚动窗口，重新计一段
	ground_alt_ref = alt_est_x10;
	ground_timer = 0;
	return 0;
}

//**********************************************************************************
// 离地静默窗口结束（2026/11/03 气压计实测后新增）
//	实测（固定支架 64Hz）：马达一转气压读数阶跃 +1412 counts = 表观 -7.1cm，
//	贴地时更大（满推力 -54.6cm 实录）。离地瞬间推力正在最大值附近，此时清
//	气压基准 = 把"最大桨流偏置"当零点；随后高度环把推力压回悬停值、偏置回落
//	→ 估计跳十几厘米 → 高度环再动作 → "快定高又上飞"反复 6~7 次。
//	所以离地后先静默 600mS（升降速度环把爬升压到 0，推力稳定到悬停点），
//	窗口结束才重清气压基准，并把悬停油门学习为此刻的实际推力。
//**********************************************************************************
static void fc_lift_quiet_end(void)
{
	FlightCtrl_SetAltRef();							// 以悬停点为高度零点（偏置已稳定）
	// 悬停油门 = 静默期末的实际推力，但夹紧到 [hover-30, hover+40]：
	// ★v29：V28_1 实录假离地（推力仅 218 就判离地）→ 窗内 vz_acc 正饱和把推力
	//   压到地板 198 → 这里学到假悬停 208，比真值低 42PWM，之后推力只能以
	//   Ki=0.001(≈38PWM/S) 慢爬，贴地憋了 1.2S。夹紧后单次误判最多偏差 30PWM。
	{
		s32 hover_lrn = thrust_last;
		if(hover_lrn < ((s32)FC_PWM_HOVER - 30)) hover_lrn = (s32)FC_PWM_HOVER - 30;
		if(hover_lrn > ((s32)FC_PWM_HOVER + 40)) hover_lrn = (s32)FC_PWM_HOVER + 40;
		pid_vz.integ = hover_lrn << 10;
	}
	pid_vz.dterm = 0;
	// ★v19：目标高度一律从 0 起步（一键起飞随后按 19.2cm/S 把目标积分爬向 2 米，
	//	FC_ST_FLY 则悬停在 0）。不再有 v18"直接锁定在气压读数"的特殊处理。
	alt_target_x10 = 0;
	alt_tgt_acc = 0;
}

//**********************************************************************************
// 7.11 flash 黑匣子（★v26：区域准备自检 + 起飞后才录 + 写头回读校验）
//	详见 FlightCtrl.h 7.11。状态机：
//	  PREP 上电→读首 4B(全FF=已擦/否则需擦)→限速擦 16 扇区→写 16B 区头→回读校验
//	  IDLE 就绪等起飞（上锁后回到此态，写指针保留，可多次起降追加）
//	  REC  已起飞（TAKEOFF/FLY），每 4 拍(20mS)写 1 条 38B
//	  DUMP 停转后按键，从 UART 直读直发
//	静态 RAM 约 40B（RAM 预算见 FlightCtrl.h 7.11 / boot.link 断言）。
//**********************************************************************************
static u8			bb_state;			// FC_BB_ST_*
static u8			bb_prep_step;		// 准备子步：0检测 1擦除 2写头
static u8			bb_prep_sec;		// 已擦扇区数
static u8			bb_prep_gap;		// 擦除限速分频
static u32			bb_wr_addr;			// 下一条记录写入地址
static u16			bb_seq;				// 记录序号
static u8			bb_div_cnt;			// 采样分频计数
static u16			bb_recs;			// 已录制条数
static u32			bb_dump_len;		// 待读出总长（含 16B 区域头）
static u32			bb_dump_off;		// 已读出偏移
static u8			bb_dump_hdr;		// 起始帧已发

void BlackBox_Init(void)				// 上电初始化：进入"区域准备"态
{										//   （static 默认 0 = IDLE，必须显式改 PREP）
	bb_state = FC_BB_ST_PREP;
	bb_prep_step = 0;
	bb_prep_sec = 0;
	bb_prep_gap = 0;
	bb_wr_addr = 0;						// 写头成功后才赋予有效地址
	bb_seq = 0;
	bb_div_cnt = 0;
	bb_recs = 0;
	bb_dump_len = 0;
	bb_dump_off = 0;
	bb_dump_hdr = 0;
	FlightCtrl_BBDiag = 0;
	FlightCtrl_BBChk0 = 0xFF;
	FlightCtrl_BBChk1 = 0xFF;
	FlightCtrl_BBChk2 = 0;
	FlightCtrl_BBChk3 = 0;
	FlightCtrl_BBFlashCap = 0;
	FlightCtrl_BBState = FC_BB_ST_PREP;
	FlightCtrl_BBRecs = 0;
	FlightCtrl_BBWRAddr = 0;
}

void BlackBox_DumpStart(void)			// 特技表演键触发（仅停转状态允许，避免
{										//   飞行中阻塞 UART/flash 影响控制节拍）
	if(fc_state != FC_ST_DISARM) return;
	if(bb_state == FC_BB_ST_DUMP) return;
	if(bb_wr_addr <= (u32)(FC_BB_BASE + 16)) return;	// 一条都没录（防 u32 下溢狂发）
	bb_dump_len = bb_wr_addr - FC_BB_BASE;	// 16B 区域头 + 已录记录
	bb_dump_off = 0;
	bb_dump_hdr = 0;
	bb_state = FC_BB_ST_DUMP;
}

//**********************************************************************************
// ★v27 组包写一条记录（REC 周期记录 与 上锁当拍收尾记录 共用）
//	38B = 17×s16(大端) + seq u16 + flags u8 + reason u8
//	is_final=1 → flags.bit6 置位，表示"停机收尾记录"，此时 bit0=0、reason=停机原因
//	容量不足不写（不覆盖旧数据），直接置 FULL
//**********************************************************************************
static void fc_bb_write_rec(u8 is_final, s32 out_r, s32 out_p, s32 thrust, s32 rate_x, s32 rate_y)
{
	u8 rec[FC_BB_REC_SIZE];
	u8 fl;

	if(bb_wr_addr + FC_BB_REC_SIZE > FC_BB_BASE + FC_BB_SIZE)
	{
		bb_state = FC_BB_ST_FULL;		// 64KB 满（42B/条 @50Hz ≈ 31.2S）
		return;
	}

	rec[0]  = (u8)(alt_est_x10 >> 8);		rec[1]  = (u8)alt_est_x10;
	rec[2]  = (u8)(alt_target_x10 >> 8);	rec[3]  = (u8)alt_target_x10;
	rec[4]  = (u8)(alt_err_dbg >> 8);		rec[5]  = (u8)alt_err_dbg;
	rec[6]  = (u8)(vz_est_x10 >> 8);		rec[7]  = (u8)vz_est_x10;
	rec[8]  = (u8)(vz_baro_x10 >> 8);		rec[9]  = (u8)vz_baro_x10;
	rec[10] = (u8)((vz_acc_q / 1000) >> 8);	rec[11] = (u8)(vz_acc_q / 1000);
	rec[12] = (u8)(a_vert_lpf >> 8);		rec[13] = (u8)a_vert_lpf;
	rec[14] = (u8)(thrust >> 8);			rec[15] = (u8)thrust;
	rec[16] = (u8)(s16Roll >> 8);			rec[17] = (u8)s16Roll;
	rec[18] = (u8)(s16Pitch >> 8);			rec[19] = (u8)s16Pitch;
	rec[20] = (u8)(out_r >> 8);				rec[21] = (u8)out_r;
	rec[22] = (u8)(out_p >> 8);				rec[23] = (u8)out_p;
	rec[24] = (u8)(htrim_roll_01 >> 8);		rec[25] = (u8)htrim_roll_01;
	rec[26] = (u8)(htrim_pitch_01 >> 8);	rec[27] = (u8)htrim_pitch_01;
	rec[28] = (u8)(hvel_vx >> 8);			rec[29] = (u8)hvel_vx;
	rec[30] = (u8)(hvel_vy >> 8);			rec[31] = (u8)hvel_vy;
	rec[32] = (u8)(s16_Throttle >> 8);		rec[33] = (u8)s16_Throttle;
	// ★v34：新增两轴陀螺原始值（内环测量角速度，0.1°/S）——贴墙/翻机等极端
	//	工况下欧拉角无法区分"真实旋转"与"估计迟钝"，原始角速度可一锤定音。
	rec[34] = (u8)(((s16)rate_x) >> 8);		rec[35] = (u8)rate_x;	// roll 角速度
	rec[36] = (u8)(((s16)rate_y) >> 8);		rec[37] = (u8)rate_y;	// pitch 角速度
	rec[38] = (u8)(bb_seq >> 8);			rec[39] = (u8)bb_seq;
	// ★v27 flags 扩展（原 bit0~2，新增 bit3~6，见 FlightCtrl.h 7.11）
	fl = 0;
	if(fc_state != FC_ST_DISARM)			fl |= 0x01;	// bit0 本拍仍解锁
	if(fc_airborne)							fl |= 0x02;	// bit1 已离地
	if(lift_quiet_cnt != 0)					fl |= 0x04;	// bit2 离地静默窗内
	if(VddVolFlag & LowBatOffFlag)			fl |= 0x08;	// bit3 低压关机位
	if((link_ok_flag == 0) || (u32NoRxdCnt > 100))
											fl |= 0x10;	// bit4 失联
	if(tdcd_cnt != 0)						fl |= 0x20;	// bit5 触地判定窗口内
	if(is_final)							fl |= 0x40;	// bit6 收尾记录
	rec[40] = fl;
	rec[41] = fc_disarm_reason;
	flash_write_page(bb_wr_addr, FC_BB_REC_SIZE, rec);
	bb_wr_addr += FC_BB_REC_SIZE;
	bb_seq++;
	bb_recs++;
}

static void fc_blackbox_tick(s32 out_r, s32 out_p, s32 thrust, s32 rate_x, s32 rate_y)	// 每控制节拍(5mS)调用
{
	switch(bb_state)
	{		//---- 上电区域准备：自检 →（必要时）擦除 → 写头 → 回读校验 ----
		case FC_BB_ST_PREP:
			// 只在【停转 + 遥控已连 + 未打摇杆组合】时推进：
			//   擦除会阻塞 30~100mS 并丢 RF 包，必须避开解锁组合的 0.5S 保持期，
			//   否则计时被拉长、解锁失败（v20/v21 的教训之一）。
			if(fc_state != FC_ST_DISARM) break;
			if(link_ok_flag == 0) break;
			if(combo_type != 0) break;

			if(bb_prep_step == 0)			// ① 检测录制区首 4 字节
			{
				u8 chk[4];
				flash_read_page(FC_BB_BASE, 4, chk);
				FlightCtrl_BBChk0 = chk[0];
				FlightCtrl_BBFlashCap = (u8)(((unsigned int)flash_read_mid() >> 16) & 0xFF);
				if(FlightCtrl_BBFlashCap < 0x11)		// <128KB：64KB 录制区越界
					FlightCtrl_BBDiag |= 0x10;
				if((chk[0] & chk[1] & chk[2] & chk[3]) == 0xFF)
				{
					bb_prep_step = 2;			// 已擦好 → 直接写头
				}
				else
				{
					FlightCtrl_BBDiag |= 0x01;	// 非空白（全 0 / 残留旧数据）
#if (FC_BB_AUTO_ERASE)
					bb_prep_step = 1;			// → 固件擦除
					bb_prep_sec = 0;
					bb_prep_gap = 0;
#else
					bb_state = FC_BB_ST_FULL;	// → 只认 BDT 擦除
#endif
				}
				break;
			}

			if(bb_prep_step == 1)			// ② 限速擦除整个录制区（16 扇区）
			{
				if(++bb_prep_gap < (u8)FC_BB_ERASE_GAP) break;
				bb_prep_gap = 0;
				flash_erase_sector(FC_BB_BASE + (u32)bb_prep_sec * 0x1000);
				bb_prep_sec++;
				if(bb_prep_sec >= (u8)(FC_BB_SIZE / 0x1000))
				{
					u8 c2[4];
					flash_read_page(FC_BB_BASE, 4, c2);
					FlightCtrl_BBChk1 = c2[0];
					if((c2[0] & c2[1] & c2[2] & c2[3]) == 0xFF)
						FlightCtrl_BBDiag |= 0x02;	// 擦除后回读全 FF
					bb_prep_step = 2;
				}
				break;
			}

			{								// ③ 写 16B 区域头 + 立即回读校验
				u8 hdr[16];					//   （判断 flash 写到底有没有生效）
				u8 rb[4];
				u8 i;
				hdr[0] = 0xA5;  hdr[1] = 0x5A;
				hdr[2] = 0xBB;  hdr[3] = 0xBB;
				hdr[4] = (u8)FC_PARAM_VER;
				hdr[5] = (u8)(FC_BB_REC_SIZE >> 8);
				hdr[6] = (u8)FC_BB_REC_SIZE;
				hdr[7] = (u8)FC_BB_REC_DIV;
				for(i = 8; i < 16; i++) hdr[i] = 0;
				flash_write_page(FC_BB_BASE, 16, hdr);
				flash_read_page(FC_BB_BASE, 4, rb);
				FlightCtrl_BBChk2 = rb[0];
				FlightCtrl_BBChk3 = rb[3];
				if((rb[0] == 0xA5) && (rb[1] == 0x5A) && (rb[2] == 0xBB) && (rb[3] == 0xBB))
					FlightCtrl_BBDiag |= 0x04;	// 写读一致 → flash 写正常
				else
					FlightCtrl_BBDiag |= 0x08;	// 写失败 → 写保护/地址越界
				bb_wr_addr = FC_BB_BASE + 16;
				bb_seq = 0;
				bb_div_cnt = 0;
				bb_recs = 0;
				bb_state = FC_BB_ST_IDLE;
			}
			break;

		case FC_BB_ST_IDLE:
			// ★v26：真正起飞后才开始记录（TAKEOFF 爬升 / FLY 悬停），
			//   解锁后在地面推油门预备的这段时间不再占容量。
			if((fc_state == FC_ST_TAKEOFF) || (fc_state == FC_ST_FLY))
			{
				if(bb_wr_addr + FC_BB_REC_SIZE > FC_BB_BASE + FC_BB_SIZE)
				{
					bb_state = FC_BB_ST_FULL;		// 已录满
					break;
				}
				bb_div_cnt = 0;
				bb_state = FC_BB_ST_REC;
			}
			break;

		case FC_BB_ST_REC:
			if(fc_state == FC_ST_DISARM)		// 上锁（落地/停桨/失联/低压）即停
			{
				// ★v27：上锁当拍补写一条【收尾记录】。原来这里直接转 IDLE 不写，
				//   于是末条永远是上锁前一拍：reason 恒为 0、flags.bit0/bit1 恒为 1，
				//   两个字段从来没记录过真实信息，停桨原因无从判断（v26 实测）。
				//   收尾记录 flags.bit6=1、bit0=0，reason = 停机原因代码。
				fc_bb_write_rec(1, out_r, out_p, thrust, rate_x, rate_y);
				if(bb_state == FC_BB_ST_REC)
					bb_state = FC_BB_ST_IDLE;	// 写指针保留，可再次起飞追加
				break;
			}
			if(++bb_div_cnt < (u8)FC_BB_REC_DIV) break;
			bb_div_cnt = 0;
			fc_bb_write_rec(0, out_r, out_p, thrust, rate_x, rate_y);
			break;

		case FC_BB_ST_DUMP:
			if(bb_dump_hdr == 0)
			{	// 起始帧 12B：26 99 A5 5A | 总长 u32 BE | rec_size u16 BE | ver | div
				u8 st[12];
				u32 total = bb_wr_addr - FC_BB_BASE;
				st[0] = 0x26;  st[1] = 0x99;
				st[2] = 0xA5;  st[3] = 0x5A;
				st[4] = (u8)(total >> 24);  st[5] = (u8)(total >> 16);
				st[6] = (u8)(total >> 8);   st[7] = (u8)total;
				st[8] = (u8)(FC_BB_REC_SIZE >> 8);
				st[9] = (u8)FC_BB_REC_SIZE;
				st[10] = (u8)FC_PARAM_VER;
				st[11] = (u8)FC_BB_REC_DIV;
				UartBBSend(st, 12);
				bb_dump_hdr = 1;
				break;
			}
			if(bb_dump_off < bb_dump_len)
			{	// flash 原始数据直读直发（32B/块，阻塞防乱序）
				u32 n = bb_dump_len - bb_dump_off;
				if(n > (u32)FC_BB_CHUNK) n = (u32)FC_BB_CHUNK;
				UartBBSendFlash(FC_BB_BASE + bb_dump_off, (u8)n);
				bb_dump_off += n;
			}
			else
			{	// 结束帧 4B：26 99 5A A5；数据保留在 flash，可重复按键读出
				u8 end[4];
				end[0] = 0x26;  end[1] = 0x99;
				end[2] = 0x5A;  end[3] = 0xA5;
				UartBBSend(end, 4);
				bb_state = FC_BB_ST_IDLE;		// 读出后可再次起飞追加录制
			}
			break;

		case FC_BB_ST_FULL:
		default:
			break;
	}
	FlightCtrl_BBState   = bb_state;
	FlightCtrl_BBRecs    = bb_recs;
	FlightCtrl_BBWRAddr  = bb_wr_addr;
}



void FlightCtrl_Tick(void)
{
	s32	rate_x, rate_y, rate_z;
	s32	roll_tgt_01, pitch_tgt_01, yaw_rate_tgt;
	s32	roll_rate_cmd, pitch_rate_cmd;
	s32	out_r, out_p, out_y, thrust;
	s32	m_front, m_rear, m_left, m_right;	// 四路马达输出（按物理位置命名）
	s32	diff_r, diff_p;						// 滚转/俯仰差动量（含 + 型布局补偿）
	s32	ang_max, yaw_max;
	s32	alt_err_x10 = 0;					// ★v19: 高度误差（单环高度PID输入，见 ⑥.9）
	u8	high_speed;
	u8	armed;

	//--------------------------------------------------------------------------
	// ① 传感器采集 + 姿态解算
	//--------------------------------------------------------------------------
#if C_FILTER_5_EN
	t3488_data_filter_handler(accel, gyro);			// IIC读6轴 + 中值滤波
#endif
	update_euler_dt(accel, gyro, FC_CTRL_DT_MS);	// 姿态解算（随控制周期自适应）
	ConvertT3488ParaToS16();						// → s16Roll/s16Pitch/s16Yaw (0.01°)
	//usart1_report_EulerDat();

	//--------------------------------------------------------------------------
	// ② 气压采样 → 高度/升降速度
	//--------------------------------------------------------------------------
	if(u8PresNewFlag != pres_last_flag)
	{
		pres_last_flag = u8PresNewFlag;
		fc_altitude_update();
	}

	//--------------------------------------------------------------------------
	// ②.5 垂向加速度低通（离地/触地检测用）
	//	静止在地面时 a_vert ≈ 0（与姿态无关，见 fc_vert_accel_01 说明）；
	//	推力超过重力的瞬间 a_vert 明显为正 → 以此判定离地，比气压可靠
	//	（气压在地面阶段被桨流污染，见 FlightCtrl.h 7.1 节说明）。
	//--------------------------------------------------------------------------
	{
		a_vert_raw = fc_vert_accel_01();

#if (FC_ACC_BIAS_EN)
		// ★ 垂向加速度零偏学习（FlightCtrl.h 7.5）：上锁(马达停转)状态学习传感器
		//   零偏/标度误差；★v34 扩展：解锁未离地的地面斜坡阶段【继续学习】——
		//   推力下的振动整流偏置与飞行最接近，而机体仍被地面
		//   支撑（a_vert 真值≈0），是"悬停推力偏置"的最佳观测窗口。V32_1/V32_2/
		//   V33_1 实录 vz_acc 恒 -29.6cm/s（贴 ±30 限幅）：仅靠停转时学的静态
		//   偏置（≈298）盖不住飞行偏置，纠漂门开之前(2S)高度估计少积分几十 cm，
		//   这正是"气压高度偏小一个量级"的主通道。离地后(fc_airborne=1)冻结。
		//   |a_vert| 超过门限视为搬动/晃动/真实起跳，暂停学习以免把动态吸进去。
		// ★ 2026/11/07：加"首个样本直接播种"。原来只做 >>8 的慢收敛，而
		//   门限 300 又和实测偏置同量级（298）—— 只要偏置略超门限，学习就
		//   永远进不来，AccBiasX10 恒为 0（本轮实测就是这个结果）。
		//   播种后第一拍即等于实测值，后续再慢速精修。
		if(fc_airborne == 0)		// 未离地：停转状态 + 地面斜坡阶段都学习
		{
			if((a_vert_raw < FC_ACC_BIAS_GATE) && (a_vert_raw > -FC_ACC_BIAS_GATE))
			{
				if(bias_seeded)
				{
					a_vert_bias += (a_vert_raw - a_vert_bias) >> FC_ACC_BIAS_SHIFT;
				}
				else
				{
					a_vert_bias = a_vert_raw;		// 首次：直接播种
					bias_seeded = 1;
				}
			}
			else
			{
				bias_seeded = 0;					// 搬动/晃动：下次重新播种
			}
		}
#endif
		// a_vert_lpf 为【扣除零偏后】的净值：离地检测/速度积分/D 项全部用它，
		// 静止时恒 ≈0，不再依赖"减掉理想重力 9810"碰巧为零。
		a_vert_lpf += ((a_vert_raw - a_vert_bias) - a_vert_lpf) >> FC_ACC_LPF_SHIFT;
#if ((FC_HTRIM_EN) || (FC_HVEL_EN))
		// 水平加速度（FlightCtrl.h 7.7/7.10）：
		// 转到"姿态测量域"（与 roll_meas/pitch_meas 同一套符号约定），
		// 这样"配平↑ → 估计角↑ → 对应加速度分量单调变化"的方向才是确定的。
		{
			s32 ax_m = FC_PITCH_SIGN * fc_acc_to_01((s16)(FC_ACC_SIGN_X * (s32)s16AccX));
			s32 ay_m = FC_ROLL_SIGN  * fc_acc_to_01((s16)(FC_ACC_SIGN_Y * (s32)s16AccY));
			ax_htrim_lpf += (ax_m - ax_htrim_lpf) >> FC_HTRIM_LP_SHIFT;
			ay_htrim_lpf += (ay_m - ay_htrim_lpf) >> FC_HTRIM_LP_SHIFT;
		}
#endif
		// 加计垂向速度积分（快通道）：0.1cm/S² × FC_CTRL_DT_MS(mS) = 0.1cm/S×mS
		// ★v19：主飞行纯气压（FC_VZ_ACC_FUSE_EN=0），但离地静默窗口内的
		//	垂直速度PID 仍要用它（见 ⑦），故积分改为无条件执行、常开；
		//	VzAccX10 遥测也恢复有效，可与 VzBaroX10 对比看振动整流污染程度。
		vz_acc_q += a_vert_lpf * FC_CTRL_DT_MS;
		// ★ 积分器本身必须钳位（2026/11/06 第四轮关键修复，原来不钳位！）：
		//   一旦零偏把 vz_acc 推过限幅，vz_acc_q 会继续深灌，之后即使零偏反向、
		//   或者气压纠漂想拉回来，也永远出不来 → 估计器永久致盲 → 失控爬升。
		//   钳位后，|vz_acc_q| 最多到限幅边界，零偏一旦回正立即可退出饱和。
		vz_acc_q = fc_clamp(vz_acc_q, -(FC_VZ_ACC_LIMIT * 1000), (FC_VZ_ACC_LIMIT * 1000));
		// ★v20 保险①：泄漏式积分（FlightCtrl.h FC_VZ_ACC_LEAK_SHIFT）。
		//   v19 实测起飞即侧翻：静默窗内振动整流偏置把 vz_acc 灌到 +30cm/s
		//   饱和 → PID 误判狂爬 → 推力坍缩坠地。泄漏后偏置无法积累成假速度
		//   （稳态假速度 = 偏置×τ，有界；真实加速度瞬态 τ 内仍能推动积分刹车）。
		vz_acc_q -= vz_acc_q >> FC_VZ_ACC_LEAK_SHIFT;
	}

	//return;
	//--------------------------------------------------------------------------
	// ③ 安全条件
	//	①失联：link_ok_flag=0 或 连续 1S(100×10mS) 没收到遥控数据
	//	②低压关机
	//	③翻机（倾角 > FC_TILT_LIMIT）
	//	以上任一成立立即上锁停转，并清除"一键起降"标志，防止恢复后突然自动起飞
	//--------------------------------------------------------------------------
	if( (link_ok_flag == 0) || (u32NoRxdCnt > 100) || (VddVolFlag & LowBatOffFlag) )
	{
		fc_disarm_reason = (VddVolFlag & LowBatOffFlag) ? 2 : 1;	// 2低压 / 1失联
		fc_state = FC_ST_DISARM;
		FunCtrlFlag &= ~CarOneKRun;
		fc_unlocked = 0;			// 失联/低压后必须重新摇杆解锁，避免恢复时突然起飞
		stunt_active = 0;
	}
	else if( (abs((s32)s16Roll) > FC_TILT_LIMIT) || (abs((s32)s16Pitch) > FC_TILT_LIMIT) )
	{
		fc_disarm_reason = 3;		// 3翻机（倾角 > 50°）
		fc_state = FC_ST_DISARM;
		FunCtrlFlag &= ~CarOneKRun;
		fc_unlocked = 0;			// 翻机后同样回到锁定
		stunt_active = 0;
	}

	//--------------------------------------------------------------------------
	// ③.5 摇杆组合操作：解锁 / 水平校准
	//	组合必须保持 FC_COMBO_HOLD_CNT 个节拍才生效，防止飞行中误触
	//--------------------------------------------------------------------------
	{
		u8 combo = 0;
#if ((FC_UNLOCK_EN) || (FC_CAL_EN))
		combo = fc_stick_combo();
#endif
		testPoint=0x80|combo;
		if(combo != combo_type)
		{
			combo_type = combo;			// 组合变了 → 重新计时
			combo_hold = 0;
			combo_done = 0;
			if((combo == 0) && (cal_state == 2)) cal_state = 0;	// 松开后清掉"完成"提示
		}
		else if(combo != 0)
		{
			if(combo_hold < FC_COMBO_HOLD_CNT) combo_hold++;
		}

		if((combo_hold >= FC_COMBO_HOLD_CNT) && (combo_done == 0))
		{
			combo_done = 1;
			if(combo == 1)								// 解锁组合
			{
#if (FC_UNLOCK_EN)
				fc_unlocked = 1;
				thr_start_ready = 0;					// 解锁时手还在杆上，须先松杆
				// ★v15: 自学习配平每次解锁清零——上次飞行若学入坏值
				//（振荡期/风中），不带到本次；用户微调配平(roll/pitch_trim_01)保留
				htrim_roll_01 = 0;
				htrim_pitch_01 = 0;
				ax_htrim_lpf = 0;
				ay_htrim_lpf = 0;
				htrim_cnt = 0;
				vx_q = 0;				// ★v16: 水平速度估计一并清零（上飞的残速不算数）
				vy_q = 0;
//				while(1)
//				{
//					wd_32k_feed();
//					testPoint=0x89;
//				}
#endif
			}
			else if(combo == 2)							// 水平校准组合
			{
#if (FC_CAL_EN)
				// 只在停转状态允许校准（校准会重置姿态与配平，飞行中做必然摔机）
				if(fc_state == FC_ST_DISARM)
				{
					fc_horiz_cal_start();
				}
#endif
			}
		}
	}

	// 水平校准采样：每拍采集一次陀螺原始数据（非阻塞）
#if (FC_CAL_EN)
	fc_horiz_cal_tick();
#endif

	//--------------------------------------------------------------------------
	// ④ 一键起降（RfSendKey_OneKRun → FunCtrlFlag.CarOneKRun，切换型按键）
	//	用边沿检测判断按键动作，而不是看标志电平：
	//	  待机 + 按下(0→1)          → 一键起飞（原地垂直升至约 2 米后悬停）
	//	  飞行中 + 按一下(翻转边沿) → 一键降落（要求四摇杆全部回中）
	//	★ 修复：旧实现把"标志=0"当成"请求降落"，推油门起飞（不经过该键）时
	//	  松杆瞬间 AllStickReleased() 成立即误入降落流程，马达骤停摔机。
	//--------------------------------------------------------------------------
	{
		u8 onek_now = (FunCtrlFlag & CarOneKRun) ? 1 : 0;
		u8 onek_edge = onek_now ^ onek_prev;

		if(onek_edge)
		{
			if(fc_state == FC_ST_DISARM)
			{
				if(onek_now && fc_can_arm())
				{
					// 一键起飞
					fc_state = FC_ST_TAKEOFF;
					takeoff_timer = 0;
					ground_timer = 0;
					desc_req_timer = 0;
					FlightCtrl_SetAltRef();					// 以地面为 0 高度
					alt_target_x10 = FC_TAKEOFF_ALT_X10;
					fc_pid_init_all();						// 积分清零、悬停油门复位
					yaw_lock_01 = (s32)s16Yaw;				// 无头模式基准机头方向
					stunt_active = 0;
					fc_disarm_reason = 0;					// 清上一轮的停桨原因
					dis_rec = 0;
					tdcd_cnt = 0;
					tdcd_alt_ref = 0;
				}
			}
			else if((FC_LAND_STICK_REL == 0) || FlightCtrl_AllStickReleased())
			{
				// 飞行中按一键键（无论从哪个状态切换）→ 一键降落
				fc_state = FC_ST_LANDING;
				land_timer = 0;
				stunt_active = 0;
			}
		}
		onek_prev = onek_now;
	}

#if (FC_THR_START_EN)
	// 推油门起飞：解锁后必须先看到「全部摇杆释放」，之后重新推油门超过阈值才起飞。
	// （解锁组合本身油门在杆上，不先松杆就推油门会带着残留状态误触发；
	//   起飞后油门杆恢复"控制升降速度"功能，回中即定高）
	if(fc_unlocked && (fc_state == FC_ST_DISARM) && FlightCtrl_AllStickReleased())
	{
		thr_start_ready = 1;
	}
	if(thr_start_ready && (fc_state == FC_ST_DISARM) &&
	   ((FunCtrlFlag & CarOneKRun) == 0) &&
	   (s16_Throttle > FC_THR_START_THR) && fc_can_arm())
	{
		fc_state = FC_ST_FLY;
		FlightCtrl_SetAltRef();							// 以当前地面为 0 高度（目标高度从 0 起步）
		fc_pid_init_all();								// 积分清零、悬停油门复位
		yaw_lock_01 = (s32)s16Yaw;						// 无头模式基准机头方向
		stunt_active = 0;
		ground_timer = 0;
		desc_req_timer = 0;
		fc_disarm_reason = 0;							// 清上一轮的停桨原因
		dis_rec = 0;
		tdcd_cnt = 0;
		tdcd_alt_ref = 0;
		thr_start_ready = 0;							// 下次起飞需重新松杆+推油门
	}
#endif

	armed = (fc_state != FC_ST_DISARM) ? 1 : 0;

	//--------------------------------------------------------------------------
	// ⑤ 摇杆 → 目标量
	//--------------------------------------------------------------------------
	high_speed = (FunCtrlFlag & CarSpeedFlag) ? 1 : 0;
	// 速度档只作用于「前进/后退/左右侧飞」的最大倾角（需求定义）
	ang_max = high_speed ? FC_ANGLE_MAX_HIGH : FC_ANGLE_MAX_LOW;
#if (FC_SPEED_AFFECT_YAW)
	yaw_max = high_speed ? FC_YAWRATE_MAX_HIGH : FC_YAWRATE_MAX_LOW;
#else
	yaw_max = FC_YAWRATE_MAX_HIGH;
#endif

	// 目标角度（0.01°）：摇杆单位是 0.1°
	roll_tgt_01  = fc_clamp((s32)s16_LRmove, -ang_max, ang_max) * 10;
	pitch_tgt_01 = fc_clamp((s32)s16_FBmove, -ang_max, ang_max) * 10;

	//--------------------------------------------------------------------------
	// ⑤.1 起飞/降落期间禁止平移：一键起降要求"原地垂直"升降，不前后左右移动
	//--------------------------------------------------------------------------
	if((fc_state == FC_ST_TAKEOFF) || (fc_state == FC_ST_LANDING))
	{
		roll_tgt_01  = 0;
		pitch_tgt_01 = 0;
	}

#if (FC_HEADLESS_EN)
	// 无头模式：把"地体系"的摇杆指令旋转到当前机体系
	if(FunCtrlFlag & CarHeadless)
	{
		s32 yaw_rel = (s32)s16Yaw - yaw_lock_01;
		if(yaw_rel >  18000) yaw_rel -= 36000;
		if(yaw_rel < -18000) yaw_rel += 36000;
		{
			s32 c = fc_cos_q15(yaw_rel);
			s32 s = fc_sin_q15(yaw_rel);
			s32 pb = (pitch_tgt_01 * c - roll_tgt_01 * s) >> 15;
			s32 rb = (pitch_tgt_01 * s + roll_tgt_01 * c) >> 15;
			pitch_tgt_01 = pb;
			roll_tgt_01  = rb;
		}
	}
#endif

	// 微调模式：用左右/前后摇杆缓慢调整配平（约 1.2°/S）
	if(FunCtrlFlag & CarMicroAdj)
	{
		trim_acc_roll  += (s32)s16_LRmove;
		trim_acc_pitch += (s32)s16_FBmove;
		roll_trim_01  += trim_acc_roll  >> 9;	trim_acc_roll  &= 0x1FF;
		pitch_trim_01 += trim_acc_pitch >> 9;	trim_acc_pitch &= 0x1FF;
		roll_trim_01  = fc_clamp(roll_trim_01,  -800, 800);
		pitch_trim_01 = fc_clamp(pitch_trim_01, -800, 800);
		roll_tgt_01  = 0;							// 微调模式下摇杆只改配平
		pitch_tgt_01 = 0;
	}

	// 目标偏航角速度（0.1°/S）
	yaw_rate_tgt = fc_clamp((s32)s16_Rotation, -yaw_max, yaw_max);

	// 油门杆不再直接生成升降速度指令：它改写目标高度，见 ⑥ 段

	//--------------------------------------------------------------------------
	// ⑤.2 特技表演：顺时针盘旋 360°（RfSendKey_StuntShow → FunCtrlFlag.CarDemoFlag）
	//	顺时针 = 负的偏航角速度；同时给一点前倾，让轨迹是圆弧而不是原地自转。
	//	用 s16Yaw 的增量累加判断转过的角度，转满 360° 后自动结束并清除标志。
	//--------------------------------------------------------------------------
#if (FC_STUNT_EN)
	if(FunCtrlFlag & CarDemoFlag)
	{
		if(stunt_active == 0)
		{
			// 只在空中启动（地面启动没有意义）
			if((fc_state == FC_ST_FLY) || (fc_state == FC_ST_TAKEOFF))
			{
				stunt_active   = 1;
				stunt_timer    = 0;
				stunt_yaw_acc  = 0;
				stunt_yaw_prev = (s32)s16Yaw;
			}
		}
		else
		{
			s32 dy = (s32)s16Yaw - stunt_yaw_prev;
			if(dy >  18000) dy -= 36000;		// s16Yaw 单位 0.01°，半圈 = 18000
			if(dy < -18000) dy += 36000;
			stunt_yaw_acc  += (dy < 0) ? -dy : dy;	// 取绝对值累加
			stunt_yaw_prev  = (s32)s16Yaw;

			stunt_timer++;
			if((stunt_yaw_acc >= 36000) || (stunt_timer > FC_STUNT_TMOUT))
			{
				stunt_active = 0;
				FunCtrlFlag &= ~CarDemoFlag;	// 转满一圈 → 自动退出特技
			}
		}
	}
	else
	{
		stunt_active = 0;
	}

	if(stunt_active)
	{
		yaw_rate_tgt  = -(s32)FC_STUNT_YAW_X10;		// 负 = 顺时针
		roll_tgt_01   = 0;
		pitch_tgt_01  = (s32)FC_STUNT_TILT_01;		// 前倾，走出圆弧
	}
#endif

	//--------------------------------------------------------------------------
	// ⑥ 油门杆 → 目标高度（★ 2026/11/02 按新定义重写，11/03 加离地门控）
	//	油门杆值 s16_Throttle（±12，1cm/S）= 目标高度的变化速率：
	//	  推到 20% → 目标高度以 2.4cm/S 匀速增长 → 飞机跟着垂直匀速上升
	//	  释放（死区内）→ 目标高度保持不变 → 悬停在目标高度
	//	  下拉 -10% → 目标高度以 1.2cm/S 匀速下降
	//	★ 只在已离地(fc_airborne)后才开始积累！地面阶段目标高度冻结为 0，
	//	  否则地面阶段(马达还没把飞机托起来)目标先跑出去十几厘米，加上桨流
	//	  把气压估计压低几十厘米（BDT 实测 -54.6cm），飞机会带着巨大的
	//	  "高度债务"起飞 → 油门饱和、一直上升停不下来。
	//--------------------------------------------------------------------------
	if((fc_state == FC_ST_FLY) && fc_airborne)
	{
		if(abs((s32)s16_Throttle) > FC_THR_DEADBAND)
		{
			// 增量累加器：s16_Throttle(1cm/S)×10(→0.1cm/S)×FC_CTRL_DT_MS(mS)
			alt_tgt_acc += (s32)s16_Throttle * 10 * FC_CTRL_DT_MS;	// 单位 0.1cm×mS
			alt_target_x10 += alt_tgt_acc / 1000;
			alt_tgt_acc -= (alt_tgt_acc / 1000) * 1000;

			// ★ 目标高度允许为负：起飞点与落地点可能不同高度
			//	（桌面起飞→地面降落，目标必须能降到负值；反之也成立）
			//	只做合理的量程限制，不做"到 0 就停"的钳位。
			if(alt_target_x10 < FC_ALT_TARGET_MIN_X10)
			{
				alt_target_x10 = FC_ALT_TARGET_MIN_X10;
				alt_tgt_acc = 0;
			}
			else if(alt_target_x10 > FC_ALT_TARGET_MAX_X10)
			{
				alt_target_x10 = FC_ALT_TARGET_MAX_X10;
				alt_tgt_acc = 0;
			}
		}
		else
		{
			alt_tgt_acc = 0;						// 死区内丢弃不足 0.1cm 的残数，目标保持
		}

		// 「刚下过降指令」余温：只有真的拉过下降杆，才允许做落地判定。
		//	否则单纯的气压漂移（估算高度自己往上飘 6cm）会被误判成落地而上锁。
		if(s16_Throttle < -FC_THR_DEADBAND)
		{
			desc_req_timer = FC_GROUND_REQ_TMOUT;
		}
		else if(desc_req_timer)
		{
			desc_req_timer--;
		}
	}

	//--------------------------------------------------------------------------
	// ⑥.5 离地检测（在 FLY / TAKEOFF 的地面阶段进行）
	//	判据A（主）：垂向加速度低通 > FC_LIFTOFF_ACC 持续 100mS（推力已明显超过
	//	      重力；静止在地面时该值≈0、与姿态无关，不受气压计桨流污染影响）
	//	判据B（★v28 辅助，OR 关系）：气压升降速度 > +20cm/S 持续 100mS。
	//	      桨流只会造成"假下降"绝不会"假爬升"，气压持续正速度 = 真离地。
	//	★ 2026/11/03 台架实测：原 25cm/S² + 50mS 在压住飞机满推力时会误触发，
	//	  改为 50cm/S²；v27_1 实录显示 200mS 的确认窗内 a_vert 振动凹陷（79/38）
	//	  会把计数清零，确认拖到推力 338（过推 88PWM → 差动余量被挤干翻机），
	//	  故 v28 把主判据提速到 100mS，并补上气压正速度辅助判据双保险。
	//	★ 离地瞬间重新清零高度基准——把地面阶段桨流压进高度估计的几十厘米
	//	  污染一次性甩掉，高度环从干净的数据开始工作。
	//--------------------------------------------------------------------------
	if(armed && (fc_airborne == 0) &&
	   ((fc_state == FC_ST_FLY) || (fc_state == FC_ST_TAKEOFF)))
	{
		// ★v29 判据A 加推力门限：thrust_last ≥ FC_LIFTOFF_THR_MIN(hover-25) 才
		//   允许 a_vert 计数。V28_1 实录：推力仅 218（真实悬停≈250）时地面斜坡
		//   振动让 a_vert>50 连满 100mS → 假离地；推力低于悬停值物理上不可能离地。
		if((thrust_last >= FC_LIFTOFF_THR_MIN) && (a_vert_lpf > FC_LIFTOFF_ACC))
		{
			if(liftoff_cnt < 1000) liftoff_cnt++;
		}
		else
		{
			liftoff_cnt = 0;
		}

		// ★v28 判据B：气压正速度（vz_baro_x10 在本节拍前段的气压滤波里已更新）
		if(vz_baro_x10 > FC_LIFTOFF_BARO_VZ)
		{
			if(baro_liftoff_cnt < 1000) baro_liftoff_cnt++;
		}
		else
		{
			baro_liftoff_cnt = 0;
		}

		if((liftoff_cnt >= FC_LIFTOFF_CNT) || (baro_liftoff_cnt >= FC_LIFTOFF_BARO_CNT))
		{
			fc_airborne = 1;
			air_timer = 0;
			// ★v27：把"本架次真正离地时施加的推力"记下来，作为静默窗口的
			//	推力地板与积分钳位中心（thrust_last = 上一拍已过斜率限制的实
			//	际推力，≈250）。标定值 FC_PWM_HOVER 追不上电池电压/桨况/起飞
			//	重量的变化，这一拍的值才是现场真值。
			liftoff_thrust = thrust_last;
			// ★ 2026/11/03 气压计实测后改为：离地瞬间【不】清气压基准——
			//   此刻推力在最大值附近，桨流偏置最大（支架上 7cm、贴地 50cm+），
			//   立即清零会把最大偏置当零点，推力回落后估计跳回十几厘米。
			//   先进静默窗口（FC_LIFT_QUIET），让升降速度环把爬升压到 0，
			//   窗口结束由 fc_lift_quiet_end() 重清基准 + 学习悬停油门。
			// ★ v34：离地瞬间清 vz_acc_q / vz_corr_q——地面斜坡阶段振动整流
			//   偏置已在积分器里积了垃圾（实录 vz_acc 常年贴 -30cm/s 限幅），
			//   静默窗的 vz 环用的就是它，不清会把偏置直接灌进推力指令；
			//   此刻 a_vert_bias 已被地面斜坡学习刷新，从零积分才是真值。
			vz_acc_q = 0;
			vz_corr_q = 0;
			lift_quiet_cnt = FC_LIFT_QUIET;
			alt_target_x10 = 0;
			alt_tgt_acc = 0;
			ground_ramp = 0;
			liftoff_cnt = 0;
			baro_liftoff_cnt = 0;
		}
	}

	// 离地后计时（触顶判定的使能延迟用）
	if(fc_airborne && (air_timer < 60000))
	{
		air_timer++;
	}

	//--------------------------------------------------------------------------
	// ⑥.9 高度误差（★v19 单环高度PID 的唯一输入，见 FlightCtrl.h 【高度环】节）：
	//	err = 目标高度 − 估计高度，死区内为 0，直接进 ⑦ 的高度PID。
	//	静默窗口/地面阶段强制 0（气压被桨流污染/高度环不参与）。
	//--------------------------------------------------------------------------
	if(armed && fc_airborne && (lift_quiet_cnt == 0))
	{
		alt_err_x10 = alt_target_x10 - alt_est_x10;
		if((alt_err_x10 < (s32)FC_ALT_DEADBAND) && (alt_err_x10 > -(s32)FC_ALT_DEADBAND))
		{
			alt_err_x10 = 0;
		}
	}
	else
	{
		alt_err_x10 = 0;
	}
	alt_err_dbg = alt_err_x10;

	switch(fc_state)
	{
	case FC_ST_TAKEOFF:
		takeoff_timer++;
		if(fc_airborne == 0)
		{
			// 地面阶段：高度环不参与（vz_cmd=0），推力由 ⑦ 的地面斜坡接管；
			// 目标高度保持 0，等离地后再开始爬向 2 米
			vz_cmd_x10 = 0;
			alt_target_x10 = 0;
			if(ground_ramp > FC_GND_RAMP_TMOUT)
			{
				fc_disarm_reason = 4;		// 4地面超时（3S 仍离不了地）
				fc_state = FC_ST_DISARM;	// 3S 仍离不了地：被卡住/被按住 → 上锁
				armed = 0;
			}
		}
		else if(lift_quiet_cnt)
		{
			// 离地静默窗口：高度环不参与，升降速度环（vz_cmd=0）先把爬升压到 0，
			//	推力稳定到悬停点后窗口结束 → 重清气压基准，再开始爬向 2 米
			vz_cmd_x10 = 0;
			alt_target_x10 = 0;
			if(--lift_quiet_cnt == 0)
			{
				fc_lift_quiet_end();
			}
		}
		else
		{
			// 以固定起降速度（19.2cm/S）原地垂直爬升到 2 米，然后转入定高悬停
			// ★v19 单环架构：目标高度按速度积分爬向 2 米，高度PID 追踪目标
			alt_tgt_acc += (s32)FC_TAKEOFF_VZ * FC_CTRL_DT_MS;		// 单位 0.1cm×mS
			alt_target_x10 += alt_tgt_acc / 1000;
			alt_tgt_acc -= (alt_tgt_acc / 1000) * 1000;
			if(alt_target_x10 > (s32)FC_TAKEOFF_ALT_X10) alt_target_x10 = FC_TAKEOFF_ALT_X10;
			if((alt_est_x10 >= (FC_TAKEOFF_ALT_X10 - 100)) || (takeoff_timer > FC_TAKEOFF_TMOUT))
			{
				fc_state = FC_ST_FLY;
				alt_target_x10 = FC_TAKEOFF_ALT_X10;
			}
		}
		break;

	case FC_ST_FLY:
	{
		if(fc_airborne == 0)
		{
			// 地面阶段：高度环不参与，推力由 ⑦ 的地面斜坡接管，目标保持 0
			vz_cmd_x10 = 0;
			alt_target_x10 = 0;
			ground_timer = 0;
			tdcd_cnt = 0;
			if(ground_ramp > FC_GND_RAMP_TMOUT)
			{
				fc_disarm_reason = 4;		// 4地面超时（3S 仍离不了地）
				fc_state = FC_ST_DISARM;	// 3S 仍离不了地：被卡住/被按住 → 上锁
				armed = 0;
			}
			break;
		}

		// ---- 离地静默窗口：高度环暂不参与（详见 fc_lift_quiet_end 注释） ----
		if(lift_quiet_cnt)
		{
			vz_cmd_x10 = 0;
			alt_target_x10 = 0;
			if(--lift_quiet_cnt == 0)
			{
				fc_lift_quiet_end();
			}
			break;
		}

		// ---- 已离地的正常定高飞行（★v19 单环：高度误差在 ⑥.9 统一算好，
		//	直接进 ⑦ 的高度PID；油门杆在 ⑥ 已把目标高度积分，无外环）----
		vz_cmd_x10 = (s32)s16_Throttle * 10;		// 遥测：杆指令升降速度（不参与控制）

		// 触顶/堵转检测（"升不上去"方向，与下面"降不下去"方向互补）：
		//	★ v19 按用户指正更名"触顶/堵转"：要求爬升却升不上去 = 被上方压住
		//	（手按住/顶到天花板/落地被桨流压低估计），不是"触地"。
		//	滑动窗口判据（FC_TDCD_WIN=2S 内高度估计涨幅 < FC_TDCD_MIN_RISE=10cm）：
		//	  ① 目标高于估计 FC_TDCD_ALTERR(10cm) 以上（v19 单环架构用高度误差
		//	    表达"要求爬升"，替代旧 vz_cmd）且垂向加速度≈0（并没有在加速）；
		//	  ② 2S 窗口内 est 只涨了不到 10cm → 被压住 → 结束飞行。
		//	★ 用"窗口涨幅"而不是 vz_est 瞬时值：气压滞后/桨流偏置时瞬时 vz_est
		//	  不可信，旧判据会在真爬升时误判成"升不上去" → 空中停桨
		//	  （2026/11/03 真机实证：松杆后一级一级往上飞，6~7 次后马达停掉）。
		//	没有它的话：落地后 est 被桨流压得比目标还低，高度环一直以为没到
		//	目标高度，马达会在地面上满油门空转、永远停不下来（BDT 抓图实证）。
		{
			u8 tdcd_hit = 0;

			// ★ 离地后 FC_TDCD_ARM_DLY(2S) 内禁止触地判定：起飞过渡期推力大起
			//	大落、桨流偏置来回跳，早判容易空中误停桨（真机实证 6~7 次脉冲
			//	后停桨；其中"停桨"多半就是它误触发）。落地被按住时推力会持续
			//	饱和，2S 后照样能判出来，不会漏。
			if(air_timer < FC_TDCD_ARM_DLY)
			{
				tdcd_cnt = 0;
			}
			else if((alt_err_x10 > (s32)FC_TDCD_ALTERR) && (a_vert_lpf < FC_LIFTOFF_ACC))
			{
				if(tdcd_cnt == 0) tdcd_alt_ref = alt_est_x10;		// 记录窗口起点
				if(tdcd_cnt < 60000) tdcd_cnt++;
				if(tdcd_cnt >= FC_TDCD_WIN)
				{
					if((alt_est_x10 - tdcd_alt_ref) < FC_TDCD_MIN_RISE)
					{
						tdcd_hit = 1;							// 2S 没升上去 → 触地
					}
					else
					{
						tdcd_cnt = 0;							// 真在爬 → 滚动窗口重计
						tdcd_alt_ref = alt_est_x10;
					}
				}
			}
			else if(tdcd_cnt != 0)
			{
				// ★衰减式计数：振动毛刺只回退不清零（台架实测压住满推力时
				//   a_vert 偶发窜高，硬清零攒不满）。
				// ★v29 修复：tdcd_cnt 是 u16，cnt=1~7 时减 8 会下溢回绕到
				//   65529+，下一拍即满足 >= FC_TDCD_WIN 误判堵转停桨
				//   （V28_1 实录：起飞 2.54S 被它误停机，reason=5）。
				if(tdcd_cnt >= 8) tdcd_cnt -= 8;
				else              tdcd_cnt = 0;
			}

			if(tdcd_hit)
			{
				fc_disarm_reason = 5;						// 触顶/堵转判定（升不上去）
				fc_state = FC_ST_DISARM;
				armed = 0;
				stunt_active = 0;
				FunCtrlFlag &= ~CarDemoFlag;
#if (FC_RELOCK_ON_LAND)
				fc_unlocked = 0;
#endif
				break;
			}
		}

		// 结束飞行检测（"降不下去"方向）——★ v17 分模式（16轮实测"拉油门立刻
		//	停飞"的 Bug 根因就在这里）：
		//	闭环定高（FC_VZ_MANUAL_EN=0）：飞机跟着目标高度走，"目标低于实际
		//	6cm 且 1S 降不下去" = 真落住，判据成立，照旧使用。
		//	手动直驱（FC_VZ_MANUAL_EN=1）：飞机跟【油门杆】走，目标高度独立
		//	下坠——拉杆 0.5S 后目标即落后实际 6cm，且推力突降时桨流偏置释放
		//	把估计高度托住不降 → 1S 窗口"降幅<4cm"成立 → 判"已落住"→ 拉杆
		//	即停桨（与落地无关的误判！）。手动模式改用专用物理判据落地。
#if (FC_VZ_MANUAL_EN == 0)
		// ★ v19：判据只剩"拉过下降杆"（desc_req）——删除旧版"目标低于实际
		//	6cm"条件（目标允许为负，目标低≠着地），窗口加长到 3S（见 .h FC_GROUND_*）。
		if(fc_ground_detect(desc_req_timer > 0))
		{
			fc_disarm_reason = 6;					// 6降不下去/已落住
			fc_state = FC_ST_DISARM;
			armed = 0;
			stunt_active = 0;
			FunCtrlFlag &= ~CarDemoFlag;
#if (FC_RELOCK_ON_LAND)
			fc_unlocked = 0;						// 落地后回到锁定（需再次组合解锁）
#endif
		}
#else
		// 手动模式落地判定：油门杆持续拉低 + 估计垂速≈0 + 垂向加速度≈0。
		//	空中拉杆时推力被压到远低于悬停（杆 -8 → thrust≈97），飞机必在
		//	下降，|vz| 条件不成立；只有真落住地面三条才同时满足。
		{
			u8 man_land_now =
				(s16_Throttle <= -(s16)FC_MAN_LAND_THR) &&
				(vz_est_x10 > -(s32)FC_MAN_LAND_VZ) && (vz_est_x10 < (s32)FC_MAN_LAND_VZ) &&
				(a_vert_lpf < FC_MAN_LAND_ACC) && (a_vert_lpf > -FC_MAN_LAND_ACC);

			if(man_land_now)
			{
				if(man_land_cnt < 60000) man_land_cnt++;
			}
			else
			{
				man_land_cnt = 0;
			}

			if(man_land_cnt >= FC_MAN_LAND_TMOUT)
			{
				fc_disarm_reason = 6;				// 6已落住（手动模式物理判据）
				fc_state = FC_ST_DISARM;
				armed = 0;
				stunt_active = 0;
				FunCtrlFlag &= ~CarDemoFlag;
#if (FC_RELOCK_ON_LAND)
				fc_unlocked = 0;
#endif
			}
		}
#endif
	}
		break;

	case FC_ST_LANDING:
		// ★v19 单环架构：目标高度按 -19.2cm/S 积分下降，高度PID 追踪；
		//	落地判据 fc_ground_detect(1)（3S 高度不变）负责收尾
		alt_tgt_acc += (s32)FC_LANDING_VZ * FC_CTRL_DT_MS;
		alt_target_x10 += alt_tgt_acc / 1000;
		alt_tgt_acc -= (alt_tgt_acc / 1000) * 1000;
		if(alt_target_x10 < FC_ALT_TARGET_MIN_X10) alt_target_x10 = FC_ALT_TARGET_MIN_X10;
		land_timer++;
		// 一键降落同样用"降不下去了"做判据：从桌面起飞降到地面时，
		//	用固定高度阈值（旧版 alt_est<=10cm）永远判不到，只能靠超时硬停。
		if(fc_ground_detect(1) || (land_timer > FC_LANDING_TMOUT))
		{
			fc_disarm_reason = (land_timer > FC_LANDING_TMOUT) ? 8 : 7;
			fc_state = FC_ST_DISARM;				// 已落住/超时 → 上锁停转
			armed = 0;
			FunCtrlFlag &= ~CarDemoFlag;
#if (FC_RELOCK_ON_LAND)
			fc_unlocked = 0;						// 落地后回到锁定（需再次组合解锁）
#endif
		}
		break;

	default:	// FC_ST_DISARM
		vz_cmd_x10 = 0;
		alt_target_x10 = 0;							// 上锁状态目标高度保持 0，起飞从 0 起步
		alt_tgt_acc = 0;							// ★v19: 目标积分累加器一并清零
		// 离地/触地状态一并复位
		fc_airborne = 0;
		liftoff_cnt = 0;
		baro_liftoff_cnt = 0;
		tdcd_cnt = 0;
		ground_ramp = 0;
		a_vert_lpf = 0;
		ground_timer = 0;
		air_timer = 0;
		lift_quiet_cnt = 0;
		thrust_last = FC_PWM_HOVER;
		liftoff_thrust = FC_PWM_HOVER;
	thrust_lpf = FC_PWM_HOVER;
	alt_err_dbg = 0;
#if (FC_VZ_MANUAL_EN)
		man_land_cnt = 0;						// ★v17: 手动模式落地判定计数清零
#endif
		ground_alt_ref = alt_est_x10;
		// 停桨现场快照（只记一次）：停桨那一刻的估算高度与倾角，
		//	用来区分"空中停桨"与"落地停桨"、以及是不是摇晃把倾角打超限
		if((fc_disarm_reason != 0) && (dis_rec == 0))
		{
			s32 tilt = abs((s32)s16Roll);
			if(abs((s32)s16Pitch) > tilt) tilt = abs((s32)s16Pitch);
			dis_alt_x10 = (s16)fc_clamp(alt_est_x10, -32768, 32767);
			dis_tilt_01 = (s16)fc_clamp(tilt, -32768, 32767);
			dis_rec = 1;
		}
		break;
	}

	//--------------------------------------------------------------------------
	// ⑦ 姿态控制：角度环 → 角速度环
	//--------------------------------------------------------------------------
	if(armed)
	{
		// ★ 角度符号实测标定入口：FC_ROLL_SIGN / FC_PITCH_SIGN（见 FlightCtrl.h）
		//   符号必须同时作用于「角度」和「陀螺角速度」，内外环才保持一致
		s32 roll_meas  = FC_ROLL_SIGN  * (s32)s16Roll;
		s32 pitch_meas = FC_PITCH_SIGN * (s32)s16Pitch;
		s32 roll_err_01, pitch_err_01;

#if (FC_HVEL_EN)
		//----------------------------------------------------------------------
		// 7.10 水平速度估计 + 漂移阻尼（v16 核心，见 FlightCtrl.h 7.10）
		//	ax_htrim_lpf/ay_htrim_lpf 本拍已由 ② 更新（原始加计水平分量 LP，
		//	meas 域）。扣除重力投影即得真实水平加速度——桨平面歪→机体加速→
		//	加计直接感测，与倾角来自 PCB 还是桨平面无关。
		//----------------------------------------------------------------------
		{
			s32 ax_h = ax_htrim_lpf + (1712L * pitch_meas) / 1000;	// g·sin(pitch)
			s32 ay_h = ay_htrim_lpf - (1712L * roll_meas)  / 1000;	// g·sin(roll)

			if(fc_airborne &&
			   (roll_tgt_01 < (s32)FC_HVEL_STICK_GATE) && (roll_tgt_01 > -(s32)FC_HVEL_STICK_GATE) &&
			   (pitch_tgt_01 < (s32)FC_HVEL_STICK_GATE) && (pitch_tgt_01 > -(s32)FC_HVEL_STICK_GATE))
			{
				// 悬停/微杆：积分（含泄漏 τ≈1.28S，防加计偏置无限积累）
				vx_q += ax_h * FC_CTRL_DT_MS;
				vy_q += ay_h * FC_CTRL_DT_MS;
				vx_q -= vx_q >> FC_HVEL_LEAK_SHIFT;
				vy_q -= vy_q >> FC_HVEL_LEAK_SHIFT;
			}
			else
			{
				// 机动中/未离地：冻结积分并快速泄放（机动残速不算漂移）
				vx_q -= vx_q >> 4;
				vy_q -= vy_q >> 4;
			}
			vx_q = fc_clamp(vx_q, -(s32)FC_HVEL_V_LIMIT * 1000, (s32)FC_HVEL_V_LIMIT * 1000);
			vy_q = fc_clamp(vy_q, -(s32)FC_HVEL_V_LIMIT * 1000, (s32)FC_HVEL_V_LIMIT * 1000);
			hvel_vx = (s16)fc_clamp(vx_q / 1000, -32768, 32767);
			hvel_vy = (s16)fc_clamp(vy_q / 1000, -32768, 32767);

			// 阻尼输出 = 反倾角指令。★v31: 取反保留（v30 由 +vx 改 -vx，方向已验证）。
			//	链路：机体后漂(vx<0)→要抵消后退应前倾加速→命令低头(err↑)→
			//	out_p>0→FRONT=-1 前减速/后加速=低头力矩 ✓负反馈。原 +vx 则命令
			//	抬头→推力后倾→越漂越快（V29_1 实测 hx_out 贴 -3° 上限与发散
			//	同向，即 v29 慢发散真凶）。htrim 学习以 hx_out 为收敛目标，
			//	随本符号自动跟随，无需另改。
			hx_out_roll  = -(((s32)hvel_vy * (s32)FC_HVEL_GAIN_Q8) >> 8);
			hx_out_pitch = -(((s32)hvel_vx * (s32)FC_HVEL_GAIN_Q8) >> 8);
			hx_out_roll  = fc_clamp(hx_out_roll,  -(s32)FC_HVEL_TGT_LIMIT, (s32)FC_HVEL_TGT_LIMIT);
			hx_out_pitch = fc_clamp(hx_out_pitch, -(s32)FC_HVEL_TGT_LIMIT, (s32)FC_HVEL_TGT_LIMIT);
		}
#else
		hx_out_roll = 0;
		hx_out_pitch = 0;
#endif

		roll_err_01  = roll_tgt_01  + roll_trim_01  + htrim_roll_01  + hx_out_roll  - roll_meas;
		pitch_err_01 = pitch_tgt_01 + pitch_trim_01 + htrim_pitch_01 + hx_out_pitch - pitch_meas;

		roll_rate_cmd  = fc_pid_calc(&pid_roll_a,  roll_err_01,  roll_meas);
		pitch_rate_cmd = fc_pid_calc(&pid_pitch_a, pitch_err_01, pitch_meas);

		rate_x = FC_ROLL_SIGN  * fc_gyro_to_rate01(s16GyroX, g_gyro_offset_x, FC_GYRO_SIGN_X);
		rate_y = FC_PITCH_SIGN * fc_gyro_to_rate01(s16GyroY, g_gyro_offset_y, FC_GYRO_SIGN_Y);
		rate_z = fc_gyro_to_rate01(s16GyroZ, g_gyro_offset_z, FC_GYRO_SIGN_Z);

		out_r = fc_pid_calc(&pid_roll,  roll_rate_cmd  - rate_x, rate_x);
		out_p = fc_pid_calc(&pid_pitch, pitch_rate_cmd - rate_y, rate_y);
		out_y = fc_pid_calc(&pid_yaw,   yaw_rate_tgt   - rate_z, rate_z);

#if (FC_UART_DBG_EN)
		// 7.9 高速调试采样（★v16: rate_cmd/rate_x 通道换成水平速度估计）
		dbg_pitch_meas = (s16)pitch_meas;
		dbg_rate_y     = (s16)rate_y;
		dbg_vx         = hvel_vx;
		dbg_vy         = hvel_vy;
		dbg_roll_meas  = (s16)roll_meas;					// ★v15: 横向漂移诊断
#endif

		// 升降速度环（D 项用垂向加速度做阻尼；用 ②.5 已算好的净值，零偏不影响差分）
#if (FC_VZ_MANUAL_EN)
		// ★ 手动油门直驱（见 FlightCtrl.h 7.6）：推力 = 杆位，无速度反馈。
		//	杆量 ±384 线性映射到 FC_MAN_THR_MID ± FC_MAN_THR_SPAN，
		//	即杆位范围 FC_MAN_THR_MID-FC_MAN_THR_SPAN ~ +SPAN（≈60~340）。
		thrust = (s32)FC_MAN_THR_MID
		       + ((s32)s16_ThrottleFine * (s32)FC_MAN_THR_SPAN) / 384;
#if (FC_MAN_VZ_DAMP_EN)
		// ★ v16 垂直速度阻尼：thrust -= K·vz_est（融合估计，加计快通道为主）。
		//	松杆≈垂直速度保持：慢爬升被自动压住（用户实测 v15 仍缓慢爬升），
		//	杆量仍占主导（阻尼限幅 ±30 PWM vs 满杆行程 ±140）。
		{
			s32 d = -(((s32)vz_est_x10 * (s32)FC_MAN_VZ_GAIN_Q8) >> 8);
			d = fc_clamp(d, -(s32)FC_MAN_VZ_LIMIT, (s32)FC_MAN_VZ_LIMIT);
			thrust += d;
		}
#endif
		thrust = fc_clamp(thrust, (s32)FC_PWM_MIN_RUN + 15, (s32)FC_THR_MAX_CMD);
		// 让积分跟着实际推力走：日后把 FC_VZ_MANUAL_EN 改回 0 时无扰切换
		pid_vz.integ = fc_clamp(thrust << 10, FC_VZ_I_MIN, FC_VZ_I_MAX);
		pid_vz.dterm = 0;
#else
#if ((FC_VZ_ACC_FUSE_EN) == 0)
		// ★v19 离地静默窗口：气压被桨流深度污染（表观 -7~-50cm 的假下降），不可用。
		//	改用【加计积分速度 + 垂直速度PID】主动控制爬升（目标 0，把起飞初始
		//	爬升刹住）——取代 v18 的"冻结推力"（用户判定：被动冻结没有道理）。
		//	窗口结束 fc_lift_quiet_end() 重清气压基准 + 学习悬停油门，
		//	之后切换到气压单环高度PID。
		if(lift_quiet_cnt)
		{
			s32 vz_acc_x10 = vz_acc_q / 1000;	// 加计积分速度(0.1cm/S)，v20 已泄漏式

			vz_est_x10 = vz_acc_x10;			// 窗口内反馈 = 加计积分速度（遥测可见）
			thrust = fc_pid_calc(&pid_vz, -vz_acc_x10, vz_est_x10);
			// ★v27 保险②：推力下限 = 【离地当拍实测推力】− FC_QUIET_THR_FLOOR。
			//   v26 实录：用标定值算地板 → 140PWM，而真实悬停≈250，飞机带着 250
			//   离地后立刻掉回地面（a_vert −576cm/s²，0.14S 后摔地）。改用实测
			//   离地推力后地板≈230，任何误判都不会把推力打到悬停点以下太多。
			if(thrust < ((s32)liftoff_thrust - (s32)FC_QUIET_THR_FLOOR))
			{
				thrust = (s32)liftoff_thrust - (s32)FC_QUIET_THR_FLOOR;
			}
			// ★v27 保险③：窗内积分钳位到【离地实测推力】±FC_QUIET_I_BAND。
			//   防止误判期间积分深灌（窗末 fc_lift_quiet_end() 用此刻推力学悬停
			//   油门，不带大偏差；且 v17 实证积分深灌后 Ki 太小泄不出来）。
			pid_vz.integ = fc_clamp(pid_vz.integ,
				((s32)liftoff_thrust - (s32)FC_QUIET_I_BAND) << 10,
				((s32)liftoff_thrust + (s32)FC_QUIET_I_BAND) << 10);
		}
		else
#endif
		// ★v19 单环高度PID：err = 目标高度 − 估计高度（⑥.9 已算好、已过死区），
		//	P 直接出推力增量、I=自适应悬停油门、D 作用在升降速度差分上（加速度阻尼）。
		thrust = fc_pid_calc(&pid_vz, alt_err_x10,
#if (FC_ACC_DAMP_EN)
		                     a_vert_raw - a_vert_bias
#else
		                     vz_est_x10
#endif
		                     );
#endif
		// 推力上限（默认 = FC_PWM_MAX；改小可留出差动余量，用于排查"顶满油门时
		// 混控失去控制权限 → 左右摇晃"）。地面阶段的推力斜坡不受此限制。
		if(thrust > FC_THR_MAX_CMD) thrust = FC_THR_MAX_CMD;
#if (FC_VZ_MANUAL_EN)
		// 手动模式：不要地面斜坡、不要离地判据参与推力（推力只由杆位决定），
		//	否则斜坡把推力顶到 400 会和杆量打架，又变成一个"隐藏的开环满推力"。
#else
		// 地面阶段（未离地）：高度环输出作废，推力走斜坡强制起飞——
		//	从 FC_PWM_TAKEOFF 起每 5mS +1，1 秒左右爬到满推力，任何真实悬停点
		//	都会被越过从而可靠离地；同时把速度环积分钉在悬停值，杜绝地面阶段
		//	积分饱和（旧版地面推油门 Thrust 直接冲到 400，离地即满推力弹射，
		//	且混控在 400 失去半边控制权限 → 左右摇晃）。
		if(((fc_state == FC_ST_FLY) || (fc_state == FC_ST_TAKEOFF)) && (fc_airborne == 0))
		{
			s32 thr_ramp;

			ground_ramp++;
			thr_ramp = (s32)FC_PWM_TAKEOFF + (s32)ground_ramp;	// 每 5mS +1
			// ★v28：斜坡封顶从 FC_PWM_MAX(400) 降到 FC_GND_RAMP_CAP(290)。
			//	v27_1 实录斜坡一路推到 338 才确认离地，过推 88PWM：差动余量
			//	min(160, t-51, 392-t) 只剩 54，姿态环拉不住俯仰偏置 → 翻机。
			//	290 留 102 差动余量；若悬停标定正确，290 已足够离地（悬停 250）。
			if(thr_ramp > FC_GND_RAMP_CAP) thr_ramp = FC_GND_RAMP_CAP;
			thrust = thr_ramp;
			// ★ 2026/11/04 第三轮关键修正：积分必须钉在【此刻实际施加的推力】上，
			//	而不是固定的 FC_PWM_HOVER(今 250)。原来离地瞬间会发生这种情况：
			//	斜坡已把推力推到 250 才把飞机托起来，一确认离地就交还给速度环，
			//	而速度环积分只有 170 → 推力按斜率(-20/5mS)在 60mS 内砸回 170
			//	→ 飞机立刻掉高度 → 积分再慢慢爬回来 → 一起来就掉、掉完再爬，
			//	这正是"脉冲爬升"的第二个来源（与气压桨流偏置叠加）。
			// ★ 2026/11/07 第五轮：钉的时候必须夹进 [i_min,i_max]！
			//	原来直接 <<10 赋值，能把积分钉到 400<<10（远超 i_max），
			//	之后 Ki=0.001 每拍只泄 0.18PWM，要 2 秒才泄回悬停 →
			//	整段时间环等于"开环满推力"，正好是脉冲爬升的发动机。
			pid_vz.integ = fc_clamp(thr_ramp << 10, FC_VZ_I_MIN, FC_VZ_I_MAX);
			pid_vz.dterm = 0;
		}
#endif

		// 推力斜率限制（非对称）：上升慢（防离地瞬间弹射），下降快（收油门
		// 要能立刻压住高度，否则目标已到、油门却还在高位 → 冲高停不下来）
		if(thrust > thrust_slew + FC_THR_SLEW_UP)			thrust = thrust_slew + FC_THR_SLEW_UP;
		else if(thrust < thrust_slew - FC_THR_SLEW_DN)		thrust = thrust_slew - FC_THR_SLEW_DN;
		thrust_slew = thrust;
		thrust_last = (s16)thrust;		// 供离地静默窗口结束时学习悬停油门

#if (FC_HTRIM_EN)
		//----------------------------------------------------------------------
		// ★ 悬停漂移自学习节拍（FlightCtrl.h 7.7，v16 改架构）：
		//	悬停附近、杆中立、不机动时，把 7.10 阻尼器的稳态输出（反倾角）
		//	慢慢吸收进配平 → 阻尼器输出回到 0 → 单通道串级，无双积分互拍
		//	（v15 直接闭环在加速度上，与阻尼互拍 = 实测 1Hz 翼摆的来源）。
		//----------------------------------------------------------------------
		if((fc_state == FC_ST_FLY) && (fc_airborne != 0) &&
		   (thrust_lpf > FC_HTRIM_THR_MIN) && (thrust_lpf < FC_HTRIM_THR_MAX) &&
		   (roll_tgt_01 < 50) && (roll_tgt_01 > -50) &&
		   (pitch_tgt_01 < 50) && (pitch_tgt_01 > -50) &&
		   (rate_x < FC_HTRIM_RATE_GATE) && (rate_x > -FC_HTRIM_RATE_GATE) &&
		   (rate_y < FC_HTRIM_RATE_GATE) && (rate_y > -FC_HTRIM_RATE_GATE) &&
		   (ax_htrim_lpf < FC_HTRIM_ACC_GATE) && (ax_htrim_lpf > -FC_HTRIM_ACC_GATE) &&
		   (ay_htrim_lpf < FC_HTRIM_ACC_GATE) && (ay_htrim_lpf > -FC_HTRIM_ACC_GATE))
		{
			if(++htrim_cnt >= FC_HTRIM_PERIOD)
			{
				s32 d;
				htrim_cnt = 0;
				// roll：向阻尼器输出收敛（阻尼输出本身就是负反馈方向的修正量）
				d = fc_clamp(hx_out_roll / FC_HTRIM_DIV,
							 -(s32)FC_HTRIM_STEP, (s32)FC_HTRIM_STEP);
				htrim_roll_01 = (s16)fc_clamp((s32)htrim_roll_01 + d,
											  -FC_HTRIM_LIMIT, FC_HTRIM_LIMIT);
				// pitch：同上
				d = fc_clamp(hx_out_pitch / FC_HTRIM_DIV,
							 -(s32)FC_HTRIM_STEP, (s32)FC_HTRIM_STEP);
				htrim_pitch_01 = (s16)fc_clamp((s32)htrim_pitch_01 + d,
											   -FC_HTRIM_LIMIT, FC_HTRIM_LIMIT);
			}
		}
		else
		{
			htrim_cnt = 0;				// 条件不满足：本窗口作废，重新累计
		}
#endif
	}
	else
	{
		out_r = 0;
		out_p = 0;
		out_y = 0;
		thrust = 0;
		thrust_slew = 0;
		// 上锁时清零积分，避免下次解锁时积分残留
		pid_roll_a.integ = 0;
		pid_pitch_a.integ = 0;
		pid_roll.integ = 0;
		pid_pitch.integ = 0;
		pid_yaw.integ = 0;
		pid_vz.integ = ((s32)FC_PWM_HOVER) << 10;
		pid_roll.dterm = 0;		pid_pitch.dterm = 0;	pid_yaw.dterm = 0;	pid_vz.dterm = 0;
		pid_roll.prev_meas = 0;	pid_pitch.prev_meas = 0;pid_yaw.prev_meas = 0;pid_vz.prev_meas = 0;
	}

	// 慢速推力（τ≈0.3S）：与 thrust_last 的差值大 = 推力正在变化。
	//	推力一变，桨流对气压的偏置立刻跟着变，此时气压算出的升降速度全是假的，
	//	由 fc_altitude_update() 用它做纠漂门控（见 FlightCtrl.h 7.4）。
	thrust_lpf += (((s32)thrust - thrust_lpf) >> 6);

	//--------------------------------------------------------------------------
	// ⑧ 十字型(+)四轴混控
	//	roll/pitch 差动量先乘 FC_MIX_GAIN_Q8 补偿（+ 型只有 2 只马达参与差动）
	//--------------------------------------------------------------------------
	diff_r = (out_r * FC_MIX_GAIN_Q8) >> 8;
	diff_p = (out_p * FC_MIX_GAIN_Q8) >> 8;

	//--------------------------------------------------------------------------
	// ★ 差动权限随推力收缩（2026/11/07 真机实测"拉油门降不下来"的治本修复）：
	//	姿态环摆动时差动可达 ±250 PWM，混控在下限 45 处削顶会把被削马达"抬"回
	//	45，而高端不削 → 平均推力 = (45 + thrust+A)/2 ≫ 杆位指令 → 收油门失效、
	//	持续上飞（详见 FlightCtrl.h FC_DIFF_ROOM_MAX 注释）。把差动幅度限制在
	//	"当前推力到上下限的余量"之内，平均推力就恒等于杆位指令。
	//	副作用：低油门时姿态权限变小——这正是想要的（收油门=要下来，姿态让位）。
	//--------------------------------------------------------------------------
	{
		s32 room = thrust - (FC_PWM_MIN_RUN + 6);				// 下限余量
		s32 room_hi = (s32)cMot_PwmDutyMax - 8 - thrust;		// 上限余量（防顶格丢权限）

		if(room > room_hi) room = room_hi;
		if(room > FC_DIFF_ROOM_MAX) room = FC_DIFF_ROOM_MAX;
		if(room < 0) room = 0;
		if(diff_r > room) diff_r = room;
		else if(diff_r < -room) diff_r = -room;
		if(diff_p > room) diff_p = room;
		else if(diff_p < -room) diff_p = -room;
#if (FC_UART_DBG_EN)
		dbg_diff_p = (s16)diff_p;		// 采样权限收缩后的实际差动（看是否贴 ±room）
#endif
	}

	m_front = thrust + diff_r*MIX_FRONT_ROLL + diff_p*MIX_FRONT_PITCH + out_y*MIX_FRONT_YAW;
	m_rear  = thrust + diff_r*MIX_REAR_ROLL  + diff_p*MIX_REAR_PITCH  + out_y*MIX_REAR_YAW;
	m_left  = thrust + diff_r*MIX_LEFT_ROLL  + diff_p*MIX_LEFT_PITCH  + out_y*MIX_LEFT_YAW;
	m_right = thrust + diff_r*MIX_RIGHT_ROLL + diff_p*MIX_RIGHT_PITCH + out_y*MIX_RIGHT_YAW;

	if(armed)
	{
		m_front = fc_clamp(m_front, FC_PWM_MIN_RUN, FC_PWM_MAX);
		m_rear  = fc_clamp(m_rear,  FC_PWM_MIN_RUN, FC_PWM_MAX);
		m_left  = fc_clamp(m_left,  FC_PWM_MIN_RUN, FC_PWM_MAX);
		m_right = fc_clamp(m_right, FC_PWM_MIN_RUN, FC_PWM_MAX);
	}
	else
	{
		m_front = 0;
		m_rear  = 0;
		m_left  = 0;
		m_right = 0;
	}

	MOT_FRONT_DUTY = (unsigned short)m_front;
	MOT_REAR_DUTY  = (unsigned short)m_rear;
	MOT_LEFT_DUTY  = (unsigned short)m_left;
	MOT_RIGHT_DUTY = (unsigned short)m_right;
	Update4MotPwmDuty();

	//--------------------------------------------------------------------------
	// ⑨ 调试观察量
	//--------------------------------------------------------------------------
	FlightCtrl_State  = fc_state;
	FlightCtrl_Unlocked = fc_unlocked;
	FlightCtrl_CalState = cal_state;
	FlightCtrl_Stunt  = stunt_active;
	FlightCtrl_DescBlock = ground_timer;
	FlightCtrl_Airborne = fc_airborne;
	FlightCtrl_AVert = (s16)fc_clamp(a_vert_lpf, -32768, 32767);
	FlightCtrl_AccBiasX10 = (s16)fc_clamp(a_vert_bias, -32768, 32767);
	FlightCtrl_OutRoll  = (s16)out_r;
	FlightCtrl_OutPitch = (s16)out_p;
	FlightCtrl_OutYaw   = (s16)out_y;
	FlightCtrl_Thrust   = (s16)thrust;
	FlightCtrl_AltX10   = (s16)fc_clamp(alt_est_x10, -32768, 32767);
	FlightCtrl_VzX10    = (s16)fc_clamp(vz_est_x10, -32768, 32767);
	FlightCtrl_VzBaroX10 = (s16)fc_clamp(vz_baro_x10, -32768, 32767);
	FlightCtrl_HTrimR01 = htrim_roll_01;
	FlightCtrl_HTrimP01 = htrim_pitch_01;
	FlightCtrl_VzAccX10 = (s16)fc_clamp(vz_acc_q / 1000, -32768, 32767);
	FlightCtrl_VzCmdX10 = (s16)fc_clamp(vz_cmd_x10, -32768, 32767);
	FlightCtrl_AltErrX10 = (s16)fc_clamp(alt_err_dbg, -32768, 32767);
	FlightCtrl_AltTargetX10 = alt_target_x10;

	// 气压计健康度 / 停桨现场（PresCnt/PresDrop 在 fc_altitude_update 里直接累加）
	FlightCtrl_PresRaw    = (s32)s32PresValue;
	FlightCtrl_PresAltX10 = pres_alt_dbg;
	FlightCtrl_DisarmReason = fc_disarm_reason;
	FlightCtrl_DisarmAltX10 = dis_alt_x10;
	FlightCtrl_DisarmTilt   = dis_tilt_01;
	// ★v23 解锁诊断镜：节拍活性 / 组合状态机 / 假失联峰值
	FlightCtrl_TickCnt++;
	FlightCtrl_ComboType = combo_type;
	FlightCtrl_ComboHold = combo_hold;
	if(u32NoRxdCnt > (u32)FlightCtrl_NoRxdPeak)
	{
		FlightCtrl_NoRxdPeak = (u16)((u32NoRxdCnt > 65535u) ? 65535u : u32NoRxdCnt);
	}

	//--------------------------------------------------------------------------
	// ★v25 flash 黑匣子状态机（FlightCtrl.h 7.11 精简版）：录制/读出
	//--------------------------------------------------------------------------
	fc_blackbox_tick(out_r, out_p, thrust, rate_x, rate_y);

#if (FC_UART_DBG_EN)
	//--------------------------------------------------------------------------
	// ⑩ UART 高速调试报文（协议见 FlightCtrl.h 7.9）：每 5mS 一包 21 字节
	//	115200 波特率下一包耗时 ~1.82mS，占用节拍带宽 ~37%，DMA 后台发送。
	//	★v25：黑匣子 UART 读出期间暂停（实时帧与读出流会互相污染解析）
	//--------------------------------------------------------------------------
	if(bb_state != FC_BB_ST_DUMP)
	{
		unsigned char pk[21];
		unsigned char i, sum;
		u8 flying = (fc_state != FC_ST_DISARM) ? 1 : 0;
		s16 v_pm  = flying ? dbg_pitch_meas : 0;
		s16 v_ry  = flying ? dbg_rate_y     : 0;
		s16 v_vx  = flying ? dbg_vx         : 0;		// ★v16: 原 pitch_rate_cmd
		s16 v_op  = flying ? (s16)out_p     : 0;
		s16 v_rm  = flying ? dbg_roll_meas  : 0;	// ★v15: 原 diff_p
		s16 v_thr = flying ? (s16)thrust    : 0;
		s16 v_vy  = flying ? dbg_vy         : 0;		// ★v16: 原 rate_x
		s16 v_vz  = flying ? (s16)fc_clamp(vz_est_x10, -32768, 32767) : 0;	// ★v16: 原 htrim

		pk[0] = 0x26;  pk[1] = 0x99;
		pk[2] = dbg_seq++;
		pk[3]  = (unsigned char)(v_pm  >> 8);  pk[4]  = (unsigned char)v_pm;
		pk[5]  = (unsigned char)(v_ry  >> 8);  pk[6]  = (unsigned char)v_ry;
		pk[7]  = (unsigned char)(v_vx  >> 8);  pk[8]  = (unsigned char)v_vx;
		pk[9]  = (unsigned char)(v_op  >> 8);  pk[10] = (unsigned char)v_op;
		pk[11] = (unsigned char)(v_rm  >> 8);  pk[12] = (unsigned char)v_rm;
		pk[13] = (unsigned char)(v_thr >> 8);  pk[14] = (unsigned char)v_thr;
		pk[15] = (unsigned char)(v_vy  >> 8);  pk[16] = (unsigned char)v_vy;
		pk[17] = (unsigned char)(v_vz  >> 8);  pk[18] = (unsigned char)v_vz;
		pk[19] = (unsigned char)FC_PARAM_VER;			// 参数版本（验证烧录）
		sum = 0;
		for(i = 2; i < 20; i++) sum += pk[i];
		pk[20] = sum;

		UartDbgSend(pk, 21);
	}
#endif

}

//**********************************************************************************
//**********************************************************************************
