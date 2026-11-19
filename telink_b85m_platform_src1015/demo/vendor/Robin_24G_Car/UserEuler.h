//********************************************************************
/*
 * UserEuler.h
 *
 *  姿态估计（定点四元数互补滤波）+ 台架姿态估计唯一配置来源
 *
 *  说明：
 *    - 旧版欧拉角积分、ATAN_LUT 与旧姿态解算入口已删除，本文件只有一套四元数估计；
 *    - 全部使用 32 位整数（Q15 四元数、Q10 rad/s、mg 加速度），
 *      因为 TC32 工具链没有 libgcc/软浮点，float 无法链接；
 *    - 坐标约定：机体 FRD（+X 前、+Y 右、+Z 下），四元数 q=[w,x,y,z] 表示
 *      "机体 FRD → 局部水平系 H"的旋转；H 的 Z 轴向下、X 轴取上电时的机头方向，
 *      相对偏航从 0 开始；正放静止时 FRD 加速度约 (0,0,-1g)。
 */
//********************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USEREULER_H_
#define VENDOR_ROBIN_24G_CAR_USEREULER_H_
//********************************************************************
//********************************************************************
// 台架姿态估计配置（唯一配置来源，改这里即可）
//   1 = 主循环只跑 IMU 采样 + 姿态估计 + 串口输出（旧控制器/混控/气压不参与）
//   0 = 恢复旧控制链（旧控制路径仍在源码里，留给后续重构）
//********************************************************************
#define C_ATT_BENCH_EN            1

// ---- 采样与串口 ----
#define C_ATT_SAMPLE_HZ           400       // MCU 读取率（台架实测目标 400Hz）
#define C_ATT_SAMPLE_US           (1000000 / C_ATT_SAMPLE_HZ)   // 标称采样周期 2500us
#define C_ATT_TX_HZ               14        // 姿态文本频率
#define C_ATT_STAT_HZ             1         // 统计文本频率

// ---- T-3488 量程与灵敏度 ----
#define C_ACC_LSB_PER_G           2048      // ±16g
#define C_GYRO_LSB_PER_DPS_X10    164       // 16.4 ×10（±2000dps）

// ---- 陀螺片内 UI 低通：IPREG SYS1 偏移 0x09，bit[2:0] ----
//   800Hz ODR：0=ODR/2 1=ODR/4 2=ODR/8 3=ODR/16 4=ODR/32 5=ODR/64
#define C_GYRO_LPF_SEL            3         // ODR/16 = 50Hz（台架初值）
#define C_GYRO_LPF_HZ_NOMINAL     50        // 仅用于串口显示

// ---- 加速度一阶低通（校准后的机体 FRD 上做）----
#define C_ACC_LPF_EN              1         // 0 = 旁路
#define C_ACC_LPF_TAU_US          5305      // 一阶时间常数：1/(2π·30Hz)

// ---- Mahony 互补滤波 ----
#define C_MAHONY_KP_Q15           16384     // 比例纠正 0.5（慢速）
#define C_ATT_TRUST_MIN_MG        700       // 加速度模长 <0.70g：暂停纠偏
#define C_ATT_TRUST_FULL_LO_MG    850       // 0.85g 起完全信任
#define C_ATT_TRUST_FULL_HI_MG    1150      // 1.15g 以内完全信任
#define C_ATT_TRUST_MAX_MG        1300      // >1.30g 暂停纠偏

// ---- 启动静止判定（连续静止一段才初始化姿态）----
#define C_ATT_INIT_STILL_US       500000    // 需要连续静止 500ms
#define C_ATT_INIT_GYRO_MAX_Q10   102       // 单轴角速度上限 0.1 rad/s
#define C_ATT_INIT_ACC_DEV_MG     60        // 相邻样本加速度变化上限
#define C_ATT_INIT_ACC_LO_MG      700       // 静止样本模长范围
#define C_ATT_INIT_ACC_HI_MG      1300

// ---- 采样连续性 ----
#define C_ATT_MAX_GAP_100US       1000      // 相邻有效样本 >100ms：重建积分端点

//********************************************************************
// 定点类型
//********************************************************************
typedef signed int    att_i32;
typedef unsigned int  att_u32;
typedef unsigned char att_u8;
typedef unsigned short att_u16;

#define ATT_Q15_ONE   32768     // Q15 的 1.0

// 四元数姿态估计器状态
typedef struct {
	att_i32 q[4];              // 四元数 w, x, y, z（Q15）
	att_i32 gyro_prev[3];      // 上一拍角速度（Q10 rad/s），用于梯形积分
	att_i32 dq_rem[4];         // Δq 整数截断的余数（1/10000 LSB）
	att_i32 trap_rem[3];       // 梯形平均的余数（1/2 Q10），避免负数取整偏置
	att_u8  have_prev;         // 是否已有一拍角速度
	att_u8  ready;             // 姿态是否已就绪
	att_u32 t_ms;              // 最近一次有效更新时刻（毫秒）
} euler_q_t;

//********************************************************************
// 接口
//********************************************************************
void Euler_Reset(euler_q_t *m);

// 用静止样本的平均加速度初始化横滚/俯仰（偏航置 0，允许斜放、倒放启动）
// acc_mg: 机体 FRD 三轴加速度，单位 mg（1g = 1000）
// 返回 1 成功、0 失败（模长过小）
att_u8 Euler_InitFromAccel(euler_q_t *m, const att_i32 acc_mg[3], att_u32 t_ms);

// 单次更新：陀螺梯形积分 + 加速度比例纠偏
//   acc_mg   : 校准（并低通）后的机体 FRD 加速度，单位 mg
//   gyro_q10 : 扣零偏后的机体 FRD 角速度，单位 Q10 rad/s
//   dt_100us : 与上一有效样本的实际时间差，单位 0.1ms
// 加速度模长过小时只跳过纠偏，陀螺仍然照常积分
att_u8 Euler_Update(euler_q_t *m, const att_i32 acc_mg[3], const att_i32 gyro_q10[3],
                    att_i32 dt_100us, att_u32 t_ms);

// 陀螺原始计数 → Q10 rad/s（先按 1/64 LSB 扣掉零偏，再乘移）
//   raw_lsb : 已经映射到 FRD 的原始计数
//   bias_q6 : 实测零偏，单位 1/64 LSB（例如 -1.1552 LSB → -74）
//   rem     : 每轴换算余数（调用者保持），把舍入丢掉的小数带到下一拍，
//             长期平均既不偏正也不偏负；静置时偏航不会一点点漂走
att_i32 Euler_GyroToQ10(att_i32 raw_lsb, att_i32 bias_q6, att_i32 *rem);

// 启动静止判定：这一拍是否可以计入"连续静止窗口"
//   acc_mg/gyr_q10 为校准后的值；prev_acc/have_prev 用于检查相邻样本的跳变
//   返回 1 静止、0 不静止（移动、倾斜过大或读数跳变）
att_u8 Euler_IsStill(const att_i32 acc_mg[3], const att_i32 gyr_q10[3],
                     const att_i32 prev_acc[3], att_u8 have_prev);

// 从四元数取欧拉角（单位 0.01 度）
//   右侧下沉横滚为正、抬头俯仰为正、俯视顺时针偏航为正
void Euler_GetEulerDeg100(const euler_q_t *m, att_i32 *roll, att_i32 *pitch, att_i32 *yaw);

//********************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USEREULER_H_ */
//********************************************************************
