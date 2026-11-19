/***************************************************************************************
 *  File: AttMahony.h
 *
 *  Mahony 互补滤波姿态解算（定点实现，纯算法模块）
 *
 *  为什么用定点：TC32 工具链没有 libgcc/软浮点库，float 运算无法链接。
 *  本模块只用 32 位整数（Q15 四元数、Q10 角速度），可在 PC 上单独编译测试。
 *
 *  职责：只接收校准后的机体 FRD 六轴和时间，输出四元数与欧拉角。
 *  不访问 I2C / UART / 控制器。
 *
 *  坐标约定：
 *    - 机体 FRD：+X 前、+Y 右、+Z 下
 *    - 四元数 q = [w, x, y, z]（Q15），表示"机体 FRD → 局部水平系 H"的旋转
 *    - 局部水平系 H：Z 轴向下、X 轴取上电时的机头方向（相对偏航为 0）
 *    - 正放静止时 FRD 加速度约 (0, 0, -1g)
 ****************************************************************************************/
#ifndef __ATT_MAHONY_H__
#define __ATT_MAHONY_H__

// 不使用 <stdint.h>：SDK 的 types.h 自己定义了 int32_t/uint32_t，会冲突。
typedef signed int    att_i32;
typedef unsigned int  att_u32;
typedef unsigned char att_u8;
typedef unsigned short att_u16;

#define ATT_Q15_ONE   32768     // Q15 的 1.0

typedef struct {
    att_i32 q[4];              // 四元数 w, x, y, z（Q15，机体 FRD → 水平系）
    att_i32 gyro_prev[3];      // 上一拍角速度（Q10 rad/s，FRD），用于梯形积分
    att_i32 dq_rem[4];         // Δq 的整数截断余数（1/10000 LSB），避免旋转速率偏慢
    att_u8  have_prev;         // 是否已有一拍角速度
    att_u8  ready;             // 姿态是否已就绪
    att_u32 t_ms;              // 最近一次有效更新的时间戳（毫秒）
} att_mahony_t;

// 复位到单位四元数（未就绪）
void AttMahony_Reset(att_mahony_t *m);

// 用一段静止平均加速度初始化横滚/俯仰（偏航置 0，允许斜放启动，输出实际倾角）
// acc_mg: 机体 FRD 三轴加速度，单位 mg（1g = 1000）
// 返回 1 成功、0 失败（模长过小）
att_u8 AttMahony_InitFromAccel(att_mahony_t *m, const att_i32 acc_mg[3], att_u32 t_ms);

// 单次更新
//   acc_mg    : 校准（并低通）后的机体 FRD 加速度，单位 mg
//   gyro_q10  : 扣零偏后的机体 FRD 角速度，单位 Q10 rad/s（1 rad/s = 1024）
//   dt_100us  : 与上一有效样本的实际时间差，单位 0.1ms（5ms → 50）
// 返回 1 表示本拍已更新，0 表示未就绪或输入异常
att_u8 AttMahony_Update(att_mahony_t *m, const att_i32 acc_mg[3], const att_i32 gyro_q10[3],
                        att_i32 dt_100us, att_u32 t_ms);

// 从四元数取欧拉角（单位 0.01 度）：右侧下沉横滚为正、抬头俯仰为正、俯视顺时针偏航为正
void AttMahony_GetEulerDeg100(const att_mahony_t *m, att_i32 *roll, att_i32 *pitch, att_i32 *yaw);

#endif // __ATT_MAHONY_H__
