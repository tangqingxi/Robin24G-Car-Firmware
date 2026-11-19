//**********************************************************************************
/*
 * PerfMeas.h
 *
 *	200Hz 控制任务的计时统计（只测量，不改控制行为）
 *
 *	① 控制任务总耗时：FlightCtrl_Tick() 入口到返回（墙钟，含中断/等待）
 *	② IMU 读取 + 中值滤波耗时：t3488_data_filter_handler() 前后
 *	③ 启动迟到：计划启动时刻 → 实际启动时刻；另记相邻两轮实际启动间隔
 *	④ 截止时间违约：计划启动 → 任务返回 超过 PERF_DEADLINE_US 的次数
 *
 *	计时源沿用 clock_time()（24MHz 系统 tick，32 位，约 179S 回绕），
 *	所有差值都用无符号相减，天然处理回绕。
 */
//**********************************************************************************
#ifndef PERF_MEAS_H_
#define PERF_MEAS_H_

#include "types.h"

// 编译开关：1 = 编译计时统计并每秒发送 0x269B 统计帧（本次测量固件打开）
//            0 = 完全编译掉，固件行为与改动前一致
#ifndef PERF_MEAS_EN
#define PERF_MEAS_EN        1
#endif

#define PERF_FRAME_VER      1               // 统计报文协议版本
#define PERF_FRAME_LEN      44              // 统计报文总长（字节，大端）
#define PERF_BUILD_ID       0x20261119u     // 构建标识（2026-11-19 测量固件）
#define PERF_REPORT_MS      1000            // 统计窗口长度（mS）
#define PERF_DEADLINE_US    5000            // 截止时间（uS），即 5mS 控制周期

// 单轮计时的上下文（planned/actual 都是 clock_time() 的 tick）
typedef struct{
	u32 planned;        // 本轮计划启动 tick
	u32 actual;         // 本轮实际启动 tick
}perf_tick_t;

void PerfMeas_Init(void);                                   // 首次使用自动调用，也可显式调用
void PerfMeas_TaskBegin(perf_tick_t *t, u32 planned_tick);  // 控制任务开始前
void PerfMeas_TaskEnd(perf_tick_t *t);                      // 控制任务返回后
void PerfMeas_ImuAdd(u32 ticks);                            // IMU 区间耗时（tick）
void PerfMeas_Poll(void);                                   // 主循环调用：约 1S 发一帧统计

#endif /* PERF_MEAS_H_ */
