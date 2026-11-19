//**********************************************************************************
/*
 * PerfMeas.c
 *
 *	200Hz 控制任务计时统计与 0x26 0x9B 统计帧
 *
 *	统计口径（务必按此解读）：
 *	  - 控制任务总耗时 = FlightCtrl_Tick() 的墙钟时间，包含任务执行期间的中断和等待
 *	    （里面也有六轴报文写入、黑匣子写入等），不代表纯 CPU 占用；
 *	  - IMU 区间（IIC 读 + 5 点中值滤波）包含在控制总耗时内，两者不能相加；
 *	  - 启动迟到 = 实际开始 tick - 计划开始 tick（计划值来自 FlightCtrlSchedule 的
 *	    u32_FlightCtrlTick 累加），首次连接等偶发长延迟保留在数据里；
 *	  - 截止违约 = 计划启动 → 任务返回 超过 PERF_DEADLINE_US(5mS) 的次数，
 *	    单窗口计数 + 上电累计计数（累计值不受串口丢帧影响）。
 *
 *	统计帧（44 字节，多字节字段一律大端）：
 *	  偏移  长度  内容
 *	   0     1    0x26            帧头1
 *	   1     1    0x9B            帧头2
 *	   2     1    ver             = PERF_FRAME_VER
 *	   3     1    seq             帧序号（0~255，漏帧可见跳变）
 *	   4     4    build_id        PERF_BUILD_ID
 *	   8     2    window_ms       本窗口长度 mS
 *	  10     2    task_cnt        窗口内控制任务调用次数
 *	  12     2    task_avg_us     控制任务平均耗时 uS
 *	  14     4    task_max_us     控制任务最大耗时 uS
 *	  18     2    imu_cnt         IMU 区间调用次数
 *	  20     2    imu_avg_us      IMU 区间平均耗时 uS
 *	  22     4    imu_max_us      IMU 区间最大耗时 uS
 *	  26     4    late_max_us     最大启动迟到 uS
 *	  30     2    int_min_us      相邻两轮实际启动间隔最小值 uS
 *	  32     2    int_max_us      相邻两轮实际启动间隔最大值 uS
 *	  34     2    miss_win        本窗口截止违约次数
 *	  36     4    miss_total      上电累计截止违约次数
 *	  40     2    drop_total      统计帧发送丢弃累计次数
 *	  42     1    reserved        保留（0）
 *	  43     1    chk             校验和 = sum(byte[2..42]) & 0xFF
 *
 *	发送策略：主循环调用 PerfMeas_Poll()，尝试发送；TX 忙则跳过并计数，
 *	          不等待、不 printf，且完全在控制任务计时区间之外。
 */
//**********************************************************************************
#include "types.h"
#include "app_public_config.h"
#include "driver.h"                 // clock_time()
#include "PerfMeas.h"
#include "UserUart.h"               // UartTrySend()

#if (PERF_MEAS_EN)

// 24MHz → 24 tick/uS
#define PERF_TICK_PER_US        (CLOCK_SYS_CLOCK_HZ / 1000000)
// 统计窗口：1S
#define PERF_REPORT_TICKS       ((u32)CLOCK_SYS_CLOCK_HZ / 1000u * PERF_REPORT_MS)
// 发送失败后的重试间隔：100mS（避免主循环里疯狂重试）
#define PERF_RETRY_TICKS        ((u32)CLOCK_SYS_CLOCK_HZ / 1000u * 100u)
// 截止时间换算成 tick
#define PERF_DEADLINE_TICKS     ((u32)PERF_DEADLINE_US * PERF_TICK_PER_US)
// 统计帧发送窗口（与六轴帧错开）：
//   六轴帧每 10mS 发一帧、约 2.4mS 发完；统计帧 44B 约 3.8mS。
//   若统计帧正在发，六轴帧的 UartDbgSend 会等它发完，把控制任务拖长 3~4mS，
//   污染测量。所以统计帧只在"六轴帧发出 3mS 之后、7mS 之前"启动发送；
//   50mS 内没有六轴帧（未开启六轴报文）时不受此限制。
#define PERF_TX_GUARD_TICKS     ((u32)CLOCK_SYS_CLOCK_HZ / 1000u * 3u)
#define PERF_TX_WINDOW_TICKS    ((u32)CLOCK_SYS_CLOCK_HZ / 1000u * 7u)
#define PERF_TX_IDLE_TICKS      ((u32)CLOCK_SYS_CLOCK_HZ / 1000u * 50u)

static u8		s_inited;
static u32		s_win_start;		// 本窗口起点 tick
static u32		s_task_cnt;			// 窗口内控制任务次数
static u32		s_task_sum_tick;	// 窗口内累计耗时 tick
static u32		s_task_max_tick;	// 窗口内最大耗时 tick
static u32		s_imu_cnt;			// 窗口内 IMU 区间次数
static u32		s_imu_sum_tick;		// 窗口内 IMU 累计 tick
static u32		s_imu_max_tick;		// 窗口内 IMU 最大 tick
static u32		s_late_max_tick;	// 窗口内最大启动迟到 tick
static u32		s_int_min_tick;		// 窗口内相邻启动间隔最小 tick
static u32		s_int_max_tick;		// 窗口内相邻启动间隔最大 tick
static u32		s_last_start;		// 上一轮实际启动 tick
static u8		s_have_last;		// 是否已有上一轮
static u32		s_miss_win;			// 本窗口截止违约次数
static u32		s_miss_total;		// 上电累计截止违约次数
static u32		s_drop_total;		// 统计帧发送丢弃累计
static u32		s_last_try;			// 上次尝试发送 tick
static u8		s_send_fail;		// 上次发送失败
static u8		s_seq;				// 统计帧序号

//**********************************************************************************
// 清零窗口统计（不影响上电累计值）
//**********************************************************************************
static void perf_reset_window(u32 now)
{
	s_win_start		= now;
	s_task_cnt		= 0;
	s_task_sum_tick	= 0;
	s_task_max_tick	= 0;
	s_imu_cnt		= 0;
	s_imu_sum_tick	= 0;
	s_imu_max_tick	= 0;
	s_late_max_tick	= 0;
	s_int_min_tick	= 0xFFFFFFFFu;
	s_int_max_tick	= 0;
	s_have_last		= 0;
	s_miss_win		= 0;
}

//**********************************************************************************
// 初始化（首次调用自动执行）
//**********************************************************************************
void PerfMeas_Init(void)
{
	u32 now = clock_time();

	s_miss_total	= 0;
	s_drop_total	= 0;
	s_seq			= 0;
	s_last_try		= now;
	s_send_fail		= 0;
	s_inited		= 1;
	perf_reset_window(now);
}

//**********************************************************************************
// tick → uS（24 分频；只在发帧时调用，不进控制热路径）
//**********************************************************************************
static u32 perf_tick_to_us(u32 tick)
{
	return tick / PERF_TICK_PER_US;
}

//**********************************************************************************
// 控制任务开始：记录计划时刻与实际时刻
//**********************************************************************************
void PerfMeas_TaskBegin(perf_tick_t *t, u32 planned_tick)
{
	if(!s_inited) PerfMeas_Init();

	t->planned = planned_tick;
	t->actual  = clock_time();
}

//**********************************************************************************
// 控制任务返回：累计耗时/最大耗时/迟到/违约/启动间隔
//	注意：本函数自己的执行时间不计入任务耗时（end 在入口处取）
//**********************************************************************************
void PerfMeas_TaskEnd(perf_tick_t *t)
{
	u32 end   = clock_time();
	u32 dur   = end - t->actual;			// 无符号相减，回绕安全
	u32 late  = t->actual - t->planned;		// 迟到（正常调度下 >= 0）
	u32 total = end - t->planned;			// 计划启动 → 返回

	s_task_cnt++;
	s_task_sum_tick += dur;
	if(dur > s_task_max_tick)	s_task_max_tick = dur;
	if(late > s_late_max_tick)	s_late_max_tick = late;

	// 截止违约：计划启动到返回超过 5mS（窗口计数 + 上电累计）
	if(total > PERF_DEADLINE_TICKS)
	{
		s_miss_win++;
		s_miss_total++;
	}

	// 相邻两轮实际启动间隔（判断"迟到后是否补跑"）
	if(s_have_last)
	{
		u32 iv = t->actual - s_last_start;
		if(iv < s_int_min_tick)	s_int_min_tick = iv;
		if(iv > s_int_max_tick)	s_int_max_tick = iv;
	}
	s_last_start = t->actual;
	s_have_last  = 1;
}

//**********************************************************************************
// IMU 区间（IIC 读 + 中值滤波）耗时，单位 tick
//**********************************************************************************
void PerfMeas_ImuAdd(u32 ticks)
{
	s_imu_cnt++;
	s_imu_sum_tick += ticks;
	if(ticks > s_imu_max_tick)	s_imu_max_tick = ticks;
}

//**********************************************************************************
// 主循环轮询：窗口到点就尝试发一帧（TX 忙则丢弃并计数，绝不等待）
//**********************************************************************************
void PerfMeas_Poll(void)
{
	unsigned char	pk[PERF_FRAME_LEN];
	unsigned char	i, sum;
	u32				now, elapsed, win_ms;
	u32				task_avg, imu_avg;

	if(!s_inited) PerfMeas_Init();

	now		= clock_time();
	elapsed	= now - s_win_start;
	if(elapsed < PERF_REPORT_TICKS) return;
	// 上次失败后等 100mS 再试，避免主循环空转重试
	if(s_send_fail && ((u32)(now - s_last_try) < PERF_RETRY_TICKS)) return;

	// 与六轴帧错开：等它发完（>=3mS）再启动统计帧，且不撞下一帧（<=7mS）
	{
		u32 since = now - g_uart_tx_start_tick;
		if(since < PERF_TX_GUARD_TICKS) return;
		if((since > PERF_TX_WINDOW_TICKS) && (since < PERF_TX_IDLE_TICKS)) return;
	}

	// 窗口毫秒数（长时间发不出去时截断，避免 u16 溢出）
	win_ms = elapsed / (PERF_TICK_PER_US * 1000u);
	if(win_ms > 0xFFFFu) win_ms = 0xFFFFu;

	pk[0] = 0x26;
	pk[1] = 0x9B;
	pk[2] = PERF_FRAME_VER;
	pk[3] = s_seq;

	pk[4]  = (unsigned char)(PERF_BUILD_ID >> 24);
	pk[5]  = (unsigned char)(PERF_BUILD_ID >> 16);
	pk[6]  = (unsigned char)(PERF_BUILD_ID >> 8);
	pk[7]  = (unsigned char)PERF_BUILD_ID;

	pk[8]  = (unsigned char)(win_ms >> 8);
	pk[9]  = (unsigned char)win_ms;

	pk[10] = (unsigned char)(s_task_cnt >> 8);
	pk[11] = (unsigned char)s_task_cnt;

	task_avg = s_task_cnt ? (perf_tick_to_us(s_task_sum_tick) / s_task_cnt) : 0;
	pk[12] = (unsigned char)(task_avg >> 8);
	pk[13] = (unsigned char)task_avg;

	pk[14] = (unsigned char)(perf_tick_to_us(s_task_max_tick) >> 24);
	pk[15] = (unsigned char)(perf_tick_to_us(s_task_max_tick) >> 16);
	pk[16] = (unsigned char)(perf_tick_to_us(s_task_max_tick) >> 8);
	pk[17] = (unsigned char)perf_tick_to_us(s_task_max_tick);

	pk[18] = (unsigned char)(s_imu_cnt >> 8);
	pk[19] = (unsigned char)s_imu_cnt;

	imu_avg = s_imu_cnt ? (perf_tick_to_us(s_imu_sum_tick) / s_imu_cnt) : 0;
	pk[20] = (unsigned char)(imu_avg >> 8);
	pk[21] = (unsigned char)imu_avg;

	pk[22] = (unsigned char)(perf_tick_to_us(s_imu_max_tick) >> 24);
	pk[23] = (unsigned char)(perf_tick_to_us(s_imu_max_tick) >> 16);
	pk[24] = (unsigned char)(perf_tick_to_us(s_imu_max_tick) >> 8);
	pk[25] = (unsigned char)perf_tick_to_us(s_imu_max_tick);

	pk[26] = (unsigned char)(perf_tick_to_us(s_late_max_tick) >> 24);
	pk[27] = (unsigned char)(perf_tick_to_us(s_late_max_tick) >> 16);
	pk[28] = (unsigned char)(perf_tick_to_us(s_late_max_tick) >> 8);
	pk[29] = (unsigned char)perf_tick_to_us(s_late_max_tick);

	// 窗口内没有两轮以上时，间隔给 0
	{
		u32 iv_min = (s_int_min_tick == 0xFFFFFFFFu) ? 0 : perf_tick_to_us(s_int_min_tick);
		u32 iv_max = perf_tick_to_us(s_int_max_tick);
		if(iv_min > 0xFFFFu) iv_min = 0xFFFFu;
		if(iv_max > 0xFFFFu) iv_max = 0xFFFFu;
		pk[30] = (unsigned char)(iv_min >> 8);
		pk[31] = (unsigned char)iv_min;
		pk[32] = (unsigned char)(iv_max >> 8);
		pk[33] = (unsigned char)iv_max;
	}

	pk[34] = (unsigned char)(s_miss_win >> 8);
	pk[35] = (unsigned char)s_miss_win;

	pk[36] = (unsigned char)(s_miss_total >> 24);
	pk[37] = (unsigned char)(s_miss_total >> 16);
	pk[38] = (unsigned char)(s_miss_total >> 8);
	pk[39] = (unsigned char)s_miss_total;

	pk[40] = (unsigned char)(s_drop_total >> 8);
	pk[41] = (unsigned char)s_drop_total;
	pk[42] = 0;							// 保留

	sum = 0;
	for(i = 2; i < 43; i++)	sum += pk[i];
	pk[43] = sum;

	s_last_try = now;
	// 尝试发送：TX 忙直接返回 0，不等待（统计在控制任务计时区间之外）
	if(UartTrySend(pk, PERF_FRAME_LEN))
	{
		s_send_fail = 0;
		s_seq++;
		perf_reset_window(now);			// 以实际发送时刻作为新窗口起点
	}
	else
	{
		s_send_fail = 1;
		s_drop_total++;
	}
}

#endif /* PERF_MEAS_EN */
//**********************************************************************************
