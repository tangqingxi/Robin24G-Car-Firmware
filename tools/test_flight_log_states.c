/*
 * 两条真实 FlightLoop 日志状态回归：
 * locked：遥控未连接、电机锁定开始采集，LOG1 后持续采样，最后 0001 明确停录。
 * unlock：真实解锁使 PWM 非零，随后停止喂包，验证先清零再导出失联/超时事件。
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "FlightLoop.h"
#include "FlightConfig.h"
#include "FlightLog.h"

unsigned short LFmotDuty;
unsigned short LBmotDuty;
unsigned short RBmotDuty;
unsigned short RFmotDuty;
void User_pwm_init_func(void) {}
void Update4MotPwmDuty(void) {}

static unsigned char flash_mem[0x20000];

void flash_read_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	memcpy(buf, &flash_mem[addr], len);
}
void flash_write_page(unsigned int addr, unsigned int len, unsigned char *buf)
{
	memcpy(&flash_mem[addr], buf, len);
}
void flash_erase_sector(unsigned int addr)
{
	memset(&flash_mem[addr], 0xFF, 4096);
}

u8 t3488_bench_init(u8 lpf, u8 *rb) { (void)lpf; if (rb) *rb = 3; return 0; }
u8 t3488_read_6axis(short *a, short *g)
{
	a[0] = -22; a[1] = 8; a[2] = 2100;
	g[0] = g[1] = g[2] = 0;
	return 0;
}
static unsigned int g_now;
u32 clock_time(void) { return g_now; }
void FlightLoop_StartupTestPwm(unsigned short out[4]);
unsigned char FlightLoop_StartupTestReady(void);
unsigned char FlightLoop_StartupTestArmed(void);

static void step(void)
{
	g_now += FLIGHT_PERIOD_TICKS;
	FlightLoop_Process();
	if (FlightLog_State() == FLIGHT_LOG_PREP && FlightLoop_CanEraseLog()) {
		FlightLog_BeginPrepare();
	}
	FlightLog_Service();
}

static void boot_ready(void)
{
	unsigned int i;

	memset(flash_mem, 0xFF, sizeof(flash_mem));
	FlightLoop_Init();
	for (i = 0; i < 140; i++) {
		step();
	}
	if (!FlightLoop_StartupTestReady()) {
		printf("FAIL startup not ready\n");
		exit(1);
	}
}

static void start_key(void)
{
	FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, FLIGHT_LOG_KEY_START);
	step();
	if (!FlightLog_Active()) {
		printf("FAIL key 0x0004 did not start\n");
		exit(1);
	}
}

static void dump(const char *path)
{
	FILE *f = fopen(path, "wb");

	if (!f) {
		printf("FAIL dump %s\n", path);
		exit(1);
	}
	fwrite(&flash_mem[FLIGHT_LOG_BASE], 1,
		   FLIGHT_LOG_END - FLIGHT_LOG_BASE, f);
	fclose(f);
}

int main(int argc, char **argv)
{
	unsigned int i;
	unsigned short pwm[4];

	if (argc < 2) return 2;
	boot_ready();
	if (strcmp(argv[1], "nokey") == 0) {
		for (i = 0; i < 100; i++) {
			step();
		}
		if (FlightLog_Active() || FlightLog_State() != FLIGHT_LOG_IDLE) {
			printf("FAIL log started without key\n");
			return 1;
		}
		printf("PASS no key no log\n");
		return 0;
	}
	if (strcmp(argv[1], "armed") == 0) {
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0002);
		step();
		if (!FlightLoop_StartupTestArmed()) {
			printf("FAIL armed precondition\n");
			return 1;
		}
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, FLIGHT_LOG_KEY_START);
		step();
		if (FlightLog_Active()) {
			printf("FAIL record key accepted while armed\n");
			return 1;
		}
		printf("PASS record key ignored while armed\n");
		return 0;
	}
	if (strcmp(argv[1], "estopkey") == 0) {
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000,
							  FLIGHT_LOG_KEY_START | 0x0001);
		step();
		FlightLoop_StartupTestPwm(pwm);
		if (FlightLog_Active() || (pwm[0] | pwm[1] | pwm[2] | pwm[3]) != 0) {
			printf("FAIL estop+record packet started or PWM nonzero\n");
			return 1;
		}
		printf("PASS estop priority blocks record key\n");
		return 0;
	}
	if (strcmp(argv[1], "estopkey") == 0) {
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000,
							  FLIGHT_LOG_KEY_START | 0x0001);
		step();
		FlightLoop_StartupTestPwm(pwm);
		if (FlightLog_Active() || (pwm[0] | pwm[1] | pwm[2] | pwm[3]) != 0) {
			printf("FAIL estop+record packet started or PWM nonzero\n");
			return 1;
		}
		printf("PASS estop priority blocks record key\n");
		return 0;
	}
	if (strcmp(argv[1], "locked") == 0) {
		start_key();
		for (i = 0; i < 400; i++) {
			step();                 // 不喂任何遥控包，持续锁定也应继续采集
		}
		if (!FlightLog_Active()) {
			printf("FAIL locked persistent state stopped log\n");
			return 1;
		}
		// 录制中再次按键：直接忽略，不能覆盖已有数据。
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0000);
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, FLIGHT_LOG_KEY_START);
		step();
		if (!FlightLog_Active()) {
			printf("FAIL second record key stopped active log\n");
			return 1;
		}
		// 故意让队列积压，停机时这些样本应计入 Dropped。
		for (i = 0; i < 80; i++) {
			g_now += FLIGHT_PERIOD_TICKS;
			FlightLoop_Process();
		}
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x1000, 0x1000, 0x0001);
		FlightLog_Service();
		if (FlightLog_Active()) {
			printf("FAIL explicit estop did not stop log\n");
			return 1;
		}
		if (FlightLog_Dropped() == 0) {
			printf("FAIL queued samples not counted as dropped\n");
			return 1;
		}
		dump("flight_log_locked.bin");
		printf("PASS locked session continued then estop stopped; dropped=%u\n",
			   FlightLog_Dropped());
		return 0;
	}

	if (strcmp(argv[1], "unlock") == 0) {
		start_key();
		FlightLoop_OnRcPacket(0x1000, 0x1000, 0x0400, 0x1000, 0x0002);
		step();
		FlightLoop_StartupTestPwm(pwm);
		if ((pwm[0] | pwm[1] | pwm[2] | pwm[3]) == 0) {
			printf("FAIL unlock PWM still zero\n");
			return 1;
		}
		// 通过真实 FlightLoop 读到的 PWM 由导出 CSV 核对；这里先继续运行几拍。
		for (i = 0; i < 20; i++) {
			step();
		}
		// 停止喂遥控包：先 200ms 指令超时，或 500ms 失联，都会从 armed 落到锁定。
		for (i = 0; i < 220; i++) {
			step();
		}
		if (FlightLog_Active()) {
			printf("FAIL link/cmd lock did not stop log\n");
			return 1;
		}
		dump("flight_log_unlock.bin");
		printf("PASS unlocked->lock event after packets stopped; dropped=%u\n",
			   FlightLog_Dropped());
		return 0;
	}
	return 2;
}
