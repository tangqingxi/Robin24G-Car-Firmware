#include <stdio.h>
#include <string.h>
#include "GroundDiag.h"
#include "FlightConfig.h"

static unsigned int now;
static int busy;
static char line[160];
static int last_len;
u32 clock_time(void) { return now; }
u8 UartTrySend(u8 *d, u8 n)
{
	if (busy) return 0;
	if (n >= sizeof(line)) return 0;
	memcpy(line, d, n);
	line[n] = 0;
	last_len = n;
	return 1;
}

int main(void)
{
	int tgt[3] = {10, -20, 30};
	int gyro[3] = {-1, 2, -3};
	int corr[3] = {4, -5, 6};
	unsigned short pwm[4] = {100, 101, 102, 103};
	short acc[3] = {10, 20, 30};
	int q[4] = {30000, 100, -200, 50};

	now = 0;
	GroundDiag_Init();
	GroundDiag_OnControl(190, 77, tgt, gyro, corr, pwm, acc, q, 1, 1, 1);
	busy = 1;
	now = 2400000;
	GroundDiag_Process();          // 忙时只丢帧，不阻塞
	busy = 0;
	now = 4800000;
	GroundDiag_Process();
	if (!strstr(line, "YAW t=") || !strstr(line, " th=190")
		|| !strstr(line, " y=77") || !strstr(line, "tr=10,-20,30")
		|| !strstr(line, "gy=-1,2,-3") || !strstr(line, "co=4,-5,6")
		|| !strstr(line, "p=100,101,102,103")
		|| !strstr(line, "q=30000,100,-200,50")
		|| !strstr(line, "j=0,0,0") || !strstr(line, "a=1")
		|| !strstr(line, "l=1")) {
		printf("FAIL ground diag line: %s\n", line);
		return 1;
	}
	/* 宽值必须被截断到 160 字节内，且仍以换行结束。 */
	{
		int n;
		for (n = 0; n < 4; n++) q[n] = 2147483647;
		for (n = 0; n < 3; n++) acc[n] = (n == 0 ? -32768 : 32767);
		pwm[0] = pwm[1] = pwm[2] = pwm[3] = 65535;
		GroundDiag_OnControl(65535, 32767, tgt, gyro, corr, pwm, acc, q, 1, 1, 1);
		now += 2400000;
		GroundDiag_Process();
		if (last_len > 160 || last_len == 0 || line[last_len - 1] != '\n') {
			printf("FAIL wide UART line len=%d\n", last_len);
			return 1;
		}
	}
	printf("PASS ground diag 10Hz nonblocking: %s", line);
	printf("RESULT OK\n");
	return 0;
}
