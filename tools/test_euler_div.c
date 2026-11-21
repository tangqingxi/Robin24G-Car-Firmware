/*
 * Euler_Update 积分除余优化验证（主机侧）
 *
 * 同一组正负角速度/dt 输入分别用：
 *   C_EULER_DIV_OPT=0：dq=num/10000; rem=num%10000;
 *   C_EULER_DIV_OPT=1：dq=num/10000; rem=num-dq*10000;
 * 编译运行，逐点比较 q/dq_rem/返回值，并检查异常步长保护。
 *
 * 编译（在 tools 目录，分别生成两个可执行文件）：
 *   gcc -std=c99 -O2 -DC_EULER_DIV_OPT=0 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
 *       test_euler_div.c ..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\UserEuler.c ^
 *       -o ..\data\test_euler_div_base.exe
 *   gcc -std=c99 -O2 -DC_EULER_DIV_OPT=1 ... -o ..\data\test_euler_div_opt.exe
 *   fc /b base.out opt.out
 */

#include <stdio.h>
#include <string.h>
#include <limits.h>

#include "UserEuler.h"

static int failures;

static void fail(const char *msg)
{
	printf("FAIL %s\n", msg);
	failures++;
}

/* 单独验证有符号整数除余公式，覆盖正负、0 和 int32 边界附近 */
static void test_div_formula(void)
{
	static const att_i32 edge[] = {
		INT_MIN, INT_MIN + 1, INT_MIN + 12345, -1000000000, -10000,
		-9999, -1, 0, 1, 9999, 10000, 10001,
		1000000000, INT_MAX - 12345, INT_MAX - 1, INT_MAX
	};
	int i;

	for (i = 0; i < (int)(sizeof(edge) / sizeof(edge[0])); i++) {
		att_i32 num = edge[i];
		att_i32 dq = num / 10000;
		att_i32 old_rem = num % 10000;
		att_i32 new_rem = num - dq * 10000;

		if (old_rem != new_rem) {
			printf("FAIL formula num=%d old=%d new=%d\n",
				   (int)num, (int)old_rem, (int)new_rem);
			failures++;
		}
	}

	/* 再扫一段跨越多个 10000 整数倍的连续点，含正负方向 */
	{
		att_i32 num;

		for (num = -35000; num <= 35000; num += 137) {
			att_i32 dq = num / 10000;
			att_i32 old_rem = num % 10000;
			att_i32 new_rem = num - dq * 10000;

			if (old_rem != new_rem) {
				printf("FAIL scan num=%d old=%d new=%d\n",
					   (int)num, (int)old_rem, (int)new_rem);
				failures++;
				break;
			}
		}
	}
}

static void print_state(const char *tag, const euler_q_t *m, int ret)
{
	printf("%s ret=%d q=%d/%d/%d/%d rem=%d/%d/%d/%d\n",
		   tag, ret,
		   (int)m->q[0], (int)m->q[1], (int)m->q[2], (int)m->q[3],
		   (int)m->dq_rem[0], (int)m->dq_rem[1],
		   (int)m->dq_rem[2], (int)m->dq_rem[3]);
}

static void test_update_sequence(void)
{
	euler_q_t m;
	att_i32 acc[3] = {0, 0, 1000};
	static const att_i32 gyro[][3] = {
		{  137,  -211,    53},
		{ -523,   307,  -941},
		{ 1024, -2048,   512},
		{ -777,   888,  -999},
		{    1,    -1,     1},
		{ 4096,  2048, -1024},
		{ -300,   600,  -900},
		{  250,  -125,    75},
		{ -999, -1000,  1001},
		{  333,   444,  -555}
	};
	int i;

	memset(&m, 0, sizeof(m));
	m.ready = 1;
	m.q[0] = 32767;
	m.q[1] = 0;
	m.q[2] = 0;
	m.q[3] = 0;

	for (i = 0; i < (int)(sizeof(gyro) / sizeof(gyro[0])); i++) {
		int ret = Euler_Update(&m, acc, gyro[i], 50, (att_u32)(i * 5));
		char tag[32];

		sprintf(tag, "step%02d g=%d/%d/%d",
				i, (int)gyro[i][0], (int)gyro[i][1], (int)gyro[i][2]);
		print_state(tag, &m, ret);
	}

	/* 异常大步长：90ms，应拒绝且四元数不变 */
	{
		att_i32 q0 = m.q[0], q1 = m.q[1], q2 = m.q[2], q3 = m.q[3];
		int ret = Euler_Update(&m, acc, gyro[0], 900, 1000);

		print_state("long_dt_90ms", &m, ret);
		if (ret != 0) {
			fail("90ms should be rejected");
		}
		if (m.q[0] != q0 || m.q[1] != q1 || m.q[2] != q2 || m.q[3] != q3) {
			fail("90ms changed q");
		}
	}

	/* 50ms 内单步超 1rad：应拒绝且四元数不变 */
	{
		att_i32 big[3] = {30000, 0, 0};
		att_i32 q0 = m.q[0], q1 = m.q[1], q2 = m.q[2], q3 = m.q[3];
		int ret = Euler_Update(&m, acc, big, 500, 1005);

		print_state("step_over_1rad", &m, ret);
		if (ret != 0) {
			fail("over-1rad step should be rejected");
		}
		if (m.q[0] != q0 || m.q[1] != q1 || m.q[2] != q2 || m.q[3] != q3) {
			fail("over-1rad changed q");
		}
	}

	/* 拒绝后恢复一拍正常更新 */
	{
		int ret = Euler_Update(&m, acc, gyro[1], 50, 1010);

		print_state("recover", &m, ret);
		if (ret != 1) {
			fail("recovery update should succeed");
		}
	}
}

int main(void)
{
	test_div_formula();
	test_update_sequence();
	if (failures) {
		printf("RESULT FAIL %d\n", failures);
		return 1;
	}
	printf("RESULT OK\n");
	return 0;
}
