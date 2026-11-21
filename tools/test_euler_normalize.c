/*
 * 四元数归一化/异常步长回归（主机侧编译 UserEuler.c，不依赖 TC32 硬件）
 *
 * 覆盖：
 *   1. 正常 400Hz（dt=2.5ms）连续更新；
 *   2. 90ms 长间隔：必须返回失败，四元数保持不变；
 *   3. 单步旋转 >1rad：必须返回失败；
 *   4. 归一化前的损坏四元数：必须返回失败且不改 q；
 *   5. 异常后下一拍 2.5ms 能恢复。
 *
 * 编译（在 tools 目录）：
 *   gcc -std=c99 -O2 -I..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car ^
 *       test_euler_normalize.c ^
 *       ..\telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\UserEuler.c ^
 *       -o test_euler_normalize.exe
 */

#include <stdio.h>
#include <string.h>

#include "UserEuler.h"

static int q_norm_ok(const euler_q_t *m)
{
	long long n2 = 0;
	long long target = (long long)ATT_Q15_ONE * ATT_Q15_ONE;
	int i;

	for (i = 0; i < 4; i++) {
		n2 += (long long)m->q[i] * m->q[i];
	}
	return n2 > target * 95 / 100 && n2 < target * 105 / 100;
}

static int q_equal(const euler_q_t *a, const att_i32 q[4])
{
	return a->q[0] == q[0] && a->q[1] == q[1]
		&& a->q[2] == q[2] && a->q[3] == q[3];
}

int main(void)
{
	euler_q_t m;
	att_i32 acc[3] = {0, 0, -1000};
	att_i32 gnormal[3] = {100, 0, 0};
	att_i32 ghigh[3] = {35700, 0, 0};
	att_i32 q_before[4];
	int i;
	int failed = 0;

	Euler_Reset(&m);
	m.ready = 1;

	/* 1) 正常 2.5ms：连续 400 拍应全部成功，模长保持 */
	for (i = 0; i < 400; i++) {
		if (!Euler_Update(&m, acc, gnormal, 25, (att_u32)(i * 3))) {
			printf("FAIL normal step %d returned 0\n", i);
			failed = 1;
			break;
		}
	}
	if (!failed && !q_norm_ok(&m)) {
		printf("FAIL normal q norm out of range\n");
		failed = 1;
	}
	if (!failed) {
		printf("PASS normal 2.5ms x400, q=%d/%d/%d/%d\n",
			   m.q[0], m.q[1], m.q[2], m.q[3]);
	}

	/* 2) 90ms 长间隔：必须失败，且 q 保持上一拍有效值 */
	memcpy(q_before, m.q, sizeof(q_before));
	if (Euler_Update(&m, acc, ghigh, 900, 2000) != 0) {
		printf("FAIL 90ms update should fail\n");
		failed = 1;
	} else if (!q_equal(&m, q_before)) {
		printf("FAIL 90ms update changed q\n");
		failed = 1;
	} else {
		printf("PASS 90ms rejected, q unchanged, have_prev=%u\n",
			   (unsigned)m.have_prev);
	}

	/* 3) 异常后下一拍 2.5ms 能恢复 */
	if (!failed && !Euler_Update(&m, acc, gnormal, 25, 2010)) {
		printf("FAIL recovery after 90ms returned 0\n");
		failed = 1;
	} else if (!failed && !q_norm_ok(&m)) {
		printf("FAIL recovery q norm out of range\n");
		failed = 1;
	} else if (!failed) {
		printf("PASS recovery after 90ms\n");
	}

	/* 4) 50ms 内单步旋转 1rad 以上：必须失败，q 不变。
	 * 用独立的估计器且没有上一拍角速度，避免梯形平均把 2000dps 拉低。 */
	{
		euler_q_t m2;

		Euler_Reset(&m2);
		m2.ready = 1;
		memcpy(q_before, m2.q, sizeof(q_before));
		if (Euler_Update(&m2, acc, ghigh, 500, 2100) != 0) {
			printf("FAIL large single step should fail\n");
			failed = 1;
		} else if (!q_equal(&m2, q_before)) {
			printf("FAIL large single step changed q\n");
			failed = 1;
		} else {
			printf("PASS large single step rejected, q unchanged\n");
		}
	}

	/* 5) 归一化前已损坏的四元数：必须失败且不改 q */
	m.q[0] = 100000;
	m.q[1] = 0;
	m.q[2] = 0;
	m.q[3] = 0;
	memcpy(q_before, m.q, sizeof(q_before));
	if (Euler_Update(&m, acc, gnormal, 25, 2200) != 0) {
		printf("FAIL corrupt q should fail\n");
		failed = 1;
	} else if (!q_equal(&m, q_before)) {
		printf("FAIL corrupt q was modified\n");
		failed = 1;
	} else {
		printf("PASS corrupt q rejected, q unchanged\n");
	}

	return failed ? 1 : 0;
}
