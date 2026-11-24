/* 真实 PressSen 驱动 + 模拟 I2C：MEAS_CFG bit4/bit5 分别表示压力/温度就绪。 */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "PressSen_Lpc2623.h"

static u8 regs[256];
static u8 fail_read;

u8 i2c_read_reg_s2(u8 reg, u8 *value)
{
	if (fail_read) {
		return 1;
	}
	*value = regs[reg];
	return 0;
}

static int expect(u8 cfg, u8 fail, u8 p_want, u8 t_want, const char *what)
{
	int bad = 0;
	regs[REG_MEAS_CFG] = cfg;
	fail_read = fail;
	if (LPC2623_IsPressureReady() != p_want) {
		printf("FAIL %s pressure cfg=0x%02X\n", what, cfg);
		bad = 1;
	}
	if (LPC2623_IsTempReady() != t_want) {
		printf("FAIL %s temperature cfg=0x%02X\n", what, cfg);
		bad = 1;
	}
	return bad;
}

int main(void)
{
	memset(regs, 0, sizeof(regs));
	if (expect(0x00, 0, 0, 0, "both clear")) return 1;
	if (expect(0x10, 0, 1, 0, "pressure only")) return 1;
	if (expect(0x20, 0, 0, 1, "temperature only")) return 1;
	if (expect(0x30, 0, 1, 1, "both ready")) return 1;
	if (expect(0x30, 1, 0, 0, "i2c failure")) return 1;
	printf("PASS LPC2623 MEAS_CFG bit4 pressure / bit5 temperature, real driver\n");
	printf("RESULT OK\n");
	return 0;
}
