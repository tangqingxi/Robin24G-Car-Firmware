/* 真实 PressSen 驱动 + 模拟 I2C 寄存器：MEAS_CFG bit4=0x10 才就绪。 */
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

static int expect(u8 cfg, u8 fail, u8 want, const char *what)
{
	regs[REG_MEAS_CFG] = cfg;
	fail_read = fail;
	if (LPC2623_IsPressureReady() != want) {
		printf("FAIL %s cfg=0x%02X fail=%u\n", what, cfg, fail);
		return 1;
	}
	return 0;
}

int main(void)
{
	memset(regs, 0, sizeof(regs));
	if (expect(0x00, 0, 0, "bit4 clear")) return 1;
	if (expect(0x10, 0, 1, "bit4 set")) return 1;
	if (expect(0x20, 0, 0, "other bit only")) return 1;
	if (expect(0x10, 1, 0, "i2c failure")) return 1;
	printf("PASS LPC2623 MEAS_CFG bit4 ready, real driver + simulated I2C\n");
	printf("RESULT OK\n");
	return 0;
}
