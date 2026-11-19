/***************************************************************************************
*	File: 	my_t3488.h
*
*	Corp: 	xxxxxx
*	Author:	C.J.LEI (JinShi. HuNan. China)
*	phone:	13923778021.
*
*	Date:	2026-08-07.
*
****************************************************************************************/

#ifndef __MY_T3488_H__
#define __MY_T3488_H__


// =====================================================================================

// --- 寄存器地址定义 ---

// DREG_BANK1 寄存器
#define T3488_REG_FIFO_COUNT        0x00
#define T3488_REG_FIFO_DATA         0x01
#define T3488_REG_SREG_CONFIG       0x02
#define T3488_REG_SREG_DATA         0x03
#define T3488_REG_FIFO_CONFIG1      0x06
#define T3488_REG_FIFO_CONFIG2      0x07
#define T3488_REG_FSD_CONFIG        0x08
#define T3488_REG_ACCEL_CONFIG0     0x0C
#define T3488_REG_GYRO_CONFIG0      0x0D
#define T3488_REG_ODR_DECIMATE      0x0E
#define T3488_REG_PWR_MGMT0         0x0F
#define T3488_REG_PWR_MGMT1         0x10
#define T3488_REG_INT1_CONFIG0      0x12
#define T3488_REG_INT1_CONFIG1      0x13
#define T3488_REG_INT2_CONFIG0      0x14
#define T3488_REG_INT2_CONFIG1      0x15
#define T3488_REG_INT1_STATUS       0x19
#define T3488_REG_INT2_STATUS       0x1C
#define T3488_REG_INT_PAD_CONFIG    0x25
#define T3488_REG_EDMP_APEX_CONFIG  0x2A
#define T3488_REG_EDMP_CFG_MISC     0x2B
#define T3488_REG_EDMP_APEX_STATUS0 0x2E
#define T3488_REG_EDMP_APEX_STATUS1 0x2F
#define T3488_REG_WHOAMI            0x72
#define T3488_REG_IREG_ADDR_15_8    0x7C
#define T3488_REG_IREG_ADDR_7_0     0x7D
#define T3488_REG_IREG_DATA         0x7E
#define T3488_REG_REG_MISC2         0x7F

// IPREG_TOP1 (间接访问, 基地址 0xAA200)
#define T3488_IPREG_FIFO_CONFIG     0x17

// IPREG_SYS1 (间接访问, 基地址 0xAA400)
#define T3488_IPREG_SYS1_GYRO_LPF   0x09

// IPREG_SYS2 (间接访问, 基地址 0xAA500)
#define T3488_IPREG_SYS2_ACCEL_LPF  0x06
#define T3488_IPREG_SYS2_ACCEL_AVG  0x08

// --- 常量 ---
#define T3488_WHOAMI_ID             0x86
#define T3488_SREG_CFG_ALL_EN       0x7F

// PIN1: AP_SDO\AP_AD0,
// AD0=GND: 0x68, AD0=VDD: 0x69,
//#define T3488_DEVICE_ADDR         	0x68


// -------------------------------------------------------------------------------------
// 关于满量程：
//
//		灵敏度 = 32768/满量程，例如：
//
//		角速度满量程±2000，那么灵敏度 = ±32768/2000 = ±16.384，
//		角速度满量程±1000，那么灵敏度 = ±32768/1000 = ±32.768,
//		角速度满量程 ±500，那么灵敏度 = ±32768/ 500 = ±65.536,
//		角速度满量程 ±250，那么灵敏度 = ±32768/ 250 = ±131.072，
//
//		加速度满量程±16g,	那么灵敏度 = ±32768/16 = ±2048,
//		加速度满量程 ±8g,	那么灵敏度 = ±32768/ 8 = ±4096,
//		加速度满量程 ±4g,	那么灵敏度 = ±32768/ 4 = ±8192,
//		加速度满量程 ±2g,	那么灵敏度 = ±32768/ 2 = ±16384,
//
//
// -------------------------------------------------------------------------------------

/// T3488_REG_ACCEL_CONFIG0
/// [6:4]:
/// 1: 16g
/// 2: 8g,
/// 3: 4g,
/// 4: 2g,
/// [3:0]:
/// 5:	800Hz,
/// 6:	400Hz,
/// 7:	200Hz,
/// 8:	100Hz,
/// 9:	50Hz,
/// 10:	25Hz,
/// 11: 12.5Hz,
/// 12:	6.25Hz,
/// 13:	3.125Hz,
/// 14:	1.5625Hz,
/// 15: 0.78125Hz,

/// T3488_REG_GYRO_CONFIG0
/// [7:4]:
/// 1:	2000dps,
/// 2:	1000dps,
/// 3:	500dps,
/// 4:	250dps,
/// [3:0]:
/// 5:	800Hz,
/// 6:	400Hz,
/// 7:	200Hz,
/// 8:	100Hz,
/// 9:	50Hz,
/// 10:	25Hz,
/// 11: 12.5Hz,

/// T3488_IPREG_SYS1_GYRO_LPF
/// [7:3]: reserved,
/// [2:0]: GYRO_LPF_BW_SEL,
///	0:	ODR/2,
///	1:	ODR/4,
///	2:	ODR/8,
///	3:	ODR/16,
///	4:	ODR/32,
///	5:	ODR/64,
///	6:	ODR/128,
///	7:	ODR/128,

/// T3488_IPREG_SYS2_ACCEL_LPF
/// [7:3]: reserved,
/// [2:0]: ACCEL_LPF_BW_SEL,
///	0:	ODR/2,
///	1:	ODR/4,
///	2:	ODR/8,
///	3:	ODR/16,
///	4:	ODR/32,
///	5:	ODR/64,
///	6:	ODR/128,
///	7:	ODR/128,


// -------------------------------------------------------------------------------------

#define C_SamplingFreq_800Hz	5	// 800Hz, 1.25ms,
#define C_SamplingFreq_400Hz	6	// 400Hz, 2.5ms,
#define C_SamplingFreq_200Hz	7	// 200Hz, 5.0ms,
#define C_SamplingFreq_100Hz	8	// 100Hz, 10ms,

#define C_SamplingFreqSel	C_SamplingFreq_100Hz


void t3488_gyro_init(void);
void t3488_gyro_read_data(void);
void t3488_gyro_read_all(int16_t *accel, int16_t *gyro);

uint8_t inv_imu_read_reg(uint16_t reg_addr, uint8_t *buf, int len);
uint8_t inv_imu_write_reg(uint16_t reg_addr, uint8_t *buf, int len);
void t3488_gyro_init2(void);

// 台架姿态估计用：带 I2C 状态的六轴读取 + 含片内陀螺低通的初始化
uint8_t t3488_read_6axis(int16_t *accel, int16_t *gyro);
uint8_t t3488_bench_init(uint8_t lpf_sel, uint8_t *lpf_readback);



// -------------------------------------------------------------------------------------
#endif // __MY_T3488_H__

