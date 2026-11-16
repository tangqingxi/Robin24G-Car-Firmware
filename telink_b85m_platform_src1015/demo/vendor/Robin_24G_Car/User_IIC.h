//**************************************************************
/*
 * User_IIC.h
 *
 *  Created on: 2026年10月2日
 *      Author: HuangGuangjian
 */
//**************************************************************
//**************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USER_IIC_H_
#define VENDOR_ROBIN_24G_CAR_USER_IIC_H_
//**************************************************************
#include "types.h"
//**************************************************************
// ICM-42670
#define ICM42670_DEVICE_ADDR		0x68	// iic slaver address,
#define ICM42670_REG_WHOAMI			0x75	// 0x63, ,,,
#define ICM42670_REG_RESET			0x02	// 0x10, ,,,
#define ICM42670_REG_TEMP_H			0x09	// temperature,
#define ICM42670_REG_TEMP_L			0x0A	//
#define ICM42670_REG_ACCEL_X_H		0x0B	// accel,
#define ICM42670_REG_ACCEL_X_L		0x0C	//
#define ICM42670_REG_ACCEL_Y_H		0x0D	//
#define ICM42670_REG_ACCEL_Y_L		0x0E	//
#define ICM42670_REG_ACCEL_Z_H		0x0F	//
#define ICM42670_REG_ACCEL_Z_L		0x10	//
#define ICM42670_REG_GYRO_X_H		0x11	// gyro,
#define ICM42670_REG_GYRO_X_L		0x12	//
#define ICM42670_REG_GYRO_Y_H		0x13	//
#define ICM42670_REG_GYRO_Y_L		0x14	//
#define ICM42670_REG_GYRO_Z_H		0x15	//
#define ICM42670_REG_GYRO_Z_L		0x16	//
#define ICM42670_REG_PWR_MGMT0		0x1F	// GYRO_MODE\ACCEL_MODE, 0x0F,
#define ICM42670_REG_GYRO_CONFIG0	0x20	// GYRO_UI_FS_SEL\GYPR_ODR,
#define ICM42670_REG_ACCEL_CONFIG0	0x21	// ACCEL_UI_FS_SEL\ACCEL_ODR,
#define ICM42670_REG_GYRO_CONFIG1	0x23	// GYRO_UI_FILT_BW,
#define ICM42670_REG_ACCEL_CONFIG1	0x24	// ACCEL_UI_FILT_BW,
#define ICM42670_REG_INTF_CONFIG1	0x36	// bit[1:0]: clksel,


// MPU6500寄存器地址
#define MPU6500_DEVICE_ADDR			0xD0	// iic slaver address,
#define MPU6500_REG_WHOAMI       	0x75	// 0x68, ,,,
#define MPU6500_REG_PWR_MGMT_1   	0x6B	//
#define MPU6500_REG_SMPLRT_DIV   	0x19	//
#define MPU6500_REG_CONFIG       	0x1A	//
#define MPU6500_REG_GYRO_CONFIG  	0x1B	//
#define MPU6500_REG_ACCEL_CONFIG 	0x1C	//
#define MPU6500_REG_ACCEL_XOUT_H 	0x3B	// accel,
#define MPU6500_REG_ACCEL_XOUT_L 	0x3C	//
#define MPU6500_REG_ACCEL_YOUT_H 	0x3D	//
#define MPU6500_REG_ACCEL_YOUT_L 	0x3E	//
#define MPU6500_REG_ACCEL_ZOUT_H 	0x3F	//
#define MPU6500_REG_ACCEL_ZOUT_L 	0x40	//
#define MPU6500_REG_TEMP_H			0x41	// temperature,
#define MPU6500_REG_TEMP_L			0x42	// temperature,
#define MPU6500_REG_GYRO_XOUT_H  	0x43	// gyro,
#define MPU6500_REG_GYRO_XOUT_L  	0x44	//
#define MPU6500_REG_GYRO_YOUT_H  	0x45	//
#define MPU6500_REG_GYRO_YOUT_L  	0x46	//
#define MPU6500_REG_GYRO_ZOUT_H  	0x47	//
#define MPU6500_REG_GYRO_ZOUT_L  	0x48	//


// T3488:
// PIN1: AP_SDO\AP_AD0,
// AD0=GND: 0x68, AD0=VDD: 0x69,
#define T3488_DEVICE_ADDR         	0x68	// 0x68\0x69,


// LPC2623 I2C设备地址 (7位地址)
// SDO=GND:0x76, SDO=VDD:0x77
#define LPC2623_I2C_ADDR          	0x76	// 0x76\0x77,

#if 0
#define LPC_SDA_PIN      	GPIO_PA0		// P_IIC_SDA,
#define LPC_SCL_PIN      	GPIO_PA1		// P_IIC_SCL,
#else
#define LPC_SDA_PIN      	P_IIC_SDA
#define LPC_SCL_PIN      	P_IIC_SCL
#endif


// -------------------------------------------------------------------------------------
//


//**************************************************************
// IIC 通信速率设定（2026-10-28 优化）
//	原理图上 SDA/SCL 有 4.7K 外部上拉（R21/R22），走线很短，
//	400kHz 完全可行；若示波器观察到上升沿太缓/通信出错，改成 200 或 100。
//	可选值：100 / 200 / 400 / 600
//**************************************************************
#define IIC_SCL_FREQ_KHZ		400

//**************************************************************
#define C_WaitingAckSel		0	// 0:for norma, 1:for at24c16,

#define C_MPU6500			1
#define C_ICM42670			2
#define C_T3488				3

#define C_GA_SENSOR_SEL		C_T3488


#if (C_GA_SENSOR_SEL==C_MPU6500)
#define SlaveAddress		MPU6500_DEVICE_ADDR
#elif (C_GA_SENSOR_SEL==C_ICM42670)
#define SlaveAddress		ICM42670_DEVICE_ADDR
#elif (C_GA_SENSOR_SEL==C_T3488)
#define SlaveAddress		T3488_DEVICE_ADDR
#endif


#define SlaveAddress1		T3488_DEVICE_ADDR
#define SlaveAddress2		LPC2623_I2C_ADDR


#if 0   // see file:"app_project_config.h", ,,,
#define P_IIC_SCL		GPIO_PA1	// OUT,
#define P_IIC_SDA		GPIO_PA0	// OUT\IN,
#endif


// -------------------------------------------------------------------------------------
// 1. old by cjlei,

void iic_pin_init(void);

void Single_WriteI2C(u8 Addr, u8 Data);
u8   Single_ReadI2C(u8 Addr);
void IIC_Read_nByte(u8 addr, u8 *p, u8 len);

void Single_WriteI2C_2(u8 Addr, u8 Data);
u8   Single_ReadI2C_2(u8 Addr);
void IIC_Read_nByte_2(u8 addr, u8 *p, u8 len);


void Single_WriteI2C_S(u8 device, u8 Addr, u8 Data);
u8   Single_ReadI2C_S(u8 device, u8 Addr);
void IIC_Read_nByte_S(u8 device, u8 addr, u8 *p, u8 len);


// -------------------------------------------------------------------------------------
// 2. new by deepseek,

/// SlaveAddress1: T3488,
uint8_t i2c_read_reg_s1(uint8_t reg, uint8_t *value);
uint8_t i2c_write_reg_s1(uint8_t reg, uint8_t value);
uint8_t i2c_read_regs_s1(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t i2c_write_regs_s1(uint8_t reg, uint8_t *data, uint8_t len);


/// SlaveAddress2: LPC2623,
uint8_t i2c_read_reg_s2(uint8_t reg, uint8_t *value);
uint8_t i2c_write_reg_s2(uint8_t reg, uint8_t value);
uint8_t i2c_read_regs_s2(uint8_t reg, uint8_t *data, uint8_t len);
uint8_t i2c_write_regs_s2(uint8_t reg, uint8_t *data, uint8_t len);




//**************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USER_IIC_H_ */
//**************************************************************

