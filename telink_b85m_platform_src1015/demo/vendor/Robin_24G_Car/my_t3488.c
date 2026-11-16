/***************************************************************************************
*	File: 	my_t3488.c
*
*	Corp: 	xxxxxx
*	Author:	C.J.LEI (JinShi. HuNan. China)
****************************************************************************************/
#include "User_IIC.h"
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserRxdFunc.h"
#include "my_t3488.h"


// =====================================================================================

void t3488_gyro_init(void)
{
	#if 0
    u8 gyro_ID=0;
     while(gyro_ID!=0x86)
     {
		gyro_ID = Single_ReadI2C(0x72);
		WaitMs(90);
        //wdt_clear();
        //printf("gyro_ID %x \n",gyro_ID);
     }
	#else
	 uint8_t i, whoami;
	 for(i=0;i<100;i++){
		 //whoami = Single_ReadI2C(T3488_REG_WHOAMI);
		 i2c_read_reg_s1(T3488_REG_WHOAMI, &whoami);
		 if(whoami == 0x86){
			 break;
		 }
		 WaitMs(10); // 200*90ms=18000ms, ,,,
	 }
	#endif

     WaitMs(90);
     i2c_write_reg_s1(0x7F,0x04);  // Soft,
     WaitMs(90);
     i2c_write_reg_s1(0x0C,0x15);  // 15:16g/800Hz, 25:8G/800Hz,
     WaitMs(90);
     i2c_write_reg_s1(0x0D,0x15);  // 15:2000dps/800Hz,
     WaitMs(90);
     i2c_write_reg_s1(0x0F,0x0F);  // Low  noise,
     WaitMs(90);

}

uint8_t rawADC[12] = {0};
int16_t accx, accy, accz;
int16_t gyrox, gyroy, gyroz;
void t3488_gyro_read_data(void)
{
//	u8 data_b=0,Data_flag=0;
    i2c_read_regs_s1(0x03, rawADC,12);

    accx  =((int16_t)(rawADC[1] << 8)|rawADC[0]);
    accy  =((int16_t)(rawADC[3] << 8)|rawADC[2]);
    accz  = ((int16_t)(rawADC[5] << 8)|rawADC[4]);

    gyrox =((int16_t)(rawADC[7] << 8)|rawADC[6]);
    gyroy = ((int16_t)(rawADC[9] << 8)|rawADC[8]);
    gyroz = ((int16_t)(rawADC[11] << 8)|rawADC[10]);

    //printf(" %d %d %d %d %d %d\r\n",accx,accy,accz,gyrox,gyroy,gyroz);
}

void t3488_gyro_read_all(int16_t *accel, int16_t *gyro)
{
	uint8_t buf[12] ={0};

	i2c_read_regs_s1(0x03, buf, 12);
	accel[0] = (buf[1]<<8) | buf[0];
	accel[1] = (buf[3]<<8) | buf[2];
	accel[2] = (buf[5]<<8) | buf[4];

	gyro[0] = ( buf[7]<<8) | buf[6];
	gyro[1] = ( buf[9]<<8) | buf[8];
	gyro[2] = (buf[11]<<8) | buf[10];
}


// =====================================================================================
//
//
// -------------------------------------------------------------------------------------
uint8_t inv_imu_read_reg(uint16_t reg_addr, uint8_t *buf, int len)
{
	uint8_t data[2];
	/* Write address first */
	data[0] = (reg_addr & 0xFF00) >> 8;
	data[1] = reg_addr & 0xFF;

    ///spi_master_write_register(0x7c, 2, data);
	i2c_write_regs_s1(0x7c, data, 2);
	WaitUs(4);
	/* Read all bytes one by one */
	for (uint32_t i = 0; i < len; i++) {
		///spi_master_read_register(0x7e, 1, &buf[i]);
		i2c_read_regs_s1(0x7e, &buf[i], 1);
		WaitUs(4);
	}

	return 0;
}

uint8_t inv_imu_write_reg(uint16_t reg_addr, uint8_t *buf, int len)
{
	uint8_t data[3];
	/* First two bytes are the address where we want to write */
	data[0] = (reg_addr & 0xFF00) >> 8;
	data[1] = reg_addr & 0xFF;
	/* 3rd byte is the first data to write*/
	data[2] = buf[0];
    ///spi_master_write_register(0x7c, 3, data);
	i2c_write_regs_s1(0x7c, data, 3);

	 WaitUs(4);
	/* Read all bytes one by one */
	for (uint32_t i = 1; i < len; i++) {
		///spi_master_write_register(0x7e, 1, &buf[i]);
		i2c_write_regs_s1(0x7e, &buf[i], 1);
	}

	return 0;
}

#define C_T3488_TEST_EN3	0	// 0:normal, 1:testing,
#if C_T3488_TEST_EN3
u8 a_t_buff2[1] = {0x00};
u8 a_t_buff3[1] = {0x00};
#endif
void t3488_gyro_init2(void)
{
	uint8_t i, whoami;
	for(i=0;i<100;i++){
		//whoami = Single_ReadI2C(T3488_REG_WHOAMI);
		i2c_read_reg_s1(T3488_REG_WHOAMI, &whoami);
		if(whoami == 0x86){
			break;
		}
		WaitMs(10); // 200*90ms=18000ms, ,,,
	}

	uint16_t reg1;
	uint8_t buff1[1];

	// config accel 16G FSR, 100hz ODR,
	#if (C_SamplingFreqSel==C_SamplingFreq_100Hz)
	i2c_write_reg_s1(0x0C, 0x18);	// 18:16g,100Hz
	#elif (C_SamplingFreqSel==C_SamplingFreq_200Hz)
	i2c_write_reg_s1(0x0C, 0x17);	// 17:16g, 200Hz,
	#elif (C_SamplingFreqSel==C_SamplingFreq_400Hz)
	i2c_write_reg_s1(0x0C, 0x16);	// 16:16g, 400Hz,
	#elif (C_SamplingFreqSel==C_SamplingFreq_800Hz)
	i2c_write_reg_s1(0x0C, 0x15);	// 15:16g, 800Hz,
	#endif

	// config gyro 2000 FSR, 100hz ODR,
	#if (C_SamplingFreqSel==C_SamplingFreq_100Hz)
	i2c_write_reg_s1(0x0D, 0x18);	// 18:2000dps,100Hz
	#elif (C_SamplingFreqSel==C_SamplingFreq_200Hz)
	i2c_write_reg_s1(0x0D, 0x17);	// 17:2000dps,200Hz
	#elif (C_SamplingFreqSel==C_SamplingFreq_400Hz)
	i2c_write_reg_s1(0x0D, 0x16);	// 16:2000dps,400Hz
	#elif (C_SamplingFreqSel==C_SamplingFreq_800Hz)
	i2c_write_reg_s1(0x0D, 0x15);	// 15:2000dps,800Hz
	#endif

	// config INT1 POLARITY_HIGH MODE_PULSE,
	i2c_write_reg_s1(0x12, 0x03);

	// config INT1 UI data ready enable,
	i2c_write_reg_s1(0x13, 0x01);

	// config INT1 Push-pull,
	i2c_write_reg_s1(0x25, 0x00);

	// config Gyro DLPF bandwidth ODR/4,
	reg1 = 0xa409;
	buff1[0] = 0x01;
	inv_imu_write_reg(reg1, &buff1[0], 1);
	WaitMs(10);
	#if C_T3488_TEST_EN3
	inv_imu_read_reg(reg1, &a_t_buff2[0], 1);
	#endif

	// config accel DLPF Bandwidth ODR/4,
	reg1 = 0xa506;
	buff1[0] = 0x01;
	inv_imu_write_reg(reg1, &buff1[0], 1);
	WaitMs(10);
	#if C_T3488_TEST_EN3
	inv_imu_read_reg(reg1, &a_t_buff3[0], 1);
	#endif

	i2c_write_reg_s1(0x0f, 0x0f);

	WaitMs(100);

}


// -------------------------------------------------------------------------------------
// file:"my_t3488.c", ,,,

