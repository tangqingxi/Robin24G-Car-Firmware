/***************************************************************************************
* 文件名: UserFilter.c
****************************************************************************************/
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
//#include "UserRxdFunc.h"
#include "my_t3488.h"
#include "User_IIC.h"
#include "UserFilter.h"
#include "UserEuler.h"

// 本轮台架固件不使用 5 点中值滤波（旧控制链），整个文件排除出构建；
// 需要恢复旧控制链时把 UserEuler.h 里的 C_ATT_BENCH_EN 改成 0。
#if (!C_ATT_BENCH_EN)

// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------
// 姿态数据结构体
typedef struct {
    float roll;                 	// 横滚角(度)
    float pitch;                	// 俯仰角(度)
    float yaw;                  	// 偏航角(度)
    float gyro_x;               	// X轴角速度
    float gyro_y;               	// Y轴角速度
    float gyro_z;               	// Z轴角速度
} Attitude_TypeDef;

/*************************** 全局变量 *******************************/
// 姿态数据
Attitude_TypeDef attitude;

/*************************** 中值滤波 *******************************/
//********************************************************************
#if C_FILTER_5_EN
//********************************************************************

#if C_FILTER_5_A_EN
//int16_t a_accelx_buf[5] = {0x00};
//int16_t a_accely_buf[5] = {0x00};
//int16_t a_accelz_buf[5] = {0x00};
//保存为二进制原始格式
uint16_t a_accelx_buf[5] = {0x00};
uint16_t a_accely_buf[5] = {0x00};
uint16_t a_accelz_buf[5] = {0x00};
#endif

#if C_FILTER_5_G_EN
//int16_t a_gyrox_buf[5] = {0x00};
//int16_t a_gyroy_buf[5] = {0x00};
//int16_t a_gyroz_buf[5] = {0x00};
//保存为二进制原始格式
uint16_t a_gyrox_buf[5] = {0x00};
uint16_t a_gyroy_buf[5] = {0x00};
uint16_t a_gyroz_buf[5] = {0x00};
#endif

uint8_t a_filter_cnt;

void t3488_data_filter_init(void)
{
	#if C_FILTER_5_A_EN
	memset(a_accelx_buf, 0, 2*5);
	memset(a_accely_buf, 0, 2*5);
	memset(a_accelz_buf, 0, 2*5);
	#endif

	#if C_FILTER_5_G_EN
	memset(a_gyrox_buf, 0, 2*5);
	memset(a_gyroy_buf, 0, 2*5);
	memset(a_gyroz_buf, 0, 2*5);
	#endif

	a_filter_cnt = 0;
}

void t3488_data_filter_handler(int16_t *accel, int16_t *gyro)
{
	int16_t accel3[3], gyro3[3];

	t3488_gyro_read_all(accel3, gyro3);

	//GJ2026/10/2
	PushRfDatToFIFO(a_accelx_buf, (unsigned short)accel3[0]);
	PushRfDatToFIFO(a_accely_buf, (unsigned short)accel3[1]);
	PushRfDatToFIFO(a_accelz_buf, (unsigned short)accel3[2]);

	PushRfDatToFIFO(a_gyrox_buf, (unsigned short)gyro3[0]);
	PushRfDatToFIFO(a_gyroy_buf, (unsigned short)gyro3[1]);
	PushRfDatToFIFO(a_gyroz_buf, (unsigned short)gyro3[2]);

	accel[0] = SortingFiFoData((int16_t *)a_accelx_buf);
	accel[1] = SortingFiFoData((int16_t *)a_accely_buf);
	accel[2] = SortingFiFoData((int16_t *)a_accelz_buf);
	gyro[0]	= SortingFiFoData((int16_t *)a_gyrox_buf);
	gyro[1]	= SortingFiFoData((int16_t *)a_gyroy_buf);
	gyro[2]	= SortingFiFoData((int16_t *)a_gyroz_buf);

}

void t3488_data_fifo_Prepare(void)		//将数据压满FIFO
{
	unsigned short i;
    for (i = 0; i < 5; i++) {
        t3488_data_filter_handler(accel, gyro);
        sleep_us(1250);			//800Hz 采集时间
    }
}

//********************************************************************
#endif	// C_FILTER_5_EN, ,,,END, ,,,
//********************************************************************





//********************************************************************
// file:"UserFilter.c", ,,,end, ,,,
//********************************************************************

#endif  // (!C_ATT_BENCH_EN)
