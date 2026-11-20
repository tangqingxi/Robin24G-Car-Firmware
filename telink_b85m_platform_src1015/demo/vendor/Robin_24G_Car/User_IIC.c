//**************************************************************
/*
 * User_IIC.c
 *
 *  更新日期：2026年11月20日
 *      作者：Tangqingxi
 */
//**************************************************************
//**************************************************************
#include "types.h"
#include "User_IIC.h"
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserRxdFunc.h"
#include "register.h"		//GPIO 寄存器定义（reg_gpio_in / reg_gpio_oe / reg_gpio_out_clear）
#include "lib/include/sys.h"	//_ASM_NOP_

//**************************************************************
//**************************************************************
#define P_IIC_SCL	IIC_SCL_Pin
#define P_IIC_SDA	IIC_SDA_Pin

//#define //MyDelay1uS()	gpio_read(Testing_Pin)
//**************************************************************
// =====================================================================================
//**************************************************************
// 高速位操作层（2026-10-28 优化）
//------------------------------------------------------------------
// 原来的实现用 gpio_write()/gpio_read()/gpio_set_output_en() 等函数，
// 每次调用都要算寄存器地址 + 函数调用开销（约 20~30 个时钟周期），
// 8 位数据 + ACK 传一次要几百 uS，严重占用飞控时间。
//
// 优化办法：
//  ① 直接操作 GPIO 寄存器（地址在编译期就是常量），一次电平切换只需 1 次写操作；
//  ② 用"开漏"方式：输出寄存器恒为 0，
//		拉低 = 打开输出使能；释放(高) = 关闭输出使能，由外部上拉电阻拉高；
//		这样每个电平只需写 1 次方向寄存器(OE)，不必再写输出寄存器；
//  ③ 延时用可配置的短循环，SCL 频率默认 400kHz（原实现约 30~60kHz）。
//
// Telink GPIO 寄存器（chip/tc122x/drivers/register.h）：
//		reg_gpio_in(pin)		: 引脚电平
//		reg_gpio_oe(pin)		: 方向；bit=1 → 禁止输出（引脚被上拉拉高）
//		reg_gpio_out_clear(pin)	: 输出清 0（写 1 有效）
//**************************************************************
#if ((IIC_SDA_Pin & 0xFF00) != (IIC_SCL_Pin & 0xFF00))
#error "IIC_SDA_Pin and IIC_SCL_Pin must be in the same GPIO group!"
#endif

#define IIC_SDA_MSK			((unsigned char)(IIC_SDA_Pin & 0xFF))
#define IIC_SCL_MSK			((unsigned char)(IIC_SCL_Pin & 0xFF))

#define IIC_REG_IN			reg_gpio_in(IIC_SDA_Pin)
#define IIC_REG_OE			reg_gpio_oe(IIC_SDA_Pin)
#define IIC_REG_OUT_CLR		reg_gpio_out_clear(IIC_SDA_Pin)

//**************************************************************
// 时序标定（按 -O2 生成的汇编估算）
//	1 个 SCL 位 = 3 次 iic_delay() + 约 15 个时钟周期的 IO 操作；
//	1 次空循环(volatile 计数，约 10 个时钟周期/次) + 进出开销约 5 个周期。
//	故：位周期 ≈ 30 × 循环次数 + 30 (系统时钟周期数)
//	标定值（24MHz）：400kHz → 1，200kHz → 3，100kHz → 7
//	★ 装机后请用示波器测一下 SCL 实际频率，偏差大就改 IIC_SCL_FREQ_KHZ 或微调下式常数
//**************************************************************
#define IIC_BIT_CYCLES		(CLOCK_SYS_CLOCK_HZ / ((u32)IIC_SCL_FREQ_KHZ * 1000u))	// 1位 = 多少个系统时钟
#define IIC_DLY_LOOPS		(((IIC_BIT_CYCLES) > 45u) ? (((IIC_BIT_CYCLES) - 30u) / 30u) : 1u)

//**************************************************************
// 方向寄存器影子（bit=1: 释放/输入； bit=0: 拉低/输出）
//**************************************************************
static unsigned char u8IicOeShadow;

#define IIC_DRIVE_LOW(msk)	do{ u8IicOeShadow &= (unsigned char)(~(msk)); IIC_REG_OE = u8IicOeShadow; }while(0)
#define IIC_RELEASE(msk)	do{ u8IicOeShadow |= (unsigned char)(msk);     IIC_REG_OE = u8IicOeShadow; }while(0)

//**************************************************************
// IIC 位延时（volatile 计数：防止编译器把空循环展开/优化掉，保证时序稳定）
//**************************************************************
static inline void iic_delay(void)
{
	volatile unsigned char i;
	for(i = (unsigned char)IIC_DLY_LOOPS; i != 0; i--)
	{
		_ASM_NOP_;		// tc32: "tnop"（见 lib/include/sys.h）
	}
}

u8	g_u8iicRxByteTmp = 0;

void iic_pin_init(void)
{
	gpio_setup_up_down_resistor(P_IIC_SCL, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(P_IIC_SDA, PM_PIN_PULLUP_10K);

	/// SCL / SDA: 均设为 GPIO，输入常开（随时可读回总线电平）
	gpio_set_func(P_IIC_SCL, AS_GPIO);
	gpio_set_func(P_IIC_SDA, AS_GPIO);
	gpio_set_input_en(P_IIC_SCL, 1);
	gpio_set_input_en(P_IIC_SDA, 1);

	// 输出寄存器恒为 0（本驱动只拉低、不推高，高电平靠上拉电阻）
	gpio_set_output_en(P_IIC_SCL, 1);
	gpio_set_output_en(P_IIC_SDA, 1);
	gpio_write(P_IIC_SCL, 0);
	gpio_write(P_IIC_SDA, 0);

	// 记录方向寄存器初值，之后两脚进入"释放"态（总线空闲为高）
	u8IicOeShadow = IIC_REG_OE;
	u8IicOeShadow |= (unsigned char)(IIC_SDA_MSK | IIC_SCL_MSK);
	IIC_REG_OE = u8IicOeShadow;
}


// SCL / SDA 方向控制（bit=1 释放=高；bit=0 拉低）
void iic_scl_as_in(void)
{
	IIC_RELEASE(IIC_SCL_MSK);		// 释放 SCL（由上拉拉高）
}

void iic_scl_as_out(void)
{
	IIC_DRIVE_LOW(IIC_SCL_MSK);		// 拉低 SCL
}


void iic_sda_as_in(void)
{
	IIC_RELEASE(IIC_SDA_MSK);		// 释放 SDA
}

void iic_sda_as_out(void)
{
	IIC_DRIVE_LOW(IIC_SDA_MSK);		// 拉低 SDA
}


void iic_scl_set(u32 u32PinValue)
{
	if(u32PinValue) IIC_RELEASE(IIC_SCL_MSK);	// 1: 释放(高)
	else            IIC_DRIVE_LOW(IIC_SCL_MSK);	// 0: 拉低
}

void iic_sda_set(u32 u32PinValue)
{
	if(u32PinValue) IIC_RELEASE(IIC_SDA_MSK);	// 1: 释放(高)
	else            IIC_DRIVE_LOW(IIC_SDA_MSK);	// 0: 拉低
}

// ----------------------------------------------
//
// ----------------------------------------------
void iic_sda_output_bit(u8 u8data)
{
	if(u8data & 0x80)		//
	{
		iic_sda_set(1); 	// SDA=H,
	}
	else
	{
		iic_sda_set(0); 	// SDA=L,
	}
}

// ----------------------------------------------
//
// ----------------------------------------------
void iic_sda_input_bit(void)
{
	if( IIC_REG_IN & IIC_SDA_MSK )		// 直接读 GPIO 输入寄存器
	{
		g_u8iicRxByteTmp |=  0x01;
	}
	else
	{
		g_u8iicRxByteTmp &= ~0x01;
	}
}

// ----------------------------------------------
//
// ----------------------------------------------
void iic_start(void)
{
	iic_sda_as_out();	//
	//MyDelay1uS();			//

	iic_scl_set(1);		// SCL=H,
	iic_sda_set(1);		// SDA=H,
	//MyDelay1uS();			//
	iic_sda_set(0);		// SDA=L,
	//MyDelay1uS();			//
}

// ----------------------------------------------
//
// ----------------------------------------------
void iic_stop(void)
{
	iic_sda_as_out();	//
	//MyDelay1uS();

	iic_scl_set(0);		// SCL=L,
	iic_sda_set(0);		// SDA=L,
	//MyDelay1uS();			//
	iic_scl_set(1);		// SCL=H,
	//MyDelay1uS();			//
	iic_sda_set(1); 		// SDA=H,
	//MyDelay1uS();			//

	iic_sda_as_in();		//
}

// ----------------------------------------------
//
// ----------------------------------------------
void iic_write_ack(void)
{
	iic_sda_as_out();	//
	//MyDelay1uS();

	iic_scl_set(0);		// SCL=L,
	//MyDelay1uS();
	iic_sda_set(0);		// SDA=L,
	//MyDelay1uS();			//
	iic_scl_set(1);		// SCL=H,
	//MyDelay1uS();			//
	iic_scl_set(0); 		// SCL=L,
	//MyDelay1uS();			//
}

// -------------------------------------------------
//
// -------------------------------------------------
u8   iic_waiting_ack(void)
{
	u8 i;

	#if C_WaitingAckSel	// 0: for norma, 1:for at24c16,
	iic_sda_set(0);		// care: add it, by cjlei, 2019-12-10, !!!
	//MyDelay1uS();			//
	#endif
	iic_sda_as_in();		// 释放 SDA，等待从机拉低
	//MyDelay1uS();			//

	iic_scl_set(1);		//
	iic_delay();		// 给从机一点时间把 SDA 拉低
	iic_delay();
	for(i=0; i<200; i++)	// 等待 ACK（超时约几十 uS）
	{
		if( !(IIC_REG_IN & IIC_SDA_MSK) )
		{
			iic_sda_as_out();
			iic_scl_set(0);
			iic_delay();
			return(0);
		}
	}

	iic_sda_as_out();
	iic_scl_set(0);
	iic_delay();
	return(1);

}

// -------------------------------------------------------
//
// -------------------------------------------------------
void iic_write_byte(u8 u8data)
{
u8  i;
u8	dat;

	dat = u8data;		//

	iic_sda_as_out();	//

	for(i=0; i<8; i++)		//
	{
		iic_scl_set(0);		// SCL=L,
		iic_delay();		// 数据建立时间

		iic_sda_output_bit(dat);	// "(u8data << 1)"
		dat<<=1;
		iic_delay();

		iic_scl_set(1);		// SCL=H,
		iic_delay();		// SCL 高电平保持
	}

	iic_scl_set(0);			//
	iic_delay();
	iic_sda_set(1);			// 释放 SDA，准备接收 ACK
	iic_delay();

}

// -------------------------------------------------------
//
// -------------------------------------------------------
u8   iic_read_byte(void)
{
u8	i;

	iic_sda_as_in();		// set SDA as input,,,

	g_u8iicRxByteTmp = 0;	//

	for(i=0; i<8; i++)
	{
		iic_scl_set(0);		// SCL=L,
		iic_delay();
		iic_scl_set(1);		// SCL=H,
		iic_delay();		// 等数据稳定后采样

		g_u8iicRxByteTmp<<=1;	// care it position ,,,
		iic_sda_input_bit();	// read SDA=? ,,,
		iic_delay();
	}

	iic_scl_set(0);		//
	iic_delay();
	iic_sda_as_out();	// set SDA as output,,,

	return g_u8iicRxByteTmp;

}


// -----------------------------------------------------------------
//
// -----------------------------------------------------------------
void Single_WriteI2C(u8 Addr, u8 Data)
{
	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

    	iic_start();                 //起始信号

	iic_write_byte((SlaveAddress<<1)|0x00);  	//发送设备地址+写信号
    	iic_waiting_ack();			//add it, by cjlei,

    	iic_write_byte(Addr);    	//内部寄存器地址，
	iic_waiting_ack();				//add it, by cjlei,

	iic_write_byte(Data);       	//内部寄存器数据，
	iic_waiting_ack();				//add it, by cjlei,

	iic_stop();                   	//发送停止信号
}


u8   Single_ReadI2C(u8 Addr)
{
	u8 Data;

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();				//add it, by cjlei,

	iic_write_byte(Addr);     		//发送存储单元地址，从0开始
	iic_waiting_ack();				//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();				//add it, by cjlei,

	Data=iic_read_byte();       	//读出寄存器数据
	iic_write_ack();				//接收应答信号

	iic_stop();						//停止信号

	return Data;

}


void IIC_Read_nByte(u8 addr, u8 *p, u8 len)
{

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();				//add it, by cjlei,

	iic_write_byte(addr);     		//发送存储单元地址，从0开始
	iic_waiting_ack();				//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();				//add it, by cjlei,

	for(u8 i=0; i<len; i++)
	{
		p[i] = iic_read_byte();  	//

		if(i < (len-1))				// can't is "(len-1)" , else sda will always is low, error !
		{
			iic_write_ack();		//
		}
	}

	iic_stop();						//停止信号
	//WaitUs(50);						//
	WaitUs(5);						//

}


// -----------------------------------------------------------------
//
// -----------------------------------------------------------------
void Single_WriteI2C_2(u8 Addr, u8 Data)
{
	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

    iic_start();                 //起始信号

	iic_write_byte((SlaveAddress2<<1)|0x00);  	//发送设备地址+写信号
    iic_waiting_ack();			//add it, by cjlei,

    iic_write_byte(Addr);    	//内部寄存器地址，
	iic_waiting_ack();			//add it, by cjlei,

	iic_write_byte(Data);       //内部寄存器数据，
	iic_waiting_ack();			//add it, by cjlei,

	iic_stop();                 //发送停止信号
}


u8   Single_ReadI2C_2(u8 Addr)
{
	u8 Data;

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress2<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();				//add it, by cjlei,

	iic_write_byte(Addr);     		//发送存储单元地址，从0开始
	iic_waiting_ack();				//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress2<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();				//add it, by cjlei,

	Data=iic_read_byte();       	//读出寄存器数据
	iic_write_ack();				//接收应答信号

	iic_stop();						//停止信号

	return Data;

}


void IIC_Read_nByte_2(u8 addr, u8 *p, u8 len)
{

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress2<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();				//add it, by cjlei,

	iic_write_byte(addr);     		//发送存储单元地址，从0开始
	iic_waiting_ack();				//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((SlaveAddress2<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();				//add it, by cjlei,

	for(u8 i=0; i<len; i++)
	{
		p[i] = iic_read_byte();  	//

		if(i < (len-1))				// can't is "(len-1)" , else sda will always is low, error !
		{
			iic_write_ack();		//
		}
	}

	iic_stop();						//停止信号
	//WaitUs(50);						//
	WaitUs(5);						//

}


// -----------------------------------------------------------------
//
// -----------------------------------------------------------------
void Single_WriteI2C_S(u8 device, u8 Addr, u8 Data)
{
	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

    iic_start();                 //起始信号

	iic_write_byte((device<<1)|0x00);  	//发送设备地址+写信号
    iic_waiting_ack();					//add it, by cjlei,

    iic_write_byte(Addr);    			//内部寄存器地址，
	iic_waiting_ack();					//add it, by cjlei,

	iic_write_byte(Data);       		//内部寄存器数据，
	iic_waiting_ack();					//add it, by cjlei,

	iic_stop();					//发送停止信号
}


u8   Single_ReadI2C_S(u8 device, u8 Addr)
{
	u8 Data;

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((device<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();					//add it, by cjlei,

	iic_write_byte(Addr);     			//发送存储单元地址，从0开始
	iic_waiting_ack();					//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((device<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();					//add it, by cjlei,

	Data=iic_read_byte();       		//读出寄存器数据
	iic_write_ack();					//接收应答信号

	iic_stop();						//停止信号

	return Data;

}


void IIC_Read_nByte_S(u8 device, u8 addr, u8 *p, u8 len)
{

	iic_sda_as_out();			//add it, by cjlei,
	//MyDelay1uS();					//

	iic_start();                   	//起始信号

	iic_write_byte((device<<1)|0x00);	//发送设备地址+写信号
	iic_waiting_ack();					//add it, by cjlei,

	iic_write_byte(addr);     			//发送存储单元地址，从0开始
	iic_waiting_ack();					//add it, by cjlei,

	iic_start();                   	//起始信号

	iic_write_byte((device<<1)|0x01);	//发送设备地址+读信号
	iic_waiting_ack();					//add it, by cjlei,

	for(u8 i=0; i<len; i++)
	{
		p[i] = iic_read_byte();  		//

		if(i < (len-1))					// can't is "(len-1)" , else sda will always is low, error !
		{
			iic_write_ack();			//
		}
	}

	iic_stop();						//停止信号
	//WaitUs(50);						//
	WaitUs(5);						//

}


/// i2c:
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief I2C延时（半周期 / 全周期）
 *		改为按 IIC_SCL_FREQ_KHZ 计算，不再固定 1uS
 */
static inline void i2c_delay_half(void)
{
    iic_delay();
}

static inline void i2c_delay_full(void)
{
    iic_delay();
    iic_delay();
}

/**
 * @brief SDA输出模式（拉低）
 */
static inline void sda_output_mode(void)
{
    IIC_DRIVE_LOW(IIC_SDA_MSK);
}

/**
 * @brief SDA输入模式（释放，由上拉拉高）
 */
static inline void sda_input_mode(void)
{
    IIC_RELEASE(IIC_SDA_MSK);
}

/**
 * @brief SCL高电平（释放）
 */
static inline void scl_high(void)
{
    IIC_RELEASE(IIC_SCL_MSK);
}

/**
 * @brief SCL低电平（拉低）
 */
static inline void scl_low(void)
{
    IIC_DRIVE_LOW(IIC_SCL_MSK);
}

/**
 * @brief SDA高电平（释放）
 */
static inline void sda_high(void)
{
    IIC_RELEASE(IIC_SDA_MSK);
}

/**
 * @brief SDA低电平（拉低）
 */
static inline void sda_low(void)
{
    IIC_DRIVE_LOW(IIC_SDA_MSK);
}

/**
 * @brief 读取SDA电平
 */
static inline uint8_t sda_read(void)
{
    return (IIC_REG_IN & IIC_SDA_MSK) ? 1 : 0;
}

/**
 * @brief I2C起始信号
 */
static void i2c_start(void)
{
    sda_output_mode();
    sda_high();
    scl_high();
    i2c_delay_half();
    sda_low();
    i2c_delay_half();
    scl_low();
}

/**
 * @brief I2C停止信号
 */
static void i2c_stop(void)
{
    sda_output_mode();
    sda_low();
    scl_high();
    i2c_delay_half();
    sda_high();
    i2c_delay_half();
}

/**
 * @brief I2C发送一个字节
 * @param byte 要发送的字节
 * @return 0:成功收到ACK, 1:收到NACK
 */
static uint8_t i2c_write_byte(uint8_t byte)
{
    uint8_t i, ack;

    sda_output_mode();

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sda_high();
        } else {
            sda_low();
        }
        byte <<= 1;
        i2c_delay_half();
        scl_high();
        i2c_delay_half();
        scl_low();
        i2c_delay_half();
    }

    /* 读取ACK */
    sda_input_mode();
    scl_high();
    i2c_delay_half();
    ack = sda_read();
    scl_low();
    i2c_delay_half();

    return ack;  /* 0=ACK, 1=NACK */
}

/**
 * @brief I2C读取一个字节
 * @param ack 是否发送ACK (0: ACK, 1: NACK)
 * @return 读取到的字节
 */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, byte = 0;

    sda_input_mode();

    for (i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high();
        i2c_delay_half();
        if (sda_read()) {
            byte |= 0x01;
        }
        scl_low();
        i2c_delay_half();
    }

    /* 发送ACK/NACK */
    sda_output_mode();
    if (ack) {
        sda_high();  /* NACK */
    } else {
        sda_low();   /* ACK */
    }
    scl_high();
    i2c_delay_half();
    scl_low();
    i2c_delay_half();

    return byte;
}

// ---------------------------------------------------------------------------
/**
 * @brief I2C写寄存器 (单字节)
 * @param reg 寄存器地址
 * @param value 写入值
 * @return 0:成功, 1:失败
 */
uint8_t i2c_write_reg_s1(uint8_t reg, uint8_t value)
{
    uint8_t ret;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(value);
    if (ret) { i2c_stop(); return 1; }
    i2c_stop();

    return 0;
}

uint8_t i2c_write_reg_s2(uint8_t reg, uint8_t value)
{
    uint8_t ret;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(value);
    if (ret) { i2c_stop(); return 1; }
    i2c_stop();

    return 0;
}


/**
 * @brief I2C读寄存器 (单字节)
 * @param reg 寄存器地址
 * @param value 读取值指针
 * @return 0:成功, 1:失败
 */
uint8_t i2c_read_reg_s1(uint8_t reg, uint8_t *value)
{
    uint8_t ret;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x01);  /* 读地址 */
    if (ret) { i2c_stop(); return 1; }

    *value = i2c_read_byte(1);  /* NACK */
    i2c_stop();

    return 0;
}

uint8_t i2c_read_reg_s2(uint8_t reg, uint8_t *value)
{
    uint8_t ret;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x01);  /* 读地址 */
    if (ret) { i2c_stop(); return 1; }

    *value = i2c_read_byte(1);  /* NACK */
    i2c_stop();

    return 0;
}


/**
 * @brief I2C连续读寄存器 (自动递增)
 * @param reg 起始寄存器地址
 * @param data 数据缓冲
 * @param len 读取长度
 * @return 0:成功, 1:失败
 */
uint8_t i2c_read_regs_s1(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret, i;

    if (len == 0) return 0;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x01);
    if (ret) { i2c_stop(); return 1; }

    for (i = 0; i < len; i++) {
        data[i] = i2c_read_byte((i == len - 1) ? 1 : 0);
    }

    i2c_stop();
    return 0;
}

uint8_t i2c_read_regs_s2(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret, i;

    if (len == 0) return 0;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x01);
    if (ret) { i2c_stop(); return 1; }

    for (i = 0; i < len; i++) {
        data[i] = i2c_read_byte((i == len - 1) ? 1 : 0);
    }

    i2c_stop();
    return 0;
}


/**
 * @brief I2C连续写寄存器 (自动递增)
 * @param reg 起始寄存器地址
 * @param data 数据缓冲
 * @param len 写入长度
 * @return 0:成功, 1:失败
 */
uint8_t i2c_write_regs_s1(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret, i;

    if (len == 0) return 0;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    for (i = 0; i < len; i++) {
        ret = i2c_write_byte(data[i]);
        if (ret) { i2c_stop(); return 1; }
    }

    i2c_stop();
    return 0;
}

uint8_t i2c_write_regs_s2(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret, i;

    if (len == 0) return 0;

    i2c_start();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    for (i = 0; i < len; i++) {
        ret = i2c_write_byte(data[i]);
        if (ret) { i2c_stop(); return 1; }
    }

    i2c_stop();
    return 0;
}



// -------------------------------------------------------------------------------------
// "User_IIC.c", ,,, ,,, ,,,end, ,,, ,,, ,,,


//**************************************************************
//**************************************************************


