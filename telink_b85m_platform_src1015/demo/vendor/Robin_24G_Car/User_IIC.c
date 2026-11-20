//**************************************************************
/*
 * User_IIC.c
 *
 *  Created on: 2026年10月2日
 *      Author: HuangGuangjian
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
// 旧接口（iic_start / iic_write_byte ...）的位延时标定
//	只有旧 IIC_* 路径用 iic_delay()（volatile 空循环）；新路径 i2c_*_s1/s2
//	用的是文件后半段独立的 tnop 位时序，两边互不影响。
//	1 个 SCL 位 = 3 次 iic_delay() + 约 15 个时钟周期的 IO 操作；
//	1 次空循环(volatile 计数，约 10 个时钟周期/次) + 进出开销约 5 个周期。
//	故：位周期 ≈ 30 × 循环次数 + 30 (系统时钟周期数)
//	标定值（24MHz）：400kHz → 1，200kHz → 3，100kHz → 7
//	★ 装机后请用示波器测一下 SCL 实际频率，偏差大就改 IIC_SCL_FREQ_KHZ 或微调下式常数
//**************************************************************
#define IIC_BIT_CYCLES		(CLOCK_SYS_CLOCK_HZ / ((u32)IIC_SCL_FREQ_KHZ * 1000u))	// 1位 = 多少个系统时钟
#define IIC_DLY_LOOPS		(((IIC_BIT_CYCLES) > 45u) ? (((IIC_BIT_CYCLES) - 30u) / 30u) : 1u)

//**************************************************************
// 引脚方向控制（开漏：bit=1 释放=高；bit=0 拉低）
//	不用静态影子变量：直接对方向寄存器读-改-写，只动 SDA/SCL 两位。
//	影子值只要被别的代码改过同组引脚就会过期，写回去会把别人的方向覆盖掉。
//**************************************************************
#define IIC_DRIVE_LOW(msk)	do{ IIC_REG_OE = (unsigned char)(IIC_REG_OE & (unsigned char)(~(msk))); }while(0)
#define IIC_RELEASE(msk)	do{ IIC_REG_OE = (unsigned char)(IIC_REG_OE | (unsigned char)(msk)); }while(0)

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

	// 两脚进入"释放"态（总线空闲为高）：只置这两位，不碰同组其他引脚
	IIC_RELEASE(IIC_SDA_MSK | IIC_SCL_MSK);
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

// ---------------------------------------------------------------------------
// 新路径（i2c_*_s1/s2）的位时序：Fast-mode，目标 SCL 接近但不超过 400kHz
//
// 24MHz 下 1 个系统时钟 ≈ 41.7ns，I2C Fast-mode 要求：
//   tLOW ≥ 1.3us（≈32 拍）、tHIGH ≥ 0.6us（≈15 拍）、
//   tSU;DAT ≥ 100ns、tSU;STA / tSU;STO ≥ 0.6us、tBUF ≥ 1.3us。
//
// 延时不用循环、不碰内存：全部由内联的 tnop（_ASM_NOP_）拼成，
// -O2 反汇编里热路径只有固定条数的 nop，没有计数器栈读写；
// 每次改方向寄存器的读-改-写和循环分支另算约 5~8 拍，已计入下面的常数。
//
// 位周期估算（按 -O2 反汇编逐条数）：写位循环体 45 条指令 =
//   30 个低电平 nop + 12 个高电平 nop + 3 次方向寄存器读改写(约 6 拍/次)
//   + 循环分支；按访存 2 拍、其余 1 拍估 ≈ 70 拍 ≈ 2.9us → SCL ≈ 340kHz，
//   tLOW ≈ 1.7us、tHIGH ≈ 0.8us，满足 Fast-mode 下限且不超过 400kHz。
// ★ 这是纸面估算，最终以引脚波形为准：上板量 SCL，偏高就加大
//   NOPS_LOW/NOPS_HIGH，偏低就减小，别让位周期低于 60 拍（400kHz）。
// ---------------------------------------------------------------------------
#define IIC_NOPS_LOW        30      // SCL 低电平保持（含 SDA 数据建立）
#define IIC_NOPS_HIGH       12      // SCL 高电平保持
#define IIC_NOPS_SU         12      // START / STOP 的建立与保持时间
#define IIC_NOPS_BUF        32      // STOP 之后的总线空闲（tBUF）

static inline void iic_nop2(void)  { _ASM_NOP_; _ASM_NOP_; }
static inline void iic_nop4(void)  { iic_nop2(); iic_nop2(); }
static inline void iic_nop8(void)  { iic_nop4(); iic_nop4(); }
static inline void iic_nop16(void) { iic_nop8(); iic_nop8(); }

// SCL 低电平保持：16+8+4+2 = 30 个 nop
static inline void iic_low_phase(void)  { iic_nop16(); iic_nop8(); iic_nop4(); iic_nop2(); }
// SCL 高电平保持：8+4 = 12 个 nop
static inline void iic_high_phase(void) { iic_nop8(); iic_nop4(); }
// START/STOP 建立时间：8+4 = 12 个 nop
static inline void iic_su_phase(void)   { iic_nop8(); iic_nop4(); }
// STOP 后的总线空闲：16+16 = 32 个 nop
static inline void iic_buf_phase(void)  { iic_nop16(); iic_nop16(); }

// 读回 SDA 电平（只读一次输入寄存器）
static inline uint8_t iic_sda_level(void)
{
	return (IIC_REG_IN & IIC_SDA_MSK) ? 1 : 0;
}

// 写一位：进入时 SCL 为低；SDA 只写一次，然后 SCL 高→低
static inline void i2c_write_bit(uint8_t bit)
{
	if (bit) {
		IIC_RELEASE(IIC_SDA_MSK);
	} else {
		IIC_DRIVE_LOW(IIC_SDA_MSK);
	}
	iic_low_phase();                // tLOW + SDA 数据建立
	IIC_RELEASE(IIC_SCL_MSK);
	iic_high_phase();               // tHIGH
	IIC_DRIVE_LOW(IIC_SCL_MSK);
}

// 读一位：进入时 SCL 为低、SDA 已释放；采样点落在高电平中段
static inline uint8_t i2c_read_bit(void)
{
	uint8_t bit;

	IIC_RELEASE(IIC_SCL_MSK);
	iic_nop8();                     // 等上拉拉高 SCL，再采 SDA
	bit = iic_sda_level();
	iic_nop4();                     // 补足高电平时间
	IIC_DRIVE_LOW(IIC_SCL_MSK);
	iic_low_phase();
	return bit;
}

// 首次 START：从 SCL/SDA 都释放的空闲状态开始，SCL 高时只产生一次 SDA 下降沿
static void i2c_start_first(void)
{
	IIC_RELEASE(IIC_SDA_MSK);
	IIC_RELEASE(IIC_SCL_MSK);
	iic_buf_phase();                // 满足 STOP 到 START 的总线空闲
	IIC_DRIVE_LOW(IIC_SDA_MSK);     // SCL 高时 SDA 下降沿 = START
	iic_su_phase();                 // tHD;STA
	IIC_DRIVE_LOW(IIC_SCL_MSK);
	iic_low_phase();
}

// 重复 START：进入时 SCL 为低；先释放 SDA 留建立时间，再释放 SCL
static void i2c_start_rep(void)
{
	IIC_RELEASE(IIC_SDA_MSK);       // SCL 低时先放开 SDA
	iic_su_phase();                 // tSU;DAT
	IIC_RELEASE(IIC_SCL_MSK);
	iic_high_phase();               // tHIGH
	IIC_DRIVE_LOW(IIC_SDA_MSK);     // SCL 高时 SDA 下降沿 = 重复 START
	iic_su_phase();                 // tHD;STA
	IIC_DRIVE_LOW(IIC_SCL_MSK);
	iic_low_phase();
}

// STOP：SCL 低时确保 SDA 为低，释放 SCL 等建立时间后再释放 SDA
static void i2c_stop(void)
{
	IIC_DRIVE_LOW(IIC_SDA_MSK);
	iic_low_phase();
	IIC_RELEASE(IIC_SCL_MSK);
	iic_su_phase();                 // tSU;STO
	IIC_RELEASE(IIC_SDA_MSK);       // STOP：SCL 高时 SDA 上升沿
	iic_buf_phase();                // tBUF
}

// 写一个字节并读 ACK：返回 0=ACK，1=NACK
static uint8_t i2c_write_byte(uint8_t byte)
{
	u32 i;
	uint8_t ack;

	for (i = 0; i < 8; i++) {
		i2c_write_bit((uint8_t)(byte & 0x80));
		byte <<= 1;
	}

	/* 第 9 个时钟：只释放一次 SDA，让从机拉低 */
	IIC_RELEASE(IIC_SDA_MSK);
	iic_low_phase();
	IIC_RELEASE(IIC_SCL_MSK);
	iic_nop8();
	ack = iic_sda_level();
	iic_nop4();
	IIC_DRIVE_LOW(IIC_SCL_MSK);

	return ack;
}

// 读一个字节并发 ACK/NACK：nack=1 时 SDA 保持释放（NACK）
static uint8_t i2c_read_byte(uint8_t nack)
{
	u32 i;
	uint8_t byte = 0;

	IIC_RELEASE(IIC_SDA_MSK);       /* 整个读字节只释放一次 */

	for (i = 0; i < 8; i++) {
		byte <<= 1;
		byte |= i2c_read_bit();
	}

	if (!nack) {
		IIC_DRIVE_LOW(IIC_SDA_MSK); /* ACK：只拉低这一次 */
	}
	iic_low_phase();
	IIC_RELEASE(IIC_SCL_MSK);
	iic_high_phase();
	IIC_DRIVE_LOW(IIC_SCL_MSK);
	if (!nack) {
		IIC_RELEASE(IIC_SDA_MSK);   /* ACK 之后放开，交给 STOP 或下一字节 */
	}

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

    i2c_start_first();
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

    i2c_start_first();
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

    i2c_start_first();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start_rep();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x01);  /* 读地址 */
    if (ret) { i2c_stop(); return 1; }

    *value = i2c_read_byte(1);  /* NACK */
    i2c_stop();

    return 0;
}

uint8_t i2c_read_reg_s2(uint8_t reg, uint8_t *value)
{
    uint8_t ret;

    i2c_start_first();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);  /* 写地址 */
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start_rep();
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

    i2c_start_first();
    ret = i2c_write_byte((SlaveAddress1 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start_rep();
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

    i2c_start_first();
    ret = i2c_write_byte((SlaveAddress2 << 1) | 0x00);
    if (ret) { i2c_stop(); return 1; }
    ret = i2c_write_byte(reg);
    if (ret) { i2c_stop(); return 1; }

    i2c_start_rep();
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

    i2c_start_first();
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

    i2c_start_first();
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


