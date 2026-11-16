//******************************************************************************
/*
 * user_IIC.c
 *
 *  Created on: 2026年10月10日
 *      Author: HuangGuangjian
 */
//******************************************************************************
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserTxdFunc.h"
//******************************************************************************
/* I2C 总线状态定义 */
typedef enum
{
    SI2C_READY = 0,
    SI2C_BUS_BUSY = 1,
    SI2C_BUS_ERROR = 2,
    SI2C_NACK = 3,
    SI2C_ACK = 4,
} I2C_Bus_State_t;

//******************************************************************************
/* PAN211 I2C 设备地址宏定义 */
#define I2C_DEV_ADDR    0x71

// T3488:
// PIN1: AP_SDO\AP_AD0,
// AD0=GND: 0x68, AD0=VDD: 0x69,
#define T3488_DEVICE_ADDR         	0x68	// 0x68\0x69,

// LPC2623 I2C设备地址 (7位地址)
// SDO=GND:0x76, SDO=VDD:0x77
#define LPC2623_I2C_ADDR          	0x76	// 0x76\0x77,

/* I2C ACK 最大超时计数宏定义，具体需要根据你的硬件来设置 */
#define ACK_TIMEOUT_DEF 10000

//******************************************************************************
/* TODO: 根据你的硬件实现
 *  - I2C_SCK引脚配置为推挽输出
 *  - I2C_SDA引脚配置为推挽输出或输入带上拉电阻模式
 *  注意：使用I2C时，CS引脚必须外部拉高或者浮空
 */
//******************************************************************************
#define SCK_PIN		GPIO_PA0
#define SDA_PIN		GPIO_PA1

#define I2C_SCK_LOW     gpio_write(SCK_PIN, 0)   /* 将I2C_SCK引脚拉低 */
#define I2C_SCK_HIGH    gpio_write(SCK_PIN, 1)   /* 将I2C_SCK引脚拉高 */
#define I2C_SDA_LOW     gpio_write(SDA_PIN, 0)   /* 将I2C_SDA引脚拉低 */
#define I2C_SDA_HIGH    gpio_write(SDA_PIN, 1)   /* 将I2C_SDA引脚拉高 */
#define I2C_SDA_STATUS  gpio_read(SDA_PIN)       /* 获取I2C_SDA电平状态 */
#define I2C_SDA_OUTPUT  gpio_init_out_High(SDA_PIN) /* 设置I2C_SDA引脚为输出模式 */
#define I2C_SDA_INPUT   gpio_intput_pullup_10k(SDA_PIN)  /* 设置I2C_SDA引脚为输入模式 */

#define I2C_DELAY()     WaitUs(1) /* 根据你的硬件实现, 保证I2C最高频率不超过2MHz */
//******************************************************************************
//******************************************************************************
/**
 * @brief I2C 开始信号
 * @param 无
 * @return 无
 * @note 调用该函数前，须保证SDA和SCL线为高电平,否则可能引起总线错误
 * @note 调用该函数后，SDA和SCL线均会被拉低，等待主机后续发送数据
 * @note 当SCL线为高电平时，拉低SDA线产生开始信号，时序如下：
 *       SCL:  ‾‾‾‾‾‾\___
 *       SDA:  ‾‾‾‾\_____
 */
//******************************************************************************
void I2C_Start(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    I2C_DELAY();    /* 延时，使用低速MCU，可删除延时语句 */
    I2C_SDA_OUTPUT; /* 设置SDA为输出模式 */
    I2C_SDA_HIGH;   /* 拉高SDA线 */
    I2C_SCK_HIGH;   /* 拉高SCK线 */
    I2C_DELAY();
    I2C_SDA_LOW;    /* 拉低SDA线，产生开始信号 */
    I2C_DELAY();
    I2C_SCK_LOW;    /* 拉低SCK线，准备发送数据 */
    I2C_DELAY();
}

//******************************************************************************
/**
 * @brief I2C 停止信号
 * @param 无
 * @return 无
 * @note 调用该函数前，须保证SCL线为低电平,否则可能引起总线错误
 * @note 调用该函数后，SDA和SCL线会被拉高，I2C总线处于空闲状态
 * @note 当SCL线为高电平时，SDA线从低电平拉高产生停止信号，时序如下：
 *       SCL:  ___/‾‾‾‾‾‾
 *       SDA:  _____/‾‾‾‾
 */
//******************************************************************************
void I2C_Stop(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    I2C_DELAY();    /* 延时，使用低速MCU，可删除延时语句 */
    I2C_SDA_OUTPUT; /* 设置SDA为输出模式 */
    I2C_SDA_LOW;    /* 拉低SDA线 */
    I2C_DELAY();
    I2C_SCK_HIGH;   /* 拉高SCK线，准备发送停止信号 */
    I2C_DELAY();
    I2C_SDA_HIGH;   /* 拉高SDA线，产生停止信号 */
    I2C_DELAY();
    I2C_SDA_INPUT;  /* 设置SDA为输入模式，释放SDA线 */
}

//******************************************************************************
/**
 * @brief I2C 发送ACK
 * @param 无
 * @return 无
 * @note 调用该函数前，须保证SCL处于低电平
 * @note 发送ACK信号后，SDA线被拉高，SCL线被拉低，时序如下：
 *       SCL:  ___/‾‾‾\_____
 *       SDA:  _________/‾‾‾
 *             ACK信号
 */
//******************************************************************************
void I2C_SEND_ACK(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    I2C_SDA_OUTPUT;  /* 设置SDA为输出模式 */
    I2C_SDA_LOW;     /* 拉低SDA线准备发送ACK信号 */
    I2C_DELAY();
    I2C_SCK_HIGH;    /* 拉高SCL线，发送ACK信号  */
    I2C_DELAY();
    I2C_SCK_LOW;     /* 拉低SCL线，准备发送下一个比特 */
    I2C_DELAY();
    I2C_SDA_HIGH;    /* 发送完ACK信号后，拉高SDA线 */
}

//******************************************************************************
/**
 * @brief I2C 发送 NACK
 * @param 无
 * @return 无
 * @note 调用该函数前，须保证SDA线为低电平
 * @note 发送NACK信号后，SDA线会被拉高，SCL线被拉低，时序如下：
 *       SCL: _____/‾‾‾\___
 *       SDA: ___/‾‾‾‾‾‾‾‾‾
 */
//******************************************************************************
void I2C_SEND_NACK(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    I2C_SDA_OUTPUT;  /* 设置SDA为输出模式 */
    I2C_SDA_HIGH;    /* 拉高SDA线准备发送NACK信号 */
    I2C_DELAY();
    I2C_SCK_HIGH;    /* 拉高SCL线，发送NACK信号  */
    I2C_DELAY();
    I2C_SCK_LOW;     /* 拉低SCL线，准备发送下一个比特 */
    I2C_DELAY();
}

//******************************************************************************
//******************************************************************************
/**
 * @brief I2C 写入单个字节
 * @param[in] Value 要写入的单字节数据
 * @return I2C 总线状态
 * @note 该函数会等待ACK信号，如果没有收到ACK信号，则返回 SI2C_NACK
 * @note 调用该函数后，SDA线会被拉低，表示接收方已准备好接收下一个字节
 *       举例：写字节时序（SCL 上升沿采样，写入 0xCC）：
 *       SCL: ____|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___
 *       SDA: __|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_______________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_________________________
 *                1       1        0      0       1       1        0       0       ACK
 *       说明: 数据在 SCL 上升沿被采样。
 */
//******************************************************************************
I2C_Bus_State_t I2C_WriteByte(unsigned char Value)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    unsigned char i;
    uint32_t Timeout = ACK_TIMEOUT_DEF;

    I2C_DELAY();      /* 延时，使用低速MCU，可删除延时语句 */

    I2C_SDA_OUTPUT;   /* 设置SDA为输出模式 */
    I2C_SCK_LOW;      /* 拉低SCL线 */
    I2C_DELAY();
    I2C_SDA_LOW;      /* 拉低SDA线，准备发送数据 */

    /* 将数据从高位到低位依次发送到I2C总线上 */
    for (i = 0; i < 8; i++)
    {
        I2C_DELAY();
        if (Value & 0x80) /* 判断最高位数据 */
        {
            I2C_SDA_HIGH; /* 如果最高位为1，则发送高电平 */
        }
        else
        {
            I2C_SDA_LOW;  /* 如果最高位为0，则发送低电平 */
        }
        Value <<= 1;      /* 左移一位，准备发送下一个比特 */
        I2C_DELAY();
        I2C_SCK_HIGH;     /* 拉高SCL线，传输一位数据 */
        I2C_DELAY();
        I2C_SCK_LOW;      /* 拉低SCL线，准备发送下一个比特 */
    }

    I2C_SDA_INPUT;        /* 设置SDA为输入模式，准备接收ACK信号 */

    I2C_DELAY();
    I2C_SCK_HIGH;         /* 拉高SCL线，准备接收ACK信号 */
    I2C_DELAY();

    do    /* 等待ACK信号 */
    {
        if (I2C_SDA_STATUS == 0) /* 如果SDA线为低电平，表示接收到ACK信号 */
        {
            I2C_SCK_LOW;
            return SI2C_ACK;  /* 返回ACK状态 */
        }
        I2C_DELAY();
    } while (--Timeout > 0);

    I2C_SDA_OUTPUT; /* 设置SDA为输出模式 */
    I2C_DELAY();
    I2C_SCK_LOW;    /* 拉低SCL线，准备发送下一个比特 */

    return SI2C_NACK;  /* 超时未接收到ACK信号，返回NACK状态 */
}

//******************************************************************************
/**
 * @brief I2C 读取单个字节
 * @return 读取的数据字节
 * @note 该函数会等待ACK信号，如果没有收到ACK信号，则返回 SI2C_NACK
 * @note 举例：写字节时序（SCL 上升沿采样，写入 0x33）：
 *       SCL: ____|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___|‾‾‾|___
 *       SDA: __________________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_______________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_________
 *                0      0       1       1        0       0        1       1        ACK
 */
//******************************************************************************
unsigned char I2C_READ_BYTE(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    unsigned char i;
    unsigned char Value = 0;

    I2C_DELAY();
    I2C_SDA_INPUT;  /* 设置SDA为输入模式，准备接收数据 */
    I2C_DELAY();
    I2C_SCK_LOW;    /* 拉低SCL线，准备接收数据 */
    I2C_DELAY();

    /* 以高比特优先MSB方式接收8位数据 */
    /* 读取数据时，SDA线为输入模式，SCL线为输出模式 */
    for (i = 0; i < 8; i++)
    {
        I2C_SCK_HIGH;   /* 拉高SCL线，在上升沿触发数据 */
        I2C_DELAY();
        Value <<= 1;    /* 变量左移一位，准备存放下一个比特 */
        if (I2C_SDA_STATUS)  /* 判断SDA引脚电平 */
        {
            Value |= 0x01;   /* 如果SDA引脚为高电平，则将Value的最低位设置为1 */
        }
        I2C_SCK_LOW;         /* 拉低SCL线，准备接收下一个比特 */
        I2C_DELAY();
    }
    return Value;
}

//******************************************************************************
/**
 * @brief I2C 写入数据
 * @param Addr 要写入的地址
 * @param Buf 要写入的数据缓冲区
 * @param Len 要写入的数据长度
 */
//******************************************************************************
void I2C_WriteBuf(unsigned char Addr, unsigned char *Buf, unsigned char Len)
{
    unsigned char i;
    I2C_Bus_State_t Status;

    I2C_Start();   /* 发送开始信号 */

    /* 发送设备地址和读写位，bit0为读写位，0表示写操作, bit7~1表示设备地址 */
    Status = I2C_WriteByte(I2C_DEV_ADDR << 1);
    if (Status != SI2C_ACK)
    {
        goto exit;
    }

    /* 发送要写入的寄存器地址，bit0为读写位，0表示写操作, bit7~1表示寄存器地址 */
    Status = I2C_WriteByte(Addr << 1);
    if (Status != SI2C_ACK)
    {
        goto exit;
    }

    /* 发送要写入寄存器的一个或多个数据 */
    for (i = 0; i < Len; i++)
    {
        Status = I2C_WriteByte(Buf[i]);
        if (Status != SI2C_ACK)
        {
            goto exit;
        }
    }
    Status = SI2C_READY;

exit:
    I2C_Stop();
}

//******************************************************************************
/**
 * @brief I2C 读取数据
 * @param Addr 要读取的地址
 * @param Buf 存储读取数据的缓冲区
 * @param Len 要读取的数据长度
 * @return 读取操作的状态
 */
//******************************************************************************
unsigned char I2C_ReadBuf(unsigned char Addr, unsigned char *Buf, unsigned char Len)
{
    unsigned char i;
    I2C_Bus_State_t Status;

    I2C_Start();  /* 发送开始信号 */

    /* 发送设备地址和读写位，bit0为读写位，0表示写操作, bit7~1表示设备地址 */
    Status = I2C_WriteByte((I2C_DEV_ADDR << 1));
    if (Status != SI2C_ACK)
    {
        goto exit;
    }

    /* 发送要读取的寄存器地址，bit0为读写位，0表示写操作, bit7~1表示寄存器地址 */
    Status = I2C_WriteByte((Addr << 1) | 0x01);
    if (Status != SI2C_ACK)
    {
        goto exit;
    }

    I2C_Start();  /* 发送重复开始信号 */

    /* 发送设备地址和读写位，bit0为读写位，1表示读操作, bit7~1表示设备地址 */
    Status = I2C_WriteByte((I2C_DEV_ADDR << 1) | 0x01);
    if (Status != SI2C_ACK)
    {
        goto exit;
    }

    /* 读取数据，最后一个字节发送NACK信号 */
    for (i = 0; i < Len; i++)
    {
        Buf[i] = I2C_READ_BYTE(); /* 读取数据 */
        if (i != Len - 1)
        {
            I2C_SEND_ACK();   /* 发送ACK信号，表示继续接收数据 */
        }
        else
        {
            I2C_SEND_NACK();  /* 发送NACK信号，表示接收完毕 */
        }
    }
    Status = SI2C_READY;

exit:
    I2C_Stop();

    return Status;
}

//******************************************************************************
/**
 * @brief I2C复位，用于清除I2C总线上的错误状态
 * @param 无
 * @return 无
 * @note MCU有可能在操作I2C总线期间发生复位或错误，导致I2C总线处于错误状态。
 * @note 该函数可将I2C总线复位到正常状态，避免后续操作失败。
 */
//******************************************************************************
void i2c_reset(void)
{
    /* TODO: 根据你的硬件实现，下面是软件I2C实现示例 */
    unsigned char Timeout = 10;
    unsigned char i = 0;
    while (Timeout--)
    {
        /* 发送8个时钟脉冲和停止信号将I2C总线复位 */
        for (i = 0; i < 8; i++)
        {
            I2C_SCK_HIGH;
            I2C_DELAY();
            I2C_SCK_LOW;
            I2C_DELAY();
        }
        I2C_Stop();

        if (I2C_SDA_STATUS) /* 如果SDA线为高电平，表示I2C总线已经复位 */
        {
            break;
        }
    }
}

//******************************************************************************
/**
 * @brief 当 SDA 复用中断时，在 I2C 空闲期间检测 SDA 引脚是否有中断事件，低电平有效。
 * @note SDA 复用中断功能的开启方法是将 IRQ_I2C_MUX_EN（Page0 0x06[3]）置1。
 * @return 1 有中断触发
 * @return 0 没有中断触发
 */
unsigned char IRQDetected(void)
{
    return I2C_SDA_STATUS == 0;
}

//******************************************************************************
//******************************************************************************



//******************************************************************************
//******************************************************************************

