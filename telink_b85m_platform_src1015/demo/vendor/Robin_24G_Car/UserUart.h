//*******************************************************************
/*
 * UserUart.h
 *
 *  Created on: 2026年7月26日
 *      Author: HuangGuangjian
 */
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_DEMO_USERUART_H_
#define VENDOR_ROBIN_24G_DEMO_USERUART_H_
//*******************************************************************
/******************set mode**********************/
#define UART_DMA  		 1     //uart use dma
#define UART_NDMA  		 2     //uart not use dma
#define UART_SOFTWARE_RX 3     //software simulates the UART receiving function,occupy an GPIO interrupt and a hardware timer
#define UART_MODE	 	UART_DMA

/***********set uart_dma interrupt type*************/
#define UART_RXDMA_IRQ   1
#define UART_RXDONE_IRQ  2
#define UART_DMA_INT_TYPE UART_RXDMA_IRQ

#define NONE            0
#define USE_CTS    		1
#define USE_RTS    		2

#define FLOW_CTR  		NONE

#define UART_1WIRE_MODE       0  //tx and rx is a same line
#define UART_2WIRE_MODE       1  //tx and rx are two lines
#define UART_WIRE_MODE        UART_2WIRE_MODE
#if( FLOW_CTR==USE_CTS)
	#define STOP_VOLT   	1			//0 :Low level stops TX.  1 :High level stops TX.
#elif (FLOW_CTR==USE_RTS)
	#define RTS_MODE		UART_RTS_MODE_AUTO 		    //It can be UART_RTS_MODE_AUTO/UART_RTS_MODE_MANUAL.
	#define RTS_THRESH		5			//UART_RTS_MODE_AUTO need.It indicates RTS trigger threshold.
	#define RTS_INVERT		1			//UART_RTS_MODE_AUTO need.0 indicates RTS_pin will change from low to hign.
	#define RTS_POLARITY	0			//UART_RTS_MODE_MANUAL need. It indicates RTS_POLARITY .
#endif
//*******************************************************************************************************
//*******************************************************************
#define Used_Uart		1

//*******************************************************************
#define Uart_Txdat_len    12      //data max ?    (Uart_Txdat_len+4) must 16 byte aligned
#define Uart_Rxdat_len    28      //data max ?    (Uart_Rxdat_len+4) must 16 byte aligned

typedef struct{
 unsigned int len;        // dma len must be 4 byte
 unsigned char UartHeader[2];		//Header(2Bytes)=0x2612
 unsigned char Command;
 unsigned char Data_1;
 unsigned char Data_2;
 unsigned char Reserve[7];
}uart_txdata_t;

typedef struct{
 unsigned int len;        // dma len must be 4 byte
 unsigned short UartHeader;		//Header(2Bytes)=0x2510
 unsigned char data_len;
 unsigned char data_Ack_Cmd;
 unsigned char Parameter[6];
 unsigned char Reserve[2];
}uart_rxdata_t;

#define UART_DATA_LEN    36      // ★v24: 252→36（黑匣子读出已删除）。
									//   调试帧 21B，36B 足够；原 252B 白占 216B 压垮栈
typedef struct{
 unsigned int len;        // dma len must be 4 byte
 unsigned char data[UART_DATA_LEN];
}uart_data_t;

uart_data_t T_txdata_user;

//---- ★v25 flash 黑匣子读出专用（阻塞式发送，复用 T_txdata_user）----
void UartBBSend(unsigned char *dat, unsigned char len);			// 小包（起始/结束帧）
void UartBBSendFlash(unsigned long addr, unsigned char len);	// flash 块直读直发

//*******************************************************************
//*******************************************************************
//extern volatile unsigned char Uart_send_cmdBak;
//extern volatile unsigned char Uart_send_KeyBak;
extern unsigned char UartTestBuff[10];

extern volatile unsigned char Waiting_Uart_Cnt;
extern volatile unsigned char g_WaitUartBack;
extern unsigned char NoRcvTmCnt;
extern volatile unsigned char utx_Packet_no;

//*******************************************************************
//*******************************************************************
//void SetUrx_gpio_wakeup(void);

void user_uart_init(void);
void user_uart_send(unsigned char *data,unsigned int len);
//void Uart_Send_Ctrl_Data(void);	//10mS loop
//void Uart_Send_Off_Cmd(void);
//void Uart_Send_On_Cmd(void);
void clear_Uart_Rcvbuffer(void);
//void Check_Uack_tmOver(void);

void UartPrintData(u32 PrintDat);

void UartDbgSend(unsigned char *dat, unsigned char len);


void Uart_irq_handler(void);
//*******************************************************************
#endif /* VENDOR_ROBIN_24G_DEMO_USERUART_H_ */
//*******************************************************************

