//*******************************************************************
/*
 * UserUart.c
 *
 *  Created on: 2026年7月26日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#include "app_public_config.h"
#include <UserUart.h>
#include "driver.h"

// ******************************************************************
//#define Used_Uart	1
// ******************************************************************
#if(Used_Uart)
//**************************************************************************
//**************************************************************************
/**
 * rec_buff_Len Set up rules:(Both conditions have to be met)
 * 1.The rec_buff_Len value is a multiple of 16 	(16 * n)
 * 2.The length of the receive data is less than (16 * n -4)
 */
//#define rec_buff_Len    32
//#define trans_buff_Len  16

#define TX     1
#define RX     2
#define UART_MODE_TRX RX

//__attribute__((aligned(4))) unsigned char rec_buff[rec_buff_Len] = {0};
//__attribute__((aligned(4))) unsigned char Utrans_buff[trans_buff_Len] = {0};
__attribute__((aligned(4))) uart_rxdata_t rec_buff;
__attribute__((aligned(4))) uart_txdata_t Utrans_buff;
__attribute__((aligned(4))) uart_rxdata_t Urx_Buffer;
volatile unsigned char uart_dmairq_tx_cnt;
volatile unsigned char uart_dmairq_rx_cnt;
volatile unsigned char uart_dmairq_err_cnt;
volatile unsigned char uart_dma_send_flag = 0;
volatile unsigned char uart_Rcv_New=0;
volatile unsigned char uart_send_end = 1;	// =1 视为空闲，允许立即启动发送（TX_DONE 中断再置 1）

//__attribute__((aligned(4))) uart_data_t Utrans_buff;

//********************************************************************
#if( UART_WIRE_MODE == UART_1WIRE_MODE)

#define	UART_RTX_PIN			GPIO_PA1	//Dat2Slave

#elif(( UART_WIRE_MODE == UART_2WIRE_MODE))

#define	Uart_Rx					Isen1_Pin	//
#define	Uart_Tx					Isen2_Pin
//#define	Uart_Rx					GPIO_PA2		//VBat_Adc
//#define	Uart_Tx					GPIO_PA5		//Dat2Slave

#endif

//********************************************************************
// for Uart control program
//********************************************************************
void user_uart_init(void)
{
	//note: dma addr must be set first before any other uart initialization! (confirmed by sihui)
	uart_recbuff_init( UART0, (unsigned char *)&rec_buff, sizeof(rec_buff));

#if( UART_WIRE_MODE == UART_1WIRE_MODE)
	uart_set_rtx_pin(UART0, UART_RTX_PIN);// the status of rtx line will be rx by default,if there is a send-action,the status of rtx-line will changed to tx,and changed to rx immediately if send over.
	uart_rtx_en(UART0);
#elif(( UART_WIRE_MODE == UART_2WIRE_MODE))
	uart_gpio_set(UART0, Uart_Tx, Uart_Rx);// uart tx/rx pin set
#endif

	uart_reset(UART0);  //uart module power-on again.

	//baud rate: 115200
	uart_init_baudrate(UART0, 115200,CLOCK_SYS_CLOCK_HZ,PARITY_NONE, STOP_BIT_ONE);
	uart_dma_enable(UART0, 0, 1); 	//uart data in hardware buffer moved by dma, so we need enable them first

#if(UART_DMA_INT_TYPE == UART_RXDMA_IRQ)
	irq_enable_type(FLD_IRQ_DMA_EN);// uart_rx use dma_rx irq

	//dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 0);
	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX, 1);   	//uart Rx dma irq enable
#elif((UART_DMA_INT_TYPE == UART_RXDONE_IRQ))
	uart_rxdone_irq_en(UART0);  //mask rx_done irq
#endif

	uart_mask_tx_done_irq_enable(UART0);

	uart_mask_error_irq_enable(UART0);// open uart_error_mask,when stop bit error or parity error,it will enter error_interrupt.

	irq_enable_type(FLD_IRQ_UART_EN);// uart_tx use uart_txdone irq

	irq_enable();

#if( FLOW_CTR ==  USE_CTS )
	uart_set_cts(UART_MODULE_SEL,1, STOP_VOLT, UART_CTS_PIN);
#elif( FLOW_CTR ==   USE_RTS )
	uart_set_rts(UART_MODULE_SEL,1, RTS_MODE, RTS_THRESH, RTS_INVERT,UART_RTS_PIN);
#endif





//	irq_enable();


//	uart_gpio_set(Uart_Tx,Uart_Rx);
//	uart_init_baudrate(115200,16000000,PARITY_NONE,STOP_BIT_ONE);
//	uart_dma_enable(0,1);
	//uart_irq_enable(UART0, 1,1);
}

//********************************************************************
//void SetUrx_gpio_wakeup(void)
//{
//	gpio_set_func(Uart_Rx,AS_GPIO);
//	gpio_set_input_en(Uart_Rx, 1);
//	gpio_set_output_en (Uart_Rx, 0);
//	gpio_setup_up_down_resistor(Uart_Rx, PM_PIN_PULLUP_10K);  //must  for stability and prevent from current leakage
//	cpu_set_gpio_wakeup(Uart_Rx, Level_Low, 1);
//}
//********************************************************************
unsigned char UartTestBuff[10]={0x01,0x02,0x03,0x04,0x05};
// Samples:	user_uart_send(UartTestBuff,4);
//********************************************************************
void user_uart_send(unsigned char *data,unsigned int len)
{
	unsigned int i;
	T_txdata_user.len=len;
	 for(i=0;i<len;i++) T_txdata_user.data[i]=data[i];
	 uart_send_dma(UART0, (unsigned char *)&T_txdata_user);
//	 //uart_send_byte(TestBuff[0]);
//	 //uart_ndma_send_byte(TestBuff[2]);
}

//********************************************************************
//********************************************************************
void UartPrintData(u32 PrintDat)
{
	unsigned char UartTestBuff[6];
	UartTestBuff[0] = 0x26;
	UartTestBuff[1] = 0x99;
	UartTestBuff[2] = PrintDat>>24;
	UartTestBuff[3] = PrintDat>>16;
	UartTestBuff[4] = PrintDat>>8;
	UartTestBuff[5] = PrintDat;
	user_uart_send(UartTestBuff,6);

}

//********************************************************************
// 高速调试报文发送（2026/11/07 新增，配合 FlightCtrl.h 7.9 使用）：
//	本 UART 的 TX 是"单缓冲 + DMA"机制：必须等上一次发送完成后才能
//	覆盖缓冲，否则 DMA 还在发送的帧会被新数据从中间截断。
//	这里带超时保护：等不到空闲就放弃本帧，绝不阻塞控制节拍。
//	注意：usart1_report_EulerDat() 与本函数共用同一缓冲/ DMA，录
//	FlightCtrl 调试报文期间请勿同时打开 Euler 报文（会互相丢帧）。
//********************************************************************
void UartDbgSend(unsigned char *dat, unsigned char len)
{
	unsigned int tmo = 40000;
	unsigned char i;

	while((!uart_send_end) && (--tmo));
	if(tmo == 0) return;					// 上一次发送未完成：放弃本帧

	for(i = 0; i < len; i++) T_txdata_user.data[i] = dat[i];
	T_txdata_user.len = len;
	uart_send_end = 0;
	uart_send_dma(UART0, (unsigned char *)&T_txdata_user);
}

//********************************************************************
// flash 黑匣子读出专用发送（★v25 精简版回归）：
//	与 UartDbgSend 的区别：发送后阻塞等发完（读出流不能丢帧/乱序）。
//	缓冲复用 T_txdata_user；读出期间 FlightCtrl 暂停调试帧发送。
//********************************************************************
void UartBBSend(unsigned char *dat, unsigned char len)	// 小包（起始/结束帧）
{
	unsigned int tmo = 200000;
	unsigned char i;

	while((!uart_send_end) && (--tmo));
	if(tmo == 0) return;
	for(i = 0; i < len; i++) T_txdata_user.data[i] = dat[i];
	T_txdata_user.len = len;
	uart_send_end = 0;
	uart_send_dma(UART0, (unsigned char *)&T_txdata_user);
	while((!uart_send_end) && (--tmo));		// 阻塞等发完
}

void UartBBSendFlash(unsigned long addr, unsigned char len)	// flash 块直读直发
{
	unsigned int tmo = 200000;

	while((!uart_send_end) && (--tmo));
	if(tmo == 0) return;
	flash_read_page(addr, len, T_txdata_user.data);
	T_txdata_user.len = len;
	uart_send_end = 0;
	uart_send_dma(UART0, (unsigned char *)&T_txdata_user);
	while((!uart_send_end) && (--tmo));		// 阻塞等发完
}

//********************************************************************
#if 0
//********************************************************************
//;Uart 通讯协议：
//;    采用标准的Uart通讯，波特率：100000bps 即:100Kbps
//;数据格式：
//;    Header(2Bytes) + Command(1Byte) + Data(1Bytes) + Chksum(1Byte)
//;    Header:    0x2612
//;    Command:
//;            00:        无功能
//;            01:        开机            Data = 0x01        //进入正常工作状态
//;            02:        关机            Data = 0x02        //进入睡眠状态
//;            03:        输出控制        Data = 1Byte
//;
//;    Chksum:    = Data
//;*********************************************************************
//;*********************************************************************
//范例：
//开机		26 12 01 01 01
//关机		26 12 02 02 02
//输出控制	26 12 03 2A 2A
//********************************************************************
_attribute_data_retention_ volatile unsigned char utx_Packet_no;
//********************************************************************
void Uart_Send_Ctrl_Data(void)	//10mS loop
{
//	g_WaitUartBack=1;
//	Waiting_Uart_Cnt=0;

	Utrans_buff.len=5;
	Utrans_buff.UartHeader[0]=0x26;
	Utrans_buff.UartHeader[1]=0x12;
	Utrans_buff.Command=0x03;
	//Uart_send_cmdBak=Utrans_buff.Command;
	Utrans_buff.Data_1=MotorFlag;
	Utrans_buff.Data_2=MotorFlag;

	uart_send_dma(UART0, (unsigned char *)&Utrans_buff);
}

//********************************************************************
void Uart_Send_Off_Cmd(void)
{
//	g_WaitUartBack=1;
//	Waiting_Uart_Cnt=0;

	Utrans_buff.len=5;
	Utrans_buff.UartHeader[0]=0x26;
	Utrans_buff.UartHeader[1]=0x12;
	Utrans_buff.Command=0x02;
	//Uart_send_cmdBak=Utrans_buff.Command;
	Utrans_buff.Data_1=0x02;
	Utrans_buff.Data_2=0x02;

	uart_send_dma(UART0, (unsigned char *)&Utrans_buff);
}

//********************************************************************
void Uart_Send_On_Cmd(void)
{
//	g_WaitUartBack=1;
//	Waiting_Uart_Cnt=0;

	Utrans_buff.len=5;
	Utrans_buff.UartHeader[0]=0x26;
	Utrans_buff.UartHeader[1]=0x12;
	Utrans_buff.Command=0x01;
	//Uart_send_cmdBak=Utrans_buff.Command;
	Utrans_buff.Data_1=0x01;
	Utrans_buff.Data_2=0x01;

	uart_send_dma(UART0, (unsigned char *)&Utrans_buff);
}

//********************************************************************
#endif  //0
//********************************************************************
void clear_Uart_Rcvbuffer(void)
{
	rec_buff.len=0;
	rec_buff.data_len=0;
	rec_buff.data_Ack_Cmd=0;

}


//********************************************************************
#if 0
//********************************************************************
//如果没有收到应答则以100mS的间隔时间重发最多9次
//********************************************************************
#define UartResendTimesMax		9
#define UartResendInterval		10		//unit=10mS

_attribute_data_retention_ volatile unsigned char Waiting_Uart_Cnt;
volatile unsigned char Uart_Resend_Cnt;
//********************************************************************
void Check_Uack_tmOver(void)
{
	if((g_WaitUartBack)&&(Uart_Resend_Cnt < UartResendTimesMax))
	{
		Waiting_Uart_Cnt++;
		if(Waiting_Uart_Cnt >  UartResendInterval)
		{
			Waiting_Uart_Cnt=0;
			Uart_Resend_Cnt++;

		}
	}
	else if(g_WaitUartBack==0)
	{
		Uart_Resend_Cnt=0;
		Waiting_Uart_Cnt=0;
	}
	else if(Uart_Resend_Cnt >= UartResendTimesMax)
	{
		g_WaitUartBack=0;
		Uart_Resend_Cnt=0;
		Waiting_Uart_Cnt=0;
	}
}


//*******************************************************************
#endif
// ******************************************************************
//for Uart interrupt
//*******************************************************************
// ******************************************************************
/**
 * rec_buff_Len Set up rules:(Both conditions have to be met)
 * 1.The rec_buff_Len value is a multiple of 16 	(16 * n)
 * 2.The length of the receive data is less than (16 * n -4)
 */
//#define rec_buff_Len    	32
//#define trans_buff_Len    	16
volatile unsigned char rts_count=1;

#if 0	//GJ 20260726
#if(UART_MODE==UART_NDMA)
extern volatile unsigned char uart_rx_flag;
extern volatile unsigned char uart_rx_done_flag;
extern volatile unsigned int  uart_ndmairq_cnt;
extern volatile unsigned char uart_ndmairq_index;
extern volatile  unsigned char uart_rx_trig_level;

#elif(UART_MODE==UART_DMA)
extern volatile unsigned char uart_dmairq_tx_cnt;
extern volatile unsigned char uart_dmairq_rx_cnt;
extern volatile unsigned char uart_dma_send_flag;
extern volatile unsigned char uart_dmairq_err_cnt;
#endif

extern volatile unsigned char uart_Rcv_New;
extern volatile unsigned char uart_send_end;

//extern unsigned char rec_buff[rec_buff_Len];
//extern unsigned char trans_buff[trans_buff_Len];
extern uart_rxdata_t rec_buff;
extern uart_txdata_t Utrans_buff;
extern uart_txdata_t Urx_Buffer;

//volatile unsigned char Urx_Buffer[16];
//extern void testing_send_uart(void);
#endif
//*******************************************************************
// ******************************************************************
_attribute_ram_code_sec_noinline_ void Uart_irq_handler(void)
{
	if(reg_uart_status1(0) & FLD_UART_TX_DONE)
	{
		uart_send_end=1;
		uart_dmairq_tx_cnt++;
//		uart_dma_send_flag = 1;
//		gpio_toggle(LED2);
		uart_clr_tx_done(UART0);
	}

#if(UART_DMA_INT_TYPE == UART_RXDMA_IRQ)
	if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_RX))
	{
		uart_Rcv_New=1;
//		testing_send_uart();
//      uart_send_dma(UART0,(unsigned char *)rec_buff);
		unsigned char *Urxpoint=(unsigned char *)&rec_buff;
		unsigned char *UrxDat=(unsigned char *)&Urx_Buffer;
		unsigned char i;
		for(i=0; i<16; i++) UrxDat[i]=Urxpoint[i];

		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		uart_dmairq_rx_cnt++;
	}
#elif(UART_DMA_INT_TYPE == UART_RXDONE_IRQ)
	if(reg_uart_status1(UART0)&FLD_UART_RXDONE_IRQ)
	{
		uart_send_dma(UART0,(unsigned char *)rec_buff);
		 reg_uart_status0(UART0) =FLD_URAT_CLEAR_RXDONE_FLAG;
		uart_dmairq_rx_cnt++;
	}
#endif

	if(uart_is_parity_error(UART0))		//when stop bit error or parity error.
	{
		uart_clear_parity_error(UART0);
//		gpio_write(LED4,1);
		uart_dmairq_err_cnt++;
	}
}

//********************************************************************
#endif  //Used_Uart
//********************************************************************
#if 0
_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
#if (UART_MODE==UART_DMA)
	if(reg_uart_status1(UART0) & FLD_UART_TX_DONE)
	{
		uart_dmairq_tx_cnt++;
		uart_dma_send_flag = 1;
		uart_clr_tx_done(UART0);
	}
	#if(UART_DMA_INT_TYPE == UART_RXDMA_IRQ)

	if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_RX))

	{
        uart_send_dma(UART0,(unsigned char *)rec_buff);

		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);

		uart_dmairq_rx_cnt++;
	}
	#elif(UART_DMA_INT_TYPE == UART_RXDONE_IRQ)
	if(reg_uart_status1(UART0)&FLD_UART_RXDONE_IRQ)
	{
		uart_send_dma(UART0,(unsigned char *)rec_buff);
		 reg_uart_status0(UART0) =FLD_URAT_CLEAR_RXDONE_FLAG;
		uart_dmairq_rx_cnt++;
	}
	#endif
	if(uart_is_parity_error(UART0))//when stop bit error or parity error.
	{
		uart_clear_parity_error(UART0);
		uart_dmairq_err_cnt++;
	}
#elif(UART_MODE==UART_NDMA)
	static unsigned char uart_ndma_irqsrc;
	uart_ndma_irqsrc = uart_ndmairq_get(UART_MODULE_SEL);
	if(uart_ndma_irqsrc)
	{
	 if(reg_uart_status1(UART_MODULE_SEL)&FLD_UART_RX_BUF_IRQ)
	 {
		 if(uart_rx_flag==0)
		 {
			 unsigned char fifo_cnt=reg_uart_buf_cnt(UART_MODULE_SEL)&FLD_UART_RX_BUF_CNT;
			 for(int i=0;i<fifo_cnt;i++){

				 if((uart_ndmairq_cnt%trans_buff_Len==0)&&(uart_ndmairq_cnt!=0))
				 {
					 uart_ndma_read_byte(UART_MODULE_SEL);
				 }else{
					 rec_buff[uart_ndmairq_cnt++] = uart_ndma_read_byte(UART_MODULE_SEL);
				 }
			 }
			 if((uart_ndmairq_cnt%trans_buff_Len==0)&&(uart_ndmairq_cnt!=0))
			 {
				 uart_rx_flag=1;
			 }

		 }
		 else
		 {
			 unsigned char uart_fifo_cnt =0;
			 uart_fifo_cnt= reg_uart_buf_cnt(UART_MODULE_SEL)&FLD_UART_RX_BUF_CNT;
			 if(uart_fifo_cnt !=0)
			 {
				 for(unsigned char j=0;j<uart_fifo_cnt;j++){
					 uart_ndma_read_byte(UART_MODULE_SEL);
				 }
			 }
		 }

	 }
	 if(reg_uart_status1(UART_MODULE_SEL)&FLD_UART_RXDONE_IRQ)
	 {
         unsigned char uart_rxbuff_cnt=0;
		 uart_rxbuff_cnt= reg_uart_buf_cnt(UART_MODULE_SEL)&FLD_UART_RX_BUF_CNT;
		 if(uart_rxbuff_cnt>0)
		 {
			 for(int j=0;j<uart_rxbuff_cnt;j++)
			{
				 if(uart_rx_flag==0)
				 {
					 rec_buff[uart_ndmairq_cnt++] = uart_ndma_read_byte(UART_MODULE_SEL);
				     if((uart_ndmairq_cnt%trans_buff_Len==0)&&(uart_ndmairq_cnt!=0))
				     {
				    	 uart_rx_flag=1;
				    	 uart_rx_done_flag=1;
				    	 break;
				     }
				 }

			}


		 }
		 if(uart_rx_flag==1)
		 {
			 uart_rx_done_flag=1;

		 }
		 reg_uart_status0(UART_MODULE_SEL) =FLD_URAT_CLEAR_RXDONE_FLAG;
		 uart_reset(UART_MODULE_SEL);//clearing the rx_buff does not clear the rx_fifo and requires uart_reset.
		 uart_ndma_clear_rx_index(UART_MODULE_SEL);
		 reg_uart_status0(UART_MODULE_SEL)=FLD_UART_CLEAR_RX_FLAG;
	 }

     if(uart_is_parity_error(UART_MODULE_SEL))
     {
    	 uart_reset(UART_MODULE_SEL);
    	 uart_ndma_clear_rx_index(UART_MODULE_SEL);
    	 uart_ndmairq_cnt=0;
    	 uart_rx_flag=0;
    	 for(int i=0;i<rec_buff_Len;i++)
		 {
    		 rec_buff[i]=0;
		 }

     }
}
#endif

}
#endif

//*******************************************************************
//*******************************************************************

