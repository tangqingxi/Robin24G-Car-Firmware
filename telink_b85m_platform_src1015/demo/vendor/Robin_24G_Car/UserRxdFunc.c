//*******************************************************************
/*
 * UserRxdFunc.c
 *
 *  Created on: 2026年8月20日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#include <UserFilter.h>
#include	"UserRxdFunc.h"
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "app_battery_check.h"
#include "app_24g_rx.h"
#include "UserSpeechFunc.h"
#include "UserPwm.h"
#include <UserUart.h>
#include "UserFilter.h"
#include "UserEuler.h"
#include "my_t3488.h"
#include "User_IIC.h"
#include "UserAdc.h"
#include "PressSen_Lpc2623.h"
#include "FlightCtrl.h"
#include "PerfMeas.h"

//*************************************************************************
#if Project_XH261009_Rxd
//**************************************************************
//*************************************************************************
unsigned char T250uS_Cnt, T2mS, R_2ms_Cnt;
unsigned char T10mSflag, T100mSflag, T500mSflag, Sys10mScnt, Sys100mScnt;
unsigned int	u32_SysTick, u32_SysCount, u32_Sys10mScnt, u32_Sys500mScnt;
unsigned int u32NoActionCnt, u32NoRxdCnt;


//**************************************************************
//**************************************************************
//**************************************************************
//将PA3口转换为SWS属性，用于Bebug程序时与上位机通信
//**************************************************************
void ChangePA3ToSWS(void)
{
	gpio_set_func(GPIO_PA3,AS_SWIRE);
	gpio_setup_up_down_resistor(GPIO_PA3,PM_PIN_PULLUP_1M);			//设置上拉
	gpio_set_output_en(GPIO_PA3, 0); //Disable output
	gpio_set_input_en(GPIO_PA3,1);//enable input
}

//**************************************************************
//将PA3口转换为GPIO属性，用于执行项目功能
//**************************************************************
void ChangePA3ToGPIO_OutLow(void)
{
	gpio_init_out_low(GPIO_PA3);
	gpio_setup_up_down_resistor(GPIO_PA3,PM_PIN_UP_DOWN_FLOAT);			//设置上拉
}

void ChangePA3ToGPIO_OutHigh(void)
{
	gpio_init_out_High(GPIO_PA3);
	gpio_setup_up_down_resistor(GPIO_PA3,PM_PIN_UP_DOWN_FLOAT);			//设置上拉
}

//*************************************************************************
//for Testing pin
//*************************************************************************
void SetTestingPin(void)
{
	gpio_set_func(Testing_Pin,AS_GPIO);
	gpio_setup_up_down_resistor(Testing_Pin,PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(Testing_Pin,0);//disable input
	gpio_set_output_en(Testing_Pin, 1); //enable output
	gpio_write(Testing_Pin, 0);
}

//**************************************************************
//250uS定时器的应用层中断服务程序
//**************************************************************
//**************************************************************
void T250uSInterrupt(void)
{
		//	User can add code, 250uS cycle
		T250uS_Cnt++;
		if(T250uS_Cnt >= 8)
		{
			T250uS_Cnt=0;
			T2mS=1;
			R_2ms_Cnt++;
			//LedScanControl();		//2mS loop
		}

		//gpio_write(Testing_Pin, 1);
//		SoftPWM_Hander();		//使用时基控制GPIO 实现PWM
		//gpio_write(Testing_Pin, 0);
		//gpio_toggle(GPIO_PA5);	//Test_Pin 翻转
}

//**************************************************************
//2000uS定时器的应用层中断服务程序
//**************************************************************
void T2000uSInterrupt(void)
{
		//	User can add code, 2000uS cycle
	R_2ms_Cnt++;
	T2mS=1;
	//LedScanControl();		//2mS loop
	//gpio_toggle(Testing_Pin);	//Test_Pin 翻转
}

//*************************************************************************
//初始化硬件接口
//*************************************************************************
void User_hw_init(void)
{
	//电源管理
	gpio_init_out_High(PowerLock_Pin);			//系统电源控制，上电后输出高电平，不操作5分钟输出低电平关机
	gpio_intput_float(LPDadc_Pin);		//低电压检测，ADC2
	gpio_intput_float(Isen1_Pin);		//大电流检测1，ADC1
	gpio_intput_float(Isen2_Pin);		//大电流检测2，ADC0
    adc_init(NDMA_M_CHN);

	//马达及舵机
	UserPwmPinInital();

	//Speech IC
	//gpio_init_out_High(SpeechCtrl);
#if (UsePwm0SoundCmd)
	InitalSpeechComm();
#endif

	user_uart_init();
	//IIC for 陀螺仪
	gpio_intput_pullup_10k(IIC_SDA_Pin);
	gpio_intput_pullup_10k(IIC_SCL_Pin);
	iic_pin_init();
	sleep_ms(10);
#if (C_ATT_BENCH_EN)
	// 台架姿态估计：传感器由 BenchAtt 自己初始化（含陀螺片内低通），
	// 这里跳过旧陀螺初始化与气压计初始化，避免重复复位和占用 I2C
#else
	t3488_gyro_init();
#if C_FILTER_5_EN
	t3488_data_filter_init();		// 中值滤波初始化以及处理
#endif
	//压力传感器
	Lpc2623_Init_Seting();
#endif

	ChangePA3ToSWS();		//SWS, 车体前灯,Low active
	//SetTestingPin();
}

//*************************************************************************
//*************************************************************************
void User_hw_Exit(void)
{
	timer_stop(TIMER0);
	timer_stop(TIMER1);
	User_pwm_Exit_func();
	User_hw_init();
	adc_power_off();

    gpio_setup_up_down_resistor(GPIO_PA3, PM_PIN_PULLUP_10K);
	cpu_set_gpio_wakeup(GPIO_PA3, Level_High, 0); // Disable analog pad wakeup
	gpio_setup_up_down_resistor(LPDadc_Pin, PM_PIN_UP_DOWN_FLOAT);
	gpio_setup_up_down_resistor(Isen1_Pin, PM_PIN_UP_DOWN_FLOAT);
	gpio_setup_up_down_resistor(Isen2_Pin, PM_PIN_UP_DOWN_FLOAT);

	gpio_setup_up_down_resistor(LFmotPwm_Pin, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(RBmotPwm_Pin, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(LBmotPwm_Pin, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(RFmotPwm_Pin, PM_PIN_PULLDOWN_100K);

	gpio_setup_up_down_resistor(IIC_SDA_Pin, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(IIC_SCL_Pin, PM_PIN_PULLUP_10K);

	gpio_setup_up_down_resistor(Testing_Pin, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(PowerLock_Pin, PM_PIN_PULLDOWN_100K);
}

//*************************************************************************
//*************************************************************************
unsigned char    link_ok_flag = 0;
unsigned char	FunCtrlFlag;

unsigned char rf_received_type, rf_received_key, testPoint;
//*************************************************************************
//*************************************************************************
//	ANT			--					XC0			--
//	GND			--					XC1			--
//	PA5(ADC4)	RMOT_F				PA7			RMOT_B
//	PA3(SWS)	speech SDA			PA6			LMOT F
//	PA4(ADC3)	LPump_F				PB2(ADC2)	Water_ADC
//	PA2			LPump_B				PB1(ADC1)	VB_ADC
//	PA1			RPump_F				PB0(ADC0)	LMOT_B
//	PA0			RPump_B				VCC			--
//*************************************************************************
//*************************************************************************
void InitalRam(void)
{
	LFmotDuty=0;
	RBmotDuty=0;
	LBmotDuty=0;
	RFmotDuty=0;
	VddVolFlag=OneSamGenFlag;
//	link_ok_flag=0;
	FunCtrlFlag = WorkingStatus;
	//InitalSpeechRam();
}

//*************************************************************************
extern Pair_Relevant_Var_T pair_var;
void GetLinkStatus(void)
{
	link_ok_flag = pair_var.pair_success_flag;
}

//*************************************************************************
//*************************************************************************
//功能：无操作5分钟进入睡眠，低电压关机后3S进入睡眠
//*************************************************************************
void NoActionCounter(void)
{
	if(VddVolFlag & LowBatOffFlag)
	{
		if(FunCtrlFlag == WorkingStatus)
		{
			FunCtrlFlag=0;
			u32NoActionCnt=0;
		}
		else {
			u32NoActionCnt++;
			//if((u32NoActionCnt > 100)&&(voic_handleflag==0))		//1S
			if(u32NoActionCnt > 100)		//1S
			{
				testPoint = 0x55;
//				while(1)
//				{
//					wd_32k_feed();
//					testPoint = 0x66;
//				}

				system_enter_sleep();
			}
		}
	}
	else {
		if((LFmotDuty | RBmotDuty | LBmotDuty | RFmotDuty) || (FunCtrlFlag&CarDemoFlag))
		{
			u32NoActionCnt=0;
			return;
		}

		if(FunCtrlFlag & WorkingStatus)
		{	//开机上电输出高电平,不操作5分钟关机
			u32NoActionCnt++;
			if(u32NoActionCnt > 30000)		//5分钟=300S 无操作关机
			{
				FunCtrlFlag=0;
				u32NoActionCnt=0;
				TurnOffPower();
			}
		}
		else {
			u32NoActionCnt++;
			//if((u32NoActionCnt > 100)&&(voic_handleflag==0))		//1S
			if((u32NoActionCnt > 100))		//1S
			{
				system_enter_sleep();
				testPoint = 0xFF;
			}
		}
	}
}

//*************************************************************************
//*************************************************************************
void system_enter_sleep(void)
{
	User_pwm_Exit_func();
    rf_set_tx_rx_off();
	User_hw_Exit();
	wd_32k_stop();
    //car_enter_deepsleep_no_retention();
	cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);
}

//*******************************************************************************************************
#if 0
//*******************************************************************************************************
////for ME4057AN8BG
//********************************************************************
//#define UsbPowIn_Pin		PA5		//Float,GPIO Input, 检测USB口电源适配器插入
#define Usb5Vin_Pin	UsbPowIn_Pin

unsigned char	ChargeFlag=0;
unsigned char	ChargeChkDbs;		//充电状态检测Debounce
unsigned char	ExitCharDbs;		//退出充电状态检测Debounce
unsigned char	ChrgDebounce;
//#define cHaveExPowBit	0		//有充电电源
//#define cRechargingBit	1		//=0为充电中，=1充满
//#define cRechargOKbit	2		//充电完成

#define 	cPowerInDebSet		10		//10*10=100mS
#define 	cPowerExitDebSet	100		//10*100=1000mS
//********************************************************************
//CHRG引脚需上拉，充电低电平，充满高电平
//功能：检测充电电源输入及充电IC状态
//********************************************************************
//********************************************************************
unsigned int	u32_ChargeInCnt;
//********************************************************************
void CheckChargeStatus(void)
{
	if(gpio_read(Usb5Vin_Pin))
	{
		ExitCharDbs=0;
		if((ChargeFlag & cHaveExPowBit)==0)
		{
			if(ChargeChkDbs < cPowerInDebSet)
			{
				ChargeChkDbs++;
			}
			else		//PowerInDbsOK
			{
				ChargeFlag |= cHaveExPowBit;
				ChargeChkDbs=0;
				//原为car_control(0,0,0)：dat1/dat2=0会被换算成"全速倒车+方向打满右"，
				//充电插入选保护应输出中点（停转+回中）
				car_control(0, RfDataMiddle, RfDataMiddle);
				//CHRG_Inital();
			}
		}
//		else		//PowerInNormal
//		{
//			if(u32_ChargeInCnt < 0xffffffff)
//			{
//				u32_ChargeInCnt++;
//			}
//
//			if(gpio_read(CHRG_Pin))		//充电中为低电平，充满为高电平
//			{
//				if(u32_ChargeInCnt == 0xFF)
//				{
//					if(ChargeFlag & cRechargingBit)
//					{
//						if(ChrgDebounce<25)
//						{
//							ChrgDebounce++;
//						}
//						else		//ChargedOk
//						{
//							ChargeFlag &= ~cRechargingBit;
//							ChargeFlag |= cRechargOKbit;
//							ChrgDebounce=0;
//						}
//					}
//					else
//					{
//						ChargeFlag |= cRechargOKbit;
//					}
//				}
//			}
//			else						//充电中
//			{
//				ChrgDebounce=0;
//				ChargeFlag |= cRechargingBit;
//				ChargeFlag &= ~cRechargOKbit;
//			}
//
//			if(u32_ChargeInCnt == 18000)		//3minutes
//			{
//				CLR_BIT(VddVolFlag, VddLow2950Flag);   // 清除
//				CLR_BIT(VddVolFlag, VddLow2800Flag);   // 清除
//				CLR_BIT(VddVolFlag, VddLow2600Flag);   // 清除
//			}
//		}
	}
	else		//NoPowerIn
	{
		u32_ChargeInCnt=0;
		ChargeChkDbs=0;
		if(ChargeFlag & cHaveExPowBit)
		{
			if(ExitCharDbs < cPowerExitDebSet)
			{
				ExitCharDbs++;
			}
			else
			{
				ExitCharDbs=0;
				ChargeFlag=0;
			}
		}
	}
}

//********************************************************************
void GetUsb5VinStatus(void)
{
    unsigned char  i = 100;
	ChargeChkDbs=0;
	while(i--)
	{
		if(gpio_read(Usb5Vin_Pin))
		{
			ChargeChkDbs++;
		}
		sleep_ms(1);
	}

	if(ChargeChkDbs > 60)
	{
		ChargeFlag |= cHaveExPowBit;
	}
	else
	{
		ChargeFlag &= ~cHaveExPowBit;
	}
}

//********************************************************************
#endif
//********************************************************************
//**************************************************************************
unsigned short CheckRxPotData(unsigned short data)
{
	if(data > RfDataMaxValue)
	{
		data=RfDataMaxValue;
	}
	else if(data < RfDataMinValue)
	{
		data=RfDataMinValue;
	}
	return data;
}

//*******************************************************************************************************
//*******************************************************************************************************
// ★ 2026/11/04 u8Divide 改为 unsigned short：油门分频改为 256 后，
//   unsigned char 形参会把 256 截断成 0（除零）——编译警告
//   "large integer implicitly truncated to unsigned type" 即由此而来。
signed short RxdDataConvert(unsigned short u16RxdDataIn, unsigned char u8Multiply, unsigned short u16Divide)
{
	int16_t s16_temper0;
	int32_t s32_temper1;

	s16_temper0 = CheckRxPotData(u16RxdDataIn);
	s16_temper0 -= RfDataMiddle;		//范围:	±3072
	s32_temper1	= s16_temper0;		//*u8Multiply;
	s16_temper0 = s32_temper1/u16Divide;

	return s16_temper0;
}

//**************************************************************************************************
//将收到的遥控器RF数据转换为
// Input: unsigned short g_Received_Direction
//			取值范围:0x0400~0x1000~0x1c00		左右各有3072级
//			Left:>0x1000 and Right:<0x1000
// Output: u16 ServoMotDuty 舵机的Duty值，取值范围:1000*6 ~ 2000*6, 中间值:1500*6
//**************************************************************************************************
//通道				功能			范围				中心值		映射目标
//Throttle			油门			0x400~0x1C00	0x1000		目标相对升降速度		±12cm/S
//Rotation			旋转			0x400~0x1C00	0x1000		目标对应偏航角速度 	±30°/S
//FBmove			前后平移		0x400~0x1C00	0x1000		目标俯仰角 ±30°
//LRmove			左右平移		0x400~0x1C00	0x1000		目标滚转角 ±30°
//*******************************************************************
//**************************************************************************************************
volatile unsigned short	u16Throttle_Rxd;		//Remotor speed data value for send
volatile unsigned short	u16Rotation_Rxd;		//Remotor rotation data value for send
volatile unsigned short	u16FBmove_Rxd;		//Remotor FBmove data value for send
volatile unsigned short	u16LRmove_Rxd;		//Remotor LRmove data value for send

unsigned short u16KeyPressed_Flag=0;	//RfSend_Keyval_def
unsigned short u16KeyAction_Flag=0;		//RfSend_Keyval_def

volatile int16_t s16_Throttle;		//目标相对升降速度	±12cm/S（满杆 1/8）, unit 1cm/S
//★ 2026/11/07 新增：高分辨率油门杆值 ±384（原始 ±3072/8）。
//	手动油门模式下直接用它的分辨率做推力映射（1 级 ≈ 0.36PWM），
//	s16_Throttle 只有 ±12 级，做手动推力映射太粗（1 级 = 11PWM）。
volatile int16_t s16_ThrottleFine;	//油门杆位置	±384（满行程）, unit 1/32 杆量
volatile int16_t s16_Rotation;		//目标对应偏航角速度 ±30°/S	=> ±300, unit 0.1°/S
volatile int16_t s16_FBmove;			//目标俯仰角 ±30°	=>	±300, unit 0.1°
volatile int16_t s16_LRmove;			//目标滚转角 ±30°	=>	±300, unit 0.1°

//RfSendKey_Xspeed=0x01,		//速度转换（高速/低速）
//RfSendKey_OneKRun=0x02,		//一键起降
//RfSendKey_StuntShow=0x04,	//特技表演
//RfSendKey_MicroAdj=0x08,	//微调模式
//RfSendKey_Headless=0x10,	//无头模式
//*******************************************************************
//生成马达控制用的Cmd2MotFlag和Cmd2MotDuty,RfDat2CtrlFlag
//1、处理遥控器上按键功能
//2、处理遥控器上4个电位器控制数据
//*******************************************************************
void Rxdata_Handler(void)
{
	if(VddVolFlag & LowBatOffFlag)
	{
		u16KeyAction_Flag=0;		//RfSend_Keyval_def
		u16KeyPressed_Flag=0;	//RfSend_Keyval_def
		FunCtrlFlag &= ~CarDemoFlag;
	}

	//目标相对升降速度	±12cm/S（2026/11/04 满杆由 96cm/S 降为 12cm/S = 原 1/8，
	//					真机起飞脉冲爬升疑似与目标升降速度过快有关，先降速观察）
	s16_Throttle = RxdDataConvert(u16Throttle_Rxd, 1, 256);	//±3072/256 => ±12

	//★ 高分辨率油门（手动油门模式用）：±3072/8 => ±384
	s16_ThrottleFine = RxdDataConvert(u16Throttle_Rxd, 1, 8);

	//目标对应偏航角速度 ±30°/S	=> ±300, unit 0.1°/S
	s16_Rotation = RxdDataConvert(u16Rotation_Rxd, 1, 10);	//±3072/10 => ±307

	//目标俯仰角 ±30°	=>	±300, unit 0.1°
	s16_FBmove = RxdDataConvert(u16FBmove_Rxd, 1, 10);	//±3072/10 => ±307

	//目标滚转角 ±30°	=>	±300, unit 0.1°
	s16_LRmove = RxdDataConvert(u16LRmove_Rxd, 1, 10);	//±3072/10 => ±307

	//Handle remotor keys, u16RemotorKeyVal

	if(u16RemotorKeyVal & RfSendKey_Xspeed)
	{	//低速/高速 调节按键
		if((u16KeyPressed_Flag & RfSendKey_Xspeed)==0)
		{
			u16KeyPressed_Flag |= RfSendKey_Xspeed;
			//切换车体低速/高速://: FunCtrlFlag.2; speed low=0; speed high=1;
			if(FunCtrlFlag & CarSpeedFlag)
			{
				FunCtrlFlag &= ~CarSpeedFlag;
			}
			else {
				FunCtrlFlag |= CarSpeedFlag;
			}
		}
	}
	else if(u16RemotorKeyVal & RfSendKey_OneKRun)
	{	//一键起降
		if((u16KeyPressed_Flag & RfSendKey_OneKRun)==0)
		{
			u16KeyPressed_Flag |= RfSendKey_OneKRun;
			//切换灯状态：
			if(FunCtrlFlag & CarOneKRun)
			{
				FunCtrlFlag &= ~CarOneKRun;
			}
			else {
				FunCtrlFlag |= CarOneKRun;
			}
		}
	}
	else if(u16RemotorKeyVal & RfSendKey_StuntShow)
	{	//特技表演
		if((u16KeyPressed_Flag & RfSendKey_StuntShow)==0)
		{
			u16KeyPressed_Flag |= RfSendKey_StuntShow;
			//★v25：暂时屏蔽特技表演真实功能，改为触发 flash 黑匣子读出
			//	（落地停机后按一下 → 从 UART 发出 0x10000 起的录制数据）
			{
				extern void BlackBox_DumpStart(void);
				BlackBox_DumpStart();
			}
		}
	}
	else if(u16RemotorKeyVal & RfSendKey_MicroAdj)
	{	//微调模式
		if((u16KeyPressed_Flag & RfSendKey_MicroAdj)==0)
		{
			u16KeyPressed_Flag |= RfSendKey_MicroAdj;
			//切换灯状态：
			if(FunCtrlFlag & CarMicroAdj)
			{
				FunCtrlFlag &= ~CarMicroAdj;
			}
			else {
				FunCtrlFlag |= CarMicroAdj;
			}
		}
	}
	else if(u16RemotorKeyVal & RfSendKey_Headless)
	{	//无头模式
		if((u16KeyPressed_Flag & RfSendKey_Headless)==0)
		{
			u16KeyPressed_Flag |= RfSendKey_Headless;
			//切换灯状态：
			if(FunCtrlFlag & CarHeadless)
			{
				FunCtrlFlag &= ~CarHeadless;
			}
			else {
				FunCtrlFlag |= CarHeadless;
			}
		}
	}
	else {
		u16KeyPressed_Flag = 0;
	}

}

//*************************************************************************
extern unsigned char UserRfRcvBuffer[20];		//RF收到的新数据
extern unsigned char RxdNewFlag;	//RF收到新数据的标志
unsigned short u16RemotorKeyVal=0;

//*************************************************************************
//UserRfRcvBuffer[8] = km_ptr->key;
//UserRfRcvBuffer[9] = km_ptr->cmd;		//0x55, 0xaa, ....
//UserRfRcvBuffer[10] = km_ptr->dat[0]	g_u8KeyVal_Buffer >> 8;	//真正的遥控键值数据高8位
//UserRfRcvBuffer[11] = km_ptr->dat[1]	g_u8KeyVal_Buffer;		//真正的遥控键值数据低8位
//UserRfRcvBuffer[12] = km_ptr->dat[2]	u16Throttle_Rxd >> 8;
//UserRfRcvBuffer[13] = km_ptr->dat[3]	u16Throttle_Rxd;
//UserRfRcvBuffer[14] = km_ptr->dat[4]	u16Rotation_Rxd >> 8;
//UserRfRcvBuffer[15] = km_ptr->dat[5]	u16Rotation_Rxd;
//UserRfRcvBuffer[16] = km_ptr->dat[6]	u16FBmove_Rxd >> 8;
//UserRfRcvBuffer[17] = km_ptr->dat[7]	u16FBmove_Rxd;
//UserRfRcvBuffer[18] = km_ptr->dat[8]	u16LRmove_Rxd >> 8;
//UserRfRcvBuffer[19] = km_ptr->dat[9]	u16LRmove_Rxd;
//*************************************************************************
#if 1
void car_work(void)
{
	if( RxdNewFlag)		//收到new RF数据
	{
		RxdNewFlag=0;
		u32NoRxdCnt = 0;
//		gpio_toggle(Testing_Pin);	//Test_Pin 翻转
		if(link_ok_flag == 0)
		{
			link_ok_flag=1;
#if (!C_ATT_BENCH_EN)
			get_GyroSensor_offset();		//获取Gyro传感器零偏（旧控制链，台架模式不调用）
#endif
			// ★v22: 黑匣子自动擦除已取消（v20/v21 擦除阻塞丢 RF 包致无法解锁），
			//   录制区改由用户烧录后用 BDT 手动擦除。
			return;
		}
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转
		rf_received_type=UserRfRcvBuffer[9];
		rf_received_key=UserRfRcvBuffer[8];
		//从UserRfRcvBuffer[10]开始是扩展数据
		u16RemotorKeyVal = (UserRfRcvBuffer[10]<<8) + UserRfRcvBuffer[11];
		u16Throttle_Rxd = (UserRfRcvBuffer[12]<<8) + UserRfRcvBuffer[13];
		u16Rotation_Rxd = (UserRfRcvBuffer[14]<<8) + UserRfRcvBuffer[15];
		u16FBmove_Rxd = (UserRfRcvBuffer[16]<<8) + UserRfRcvBuffer[17];
		u16LRmove_Rxd = (UserRfRcvBuffer[18]<<8) + UserRfRcvBuffer[19];

		Rxdata_Handler();
	}
	else
	{
		//失联保护（停转+回中）已移至CarCtrl_10ms()统一处理
		if(u32NoRxdCnt > 300)
		{
			link_ok_flag=0;
		}

	}
}

#endif

//*************************************************************************
#if 0
//*************************************************************************
unsigned char 	LedCtrlCnt, LedCtrlFlag, SwsCtrlCnt;
//*************************************************************************
#define cStateledBit		0x01
#define cLampledBit			0x02
#define LightStateled()		LedCtrlFlag |= cStateledBit
#define DarkStateled()		LedCtrlFlag &= ~cStateledBit
#define LightLampled()		LedCtrlFlag |= cLampledBit
#define DarkLampled()		LedCtrlFlag &= ~cLampledBit
#define cPairDispCycle	100
#define cPairDispDuty	50
#define cLvDispCycle	100
#define cLvDispDuty		10

#define cLedFlashLevMax		5
const unsigned char LedFlash_Dutytab[6] = {0xff, 50, 25, 16, 12, 10};	//Duty表格,0,1~5Hz
const unsigned char LedFlash_Cycletab[6] = {0xff, 100, 50, 33, 25, 20};	//Cycle表格,0,1~5Hz
//*************************************************************************
//*************************************************************************
void LedScanControl(void)		//2mS loop
{
	SwsCtrlCnt++;
	if(SwsCtrlCnt >= 3)	SwsCtrlCnt=0;
	 if(SwsCtrlCnt == 0)
	 {
			if(LedCtrlFlag & cStateledBit)
			{
				ChangePA3ToGPIO_OutLow();		//SWS, 车体前灯,Low active
			}
			else {
				ChangePA3ToSWS();
			}
	 }
	 else {
		 ChangePA3ToSWS();
	}
}

//*************************************************************************
//①RF连接前：该LED以1Hz频率、50% Duty闪烁
//②RF连接后：该LED常亮；
//低压提示功能：当电源电压在低压提示范围内时ＬＥＤ短快闪（1Hz 10% Duty），控制正常，提示更换电池；
//*************************************************************************
void PairingDisplay(void)		//配对显示：配对中闪烁，匹配完成常亮
{
	//状态灯
	if(VddVolFlag & LowBatWarnFlag)
	{
		if(LedCtrlCnt == 0)
		{
			LightStateled();
		}
		else if(LedCtrlCnt == cLvDispDuty)
		{
			DarkStateled();
		}
		LedCtrlCnt++;
		if(LedCtrlCnt > cLvDispCycle)
		{
			LedCtrlCnt=0;
		}
	}
	else {
		if(FunCtrlFlag & WorkingStatus)
		{
			if(link_ok_flag==0)
			{	//RF没有连接上, 1Hz flash
				if(LedCtrlCnt == 0)
				{
					LightStateled();
				}
				else if(LedCtrlCnt == cPairDispDuty)
				{
					DarkStateled();
				}
				LedCtrlCnt++;
				if(LedCtrlCnt > cPairDispCycle)
				{
					LedCtrlCnt=0;
				}
			}
			else {	//RF已连接上时常亮
				LightStateled();
			}
		}
		else {
			DarkStateled();
		}
	}

}

//*************************************************************************
//*************************************************************************
//演示程序
//*************************************************************************
//注：在中途时按其他按键会中断表演；
//*************************************************************************
#define cDemoDutyMax	16
#define cDemoDutyLimit	4
//*************************************************************************
unsigned char	DemoProcess, SubProcess;
unsigned short 	DemoRunCnt;
//*************************************************************************
//SubProcess=0	开始配置功能和时间
//SubProcess=2	执行功能并达到目标时间
//SubProcess=3	停止并达到目标时间
//SubProcess=4	结束本次功能并进入下一DemoProcess
//*************************************************************************

//*************************************************************************
//演示功能：
//1、 启动声（0x03）
//2、 鸣笛声（0x04），同时前进
//3、 放气
//4、 铃铛声（0x05）
//5、 哐当哐当声（0x07）
//6、 关机声（0x02）
//结束
//*************************************************************************
//演示控制函数，在系统主程序10mS loop中调用
//*************************************************************************
void CarDemoHandler(void)
{
	if(FunCtrlFlag & CarDemoFlag)
	{
		//u32NoRxdCnt = 0;		//清零此计数避免断连超时关机
		switch(DemoProcess)
		{
		case 0:
			DemoProcess=1;
			break;


		case 6:
			DemoProcess=0;
			break;

		case 7:		//结束
			DemoProcess=0;
			FunCtrlFlag &= ~CarDemoFlag;
			break;
		}
		//UpdateMotPwm();
	}
	else
	{
		DemoProcess=0;
		SubProcess=0;
		DemoRunCnt=0;
	}
}

//*************************************************************************
//演示程序结束
//*************************************************************************
#endif
//*************************************************************************
//*************************************************************************
void UserPowOnStart(void)
{
	InitalRam();		//初始化用户变量
//	GetUsb5VinStatus();
	GetBatteryVoltage();	//测出电源电压
#if (C_ATT_BENCH_EN)
	// 台架姿态估计：不初始化旧控制器/混控/气压采样，电机保持停转
	// （电机引脚在 UserPwmPinInital 里已经设为低电平输出）
#else
	User_pwm_init_func();
	StartPressureSample();
	FlightCtrl_Init();		//飞行姿态控制初始化（含高度零点）
#endif
}

//*****************************************************
u32 u32_TmBaseTick;
//*****************************************************
void TimeBaseGenerate(void)
{
	u32_SysTick++;		//可用于产生随机数等;

	if(clock_time_exceed(u32_TmBaseTick, 10000))	//10mS
	{
		u32_TmBaseTick = clock_time();
		u32_SysCount++;
		T10mSflag=1;
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转

		if(u32NoRxdCnt < 0xffffffff)
		{
			u32NoRxdCnt++;
		}

		u32_Sys10mScnt++;
		Sys10mScnt++;
		if(Sys10mScnt >= 10)		//100mS Counter
		{
			Sys10mScnt=0;
			T100mSflag = 1;
			Sys100mScnt++;
			if(Sys100mScnt >= 5)	//500mS Counter
			{
				Sys100mScnt=0;
				T500mSflag=1;

				if(u32_Sys500mScnt < 0xffffffff)
				{
					u32_Sys500mScnt++;
					if (0 == (u32_Sys500mScnt%2))
					{
						rc_24m_cal();
					}
				}
			}
		}
	}
}

void ClrCarRunPara(void)
{
	FunCtrlFlag &= ~(CarDemoFlag | CarOneKRun | CarMicroAdj);
	FlightCtrl_Reset();		//飞行控制：上锁停转
	LFmotDuty=0;
	RBmotDuty=0;
	LBmotDuty=0;
	RFmotDuty=0;
	//DemoProcess=0;
	//SubProcess=0;
	//DemoRunCnt=0;
}

//*******************************************************************
u32	u32_SampleTick;
//*******************************************************************
void PressureSampleTick(void)
{
	if(clock_time_exceed(u32_SampleTick, 15625))	//15625uS
	{
		//u32_SampleTick = clock_time();
		u32_SampleTick += 375000;		//15625*24=375,000
		//gpio_write(Testing_Pin, 1);
		Get_LPC2623_Value();
//		gpio_write(Testing_Pin, 0);
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转

//		UartPrintData(s32PresValue);

	}
}

void StartPressureSample(void)
{
	u32_SampleTick = clock_time();
	Get_LPC2623_Value();
}

//*****************************************************
//飞行姿态控制调度
//	每 FC_CTRL_DT_MS (mS) 执行一次 FlightCtrl_Tick()：
//	读陀螺仪 → 姿态解算 → 姿态/偏航/高度控制 → 混控 → 更新4路马达PWM
//	（原 10mS 的姿态解算已改为此节拍，飞控需要更快的控制周期）
//*****************************************************
u32	u32_FlightCtrlTick;
//*****************************************************
void FlightCtrlSchedule(void)
{
	if(clock_time_exceed(u32_FlightCtrlTick, ((u32)FC_CTRL_DT_MS)*1000))	//FC_CTRL_DT_MS mS
	{
		u32_FlightCtrlTick += ((u32)FC_CTRL_DT_MS) * 1000 * CLOCK_SYS_CLOCK_1US;
		//gpio_write(Testing_Pin, 1);
#if (PERF_MEAS_EN)
		// 计时：planned = 本轮计划启动 tick；实际启动/返回在 PerfMeas 内部取
		{
			static perf_tick_t s_perf_tick;
			PerfMeas_TaskBegin(&s_perf_tick, u32_FlightCtrlTick);
			FlightCtrl_Tick();
			PerfMeas_TaskEnd(&s_perf_tick);
		}
#else
		FlightCtrl_Tick();
#endif
		//gpio_write(Testing_Pin, 0);
	}
}

//*****************************************************
//for function control
//*****************************************************
//unsigned short u16KeyTxBuffer;	//待发送键值
unsigned char RxdStatus;
unsigned char u8T500mSCnt;
unsigned char u8StopStaCnt=0;
//unsigned char u8VoiceStChkDelayCnt=0;

//extern void PrepareQureyCmd(void);
//extern void PrepareRemotorCmd(void);
//*****************************************************
void RxdFunction_loop(void)
{
	if(T10mSflag)		//10mS
	{
		T10mSflag=0;
		//gpio_write(Testing_Pin, 1);

#if(UseTm0SoundCmd)
		if(u16VoiceSentCnt < 0xffff)
		{
			u16VoiceSentCnt++;
		}
		SendVoiceDelay();
#endif	//#if(UseTm0SoundCmd)

//		if(u8VoiceStChkDelayCnt)
//		{
//			u8VoiceStChkDelayCnt--;
//		}
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转
		//gpio_toggle(GPIO_PA5);	//Test_Pin 翻转

		//CarDemoHandler();	//10mS loop
		if(FunCtrlFlag & WorkingStatus)
		{
			//PairingDisplay();		//配对显示：配对中闪烁，匹配完成常亮
			//注：陀螺仪采集 + 姿态解算 + 姿态/高度控制已全部移入 FlightCtrl_Tick()，
			//    由下面的 FlightCtrlSchedule() 以 FC_CTRL_DT_MS(5mS) 节拍调用
		}

		//OverCurrentHandler();	//10mS loop


		if(VddVolFlag & LowBatOffFlag)
		{
			ClrCarRunPara();
			Update4MotPwmDuty();

		}

		//NoActionCounter();
		//gpio_write(Testing_Pin, 0);
	}

	if(T100mSflag)
	{
		T100mSflag=0;
		RxdStatus++;

	}

	if(T500mSflag)
	{
		T500mSflag=0;
		u8T500mSCnt++;
		GetBatteryVoltage();	//测出电源电压
//		if ((u8T500mSCnt%2)==0)
//		{
//			SendLedCtrlCmd();
//		}

		//MotorFlag = (u32_Sys500mScnt%256);
		//Uart_Send_Ctrl_Data();	//10mS loop
	}

	FlightCtrlSchedule();		//飞行姿态控制：FC_CTRL_DT_MS 节拍

	if((VddVolFlag & LowBatOffFlag)==0)
	{
		car_work();			//正常工作时RF接收处理程序
	}
}

//**************************************************************
// 台架姿态估计（原独立模块已合入本文件）
//   T-3488 直读 → FRD 映射与校准 → 加计一阶低通 → 四元数姿态 → 串口文本
//   不进入旧遥控/控制/混控/气压路径，电机保持停转
//**************************************************************
#define BENCH_TICKS_PER_US      (CLOCK_SYS_CLOCK_HZ / 1000000u)
#define BENCH_TICKS_PER_MS      (CLOCK_SYS_CLOCK_HZ / 1000u)
#define BENCH_TICKS_PER_100US   (CLOCK_SYS_CLOCK_HZ / 10000u)
#define BENCH_SAMPLE_TICKS      (CLOCK_SYS_CLOCK_HZ / C_ATT_SAMPLE_HZ)
#define BENCH_TX_TICKS          (CLOCK_SYS_CLOCK_HZ / C_ATT_TX_HZ)
#define BENCH_STAT_TICKS        (CLOCK_SYS_CLOCK_HZ / C_ATT_STAT_HZ)
#define BENCH_INIT_STILL_TICKS  ((u32)C_ATT_INIT_STILL_US * (CLOCK_SYS_CLOCK_HZ / 1000000u))

// 实测校准参数（来源：data/imu_axis_map_frd.json、data/accel_cal.json、data/gyro_bias_frd.json）
//   轴映射：v_frd = R × v_sensor（X=+X_s、Y=-Y_s、Z=-Z_s，行列式 +1）
//   加速度：FRD 零偏 LSB、比例 Q14；陀螺：FRD 零偏 LSB
// 轴映射（data/imu_axis_map_frd.json）：X_frd=+X_s、Y_frd=-Y_s、Z_frd=-Z_s
// 采集时按这个符号置换直接重排变号，不再做通用矩阵乘法
static const s32 s_bench_acc_off_lsb[3]     = { -3, -8, -29 };
static const s32 s_bench_acc_scale_q14[3]   = { 16436, 16429, 16357 };
static const s32 s_bench_gyro_bias_lsb[3]   = { -1, -3, -1 };

#define BENCH_MG_PER_LSB        1000        // 1g = 2048 LSB → mg = lsb*1000/2048
#define BENCH_LSB_PER_G         2048
#define BENCH_GYRO_Q10_NUM      17852       // 1.0896 × 16384：LSB → Q10 rad/s 的乘移系数

static u8   s_bench_inited;
static u32  s_bench_retry_tick;
static u32  s_bench_next_tick;
static u32  s_bench_last_ts;
static u8   s_bench_have_ts;
static u32  s_bench_dt_rem;             // tick 余数：0.1ms 换算不累积截断误差
static euler_q_t s_bench_att;
static s32  s_bench_lpf[3];
static u8   s_bench_lpf_ok;
static u32  s_bench_still_ticks;        // 启动判稳：已连续静止的 tick
static s32  s_bench_still_sum[3];
static u32  s_bench_still_n;
static s32  s_bench_prev_acc[3];
static u8   s_bench_prev_ok;
static u32  s_bench_wait_tick;
static u32  s_bench_tx_tick;
static u32  s_bench_stat_tick;

static struct {
	u32 valid, fail, gap;              // 有效样本 / I2C 读失败 / >100ms 长间断
	u16 dt_min_us, dt_max_us;          // 相邻有效样本间隔（原始时钟差换算）
	u16 rd_min_us, rd_max_us;          // I2C 读取耗时
	u16 up_min_us, up_max_us;          // 预处理 + 姿态更新耗时
	u16 cyc_min_us, cyc_max_us;        // 整拍耗时（含本拍串口格式化与提交 DMA）
	u32 rd_sum_us, up_sum_us, cyc_sum_us;
	u32 miss;                          // 整拍超过标称采样周期（截止超期）次数
	u32 skip;                          // 调度落后时跳过的采样节拍数
	u32 euler_ok;                      // Euler_Update 成功次数
} s_bench_stat;

// 加速度一阶低通系数缓存（Q15）：α 只与 dt 有关，dt 变化超过 50us 才重算
static att_i32 s_bench_lpf_alpha;
static u32     s_bench_lpf_alpha_dt_us;

// 文本格式化（不用 printf / 浮点）
static char *bench_put_str(char *p, const char *s)
{
	while (*s) *p++ = *s++;
	return p;
}

static char *bench_put_u32(char *p, u32 v)
{
	char tmp[10];
	u8 n = 0;

	if (v == 0) {
		*p++ = '0';
		return p;
	}
	while (v && n < sizeof(tmp)) {
		tmp[n++] = (char)('0' + (v % 10));
		v /= 10;
	}
	while (n) *p++ = tmp[--n];
	return p;
}

// 输出带符号定点数：v 的单位是 0.01
static char *bench_put_fixed2(char *p, s32 v)
{
	u32 a;

	if (v < 0) {
		*p++ = '-';
		a = (u32)(-v);
	} else {
		*p++ = '+';
		a = (u32)v;
	}
	p = bench_put_u32(p, a / 100);
	*p++ = '.';
	*p++ = (char)('0' + ((a % 100) / 10));
	*p++ = (char)('0' + (a % 10));
	return p;
}

// 先在独立缓冲里拼好，再由 UartTrySend 在确认空闲后复制到 DMA 缓冲
static void bench_send(char *buf, int len)
{
	(void)UartTrySend((unsigned char *)buf, (u8)len);
}

static void bench_still_reset(void)
{
	s_bench_still_ticks = 0;
	s_bench_still_n = 0;
	s_bench_still_sum[0] = 0;
	s_bench_still_sum[1] = 0;
	s_bench_still_sum[2] = 0;
}

// 传感器初始化（含陀螺片内低通写后回读）
static void BenchAtt_Init(void)
{
	u32 now = clock_time();
	u8 lpf_rb = 0;
	u8 st;
	char buf[160];
	char *q;

	if ((s32)(now - s_bench_retry_tick) < 0) {
		return;
	}
	s_bench_retry_tick = now + CLOCK_SYS_CLOCK_HZ;      // 失败后 1s 重试

	st = t3488_bench_init(C_GYRO_LPF_SEL, &lpf_rb);
	if (st) {
		q = buf;
		q = bench_put_str(q, "ATT init fail st=");
		q = bench_put_u32(q, st);
		q = bench_put_str(q, " lpf_rb=");
		q = bench_put_u32(q, lpf_rb);
		*q++ = '\n';
		bench_send(buf, (int)(q - buf));
		return;
	}

	q = buf;
	q = bench_put_str(q, "ATT init ok: T3488 16g/2000dps 800Hz, gyro_lpf_sel=");
	q = bench_put_u32(q, C_GYRO_LPF_SEL);
	q = bench_put_str(q, " (ODR/16, ~");
	q = bench_put_u32(q, C_GYRO_LPF_HZ_NOMINAL);
	q = bench_put_str(q, "Hz), readback=");
	q = bench_put_u32(q, lpf_rb);
	*q++ = '\n';
	bench_send(buf, (int)(q - buf));

	Euler_Reset(&s_bench_att);
	s_bench_lpf_ok = 0;
	s_bench_have_ts = 0;
	s_bench_dt_rem = 0;
	s_bench_prev_ok = 0;
	bench_still_reset();
	s_bench_stat.valid = s_bench_stat.fail = s_bench_stat.gap = 0;
	s_bench_stat.dt_min_us = 0xFFFF; s_bench_stat.dt_max_us = 0;
	s_bench_stat.rd_min_us = 0xFFFF; s_bench_stat.rd_max_us = 0; s_bench_stat.rd_sum_us = 0;
	s_bench_stat.up_min_us = 0xFFFF; s_bench_stat.up_max_us = 0; s_bench_stat.up_sum_us = 0;
	s_bench_stat.cyc_min_us = 0xFFFF; s_bench_stat.cyc_max_us = 0; s_bench_stat.cyc_sum_us = 0;
	s_bench_stat.miss = 0;
	s_bench_stat.skip = 0;
	s_bench_stat.euler_ok = 0;
	s_bench_lpf_alpha = 0;
	s_bench_lpf_alpha_dt_us = 0;

	s_bench_next_tick = clock_time() + BENCH_SAMPLE_TICKS;
	s_bench_inited = 1;
}

// 启动阶段：用一段连续静止样本的平均加速度定姿（移动/间断就重开窗口）
static void BenchAtt_Startup(att_i32 acc_mg[3], att_i32 gyr_q10[3], u32 dt_ticks, u32 ts)
{
	u8 still;
	int i;

	still = Euler_IsStill(acc_mg, gyr_q10, s_bench_prev_acc, s_bench_prev_ok);

	if (!still) {
		bench_still_reset();
	} else {
		s_bench_still_sum[0] += acc_mg[0];
		s_bench_still_sum[1] += acc_mg[1];
		s_bench_still_sum[2] += acc_mg[2];
		s_bench_still_n++;
		s_bench_still_ticks += dt_ticks;

		if (s_bench_still_ticks >= BENCH_INIT_STILL_TICKS && s_bench_still_n > 0) {
			att_i32 avg[3];
			avg[0] = s_bench_still_sum[0] / (att_i32)s_bench_still_n;
			avg[1] = s_bench_still_sum[1] / (att_i32)s_bench_still_n;
			avg[2] = s_bench_still_sum[2] / (att_i32)s_bench_still_n;
			if (Euler_InitFromAccel(&s_bench_att, avg, ts / BENCH_TICKS_PER_MS)) {
				att_i32 r, p, y;
				char buf[96];
				char *q;

				Euler_GetEulerDeg100(&s_bench_att, &r, &p, &y);
				q = buf;
				q = bench_put_str(q, "ATT ready: r=");
				q = bench_put_fixed2(q, r);
				q = bench_put_str(q, " p=");
				q = bench_put_fixed2(q, p);
				q = bench_put_str(q, " (still ");
				q = bench_put_u32(q, s_bench_still_ticks / BENCH_TICKS_PER_MS);
				q = bench_put_str(q, "ms)\n");
				bench_send(buf, (int)(q - buf));
			} else {
				bench_still_reset();
			}
		}
	}

	for (i = 0; i < 3; i++) {
		s_bench_prev_acc[i] = acc_mg[i];
	}
	s_bench_prev_ok = 1;
}

// 台架主处理：主循环每圈调用一次，内部按 200Hz 采样
static void BenchAtt_Process(void)
{
	u32 now;
	int16_t raw_acc[3], raw_gyr[3];
	att_i32 acc_mg[3], gyr_q10[3];
	att_i32 acc_frd[3], gyr_frd[3];
	u32 dt_ticks = 0;
	att_i32 dt_100us = 0;
	u8 st;
	u32 t_read0, t_read1, t_up0, t_up1, t_cyc0;
	int i;

	if (!s_bench_inited) {
		BenchAtt_Init();
		return;
	}

	now = clock_time();
	if ((s32)(now - s_bench_next_tick) < 0) {
		return;
	}
	s_bench_next_tick += BENCH_SAMPLE_TICKS;
	if ((s32)(clock_time() - s_bench_next_tick) > 0) {
		// 落后了：算清跳过了几个采样节拍再重排下一拍
		u32 late_ticks = clock_time() - s_bench_next_tick;
		s_bench_stat.skip += late_ticks / BENCH_SAMPLE_TICKS + 1;
		s_bench_next_tick = clock_time() + BENCH_SAMPLE_TICKS;
	}

	// ---- 1) 一次连续读 12 字节（整拍计时从读之前开始）----
	t_read0 = clock_time();
	t_cyc0 = t_read0;
	st = t3488_read_6axis(raw_acc, raw_gyr);
	t_read1 = clock_time();
	{
		u16 us = (u16)((t_read1 - t_read0) / BENCH_TICKS_PER_US);
		if (us < s_bench_stat.rd_min_us) s_bench_stat.rd_min_us = us;
		if (us > s_bench_stat.rd_max_us) s_bench_stat.rd_max_us = us;
		s_bench_stat.rd_sum_us += us;
	}
	if (st) {
		// 读失败：不发布零值、不更新估计，只累加错误计数
		s_bench_stat.fail++;
		return;
	}

	// ---- 2) 预处理 + 姿态更新（计时）----
	t_up0 = clock_time();

	// 映射到机体 FRD：实测映射就是符号置换（X_frd=+X_s、Y_frd=-Y_s、Z_frd=-Z_s），
	// 直接重排变号，省掉每拍 18 次乘加
	acc_frd[0] =  (att_i32)raw_acc[0];
	acc_frd[1] = -(att_i32)raw_acc[1];
	acc_frd[2] = -(att_i32)raw_acc[2];
	gyr_frd[0] =  (att_i32)raw_gyr[0];
	gyr_frd[1] = -(att_i32)raw_gyr[1];
	gyr_frd[2] = -(att_i32)raw_gyr[2];
	for (i = 0; i < 3; i++) {
		att_i32 lsb = (acc_frd[i] - s_bench_acc_off_lsb[i]) * s_bench_acc_scale_q14[i] >> 14;
		// LSB → mg：1000/2048 = 125/256，用乘移代替整除
		acc_mg[i] = (lsb * 125) >> 8;

		lsb = gyr_frd[i] - s_bench_gyro_bias_lsb[i];
		// LSB → Q10 rad/s：1.0896 ≈ 17852/16384，同样用乘移代替整除
		gyr_q10[i] = (lsb * BENCH_GYRO_Q10_NUM) >> 14;
	}

	// ---- 3) 时间差（u32 相减，天然处理回绕）----
	if (!s_bench_have_ts) {
		s_bench_last_ts = t_read1;
		s_bench_have_ts = 1;
		dt_ticks = 0;
	} else {
		dt_ticks = t_read1 - s_bench_last_ts;
		s_bench_last_ts = t_read1;
	}
	if (dt_ticks) {
		// STA 用原始时钟差直接换算成 µs（四舍五入），不做 100µs 截断
		u16 us = (u16)((dt_ticks + BENCH_TICKS_PER_US / 2) / BENCH_TICKS_PER_US);
		if (us < s_bench_stat.dt_min_us) s_bench_stat.dt_min_us = us;
		if (us > s_bench_stat.dt_max_us) s_bench_stat.dt_max_us = us;
	}

	if (dt_ticks > (u32)C_ATT_MAX_GAP_100US * BENCH_TICKS_PER_100US) {
		// 长间断：重建积分端点，不补算缺失区间
		s_bench_stat.gap++;
		s_bench_dt_rem = 0;
		s_bench_att.have_prev = 0;
		bench_still_reset();
		dt_ticks = 0;
	}

	if (dt_ticks) {
		// 积分用的 dt：0.1ms 为单位，tick 余数留到下一拍，避免每拍向下截断
		s_bench_dt_rem += dt_ticks;
		dt_100us = (att_i32)(s_bench_dt_rem / BENCH_TICKS_PER_100US);
		s_bench_dt_rem -= (u32)dt_100us * BENCH_TICKS_PER_100US;
	}

	// ---- 4) 加速度一阶低通（FRD 校准之后）----
	if (!s_bench_lpf_ok) {
		s_bench_lpf[0] = acc_mg[0];
		s_bench_lpf[1] = acc_mg[1];
		s_bench_lpf[2] = acc_mg[2];
		s_bench_lpf_ok = 1;
	} else if (C_ACC_LPF_EN && dt_ticks > 0) {
		u32 dt_us = (dt_ticks + BENCH_TICKS_PER_US / 2) / BENCH_TICKS_PER_US;
		att_i32 alpha;

		// α 只跟 dt 有关：变化超过 50us 才重算，正常节拍下不用每拍做除法
		if (dt_us + 50 < s_bench_lpf_alpha_dt_us
		 || dt_us > s_bench_lpf_alpha_dt_us + 50) {
			s_bench_lpf_alpha = (att_i32)((dt_us * 32768) / (C_ACC_LPF_TAU_US + dt_us));
			s_bench_lpf_alpha_dt_us = dt_us;
		}
		alpha = s_bench_lpf_alpha;
		for (i = 0; i < 3; i++) {
			s_bench_lpf[i] += (alpha * (acc_mg[i] - s_bench_lpf[i])) >> 15;
		}
	} else if (!C_ACC_LPF_EN) {
		s_bench_lpf[0] = acc_mg[0];
		s_bench_lpf[1] = acc_mg[1];
		s_bench_lpf[2] = acc_mg[2];
	}

	// ---- 5) 启动判稳 / 姿态更新 ----
	if (!s_bench_att.ready) {
		BenchAtt_Startup(s_bench_lpf, gyr_q10, dt_ticks, t_read1);

		if ((u32)(now - s_bench_wait_tick) >= BENCH_TICKS_PER_MS * 500) {
			char buf[96];
			char *q;

			s_bench_wait_tick = now;
			q = buf;
			q = bench_put_str(q, "ATT wait still ");
			q = bench_put_fixed2(q, (att_i32)(s_bench_still_ticks / (CLOCK_SYS_CLOCK_HZ / 100)));
			q = bench_put_str(q, "/");
			q = bench_put_fixed2(q, (att_i32)(C_ATT_INIT_STILL_US / 10000));
			q = bench_put_str(q, "s\n");
			bench_send(buf, (int)(q - buf));
		}
	} else if (dt_100us > 0) {
		if (Euler_Update(&s_bench_att, s_bench_lpf, gyr_q10, dt_100us,
		                 t_read1 / BENCH_TICKS_PER_MS)) {
			s_bench_stat.euler_ok++;          // 实际完成四元数更新的次数
		}
	}

	t_up1 = clock_time();
	{
		u16 us = (u16)((t_up1 - t_up0) / BENCH_TICKS_PER_US);
		if (us < s_bench_stat.up_min_us) s_bench_stat.up_min_us = us;
		if (us > s_bench_stat.up_max_us) s_bench_stat.up_max_us = us;
		s_bench_stat.up_sum_us += us;
	}
	s_bench_stat.valid++;

	// ---- 6) 串口输出（在估计更新之外，先拼到独立缓冲）----
	now = clock_time();
	if (s_bench_att.ready && (u32)(now - s_bench_tx_tick) >= BENCH_TX_TICKS) {
		char buf[96];
		char *q;
		att_i32 roll, pitch, yaw;

		s_bench_tx_tick = now;
		Euler_GetEulerDeg100(&s_bench_att, &roll, &pitch, &yaw);

		q = buf;
		q = bench_put_str(q, "ATT t=");
		q = bench_put_u32(q, now / BENCH_TICKS_PER_MS);
		q = bench_put_str(q, " rdy=");
		q = bench_put_u32(q, s_bench_att.ready);
		q = bench_put_str(q, " r=");
		q = bench_put_fixed2(q, roll);
		q = bench_put_str(q, " p=");
		q = bench_put_fixed2(q, pitch);
		q = bench_put_str(q, " y=");
		q = bench_put_fixed2(q, yaw);
		*q++ = '\n';
		bench_send(buf, (int)(q - buf));
	}

	if ((u32)(now - s_bench_stat_tick) >= BENCH_STAT_TICKS) {
		char buf[160];
		char *q;

		s_bench_stat_tick = now;
		q = buf;
		q = bench_put_str(q, "STA n=");
		q = bench_put_u32(q, s_bench_stat.valid);
		q = bench_put_str(q, " f=");
		q = bench_put_u32(q, s_bench_stat.fail);
		q = bench_put_str(q, " g=");
		q = bench_put_u32(q, s_bench_stat.gap);
		q = bench_put_str(q, " dt=");
		q = bench_put_u32(q, s_bench_stat.dt_min_us == 0xFFFF ? 0 : s_bench_stat.dt_min_us);
		*q++ = '/';
		q = bench_put_u32(q, s_bench_stat.dt_max_us);
		q = bench_put_str(q, " rd=");
		q = bench_put_u32(q, s_bench_stat.valid ? (s_bench_stat.rd_sum_us / s_bench_stat.valid) : 0);
		*q++ = '/';
		q = bench_put_u32(q, s_bench_stat.rd_max_us);
		q = bench_put_str(q, " up=");
		q = bench_put_u32(q, s_bench_stat.valid ? (s_bench_stat.up_sum_us / s_bench_stat.valid) : 0);
		*q++ = '/';
		q = bench_put_u32(q, s_bench_stat.up_max_us);
		// 以下为 400Hz 验收追加：整拍耗时 / 截止超期 / 跳过节拍 / 姿态更新次数
		q = bench_put_str(q, " cyc=");
		q = bench_put_u32(q, s_bench_stat.valid ? (s_bench_stat.cyc_sum_us / s_bench_stat.valid) : 0);
		*q++ = '/';
		q = bench_put_u32(q, s_bench_stat.cyc_max_us);
		q = bench_put_str(q, " miss=");
		q = bench_put_u32(q, s_bench_stat.miss);
		q = bench_put_str(q, " skip=");
		q = bench_put_u32(q, s_bench_stat.skip);
		q = bench_put_str(q, " eul=");
		q = bench_put_u32(q, s_bench_stat.euler_ok);
		*q++ = '\n';
		bench_send(buf, (int)(q - buf));
	}

	// ---- 7) 整拍耗时：从本次读取前到本拍串口格式化与提交 DMA 之后 ----
	{
		u32 t_cyc1 = clock_time();
		u16 us = (u16)((t_cyc1 - t_cyc0) / BENCH_TICKS_PER_US);

		if (us < s_bench_stat.cyc_min_us) s_bench_stat.cyc_min_us = us;
		if (us > s_bench_stat.cyc_max_us) s_bench_stat.cyc_max_us = us;
		s_bench_stat.cyc_sum_us += us;
		if (us > C_ATT_SAMPLE_US) {
			s_bench_stat.miss++;        // 整拍超过标称采样周期：截止超期
		}
	}
}

//**************************************************************
void User_main_loop_process(void)
{
	wd_32k_feed();

#if (C_ATT_BENCH_EN)
	// 台架姿态估计入口：只读 T-3488 + 串口文本输出
	// 不进入旧遥控解码、姿态控制、混控、气压高度等路径（电机保持停转）
	BenchAtt_Process();
	return;
#endif

	d24_rx_main_loop();

#if (BATT_CHECK_ENABLE)
	user_batt_check_proc();//batt check proc
#endif

#if (APP_TEST_ENABLE)
	// main loop code
	app_test_loop();
#endif

	TimeBaseGenerate();
	RxdFunction_loop();
	PressureSampleTick();

#if (PERF_MEAS_EN)
	// 性能统计帧：约 1S 一帧，尝试发送（TX 忙则丢弃计数），不等待、不 printf
	PerfMeas_Poll();
#endif
}

//*******************************************************************
#endif  //Project_XH261009_Rxd
//*******************************************************************

