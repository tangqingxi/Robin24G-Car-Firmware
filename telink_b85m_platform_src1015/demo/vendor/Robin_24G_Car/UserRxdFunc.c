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
#if (C_RC_LOOP_400_EN)
#include "L400Diag.h"
#include "L400Motor.h"
#endif
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
			//★v25：特技键改触发 flash 黑匣子读出（落地停机后按一下，
			//	从 UART 发出 0x10000 起的录制数据）。
			//	黑匣子属于旧控制固件（FlightCtrl.c），台架/维护/联调镜像都不编译，
			//	这里一并排除，否则联调镜像链接时会找不到符号。
#if (!C_ATT_BENCH_EN)
			{
				extern void BlackBox_DumpStart(void);
				BlackBox_DumpStart();
			}
#endif
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
#if (C_RC_LOOP_400_EN)
void RcLoop_OnPacket(void);
static void RcLoop_RecordRx(u32 ticks);
#endif
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
#if (!C_RC_LOOP_400_EN)
			return;
#endif
			// L400 不在这里丢弃重连后的第一包：0001/0002 可能正好是这一包，
			// 必须继续映射/校验并交给按键事件状态机。
		}
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转
#if (C_RC_LOOP_400_EN)
		// 无线收包处理计时起点：整包解码 + 映射 + 按键 + 时间戳
		u32 t_rx0 = clock_time();
#endif
		rf_received_type=UserRfRcvBuffer[9];
		rf_received_key=UserRfRcvBuffer[8];
		//从UserRfRcvBuffer[10]开始是扩展数据
		u16RemotorKeyVal = (UserRfRcvBuffer[10]<<8) + UserRfRcvBuffer[11];
		u16Throttle_Rxd = (UserRfRcvBuffer[12]<<8) + UserRfRcvBuffer[13];
		u16Rotation_Rxd = (UserRfRcvBuffer[14]<<8) + UserRfRcvBuffer[15];
		u16FBmove_Rxd = (UserRfRcvBuffer[16]<<8) + UserRfRcvBuffer[17];
		u16LRmove_Rxd = (UserRfRcvBuffer[18]<<8) + UserRfRcvBuffer[19];

#if (C_RC_LOOP_400_EN)
		// L400 新路径只保留有效整包解码后的映射、按键与时间戳；
		// 旧按键业务和重复摇杆换算不再进入这条路径。
		RcLoop_OnPacket();
		RcLoop_RecordRx(clock_time() - t_rx0);
#else
		Rxdata_Handler();
#endif
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

// 台架姿态估计整块只在姿态镜像里编译；维护镜像用 -DC_MAINT_269A_EN=1 构建，
// 这块整体不参与，省 Flash 也避免"定义了没人用"的告警。
#if (!C_MAINT_269A_EN) && (!C_RC_BRINGUP_EN)
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
// 陀螺零偏（机体 FRD，1/64 LSB）：实测 -1.4531 / -3.6250 / -5.9531 LSB
//   即 -93/64、-232/64、-381/64；保留小数 LSB，换算到 Q10 前扣掉，
//   避免整数截断带来的常值漂移
static const s32 s_bench_gyro_bias_q6[3]    = { -93, -232, -381 };

#define BENCH_MG_PER_LSB        1000        // 1g = 2048 LSB → mg = lsb*1000/2048
#define BENCH_LSB_PER_G         2048

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

// 分项计时：从姿态就绪开始统计 30 秒，按这一拍实际做了哪些文本格式化分类
//   0=普通拍（没有输出） 1=只处理 ATT 2=只处理 STA 3=ATT 和 STA 都处理
//   发送因串口忙被丢弃的拍也算"处理过"，因为格式化和提交都真的执行了
static struct {
	u32 cnt[4];            // 各分类的拍数
	u32 over[4];           // 各分类里整拍超过标称采样周期（2500us）的次数
	u16 max_us[4];         // 各分类的整拍最大耗时
} s_bench_cat;
static const char *s_bench_cat_name[4] = { " c0=", " c1=", " c2=", " c3=" };

// STA 单项耗时：从开始生成 STA 文本到尝试提交 DMA，窗口内累计
static u32 s_bench_sta_cnt;
static u32 s_bench_sta_sum_us;
static u16 s_bench_sta_max_us;

// 30 秒测量窗口：姿态就绪后开始累计；时间到就冻结快照，之后挑一个普通 STA
// 时隙用 STA2 顶替一次 STA；UART 忙就保留快照，下个时隙再试
#define C_ATT_CAT_WINDOW_MS   30000u

#define BENCH_CAT_IDLE        0        // 还没就绪，不统计
#define BENCH_CAT_MEASURING   1        // 正在统计
#define BENCH_CAT_FROZEN      2        // 已冻结，等待 STA2 时隙
#define BENCH_CAT_DONE        3        // STA2 已经发出，不再重复

static u8  s_bench_cat_state;
static u32 s_bench_cat_start_ms;       // 窗口起点（毫秒）

// STA2 时隙发送后的状态迁移：发送成功 -> DONE（不再重复）；
// UART 忙 -> 保持 FROZEN，下个 STA 时隙再试
static u8 bench_cat_after_send(u8 state, u8 sent_ok)
{
	if (state != BENCH_CAT_FROZEN) {
		return state;
	}
	return sent_ok ? BENCH_CAT_DONE : BENCH_CAT_FROZEN;
}

// 冻结后的分项快照（每次上电只做一轮 30 秒）
static struct {
	u32 cnt[4];
	u32 over[4];
	u16 max_us[4];
	u32 sta_cnt;
	u32 sta_sum_us;
	u16 sta_max_us;
} s_bench_cat_snap;

// 加速度一阶低通系数缓存（Q15）：α 只与 dt 有关，dt 变化超过 50us 才重算
static att_i32 s_bench_lpf_alpha;
static u32     s_bench_lpf_alpha_dt_us;

// 陀螺换算余数（每轴），把 Q10 小数部分带到下一拍
static att_i32 s_bench_gyro_rem[3];

// FRD Z 轴窗口均值（每个 STA 周期清零）：扣偏前用 LSB，扣偏后用 1/64 LSB
static s32 s_bench_z_pre_sum;
static s32 s_bench_z_post_sum;
static u32 s_bench_z_cnt;

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

// 输出定宽十六进制（用移位取每个字符，避免引入 printf）
//   v      : 数值（负数按补码输出）
//   digits : 固定宽度（1..8）
static char *bench_put_hex(char *p, u32 v, u8 digits)
{
	u8 i;

	for (i = digits; i > 0; i--) {
		u8 nib = (u8)((v >> ((u8)((i - 1) * 4))) & 0x0Fu);

		*p++ = (char)((nib < 10) ? ('0' + nib) : ('A' + nib - 10));
	}
	return p;
}

// 输出带符号整数（四元数分量用）
static char *bench_put_s32(char *p, s32 v)
{
	if (v < 0) {
		*p++ = '-';
		return bench_put_u32(p, (u32)(-v));
	}
	return bench_put_u32(p, (u32)v);
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
	s_bench_z_pre_sum = 0;
	s_bench_z_post_sum = 0;
	s_bench_z_cnt = 0;
	s_bench_gyro_rem[0] = 0;
	s_bench_gyro_rem[1] = 0;
	s_bench_gyro_rem[2] = 0;
	{
		u8 k;

		for (k = 0; k < 4; k++) {
			s_bench_cat.cnt[k] = 0;
			s_bench_cat.over[k] = 0;
			s_bench_cat.max_us[k] = 0;
			s_bench_cat_snap.cnt[k] = 0;
			s_bench_cat_snap.over[k] = 0;
			s_bench_cat_snap.max_us[k] = 0;
		}
	}
	s_bench_sta_cnt = 0;
	s_bench_sta_sum_us = 0;
	s_bench_sta_max_us = 0;
	s_bench_cat_snap.sta_cnt = 0;
	s_bench_cat_snap.sta_sum_us = 0;
	s_bench_cat_snap.sta_max_us = 0;
	s_bench_cat_state = BENCH_CAT_IDLE;
	s_bench_cat_start_ms = 0;

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
				char buf[64];
				char *q;

				// 这里不再算欧拉角：姿态角统一由上位机从四元数换算
				q = buf;
				q = bench_put_str(q, "ATT ready (still ");
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
	u8 did_att = 0;          // 本拍是否真的拼了 ATT 文本
	u8 did_sta = 0;          // 本拍是否真的拼了 STA 文本
	int i;

	if (!s_bench_inited) {
		BenchAtt_Init();
		return;
	}

	now = clock_time();
	if ((s32)(now - s_bench_next_tick) < 0) {
		return;                         // 还没到下一拍
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

		// 陀螺：按 1/64 LSB 扣零偏后换算到 Q10（乘移 + 对称舍入）
		gyr_q10[i] = Euler_GyroToQ10(gyr_frd[i], s_bench_gyro_bias_q6[i],
		                             &s_bench_gyro_rem[i]);
	}

	// FRD Z 轴窗口均值：扣偏前（LSB）与扣偏后（1/64 LSB），供静置漂移核对
	s_bench_z_pre_sum += gyr_frd[2];
	s_bench_z_post_sum += (((att_i32)gyr_frd[2] << 6) - s_bench_gyro_bias_q6[2]);
	s_bench_z_cnt++;

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
		char buf[80];
		char *q;

		s_bench_tx_tick = now;
		did_att = 1;

		// 只发 Q15 四元数（w/x/y/z，32768 = 1.0），欧拉角交给上位机换算，
		// 这样 14Hz 输出这一拍不做 atan2，也不会挤占下一拍的采样预算
		q = buf;
		q = bench_put_str(q, "ATT t=");
		q = bench_put_u32(q, now / BENCH_TICKS_PER_MS);
		q = bench_put_str(q, " rdy=1 q=");
		q = bench_put_s32(q, s_bench_att.q[0]);
		*q++ = '/';
		q = bench_put_s32(q, s_bench_att.q[1]);
		*q++ = '/';
		q = bench_put_s32(q, s_bench_att.q[2]);
		*q++ = '/';
		q = bench_put_s32(q, s_bench_att.q[3]);
		*q++ = '\n';
		bench_send(buf, (int)(q - buf));
	}

	if ((u32)(now - s_bench_stat_tick) >= BENCH_STAT_TICKS) {
		s_bench_stat_tick = now;
		did_sta = 1;

		if (s_bench_cat_state == BENCH_CAT_FROZEN) {
			// 30 秒窗口已结束：用 STA2 顶替这一次 STA，只发一次；
			// 串口忙（UartTrySend 返回 0）就保留快照，下个时隙再来
			char buf2[128];
			char *r2 = buf2;
			u8 k;

			r2 = bench_put_str(r2, "STA2");
			for (k = 0; k < 4; k++) {
				r2 = bench_put_str(r2, s_bench_cat_name[k]);
				r2 = bench_put_u32(r2, s_bench_cat_snap.cnt[k]);
				*r2++ = '/';
				r2 = bench_put_u32(r2, s_bench_cat_snap.max_us[k]);
				*r2++ = '/';
				r2 = bench_put_u32(r2, s_bench_cat_snap.over[k]);
			}
			r2 = bench_put_str(r2, " sta=");
			r2 = bench_put_u32(r2, s_bench_cat_snap.sta_cnt
			                       ? (s_bench_cat_snap.sta_sum_us / s_bench_cat_snap.sta_cnt) : 0);
			*r2++ = '/';
			r2 = bench_put_u32(r2, s_bench_cat_snap.sta_max_us);
			// 单次输出没有"上一回发送"可参考，诊断字段保留 0，格式与上位机解析不变
			r2 = bench_put_str(r2, " diag=0 mrg=0 lat=0");
			*r2++ = '\n';

			// 发送成功就标记完成，忙则保持冻结状态，下个 STA 时隙重试
			s_bench_cat_state = bench_cat_after_send(
			    s_bench_cat_state,
			    (u8)(UartTrySend((unsigned char *)buf2, (u8)(r2 - buf2)) ? 1 : 0));
		} else {
			char buf[160];
			char *q;
			u32 t_sta0 = clock_time();       // STA 单独计时的起点：开始拼文本

			// 定宽十六进制：板端只发原始计数、累加值和峰值，平均耗时与
			// Z 轴均值都交给上位机算；字段宽度固定，最长约 148 字节，
			// 能放进 UART_DATA_LEN（160）缓冲区
			q = buf;
			q = bench_put_str(q, "STA n=");
			q = bench_put_hex(q, s_bench_stat.valid, 6);
			q = bench_put_str(q, " f=");
			q = bench_put_hex(q, s_bench_stat.fail, 4);
			q = bench_put_str(q, " g=");
			q = bench_put_hex(q, s_bench_stat.gap, 4);
			q = bench_put_str(q, " dt=");
			q = bench_put_hex(q, (s_bench_stat.dt_min_us == 0xFFFF ? 0 : s_bench_stat.dt_min_us), 4);
			*q++ = '/';
			q = bench_put_hex(q, s_bench_stat.dt_max_us, 4);
			q = bench_put_str(q, " rd=");
			q = bench_put_hex(q, s_bench_stat.rd_sum_us, 8);
			*q++ = '/';
			q = bench_put_hex(q, s_bench_stat.rd_max_us, 4);
			q = bench_put_str(q, " up=");
			q = bench_put_hex(q, s_bench_stat.up_sum_us, 8);
			*q++ = '/';
			q = bench_put_hex(q, s_bench_stat.up_max_us, 4);
			q = bench_put_str(q, " cyc=");
			q = bench_put_hex(q, s_bench_stat.cyc_sum_us, 8);
			*q++ = '/';
			q = bench_put_hex(q, s_bench_stat.cyc_max_us, 4);
			q = bench_put_str(q, " miss=");
			q = bench_put_hex(q, s_bench_stat.miss, 6);
			q = bench_put_str(q, " skip=");
			q = bench_put_hex(q, s_bench_stat.skip, 6);
			q = bench_put_str(q, " eul=");
			q = bench_put_hex(q, s_bench_stat.euler_ok, 6);

			// Z 轴：这一秒的累加值（扣偏前 LSB / 扣偏后 1/64 LSB）与样本数，
			// 均值由上位机算；写完清零，下一行重新累计
			q = bench_put_str(q, " z=");
			q = bench_put_hex(q, (u32)s_bench_z_pre_sum, 8);
			*q++ = '/';
			q = bench_put_hex(q, (u32)s_bench_z_post_sum, 8);
			*q++ = '/';
			q = bench_put_hex(q, s_bench_z_cnt, 6);
			s_bench_z_pre_sum = 0;
			s_bench_z_post_sum = 0;
			s_bench_z_cnt = 0;

			*q++ = '\n';
			bench_send(buf, (int)(q - buf));

			// STA 从开始拼文本到尝试提交 DMA 的耗时（窗口内累计）
			{
				u32 us = (clock_time() - t_sta0) / BENCH_TICKS_PER_US;

				if (us > s_bench_sta_max_us) s_bench_sta_max_us = (u16)us;
				s_bench_sta_sum_us += us;
				s_bench_sta_cnt++;
			}
		}
	}

	// ---- 7) 整拍耗时：从本次读取前到本拍串口格式化与提交 DMA 之后 ----
	{
		u32 t_cyc1 = clock_time();
		u16 us = (u16)((t_cyc1 - t_cyc0) / BENCH_TICKS_PER_US);
		u8 cat = (u8)((did_att ? 1 : 0) | (did_sta ? 2 : 0));

		// 姿态就绪后开始 30 秒测量窗口
		if (s_bench_cat_state == BENCH_CAT_IDLE && s_bench_att.ready) {
			s_bench_cat_state = BENCH_CAT_MEASURING;
			s_bench_cat_start_ms = (u32)(now / BENCH_TICKS_PER_MS);
		}

		// 窗口内按拍累计：拍数、该分类整拍最大耗时、该分类超过 2500us 的次数
		if (s_bench_cat_state == BENCH_CAT_MEASURING) {
			s_bench_cat.cnt[cat]++;
			if (us > s_bench_cat.max_us[cat]) s_bench_cat.max_us[cat] = us;
			if (us > C_ATT_SAMPLE_US) s_bench_cat.over[cat]++;

			// 30 秒到点：把累计结果整体冻结，等 STA 时隙输出 STA2
			if ((u32)((u32)(now / BENCH_TICKS_PER_MS) - s_bench_cat_start_ms)
			        >= C_ATT_CAT_WINDOW_MS) {
				u8 k;

				for (k = 0; k < 4; k++) {
					s_bench_cat_snap.cnt[k] = s_bench_cat.cnt[k];
					s_bench_cat_snap.max_us[k] = s_bench_cat.max_us[k];
					s_bench_cat_snap.over[k] = s_bench_cat.over[k];
				}
				s_bench_cat_snap.sta_cnt = s_bench_sta_cnt;
				s_bench_cat_snap.sta_sum_us = s_bench_sta_sum_us;
				s_bench_cat_snap.sta_max_us = s_bench_sta_max_us;
				s_bench_cat_state = BENCH_CAT_FROZEN;
			}
		}

		if (us < s_bench_stat.cyc_min_us) s_bench_stat.cyc_min_us = us;
		if (us > s_bench_stat.cyc_max_us) s_bench_stat.cyc_max_us = us;
		s_bench_stat.cyc_sum_us += us;
		if (us > C_ATT_SAMPLE_US) {
			s_bench_stat.miss++;        // 整拍超过标称采样周期：截止超期
		}
	}
}

#endif /* !C_MAINT_269A_EN */

#if (C_MAINT_269A_EN)
// =====================================================================================
// 地面维护模式：只把 T-3488 的原始六轴计数打成 0x269A 帧（约 100Hz）
//   * 取的是传感器原坐标系、轴映射/扣偏/软件滤波之前的原始计数；
//   * 帧里的零偏字段填 0，flags 也是 0（未解锁、未扣偏、未软件滤波）；
//   * 轴映射与加计校准都由上位机脚本完成，固件不参与；
//   * 电机保持停转：维护模式不初始化 PWM，也不进旧控制链。
// =====================================================================================
#define MAINT_SAMPLE_HZ        100
#define MAINT_SAMPLE_TICKS     (CLOCK_SYS_CLOCK_HZ / MAINT_SAMPLE_HZ)

static u8  s_maint_inited;
static u32 s_maint_retry_tick;
static u32 s_maint_next_tick;

// 与台架固件完全相同的传感器初始化：±16g / ±2000dps / 800Hz ODR / 片内低通
static void Maint_Init(void)
{
	u32 now = clock_time();
	u8 lpf_rb = 0;

	if ((s32)(now - s_maint_retry_tick) < 0) {
		return;
	}
	s_maint_retry_tick = now + CLOCK_SYS_CLOCK_HZ;      // 失败后 1s 重试

	if (t3488_bench_init(C_GYRO_LPF_SEL, &lpf_rb)) {
		return;
	}
	s_maint_next_tick = clock_time() + MAINT_SAMPLE_TICKS;
	s_maint_inited = 1;
}

static void Maint_Process(void)
{
	static const s16 zero_bias[3] = { 0, 0, 0 };
	int16_t raw_acc[3], raw_gyr[3];
	u32 now;

	if (!s_maint_inited) {
		Maint_Init();
		return;
	}

	now = clock_time();
	if ((s32)(now - s_maint_next_tick) < 0) {
		return;
	}
	s_maint_next_tick += MAINT_SAMPLE_TICKS;
	if ((s32)(clock_time() - s_maint_next_tick) > 0) {
		s_maint_next_tick = clock_time() + MAINT_SAMPLE_TICKS;   // 落后就重排
	}

	// 一次连续读 12 字节：原坐标系、未映射、未扣偏、未滤波
	if (t3488_read_6axis(raw_acc, raw_gyr)) {
		return;                     // 读失败不发帧，避免把 0 当成数据送去标定
	}

	// 0x269A：flags=0（未解锁/未扣偏/未软件滤波），零偏字段填 0
	UartSensorFrameSend((const s16 *)raw_acc, (const s16 *)raw_gyr, zero_bias, 0);
}
#endif /* C_MAINT_269A_EN */

#if (C_RC_BRINGUP_EN)
// =====================================================================================
// 遥控输入联调：只做 2.4G 收包 → 通道解码 → 映射 → 低频串口观察，电机保持停转
//   无线报文与配对沿用现有流程（d24_rx_main_loop / car_work / Rxdata_Handler），
//   只在接收端把原始通道重新映射成四个控制指令：
//     右杆上下 FBmove   → 油门（推上 = 指令增大）
//     右杆左右 LRmove   → 偏航（推右 = 机头右转）
//     左杆上下 Throttle → 俯仰（推前 = 机头下俯）
//     左杆左右 Rotation → 横滚（推右 = 机体右倾）
//   实机拨杆若发现某轴方向相反，把下面对应的 SIGN 改成 -1 即可。
// =====================================================================================
#define RC_BRINGUP_HZ          5
#define RC_BRINGUP_TICKS       (CLOCK_SYS_CLOCK_HZ / RC_BRINGUP_HZ)

// 每轴符号（已按 2026-11-20 实机拨杆读数确认）
//   只有横滚与协议假设相反：实际推右时 Rotation 原始值给出的是负方向，
//   这里取 -1 之后才是"推右 = 机体右倾"，其余三轴保持 +1
#define RC_SIGN_THROTTLE       (+1)
#define RC_SIGN_YAW            (+1)
#define RC_SIGN_PITCH          (+1)
#define RC_SIGN_ROLL           (-1)

// 映射后的四轴指令（±307 量级，供后续 400Hz 控制环使用）
static volatile s16 s_rc_cmd_throttle;
static volatile s16 s_rc_cmd_yaw;
static volatile s16 s_rc_cmd_pitch;
static volatile s16 s_rc_cmd_roll;
static volatile u8  s_rc_cmd_valid;         // 1 = 收到有效包；0 = 失联

static u32 s_rc_report_tick;

// 字符串 → 文本
static char *rc_put_str(char *p, const char *s)
{
	while (*s) *p++ = *s++;
	return p;
}

// 无符号整数 → 文本
static char *rc_put_u32(char *p, u32 v)
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

// 四位十六进制（键值用）
static char *rc_put_hex4(char *p, u16 v)
{
	u8 i;

	for (i = 4; i > 0; i--) {
		u8 nib = (u8)((v >> ((u8)((i - 1) * 4))) & 0x0Fu);

		*p++ = (char)((nib < 10) ? ('0' + nib) : ('A' + nib - 10));
	}
	return p;
}

// 带符号整数 → 文本
static char *rc_put_s16(char *p, s16 v)
{
	if (v < 0) {
		*p++ = '-';
		return rc_put_u32(p, (u32)(-v));
	}
	*p++ = '+';
	return rc_put_u32(p, (u32)v);
}

// 每圈调用：收到有效包就刷新指令，失联就清零并标记无效
static void RcBringup_Update(void)
{
	s16 fb  = RxdDataConvert(u16FBmove_Rxd, 1, 10);    // 右杆上下
	s16 lr  = RxdDataConvert(u16LRmove_Rxd, 1, 10);    // 右杆左右
	s16 thr = RxdDataConvert(u16Throttle_Rxd, 1, 10);  // 左杆上下
	s16 rot = RxdDataConvert(u16Rotation_Rxd, 1, 10);  // 左杆左右

	if (link_ok_flag) {
		s_rc_cmd_throttle = (s16)(RC_SIGN_THROTTLE * fb);
		s_rc_cmd_yaw      = (s16)(RC_SIGN_YAW * lr);
		s_rc_cmd_pitch    = (s16)(RC_SIGN_PITCH * thr);
		s_rc_cmd_roll     = (s16)(RC_SIGN_ROLL * rot);
		s_rc_cmd_valid    = 1;
	} else {
		// 失联：指令回零并标记无效，后续控制环据此保持停转
		s_rc_cmd_throttle = 0;
		s_rc_cmd_yaw      = 0;
		s_rc_cmd_pitch    = 0;
		s_rc_cmd_roll     = 0;
		s_rc_cmd_valid    = 0;
	}
}

// 低频输出一行：原始四通道、映射后四轴、按键、链路状态
static void RcBringup_Report(void)
{
	u32 now = clock_time();
	char buf[128];
	char *q;

	if ((s32)(now - s_rc_report_tick) < 0) {
		return;
	}
	s_rc_report_tick = now + RC_BRINGUP_TICKS;

	q = buf;
	q = rc_put_str(q, "RC lk=");
	q = rc_put_u32(q, link_ok_flag ? 1u : 0u);
	q = rc_put_str(q, " vld=");
	q = rc_put_u32(q, s_rc_cmd_valid ? 1u : 0u);
	q = rc_put_str(q, " raw=");
	q = rc_put_u32(q, u16Throttle_Rxd);      // 左杆上下（原始）
	*q++ = '/';
	q = rc_put_u32(q, u16Rotation_Rxd);      // 左杆左右（原始）
	*q++ = '/';
	q = rc_put_u32(q, u16FBmove_Rxd);        // 右杆上下（原始）
	*q++ = '/';
	q = rc_put_u32(q, u16LRmove_Rxd);        // 右杆左右（原始）
	q = rc_put_str(q, " cmd=");
	q = rc_put_s16(q, s_rc_cmd_throttle);
	*q++ = '/';
	q = rc_put_s16(q, s_rc_cmd_yaw);
	*q++ = '/';
	q = rc_put_s16(q, s_rc_cmd_pitch);
	*q++ = '/';
	q = rc_put_s16(q, s_rc_cmd_roll);
	q = rc_put_str(q, " key=");
	q = rc_put_hex4(q, u16RemotorKeyVal);
	*q++ = '\n';
	UartDbgSend((unsigned char *)buf, (u8)(q - buf));
}
#endif /* C_RC_BRINGUP_EN */

#if (C_RC_LOOP_400_EN)
// =====================================================================================
// 200Hz 地面联调：收包 + 姿态估计 + 角速度 P 环（电机始终停转）
//   目标角速度固定为 0，纠偏量 = Kp × (0 − 实测角速度)，只观察不出力；
//   本块只保存纠偏量，不写 PWM，电机停转由 C_ATT_BENCH_EN 分支保证。
//   姿态部分复用台架镜像已验证的静止初始化、加计一阶低通和长间断处理。
//   串口：10Hz 观察行（包龄/角速度/纠偏）+ 1Hz 统计行（采样/超时/跳拍）。
// =====================================================================================
// 台架左侧上/下键抓包模式：只记录原始按键字节，不初始化电机 PWM，保持四路低电平。
// 最终飞控镜像默认 0；抓包时单独用 -DC_L400_KEY_CAPTURE=1 构建。
#ifndef C_L400_KEY_CAPTURE
#define C_L400_KEY_CAPTURE 0
#endif

#define L400_HZ                200
#define L400_PERIOD_TICKS      (CLOCK_SYS_CLOCK_HZ / L400_HZ)          // 5ms = 120000 tick
#define L400_TICKS_PER_MS      (CLOCK_SYS_CLOCK_HZ / 1000)            // 24000 tick/ms
#define L400_TICKS_PER_100US   (CLOCK_SYS_CLOCK_HZ / 10000)           // 2400 tick/0.1ms
#define L400_TICKS_PER_US      (CLOCK_SYS_CLOCK_HZ / 1000000)         // 24 tick/us
#define L400_REPORT_TICKS      (CLOCK_SYS_CLOCK_HZ / 10)              // 100ms 观察行
// 报告预算：用上次实测的“组帧+提交”耗时 + 余量，分别判断观测/统计行。
// 首次发送用保守默认值，之后由实际耗时更新。
#define L400_TX_MARGIN_US      100
#define L400_OBS_COST_DEFAULT_US  100
#define L400_STAT_COST_DEFAULT_US 300

// 指令超时：超过这么久没有有效摇杆包，摇杆指令失效（松杆实测约 20~30ms 一包）
#define L400_CMD_TIMEOUT_MS    200
#define L400_CMD_TIMEOUT_TICKS ((u32)L400_CMD_TIMEOUT_MS * L400_TICKS_PER_MS)

// ---- 手动油门 / 十字(+)四电机混控 / 解锁安全 ----
// 油门/混控常量、事件码（0001 急停/0002 解锁）和事件状态机在 L400Motor.h。
// A/B 诊断继续用“特技”键，避免与解锁/急停混用。

// 锁定原因在 L400Motor.h（L400Q 的 x 字段）。
// 链路超时：超过这么久连心跳都没有，判为无线失联
#define L400_LINK_TIMEOUT_MS   500
#define L400_LINK_TIMEOUT_TICKS ((u32)L400_LINK_TIMEOUT_MS * L400_TICKS_PER_MS)
// 摇杆满量程 ±307（L400Motor.h）；外环输出：横滚/俯仰目标姿态 ±30°，
// 偏航目标角速度 ±3rad/s
#define L400_MAX_TILT_DEG100   3000
#define L400_YAW_RATE_MAX_Q10  3072
// 外环 P：重力方向叉积误差 Q15 × 1500 / 256，右移 13 位成 Q10；
// 30° 倾角误差（sin≈0.5）约输出 3000 Q10 ≈ 2.93rad/s，与上一版手感接近
#define L400_ATT_KP_Q8         1500
#define L400_RATE_LIMIT_Q10    4096
// 内环 P=1/4；I/D 初始为 0，保留积分/微分状态供下一版调参
#define L400_KP_Q8             64
#define L400_KI_Q16            0
#define L400_KD_Q8             0
#define L400_RATE_INT_MAX      1000000
#define L400_CORR_LIMIT_Q10    4096
// 启动静止窗口：与台架固件一致，连续静止 500ms 才定姿
#define L400_INIT_STILL_TICKS  ((u32)C_ATT_INIT_STILL_US * L400_TICKS_PER_US)
// 计数输出上限：长时间运行也不会把一行撑出 UART_DATA_LEN
#define L400_CNT_MAX           9999999u
#define L400_MS_MAX            9999u

// 收包时间全部保存原始 tick：链路用"任何包"，指令用"有效摇杆包"
static s32 s_l400_cmd[4];          // 油门/偏航/俯仰/横滚（±307，供后续控制环）
static u16 s_l400_key;
#if C_L400_KEY_CAPTURE
static u16 s_l400_cap_last_key;    // 左侧上/下键抓包：同一按键状态只记一次
static u8  s_l400_cap_last_rk;
static u8  s_l400_cap_last_b8;
static u8  s_l400_cap_last_b9;
static u8  s_l400_cap_last_b10;
static u8  s_l400_cap_last_b11;
static u32 s_l400_cap_last_tick;
static u8  s_l400_cap_have;
#endif
static u8  s_l400_cmd_ok;          // 摇杆指令有效（最近有效摇杆包未超时）
static u8  s_l400_link;            // 无线链路有效（任何包，含心跳）
static u8  s_l400_have_pkt;        // 是否收到过有效摇杆包
static u8  s_l400_have_link;       // 是否收到过无线包
static u32 s_l400_pkt_tick;        // 最近有效摇杆包（原始时钟 tick）
static u32 s_l400_link_tick;       // 最近无线包（原始时钟 tick）
static u32 s_l400_gap_max_tick;    // 本报告周期内相邻有效摇杆包最大间隔
static u32 s_l400_bad;             // 原始值越界、换算前被丢弃的包数

// 姿态与调度状态
static euler_q_t s_l400_att;
static u8  s_l400_inited;
static u32 s_l400_retry_tick;      // 传感器初始化失败后的重试时刻
static u32 s_l400_next_tick;       // 下一拍原定拍点
static u32 s_l400_last_ts;         // 上一有效样本读取完成 tick
static u8  s_l400_have_ts;
static u32 s_l400_dt_rem;          // 0.1ms 换算余数
static s32 s_l400_gyro_rem[3];     // 陀螺 Q10 换算余数

// 加计一阶低通（参数与台架固件相同）
static s32 s_l400_lpf[3];
static u8  s_l400_lpf_ok;
static u32 s_l400_lpf_alpha;       // Q15，只随 dt 变化
static u32 s_l400_lpf_alpha_dt_us;

// 启动静止判定（与台架固件相同）
static u32 s_l400_still_ticks;
static s32 s_l400_still_sum[3];
static u32 s_l400_still_n;
static s32 s_l400_prev_acc[3];
static u8  s_l400_prev_ok;

// 控制环输出（只观察，不写电机）
static s32 s_l400_rate[3];         // 扣偏后实测角速度 Q10 rad/s
static s32 s_l400_corr[3];         // 内环 PID 纠偏量 Q10 rad/s
static u8  s_l400_ctrl_ok;         // 控制输出使能：姿态就绪且链路/指令有效
static s32 s_l400_tgt_att[2];      // 外环目标横滚/俯仰，0.01°（仅上位机显示）
static s32 s_l400_tgt_up[3];       // 目标重力反方向（向上比力）Q15，收包时换算
static s32 s_l400_tgt_yaw_rate;    // 偏航杆目标角速度 Q10 rad/s，收包时换算
static s32 s_l400_tgt_rate[3];     // 外环输出/偏航目标角速度 Q10 rad/s
static s32 s_l400_stick_last[2];   // 上次重算目标姿态时的横滚/俯仰杆量
static s32 s_l400_stick_yaw_last;  // 上次重算偏航目标时的偏航杆量
static u8  s_l400_tgt_valid;       // 目标缓存是否有效；失联后清零强制首包重算
#if (L400_KI_Q16 != 0)
static s32 s_l400_rate_integ[3];   // 内环积分状态；I=0 时不编译
#endif
#if (L400_KD_Q8 != 0)
static s32 s_l400_rate_prev_err[3]; // 内环上一拍误差；D=0 时不编译
static u8  s_l400_rate_have_prev;
#endif

// 统计
static u32 s_l400_sample;          // 实际采样次数（成功读到的六轴样本）
static u32 s_l400_fail;            // 六轴读失败次数
static u32 s_l400_euler_ok;        // 姿态更新次数（Euler_Update 成功）
static u32 s_l400_miss;            // 整拍超期：原定拍点→串口报告完成超过 1 个周期
static u32 s_l400_ctrl_miss;       // 控制超期：原定拍点→控制计算完成超过 1 个周期
static u32 s_l400_skip;            // 跳拍：迟到超过 1 个周期时丢弃的节拍数
static u32 s_l400_txd;             // 串口报告因余量不足/串口忙而延期的次数
static u32 s_l400_ctrl_max_us;     // 本统计周期内最大控制完成时延 us
static u32 s_l400_ctrl_peak_us;    // 上电以来最大控制完成时延 us
static u32 s_l400_tx_tick;         // 下一次观察行时刻
static u32 s_l400_stat_tick;       // 下一次统计行时刻
static u8  s_l400_stat_deferred;   // 1Hz 统计行因余量不足/串口忙延期
static u8  s_l400_tx_deferred;     // 10Hz 观察行因余量不足/串口忙延期
static u32 s_l400_obs_cost_us = L400_OBS_COST_DEFAULT_US;  // 上次观测行实测耗时
static u32 s_l400_stat_cost_us = L400_STAT_COST_DEFAULT_US; // 上次统计行实测耗时
static u32 s_l400_tx_defer_budget; // 报告延期原因：下一拍余量不足
static u32 s_l400_tx_defer_busy;   // 报告延期原因：UART/DMA 忙
static u8  s_l400_tx_diag_due;     // 需要补发一行 L400T 报告延期计数
static u32 s_l400_tx_diag_tick;    // L400T 最短发送间隔（1s）

// A/B 分段诊断：A 只收包+姿态，B 再开双环；两段都不发常规遥测，
// 结束后一次性输出分项耗时。等待链路稳定的 2s 内仍正常发遥测。
#define L400_DIAG_SETTLE_TICKS  (CLOCK_SYS_CLOCK_HZ * 2)
#define L400_DIAG_SEG_TICKS     (CLOCK_SYS_CLOCK_HZ * 10)
// 台架按需触发：按一下遥控“特技”键；L400 路径不再执行旧按键业务
#define L400_DIAG_ARM_KEY       RfSendKey_StuntShow

typedef struct {
	u32 late_avg, late_max;
	u32 exec_avg, exec_max;
	u32 cmiss, skip;
	u32 rfirq_avg, rfirq_max;
	u32 rf_avg, rf_max;
	u32 app_avg, app_max;
} l400_diag_t;

static L400Diag s_l400_diag;           // 纯逻辑 A/B 状态机
static L400DiagTx s_l400_diag_tx;      // 汇总行待发队列
static u8  s_l400_diag_enabled;        // 台架按键已按需开启；正常路径为 0
static u8  s_l400_diag_active;         // A/B 或待发汇总期间暂缓常规遥测
static u8  s_l400_diag_ctrl_enable;    // A=0，B/正常=1
static u8  s_l400_diag_have_a;         // A 段快照是否已完成
static u16 s_l400_diag_key_prev;       // 按键边沿检测

// 电机安全状态机：上电锁定，必须油门低位 + 长按解锁键才允许输出
static L400MArm s_l400_arm;
static u8  s_l400_att_ever_ready;      // 已成功更新过姿态，用于区分上电锁定/姿态异常
static u8  s_l400_pwm_ready;           // PWM 外设已初始化
static l400_diag_t s_l400_diag_a;
static l400_diag_t s_l400_diag_b;
static u32 s_l400_diag_rf_sum_ticks;   // d24_rx_main_loop 累计 tick
static u32 s_l400_diag_rf_max_ticks;
static u32 s_l400_diag_rf_cnt;

// 射频中断计时有 app_24g_rf.c 维护；只在诊断段打开
extern volatile unsigned char g_rf_irq_meas_en;
extern volatile unsigned int  g_rf_irq_sum_ticks;
extern volatile unsigned int  g_rf_irq_max_ticks;
extern volatile unsigned int  g_rf_irq_cnt;

// 分项耗时（本统计窗口累计/峰值，单位 us；统计行发出后清零）
static u32 s_l400_late_sum_us;     // 原定拍点 → 实际采样开始
static u32 s_l400_late_max_us;
static u32 s_l400_exec_sum_us;     // IMU 读取 + 估计 + P 环
static u32 s_l400_exec_max_us;
// prep：采样开始到 Euler_Update 之前；eul：只包住 Euler_Update。
// 两者用独立 1Hz 窗口，L400P 发出后清零，便于优化前后对照。
static u32 s_l400_prep_sum_us;
static u32 s_l400_prep_max_us;
static u32 s_l400_prep_cnt;
static u32 s_l400_eul_sum_us;
static u32 s_l400_eul_max_us;
static u32 s_l400_eul_cnt;
static u32 s_l400_prep_tick;       // 下一行 L400P 的最早发送时刻
static u32 s_l400_rx_sum_us;       // 无线整包处理（解码/映射/按键/时间戳，不含 RF 驱动）
static u32 s_l400_rx_max_us;
static u32 s_l400_tx_sum_us;       // 串口报告格式化与提交
static u32 s_l400_tx_max_us;
static u32 s_l400_tick_cnt;        // 本窗口 late/exec 的拍数
static u32 s_l400_rx_cnt;          // 本窗口无线包数
static u32 s_l400_tx_cnt;          // 本窗口报告次数

// 有效摇杆包到达时换算目标姿态/目标重力方向；控制拍只做重力叉积和 PID
static void L400_UpdateStickTargets(void);
// 电机安全停止（供收包/诊断/控制路径调用，具体实现在 L400 控制段）
static void L400_MotorStop(u8 reason);

#if C_L400_KEY_CAPTURE
static char *l400_put_str(char *p, const char *s);
static char *l400_put_u32(char *p, u32 v);
static char *l400_put_hex(char *p, u32 v, u8 digits);

// 台架左侧上/下键抓包：每次 key/rk/原始字节变化打印一行；阻塞发送只用于抓包镜像。
static void L400_KeyCapture(u32 now)
{
	u8 b8 = UserRfRcvBuffer[8];
	u8 b9 = UserRfRcvBuffer[9];
	u8 b10 = UserRfRcvBuffer[10];
	u8 b11 = UserRfRcvBuffer[11];
	u8 changed = !s_l400_cap_have
		|| s_l400_key != s_l400_cap_last_key
		|| rf_received_key != s_l400_cap_last_rk
		|| b8 != s_l400_cap_last_b8
		|| b9 != s_l400_cap_last_b9
		|| b10 != s_l400_cap_last_b10
		|| b11 != s_l400_cap_last_b11;

	if (!changed) {
		return;
	}
	{
		char buf[96];
		char *q = buf;

		q = l400_put_str(q, "L400K key=");
		q = l400_put_hex(q, s_l400_key, 4);
		q = l400_put_str(q, " rk=");
		q = l400_put_hex(q, rf_received_key, 2);
		q = l400_put_str(q, " b=");
		q = l400_put_hex(q, b8, 2);
		*q++ = '/';
		q = l400_put_hex(q, b9, 2);
		*q++ = '/';
		q = l400_put_hex(q, b10, 2);
		*q++ = '/';
		q = l400_put_hex(q, b11, 2);
		q = l400_put_str(q, " dt=");
		q = l400_put_u32(q, s_l400_cap_have
						? (now - s_l400_cap_last_tick) / L400_TICKS_PER_MS : 0);
		*q++ = '\n';
		UartDbgSend((unsigned char *)buf, (u8)(q - buf));
	}
	s_l400_cap_last_key = s_l400_key;
	s_l400_cap_last_rk = rf_received_key;
	s_l400_cap_last_b8 = b8;
	s_l400_cap_last_b9 = b9;
	s_l400_cap_last_b10 = b10;
	s_l400_cap_last_b11 = b11;
	s_l400_cap_last_tick = now;
	s_l400_cap_have = 1;
}
#endif

#if (!C_L400_KEY_CAPTURE)
// 每个 RF 包处理一次 0001/0002 事件；packet_valid=0 的包也会消费事件沿，
// 因此无效摇杆包里的 0002 不会等到后续重复包再解锁。
static void L400_ProcessKeyEvent(u8 packet_valid)
{
	u8 thr_low = (s_l400_cmd[0] <= L400_ARM_THR_MAX) ? 1 : 0;
	u8 sticks_center = (s_l400_cmd[1] >= -L400_ARM_STICK_MAX
						&& s_l400_cmd[1] <= L400_ARM_STICK_MAX
						&& s_l400_cmd[2] >= -L400_ARM_STICK_MAX
						&& s_l400_cmd[2] <= L400_ARM_STICK_MAX
						&& s_l400_cmd[3] >= -L400_ARM_STICK_MAX
						&& s_l400_cmd[3] <= L400_ARM_STICK_MAX) ? 1 : 0;
	u8 stop = L400M_EventStep(&s_l400_arm, s_l400_key, packet_valid,
							  thr_low, sticks_center, s_l400_att.ready,
							  s_l400_link, s_l400_diag_enabled);

	if (stop) {
		L400_MotorStop(stop);
	}
}
#endif

// 整包解码完成后的提交：全零心跳只更新链路，摇杆值换算前先验证
void RcLoop_OnPacket(void)
{
	u16 thr = u16Throttle_Rxd, rot = u16Rotation_Rxd;
	u16 fb = u16FBmove_Rxd, lr = u16LRmove_Rxd;
	u16 key = u16RemotorKeyVal;
	u32 now = clock_time();

	// 任何成功解码的包都刷新链路时刻（心跳也说明遥控器还在）
	s_l400_link = 1;
	s_l400_link_tick = now;
	s_l400_have_link = 1;

	// 按键值先更新/抓包，再判断全零心跳；左侧上/下键可能只带按键、四摇杆为 0
	s_l400_key = key;
#if C_L400_KEY_CAPTURE
	L400_KeyCapture(now);
#endif

	if (thr == 0 && rot == 0 && fb == 0 && lr == 0) {
#if (!C_L400_KEY_CAPTURE)
		L400_ProcessKeyEvent(0);    // 心跳中的 0001/0002 也要消费事件沿
#endif
		return;                    // 全零心跳：只更新链路/按键，不动摇杆指令
	}

	// 换算前先验证原始值：RxdDataConvert 内部会把越界值钳到 0x0400/0x1C00，
	// 先换算再判断就分不清"满杆"和"坏包"，所以越界包在这里直接丢弃
	if (thr < RfDataMinValue || thr > RfDataMaxValue ||
		rot < RfDataMinValue || rot > RfDataMaxValue ||
		fb  < RfDataMinValue || fb  > RfDataMaxValue ||
		lr  < RfDataMinValue || lr  > RfDataMaxValue) {
		s_l400_bad++;
#if (!C_L400_KEY_CAPTURE)
		L400_ProcessKeyEvent(0);    // 无效摇杆包：急停仍有效，解锁被丢弃
#endif
		return;
	}

	// 实机确认的映射：FB→油门、LR→偏航、Throttle→俯仰、Rotation→横滚(-1)
	s_l400_cmd[0] = RxdDataConvert(fb, 1, 10);
	s_l400_cmd[1] = RxdDataConvert(lr, 1, 10);
	s_l400_cmd[2] = RxdDataConvert(thr, 1, 10);
	s_l400_cmd[3] = -RxdDataConvert(rot, 1, 10);

	// 有效摇杆包间隔也用原始 tick 记录，供实测失联门限（心跳不参与）
	if (s_l400_have_pkt) {
		u32 gap = now - s_l400_pkt_tick;

		if (gap > s_l400_gap_max_tick) {
			s_l400_gap_max_tick = gap;
		}
	}
	s_l400_pkt_tick = now;
	s_l400_have_pkt = 1;
	s_l400_cmd_ok = 1;
#if (!C_L400_KEY_CAPTURE)
	L400_ProcessKeyEvent(1);        // 该包校验通过且摇杆有效
#endif
	// 摇杆目标只在有效收包时换算，200Hz 控制拍不再做角度→重力方向的三角函数
	L400_UpdateStickTargets();

	// 台架按需开启 A/B：遥控“特技”键上升沿；正常飞/不按键时完全不进诊断
	if ((s_l400_key & L400_DIAG_ARM_KEY)
		&& !(s_l400_diag_key_prev & L400_DIAG_ARM_KEY)
		&& !s_l400_diag_active && s_l400_diag_tx.count == 0) {
		s_l400_diag_enabled = 1;
		L400Diag_Init(&s_l400_diag, L400_DIAG_SETTLE_TICKS, L400_DIAG_SEG_TICKS);
		L400DiagTx_Init(&s_l400_diag_tx);
		s_l400_diag_ctrl_enable = 1;
		s_l400_diag_have_a = 0;
		s_l400_diag_rf_sum_ticks = 0;
		s_l400_diag_rf_max_ticks = 0;
		s_l400_diag_rf_cnt = 0;
		g_rf_irq_meas_en = 0;
		L400_MotorStop(L400_LOCK_DIAG);
	}
	s_l400_diag_key_prev = s_l400_key;
}

// 累加一次分项耗时（us）：sum 用于平均值，max 用于峰值
static void l400_add_timing(u32 *sum, u32 *max, u32 us)
{
	*sum += us;
	if (us > *max) {
		*max = us;
	}
}

// 无线收包处理耗时：car_work 从整包解码到映射/按键/时间戳提交完成；
// 不含 d24_rx_main_loop 的射频驱动/配对部分。
static void RcLoop_RecordRx(u32 ticks)
{
	u32 us = (ticks + L400_TICKS_PER_US / 2) / L400_TICKS_PER_US;

	l400_add_timing(&s_l400_rx_sum_us, &s_l400_rx_max_us, us);
	s_l400_rx_cnt++;
}

// 文本格式化（不用 printf / 浮点）
static char *l400_put_str(char *p, const char *s)
{
	while (*s) {
		*p++ = *s++;
	}
	return p;
}

static char *l400_put_u32(char *p, u32 v)
{
	char t[10];
	u8 n = 0;

	if (!v) {
		*p++ = '0';
		return p;
	}
	while (v && n < sizeof(t)) {
		t[n++] = (char)('0' + v % 10);
		v /= 10;
	}
	while (n) {
		*p++ = t[--n];
	}
	return p;
}

// 定宽十六进制：4 位或 6 位，避免长跑后一行超出 UART_DATA_LEN
static char *l400_put_hex(char *p, u32 v, u8 digits)
{
	u8 i;

	for (i = digits; i > 0; i--) {
		u8 nib = (u8)((v >> ((u8)((i - 1) * 4))) & 0x0Fu);

		*p++ = (char)((nib < 10) ? ('0' + nib) : ('A' + nib - 10));
	}
	return p;
}

// 计数封顶：超过上限就停在 cap，配合定宽输出
static u32 l400_cap(u32 v, u32 cap)
{
	return (v > cap) ? cap : v;
}

// Q15 四元数分量 → 4 位有符号十六进制（-32768…32767，两补码低 16 位）；
// +32768 夹到 +32767，避免 0x8000 被上位机误读成 -32768。
static char *l400_put_q15(char *p, s32 v)
{
	if (v > 32767) {
		v = 32767;
	} else if (v < -32768) {
		v = -32768;
	}
	return l400_put_hex(p, (u32)v, 4);
}

// Q10 角速度 → 5 位有符号二十位十六进制（低 20 位两补码），Python 负责还原
static char *l400_put_q10_20(char *p, s32 v)
{
	return l400_put_hex(p, (u32)v & 0xFFFFFu, 5);
}

// 清空启动静止窗口
static void L400_StillReset(void)
{
	s_l400_still_ticks = 0;
	s_l400_still_n = 0;
	s_l400_still_sum[0] = 0;
	s_l400_still_sum[1] = 0;
	s_l400_still_sum[2] = 0;
}

// 传感器初始化成功后清空全部运行状态（统计从 0 开始）
static void L400_StateReset(void)
{
	int i;

	Euler_Reset(&s_l400_att);
	s_l400_last_ts = 0;
	s_l400_have_ts = 0;
	s_l400_dt_rem = 0;
	s_l400_lpf_ok = 0;
	s_l400_lpf_alpha = 0;
	s_l400_lpf_alpha_dt_us = 0;
	s_l400_prev_ok = 0;
	for (i = 0; i < 3; i++) {
		s_l400_gyro_rem[i] = 0;
		s_l400_lpf[i] = 0;
		s_l400_rate[i] = 0;
		s_l400_corr[i] = 0;
		s_l400_tgt_rate[i] = 0;
		s_l400_tgt_up[i] = 0;
#if (L400_KI_Q16 != 0)
		s_l400_rate_integ[i] = 0;
#endif
#if (L400_KD_Q8 != 0)
		s_l400_rate_prev_err[i] = 0;
#endif
		s_l400_still_sum[i] = 0;
		s_l400_prev_acc[i] = 0;
	}
	L400_StillReset();
	s_l400_sample = 0;
	s_l400_fail = 0;
	s_l400_euler_ok = 0;
	s_l400_miss = 0;
	s_l400_ctrl_miss = 0;
	s_l400_skip = 0;
	s_l400_txd = 0;
	s_l400_obs_cost_us = L400_OBS_COST_DEFAULT_US;
	s_l400_stat_cost_us = L400_STAT_COST_DEFAULT_US;
	s_l400_tx_defer_budget = 0;
	s_l400_tx_defer_busy = 0;
	s_l400_tx_diag_due = 0;
	s_l400_tx_diag_tick = 0;
	s_l400_ctrl_max_us = 0;
	s_l400_ctrl_peak_us = 0;
	s_l400_ctrl_ok = 0;
	s_l400_tgt_att[0] = 0;
	s_l400_tgt_att[1] = 0;
	s_l400_tgt_yaw_rate = 0;
	s_l400_stick_last[0] = 0;
	s_l400_stick_last[1] = 0;
	s_l400_stick_yaw_last = 0;
	s_l400_tgt_valid = 0;
	L400Diag_Init(&s_l400_diag, L400_DIAG_SETTLE_TICKS, L400_DIAG_SEG_TICKS);
	L400DiagTx_Init(&s_l400_diag_tx);
	s_l400_diag_enabled = 0;
	s_l400_diag_active = 0;
	s_l400_diag_key_prev = 0;
	s_l400_diag_ctrl_enable = 1;
	L400M_ArmInit(&s_l400_arm);
	s_l400_att_ever_ready = 0;
	s_l400_diag_have_a = 0;
	s_l400_diag_rf_sum_ticks = 0;
	s_l400_diag_rf_max_ticks = 0;
	s_l400_diag_rf_cnt = 0;
	g_rf_irq_meas_en = 0;
	g_rf_irq_sum_ticks = 0;
	g_rf_irq_max_ticks = 0;
	g_rf_irq_cnt = 0;
#if (L400_KD_Q8 != 0)
	s_l400_rate_have_prev = 0;
#endif
	s_l400_late_sum_us = 0;
	s_l400_late_max_us = 0;
	s_l400_exec_sum_us = 0;
	s_l400_exec_max_us = 0;
	s_l400_prep_sum_us = 0;
	s_l400_prep_max_us = 0;
	s_l400_prep_cnt = 0;
	s_l400_eul_sum_us = 0;
	s_l400_eul_max_us = 0;
	s_l400_eul_cnt = 0;
	s_l400_rx_sum_us = 0;
	s_l400_rx_max_us = 0;
	s_l400_tx_sum_us = 0;
	s_l400_tx_max_us = 0;
	s_l400_tick_cnt = 0;
	s_l400_rx_cnt = 0;
	s_l400_tx_cnt = 0;
	s_l400_stat_deferred = 0;
	s_l400_tx_deferred = 0;
	s_l400_cmd[0] = 0;
	s_l400_cmd[1] = 0;
	s_l400_cmd[2] = 0;
	s_l400_cmd[3] = 0;
	s_l400_key = 0;
	s_l400_cmd_ok = 0;
	s_l400_link = 0;
	s_l400_have_pkt = 0;
	s_l400_have_link = 0;
	s_l400_gap_max_tick = 0;
	s_l400_bad = 0;
}

// 传感器初始化：含陀螺片内低通写后回读；失败 1s 后重试
static void L400_MotorStop(u8 reason);

static void L400_Init(void)
{
	u32 now = clock_time();
	u8 rb = 0;
	u8 st;
	char buf[64];
	char *q;

	if ((s32)(now - s_l400_retry_tick) < 0) {
		return;
	}
	s_l400_retry_tick = now + CLOCK_SYS_CLOCK_HZ;      // 失败后 1s 重试

	st = t3488_bench_init(C_GYRO_LPF_SEL, &rb);
	if (st) {
		q = buf;
		q = l400_put_str(q, "L400 init fail st=");
		q = l400_put_u32(q, st);
		q = l400_put_str(q, " lpf_rb=");
		q = l400_put_u32(q, rb);
		*q++ = '\n';
		UartDbgSend((unsigned char *)buf, (u8)(q - buf));
		return;
	}

	L400_StateReset();

	// 正常镜像启动 PWM 并保持四路 0；抓包镜像完全不初始化 PWM，引脚保持低电平。
#if (!C_L400_KEY_CAPTURE)
	if (!s_l400_pwm_ready) {
		User_pwm_init_func();
		s_l400_pwm_ready = 1;
	}
#endif
	L400_MotorStop(L400_LOCK_BOOT);

	q = buf;
	q = l400_put_str(q, "L400 init ok lpf_rb=");
	q = l400_put_u32(q, rb);
	*q++ = '\n';
	UartDbgSend((unsigned char *)buf, (u8)(q - buf));

	// 初始化提示行发完再排第一拍，避免启动提示占用 200Hz 周期
	now = clock_time();
	s_l400_next_tick = now + L400_PERIOD_TICKS;
	s_l400_tx_tick = now + L400_REPORT_TICKS;
	s_l400_stat_tick = now + CLOCK_SYS_CLOCK_HZ;
	s_l400_prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	s_l400_inited = 1;
}

// 启动阶段：用一段连续静止样本的平均加速度定姿（移动/间断就重开窗口）
static void L400_Startup(const att_i32 acc_mg[3], const att_i32 gyr_q10[3],
						 u32 dt_ticks, u32 ts)
{
	u8 still;
	int i;

	still = Euler_IsStill(acc_mg, gyr_q10, s_l400_prev_acc, s_l400_prev_ok);

	if (!still) {
		L400_StillReset();
	} else {
		s_l400_still_sum[0] += acc_mg[0];
		s_l400_still_sum[1] += acc_mg[1];
		s_l400_still_sum[2] += acc_mg[2];
		s_l400_still_n++;
		s_l400_still_ticks += dt_ticks;

		if (s_l400_still_ticks >= L400_INIT_STILL_TICKS && s_l400_still_n > 0) {
			att_i32 avg[3];

			avg[0] = s_l400_still_sum[0] / (att_i32)s_l400_still_n;
			avg[1] = s_l400_still_sum[1] / (att_i32)s_l400_still_n;
			avg[2] = s_l400_still_sum[2] / (att_i32)s_l400_still_n;
			if (!Euler_InitFromAccel(&s_l400_att, avg, ts / L400_TICKS_PER_MS)) {
				L400_StillReset();     // 模长等条件不满足：重开静止窗口
			}
		}
	}

	for (i = 0; i < 3; i++) {
		s_l400_prev_acc[i] = acc_mg[i];
	}
	s_l400_prev_ok = 1;
}

// 报告延期计数：一次报告事件只计一次；按原因分“余量不足”和“DMA 忙”
static void l400_note_tx_defer(u8 *flag, u8 busy)
{
	if (!*flag) {
		*flag = 1;
		s_l400_txd++;
		if (busy) {
			s_l400_tx_defer_busy++;
		} else {
			s_l400_tx_defer_budget++;
		}
		s_l400_tx_diag_due = 1;
	}
}

// 统计行发出后清空本窗口累计量；统计行自身的 tx 开销计入下一窗口
static void L400_ResetWindow(void)
{
	s_l400_ctrl_max_us = 0;
	s_l400_gap_max_tick = 0;
	s_l400_late_sum_us = 0;
	s_l400_late_max_us = 0;
	s_l400_exec_sum_us = 0;
	s_l400_exec_max_us = 0;
	s_l400_rx_sum_us = 0;
	s_l400_rx_max_us = 0;
	s_l400_tx_sum_us = 0;
	s_l400_tx_max_us = 0;
	s_l400_tick_cnt = 0;
	s_l400_rx_cnt = 0;
	s_l400_tx_cnt = 0;
}

// 1Hz 统计行：非阻塞提交；返回 1 已交给 DMA，0 串口忙（由调用方延期）
static u8 L400_SendStats(u32 t_tx0)
{
	char buf[UART_DATA_LEN];
	char *q = buf;
	u32 late_avg, exec_avg, rx_avg, tx_avg;

	// 本窗口平均值；本拍 late/exec 已累加进来，tx 报告本身
	// 要等发完才计时，所以 tx 行显示的是上一个窗口的值。
	late_avg = s_l400_tick_cnt ? s_l400_late_sum_us / s_l400_tick_cnt : 0;
	exec_avg = s_l400_tick_cnt ? s_l400_exec_sum_us / s_l400_tick_cnt : 0;
	rx_avg = s_l400_rx_cnt ? s_l400_rx_sum_us / s_l400_rx_cnt : 0;
	tx_avg = s_l400_tx_cnt ? s_l400_tx_sum_us / s_l400_tx_cnt : 0;

	// 全部定宽十六进制；最长正好 160 字节（含换行），等于 UART_DATA_LEN，
	// 因此 UartTrySend 的长度检查仍然通过，不会截断。
	q = l400_put_str(q, "L400S bad=");
	q = l400_put_hex(q, l400_cap(s_l400_bad, 0xFFFFu), 4);
	q = l400_put_str(q, " n=");
	q = l400_put_hex(q, l400_cap(s_l400_sample, 0xFFFFFFu), 6);
	q = l400_put_str(q, " f=");
	q = l400_put_hex(q, l400_cap(s_l400_fail, 0xFFFFu), 4);
	q = l400_put_str(q, " eul=");
	q = l400_put_hex(q, l400_cap(s_l400_euler_ok, 0xFFFFFFu), 6);
	q = l400_put_str(q, " miss=");
	q = l400_put_hex(q, l400_cap(s_l400_miss, 0xFFFFFFu), 6);
	q = l400_put_str(q, " cmiss=");
	q = l400_put_hex(q, l400_cap(s_l400_ctrl_miss, 0xFFFFFFu), 6);
	q = l400_put_str(q, " skip=");
	q = l400_put_hex(q, l400_cap(s_l400_skip, 0xFFFFFFu), 6);
	q = l400_put_str(q, " txd=");
	q = l400_put_hex(q, l400_cap(s_l400_txd, 0xFFFFFFu), 6);
	q = l400_put_str(q, " ctl=");
	q = l400_put_hex(q, l400_cap(s_l400_ctrl_max_us, 0xFFFFu), 4);
	*q++ = '/';
	q = l400_put_hex(q, l400_cap(s_l400_ctrl_peak_us, 0xFFFFu), 4);
	q = l400_put_str(q, " late=");
	q = l400_put_hex(q, l400_cap(late_avg, 0xFFFFu), 4);
	*q++ = '/';
	q = l400_put_hex(q, l400_cap(s_l400_late_max_us, 0xFFFFu), 4);
	q = l400_put_str(q, " exec=");
	q = l400_put_hex(q, l400_cap(exec_avg, 0xFFFFu), 4);
	*q++ = '/';
	q = l400_put_hex(q, l400_cap(s_l400_exec_max_us, 0xFFFFu), 4);
	q = l400_put_str(q, " rx=");
	q = l400_put_hex(q, l400_cap(rx_avg, 0xFFFFu), 4);
	*q++ = '/';
	q = l400_put_hex(q, l400_cap(s_l400_rx_max_us, 0xFFFFu), 4);
	q = l400_put_str(q, " tx=");
	q = l400_put_hex(q, l400_cap(tx_avg, 0xFFFFu), 4);
	*q++ = '/';
	q = l400_put_hex(q, l400_cap(s_l400_tx_max_us, 0xFFFFu), 4);
	*q++ = '\n';

	if (!UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		return 0;                  // 串口忙：不等待，下一拍再试
	}
	s_l400_stat_tick = t_tx0 + CLOCK_SYS_CLOCK_HZ;
	s_l400_tx_tick = t_tx0 + L400_REPORT_TICKS;   // 这一拍不再发观察行
	L400_ResetWindow();
	return 1;
}

// 10Hz 观察行：非阻塞提交；返回 1 已交给 DMA，0 串口忙（由调用方延期）
// 常规姿态观测：短的定宽 L400Q 报文，只带 Q15 四元数、原始 Q10 角速度/
// 纠偏和必要状态；角度/°/s 等数值换算全部留给 Python。
static u8 L400_SendObs(u32 t_tx0)
{
	char buf[UART_DATA_LEN];
	char *q = buf;
	u32 age_link, age_pkt, gap_ms;
	u8 flags;

	age_link = s_l400_have_link
			 ? ((t_tx0 - s_l400_link_tick + L400_TICKS_PER_MS / 2)
				/ L400_TICKS_PER_MS)
			 : L400_MS_MAX;
	age_pkt = s_l400_have_pkt
			? ((t_tx0 - s_l400_pkt_tick + L400_TICKS_PER_MS / 2)
			   / L400_TICKS_PER_MS)
			: L400_MS_MAX;
	gap_ms = (s_l400_gap_max_tick + L400_TICKS_PER_MS / 2) / L400_TICKS_PER_MS;

	flags = (u8)((s_l400_att.ready ? 0x01 : 0)
				 | (s_l400_link ? 0x02 : 0)
				 | (s_l400_cmd_ok ? 0x04 : 0)
				 | (s_l400_ctrl_ok ? 0x08 : 0));

	// 最长约 90 字节，远短于 L400S；所有字段定宽，便于上位机按列解析
	q = l400_put_str(q, "L400Q f=");
	q = l400_put_hex(q, flags, 1);
	q = l400_put_str(q, " a=");
	q = l400_put_hex(q, l400_cap(age_link, 0xFFFFu), 4);
	q = l400_put_str(q, " v=");
	q = l400_put_hex(q, l400_cap(age_pkt, 0xFFFFu), 4);
	q = l400_put_str(q, " k=");
	q = l400_put_hex(q, l400_cap(gap_ms, 0xFFFFu), 4);
	q = l400_put_str(q, " q=");
	q = l400_put_q15(q, s_l400_att.q[0]);
	*q++ = '/';
	q = l400_put_q15(q, s_l400_att.q[1]);
	*q++ = '/';
	q = l400_put_q15(q, s_l400_att.q[2]);
	*q++ = '/';
	q = l400_put_q15(q, s_l400_att.q[3]);
	q = l400_put_str(q, " r=");
	q = l400_put_q10_20(q, s_l400_rate[0]);
	*q++ = '/';
	q = l400_put_q10_20(q, s_l400_rate[1]);
	*q++ = '/';
	q = l400_put_q10_20(q, s_l400_rate[2]);
	q = l400_put_str(q, " c=");
	q = l400_put_hex(q, (u32)s_l400_corr[0] & 0xFFFFu, 4);
	*q++ = '/';
	q = l400_put_hex(q, (u32)s_l400_corr[1] & 0xFFFFu, 4);
	*q++ = '/';
	q = l400_put_hex(q, (u32)s_l400_corr[2] & 0xFFFFu, 4);
	// 锁定状态和四路实际占空比；顺序固定为 前/后/左/右（3 位十六进制）
	q = l400_put_str(q, " m=");
	q = l400_put_hex(q, s_l400_arm.armed, 1);
	q = l400_put_str(q, " x=");
	q = l400_put_hex(q, s_l400_arm.reason, 1);
	q = l400_put_str(q, " pwm=");
	q = l400_put_hex(q, MOT_FRONT_DUTY, 3);
	*q++ = '/';
	q = l400_put_hex(q, MOT_REAR_DUTY, 3);
	*q++ = '/';
	q = l400_put_hex(q, MOT_LEFT_DUTY, 3);
	*q++ = '/';
	q = l400_put_hex(q, MOT_RIGHT_DUTY, 3);
	*q++ = '\n';

	if (!UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		return 0;                  // 串口忙：不等待，下一拍再试
	}
	s_l400_tx_tick = t_tx0 + L400_REPORT_TICKS;
	s_l400_gap_max_tick = 0;
	return 1;
}

// 对称限幅，避免外环/内环在大误差或积分饱和时输出过大
static s32 l400_clamp_s32(s32 v, s32 lim)
{
	if (v > lim) {
		return lim;
	}
	if (v < -lim) {
		return -lim;
	}
	return v;
}

// 四路实际占空比：MOT_* 宏按 V12 的“物理位置→PWM 变量”映射，
// 顺序为前/后/左/右；锁定或未初始化时只写 0。
static void L400_MotorWrite(u16 front, u16 rear, u16 left, u16 right)
{
	if (!s_l400_pwm_ready) {
		return;
	}
	MOT_FRONT_DUTY = front;
	MOT_REAR_DUTY = rear;
	MOT_LEFT_DUTY = left;
	MOT_RIGHT_DUTY = right;
	Update4MotPwmDuty();
}

// 立即锁定并清四路；reason 非 0 时记录锁定原因。故障/急停后必须重新解锁。
static void L400_MotorStop(u8 reason)
{
	L400M_ArmLock(&s_l400_arm, reason);
	L400_MotorWrite(0, 0, 0, 0);
}

// 200Hz 混控：手动油门 → 推力；三轴纠偏 Q10 → PWM 差动。
// 横滚：左 +dr，右 -dr；俯仰：前 -dp，后 +dp；偏航：前/后 +dy，左/右 -dy。
// 差动限幅 ±160 PWM，并按当前推力的上下余量做统一缩放，保证四路平均推力不漂。
static void L400_MotorUpdate(void)
{
	L400MMix mix;
	unsigned short out[4];

	if (!s_l400_arm.armed) {
		L400_MotorWrite(0, 0, 0, 0);
		return;
	}
	if (!s_l400_att.ready || !s_l400_link || !s_l400_cmd_ok
		|| !s_l400_ctrl_ok || s_l400_diag_enabled) {
		if (!s_l400_link) {
			L400_MotorStop(L400_LOCK_LINK);
		} else if (!s_l400_cmd_ok) {
			L400_MotorStop(L400_LOCK_CMD);
		} else if (s_l400_diag_enabled) {
			L400_MotorStop(L400_LOCK_DIAG);
		} else {
			L400_MotorStop(L400_LOCK_ATT);
		}
		return;
	}

	// 纯逻辑模块完成油门、三轴差动、统一限幅；未解锁时输出强制为 0
	L400M_Mix(L400M_Thrust(s_l400_cmd[0]),
			  s_l400_corr[0], s_l400_corr[1], s_l400_corr[2], &mix);
	L400M_Output(&s_l400_arm, &mix, out);
	L400_MotorWrite(out[0], out[1], out[2], out[3]);
}

// 目标角 0.01° → sin/cos Q15；目标角限幅在 ±30° 内，泰勒展开误差很小
static s32 l400_sin_q15(s32 ang_01)
{
	s32 r, r2, r3, r5;

	r  = (ang_01 * 5857) >> 10;                 // 弧度 Q15
	r2 = (r * r) >> 15;
	r3 = (r2 * r) >> 15;
	r5 = (r3 * r2) >> 15;
	return r - r3 / 6 + r5 / 120;
}

static s32 l400_cos_q15(s32 ang_01)
{
	s32 r, r2, r4, r6;

	r  = (ang_01 * 5857) >> 10;                 // 弧度 Q15
	r2 = (r * r) >> 15;
	r4 = (r2 * r2) >> 15;
	r6 = (r4 * r2) >> 15;
	return ATT_Q15_ONE - r2 / 2 + r4 / 24 - r6 / 720;
}

// 有效收包时换算：摇杆→目标姿态角、目标向上比力方向、偏航目标角速度。
// 200Hz 控制拍只做四元数重力方向与目标方向的叉积，不再每拍算三角函数。
static void L400_UpdateStickTargets(void)
{
	s32 roll_cmd = s_l400_cmd[3];
	s32 pitch_cmd = s_l400_cmd[2];
	s32 yaw_cmd = s_l400_cmd[1];
	u8 att_changed = !s_l400_tgt_valid
		|| roll_cmd != s_l400_stick_last[0]
		|| pitch_cmd != s_l400_stick_last[1];
	u8 yaw_changed = !s_l400_tgt_valid
		|| yaw_cmd != s_l400_stick_yaw_last;

	// 所有有效包都会走到这里刷新包龄/链路；只有杆量变化才做三角函数重算
	if (!att_changed && !yaw_changed) {
		return;
	}

	if (att_changed) {
		s32 roll, pitch, sr, cr, sp, cp;

		// 横滚杆 → 目标横滚角；俯仰杆推前为正，而欧拉俯仰正为抬头，所以取负
		roll = l400_clamp_s32(
			(roll_cmd * L400_MAX_TILT_DEG100) / L400_STICK_MAX,
			L400_MAX_TILT_DEG100);
		pitch = l400_clamp_s32(
			-(pitch_cmd * L400_MAX_TILT_DEG100) / L400_STICK_MAX,
			L400_MAX_TILT_DEG100);
		s_l400_tgt_att[0] = roll;
		s_l400_tgt_att[1] = pitch;

		// 目标向上比力（与静止加速度计读数同向）：
		// u = (sinθ, -sinφcosθ, -cosφcosθ)，Q15；重力方向与之相反
		sr = l400_sin_q15(roll);
		cr = l400_cos_q15(roll);
		sp = l400_sin_q15(pitch);
		cp = l400_cos_q15(pitch);
		s_l400_tgt_up[0] = sp;
		s_l400_tgt_up[1] = -((sr * cp) >> 15);
		s_l400_tgt_up[2] = -((cr * cp) >> 15);
		s_l400_stick_last[0] = roll_cmd;
		s_l400_stick_last[1] = pitch_cmd;
	}

	if (yaw_changed) {
		s_l400_tgt_yaw_rate = l400_clamp_s32(
			(yaw_cmd * L400_YAW_RATE_MAX_Q10) / L400_STICK_MAX,
			L400_YAW_RATE_MAX_Q10);
		s_l400_stick_yaw_last = yaw_cmd;
	}
	s_l400_tgt_valid = 1;
}

// 只清输出、保留已收到的摇杆目标；读失败/本拍无新姿态时用，
// 下一拍读数恢复后可立即按原目标继续控制。
static void L400_ControlOutputsClear(void)
{
	int i;

	s_l400_ctrl_ok = 0;
	for (i = 0; i < 3; i++) {
		s_l400_tgt_rate[i] = 0;
		s_l400_corr[i] = 0;
#if (L400_KI_Q16 != 0)
		s_l400_rate_integ[i] = 0;
#endif
#if (L400_KD_Q8 != 0)
		s_l400_rate_prev_err[i] = 0;
#endif
	}
#if (L400_KD_Q8 != 0)
	s_l400_rate_have_prev = 0;
#endif
}

// 失联、指令超时、姿态未就绪：输出禁止，并把摇杆目标一起清零
static void L400_ControlInhibit(void)
{
	L400_ControlOutputsClear();
	s_l400_tgt_att[0] = 0;
	s_l400_tgt_att[1] = 0;
	s_l400_tgt_up[0] = 0;
	s_l400_tgt_up[1] = 0;
	s_l400_tgt_up[2] = 0;
	s_l400_tgt_yaw_rate = 0;
	s_l400_tgt_valid = 0;       // 失联/超时后首个有效包必须重新换算
}

// 200Hz 控制：四元数推算的向上比力方向 vs 目标向上方向，叉积取横滚/俯仰误差；
// 目标角速度再进内环 PID（P=1/4，I/D 为 0 时对应状态更新被编译掉），电机仍停转。
static void L400_ControlUpdate(void)
{
	s32 q0, q1, q2, q3;
	s32 ax, ay, az;
	s32 ex, ey, tgt, err, p, corr;
#if (L400_KD_Q8 != 0)
	s32 d_err;
#endif
	int i;

	if (!s_l400_att.ready || !s_l400_link || !s_l400_cmd_ok) {
		L400_ControlInhibit();
		return;
	}

	q0 = s_l400_att.q[0];
	q1 = s_l400_att.q[1];
	q2 = s_l400_att.q[2];
	q3 = s_l400_att.q[3];

	// 四元数（机体→水平系）推算的向上比力方向，机体 FRD、Q15：
	// 与 UserEuler 的 Mahony 纠偏使用同一个表达式；拆开每项避免中间和溢出
	ax = ((q0 * q2) >> 14) - ((q1 * q3) >> 14);
	ay = -(((q0 * q1) >> 14) + ((q2 * q3) >> 14));
	az = (((q1 * q1) >> 14) + ((q2 * q2) >> 14)) - ATT_Q15_ONE;

	// 目标向上方向 × 实际向上方向，x/y 分量就是横滚/俯仰误差（Q15）；
	// 偏航分量不进外环，偏航杆仍直接给目标角速度。
	ex = ((s_l400_tgt_up[1] * az) >> 15) - ((s_l400_tgt_up[2] * ay) >> 15);
	ey = ((s_l400_tgt_up[2] * ax) >> 15) - ((s_l400_tgt_up[0] * az) >> 15);

	tgt = (ex * L400_ATT_KP_Q8) >> 13;
	s_l400_tgt_rate[0] = l400_clamp_s32(tgt, L400_RATE_LIMIT_Q10);
	tgt = (ey * L400_ATT_KP_Q8) >> 13;
	s_l400_tgt_rate[1] = l400_clamp_s32(tgt, L400_RATE_LIMIT_Q10);
	s_l400_tgt_rate[2] = s_l400_tgt_yaw_rate;

	// 内环 PID：误差 = 目标角速度 − 实测角速度；P=1/4
	for (i = 0; i < 3; i++) {
		err = s_l400_tgt_rate[i] - s_l400_rate[i];
		p = (err * L400_KP_Q8) >> 8;
		corr = p;

#if (L400_KI_Q16 != 0)
		s_l400_rate_integ[i] += err;
		s_l400_rate_integ[i] = l400_clamp_s32(s_l400_rate_integ[i],
											 L400_RATE_INT_MAX);
		corr += (s_l400_rate_integ[i] * L400_KI_Q16) >> 16;
#endif
#if (L400_KD_Q8 != 0)
		d_err = s_l400_rate_have_prev
				? (err - s_l400_rate_prev_err[i]) : 0;
		s_l400_rate_prev_err[i] = err;
		corr += (d_err * L400_KD_Q8) >> 8;
#endif
		s_l400_corr[i] = l400_clamp_s32(corr, L400_CORR_LIMIT_Q10);
	}
#if (L400_KD_Q8 != 0)
	s_l400_rate_have_prev = 1;
#endif
	s_l400_ctrl_ok = 1;
}

// tick → us（四舍五入）
static u32 l400_ticks_to_us(u32 ticks)
{
	return (ticks + L400_TICKS_PER_US / 2) / L400_TICKS_PER_US;
}

// 诊断段开始前清零分段统计；累计采样/姿态更新次数不清，方便看总进度
static void L400_DiagResetStats(void)
{
	s_l400_miss = 0;
	s_l400_ctrl_miss = 0;
	s_l400_skip = 0;
	s_l400_ctrl_max_us = 0;
	s_l400_late_sum_us = 0;
	s_l400_late_max_us = 0;
	s_l400_exec_sum_us = 0;
	s_l400_exec_max_us = 0;
	s_l400_prep_sum_us = 0;
	s_l400_prep_max_us = 0;
	s_l400_prep_cnt = 0;
	s_l400_eul_sum_us = 0;
	s_l400_eul_max_us = 0;
	s_l400_eul_cnt = 0;
	s_l400_rx_sum_us = 0;
	s_l400_rx_max_us = 0;
	s_l400_tx_sum_us = 0;
	s_l400_tx_max_us = 0;
	s_l400_tx_defer_budget = 0;
	s_l400_tx_defer_busy = 0;
	s_l400_tx_diag_due = 0;
	s_l400_tick_cnt = 0;
	s_l400_rx_cnt = 0;
	s_l400_tx_cnt = 0;
	s_l400_gap_max_tick = 0;
	s_l400_diag_rf_sum_ticks = 0;
	s_l400_diag_rf_max_ticks = 0;
	s_l400_diag_rf_cnt = 0;
	g_rf_irq_sum_ticks = 0;
	g_rf_irq_max_ticks = 0;
	g_rf_irq_cnt = 0;
}

// 把当前分段累计量折算成平均值/峰值，单位统一为 us
static void L400_DiagTake(l400_diag_t *d)
{
	u32 n = s_l400_tick_cnt;

	d->late_avg = n ? s_l400_late_sum_us / n : 0;
	d->late_max = s_l400_late_max_us;
	d->exec_avg = n ? s_l400_exec_sum_us / n : 0;
	d->exec_max = s_l400_exec_max_us;
	d->cmiss = s_l400_ctrl_miss;
	d->skip = s_l400_skip;

	if (g_rf_irq_cnt) {
		d->rfirq_avg = l400_ticks_to_us(g_rf_irq_sum_ticks / g_rf_irq_cnt);
	} else {
		d->rfirq_avg = 0;
	}
	d->rfirq_max = l400_ticks_to_us(g_rf_irq_max_ticks);

	if (s_l400_diag_rf_cnt) {
		d->rf_avg = l400_ticks_to_us(s_l400_diag_rf_sum_ticks / s_l400_diag_rf_cnt);
	} else {
		d->rf_avg = 0;
	}
	d->rf_max = l400_ticks_to_us(s_l400_diag_rf_max_ticks);

	d->app_avg = s_l400_rx_cnt ? s_l400_rx_sum_us / s_l400_rx_cnt : 0;
	d->app_max = s_l400_rx_max_us;
}

// 把一段结果格式化成一行并放入待发队列；真正发送由 L400_DiagFlush 逐行尝试
static void L400_DiagQueueSeg(char seg, const l400_diag_t *d)
{
	char buf[L400_DIAG_LINE_LEN];
	char *q = buf;

	q = l400_put_str(q, "L400D ");
	*q++ = seg;
	q = l400_put_str(q, " late=");
	q = l400_put_u32(q, d->late_avg);
	*q++ = '/';
	q = l400_put_u32(q, d->late_max);
	q = l400_put_str(q, " exec=");
	q = l400_put_u32(q, d->exec_avg);
	*q++ = '/';
	q = l400_put_u32(q, d->exec_max);
	q = l400_put_str(q, " cmiss=");
	q = l400_put_u32(q, d->cmiss);
	q = l400_put_str(q, " skip=");
	q = l400_put_u32(q, d->skip);
	q = l400_put_str(q, " rfirq=");
	q = l400_put_u32(q, d->rfirq_avg);
	*q++ = '/';
	q = l400_put_u32(q, d->rfirq_max);
	q = l400_put_str(q, " rf=");
	q = l400_put_u32(q, d->rf_avg);
	*q++ = '/';
	q = l400_put_u32(q, d->rf_max);
	q = l400_put_str(q, " app=");
	q = l400_put_u32(q, d->app_avg);
	*q++ = '/';
	q = l400_put_u32(q, d->app_max);
	*q++ = '\n';
	L400DiagTx_Enqueue(&s_l400_diag_tx, buf, (unsigned char)(q - buf));
}

// 中止行：保留中止发生的段、原因，以及 A 段是否已完成
static void L400_DiagQueueAbort(void)
{
	char buf[L400_DIAG_LINE_LEN];
	char *q = buf;

	q = l400_put_str(q, "L400D ABORT seg=");
	*q++ = s_l400_diag.abort_seg ? 'B' : 'A';
	q = l400_put_str(q, " reason=");
	q = l400_put_str(q, (s_l400_diag.abort_reason == L400_DIAG_REASON_CMD)
					 ? "cmd" : "link");
	q = l400_put_str(q, " haveA=");
	q = l400_put_u32(q, s_l400_diag_have_a);
	*q++ = '\n';
	L400DiagTx_Enqueue(&s_l400_diag_tx, buf, (unsigned char)(q - buf));
}

// UART 非阻塞发送回调：返回 0 表示忙，队列保留当前行
static u8 L400_DiagTrySend(const char *s, u8 len)
{
	return UartTrySend((unsigned char *)s, len);
}

// 每个 200Hz 拍推进纯逻辑状态机，并在事件点抓取当前段统计快照
static void L400_DiagPoll(u32 now)
{
	u8 valid = (s_l400_att.ready && s_l400_link && s_l400_cmd_ok) ? 1 : 0;
	u8 reason = !s_l400_link ? L400_DIAG_REASON_LINK
			  : (!s_l400_cmd_ok ? L400_DIAG_REASON_CMD
								: L400_DIAG_REASON_NONE);
	unsigned int ev = L400Diag_Poll(&s_l400_diag, now, valid, reason);

	if (ev & L400_DIAG_EV_START_A) {
		s_l400_diag_active = 1;
		s_l400_diag_ctrl_enable = 0;
		s_l400_diag_have_a = 0;
		L400_MotorStop(L400_LOCK_DIAG);
		L400_ControlOutputsClear();
		L400_DiagResetStats();
		g_rf_irq_meas_en = 1;
	}
	if (ev & L400_DIAG_EV_A_DONE) {
		L400_DiagTake(&s_l400_diag_a);
		s_l400_diag_have_a = 1;
		s_l400_diag_ctrl_enable = 1;
		L400_DiagResetStats();
	}
	if (ev & L400_DIAG_EV_B_DONE) {
		L400_DiagTake(&s_l400_diag_b);
		g_rf_irq_meas_en = 0;
		L400_DiagQueueSeg('A', &s_l400_diag_a);
		L400_DiagQueueSeg('B', &s_l400_diag_b);
		s_l400_diag_ctrl_enable = 1;
		L400_DiagResetStats();
	}
	if (ev & L400_DIAG_EV_ABORT) {
		g_rf_irq_meas_en = 0;
		if (s_l400_diag_have_a) {
			L400_DiagQueueSeg('A', &s_l400_diag_a);
		}
		L400_DiagQueueAbort();
		s_l400_diag_ctrl_enable = 1;
		L400_DiagResetStats();
	}
}

// 常规报告延期原因短行：只在 1s 节流后尝试，失败保留 due
static u8 L400_SendTxDiag(u32 now)
{
	char buf[48];
	char *q = buf;

	if (!s_l400_tx_diag_due
		|| (s32)(now - s_l400_tx_diag_tick) < 0) {
		return 0;
	}
	q = l400_put_str(q, "L400T budget=");
	q = l400_put_u32(q, s_l400_tx_defer_budget);
	q = l400_put_str(q, " busy=");
	q = l400_put_u32(q, s_l400_tx_defer_busy);
	*q++ = '\n';
	if (!UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		return 0;
	}
	s_l400_tx_diag_due = 0;
	s_l400_tx_diag_tick = now + CLOCK_SYS_CLOCK_HZ;
	return 1;
}

// 每秒一行 prep/Euler_Update 拆分耗时；发送成功后开始新窗口
static u8 L400_SendPrep(u32 now)
{
	char buf[72];
	char *q = buf;
	u32 prep_avg = s_l400_prep_cnt ? s_l400_prep_sum_us / s_l400_prep_cnt : 0;
	u32 eul_avg = s_l400_eul_cnt ? s_l400_eul_sum_us / s_l400_eul_cnt : 0;

	if ((s32)(now - s_l400_prep_tick) < 0) {
		return 0;
	}

	q = l400_put_str(q, "L400P prep=");
	q = l400_put_u32(q, prep_avg);
	*q++ = '/';
	q = l400_put_u32(q, s_l400_prep_max_us);
	q = l400_put_str(q, " eul=");
	q = l400_put_u32(q, eul_avg);
	*q++ = '/';
	q = l400_put_u32(q, s_l400_eul_max_us);
	q = l400_put_str(q, " n=");
	q = l400_put_u32(q, s_l400_prep_cnt);
	q = l400_put_str(q, " e=");
	q = l400_put_u32(q, s_l400_eul_cnt);
	*q++ = '\n';
	if (!UartTrySend((unsigned char *)buf, (u8)(q - buf))) {
		return 0;
	}

	s_l400_prep_sum_us = 0;
	s_l400_prep_max_us = 0;
	s_l400_prep_cnt = 0;
	s_l400_eul_sum_us = 0;
	s_l400_eul_max_us = 0;
	s_l400_eul_cnt = 0;
	s_l400_prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	return 1;
}

// 汇总行分开发送：UartTrySend 忙时保留；全部发完后恢复正常遥测
static void L400_DiagFlush(u32 now)
{
	L400DiagTx_Flush(&s_l400_diag_tx, L400_DiagTrySend);

	if (s_l400_diag_active && s_l400_diag_tx.count == 0
		&& (s_l400_diag.state == L400_DIAG_STATE_DONE
			|| s_l400_diag.state == L400_DIAG_STATE_ABORT)) {
		s_l400_diag_active = 0;
		s_l400_diag_enabled = 0;
		s_l400_tx_tick = now + L400_REPORT_TICKS;
		s_l400_stat_tick = now + CLOCK_SYS_CLOCK_HZ;
		s_l400_prep_tick = now + CLOCK_SYS_CLOCK_HZ;
	}
}

// d24_rx_main_loop 分段计时；只在诊断段累计
static void L400_DiagRecordRfLoop(u32 ticks)
{
	if (!s_l400_diag_active) {
		return;
	}
	s_l400_diag_rf_sum_ticks += ticks;
	if (ticks > s_l400_diag_rf_max_ticks) {
		s_l400_diag_rf_max_ticks = ticks;
	}
	s_l400_diag_rf_cnt++;
}

static void L400_Process(void)
{
	int16_t raw_a[3], raw_g[3];
	att_i32 acc_frd[3], acc_mg[3], gyr_q10[3];
	u32 now, planned, t_sample0, t_exec1, t_read1 = 0, end, total;
	u32 t_prep1 = 0, t_eul1 = 0;
	u32 late_us, exec_us, ctrl_us, ctrl_total;
	u32 dt_ticks = 0;
	att_i32 dt_100us = 0;
	u8 att_updated = 0;
	u8 prep_timed = 0;
	u8 eul_timed = 0;
	int i;

	if (!s_l400_inited) {
		L400_Init();
		return;
	}

	now = clock_time();
	if ((s32)(now - s_l400_next_tick) < 0) {
		return;
	}

	// 原定拍点：截止超时和跳拍都从这里起算，包含主循环迟到的时间
	planned = s_l400_next_tick;
	s_l400_next_tick = planned + L400_PERIOD_TICKS;
	{
		u32 late = now - planned;

		if (late >= L400_PERIOD_TICKS) {
			// 迟到超过一个周期：已经错过的拍直接丢弃，从当前时刻重排
			s_l400_skip += late / L400_PERIOD_TICKS;
			s_l400_next_tick = now + L400_PERIOD_TICKS;
		}
	}

	// 每拍检查两种有效期：全部用原始 tick 比较，不经过毫秒截断
	if (s_l400_have_link
		&& (u32)(now - s_l400_link_tick) > L400_LINK_TIMEOUT_TICKS) {
		s_l400_link = 0;
	}
	if (s_l400_have_pkt
		&& (u32)(now - s_l400_pkt_tick) > L400_CMD_TIMEOUT_TICKS) {
		s_l400_cmd_ok = 0;
	}
	// 链路/指令已经无效时，不等 IMU 读数成功也先把目标与输出清零
	if (!s_l400_cmd_ok || !s_l400_link) {
		L400_ControlInhibit();
	}
	if (!s_l400_link) {
		L400_MotorStop(L400_LOCK_LINK);
	} else if (!s_l400_cmd_ok) {
		L400_MotorStop(L400_LOCK_CMD);
	}
	// A/B 诊断只在台架按键开启后运行；未开启或等待无有效状态时不逐拍调用
	if (s_l400_diag_enabled
		&& (s_l400_diag.state == L400_DIAG_STATE_WAIT
			|| s_l400_diag.state == L400_DIAG_STATE_A
			|| s_l400_diag.state == L400_DIAG_STATE_B)) {
		u8 diag_valid = (s_l400_att.ready && s_l400_link
						 && s_l400_cmd_ok) ? 1 : 0;

		if (s_l400_diag_active || s_l400_diag.valid_seen || diag_valid) {
			L400_DiagPoll(now);
		}
	}
	if (s_l400_diag_tx.count) {
		L400_DiagFlush(now);
	}

	// 实际采样开始：从原定拍点到这里就是主循环调度迟到量；
	// 后面 t_exec1 - t_sample0 是 IMU + 估计 + P 环的执行时间。
	t_sample0 = clock_time();
	late_us = (t_sample0 - planned) / L400_TICKS_PER_US;
	l400_add_timing(&s_l400_late_sum_us, &s_l400_late_max_us, late_us);
	s_l400_tick_cnt++;

	// 读一次 12 字节；失败不发布零值，只累计失败数，本拍仍走完计时与报告
	if (t3488_read_6axis(raw_a, raw_g)) {
		s_l400_fail++;
		t_prep1 = clock_time();
		prep_timed = 1;
		L400_ControlOutputsClear(); // 本拍没有新角速度：只清输出，保留摇杆目标
		L400_MotorStop(L400_LOCK_ATT);
	} else {
		s_l400_sample++;
		t_read1 = clock_time();

		// 映射到机体 FRD（实测符号置换），再扣零偏/校准
		acc_frd[0] = (att_i32)raw_a[0];
		acc_frd[1] = -(att_i32)raw_a[1];
		acc_frd[2] = -(att_i32)raw_a[2];
		gyr_q10[0] = Euler_GyroToQ10((att_i32)raw_g[0], s_bench_gyro_bias_q6[0],
									 &s_l400_gyro_rem[0]);
		gyr_q10[1] = Euler_GyroToQ10(-(att_i32)raw_g[1], s_bench_gyro_bias_q6[1],
									 &s_l400_gyro_rem[1]);
		gyr_q10[2] = Euler_GyroToQ10(-(att_i32)raw_g[2], s_bench_gyro_bias_q6[2],
									 &s_l400_gyro_rem[2]);
		for (i = 0; i < 3; i++) {
			att_i32 lsb = (acc_frd[i] - s_bench_acc_off_lsb[i])
						  * s_bench_acc_scale_q14[i] >> 14;

			acc_mg[i] = (lsb * 125) >> 8;                       // LSB → mg：1000/2048
			s_l400_rate[i] = gyr_q10[i];                        // 扣偏后角速度
		}

		// 时间差以上一次成功读取的完成时刻为基准，u32 相减天然处理回绕
		if (!s_l400_have_ts) {
			s_l400_last_ts = t_read1;
			s_l400_have_ts = 1;
			dt_ticks = 0;
		} else {
			dt_ticks = t_read1 - s_l400_last_ts;
			s_l400_last_ts = t_read1;
		}

		// 长间断：重建积分端点，不补算缺失区间（与台架固件一致）
		if (dt_ticks > (u32)C_ATT_MAX_GAP_100US * L400_TICKS_PER_100US) {
			s_l400_dt_rem = 0;
			s_l400_att.have_prev = 0;
			L400_StillReset();
			dt_ticks = 0;
		}

		// 积分 dt 用 0.1ms：tick 余数留到下一拍，避免每拍向下截断
		if (dt_ticks) {
			s_l400_dt_rem += dt_ticks;
			dt_100us = (att_i32)(s_l400_dt_rem / L400_TICKS_PER_100US);
			s_l400_dt_rem -= (u32)dt_100us * L400_TICKS_PER_100US;
		}

		// 加速度一阶低通：在 FRD 校准之后做，α 只在 dt 变化超过 50us 时重算
		if (!s_l400_lpf_ok) {
			s_l400_lpf[0] = acc_mg[0];
			s_l400_lpf[1] = acc_mg[1];
			s_l400_lpf[2] = acc_mg[2];
			s_l400_lpf_ok = 1;
		} else if (C_ACC_LPF_EN && dt_ticks > 0) {
			u32 dt_us = (dt_ticks + L400_TICKS_PER_US / 2) / L400_TICKS_PER_US;
			att_i32 alpha;

			if (dt_us + 50 < s_l400_lpf_alpha_dt_us
				|| dt_us > s_l400_lpf_alpha_dt_us + 50) {
				s_l400_lpf_alpha = (u32)((dt_us * 32768u) / (C_ACC_LPF_TAU_US + dt_us));
				s_l400_lpf_alpha_dt_us = dt_us;
			}
			alpha = (att_i32)s_l400_lpf_alpha;
			for (i = 0; i < 3; i++) {
				s_l400_lpf[i] += (alpha * (acc_mg[i] - s_l400_lpf[i])) >> 15;
			}
		} else if (!C_ACC_LPF_EN) {
			s_l400_lpf[0] = acc_mg[0];
			s_l400_lpf[1] = acc_mg[1];
			s_l400_lpf[2] = acc_mg[2];
		}

		// 启动静止判定 / 姿态更新：与台架固件同一套逻辑
		if (!s_l400_att.ready) {
			L400_Startup(s_l400_lpf, gyr_q10, dt_ticks, t_read1);
			t_prep1 = clock_time();
			prep_timed = 1;
		} else if (dt_100us > 0) {
			// prep 到此为止；eul 只包住 Euler_Update，便于判断优化落点
			t_prep1 = clock_time();
			prep_timed = 1;
			if (Euler_Update(&s_l400_att, s_l400_lpf, gyr_q10, dt_100us,
							 t_read1 / L400_TICKS_PER_MS)) {
				s_l400_euler_ok++;
				s_l400_att_ever_ready = 1;
				att_updated = 1;
			}
			t_eul1 = clock_time();
			eul_timed = 1;
		} else {
			t_prep1 = clock_time();
			prep_timed = 1;
		}

		// 姿态本拍有更新才跑控制；启动、长间断或更新失败都先禁止输出
		if (att_updated) {
			if (s_l400_diag_ctrl_enable) {
				L400_ControlUpdate();
			} else {
				L400_ControlOutputsClear();   // A 段只跑姿态，不开双环
			}
		} else {
			// 启动/长间断/更新失败：本拍无新姿态，先清输出但保留目标
			L400_ControlOutputsClear();
		}
	}

	// 拆分统计：prep=采样→Euler 之前，eul=只包住 Euler_Update
	if (prep_timed) {
		u32 prep_us = (t_prep1 - t_sample0) / L400_TICKS_PER_US;

		l400_add_timing(&s_l400_prep_sum_us, &s_l400_prep_max_us, prep_us);
		s_l400_prep_cnt++;
	}
	if (eul_timed) {
		u32 eul_us = (t_eul1 - t_prep1) / L400_TICKS_PER_US;

		l400_add_timing(&s_l400_eul_sum_us, &s_l400_eul_max_us, eul_us);
		s_l400_eul_cnt++;
	}

	// 电机安全门：失联/指令超时/姿态未更新/诊断段一律先锁四路；
	// 全部有效后才允许按键解锁和混控输出。
	if (!s_l400_link) {
		L400_MotorStop(L400_LOCK_LINK);
	} else if (!s_l400_cmd_ok) {
		L400_MotorStop(L400_LOCK_CMD);
	} else if (!att_updated) {
		L400_MotorStop(s_l400_att_ever_ready
					   ? L400_LOCK_ATT : L400_LOCK_BOOT);
	} else if (!s_l400_ctrl_ok) {
		L400_MotorStop(L400_LOCK_ATT);
	} else if (s_l400_diag_enabled) {
		L400_MotorStop(L400_LOCK_DIAG);
	}
#if (!C_L400_KEY_CAPTURE)
	L400_MotorUpdate();
#endif

	// 控制完成时刻：包含 IMU 读取、标定/低通、姿态更新、控制混控；
	// 不含后面的串口报告，因此可单独统计“控制完成超期”。
	t_exec1 = clock_time();
	exec_us = (t_exec1 - t_sample0) / L400_TICKS_PER_US;
	l400_add_timing(&s_l400_exec_sum_us, &s_l400_exec_max_us, exec_us);

	ctrl_total = t_exec1 - planned;
	ctrl_us = ctrl_total / L400_TICKS_PER_US;
	if (ctrl_us > s_l400_ctrl_max_us) {
		s_l400_ctrl_max_us = ctrl_us;
	}
	if (ctrl_us > s_l400_ctrl_peak_us) {
		s_l400_ctrl_peak_us = ctrl_us;
	}
	if (ctrl_total > L400_PERIOD_TICKS) {
		s_l400_ctrl_miss++;       // 控制计算完成已越过下一拍原定拍点
	}

	// 低频串口输出：200Hz 到点先保证采样/控制，报告只在本拍余量足够时
	// 非阻塞提交；余量不足或串口忙就延期到下一拍，绝不等待 DMA。
	// A/B 诊断段暂缓所有常规格式化，结束后由待发队列逐行输出。
	if (!s_l400_diag_active) {
		u32 t_tx0 = clock_time();
		u32 t_tx1;
		s32 remain_ticks = (s32)(s_l400_next_tick - t_tx0);
		u8 attempted_stat = 0;
		u8 attempted_obs = 0;
		u8 stat_due = ((s32)(t_tx0 - s_l400_stat_tick) >= 0) ? 1 : 0;
		u8 main_due = ((s32)(t_tx0 - s_l400_tx_tick) >= 0) ? 1 : 0;
		u8 stat_ok = (remain_ticks >= (s32)((s_l400_stat_cost_us
						+ L400_TX_MARGIN_US) * L400_TICKS_PER_US)) ? 1 : 0;
		u8 obs_ok = (remain_ticks >= (s32)((s_l400_obs_cost_us
					   + L400_TX_MARGIN_US) * L400_TICKS_PER_US)) ? 1 : 0;

		if (stat_due && stat_ok) {
			// 统计行预算够：统计优先，观察行让位
			attempted_stat = 1;
			if (L400_SendStats(t_tx0)) {
				s_l400_stat_deferred = 0;
				s_l400_tx_deferred = 0;
			} else {
				l400_note_tx_defer(&s_l400_stat_deferred, 1);
			}
		} else if (stat_due) {
			// 统计行放不下：先记余量不足；若观察行也到点且放得下，先发短观测
			l400_note_tx_defer(&s_l400_stat_deferred, 0);
			if (main_due && obs_ok) {
				attempted_obs = 1;
				if (L400_SendObs(t_tx0)) {
					s_l400_tx_deferred = 0;
				} else {
					l400_note_tx_defer(&s_l400_tx_deferred, 1);
				}
			}
		} else if (main_due) {
			if (obs_ok) {
				attempted_obs = 1;
				if (L400_SendObs(t_tx0)) {
					s_l400_tx_deferred = 0;
				} else {
					l400_note_tx_defer(&s_l400_tx_deferred, 1);
				}
			} else {
				l400_note_tx_defer(&s_l400_tx_deferred, 0);
			}
		}

		// 实测本次组帧+提交耗时，分别更新观测/统计预算；只统计真正拼过文本的拍
		t_tx1 = clock_time();
		{
			u32 dt_us = (t_tx1 - t_tx0) / L400_TICKS_PER_US;

			if (attempted_stat) {
				u32 cost = (u32)l400_clamp_s32((s32)dt_us, 1000);

				if (cost < 10) {
					cost = 10;
				}
				s_l400_stat_cost_us = cost;
				l400_add_timing(&s_l400_tx_sum_us, &s_l400_tx_max_us, dt_us);
				s_l400_tx_cnt++;
			}
			if (attempted_obs) {
				u32 cost = (u32)l400_clamp_s32((s32)dt_us, 1000);

				if (cost < 10) {
					cost = 10;
				}
				s_l400_obs_cost_us = cost;
				l400_add_timing(&s_l400_tx_sum_us, &s_l400_tx_max_us, dt_us);
				s_l400_tx_cnt++;
			}
		}

		// 本拍没有常规报告占用时，补发 prep/Euler 拆分耗时或延期原因短状态
		if (!attempted_stat && !attempted_obs
			&& (obs_ok || remain_ticks >= (s32)((u32)200 * L400_TICKS_PER_US))) {
			L400_SendPrep(t_tx0);
			L400_SendTxDiag(t_tx0);
		}
	}

	// 整拍统计：原定拍点→本拍全部处理（含串口报告）完成
	end = clock_time();
	total = end - planned;
	if (total > L400_PERIOD_TICKS) {
		s_l400_miss++;             // 含串口报告的整拍完成已越过下一拍
	}
}
#endif

//**************************************************************
void User_main_loop_process(void)
{
	wd_32k_feed();

#if (C_ATT_BENCH_EN)
#if (C_RC_LOOP_400_EN)
	// 200Hz 到点时先执行采样/控制；射频中断仍在后台收包，
	// d24_rx_main_loop 紧随其后处理配对/ACK，应用层收包放到控制之后。
	L400_Process();
	if (s_l400_diag_active) {
		u32 t_rf0 = clock_time();

		d24_rx_main_loop();
		L400_DiagRecordRfLoop(clock_time() - t_rf0);
	} else {
		d24_rx_main_loop();
	}
	TimeBaseGenerate();
	car_work();
	return;
#elif (C_RC_BRINGUP_EN)
	// 遥控联调入口：2.4G 收包 + 通道映射 + 低频串口观察，电机保持停转
	d24_rx_main_loop();
	TimeBaseGenerate();          // 10ms 时基，失联计数在这里累加
	car_work();                  // 现有收包处理：解码原始通道与按键
	RcBringup_Update();          // 更新映射后的四轴指令
	RcBringup_Report();          // 约 5Hz 输出观察行
	return;
#elif (C_MAINT_269A_EN)
	// 地面维护入口：只读 T-3488 原始六轴 + 0x269A 二进制帧（校准用）
	Maint_Process();
#else
	// 台架姿态估计入口：只读 T-3488 + 串口文本输出
	// 不进入旧遥控解码、姿态控制、混控、气压高度等路径（电机保持停转）
	BenchAtt_Process();
#endif
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

