//*******************************************************************
/*
 * UserRxdFunc.h
 *
 *  Created on: 2026年8月20日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USERRXDFUNC_H_
#define VENDOR_ROBIN_24G_CAR_USERRXDFUNC_H_
//*******************************************************************
//*******************************************************************
//*******************************************************************
#define Project_XH261009_Rxd		1
//*******************************************************************
//*******************************************************************
#define Audio_Send_Data         0		//需要与语音IC通信时置1

//*******************************************************************
//*******************************************************************
#if(Project_XH261009_Rxd)
//*******************************************************************
//Pin define
//Robin Flash SOP16 Pins
//*************************************************************************
//	ANT			--					XC0			--
//	GND			--					XC1			--
//	PA5(ADC4)	LFmotPwm_Pin		PA7			LBmotPwm_Pin
//	PA3(SWS)	SWSLed				PA6			RFmotPwm_Pin
//	PA4(ADC3)	RBmotPwm_Pin		PB2(ADC2)	LPDadc_Pin
//	PA2			PowerLock_Pin		PB1(ADC1)	Isen1_Pin
//	PA1			IIC_SCL				PB0(ADC0)	Isen2_Pin
//	PA0			IIC_SDA				VCC			--
//*************************************************************************
// =================================================
//for Project_XH261009 四轴飞行器
// =================================================
// =================================================
//电源管理
#define PowerLock_Pin		GPIO_PA2		//系统电源控制，上电后输出高电平，不操作5分钟输出低电平关机
#define LPDadc_Pin			GPIO_PB2		//低电压检测，ADC2
#define Isen1_Pin			GPIO_PB1		//大电流检测1，ADC1
#define Isen2_Pin			GPIO_PB0		//大电流检测2，ADC0

//马达PWM
#define LFmotPwm_Pin     	GPIO_PA5		//Pwm1, to motor driver 前
#define RBmotPwm_Pin     	GPIO_PA4		//Pwm0, to motor driver 左
#define	LBmotPwm_Pin    	GPIO_PA7		//Pwm3, to motor driver	后
#define RFmotPwm_Pin		GPIO_PA6		//Pwm2, to motor driver	右

//Speech IC

//IIC for 陀螺仪
#define IIC_SDA_Pin			GPIO_PA0
#define IIC_SCL_Pin			GPIO_PA1

//LED
#define SWSLed_Pin			GPIO_PA3		//SWS, 车体前灯,Low active

#define Testing_Pin		PowerLock_Pin

//*************************************************************************
//*************************************************************************
#if(Audio_Send_Data)
//*************************************************************************
//#define PIN_TIME0_OUT_1    SpeechCtrl
//
//#define SEND_START_LOW_US 5000
//#define SEND_BIT1_HIGH_US 1500
//#define SEND_BIT1_LOW_US 500
//#define SEND_BIT0_HIGH_US 500
//#define SEND_BIT0_LOW_US 1500
//
////for vice-mask
//#define RX_voice_1 0x01    //LED常亮
//#define RX_voice_2  0x02    //LED灭
//#define RX_voice_3 0x03	  //LED3hz闪
//#define RX_voice_4 0x04	  //吐舌声音
//#define RX_voice_5 0x05	  //警戒声音
//
//typedef enum {
//	LedState_Null = 0,
//	LedState_Light = 1,
//    LedState_Dark = 2,
//	LedState_Flash = 3
//} Led_Status_t;		//for u8LedStatus
//
//
//typedef enum {
//    SEND_STATE_IDLE = 0,
//    SEND_STATE_Delay,
//    SEND_STATE_START_LOW,
//    SEND_STATE_HIGH,
//    SEND_STATE_LOW
//} send_state_t;
//
//extern send_state_t send_state ;
//extern unsigned char g_send_data;
//extern unsigned char send_bit_index;
//extern unsigned char voic_handleflag;		//=1正在发送语音命令；=0已经发送完成
//extern unsigned short	u16VoiceSentCnt;	//从语音命令发送完毕开始计时
//extern Led_Status_t u8LedStatus;
//
//extern void Send_Voice_data(unsigned char audio_data);
//extern void set_timer0_capture_us(unsigned int us);
//void SendVoiceDelay(void);
//void SendTurnOffSpeechIC_cmd(void);
//void StoreVoiceNumBak(unsigned char VoiceID);		//保存播放的语音ID号
//void RandomPlayVoice(void);		//随机播放一段语音
//void VoicePlayHandler(void);		//10mS loop
//
//void SendLedCtrlCmd(void);
//void LedStatusHandler(void);		//10mS loop
//
//
//*************************************************************************
#endif		//Audio_Send_Data
//*************************************************************************
//*************************************************************************
//typedef enum{
//	RfSendKey_Released=0,       //按键释放
//	RfSendKey_Xspeed=0x01,		//速度转换（高速/低速）
//	RfSendKey_OneKRun=0x02,		//一键起降
//	RfSendKey_StuntShow=0x04,	//特技表演
//	RfSendKey_MicroAdj=0x08,	//微调模式
//	RfSendKey_Headless=0x10,	//无头模式
//	RfSendKey_LAST,
//} RfSend_Keyval_def;		//for g_u8KeyVal_Buffer

#define RfSendKey_Released 	0        //按键释放
#define RfSendKey_Xspeed	1		//速度转换（高速/低速）
#define RfSendKey_OneKRun	2		//一键起降
#define RfSendKey_StuntShow	4		//特技表演
#define RfSendKey_MicroAdj	8		//微调模式
#define RfSendKey_Headless	0x10	//无头模式

// Define key bitmask, for key program
#define cAllKeysMask		0x1F

extern unsigned short u16RemotorKeyVal;
//**********************************************************************************
//实际电位器装配电气参数方向:
//速度（油门拉）增大 ==> Adc值变大
//旋转向右 ==> 	Adc值变小
//前后平移向后 ==>	Adc值变小
//左右平移向左 ==>	Adc值变小
//**********************************************************************************
//油门为左摇杆上下移动杆：管控本飞行器的上下移动速度；左摇杆上下移动杆处于释放状态时，飞行器悬停;
//						上推则向上移动（推动幅度与移动速度对应），下拉则向下移动推动幅度与移动速度对应）；
//(左摇杆左右移动杆)Rotation旋转（对应偏航）：管控本飞行器的机头方向；左摇杆左右移动杆处于释放状态时，飞行器机头方向不变;
//						左推则机头方向向左旋转（推动幅度与旋转速度对应），右推则机头方向向右旋转（推动幅度与旋转速度对应）；
//*******************************************************************
//通道				功能			范围				中心值		映射目标
//Throttle			油门			0x400~0x1C00	0x1000		目标相对升降速度		±12cm/S（满杆 1/8）
//Rotation			旋转			0x400~0x1C00	0x1000		目标对应偏航角速度 	±30°/S
//FBmove			前后平移		0x400~0x1C00	0x1000		目标俯仰角 ±30°
//LRmove			左右平移		0x400~0x1C00	0x1000		目标滚转角 ±30°
//*******************************************************************
#define RfDataMaxValue	0x1C00
#define RfDataMinValue	0x0400
#define RfDataMiddle	0x1000

extern volatile unsigned short	u16Throttle_Rxd;		//Remotor speed data value for send
extern volatile unsigned short	u16Rotation_Rxd;		//Remotor rotation data value for send
extern volatile unsigned short	u16FBmove_Rxd;		//Remotor FBmove data value for send
extern volatile unsigned short	u16LRmove_Rxd;		//Remotor LRmove data value for send
//**********************************************************************************
//**********************************************************************************
#define	Light_SwsLed	ChangePA3ToGPIO_OutHigh()
#define Dark_SwsLed		ChangePA3ToGPIO_OutLow()
#define TurnOffPower()	gpio_write(PowerLock_Pin, 0)
//*******************************************************************
//*******************************************************************
extern unsigned char T250uS_Cnt, T2mS, R_2ms_Cnt;
extern unsigned char T10mSflag, T500mSflag, Sys10mScnt;
extern unsigned int	u32_SysCount, u32_Sys10mScnt, u32_Sys500mScnt;
extern unsigned char rf_received_type, rf_received_key, testPoint;
extern unsigned char UserRfRcvBuffer[20];
extern unsigned char RxdNewFlag;	//收到新数据

extern unsigned char    link_ok_flag;

//extern unsigned char	LargeCurrentFlag;
extern unsigned char	FunCtrlFlag;
#define WorkingStatus	0x01	//: FunCtrlFlag.0; =1 for system working
#define CarDemoFlag		0x02	//: FunCtrlFlag.1; //特技表演,RfSendKey_StuntShow
#define CarSpeedFlag	0x04	//: FunCtrlFlag.2; speed low=0; speed high=1;
#define CarLightFlag	0x08	//: FunCtrlFlag.3; Light on =1;
#define CarOneKRun		0x10	//: FunCtrlFlag.4; //一键起降
#define CarHeadless		0x20	//: FunCtrlFlag.5; //无头模式
#define CarMicroAdj		0x40	//: FunCtrlFlag.6; //微调模式

extern unsigned char	DemoProcess, SubProcess;
//extern unsigned char u8VoiceStChkDelayCnt;

//*******************************************************************
void User_hw_init(void);
void User_hw_Exit(void);

void ChangePA3ToSWS(void);
void ChangePA3ToGPIO_OutLow(void);
void ChangePA3ToGPIO_OutHigh(void);
void T250uSInterrupt(void);
void T2000uSInterrupt(void);
void NoActionCounter(void);
void system_enter_sleep(void);
//void GetUsb5VinStatus(void);
//void CheckChargeStatus(void);

void GetBatteryVoltage(void);	//运行一次用时280uS
//void LargeCurrentProtect(void);
//void ChkLargeCurrentProtExit(void);
//void OverCurrentHandler(void);	//10mS loop
//void AtomizerChecker(void);	//10mS loop


void InitalRam(void);

void car_work(void);

extern unsigned short u16RfSpeedVal;
extern unsigned short u16RfDirectionVal;
extern unsigned short u16DirectionAssist;	//漂移辅助输出（叠加反打修正后的方向值）
void LedScanControl(void);		//2mS loop
//void SetLedPins(void);		//10mS loop
void PairingDisplay(void);		//配对显示：配对中闪烁，匹配完成常亮

void CarDemoHandler(void);	//10mS loop

void TimeBaseGenerate(void);
void UserPowOnStart(void);
void User_main_loop_process(void);

//*******************************************************************
#endif	//Project_XH261009_Rxd
//*******************************************************************



//*******************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USERRXDFUNC_H_ */
//*******************************************************************


