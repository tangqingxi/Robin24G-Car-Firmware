//*******************************************************************
/*
 * UserTxdFunc.h
 *
 *  Created on: 2026年8月19日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_REMOTE_USERTXDFUNC_H_
#define VENDOR_ROBIN_24G_REMOTE_USERTXDFUNC_H_
//*******************************************************************
//*******************************************************************
#define Project_XH260909_Remotor		1
//*******************************************************************
//*******************************************************************
#define Audio_Send_Data         0
#define Used_ADC                0
//*******************************************************************
//*******************************************************************
//*******************************************************************
#if(Project_XH260909_Remotor)
//*******************************************************************
//========================================================================================================
// 引脚分配 (按《漂移车HX260909 TX》原理图)
//遥控器：
//三个按键：分别为On/Off、Speed（Low/High）、Light
//4个电位器：Throttle（油门）、Direction（方向_Gyroscope）、Range（调节舵机幅度）、Middle（舵机中点调节）
//LED指示灯：1枚，配对指示（RF连接前闪烁、连接后常亮，电量不足断闪）
//========================================================================================================
//Pin define
//Robin Flash SOP16 Pins
//*************************************************************************
//	ANT			--					XC0			--
//	GND			--					XC1			--
//	PA5(ADC4)	LRmoveAdc_Pin		PA7			KeyCom_Pin
//	PA3(SWS)	SWS_PIN				PA6			PotPowLed_Pin
//	PA4(ADC3)	FBmoveAdc_Pin		PB2(ADC2)	Throttle_Pin
//	PA2			KeySige2_Pin		PB1(ADC1)	Rotation_Pin
//	PA1			KeySige3_Pin		PB0(ADC0)	KeySige1_Pin
//	PA0			PwmBz_Pin			VCC			--
//*************************************************************************
//	按键6个, 2*3键阵，分别是KeyCom_Pin、Gnd、KeySige1_Pin、KeySige2_Pin、KeySige3_Pin
//	(KeyCom_Pin, KeySige1_Pin) 	=> OneKeyRun		一键起降(长按)
//	(Gnd, KeySige1_Pin) 		=> ChangeSpeed		速度转换（高速/低速）
//	(KeyCom_Pin, KeySige2_Pin) 	=> MicroAdjMode		微调模式
//	(Gnd, KeySige2_Pin) 		=> StuntShow		特技表演
//	(KeyCom_Pin, KeySige3_Pin) 	=> HeadlessMode		无头模式(长按)
//	(Gnd, KeySige3_Pin) 		=> OnOffKey			On/Off

//========================================================================================================
//---------- 按键 ----------
#define KeyCom_Pin    	GPIO_PA7   	//
#define KeySige1_Pin    GPIO_PB0   	//
#define KeySige2_Pin    GPIO_PA2   	//
#define KeySige3_Pin    GPIO_PA1   	//

//---------- 指示灯\蜂鸣器 ----------
#define PotPowLed_Pin    GPIO_PA6   //对码灯 (配对前1.7HZ50%闪，配对后常亮)/电位器电源
#define PwmBz_Pin        GPIO_PA0   //蜂鸣器（2KHZ驱动频率）

//---------- 电位器 ----------
//油门为左摇杆上下移动杆：管控本飞行器的上下移动速度；左摇杆上下移动杆处于释放状态时，飞行器悬停;
//						上推则向上移动（推动幅度与移动速度对应），下拉则向下移动推动幅度与移动速度对应）；
//(左摇杆左右移动杆)Rotation旋转（对应偏航）：管控本飞行器的机头方向；左摇杆左右移动杆处于释放状态时，飞行器机头方向不变;
//						左推则机头方向向左旋转（推动幅度与旋转速度对应），右推则机头方向向右旋转（推动幅度与旋转速度对应）；
#define Throttle_Pin     GPIO_PB2   //Throttle（油门）,油门为左摇杆上下移动杆：管控本飞行器的上下移动速度
#define Rotation_Pin     GPIO_PB1   //
#define FBmoveAdc_Pin    GPIO_PA4   //前后平移
#define LRmoveAdc_Pin    GPIO_PA5   //左右平移

//---------- 烧录/调试口 ----------
#define SWS_PIN          GPIO_PA3   //SWS (默认 AS_SWIRE, 不用作 GPIO)

//---------- 兼容旧命名 (与 HspdCar 框架保持符号一致) ----------
#define LED_Ctrl         PotPowLed_Pin
#define KeyIn0           KeySige1_Pin
#define KeyIn1           KeySige2_Pin
#define KeyIn2           KeySige3_Pin
#define VR_POWER_PIN	PotPowLed_Pin

#define Testing          GPIO_PA0	  //gpio_write(Testing) 测试点
#define Testing_Pin		Testing

#define PA0_forTesting	1
#define PA0_forPwmBz	2
#define PA0_funSelect	PA0_forTesting		//PA0_forPwmBz		//

#define AS_PWMx         PWM0
#define PWM_ID			PWM0_ID
#define PWM_Clock		CLOCK_SYS_CLOCK_HZ
#define PWM_Cycle		8889		//2.7K
#define PWM_Duty_default	PWM_Cycle/2		//2K
//========================================================================================================
//*************************************************************************
#if(Audio_Send_Data)
//*************************************************************************
#define PIN_TIME0_OUT_1    SpeechCtrl

#define SEND_START_LOW_US 5000
#define SEND_BIT1_HIGH_US 1500
#define SEND_BIT1_LOW_US 500
#define SEND_BIT0_HIGH_US 500
#define SEND_BIT0_LOW_US 1500

typedef enum {
    SEND_STATE_IDLE = 0,
    SEND_STATE_START_LOW,
    SEND_STATE_HIGH,
    SEND_STATE_LOW
} send_state_t;


extern send_state_t send_state ;
extern unsigned char g_send_data;
extern unsigned char send_bit_index;
extern void send_data(unsigned char audio_data);
extern void set_timer0_capture_us(unsigned int us);
//*************************************************************************
#endif		//Audio_Send_Data
//*************************************************************************
//*******************************************************************
//良好调参的串级PID系统，阶跃响应应表现出以下特征：
//指标			期望值			含义
//上升时间		50~150ms		飞行器快速响应姿态变化
//超调量			5~15%			轻微超调是可接受的，过大说明D项不足
//调节时间		200~500ms		飞行器在0.5秒内稳定到目标角度
//稳态误差		< 0.5度			积分项应消除大部分稳态误差
//振荡次数		0~1次			不应出现持续振荡
//*******************************************************************
//*******************************************************************
//*******************************************************************
extern unsigned char T10mSflag, Sys10mScnt, T100mSflag, Sys100mScnt, T500mSflag;
extern unsigned int	u32_SysTick, u32_Sys10mScnt, u32_Sys500mScnt;
extern unsigned char testPoint;

extern unsigned char    link_ok_flag;

extern unsigned char	FunCtrlFlag;
#define WorkingStatus	0x01	//: FunCtrlFlag.0; =1 for system working
//#define CarDemoFlag	0x02	//: FunCtrlFlag.1;

#if Used_ADC
//for ADC
extern unsigned short w_AD_Val;
extern unsigned short w_AD_Voltage;
extern unsigned short w_VbatData;				//ADC采样的电池电压保存值
extern unsigned char VddVolFlag;
#define  LowBatWarnFlag		0x04
#define  LowBatSlowFlag		0x02
#define  LowBatOffFlag		0x01
#define  OneSamGenFlag		0x80		//允许一次采样产生低电压标志

#define SET_BIT(var, bit)    ((var) |=  (bit))
#define CLR_BIT(var, bit)    ((var) &= ~(bit))
#define TOG_BIT(var, bit)    ((var) ^=  (bit))
#define IS_BIT_SET(var, bit) (((var) &  (bit)) != 0)
//Samples:
//SET_BIT(VddVolFlag, LowBatWarnFlag);   // 置位
//CLR_BIT(VddVolFlag, LowBatWarnFlag);   // 清除
//if(IS_BIT_SET(VddVolFlag, LowBatWarnFlag)){} // 判断标志
#endif 	//#if Used_ADC
//*****************************************************
//for function control
//*****************************************************
// Define key bitmask, for rf send:g_u8KeyVal_Buffer
//#define KEY_NONE        0x00
typedef enum{
	RfSendKey_Released=0,       //按键释放
	RfSendKey_Xspeed=0x01,		//速度转换（高速/低速）
	RfSendKey_OneKRun=0x02,		//一键起降
	RfSendKey_StuntShow=0x04,	//特技表演
	RfSendKey_MicroAdj=0x08,	//微调模式
	RfSendKey_Headless=0x10,	//无头模式
	RfSendKey_LAST,
}RfSend_Keyval_def;		//for g_u8KeyVal_Buffer

extern unsigned short u16KeyTxBuffer;	//待发送键值
extern unsigned char g_u8KeyVal_Buffer;
extern unsigned char g_u8FirstPowerOn;

extern unsigned char g_u8BeepFlag;
typedef enum{
	Beep_off=0,        	        //蜂鸣器关
	Beep_OneTimes=0x01,			//Beep 100mS 1次
	Beep_TwoTimes=0x02,			//Beep 100mS 间隔100mS 2次
	Beep_ManualMode=0x03,		//手动设置模式
	Beep_Mode_LAST,
}Beep_Ctrl_def;		//for g_u8BeepFlag


//*******************************************************************
void User_hw_init(void);
void User_hw_Exit(void);

void ChangePA3ToSWS(void);
void ChangePA3ToGPIO_OutLow(void);
void GetLinkStatus(void);

void InitalRam(void);
void system_enter_sleep(void);
void NoActionCounter(void);
//void GetBatteryVoltage(void);	//运行一次用时280uS

void UserPowOnStart(void);
void TimeBaseGenerate(void);
void SetTestingPin(void);
void PairingDisplay(void);		//配对显示：配对中闪烁，匹配完成常亮
void OnOffHandler(void);		//关机状态下短按K2开机,开机状态下长按K2关机
void KeyFuncHandler(void);
void TxdFunction_loop(void);
void User_main_loop_process(void);

void BeepHandler(void);		//10mS loop
void ManualPlayBeep(unsigned char BeepTimes, unsigned char BeepCycle, unsigned char BeepDuty);
void StopBeepPlay(void);
void PlayBeepOneTimes(void);
void PlayBeepTwoTimes(void);

//*******************************************************************
#endif	//Project_XH260909_Remotor
//*******************************************************************

//*******************************************************************
#endif /* VENDOR_ROBIN_24G_REMOTE_USERTXDFUNC_H_ */
//*******************************************************************

