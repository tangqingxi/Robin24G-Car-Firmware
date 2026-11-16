//*************************************************************************
/*
 * UserTxdFunc.c
 *
 *  Created on: 2026年8月19日
 *      Author: HuangGuangjian
 */
//*************************************************************************
//*************************************************************************
#include "UserTxdFunc.h"
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "app_battery_check.h"
#include "app_24g_rf.h"
#include "UserKeyboard.h"
#include "UserAdc.h"
//*************************************************************************
//*************************************************************************
#if Project_XH260909_Remotor
//*************************************************************************
#if(Audio_Send_Data)
//*************************************************************************
send_state_t send_state = SEND_STATE_IDLE;
unsigned char g_send_data = 0;
unsigned char send_bit_index = 0;

//*************************************************************************
void InitalSpeechComm(void)
{
	gpio_init_out_High(SpeechCtrl);
	gpio_setup_up_down_resistor(SpeechCtrl, PM_PIN_UP_DOWN_FLOAT);

    timer0_set_mode(TIMER_MODE_SYSCLK,0,500 * CLOCK_SYS_CLOCK_1US);
}

inline void set_timer0_capture_us(unsigned int us)
 {
    unsigned int tick = us * CLOCK_SYS_CLOCK_1US;
    reg_tmr0_capt0 = tick & 0xff;
    reg_tmr0_capt1 = (tick >> 8) & 0xff;
    reg_tmr0_capt2 = (tick >> 16) & 0xff;
}

void Send_Data_init(void)
{
    gpio_set_func(PIN_TIME0_OUT_1 ,AS_GPIO);
	gpio_set_output_en(PIN_TIME0_OUT_1, 1); 		//enable output
	gpio_set_input_en(PIN_TIME0_OUT_1 ,0);			//disable input
	gpio_write(PIN_TIME0_OUT_1, 1);

    timer0_set_mode(TIMER_MODE_SYSCLK,0,500 * CLOCK_SYS_CLOCK_1US);
}

void send_data(unsigned char audio_data)
{

	g_send_data = audio_data;
	send_bit_index = 0;
//          audio_send_end = 0;
		// if(data == 0)
	//gpio_write(PIN_TIME0_OUT_1, 0);
	gpio_init_out_low(SpeechCtrl);
	  //else
		//gpio_write(PIN_TIME0_OUT_1, 1);
	set_timer0_capture_us(SEND_START_LOW_US);
	send_state = SEND_STATE_START_LOW;
	timer_clear_interrupt_status(TMR_STA_TMR0);
	timer_start(TIMER0);
}

//*************************************************************************
#endif		//Audio_Send_Data
//*************************************************************************
//**************************************************************
unsigned char T10mSflag, Sys10mScnt, T100mSflag, Sys100mScnt, T500mSflag;
unsigned int	u32_SysTick, u32_Sys10mScnt, u32_Sys500mScnt;
unsigned int u32NoActionCnt;
//**************************************************************
void ChangePA3ToSWS(void)
{
	gpio_set_func(GPIO_PA3,AS_SWIRE);
	gpio_setup_up_down_resistor(GPIO_PA3,PM_PIN_PULLUP_1M);			//设置上拉
	gpio_set_output_en(GPIO_PA3, 0); //Disable output
	gpio_set_input_en(GPIO_PA3,1);//enable input
}


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

//void Leds_Inital(void)
//{
//	gpio_init_out_low(LED_Ctrl);		//高电平点亮
//	gpio_init_out_low(LED2_PIN);		//高电平点亮
//}

//**************************************************************
//硬件PWM设定（适合做喷雾、LED调光调色、幻彩灯控制、PWM0通讯等）
//**************************************************************
void InitalUserPwm(void)
{
	pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ);		//FPWM = FSystem clock / (PWM_CLKDIV+1)
	gpio_set_func(PwmBz_Pin, AS_PWMx);
	pwm_set_mode(PWM_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM_ID, PWM_Cycle, PWM_Duty_default);
	//pwm_start(PWM_ID);
}

//**************************************************************
//开PWM
//**************************************************************
unsigned char g_u8BeepFlag=0;
unsigned char g_u8BeepCnt=0;
unsigned char u8BeepTimes, u8BeepCycle, u8BeepDuty, u8BeepFlagBak;
//**************************************************************
void BeepHandler(void)		//10mS loop
{
	if(g_u8BeepFlag != u8BeepFlagBak)
	{
		u8BeepFlagBak = g_u8BeepFlag;
		//初始设定
		if(g_u8BeepFlag == Beep_OneTimes)
		{
			g_u8BeepCnt=0;
			u8BeepTimes=1;
			u8BeepCycle=20;		//200mS
			u8BeepDuty=10;		//100mS
			pwm_start(PWM_ID);
		}
		else if(g_u8BeepFlag == Beep_TwoTimes)
		{
			g_u8BeepCnt=0;
			u8BeepTimes=2;
			u8BeepCycle=20;		//200mS
			u8BeepDuty=10;		//100mS
			pwm_start(PWM_ID);
		}
		else if(g_u8BeepFlag == Beep_ManualMode)
		{
			g_u8BeepCnt=0;
			pwm_start(PWM_ID);
		}
	}
	else {
		if(g_u8BeepFlag)
		{
			g_u8BeepCnt++;
			if(g_u8BeepCnt == u8BeepDuty)
			{
				pwm_stop(PWM_ID);

			}
			else if(g_u8BeepCnt >= u8BeepCycle)
			{
				g_u8BeepCnt=0;
				u8BeepTimes--;
				if(u8BeepTimes==0)
				{	//播放完成
					g_u8BeepFlag=0;
					u8BeepFlagBak=0;
				}
				else
				{
					pwm_start(PWM_ID);
				}
			}
		}
	}
}

void ManualPlayBeep(unsigned char BeepTimes, unsigned char BeepCycle, unsigned char BeepDuty)
{
	u8BeepTimes=BeepTimes;
	u8BeepCycle=BeepCycle;		//unit 10mS
	u8BeepDuty=BeepDuty;		//unit 10mS
	g_u8BeepFlag = Beep_ManualMode;
}

void StopBeepPlay(void)
{
	pwm_stop(PWM_ID);
	g_u8BeepFlag=0;
	u8BeepFlagBak=0;
	g_u8BeepCnt=0;
}

void PlayBeepOneTimes(void)
{
	g_u8BeepFlag = Beep_OneTimes;
}

void PlayBeepTwoTimes(void)
{
	g_u8BeepFlag = Beep_TwoTimes;
}

//*************************************************************************
//*************************************************************************
void User_hw_init(void)
{
	//for directKey
	KeyBoard_gpio_init();

	ChangePA3ToSWS();

	gpio_init_out_low(VR_POWER_PIN);	//VCC_RP (采样时拉高, 闲置时拉低省电)
	gpio_intput_float(Throttle_Pin);		//电位器中间抽头 (ADC 输入)
	gpio_intput_float(Rotation_Pin);		//电位器中间抽头 (ADC 输入)
	gpio_intput_float(FBmoveAdc_Pin);		//电位器中间抽头 (ADC 输入)
	gpio_intput_float(LRmoveAdc_Pin);		//电位器中间抽头 (ADC 输入)

	//for LED
	//Leds_Inital();

#if(Audio_Send_Data)
	//for Speech communication
	InitalSpeechComm();
#endif		//Audio_Send_Data

#if(PA0_funSelect == PA0_forTesting)
	SetTestingPin();
#else
    //硬件PWM设定（适合做喷雾、LED调光调色、幻彩灯控制、PWM0通讯等）
	InitalUserPwm();
#endif		//#if(PA0_funSelect == PA0_forTesting)

}

//*************************************************************************
//*************************************************************************
void User_hw_Exit(void)
{
	timer_stop(TIMER0);
	timer_stop(TIMER1);
	User_hw_init();
	keyboard_enter_Sleep_setup();

	gpio_setup_up_down_resistor(VR_POWER_PIN, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(Testing_Pin, PM_PIN_PULLDOWN_100K);

#if(Audio_Send_Data)
	//for Speech communication
	gpio_init_out_low(SpeechCtrl);
	gpio_setup_up_down_resistor(SpeechCtrl, PM_PIN_PULLDOWN_100K);
#endif		//Audio_Send_Data
}

//*************************************************************************
//*************************************************************************
unsigned char    link_ok_flag = 0;
unsigned char	FunCtrlFlag;

unsigned char 	testPoint=0;
//*************************************************************************
//*************************************************************************
void InitalRam(void)
{
	VddVolFlag=OneSamGenFlag;
	link_ok_flag=0;
	FunCtrlFlag=0;
//	FunCtrlFlag |= WorkingStatus;
}

extern u8 pair_success_flag;
void GetLinkStatus(void)
{
	if(link_ok_flag == 0)
	{
		if(pair_success_flag)
		{
			link_ok_flag=1;
			PowerOn_Get_Middle();			// Get middle value from adc at power on
		}
	}

}

//*************************************************************************
//*************************************************************************
//*************************************************************************
void NoActionCounter(void)
{
	if(key_Pressed_Flag)
	{
		u32NoActionCnt=0;
		return;
	}

	if(FunCtrlFlag & WorkingStatus)
	{
		u32NoActionCnt++;
		if(link_ok_flag == 0)
		{
			if(u32NoActionCnt < 3000)		//30S
			{
				return;
			}
		}
		else {
			if((Throttle_SendData | Rotation_SendData | FBmove_SendData | LRmove_SendData) != RfDataMiddle)	u32NoActionCnt=0;

			if(u32NoActionCnt < 18000)		//3 minutes = 18000
			{
				return;
			}

		}
		FunCtrlFlag &= ~WorkingStatus;
		link_ok_flag = 0;
		pair_success_flag=0;
		u32NoActionCnt=0;
		testPoint = 0x60;
	}
	else if(FunCtrlFlag == 0){
		u32NoActionCnt++;
		if(u32NoActionCnt > 50)		//500mS
		{
			system_enter_sleep();
		}
	}
}

//*************************************************************************
//*************************************************************************
void system_enter_sleep(void)
{
    rf_set_tx_rx_off();
	User_hw_Exit();
	wd_32k_stop();
    //car_enter_deepsleep_no_retention();
	cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);
}

//*************************************************************************
//for ADC Program
//*************************************************************************
#if Used_ADC
//*************************************************************************
unsigned short w_AD_Val;
unsigned short w_AD_Voltage;
unsigned short w_VbatData;				//ADC采样的电池电压保存值
//*************************************************************************
adc_chn_cfg_t chn_cfg1 =
{
        .divider     = ADC_VBAT_DIV_OFF,
        .v_ref       = ADC_VREF_1P2V,
        .pre_scale   = ADC_PRESCALE_1F4,
        .sample_freq = ADC_SAMPLE_FREQ_96K,
        .input_p     = ADC_GPIO_PB0 >> 12,
        .input_n     = GND,
};

extern unsigned short g_adc_vref[];
extern signed char    g_adc_vref_offset[];
extern unsigned short g_adc_gpio_calib_vref;
extern signed char    g_adc_gpio_calib_vref_offset;
//*************************************************************************
void ADC_GPIOpin_sample_init(adc_sample_chn_e chn, adc_input_pin_def_e pin)
{
    adc_pin_config(ADC_GPIO_MODE, pin);
    adc_chn_config(chn, chn_cfg1);

    g_adc_vref[chn]        = g_adc_gpio_calib_vref;        //set gpio sample calib vref
    g_adc_vref_offset[chn] = g_adc_gpio_calib_vref_offset; //set adc_vref_offset as adc_gpio_calib_vref_offset
}


//*************************************************************************
//*************************************************************************
void adc_user_init(adc_input_pin_mode_e mode, GPIO_PinTypeDef pin)
{
	if(mode == ADC_VBAT_MODE)
	{
		adc_init(NDMA_M_CHN);
		adc_vbat_sample_init(ADC_M_CHANNEL);
	}
	else if(mode == ADC_GPIO_MODE)
	{
        adc_init(NDMA_M_CHN);
        if(pin == GPIO_PB0)				ADC_GPIOpin_sample_init(ADC_M_CHANNEL, ADC_GPIO_PB0);
        else if(pin == GPIO_PB1)		ADC_GPIOpin_sample_init(ADC_M_CHANNEL, ADC_GPIO_PB1);
        else if(pin == GPIO_PB2)		ADC_GPIOpin_sample_init(ADC_M_CHANNEL, ADC_GPIO_PB2);
        else if(pin == GPIO_PA4)		ADC_GPIOpin_sample_init(ADC_M_CHANNEL, ADC_GPIO_PA4);
        else if(pin == GPIO_PA5)		ADC_GPIOpin_sample_init(ADC_M_CHANNEL, ADC_GPIO_PA5);
	}
	adc_power_on();
}

//*************************************************************************
// output：w_AD_Val
//**************************************************************************
void Get_ADCval(void)		//采样16次Adc并求平均值
{
	w_AD_Val = adc_get_result(NDMA, ADC_M_CHANNEL);		//adc_sample_and_get_result();
	w_AD_Voltage = w_AD_Val;		//adc_calculate_voltage(ADC_M_CHANNEL, w_AD_Val);
}

//**************************************************************************
//*************************************************************************
#define	LowBatWarnThresh 	2500
#define	LowBatSlowThresh 	2300
#define	LowBatOffThresh 	2200
//*********************
unsigned char VddVolFlag;
//#define  LowBatWarnFlag		0x04
//#define  LowBatSlowFlag		0x02
//#define  LowBatOffFlag		0x01
//#define  OneSamGenFlag		0x80		//允许一次采样产生低电压标志
unsigned char BatteryLowCnt;
unsigned char BatLowerCnt;
unsigned char BatLowOffCnt;
unsigned short u16BatOffTmCnt;
//*************************************************************************
void GetBatteryVoltage(void)	//运行一次用时280uS
{
	//设置采样通道

	adc_user_init(ADC_VBAT_MODE, 0);
	sleep_us(20);
	Get_ADCval();
	//testPoint = 0x1d;

	w_VbatData = w_AD_Voltage;

	//if(LowBatWarnFlag)
	if(IS_BIT_SET(VddVolFlag, LowBatWarnFlag))
	{
		//if(LowBatSlowFlag)
		if(IS_BIT_SET(VddVolFlag, LowBatSlowFlag))
		{
			//if(LowBatOffFlag==0)
			if(IS_BIT_SET(VddVolFlag, LowBatOffFlag)==0)
			{
				u16BatOffTmCnt++;
				if(u16BatOffTmCnt > 4096)
				{
					//LowBatOffFlag=1;
					SET_BIT(VddVolFlag, LowBatOffFlag);
				}
				else
				{
					if(w_VbatData < LowBatOffThresh)
					{
						BatLowOffCnt++;
						if(BatLowOffCnt >= 180)
						{
							//LowBatOffFlag=1;
							SET_BIT(VddVolFlag, LowBatOffFlag);
						}
					}
					else
					{
						BatLowOffCnt=0;
					}
				}
			}
		}
		else
		{
			u16BatOffTmCnt=0;
			if(w_VbatData < LowBatSlowThresh)
			{
				BatLowerCnt++;
				if(BatLowerCnt >= 180)
				{
					//LowBatSlowFlag=1;
					SET_BIT(VddVolFlag, LowBatSlowFlag);
				}
			}
			else
			{
				BatLowerCnt=0;
			}
		}
	}
	else
	{
		if(w_VbatData < LowBatWarnThresh)
		{
			BatteryLowCnt++;
			if(BatteryLowCnt >= 180)
			{
				//LowBatWarnFlag=1;
				SET_BIT(VddVolFlag, LowBatWarnFlag);
			}
		}
		else
		{
			BatteryLowCnt=0;
		}
	}

	//if(OneSamGenFlag)
	if(IS_BIT_SET(VddVolFlag, OneSamGenFlag))
	{
		//OneSamGenFlag=0;
		CLR_BIT(VddVolFlag, OneSamGenFlag);

		if(w_VbatData < LowBatWarnThresh)
		{
			//LowBatWarnFlag=1;
			SET_BIT(VddVolFlag, LowBatWarnFlag);
			if(w_VbatData < LowBatSlowThresh)
			{
				//LowBatSlowFlag=1;
				SET_BIT(VddVolFlag, LowBatSlowFlag);
				if(w_VbatData < LowBatOffThresh)
				{
					//LowBatOffFlag=1;
					SET_BIT(VddVolFlag, LowBatOffFlag);
				}
			}
		}
	}

}

//*************************************************************************
// for Adc pin samples
//*************************************************************************
#if 0
//*************************************************************************
unsigned short	w_LMC_AdcData;		//adc chanel 8 sample value,for 右马达电流
//*************************************************************************
void LMC_adc_Sampling(void)		//for Left motor current samples
{
	//设置采样通道
	adc_user_init(ADC_GPIO_MODE, LMOT_ADC);
	sleep_us(20);
	Get_ADCval();

	w_LMC_AdcData = w_AD_Val;
}

//*************************************************************************
unsigned short	w_RMC_AdcData;		//adc chanel 9 sample value,for 左马达电流
//*************************************************************************
void RMC_adc_Sampling(void)		//for Right motor current samples
{
	//设置采样通道
	adc_user_init(ADC_GPIO_MODE, RMOT_ADC);
	sleep_us(20);
	Get_ADCval();

	w_RMC_AdcData = w_AD_Val;
}

//*************************************************************************
#endif
//*************************************************************************
#endif	//Used_ADC
//*************************************************************************
//**************************************************************
void UserPowOnStart(void)
{
	InitalRam();		//初始化用户变量
//	GetUsb5VinStatus();	//获取充电用USB 5V输入状态
	GetBatteryVoltage();	//测出电源电压
	PowerOn_Get_Middle();			// Get middle value from adc at power on
	ClearKeyBoardFlags();
	g_u8FirstPowerOn=1;
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
		T10mSflag=1;
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转

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

//*************************************************************************
unsigned char 	LedCtrlCnt, LedCtrlFlag;
//*************************************************************************
#define cStateledBit		0x01
#define cLampledBit			0x02
#define LightStateled()		LedCtrlFlag |= cStateledBit
#define DarkStateled()		LedCtrlFlag &= ~cStateledBit
#define LightLampled()		LedCtrlFlag |= cLampledBit
#define DarkLampled()		LedCtrlFlag &= ~cLampledBit
#define cPairDispCycle	60
#define cPairDispDuty	30
#define cShortDispCycle	50
#define cShortDispDuty	5

//*************************************************************************
//*************************************************************************
//void SetLedPins(void)		//10mS loop
//{
//	if(LedCtrlFlag & cStateledBit)
//	{
//		gpio_write(LED_Ctrl, 1);
//	}
//	else {
//		gpio_write(LED_Ctrl, 0);		//H电平点亮
//	}
//
////	if(LedCtrlFlag & cLampledBit)
////	{
////		gpio_write(LED2_PIN, 1);
////	}
////	else {
////		gpio_write(LED2_PIN, 0);		//H电平点亮
////	}
//}

//*************************************************************************
const unsigned char StepLev_Duty_tab[10] ={
		0, 4, 6, 8, 10, 12, 14, 16, 18, 20
};
//*************************************************************************
void PairingDisplay(void)		//配对显示：配对中闪烁，匹配完成常亮
{
	if(FunCtrlFlag & WorkingStatus)
	{
		if(link_ok_flag==0)
		{	//RF没有连接上, 1.67Hz flash
			if(LedCtrlCnt == 0)
			{
				//LightStateled();
				gpio_write(LED_Ctrl, 1);		//H电平点亮
			}
			else if(LedCtrlCnt == cPairDispDuty)
			{
				//DarkStateled();
				gpio_write(LED_Ctrl, 0);		//H电平点亮
			}
			LedCtrlCnt++;
			if(LedCtrlCnt > cPairDispCycle)
			{
				LedCtrlCnt=0;
			}
		}
		else {	//RF已连接上, 电压正常时常亮，低电压断闪
//			if(VddVolFlag & LowBatWarnFlag)
//			{
//
//				if(LedCtrlCnt == 0)
//				{
//					LightStateled();
//				}
//				else if(LedCtrlCnt == cShortDispDuty)
//				{
//					DarkStateled();
//				}
//				LedCtrlCnt++;
//				if(LedCtrlCnt > cShortDispCycle)
//				{
//					LedCtrlCnt=0;
//				}
//			}
//			else {	//电压时常亮
//				LightStateled();
//				LedCtrlCnt=0;
//			}
			gpio_write(LED_Ctrl, 1);		//H电平点亮
		}
	}
	else {
		//DarkStateled();
		gpio_write(LED_Ctrl, 0);		//H电平点亮
	}
}

//*****************************************************
unsigned char g_u8FirstPowerOn;
unsigned char g_u8KeyVal_Buffer=0;
unsigned char g_u8NewKforSend=0;	//=0: 无键值需要发送；!=0:发送次数
#define cSendTimes	20
//*****************************************************
//键值处理分为Remotor内部和RF外发两部分：
//一、Remotor内部
//		1、关机状态下短按OnOff开机
//		2、开机状态下长按OnOff关机
//二、RF外发，开机状态下才有效
//		1、短按K1为鸣钟
//		2、短按K2为启动/熄火
//		3、短按K3为汽笛声
//*****************************************************
void OnOffHandler(void)		//关机状态下短按K2开机,开机状态下长按K2关机
{
	if(FunCtrlFlag & WorkingStatus)
	{
		if(Longkey_Action && (key_final_value == LongKBVal03))
		{	//长按关机
			Longkey_Action=0;
			FunCtrlFlag &= ~WorkingStatus;
			link_ok_flag = 0;
			pair_success_flag=0;
		}
		if(IS_BIT_SET(VddVolFlag, LowBatOffFlag))
		{
			FunCtrlFlag &= ~WorkingStatus;
			link_ok_flag = 0;
			pair_success_flag=0;
		}
	}
	else {	//关机状态下
		if(IS_BIT_SET(VddVolFlag, LowBatOffFlag)==0)
		{
			if((key_Action_Flag) && (key_final_value == KEY_OnOffKey))
			{
				key_Action_Flag=0;
				FunCtrlFlag |= WorkingStatus;
			}
		}
	}
}

void KeyFuncHandler(void)
{
	if((FunCtrlFlag & WorkingStatus) && link_ok_flag)
	{	//开机状态下
		if(key_Action_Flag)
		{
			key_Action_Flag=0;
			g_u8NewKforSend=cSendTimes;

			if(key_final_value == KEY_ChangeSpeed)
			{	//速度转换（高速/低速）
				g_u8KeyVal_Buffer = RfSendKey_Xspeed;
			}
//			else if(key_final_value == KEY_OneKeyRun)
//			{	//一键起降
//				g_u8KeyVal_Buffer = RfSendKey_OneKRun;
//			}
			else if(key_final_value == KEY_StuntShow)
			{	//特技表演
				g_u8KeyVal_Buffer = RfSendKey_StuntShow;
			}
			else if(key_final_value == KEY_MicroAdjMode)
			{	//微调模式
				g_u8KeyVal_Buffer = RfSendKey_MicroAdj;
			}
//			else if(key_final_value == KEY_HeadlessMode)
//			{	//无头模式
//				g_u8KeyVal_Buffer = RfSendKey_Headless;
//			}

		}
		else if(Longkey_Action)
		{
			Longkey_Action=0;
			g_u8NewKforSend=cSendTimes;

			if(key_final_value == LongKBVal02)
			{	//无头模式(长按)
				g_u8KeyVal_Buffer = RfSendKey_Headless;
			}
			else if(key_final_value == LongKBVal01)
			{	//一键起降(长按)
				g_u8KeyVal_Buffer = RfSendKey_OneKRun;
			}
		}
	}
}

//*****************************************************
//for function control
//*****************************************************
unsigned short u16KeyTxBuffer;	//待发送键值
//*****************************************************
void TxdFunction_loop(void)
{
	if(T10mSflag)		//10mS
	{
		T10mSflag=0;
		//gpio_toggle(Testing_Pin);	//Test_Pin 翻转

		//gpio_write(Testing_Pin, 1);
		GetBatteryVoltage();	//测出电源电压
		//HandleRemoterAdcData();
		//gpio_write(Testing_Pin, 0);

		UserKeyBoardScanFunc();
		OnOffHandler();		//关机状态下短按K2开机,开机状态下长按K2关机
		PairingDisplay();		//配对显示：配对中闪烁，匹配完成常亮
		if(FunCtrlFlag & WorkingStatus)
		{
			KeyFuncHandler();
			GetLinkStatus();
			if(link_ok_flag)
			{
				HandleRemoterAdcData();
				//这里还需要发需要发送的数据填到u16KeyTxBuffer中;***********************
				unsigned char	Txbuf[RF_CMD_DATA_DAT_LEN];
				if(g_u8NewKforSend)
				{
					g_u8NewKforSend--;
					u16KeyTxBuffer = g_u8KeyVal_Buffer;
					Txbuf[0] = g_u8KeyVal_Buffer >> 8;
					Txbuf[1] = g_u8KeyVal_Buffer;
				}
				else {
					u16KeyTxBuffer = 0;
					Txbuf[0] = 0;
					Txbuf[1] = 0;
				}
				Txbuf[2] = Throttle_SendData >> 8;
				Txbuf[3] = Throttle_SendData;
				Txbuf[4] = Rotation_SendData >> 8;
				Txbuf[5] = Rotation_SendData;
				Txbuf[6] = FBmove_SendData >> 8;
				Txbuf[7] = FBmove_SendData;
				Txbuf[8] = LRmove_SendData >> 8;
				Txbuf[9] = LRmove_SendData;

				remote_send_cmd(CMD_KEY, u16KeyTxBuffer, Txbuf, RF_CMD_DATA_DAT_LEN);		//remote_send_cmd(u8 type, u16 event, u8 *p_dat, u8 dat_len)
			}
		}

		//SetLedPins();		//10mS loop
		NoActionCounter();
	}

	if(T100mSflag)
	{
		T100mSflag=0;

	}

	if(T500mSflag)
	{
		T500mSflag=0;
//		if (0 == (u32_Sys500mScnt%8))
//		{
//			send_data(0x01);
//			gpio_toggle(Testing_Pin);	//Test_Pin 翻转

//		}

	}
}

//**************************************************************
void User_main_loop_process(void)
{
	wd_32k_feed();
	/*remote will send cmd and receive ack*/
	if(FunCtrlFlag & WorkingStatus)
	{
		d24_tx_main_loop();
	//	remote_loop();

	}

#if (BATT_CHECK_ENABLE)
	//user_batt_check_proc();//batt check proc
#endif

#if (APP_TEST_ENABLE)
	// main loop code
	app_test_loop();
#endif

	TimeBaseGenerate();
	TxdFunction_loop();

}

//*************************************************************************
#endif  //Project_XH260909_Remotor
//*************************************************************************




