//*************************************************************************
/*
 * UserSpeechFunc.c
 *
 *  Created on: 2026年8月30日
 *      Author: HuangGuangjian
 */
//*************************************************************************
//*************************************************************************
#include "app_public_config.h"
#include "UserSpeechFunc.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserPwm.h"

//*************************************************************************
#if(UseSpeechCmd)
//*************************************************************************
unsigned char voice_switchflag=0;		//语音开机/关闭控制变量
unsigned char voic_handleflag=0;	 //=1正在发送语音命令；=0已经发送完成

#if(UseSpeechLED)
SphLed_Status_t u8LedStatus=SphLedSt_Null;
SphLed_Status_t u8LedStateBak=SphLedSt_Null;
#endif		//#if(UseSpeechLED)

//*************************************************************************
#if(UseTm0SoundCmd)
//*************************************************************************
Speech_send_state_t speech_send_state = SphSend_State_idle;
unsigned char g_send_data = 0x96;
unsigned char send_bit_index = 0;
unsigned short	u16VoiceSentCnt;	//从语音命令发送完毕开始计时

//*************************************************************************
//500uS定时器中断服务程序中调用
//*************************************************************************
void SpeechCmd_IrqHandler(void)
{
	if(timer_get_interrupt_status(TMR_STA_TMR0))
	{
		timer_clear_interrupt_status(TMR_STA_TMR0);

		if (speech_send_state == SphSend_State_Start_Low) {
			// 起始低结束，开始第一位的高电平
			gpio_write(SpeechCmdTxPin, 1);
			send_bit_index = 0;
			u16 high_us = (g_send_data & 0x80) ? SEND_BIT1_HIGH_US : SEND_BIT0_HIGH_US;
			set_timer0_capture_us(high_us);
			speech_send_state = SphSend_State_High;
		} else if (speech_send_state == SphSend_State_High) {
			// 高电平结束，发送低电平
			u8 bit = (g_send_data & (0x80 >> send_bit_index)) ? 1 : 0;
			u16 low_us = bit ? SEND_BIT1_LOW_US : SEND_BIT0_LOW_US;
			gpio_write(SpeechCmdTxPin, 0);
			set_timer0_capture_us(low_us);
			speech_send_state = SphSend_State_Low;
		} else if (speech_send_state == SphSend_State_Low) {
			// 低电平结束，处理下一位或结束
			send_bit_index++;
			if (send_bit_index < 8) {
				u8 bit = (g_send_data & (0x80 >> send_bit_index)) ? 1 : 0;
				u16 high_us = bit ? SEND_BIT1_HIGH_US : SEND_BIT0_HIGH_US;
				gpio_write(SpeechCmdTxPin, 1);
				set_timer0_capture_us(high_us);
				speech_send_state = SphSend_State_High;
			} else {
				// 发送完成
				gpio_write(SpeechCmdTxPin, 1);
				timer_stop(TIMER0);
				speech_send_state = SphSend_State_idle;
				ChangePA3ToSWS();
				voic_handleflag=0;
				u16VoiceSentCnt=0;
			}
		} else {
			// IDLE状态，如果意外进入则停止定时器
			timer_stop(TIMER0);
		 //gpio_write(SpeechCmdTxPin, 0);  //CX
		}

		// 正在发送数据时，跳过RF中断处理以保证PB1时序精度。
		// RF中断状态会保持pending，发送完成后在下次irq_handler中被处理。
		if (speech_send_state != SphSend_State_idle) {
			return;
		}
	}
}

//*************************************************************************
//*******************************************************************
inline void set_timer0_capture_us(unsigned int us)
 {
    unsigned int tick = us * CLOCK_SYS_CLOCK_1US;
    reg_tmr0_capt0 = tick & 0xff;
    reg_tmr0_capt1 = (tick >> 8) & 0xff;
    reg_tmr0_capt2 = (tick >> 16) & 0xff;
}

//*************************************************************************
void InitalSpeechComm(void)
{
	gpio_init_out_High(SpeechCmdTxPin);
	//gpio_setup_up_down_resistor(SpeechCmdTxPin, PM_PIN_UP_DOWN_FLOAT);
#if(UseSpeechState)
	gpio_intput_pulldown_100k(SpeechST_Pin);
#endif	//UseSpeechState

	//ChangePA3ToSWS();
    timer0_set_mode(TIMER_MODE_SYSCLK,0,500 * CLOCK_SYS_CLOCK_1US);
}

//*******************************************************************
void SphCmdTxPin_Tm_init(void)
{
	gpio_init_out_High(SpeechCmdTxPin);

#if(UseSpeechState)
	gpio_intput_pulldown_100k(SpeechST_Pin);
#endif	//UseSpeechState
    timer0_set_mode(TIMER_MODE_SYSCLK,0,500 * CLOCK_SYS_CLOCK_1US);
}

//*******************************************************************
void Send_Voice_data(unsigned char audio_data)
{

	g_send_data = audio_data;
	send_bit_index = 0;
	set_timer0_capture_us(SEND_START_LOW_US);
	speech_send_state = SphSend_State_Delay;		//SphSend_State_Start_Low;

	voic_handleflag=1;
}

//*******************************************************************
//10mS loop中控制:语音命令发送完成后需要延迟cVoiceCmdInterval才开始发送下一组命令
//*******************************************************************
#define cVoiceCmdInterval	10		//100mS
//*******************************************************************
void SendVoiceDelay(void)
{
	if(speech_send_state == SphSend_State_Delay)
	{
		if(u16VoiceSentCnt > cVoiceCmdInterval)		//200mS
		{
			gpio_init_out_low(SpeechCmdTxPin);
			speech_send_state = SphSend_State_Start_Low;
			timer_clear_interrupt_status(TMR_STA_TMR0);
			timer_start(TIMER0);
		}
	}
}

//*******************************************************************
void SendVoiceNow(void)
{
	if(speech_send_state == SphSend_State_Delay)
	{
		gpio_init_out_low(SpeechCmdTxPin);
		speech_send_state = SphSend_State_Start_Low;
		timer_clear_interrupt_status(TMR_STA_TMR0);
		timer_start(TIMER0);
	}
}

//*************************************************************************
#endif	//#if(UseTm0SoundCmd)
//*************************************************************************
//*******************************************************************
void InitalSpeechRam(void)
{
	voice_switchflag = 1;		//上电声音默认开启
#if(UseSpeechLED)
	u8LedStatus = SphLedSt_Flash;
#endif		//#if(UseSpeechLED)
}

//*******************************************************************
#if(UseSpeechState)			//使用语音IC的播放状态
unsigned char GetVoiceStatus(void)
{
	if(u8VoiceStChkDelayCnt)	return 1;
	unsigned char temp;
	temp = gpio_read(SpeechST_Pin);
	return temp;
}

//*************************************************************************
#endif		//(UseSpeechState)			//使用语音IC的播放状态
//*************************************************************************
//*******************************************************************
//for 专用部分
//*******************************************************************






//*******************************************************************
//*******************************************************************
#if(UseSpeechLED)
//*******************************************************************
void SendLedCtrlCmd(void)
{
	if(u8LedStateBak == SphLedSt_Dark)
	{
		Send_Voice_data(RX_voice_2);
	}
	else if(u8LedStateBak == SphLedSt_Light)
	{
		Send_Voice_data(RX_voice_1);
	}
	else if(u8LedStateBak == SphLedSt_Flash)
	{
		Send_Voice_data(RX_voice_3);
	}
}

//*******************************************************************
//SphLed_Status_t u8LedStatus=SphLedSt_Dark;
//SphLed_Status_t u8LedStateBak=SphLedSt_Dark;
//*******************************************************************
void LedStatusHandler(void)		//10mS loop
{
	if(!link_ok_flag)
	{	//未连接遥控器, 全部闪烁
		u8LedStatus = SphLedSt_Flash;
	}
	else{
		u8LedStatus = SphLedSt_Light;
	}


	if(u8LedStateBak != u8LedStatus)
	{
		u8LedStateBak = u8LedStatus;
		SendLedCtrlCmd();
	}
}

//*******************************************************************
#endif	//#if(UseSpeechLED)
//*******************************************************************
//*************************************************************************
#endif		//#if(UseSpeechCmd)
//*************************************************************************


