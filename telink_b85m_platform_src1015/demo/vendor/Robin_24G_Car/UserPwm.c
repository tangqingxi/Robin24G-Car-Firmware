//*************************************************************************
/*
 * UserPwm.c
 *
 *  Created on: 2026年9月25日
 *      Author: HuangGuangjian
 */
//*************************************************************************
//*************************************************************************
// ******************************************************************
#include "UserPwm.h"
#include "driver.h"
#include "app_config.h"
#include "string.h"
#include "app_public_config.h"
#include "app_toy/driver_layer/hw_gpio.h"
#include "UserSpeechFunc.h"
// ******************************************************************
// ******************************************************************
#if (UsePwm0SoundCmd)
// ******************************************************************
// ******************************************************************
#define	PWM0_PIN		SpeechCtrl		//GPIO_PA1
// ******************************************************************
// ******************************************************************
volatile unsigned short ir_dma_send_buff[32]={0};
//volatile unsigned short il_wave_send_buff[16]={0};
//volatile unsigned short il_wave_receive_buff[18]={0};

// ******************************************************************
extern unsigned char voic_handleflag;		//=1正在发送语音命令；=0已经发送完成
// ******************************************************************
void Pwm0IrqHandler(void)
{
#if(IrPwm0DmaIntEn)
	if(pwm_get_interrupt_status(PWM_IRQ_PWM0_IR_DMA_FIFO_DONE)){
		pwm_clear_interrupt_status(PWM_IRQ_PWM0_IR_DMA_FIFO_DONE);

		gpio_init_out_High(PWM0_PIN);
		voic_handleflag=0;
	}

#endif
}

// ******************************************************************
// ******************************************************************
void Pwm0_SendSoundCmd(unsigned char u8Cmd)
{

	/*****************************pwm ir dma fifo send waveform*****************************/

		gpio_set_func(PWM0_PIN, PWM0_N);
		pwm_set_mode(PWM0_ID, PWM_IR_DMA_FIFO_MODE);
		pwm_set_cycle_and_duty(PWM0_ID, 48000, 12000);  //周期为2000uS => 0	: High 500 /Low 1500
		pwm_set_pwm0_shadow_cycle_and_duty(48000, 36000);  //周期为2000uS => 1	:High 1500/Low 500
		pwm_n_revert(PWM0_ID);

		unsigned char index=2,bitPos;
		ir_dma_send_buff[index++]= pwm_config_dma_fifo_waveform(0, PWM0_PULSE_NORMAL, 3);		//Low 6000uS

		bitPos=0x80;
		while(bitPos)
		{
			if(u8Cmd & bitPos)
				ir_dma_send_buff[index++]= pwm_config_dma_fifo_waveform(1, PWM0_PULSE_SHADOW, 1);
			else
				ir_dma_send_buff[index++]= pwm_config_dma_fifo_waveform(1, PWM0_PULSE_NORMAL, 1);
			bitPos=bitPos>>1;
		}

		unsigned int length = index*2 - 4;//The first four bytes are data length bytes, not included in the actual length to be sent
		unsigned char* buff = (unsigned char*)&ir_dma_send_buff[0];
		buff[0]= length&0xff;
		buff[1]= (length>>8)&0xff;
		buff[2]= (length>>16)&0xff;
		buff[3]= (length>>24)&0xff;
		pwm_set_dma_address(&ir_dma_send_buff);

		pwm_set_interrupt_enable(PWM_IRQ_PWM0_IR_DMA_FIFO_DONE);
		//irq_set_mask(FLD_IRQ_SW_PWM_EN);
		pwm_start_dma_ir_sending();
}

//*************************************************************************
void InitalSpeechComm(void)
{
	gpio_init_out_High(PWM0_PIN);
	//gpio_setup_up_down_resistor(SpeechCmdTxPin, PM_PIN_UP_DOWN_FLOAT);
#if(UseSpeechState)
	gpio_intput_pulldown_100k(SpeechST_Pin);
#endif	//UseSpeechState

	//ChangePA3ToSWS();
}

//*******************************************************************
void Send_Voice_data(unsigned char audio_data)
{
	Pwm0_SendSoundCmd(audio_data);
	voic_handleflag=1;
}

//*************************************************************************
//*************************************************************************
#endif	//#if (UsePwm0SoundCmd)
//*************************************************************************
//*************************************************************************
// ******************************************************************
#if (UsePwmxAsAtomizer)
// ******************************************************************
//**************************************************************
//硬件PWM设定（适合做喷雾、LED调光调色、幻彩灯控制、PWM0通讯等）
//**************************************************************
#define AS_PWMx         PWM2
#define PWM_ID			PWM2_ID
#define PWM_Clock		CLOCK_SYS_CLOCK_HZ
#define PWM_Cycle		222		//108K
#define PWM_Duty_default	PWM_Cycle/2		//108K
//**************************************************************
void InitalUserPwm(void)
{
	pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ);		//FPWM = FSystem clock / (PWM_CLKDIV+1)
	gpio_set_func(Atomizer_Ctrl, AS_PWMx);
	pwm_set_mode(PWM_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM_ID, PWM_Cycle, PWM_Duty_default);
	//pwm_start(PWM_ID);
}

//**************************************************************
//开雾化器
//**************************************************************
unsigned char g_u8AtomizerOnFlag=0;
//**************************************************************
void TurnOnAtomizer(void)
{
	if((g_u8AtomizerOnFlag==0)&&(NoAtomizerFlag==0))
	{
		pwm_start(PWM_ID);
		g_u8AtomizerOnFlag=1;
	}
}

//**************************************************************
//关雾化器
//**************************************************************
void TurnOffAtomizer(void)
{
	if(g_u8AtomizerOnFlag)
	{
		pwm_stop(PWM_ID);
		g_u8AtomizerOnFlag=0;
	}
}

//**************************************************************
#endif		//#if (UsePwmxAsAtomizer)
//**************************************************************
//马达PWM
//#define LFmotPwm_Pin     	GPIO_PA5		//Pwm1, to motor driver
//#define RBmotPwm_Pin     	GPIO_PA4		//Pwm0, to motor driver
//#define	LBmotPwm_Pin    	GPIO_PA7		//Pwm3, to motor driver
//#define RFmotPwm_Pin		GPIO_PA6		//Pwm2, to motor driver
//**************************************************************************************************
unsigned short	LFmotDuty=0;
unsigned short	RBmotDuty=0;
unsigned short	LBmotDuty=0;
unsigned short	RFmotDuty=0;

//#define Mot_PWM_Cycle		400			//15KHz(67uS)
//#define cMot_PwmDutyMax		400
//#define cMot_PwmDutyMin		0

//马达PWM
#define LFmot_PWM	PWM1
#define RBmot_PWM	PWM0
#define LBmot_PWM	PWM3
#define RFmot_PWM	PWM2

#define LFmot_PWM_ID	PWM1_ID
#define RBmot_PWM_ID	PWM0_ID
#define LBmot_PWM_ID	PWM3_ID
#define RFmot_PWM_ID	PWM2_ID

//**************************************************************************************************
void UserPwmPinInital(void)
{
	gpio_init_out_low(LFmotPwm_Pin);
	gpio_init_out_low(RBmotPwm_Pin);
	gpio_init_out_low(LBmotPwm_Pin);
	gpio_init_out_low(RFmotPwm_Pin);

}

//**************************************************************************************************
//**************************************************************************************************
void User_pwm_init_func(void)
{
	pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ/4);		//6MHz

	LFmotDuty=0;
	RBmotDuty=0;
	LBmotDuty=0;
	RFmotDuty=0;

//	LFmotDuty=200;
	gpio_set_func(LFmotPwm_Pin, PWM1_N);
	pwm_set_mode(PWM1_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM1_ID, Mot_PWM_Cycle, LFmotDuty);
	pwm_start(PWM1_ID);

//	RBmotDuty=300;
	gpio_set_func(RBmotPwm_Pin, PWM0_N);
	pwm_set_mode(PWM0_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM0_ID, Mot_PWM_Cycle, RBmotDuty);
	pwm_start(PWM0_ID);

//	LBmotDuty=80;
	gpio_set_func(LBmotPwm_Pin, PWM3_N);
	pwm_set_mode(PWM3_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM3_ID, Mot_PWM_Cycle, LBmotDuty);
	pwm_start(PWM3_ID);

//	RFmotDuty=40;
	gpio_set_func(RFmotPwm_Pin, PWM2_N);
	pwm_set_mode(PWM2_ID, PWM_NORMAL_MODE);
	pwm_set_cycle_and_duty(PWM2_ID, Mot_PWM_Cycle, RFmotDuty);
	pwm_start(PWM2_ID);
//	while(1)
//	{
//		wd_32k_feed();
//	}

}

//**************************************************************************************************
//**************************************************************************************************
void User_pwm_Exit_func(void)
{
	pwm_stop(PWM0_ID);
	pwm_stop(PWM1_ID);
	pwm_stop(PWM2_ID);
	pwm_stop(PWM3_ID);
	UserPwmPinInital();
}

//**************************************************************************************************
//**************************************************************************************************
void UpdateLFmotPwmDuty(void)
{
	pwm_set_cycle_and_duty(PWM1_ID, Mot_PWM_Cycle, LFmotDuty);
}

void UpdateLBmotPwmDuty(void)
{
	pwm_set_cycle_and_duty(PWM3_ID, Mot_PWM_Cycle, LBmotDuty);
}

void UpdateRFmotPwmDuty(void)
{
	pwm_set_cycle_and_duty(PWM2_ID, Mot_PWM_Cycle, RFmotDuty);
}

void UpdateRBmotPwmDuty(void)
{
	pwm_set_cycle_and_duty(PWM0_ID, Mot_PWM_Cycle, RBmotDuty);
}


void Update4MotPwmDuty(void)
{
	pwm_set_cycle_and_duty(PWM1_ID, Mot_PWM_Cycle, LFmotDuty);
	pwm_set_cycle_and_duty(PWM3_ID, Mot_PWM_Cycle, LBmotDuty);
	pwm_set_cycle_and_duty(PWM2_ID, Mot_PWM_Cycle, RFmotDuty);
	pwm_set_cycle_and_duty(PWM0_ID, Mot_PWM_Cycle, RBmotDuty);
}

//**************************************************************************
//**************************************************************************
unsigned short CheckPwmDuty(unsigned short duty)
{
	if(duty > cMot_PwmDutyMax)
	{
		duty=cMot_PwmDutyMax;
	}
	else if(duty < cMot_PwmDutyMin)
	{
		duty=cMot_PwmDutyMin;
	}
	return duty;
}

//**************************************************************************
//**************************************************************************
//**************************************************************************************************
//**************************************************************************************************

