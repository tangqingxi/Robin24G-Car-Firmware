//*************************************************************************
/*
 * UserSpeechFunc.h
 *
 *  Created on: 2026年8月30日
 *      Author: HuangGuangjian
 */
//*************************************************************************
//*************************************************************************
#ifndef VENDOR_ROBIN_24G_CAR_USERSPEECHFUNC_H_
#define VENDOR_ROBIN_24G_CAR_USERSPEECHFUNC_H_
//*************************************************************************
//*************************************************************************
#define UseSpeechCmd	0		//使用语音命令控制语音IC播放声音
#define UseTm0SoundCmd	0		//使用Timer0产生语音命令控制语音IC播放声音
#define UseSpeechState	0		//使用语音IC的播放状态
#define UseSpeechLED	0		//使用语音命令控制语音IC管控LED
//*************************************************************************
//*************************************************************************
#if(UseSpeechCmd)
//*************************************************************************

//*************************************************************************
#if(UseTm0SoundCmd)
//*************************************************************************
//for 共用部分
//*************************************************************************
#define SpeechCmdTxPin    SpeechCtrl		//定义Speech命令发送脚

#define SEND_START_LOW_US 5000
#define SEND_BIT1_HIGH_US 1500
#define SEND_BIT1_LOW_US 600		//500
#define SEND_BIT0_HIGH_US 600		//500
#define SEND_BIT0_LOW_US 1500

typedef enum {
    SphSend_State_idle = 0,
    SphSend_State_Delay,
    SphSend_State_Start_Low,
    SphSend_State_High,
    SphSend_State_Low
} Speech_send_state_t;

//*************************************************************************
//*************************************************************************
extern Speech_send_state_t speech_send_state ;
extern unsigned char g_send_data;
extern unsigned char send_bit_index;
extern unsigned short	u16VoiceSentCnt;	//从语音命令发送完毕开始计时

//*************************************************************************
void SpeechCmd_IrqHandler(void);		//500uS定时器中断服务程序中调用
void set_timer0_capture_us(unsigned int us);
void SendVoiceDelay(void);		//10mS loop中控制:语音命令发送完成后需要延迟cVoiceCmdInterval才开始发送下一组命令
void InitalSpeechComm(void);		//初始化语音命令发送脚和定时器（PA3脚）
void SphCmdTxPin_Tm_init(void);		//初始化语音命令发送脚和定时器（非PA3）
void Send_Voice_data(unsigned char audio_data);


//*************************************************************************
#endif	//#if(UseTm0SoundCmd)
//*************************************************************************
//*************************************************************************
//for 专用部分
//*************************************************************************
typedef enum {
	Sound_PairedOK = 0x01,	  //对码成功（嘟声）
	Sound_TurnOff = 0x02,	  //关机声（蒸汽声）
	Sound_StartOn = 0x03,	  //启动声（火车声）
	Sound_Hooter = 0x04,	  //鸣笛声（鸣笛一声）
	Sound_Bell = 0x05,	  //铃铛声（铃铛两声）
	Sound_Di = 0x06,	  //滴声
	Sound_Didi = 0x07,	  //滴滴声
	Sound_OffSound = 0x08,	//关声音（休眠）
	Sound_Last
} SoundIndex_t;

typedef enum {
	Sound_RunLev_1 = 0x10,	  //加速1档声
	Sound_RunLev_2 = 0x20,	  //加速2档声
	Sound_RunLev_3 = 0x30,	  //加速3档声
	Sound_RunLev_4 = 0x40,	  //加速4档声
	Sound_RunLev_5 = 0x50,	  //加速5档声
	Sound_RunLev_6 = 0x60,	  //加速6档声
	Sound_RunLev_7 = 0x70,	  //加速7档声
	Sound_RunLev_8 = 0x80,	  //加速8档声
	Sound_RunLev_9 = 0x90,	  //加速9档声
	Sound_RunLev_Last
} TrainRunSound_t;

//*************************************************************************
//*************************************************************************
extern unsigned char voic_handleflag;		//=1正在发送语音命令；=0已经发送完成
extern unsigned char voice_switchflag;		//语音播放开关：0声音关闭，1声音开
//extern unsigned char u8PlayFlag;		//开始播放声音时=1

//*************************************************************************
//for 共用部分
//*************************************************************************
void InitalSpeechRam(void);
//*************************************************************************
//for 专用部分
//*************************************************************************

//---------------------------------------------
#if(UseSpeechLED)
//---------------------------------------------
typedef enum {
	SphLedSt_Null = 0,
	SphLedSt_Light = 1,
    SphLedSt_Dark = 2,
	SphLedSt_Flash = 3
} SphLed_Status_t;		//for u8LedStatus

extern SphLed_Status_t u8LedStatus;

void SendLedCtrlCmd(void);
void SendVoiceDelay(void);
void SendVoiceNow(void);
void LedStatusHandler(void);		//10mS loop


//---------------------------------------------
#endif	//UseSpeechLED
//---------------------------------------------
#if(UseSpeechState)			//使用语音IC的播放状态
//*************************************************************************
//*************************************************************************
#define SpeechST_Pin		SpeechST

unsigned char GetVoiceStatus(void);

//*************************************************************************
#endif		//(UseSpeechState)			//使用语音IC的播放状态
//*************************************************************************
//*************************************************************************
#endif		//UseSpeechCmd
//*************************************************************************

//*************************************************************************
#endif /* VENDOR_ROBIN_24G_CAR_USERSPEECHFUNC_H_ */
//*************************************************************************

