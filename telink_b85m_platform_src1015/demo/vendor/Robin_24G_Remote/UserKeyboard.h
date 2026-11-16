//*******************************************************************
/*
 * UserKeyboard.h
 *
 *  Created on: 2026年8月19日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#ifndef VENDOR_ROBIN_24G_REMOTE_USERKEYBOARD_H_
#define VENDOR_ROBIN_24G_REMOTE_USERKEYBOARD_H_
//*******************************************************************
//*******************************************************************
#define KEYBOARD_ENABLE     1
#define DirectKey_ENABLE    0
#define SwitchScan_ENABLE	0
//*******************************************************************
// ******************************************************************
#if KEYBOARD_ENABLE

#define Used_CountKB		0	//需要使用连续按键时置1
#define Used_LongKB			1	//需要使用长按按键时置1

//define Keyboard pins
//行
#define KB_ROW_01	KeyCom_Pin
#define KB_ROW_02	KeyCom_Pin
//列
#define KB_COL_01	KeySige1_Pin
#define KB_COL_02	KeySige2_Pin
#define KB_COL_03	KeySige3_Pin

// ******************************************************************
//	(KeyCom_Pin, KeySige1_Pin) 	=> OneKeyRun		一键起降(长按)
//	(Gnd, KeySige1_Pin) 		=> ChangeSpeed		速度转换（高速/低速）
//	(KeyCom_Pin, KeySige2_Pin) 	=> MicroAdjMode		微调模式
//	(Gnd, KeySige2_Pin) 		=> StuntShow		特技表演
//	(KeyCom_Pin, KeySige3_Pin) 	=> HeadlessMode		无头模式(长按)
//	(Gnd, KeySige3_Pin) 		=> OnOffKey			On/Off
// ******************************************************************

typedef enum{
	KEY_Matrix_none=0,        	        //按键释放
	KEY_Matrix_01=0x01,
	KEY_Matrix_02=0x02,
	KEY_Matrix_03=0x03,
	KEY_Matrix_04=0x04,
	KEY_Matrix_05=0x05,
	KEY_Matrix_06=0x06,
	KEY_Matrix_LAST,
}KB_type_def;

#define KEY_Released 		KEY_Matrix_none
#define KEY_ChangeSpeed		KEY_Matrix_01				//速度转换（高速/低速）
#define KEY_OneKeyRun		KEY_Matrix_02				//一键起降
#define KEY_StuntShow		KEY_Matrix_03				//特技表演
#define KEY_MicroAdjMode	KEY_Matrix_04				//微调模式
#define KEY_OnOffKey		KEY_Matrix_05				//On/Off
#define KEY_HeadlessMode	KEY_Matrix_06				//无头模式(长按)

#if Used_CountKB
#define cCountKBNum		3		//需要计数按键的个数

#define CountKeySel_01	KEY_Forward		//定义选择的第1计数按键
#define CountKeyVal01	0xE1

#define CountKeySel_02	KEY_Backward	//定义选择的第2计数按键
#define CountKeyVal02	0xE2

#define CountKeySel_03	KEY_Left	//定义选择的第3计数按键
#define CountKeyVal03	0xE3
#endif		//Used_CountKB

#if Used_LongKB
#define cLongKBNum		3		//需要长按键的个数

#define LongKBSel_01	KEY_OneKeyRun		//定义选择的第1长按按键
#define cLongKB01_Tcnt	120					//第1长按按键时长（1.2S）
#define LongKBVal01		0xF1

#define LongKBSel_02	KEY_HeadlessMode		//定义选择的第2长按按键
#define cLongKB02_Tcnt	120						//第2长按按键时长（1.2S）
#define LongKBVal02		0xF2

#define LongKBSel_03	KEY_OnOffKey		//定义选择的第3长按按键
#define cLongKB03_Tcnt	120						//第3长按按键时长（1.2S）
#define LongKBVal03		0xF3

#endif	//Used_LongKB

//*******************************************************************
// ******************************************************************
extern unsigned char key_final_value;		//键盘扫描最终得到的有效键值（非KEY_Matrix_none）
extern unsigned char key_Action_Flag;		//扫到新有效键值时置位，处理键值功能时清除之
extern unsigned char Longkey_Action;		//扫到新有效长键值时置位，处理键值功能时清除之
extern unsigned char key_Pressed_Flag;		//表示按键是被按住的
extern unsigned char key_Released_Flag;		//表示按键是被释放开的
extern unsigned int key_Pressed_cnt;			//按住按键的时间计数值
extern unsigned int key_Released_cnt;		//释放开按键的时间计数值
extern unsigned char key_CtnTimes_Val;		//连续按键的次数输出值
extern unsigned char key_CtnAct_Flag;		//连续按键的次数输出值有效标志
extern unsigned char key_CtnAct_Value;		//连续按键的输出值
extern unsigned char key_CtnAdj_Flag;		//连续加减按键的输出值有效标志
extern unsigned char key_CtnAdj_Value;		//连续加减按键的输出值
extern unsigned char key_Released_Act;		//新有效键释放开

//*******************************************************************
// ******************************************************************
extern void KeyBoard_gpio_init(void);
void ClearKeyBoardFlags(void);
extern unsigned char KeyBoard_scan_func(void);
extern void KeyBoard_row_gpio_pulldowm(void);
extern void UserKeyBoardScanFunc(void);
extern void keyboard_enter_Sleep_setup(void);
//*******************************************************************
// ******************************************************************
#endif		//KEYBOARD_ENABLE
//*******************************************************************

//*******************************************************************
//*********************************************************
#if DirectKey_ENABLE
//*********************************************************
#define Used_CountdirKey	0	//需要使用连续按键时置1
#define Used_LongDirKey		0	//需要使用长按按键时置1

#define DirKey_In0	SpdAdjKey_Pin   //K1 低速/高速 调节按键
#define DirKey_In1	LightKey_Pin    //K2 灯开关按键
#define DirKey_In2	OnOffKey_Pin    //K3 软开关机按键

typedef enum{
	KeyReleased=0x00,    	//按键释放
	KEY_Direct1=0x01,     	//K1 低速/高速 调节按键
	KEY_Direct2=0x02,     	//K2 灯开关按键
	KEY_Direct3=0x04,     	//K3 软开关机按键
	KEY_Direct4=0x08,     	//
	KEY_Direct5=0x10,     	//
	KEY_Direct6=0x20,     	//
	KEY_Direct7=0x40,     	//
	KEY_Direct8=0x80,     	//
}DirKB_type_def;

// Define key bitmask, for key program
#define Dkey1_SpdAdj_Val	KEY_Direct1				//K1 低速/高速 调节按键
#define Dkey2_Light_Val		KEY_Direct2				//K2 灯开关按键
#define Dkey3_OnOff_Val		KEY_Direct3				//K3 软开关机按键
#define cAllKeysMask		0x07

#define LongDkSel_01	Dkey3_OnOff_Val		//定义选择的第3长按按键
#define cLongDk01_Tcnt	100				//第1长按按键时长（1S）

//*********************************************************
//变量
//*********************************************************
extern unsigned char Dkey_final_value;		//键盘扫描最终得到的有效键值（非KeyReleased）
extern unsigned char Dkey_Action_Flag;		//扫到新有效键值时置位，处理键值功能时清除之
extern unsigned char Dkey_Action_Long;		//扫到新有效长按键值时置位，处理键值功能时清除之
extern unsigned char Dkey_Pressed_Flag;		//表示按键是被按住的
extern unsigned char Dkey_Released_Flag;		//表示按键是被释放开的
extern unsigned int Dkey_Pressed_cnt;			//按住按键的时间计数值
extern unsigned int Dkey_Released_cnt;		//释放开按键的时间计数值

//*********************************************************
void	DirectKeyInital(void);
void ClearKeyFlags(void);
void Directkey_enter_Sleep_setup(void);
unsigned char Dkeyscan_scan_func(void);
void	DirectKeyScanFunc(void);

//*********************************************************
#endif
//*********************************************************

//*********************************************************
#if SwitchScan_ENABLE
//*********************************************************
#define Switch_Pin_01	SW_Pin1
#define Switch_Pin_02	SW_Pin2
#define Switch_Pin_03	SW_Pin3
#define Switch_Pin_04	SW_Pin4

typedef enum{
	SW_NONE	= 0x00,    	//Switch none
	SW_Positin1=0x01,     	//
	SW_Positin2=0x02,     	//
	SW_Positin3=0x03,     	//
	SW_Positin4=0x04,     	//
}SwitchPos_type_def;

void SwitchPosInital(void);
unsigned char SwitchPos_scan_func(void);

//*********************************************************
#endif	//SwitchScan_ENABLE
//*********************************************************

//********************************************************************



//*******************************************************************
#endif /* VENDOR_ROBIN_24G_REMOTE_USERKEYBOARD_H_ */
//*******************************************************************


