//*******************************************************************
/*
 * UserKeyboard.c
 *
 *  Created on: 2026年8月19日
 *      Author: HuangGuangjian
 */
//*******************************************************************
//*******************************************************************
#include "app_config.h"
#include "driver.h"
#include "UserKeyboard.h"
#include <UserTxdFunc.h>
#include "app_toy/driver_layer/hw_gpio.h"
//********************************************************************
//********************************************************************
#if KEYBOARD_ENABLE
//********************************************************************
#define KB_COL_NUM   3	//列
#define KB_ROW_NUM   2	//行

const GPIO_PinTypeDef gpio_column[KB_COL_NUM]={KB_COL_01, KB_COL_02, KB_COL_03};//Input
const GPIO_PinTypeDef gpio_row[KB_ROW_NUM]   ={KB_ROW_01,KB_ROW_02};//Output

const unsigned char key_table[KB_ROW_NUM][KB_COL_NUM] = {//Key_value
		{ KEY_Matrix_01, KEY_Matrix_03, KEY_Matrix_05},
		{ KEY_Matrix_02, KEY_Matrix_04, KEY_Matrix_06},
};

#if Used_CountKB
const unsigned char ComtinueKBSel_Tab[KB_ROW_NUM][cCountKBNum] = {CountKeySel_01, CountKeySel_02, CountKeySel_03};
const unsigned char ComtinueKBval_Tab[KB_ROW_NUM][cCountKBNum] = {CountKeyVal01, CountKeyVal02, CountKeyVal03};
#endif

//********************************************************************
//********************************************************************
void keyboard_row_gpio_init(GPIO_PinTypeDef row_pin,GPIO_PullTypeDef gpio_pull)
{
	gpio_set_func(row_pin,AS_GPIO);
	gpio_set_output_en(row_pin,0);
	gpio_set_input_en(row_pin,1);
	gpio_write(row_pin,0);
	gpio_setup_up_down_resistor(row_pin,gpio_pull);
}

void keyboard_col_gpio_init(GPIO_PinTypeDef col_pin,GPIO_PullTypeDef col_pull)
{
	gpio_set_func(col_pin,AS_GPIO);
	gpio_set_output_en(col_pin,0);
	gpio_set_input_en(col_pin,1);
	gpio_setup_up_down_resistor(col_pin,col_pull);
	gpio_write(col_pin,0);
	cpu_set_gpio_wakeup(col_pin, Level_Low, 1);
}


//********************************************************************
//********************************************************************
void keyboard_set_row(const GPIO_PinTypeDef* ks_row,unsigned char row_cnt, GPIO_PullTypeDef gpio_pull)
{
	unsigned char i_cnt = 0;

	for(i_cnt = 1;i_cnt < row_cnt;i_cnt++){
		keyboard_row_gpio_init(ks_row[i_cnt],gpio_pull);
	}
}

void keyboard_set_col(const GPIO_PinTypeDef* ks_col,unsigned char col_cnt, GPIO_PullTypeDef col_pull)
{
	unsigned char i_cnt = 0;

	for(i_cnt = 0;i_cnt < col_cnt;i_cnt++){
		keyboard_col_gpio_init(ks_col[i_cnt],col_pull);
	}
}

//********************************************************************
//********************************************************************
void KeyBoard_gpio_init(void)
{
	keyboard_set_row(gpio_row,KB_ROW_NUM,PM_PIN_PULLDOWN_100K);
	keyboard_set_col(gpio_column,KB_COL_NUM,PM_PIN_PULLDOWN_100K);
}

void ClearKeyBoardFlags(void)
{
	key_final_value = 0;
	key_Action_Flag = 0;
	Longkey_Action = 0;
	key_Pressed_Flag = 0;
	key_Released_Flag = 0;
	key_Pressed_cnt = 0;
	key_Released_cnt = 0;
}

//********************************************************************
//********************************************************************
#if 1
unsigned char KeyBoard_scan_func(void)
{
	unsigned char row_i,col_j;
	unsigned char key_value = 0;

	keyboard_set_col(gpio_column,KB_COL_NUM,PM_PIN_PULLUP_10K);
	keyboard_set_row(gpio_row,KB_ROW_NUM,PM_PIN_PULLUP_10K);//设置行IO为浮空

	for(row_i=0;row_i<KB_ROW_NUM;row_i++)
	{
		if(row_i != 0)
		{
			gpio_set_input_en(gpio_row[row_i],0);//disable input
			gpio_write(gpio_row[row_i], 0);
			gpio_set_output_en(gpio_row[row_i], 1); //enable output
			//gpio_setup_up_down_resistor(gpio_row[row_i],PM_PIN_PULLDOWN_100K);
		}
		sleep_us(20);

		for(col_j=0;col_j<KB_COL_NUM;col_j++)
		{
			if(gpio_read(gpio_column[col_j])==0)
			{
				//key_value |= key_table[row_i][col_j];
				key_value = key_table[row_i][col_j];
				break;
			}
		}
		gpio_set_output_en(gpio_row[row_i], 0); //enable output
		gpio_set_input_en(gpio_row[row_i],1);//disable input
		if(key_value)	break;
	}
	keyboard_set_col(gpio_column,KB_COL_NUM,PM_PIN_PULLDOWN_100K);
	keyboard_set_row(gpio_row,KB_ROW_NUM,PM_PIN_PULLDOWN_100K);//设置行IO为浮空
	return key_value;
}

#else

unsigned int KeyBoard_scan_func(void)
{
	unsigned char row_i,col_j;
	unsigned int key_value = KEY_Matrix_none;

	for(row_i=0;row_i<KB_ROW_NUM;row_i++)
	{
		if(row_i!=0)
		{
			gpio_setup_up_down_resistor(gpio_row[row_i],PM_PIN_PULLDOWN_100K);
			sleep_us(20);

		}

		for(col_j=0;col_j<KB_COL_NUM;col_j++)
		{
			if(gpio_read(gpio_column[col_j])==0)
			{//有按键按下
				key_value=key_value | key_table[row_i][col_j];
			}
		}

		if(row_i!=0)
		{
			gpio_setup_up_down_resistor(gpio_row[row_i],PM_PIN_PULLUP_1M);
			sleep_us(20);
		}
	}
	return key_value;
}

#endif
//********************************************************************
//********************************************************************
void KeyBoard_row_gpio_pulldowm(void)
{
	unsigned char i;
	for(i=1;i<KB_ROW_NUM;i++)
		gpio_setup_up_down_resistor(gpio_row[i],PM_PIN_PULLDOWN_100K);
}

//********************************************************************
unsigned char key_Scan_Buffer=0;
unsigned char key_Debounce_cnt = 0;
unsigned char key_final_value=0;		//键盘扫描最终得到的有效键值（非KEY_Matrix_none）
unsigned char key_Action_Flag=0;		//扫到新有效键值时置位，处理键值功能时清除之
unsigned char Longkey_Action=0;			//扫到新有效长键值时置位，处理键值功能时清除之
unsigned char key_Pressed_Flag=0;		//表示按键是被按住的
unsigned char key_Released_Flag=0;		//表示按键是被释放开的
unsigned int key_Pressed_cnt=0;			//按住按键的时间计数值
unsigned int key_Released_cnt=0;		//释放开按键的时间计数值
unsigned char key_Released_Act=0;		//新有效键释放开

unsigned char key_Count_Flag=0;		//=1表示开始计算连续按键
unsigned char key_CtnTimes_Cnt=0;		//连续按键的次数计数
unsigned char key_CtnTimes_Val=0;		//连续按键的次数输出值
unsigned char key_CtnAct_Flag=0;		//连续按键的次数输出值有效标志
unsigned char key_CtnAct_Value=0;		//连续按键的输出值
unsigned char key_CtnAdj_Flag=0;		//连续加减按键的输出值有效标志
unsigned char key_CtnAdj_Value=0;		//连续加减按键的输出值

//********************************************************************
//********************************************************************
void UserKeyBoardScanFunc(void)
{
	unsigned char key_value = 0;
	key_value = KeyBoard_scan_func();
	if(key_value==key_Scan_Buffer)
	{
		if(key_Debounce_cnt < 4)
		{
			key_Debounce_cnt++;
		}
		else
		{

			if(key_Scan_Buffer==KEY_Matrix_none)
			{
				key_Pressed_Flag=0;
				if(key_Released_Flag==0)
				{
					key_Released_Flag=1;
					key_Released_Act=1;
				}
				key_Released_cnt++;
				key_Pressed_cnt=0;

				key_CtnAdj_Flag=0;
				key_CtnAdj_Value=KEY_Matrix_none;

				if(key_Released_cnt > 60)		// >600mS, 连续按键的间隔不应超过600mS
				{
					#if Used_CountKB
					if(key_Count_Flag)
					{
						key_Count_Flag=0;
						key_CtnAct_Flag=1;		//连续按键的次数输出值有效标志
						key_CtnTimes_Val=key_CtnTimes_Cnt;		//连续按键的次数输出值
						if((key_final_value == CountKeySel_01)&&(key_CtnTimes_Val==9))
						{
							key_CtnAct_Value=CountKeyVal01;
						}
						else if((key_final_value == CountKeySel_02)&&(key_CtnTimes_Val==3))
						{
							key_CtnAct_Value=CountKeyVal02;
						}
						else if((key_final_value == CountKeySel_03)&&(key_CtnTimes_Val==5))
						{
							key_CtnAct_Value=CountKeyVal03;
						}
						else
						{
							key_CtnAct_Flag=0;		//连续按键的次数输出值有效标志
							key_CtnAct_Value=KEY_Matrix_none;
						}
					}
					#endif	//Used_CountKB
				}
			}
			else
			{
				key_Released_Flag=0;
				if(key_Pressed_Flag==0)
				{
					key_final_value=key_Scan_Buffer;
					key_Action_Flag=1;
					key_Pressed_Flag=1;
					key_Pressed_cnt=4;
#if	Used_CountKB
					if(key_Count_Flag==0)
					{
						key_Count_Flag=1;		//=1表示开始计算连续按键
						key_CtnTimes_Cnt=1;		//连续按键的次数计数
					}
					else
					{
						key_CtnTimes_Cnt++;
					}
#endif	//Used_CountKB
				}
				else
				{
//					if(key_final_value!=key_Scan_Buffer)
//					{
//						key_final_value=key_Scan_Buffer;
//						key_Action_Flag=1;
//						key_Pressed_cnt=0;
//					}
					key_Pressed_cnt++;
					key_Released_cnt=0;
#if Used_LongKB
					if(key_final_value==LongKBSel_01)
					{
						if(key_Pressed_cnt == cLongKB01_Tcnt)		//Long pressed 1.2S
						{
							Longkey_Action=1;
							key_final_value=LongKBVal01;
						}
					}
					else if(key_final_value==LongKBSel_02)			//长按键1.2秒钟
					{
						if(key_Pressed_cnt == cLongKB02_Tcnt)		//Long pressed 1.2S
						{
							Longkey_Action=1;
							key_final_value=LongKBVal02;
						}

					}
					else if(key_final_value==LongKBSel_03)			//长按键1.2秒钟
					{
						if(key_Pressed_cnt == cLongKB03_Tcnt)		//Long pressed 1.2S
						{
							Longkey_Action=1;
							key_final_value=LongKBVal03;
						}

					}
#endif	//Used_LongKB

#if Used_ContinueKB
					unsigned char Temp=0;
					for(unsigned char i=0; i<cCountKBNum; i++)
					{
						if(key_final_value == ComtinueKBSel_Tab[i])
						{
							Temp=i+1;
							break;
						}
					}
					if(Temp)
					{
						if(key_Pressed_cnt > 100)				//连续加减按键以按住键超过1S开始有效
						{
							key_CtnAdj_Flag=1;
							key_CtnAdj_Value=ComtinueKBSel_Tab[Temp-1];
						}
					}
#endif	//Used_ContinueKB
				}
			}

		}
	}
	else
	{
		key_Debounce_cnt=0;
		key_Scan_Buffer=key_value;
	}
//scan key board finished

}

//********************************************************************
//********************************************************************
void keyboard_set_col_Wakeup(const GPIO_PinTypeDef* ks_col,unsigned char col_cnt)
{
	unsigned char i_cnt = 0;

	for(i_cnt = 0;i_cnt < col_cnt;i_cnt++)
	{
		if(gpio_read(ks_col[i_cnt])==1)
		{
			cpu_set_gpio_wakeup(ks_col[i_cnt], Level_Low, 1);
		}
		else
		{
			cpu_set_gpio_wakeup(ks_col[i_cnt], Level_High, 1);
		}
	}
}

//********************************************************************
//********************************************************************
void keyboard_enter_Sleep_setup(void)
{
	KeyBoard_row_gpio_pulldowm();
	keyboard_set_col(gpio_column,KB_COL_NUM,PM_PIN_PULLUP_1M);
	sleep_us(20);
	keyboard_set_col_Wakeup(gpio_column,KB_COL_NUM);
}

//********************************************************************
//********************************************************************
#endif		//KEYBOARD_ENABLE
//********************************************************************

//*******************************************************************
#if DirectKey_ENABLE
//*******************************************************************************
#define DirectKey_NUM   3

const GPIO_PinTypeDef gpio_DirectKey[DirectKey_NUM]={DirKey_In0, DirKey_In1, DirKey_In2};//Direct Key的IO
const unsigned char Dkey_table[DirectKey_NUM] = {KEY_Direct1, KEY_Direct2, KEY_Direct3};	//按键表格
//*******************************************************************************
//*******************************************************************************
void	DirectKeyInital(void)
{
	unsigned char i;
	for(i=0;i<DirectKey_NUM;i++)
	{
		gpio_set_func(gpio_DirectKey[i] ,AS_GPIO);
		gpio_setup_up_down_resistor(gpio_DirectKey[i] ,PM_PIN_PULLUP_10K);			//设置pull up
		gpio_set_output_en(gpio_DirectKey[i], 0); //Disable output
		gpio_set_input_en(gpio_DirectKey[i] ,1);//enable input
		//gpio_write(gpio_DirectKey[i], 0);

	}

}

void ClearKeyFlags(void)
{
	Dkey_final_value = 0;
	Dkey_Action_Flag = 0;
	Dkey_Action_Long = 0;
	Dkey_Pressed_Flag = 0;
	Dkey_Released_Flag = 0;
	Dkey_Pressed_cnt = 0;
	Dkey_Released_cnt = 0;
}

void Directkey_enter_Sleep_setup(void)
{
	DirectKeyInital();
//	unsigned char i;
//	for(i=0;i<DirectKey_NUM;i++)
//	{
//		gpio_setup_up_down_resistor(gpio_DirectKey[i],PM_PIN_PULLUP_1M);
//		cpu_set_gpio_wakeup(gpio_DirectKey[i], Level_Low, 1);	//输出使能开启
//	}
	cpu_set_gpio_wakeup(DirKey_In2, Level_Low, 1);	//输出使能开启
}

unsigned char Dkeyscan_scan_func(void)
{
	unsigned char Dir_i;
	unsigned char key_value = 0;
	for(Dir_i=0;Dir_i<DirectKey_NUM;Dir_i++)
	{
		gpio_setup_up_down_resistor(gpio_DirectKey[Dir_i] ,PM_PIN_PULLUP_10K);			//设置上拉10K

	}
	//DirectKeyInital();
	sleep_us(20);

	for(Dir_i=0;Dir_i<DirectKey_NUM;Dir_i++)
	{
		if(gpio_read(gpio_DirectKey[Dir_i])==0)
		{//有按键按下
			key_value=key_value | Dkey_table[Dir_i];
		}
	}
	for(Dir_i=0;Dir_i<DirectKey_NUM;Dir_i++)
	{
		gpio_setup_up_down_resistor(gpio_DirectKey[Dir_i] ,PM_PIN_PULLDOWN_100K);			//设置下拉100K
	}
	return key_value;
}

//********************************************************************
unsigned char Dkey_Onoff_bak=0;
unsigned char Dkey_Scan_Buffer=0;
unsigned char Dkey_Debounce_cnt = 0;
unsigned char Dkey_final_value=0;		//键盘扫描最终得到的有效键值（非KeyReleased）
unsigned char Dkey_Action_Flag=0;		//扫到新有效键值时置位，处理键值功能时清除之
unsigned char Dkey_Action_Long=0;		//扫到新有效长按键值时置位，处理键值功能时清除之
unsigned char Dkey_Pressed_Flag=0;		//表示按键是被按住的
unsigned char Dkey_Released_Flag=0;		//表示按键是被释放开的
unsigned int Dkey_Pressed_cnt=0;		//按住按键的时间计数值
unsigned int Dkey_Released_cnt=0;		//释放开按键的时间计数值
//********************************************************************
void	DirectKeyScanFunc(void)
{
	unsigned char key_value = 0;
	key_value = Dkeyscan_scan_func();
	if(g_u8FirstPowerOn)
	{
		g_u8FirstPowerOn=0;
		if(key_value == Dkey3_OnOff_Val)
		{
			Dkey_Debounce_cnt=4;
			Dkey_Scan_Buffer=key_value;
		}
	}

	if(key_value==Dkey_Scan_Buffer)
	{
		if(Dkey_Debounce_cnt < 4)
		{
			Dkey_Debounce_cnt++;
		}
		else
		{
			if(Dkey_Scan_Buffer==KeyReleased)
			{
				if((Dkey_Pressed_Flag)&&(Dkey_Pressed_cnt < cLongDk01_Tcnt))
				{
					if((Dkey_final_value==LongDkSel_01) && (FunCtrlFlag & WorkingStatus))
					{
						if(Dkey_Onoff_bak==0)
						{
							Dkey_Action_Flag=1;
						}
						else {
							Dkey_Onoff_bak=0;
						}
					}

				}
				Dkey_Pressed_Flag=0;
				Dkey_Released_Flag=1;
				Dkey_Released_cnt++;
				Dkey_Pressed_cnt=0;
			}
			else
			{
				Dkey_Released_Flag=0;
				if(Dkey_Pressed_Flag==0)
				{
					Dkey_final_value=Dkey_Scan_Buffer;
					Dkey_Pressed_Flag=1;
					if((Dkey_final_value != LongDkSel_01) || ((FunCtrlFlag & WorkingStatus) ==0))
					{
						Dkey_Action_Flag=1;
						if((FunCtrlFlag & WorkingStatus) ==0)
						{
							Dkey_Onoff_bak=1;
						}
					}
				}
				else
				{
					if(Dkey_final_value!=Dkey_Scan_Buffer)
					{
						Dkey_final_value=Dkey_Scan_Buffer;
//						Dkey_Action_Flag=1;
						Dkey_Pressed_cnt=0;
					}
					else {
						Dkey_Pressed_cnt++;
					}
					Dkey_Released_cnt=0;

#if Used_LongDirKey
					if(Dkey_final_value==LongDkSel_01)
					{
						if(Dkey_Pressed_cnt >= cLongDk01_Tcnt)		//Long pressed 1S
						{
							Dkey_Action_Long=1;
						}
					}
#endif	//Used_LongDirKey
				}
			}

		}
	}
	else
	{
		Dkey_Debounce_cnt=0;
		Dkey_Scan_Buffer=key_value;
	}
//scan key board finished

}

//*******************************************************************************
#endif
//*******************************************************************************

//*******************************************************************
#if SwitchScan_ENABLE
//*******************************************************************************
#define SwitchPos_NUM   4

const GPIO_PinTypeDef gpio_SwitchPos[SwitchPos_NUM]={Switch_Pin_01, Switch_Pin_02, Switch_Pin_03, Switch_Pin_04};
const unsigned char Switch_table[SwitchPos_NUM] = {SW_Positin1, SW_Positin2, SW_Positin3, SW_Positin4};		//Switch表格
//*******************************************************************************
//*******************************************************************************
void SwitchPosInital(void)
{
	unsigned char i;
	for(i=0;i<SwitchPos_NUM;i++)
	{
		gpio_set_func(gpio_SwitchPos[i] ,AS_GPIO);
		gpio_setup_up_down_resistor(gpio_SwitchPos[i] ,PM_PIN_PULLDOWN_100K);			//设置pull down
		gpio_set_output_en(gpio_SwitchPos[i], 0); //Disable output
		gpio_set_input_en(gpio_SwitchPos[i] ,1);//enable input
		//gpio_write(gpio_SwitchPos[i], 0);
	}

}

unsigned char SwitchPos_scan_func(void)
{
	unsigned char SW_i;
	unsigned char sw_value = 0;
	SwitchPosInital();
	for(SW_i=0;SW_i<SwitchPos_NUM;SW_i++)
	{
		gpio_setup_up_down_resistor(gpio_SwitchPos[SW_i] ,PM_PIN_PULLUP_10K);			//设置上拉10K

	}
	sleep_us(20);

	for(SW_i=0;SW_i<SwitchPos_NUM;SW_i++)
	{
		if(gpio_read(gpio_SwitchPos[SW_i])==0)
		{//有按键按下
			sw_value=Switch_table[SW_i];
			break;
		}
	}
	if(SW_i == SwitchPos_NUM)
	{
		sw_value = SW_Positin1;
	}
	for(SW_i=0;SW_i<SwitchPos_NUM;SW_i++)
	{
		gpio_setup_up_down_resistor(gpio_SwitchPos[SW_i] ,PM_PIN_PULLDOWN_100K);			//设置下拉100K
	}
	return sw_value;
}

//*******************************************************************************
#endif	//SwitchScan_ENABLE
//*******************************************************************************
//*******************************************************************************
//*******************************************************************************


