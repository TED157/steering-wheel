#include "RefereeThread.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "CMS.h"
#include "RefereeBehaviour.h"
#include "Client_UI.h"
#include "CanPacket.h"
#include "CalculateThread.h"
#include "UIThread.h"

//需要添加UI慢速
//需要添加底盘停止的UI 
//Graph_Data deng1,kuang,deng2,deng3,deng4,fu,imagex,imagex1,imagex2,imagex3,imagey,imagey1,imagey2,imagey3,imagey4;
//String_Data bullet,bullet3,DAFU,Abuff,Pbuff,Cbuff,state,ZIMIAO,dafustate,zimiaostate,dafustate1,zimiaostate1,state4;
//Float_Data capacityD,Min,Sec;
//int Time=0,M=10,S=0;
//int BTime=0,BM=10,BS=0;
//Float_Data BMin,BSec;

//Graph_Data Frame;//自瞄框
////准星横线
//Graph_Data Collimation_1;
//Graph_Data Collimation_2;
//Graph_Data Collimation_3;
//Graph_Data Collimation_4;
////准星数线
//Graph_Data Collimation_5;
////准心距离线
//Graph_Data Collimation_6;
//Graph_Data Collimation_7;
//Graph_Data Collimation_8;
//Graph_Data Collimation_9;
//Graph_Data Collimation_10;
//Graph_Data Collimation_11;

//String_Data Capcity,HitRune,AiMBot;
//Float_Data CapData;
//String_Data HitRuneStatus,AiMBotStatus;

//String_Data BulletCover,ChassisStatue;
//Graph_Data BulletCircle,ChassisStatueCircle;

//String_Data ChassisMove;
//Graph_Data ChassisMoveCircle;

//String_Data s_rune,b_rune;

//uint32_t flash_time = 0;
//uint8_t s_rune_flag = 0;
//uint8_t b_rune_flag = 0;
//int s_time_rune;
//int b_time_rune;

Graph_Data imagex,imagey,x1,x6,x7,x8,x9,x10,x11,x12,x13,x14;
Graph_Data circle1,circle2,circle3,circle4;
Graph_Data Pingheng;
String_Data Ammo,Heat,aimbot,autofire,Mode,close1,open1,open2,close2,open3,close3,open4,close4,open5,close5,roting,noforce,fallow,stop;
String_Data capcity;
String_Data rune;
Float_Data CapData;
int mode_flag=0;

extern float CapChageVoltage;
extern EulerSystemMeasure_t    Imu;
extern DMA_HandleTypeDef hdma_usart6_tx;
ChassisMode_e mode=NOFORCE;
uint8_t fire_mode,shoot_mode,cover_mode,aim_mode;
uint8_t mode_change_flag;//bit 0-7 底盘模式,自动开火，单双发，弹舱盖开合，打击对象

void UI(void const * argument)
{
	uint16_t flashtime=0;
	num=0;top=0;
	osDelay(1000);
	mode=NOFORCE;
	//固定UI图层
	//参考线
	Line_Draw(&imagex,"xck",UI_Graph_ADD,0,UI_Color_Green,2,6900,540,7310,540);
	Line_Draw(&imagey,"yck",UI_Graph_ADD,0,UI_Color_Green,1,7105,900,7105,100);
	Line_Draw(&x1,"x01",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,520,7160,520);
	Line_Draw(&x6,"x06",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,420,7160,420);
	Line_Draw(&x13,"x13",UI_Graph_ADD,0,UI_Color_Pink,6,6706,40,6910,340);
	Line_Draw(&x14,"x14",UI_Graph_ADD,0,UI_Color_Pink,6,7504,40,7300,340);
	Float_Draw(&CapData,"cpd",UI_Graph_ADD,0,UI_Color_Yellow,14,5,2,878,158,CMS_Data.cms_cap_v*1000);
	UI_ReFresh(7,imagex,imagey,x1,x6,x13,x14,CapData);	
	//模式切换
	Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,14,27,2,78,782,"MODE\nFIRE\nSINGLE\nCOVER\nAMMO");
	Char_ReFresh(Mode);
	//默认模式	
	Char_Draw(&noforce,"nof",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,782,"noforce");
	Char_ReFresh(noforce);
	Char_Draw(&open2,"opp",UI_Graph_ADD,1,UI_Color_Purplish_red,16,4,2,238,741,"manu");
	Char_ReFresh(open2);
	Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,700,"close ");
	Char_ReFresh(open3);
	
	if( (PTZ.PTZStatusInformation   & 16 ) == 16)
	{
		Char_Draw(&open1,"op",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,659,"open  ");
		Char_ReFresh(open1);
	}
	else
	{
		Char_Draw(&open1,"op",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,659,"close ");
		Char_ReFresh(open1);		
	}
	Char_Draw(&rune,"run",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,618,"NORMAL");
	Char_ReFresh(rune);
	while(1)
    {
		if(Chassis.Mode==FALLOW&&mode==NOFORCE){
			//固定UI图层
			//参考线
			Line_Draw(&imagex,"xck",UI_Graph_ADD,0,UI_Color_Green,2,6900,540,7310,540);
			Line_Draw(&imagey,"yck",UI_Graph_ADD,0,UI_Color_Green,1,7105,900,7105,100);
			Line_Draw(&x1,"x01",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,520,7160,520);
			Line_Draw(&x6,"x06",UI_Graph_ADD,0,UI_Color_Cyan,1,7050,420,7160,420);
			Line_Draw(&x13,"x13",UI_Graph_ADD,0,UI_Color_Pink,6,6706,40,6910,340);
			Line_Draw(&x14,"x14",UI_Graph_ADD,0,UI_Color_Pink,6,7504,40,7300,340);
			Float_Draw(&CapData,"cpd",UI_Graph_ADD,0,UI_Color_Yellow,14,5,2,878,158,CMS_Data.cms_cap_v*1000);
			UI_ReFresh(7,imagex,imagey,x1,x6,x13,x14,CapData);	
			//模式切换
			Char_Draw(&Mode,"mod",UI_Graph_ADD,0,UI_Color_Green,14,27,2,78,782,"MODE\nFIRE\nSINGLE\nCOVER\nAMMO");
			Char_ReFresh(Mode);
			//默认模式	
			Char_Draw(&noforce,"nof",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,782,"fallow ");
			Char_ReFresh(noforce);
			Char_Draw(&open2,"opp",UI_Graph_ADD,1,UI_Color_Purplish_red,16,4,2,238,741,"manu");
			Char_ReFresh(open2);
			Char_Draw(&open3,"oop",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,700,"close ");
			Char_ReFresh(open3);
			Char_Draw(&open1,"op",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,659,"close ");
			Char_ReFresh(open1);		
			Char_Draw(&rune,"run",UI_Graph_ADD,1,UI_Color_Purplish_red,16,7,2,238,618,"NORMAL");
			Char_ReFresh(rune);
			mode=FALLOW;
			fire_mode=0x00;
			shoot_mode=0x00;
			cover_mode=0x00;
		}
		Float_Draw(&CapData,"cpd",UI_Graph_ADD,0,UI_Color_Yellow,14,5,2,878,158,CMS_Data.cms_cap_v*1000);
		UI_ReFresh(1,CapData);
		mode_change_flag=0x00;
		//模式切换检测
		//底盘模式切换
		if(Chassis.Mode != mode){
			mode_change_flag |= (uint8_t) (1 << 0);
			mode=Chassis.Mode;
		}
		//开火模式转换
		if(fire_mode!=(PTZ.PTZStatusInformation&64)){
			mode_change_flag |= (uint8_t) (1<<1);
			fire_mode=(PTZ.PTZStatusInformation&64);
		}
		//单双发模式切换
		if(shoot_mode!=(PTZ.AimTargetRequest&0x02)){
			mode_change_flag |= (uint8_t) (1<<2);
			shoot_mode=(PTZ.AimTargetRequest&0x02);
		}
		//弹舱盖开合
		if(cover_mode!=(PTZ.PTZStatusInformation&16)){
			mode_change_flag |= (uint8_t) (1<<3);
			cover_mode=(PTZ.PTZStatusInformation&16);
		}
		//打击对象切换
		if(aim_mode!=(PTZ.AimTargetRequest&0x31)){
			mode_change_flag |= (uint8_t) (1<<4);
			aim_mode=(PTZ.AimTargetRequest&0x31);
		}
		//************************************底盘模式**********************************
		if(mode_change_flag&0x01){
			if(Chassis.Mode == NOFORCE)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,770,"noforce");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == ROTING)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Green,16,7,2,6820,770,"rotate");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == FALLOW)
			{ 
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Cyan,16,7,2,6820,770,"fallow");
				Char_ReFresh(noforce);
			}
			else if(Chassis.Mode == STOP)
			{
				Char_Draw(&noforce,"nof",UI_Graph_Change,1,UI_Color_Cyan,16,7,2,6820,770,"stop");
				Char_ReFresh(noforce);
			}
		}
		//************************************自动开火********************************
		if(mode_change_flag&0x02){
			if(  (PTZ.PTZStatusInformation     &  64) == 64)
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,1,UI_Color_Purplish_red,16,4,2,6820,740,"auto");
				Char_ReFresh(open2);
			}
			else
			{
				Char_Draw(&open2,"opp",UI_Graph_Change,1,UI_Color_Purplish_red,16,4,2,6820,740,"manu");
				Char_ReFresh(open2);	
			}	
		}
		//*********************************单发*********************************
		if(mode_change_flag&0x04){
			if(   (PTZ.AimTargetRequest & 0x02) == 0x02){
				Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,710,"open  ");
				Char_ReFresh(open3);
			}
			else
			{
				Char_Draw(&open3,"oop",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,710,"close ");
				Char_ReFresh(open3);	
			}	
		}
		//************************************弹舱盖**********************************	
		if(mode_change_flag&0x08){
			if( (PTZ.PTZStatusInformation   & 16 ) == 16)
			{
				Char_Draw(&open1,"op",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,800,"open  ");
				Char_ReFresh(open1);
			}
			else
			{
				Char_Draw(&open1,"op",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,800,"close ");
				Char_ReFresh(open1);		
			}
		} 
		/************************rune*******************************/
		if(mode_change_flag&0x10){
			if(PTZ.AimTargetRequest & 0x20)
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,830,"BIG");
				Char_ReFresh(rune);
			}
			else if(PTZ.AimTargetRequest & 0x10)
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,830,"SMALL");
				Char_ReFresh(rune);
			}
			else 
			{
				Char_Draw(&rune,"run",UI_Graph_Change,1,UI_Color_Purplish_red,16,7,2,6820,830,"NORMAL");
				Char_ReFresh(rune);	
			}
		}
        osDelay(1);
    }
 }