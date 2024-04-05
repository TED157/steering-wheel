#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"
#include "usart.h"
#include "MahonyAHRS.h"
#include "arm_math.h"
#include "CMS.h"

#define START_POWER 15.0f

Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;
extern ext_power_heat_data_t power_heat_data_t;
uint32_t F_Motor[8];
float WheelAngle[4];

fp32 wz;
fp32 roting_speed;
fp32 Angle_zero_6020[4] = {-73.3, -68.8, -124.8, 29.2};
//fp32 Angle_zero_6020[4] = {0, 0, 0, 0};
fp32 Direction[5] = {-1.0, -1.0, 1.0, 1.0, -1.0};
fp32 Maxspeed = 6000.0f;
fp32 speed[4];
fp32 angle[4];
KFP Power_kf;
fp32 Power_Max = 45.0f;
float angle_minus;
float run_per;
//power control
float last_speed[8] = {0};
fp32 he = 0;
float kp = 1.5 * 1.99999999e-06;
float lijupower = 0.0f;
uint8_t power_flag=0;
float power_scale;
fp32 v_gain=0;

uint8_t Mode_last;
uint8_t Mode_now;
uint8_t stop_flag=0;
uint8_t stop_countdown=0;

pid_type_def follow_yaw;
pid_type_def follow;
pid_type_def left_front_6020_speed_pid;
pid_type_def right_front_6020_speed_pid;
pid_type_def right_back_6020_speed_pid;
pid_type_def left_back_6020_speed_pid;
pid_type_def left_front_6020_position_pid;
pid_type_def right_front_6020_position_pid;
pid_type_def right_back_6020_position_pid;
pid_type_def left_back_6020_position_pid;
pid_type_def left_front_3508_pid;
pid_type_def right_front_3508_pid;
pid_type_def right_back_3508_pid;
pid_type_def left_back_3508_pid;
pid_type_def power_control_pid;
first_order_filter_type_t current_6020_filter_type;
first_order_filter_type_t current_3508_filter_type;
first_order_filter_type_t referee_power;
first_order_filter_type_t wheel_angle_1;
first_order_filter_type_t wheel_angle_2;
first_order_filter_type_t wheel_angle_3;
first_order_filter_type_t wheel_angle_4;
first_order_filter_type_t wz_filter;



fp32 follow_angle;
fp32 follow_yaw_PID[3]={0.08,0,1};
fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 left_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_front_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 right_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_back_6020_speed_PID[3] = {SPEED_6020_KP, SPEED_6020_KI, SPEED_6020_KD};
fp32 left_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_front_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 right_back_6020_position_PID[3] = {POSITION_6020_KP, POSITION_6020_KI, POSITION_6020_KD};
fp32 left_back_6020_position_PID[3] = {3.8, POSITION_6020_KI, 3.0};
fp32 left_front_3508_PID[3] = {speed_3508_KP*1.2, speed_3508_KI, speed_3508_KD};
fp32 right_front_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 right_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 left_back_3508_PID[3] = {speed_3508_KP, speed_3508_KI, speed_3508_KD};
fp32 power_control_PID[3] = {power_control_KP*1.2, power_control_KI, power_control_KD};

extern motor_measure_t LEFT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_FRONT_6020_Measure;
extern motor_measure_t RIGHT_BACK_6020_Measure;
extern motor_measure_t LEFT_BACK_6020_Measure;
extern motor_measure_t LEFT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_FRONT_3508_Measure;
extern motor_measure_t RIGHT_BACK_3508_Measure;
extern motor_measure_t LEFT_BACK_3508_Measure;
extern motor_measure_t YawMotorMeasure;
OfflineMonitor_t Offline;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void ChassisCurrentUpdate();
void RefereeInfUpdate(ext_game_robot_status_t *referee);
void ChassisInfUpdate();
void Angle_Speed_calc();
void CMS__();
uint8_t chassis_powerloop(Chassis_t *Chassis);

void CalculateThread(void const *pvParameters)
{

	ChassisInit();

	while (1)
	{
		//Remote = *get_remote_control_point();
		
		DeviceOfflineMonitorUpdate(&Offline);
		ChassisModeUpdate();
		ChassisInfUpdate();
		RefereeInfUpdate(&Referee);
		GimbalEulerSystemMeasureUpdate(&Imu);
		ChassisCommandUpdate();
		chassis_powerloop(&Chassis);
		CMS__();
		Chassis_Control(Chassis.Current[0],
						Chassis.Current[1],
						Chassis.Current[2],
						Chassis.Current[3],
						Chassis.Current[4],
						Chassis.Current[5],
						Chassis.Current[6],
						Chassis.Current[7]);
	
		osDelay(1);
	}
}

void ChassisInit()
{
	PID_init(&left_front_6020_speed_pid, PID_POSITION, left_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_front_6020_speed_pid, PID_POSITION, right_front_6020_speed_PID, 30000, 10000);
	PID_init(&right_back_6020_speed_pid, PID_POSITION, right_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_back_6020_speed_pid, PID_POSITION, left_back_6020_speed_PID, 30000, 10000);
	PID_init(&left_front_6020_position_pid, PID_POSITION, left_front_6020_position_PID, 300, 60);              //6020
	PID_init(&right_front_6020_position_pid, PID_POSITION, right_front_6020_position_PID, 300, 60);
	PID_init(&right_back_6020_position_pid, PID_POSITION, right_back_6020_position_PID, 300, 60);
	PID_init(&left_back_6020_position_pid, PID_POSITION, left_back_6020_position_PID, 300, 60);

	PID_init(&left_front_3508_pid, PID_POSITION, left_front_3508_PID, 16384, 1000);
	PID_init(&right_front_3508_pid, PID_POSITION, right_front_3508_PID, 16384, 1000);
	PID_init(&right_back_3508_pid, PID_POSITION, right_back_3508_PID, 16384, 1000);							//3508
	PID_init(&left_back_3508_pid, PID_POSITION, left_back_3508_PID, 16384, 1000);
	
	PID_init(&follow_yaw,PID_POSITION,follow_yaw_PID,1,1);
	PID_init(&follow,PID_POSITION,follow_PID,2,1);
	
	//KalmanFilter_init(&Power_kf, 0.0f , 0.0001f,0.0118f ,0.0,50.0,2.0);//A,B,P,Q,R                   //功率
	first_order_filter_init(&current_6020_filter_type,0.002,0.1);
	first_order_filter_init(&current_3508_filter_type,0.002,0.1);
	first_order_filter_init(&wheel_angle_1,0.001,0.1);
	first_order_filter_init(&wheel_angle_2,0.001,0.1);
	first_order_filter_init(&wheel_angle_3,0.001,0.1);
	first_order_filter_init(&wheel_angle_4,0.001,0.1);

	CMS_Data.charge_flag=1;
};

void ChassisInfUpdate()
{
	memcpy(&Chassis.Motor3508[0], &LEFT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[1], &RIGHT_FRONT_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[2], &RIGHT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor3508[3], &LEFT_BACK_3508_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[0], &LEFT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[1], &RIGHT_FRONT_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[2], &RIGHT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
	memcpy(&Chassis.Motor6020[3], &LEFT_BACK_6020_Measure, sizeof(LEFT_FRONT_3508_Measure));
}

void ChassisModeUpdate()
{
	
	switch (PTZ.ChassisStatueRequest)
	{
	case 0x01:
		Chassis.Mode = NOFORCE;
		break;
	case 0x12:
	case 0x32:
		Chassis.Mode = ROTING;
		break;
	case 0x0A:
	case 0x2A:
		Chassis.Mode = FALLOW;
		break;
	case 0x06:
	case 0x26:
		Chassis.Mode = STOP;
		break;

	default:
		break;
	}
	if((PTZ.ChassisStatueRequest & (0x01 << 5)) != 0)
	{
		Chassis.CapKey = 1;
	}
	else Chassis.CapKey = 0;
}

void ChassisCommandUpdate()
{
	
	
		
		//Chassis.wz = -Remote.rc.ch[2] / 660.0f * (1.0f + Chassis.Power_Proportion / Power_Max);

	if (Chassis.Mode == NOFORCE || Offline.PTZnode ==1)
	{
		Chassis.Current[0] = 0;
		Chassis.Current[1] = 0;
		Chassis.Current[2] = 0;
		Chassis.Current[3] = 0;
		Chassis.Current[4] = 0;
		Chassis.Current[5] = 0;
		Chassis.Current[6] = 0;
		Chassis.Current[7] = 0;
		return;
	}
	if (Chassis.Mode == FALLOW || Chassis.Mode == ROTING || Chassis.Mode == STOP ) //
	{
		follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
		
		if (Chassis.Mode == FALLOW)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			if(angle_minus>180) angle_minus-=360;
			else if(angle_minus<-180) angle_minus+=360;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI)) * (v_gain );
			Chassis.vy =  ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI)) * (v_gain );
			if(Chassis.vx*Chassis.vy !=0)  follow.max_out=0.9;
			else follow.max_out=2;
			Chassis.wz = -PID_calc(&follow,YawMotorMeasure.angle,follow_angle); //* (1.0f + Chassis.Power_Proportion /Power_Max );
			//Chassis.wz = 0;
		}
		else if (Chassis.Mode == ROTING)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle - YawMotorMeasure.speed_rpm * 0.27;
			Chassis.wz = sin(v_gain/4.2)*4;
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI));//* (1.0f + Chassis.Power_Proportion /Power_Max );
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI));//* (1.0f + Chassis.Power_Proportion /Power_Max );
		}
		else if (Chassis.Mode == STOP)
		{
			angle_minus = -YawMotorMeasure.angle + FollowAngle;
			Chassis.vy = ((PTZ.FBSpeed / 32767.0f) * cos(angle_minus/180.0*PI) - (PTZ.LRSpeed / 32767.0f) * sin(angle_minus/180.0*PI));
			Chassis.vx = ((PTZ.FBSpeed / 32767.0f) * sin(angle_minus/180.0*PI) + (PTZ.LRSpeed / 32767.0f) * cos(angle_minus/180.0*PI));
			Chassis.wz = 0.0;
		}
		
	}
			/********************************	6020角度解算         ***********************/ // 问题   i++  &&

		
		if (Fabs(PTZ.FBSpeed / 32767.0) > 0.05 || Fabs(PTZ.LRSpeed / 32767.0) > 0.05 )
		{
			for (uint8_t i = 0; i < 4; )
			{
				Chassis.WheelAngle[i] = atan2((Chassis.vy) + Chassis.wz * gen2 * Direction[i], (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1])) / 3.1415927 * 180.0 + Angle_zero_6020[i]; // ?????????
				i++;
			}
			stop_flag=1;
		}
		else
		{
			if(stop_flag==1)
			{
				stop_countdown=60;
				stop_flag=2;
			}
			if(stop_countdown!=1)
			{
				stop_countdown--;
			}
			else{
			stop_flag=0;
			if(Chassis.wz > 0)
			{
			Chassis.WheelAngle[0] = -135.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = -45.0f + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = 45.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 135.0f + Angle_zero_6020[3];
			}
			else if(Chassis.wz < 0)
			{
			Chassis.WheelAngle[0] = 45.0f + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 135.0f + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = -135.0f + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = -45.0f + Angle_zero_6020[3];							
			}	
			else if(Chassis.wz == 0)
			{
			Chassis.WheelAngle[0] = 0 + Angle_zero_6020[0];
			Chassis.WheelAngle[1] = 0 + Angle_zero_6020[1]; // 默认角度
			Chassis.WheelAngle[2] = 0 + Angle_zero_6020[2];
			Chassis.WheelAngle[3] = 0 + Angle_zero_6020[3];						
			}
		}
		}
		Chassis.WheelAngle[0] = loop_fp32_constrain(Chassis.WheelAngle[0], LEFT_FRONT_6020_Measure.angle - 180.0f, LEFT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[1] = loop_fp32_constrain(Chassis.WheelAngle[1], RIGHT_FRONT_6020_Measure.angle - 180.0f, RIGHT_FRONT_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[2] = loop_fp32_constrain(Chassis.WheelAngle[2], RIGHT_BACK_6020_Measure.angle - 180.0f, RIGHT_BACK_6020_Measure.angle + 180.0f);
		Chassis.WheelAngle[3] = loop_fp32_constrain(Chassis.WheelAngle[3], LEFT_BACK_6020_Measure.angle - 180.0f, LEFT_BACK_6020_Measure.angle + 180.0f);		
		/***********************                 3508速度解算                    ******************************/
				//电容的使用
		if(((CMS_Data.cms_status) & (uint16_t) 1) != 1 && CMS_Data.TxOpen == 1)
		{
			Chassis.vx = 2 * Chassis.vx ;
			Chassis.vy = 2 * Chassis.vy ;
			Chassis.wz = 2 * Chassis.wz ;
			CMS_Data.Mode = 1;
		}
		else CMS_Data.Mode = 0;

		for (uint8_t i = 0; i < 4;)
		{
			speed[i] = sqrtf((Chassis.vy + Chassis.wz * gen2 * Direction[i]) * (Chassis.vy + Chassis.wz * gen2 * Direction[i]) 
							+ (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]) * (Chassis.vx + Chassis.wz * gen2 * Direction[i + 1]));
			i++;
		}
		Chassis.WheelSpeed[0] = -speed[0];
		Chassis.WheelSpeed[1] = speed[1];
		Chassis.WheelSpeed[2] = speed[2];
		Chassis.WheelSpeed[3] = -speed[3];
		if(stop_flag==2){
			Chassis.WheelSpeed[0] = 0;
			Chassis.WheelSpeed[1] = 0;
			Chassis.WheelSpeed[2] = 0;
			Chassis.WheelSpeed[3] = 0;
		}
//		if(Fabs(Fabs(LEFT_FRONT_6020_Measure.angle-Chassis.WheelAngle[0])>1.5
//			||RIGHT_FRONT_6020_Measure.angle-Chassis.WheelAngle[1])>1.5 
//			|| Fabs(RIGHT_BACK_6020_Measure.angle-Chassis.WheelAngle[2])>1.5
//		    ||Fabs(LEFT_BACK_6020_Measure.angle-Chassis.WheelAngle[3])>1.5)
//			wheel_flag=1;

		Angle_Speed_calc(); // 角度优化

		Chassis.speed_6020[0] = PID_calc(&left_front_6020_position_pid, LEFT_FRONT_6020_Measure.angle, Chassis.WheelAngle[0]);
		Chassis.speed_6020[1] = PID_calc(&right_front_6020_position_pid, RIGHT_FRONT_6020_Measure.angle, Chassis.WheelAngle[1]);
		Chassis.speed_6020[2] = PID_calc(&right_back_6020_position_pid, RIGHT_BACK_6020_Measure.angle, Chassis.WheelAngle[2]);
		Chassis.speed_6020[3] = PID_calc(&left_back_6020_position_pid, LEFT_BACK_6020_Measure.angle, Chassis.WheelAngle[3]);
	


		
	ChassisCurrentUpdate();

	
	Mode_last = Mode_now;
	Mode_now = Chassis.Mode;

}

	

void Angle_Speed_calc()
{
	for (uint8_t i = 0; i < 4; )
	{
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle > 90.0f)
		{
			Chassis.WheelAngle[i] -= 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		if (Chassis.WheelAngle[i] - Chassis.Motor6020[i].angle < -90.0f)
		{
			Chassis.WheelAngle[i] += 180.0f;
			Chassis.WheelSpeed[i] = -Chassis.WheelSpeed[i];
		}
		i++;
	}
	
//	first_order_filter_cali(&wheel_angle_1,Chassis.WheelAngle[0]);
//			first_order_filter_cali(&wheel_angle_2,Chassis.WheelAngle[1]);
//			first_order_filter_cali(&wheel_angle_3,Chassis.WheelAngle[2]);
//			first_order_filter_cali(&wheel_angle_4,Chassis.WheelAngle[3]);
//			Chassis.WheelAngle[0] = wheel_angle_1.out;
//			Chassis.WheelAngle[1] = wheel_angle_2.out;
//			Chassis.WheelAngle[2] = wheel_angle_3.out;
//			Chassis.WheelAngle[3] = wheel_angle_4.out;
	
}

void ChassisCurrentUpdate()
{
	Chassis.Current[0] = PID_calc(&left_front_6020_speed_pid, LEFT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[0]);;
	Chassis.Current[1] = PID_calc(&right_front_6020_speed_pid, RIGHT_FRONT_6020_Measure.speed_rpm, Chassis.speed_6020[1]);
	Chassis.Current[2] = PID_calc(&right_back_6020_speed_pid, RIGHT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[2]);;
	Chassis.Current[3] = PID_calc(&left_back_6020_speed_pid, LEFT_BACK_6020_Measure.speed_rpm, Chassis.speed_6020[3]);
	
	Chassis.Current[4] = PID_calc(&left_front_3508_pid, LEFT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[0]);;
	Chassis.Current[5] = PID_calc(&right_front_3508_pid, RIGHT_FRONT_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[1]);
	Chassis.Current[6] = PID_calc(&right_back_3508_pid, RIGHT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[2]);
	Chassis.Current[7] = PID_calc(&left_back_3508_pid, LEFT_BACK_3508_Measure.speed_rpm / Maxspeed, Chassis.WheelSpeed[3]);
}

void RefereeInfUpdate(ext_game_robot_status_t *referee)
{
	memcpy(referee, &robot_state, sizeof(ext_game_robot_status_t));
	switch(referee->chassis_power_limit)
	{
		case 45:
			Power_Max = 45;kp=1.5 * 1.99999999e-06;v_gain=1;break;
		case 50:
			Power_Max = 50;kp=1.5 * 1.99999999e-06;v_gain=1;break;
		case 55:
			Power_Max = 55;kp=1.5 * 1.99999999e-06;v_gain=1;break;
		case 60:	
			Power_Max = 60;kp=1.22 * 1.99999999e-06;v_gain=2.2;break;
		case 65:	
			Power_Max = 65;kp=1.20 * 1.99999999e-06;v_gain=2.5;break;
		case 70:	
			Power_Max = 70;kp=1.18 * 1.99999999e-06;v_gain=2.5;break;
		case 75:	
			Power_Max = 75;kp=1.18 * 1.99999999e-06;v_gain=2.7;break;
		case 80:
			Power_Max = 80;kp=1.18 * 1.99999999e-06;v_gain=3.0;break;
		case 85:	
			Power_Max = 85;kp=1.18 * 1.99999999e-06;v_gain=3.0;break;
		case 90:	
			Power_Max = 90;kp=1.16 * 1.99999999e-06;v_gain=2.8;break;
		case 95:	
			Power_Max = 95;kp=1.16 * 1.99999999e-06;v_gain=2.8;break;
		case 100:	
			Power_Max = 100;kp=1.16 * 1.99999999e-06;v_gain=2.9;break;
		case 120:
			Power_Max = 120;v_gain=3.4;break;
		default:
			Power_Max = 45;v_gain=3;break;
		
	}
}


extern uint16_t cms_offline_counter;
void CMS__()
{
	if(CMS_Data.cms_cap_v < 12){
		CMS_Data.charge_flag=1;
	}
	else if(CMS_Data.cms_cap_v > 18){
		CMS_Data.charge_flag=0;
	}
	if((Chassis.CapKey) && CMS_Data.cms_cap_v > 12 && CMS_Data.charge_flag==0)
	{
		CMS_Data.TxOpen = 1;
	}
	else CMS_Data.TxOpen =0;	
	if(power_heat_data_t.buffer_energy < 30 || cms_offline_counter > 200) //cms用不了
	{
		CMS_Data.TxOpen = 0;	
	}
	cms_offline_counter ++;
}
uint8_t chassis_limit_update_flag=0;
void chassis_limit_update(void)
{
	if(Referee.chassis_power_limit!=Power_Max)
	{
		chassis_limit_update_flag=1;
	}
	else{
		chassis_limit_update_flag=0;
	}
}

float Plimit = 0;
uint8_t chassis_powerloop(Chassis_t *Chassis)
{

	// 电池供电
	// 计算CMS充电功率

	// 数学模型预测功率（不使用CMS功率计的功率计算）
	he = fabs(2 * Chassis->Motor3508[0].speed_rpm - last_speed[0]) * fabs((float)Chassis->Current[4]) * kp +
		 fabs(2 * Chassis->Motor3508[1].speed_rpm - last_speed[1]) * fabs((float)Chassis->Current[5]) * kp +
		 fabs(2 * Chassis->Motor3508[2].speed_rpm - last_speed[2]) * fabs((float)Chassis->Current[6]) * kp +
		 fabs(2 * Chassis->Motor3508[3].speed_rpm - last_speed[3]) * fabs((float)Chassis->Current[7]) * kp+
		 fabs(2 * Chassis->Motor6020[0].speed_rpm - last_speed[4]) * fabs((float)Chassis->Current[0]) * kp+
		 fabs(2 * Chassis->Motor6020[1].speed_rpm - last_speed[5]) * fabs((float)Chassis->Current[1]) * kp+
		 fabs(2 * Chassis->Motor6020[2].speed_rpm - last_speed[6]) * fabs((float)Chassis->Current[2]) * kp+
		 fabs(2 * Chassis->Motor6020[3].speed_rpm - last_speed[7]) * fabs((float)Chassis->Current[3]) * kp;

	last_speed[0] = Chassis->Motor3508[0].speed_rpm;
	last_speed[1] = Chassis->Motor3508[0].speed_rpm;
	last_speed[2] = Chassis->Motor3508[0].speed_rpm;
	last_speed[3] = Chassis->Motor3508[0].speed_rpm;

	lijupower = he + START_POWER;

	if (CMS_Data.cms_cap_v <= 15 || power_heat_data_t.buffer_energy <= 30 || cms_offline_counter > 500)
	{
		power_flag = 0;
	}

	if (CMS_Data.TxOpen==1)
	{
		Power_Max += 30;
	}
    if(power_flag == 0){
		if (power_heat_data_t.buffer_energy > 58)
			
		{
			Plimit = 0.01;
		}
		if (power_heat_data_t.buffer_energy < 40 && power_heat_data_t.buffer_energy >= 35)
			
		{
			Plimit = 0.45;
		}
		else if (power_heat_data_t.buffer_energy < 35 && power_heat_data_t.buffer_energy >= 30)
		{
			Plimit = 0.4;
			//power_scale = (Power_Max-2) / lijupower;
		}
		else if (power_heat_data_t.buffer_energy < 30 && power_heat_data_t.buffer_energy >= 20)
		{
			Plimit = 0.2;
			//power_scale = (Power_Max-2) / lijupower;
			
		}
		else if (power_heat_data_t.buffer_energy < 20 && power_heat_data_t.buffer_energy >= 10)
		{
			Plimit = 0.1;
			//power_scale = (Power_Max-2) / lijupower;
		}
		else if (power_heat_data_t.buffer_energy < 10 && power_heat_data_t.buffer_energy >= 0)
		{
			Plimit = 0.05;
			//power_scale = (Power_Max-2) / lijupower;}
		}
		else
		{
			Plimit = 1;
			//power_scale = 1;
		}
	}
	if (lijupower > Power_Max && power_flag == 0)
	{
		power_scale = (Power_Max-2) / lijupower;
		Chassis->Current[0] *= (power_scale) * (Plimit);
		Chassis->Current[1] *= (power_scale) * (Plimit);
		Chassis->Current[2] *= (power_scale) * (Plimit);
		Chassis->Current[3] *= (power_scale) * (Plimit);
		Chassis->Current[4] *= (power_scale) * (Plimit);
		Chassis->Current[5] *= (power_scale) * (Plimit);
		Chassis->Current[6] *= (power_scale) * (Plimit);
		Chassis->Current[7] *= (power_scale) * (Plimit);

		CMS_Data.charge_limit = 0.0f;
	}
	else
	{
		CMS_Data.charge_limit = Power_Max - lijupower;
	}

	//			if(CMS_charge_power < 0.0f)
	//			{
	//			CMS_charge_power = 0.0f;
	//			}

	//		if(chassis_power_buffer < chassis_buffer_limit)

	//	if(Chassis->power_mode == BATTERY)
	//	{
	//		if(lijupower > powermax)
	//		{
	//			power_scale = powermax/lijupower;
	//
	//			Chassis->Output.LF *= (power_scale);
	//			Chassis->Output.LB *= (power_scale);
	//			Chassis->Output.RF *= (power_scale);
	//			Chassis->Output.RB *= (power_scale);
	//
	//			CMS_charge_power = 0.0f;
	//		}else
	//		{
	//			CMS_charge_power = powermax - lijupower;
	////			CMS_charge_power = 0 ;
	//		}
	//			if(CMS_charge_power < 0.0f)
	//			{
	//			CMS_charge_power = 0.0f;
	//			}
	//	}
	//
	////电容供电
	//	if(Chassis->power_mode == CAPACITY)
	//	{
	//		CMS_charge_power = powermax;
	//		if(CMS_charge_power < 0.0f)
	//		{
	//			CMS_charge_power = 0.0f;
	//		}
	//	}

	return 0;
}

