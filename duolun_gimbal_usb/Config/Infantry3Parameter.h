#ifndef __INFANTRY3_PARAMETER_H
#define __INFANTRY3_PARAMETER_H

#include "struct_typedef.h"

#define GM6020_MAX_OUTPUT                       30000
#define GM6020_MAX_IOUTPUT                      8000
#define M3508_MAX_OUTPUT                        16384
#define M3508_MAX_IOUTPUT                       6000
#define M2006_MAX_OUTPUT                        10000
#define M2006_MAX_IOUTPUT                       50007



#define PITCH_MAX_SPEED                         1000
#define PITCH_MAX_ISPEED                        6
#define YAW_MAX_SPEED                           1000
#define YAW_MAX_ISPEED                          8

#define AMMO_SPEEDSET_10MS                      2000
#define AMMO_SPEEDSET_12MS                      5000
#define AMMO_SPEEDSET_14MS                      4500
#define AMMO_SPEEDSET_15MS_LOW									4450	
#define AMMO_SPEEDSET_15MS                      5000//4150//30ms7800//18ms5100//15ms4570//4215
#define AMMO_SPEEDSET_16MS                      5100
#define AMMO_SPEEDSET_18MS                      5150
#define AMMO_SPEEDSET_22MS                      6200
#define AMMO_SPEEDSET_30MS                      7300//8800
#define AMMO_SPEEDSET_30MS_L                    7050//7400//7800
#define AMMO_SPEEDSET_30MS_R                    7050

#define ROTOR_SPEEDSET_FORWARD                  4670.0f//5000.0f//5000
#define ROTOR_SPEEDSET_FORWARD_FAST             7000.0f
#define ROTOR_SPEEDSET_BACKWARD                 -2000.0f
#define ROTOR_SPEEDSET_FORWARD_RUNE             4600.0f//5000.0f//5000

#define HIGH_FREQ_COOLING_COEFFICIENT			360//440
#define LOW_FREQ_COOLING_COEFFICIENT			200//4-  40
#define ROTOR_TIMESET_BUSY                      59//50
#define ROTOR_TIMESET_BUSY_FAST                 1
#define ROTOR_TIMESET_COOLING                   0
#define ROTOR_TIMESET_RESERVE                   80 //80 30
#define ROTOR_LAGGING_COUNTER_MAX               50//28
#define ROTOR_TIMESET_COOLING_SECOND            0

#define ROTOR_TIMESET_COOLING_MAXEXTRA          60

#define DELTA_HEAT_MAX							80

#define ROTOR_UNABLE_KP                         0.0f
#define ROTOR_UNABLE_KI                         0.0f
#define ROTOR_UNABLE_KD                         0.0f
fp32 ROTOR_UNABLE[3] = { ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD };

#define ROTOR_FORWARD_KP                       	130.0f//150f
#define ROTOR_FORWARD_KI                        0.0f
#define ROTOR_FORWARD_KD                        15.0f
fp32 ROTOR_FORWARD[3] = { ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD };

#define ROTOR_STOP_KP                           10.0f//30.0f
#define ROTOR_STOP_KI                           0.0f
#define ROTOR_STOP_KD                           3.0f
fp32 ROTOR_STOP[3] = { ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD };

#define ROTOR_BACK_KP                           100.0f
#define ROTOR_BACK_KI                           0.0f
#define ROTOR_BACK_KD                           0.0f
fp32 ROTOR_BACK[3] = { ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD };




//  无力云台参数
//  YAW轴角速度环
#define YAW_SPEED_NO_FORCE_KP                   0.0f
#define YAW_SPEED_NO_FORCE_KI                   0.0f
#define YAW_SPEED_NO_FORCE_KD                   0.0f
fp32 YAW_SPEED_NO_FORCE[3] = { YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD };
//  YAW轴角度环
#define YAW_ANGLE_NO_FORCE_KP                   0.0f
#define YAW_ANGLE_NO_FORCE_KI                   0.0f
#define YAW_ANGLE_NO_FORCE_KD                   0.0f
fp32 YAW_ANGLE_NO_FORCE[3] = { YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD };
//  PITCH轴角速度环
#define PITCH_SPEED_NO_FORCE_KP                 0.0f
#define PITCH_SPEED_NO_FORCE_KI                 0.0f
#define PITCH_SPEED_NO_FORCE_KD                 0.0f
fp32 PITCH_SPEED_NO_FORCE[3] = { PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD };
//  PITCH轴角度环
#define PITCH_ANGLE_NO_FORCE_KP                 0.0f
#define PITCH_ANGLE_NO_FORCE_KI                 0.0f
#define PITCH_ANGLE_NO_FORCE_KD                 0.0f
fp32 PITCH_ANGLE_NO_FORCE[3] = { PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD };


//  归位云台参数
//  YAW轴角速度环
#define YAW_SPEED_RESET_POSITION_KP             160.0f//36.0f
#define YAW_SPEED_RESET_POSITION_KI             0.0f
#define YAW_SPEED_RESET_POSITION_KD             0.0f
fp32 YAW_SPEED_RESET_POSITION[3] = { YAW_SPEED_RESET_POSITION_KP, YAW_SPEED_RESET_POSITION_KI, YAW_SPEED_RESET_POSITION_KD };
//  YAW轴角度环
#define YAW_ANGLE_RESET_POSITION_KP             28.0f//18.0f
#define YAW_ANGLE_RESET_POSITION_KI             0.0f
#define YAW_ANGLE_RESET_POSITION_KD             0.0f
fp32 YAW_ANGLE_RESET_POSITION[3] = { YAW_ANGLE_RESET_POSITION_KP, YAW_ANGLE_RESET_POSITION_KI, YAW_ANGLE_RESET_POSITION_KD };
//  PITCH轴角速度环
#define PITCH_SPEED_RESET_POSITION_KP           80.0f
#define PITCH_SPEED_RESET_POSITION_KI           0.0f
#define PITCH_SPEED_RESET_POSITION_KD           0.0f
fp32 PITCH_SPEED_RESET_POSITION[3] = { PITCH_SPEED_RESET_POSITION_KP, PITCH_SPEED_RESET_POSITION_KI, PITCH_SPEED_RESET_POSITION_KD };
//  PITCH轴角度环
#define PITCH_ANGLE_RESET_POSITION_KP           16.0f
#define PITCH_ANGLE_RESET_POSITION_KI           0.5f
#define PITCH_ANGLE_RESET_POSITION_KD           0.0f
fp32 PITCH_ANGLE_RESET_POSITION[3] = { PITCH_ANGLE_RESET_POSITION_KP, PITCH_ANGLE_RESET_POSITION_KI, PITCH_ANGLE_RESET_POSITION_KD };


//  手动控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP             360.0f//360.0f
#define YAW_SPEED_MANUAL_OPERATE_KI             0.0f//0.1f,wyf:0.01
#define YAW_SPEED_MANUAL_OPERATE_KD             0.0f
fp32 YAW_SPEED_MANUAL_OPERATE[3] = { YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD };
//  YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP             40.0f//39.1f
#define YAW_ANGLE_MANUAL_OPERATE_KI             0.0001f//0.015f,wyf:0.02
#define YAW_ANGLE_MANUAL_OPERATE_KD             40.0f//22.0f
fp32 YAW_ANGLE_MANUAL_OPERATE[3] = { YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD };
//  PITCH轴角速度环
#define PITCH_SPEED_MANUAL_OPERATE_KP           160.0f//240.0f
#define PITCH_SPEED_MANUAL_OPERATE_KI           0.05f
#define PITCH_SPEED_MANUAL_OPERATE_KD           0.0f//58
fp32 PITCH_SPEED_MANUAL_OPERATE[3] = { PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD };
//  PITCH轴角度环
#define PITCH_ANGLE_MANUAL_OPERATE_KP           60.0f//40.0f
#define PITCH_ANGLE_MANUAL_OPERATE_KI           0.1f//0.045f
#define PITCH_ANGLE_MANUAL_OPERATE_KD           15.0f//20.0f
fp32 PITCH_ANGLE_MANUAL_OPERATE[3] = { PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD };



//  自瞄控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_OPERATE_KP             340.0f//360
#define YAW_SPEED_AIMBOT_OPERATE_KI             0.00f//0.1f
#define YAW_SPEED_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_SPEED_AIMBOT_OPERATE[3] = { YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD };
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_OPERATE_KP             18.0f//25.0f//36.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KI             0.00f//0.0f// 0.015f
#define YAW_ANGLE_AIMBOT_OPERATE_KD             100.0f//2.0f//25.0f
fp32 YAW_ANGLE_AIMBOT_OPERATE[3] = { YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD };
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_OPERATE_KP           170.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KI           0.00f
#define PITCH_SPEED_AIMBOT_OPERATE_KD           55.5f
fp32 PITCH_SPEED_AIMBOT_OPERATE[3] = { PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD };
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_OPERATE_KP           20.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KI           0.00f
#define PITCH_ANGLE_AIMBOT_OPERATE_KD           70.5f
fp32 PITCH_ANGLE_AIMBOT_OPERATE[3] = { PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD };


//  打符云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_RUNES_KP               350.0f
#define YAW_SPEED_AIMBOT_RUNES_KI               0.000f//0.1f
#define YAW_SPEED_AIMBOT_RUNES_KD               20.0f
fp32 YAW_SPEED_AIMBOT_RUNES[3] = { YAW_SPEED_AIMBOT_RUNES_KP, YAW_SPEED_AIMBOT_RUNES_KI, YAW_SPEED_AIMBOT_RUNES_KD };
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_RUNES_KP               42.0f//24.0f//36.0f
#define YAW_ANGLE_AIMBOT_RUNES_KI               0.0015f//0.0f// 0.015f
#define YAW_ANGLE_AIMBOT_RUNES_KD               90.0f//2.0f//25.0f
fp32 YAW_ANGLE_AIMBOT_RUNES[3] = { YAW_ANGLE_AIMBOT_RUNES_KP, YAW_ANGLE_AIMBOT_RUNES_KI, YAW_ANGLE_AIMBOT_RUNES_KD };
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_RUNES_KP             260.0f
#define PITCH_SPEED_AIMBOT_RUNES_KI             0.03f
#define PITCH_SPEED_AIMBOT_RUNES_KD             10.0f
fp32 PITCH_SPEED_AIMBOT_RUNES[3] = { PITCH_SPEED_AIMBOT_RUNES_KP, PITCH_SPEED_AIMBOT_RUNES_KI, PITCH_SPEED_AIMBOT_RUNES_KD };
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_RUNES_KP             50.0f
#define PITCH_ANGLE_AIMBOT_RUNES_KI             0.01f
#define PITCH_ANGLE_AIMBOT_RUNES_KD             61.0f
fp32 PITCH_ANGLE_AIMBOT_RUNES[3] = { PITCH_ANGLE_AIMBOT_RUNES_KP, PITCH_ANGLE_AIMBOT_RUNES_KI, PITCH_ANGLE_AIMBOT_RUNES_KD };


//  摩擦轮参数

#define AMMO_LEFT_SPEED_30MS_KP                 5.8f
#define AMMO_LEFT_SPEED_30MS_KI                 0.0f
#define AMMO_LEFT_SPEED_30MS_KD                 5.2f
fp32 AMMO_LEFT_SPEED_30MS[3] = { AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD };

#define AMMO_RIGHT_SPEED_30MS_KP                5.8f
#define AMMO_RIGHT_SPEED_30MS_KI                0.0f
#define AMMO_RIGHT_SPEED_30MS_KD                5.2f
fp32 AMMO_RIGHT_SPEED_30MS[3] = { AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD };



#endif
