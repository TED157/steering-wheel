#include "cmsis_os.h"
#include "feet_motor.h"
#include "bsp_can.h"
#include "CalculateThread.h"
#include "InterruptService.h"
#include "Setting.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t             send_mail_box_1;
static uint32_t             send_mail_box_2;
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              MotorSendBuffer[16];


void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);

void Chassis_Control(int16_t left_front_6020, int16_t right_front_6020,int16_t right_back_6020, int16_t left_back_6020,
					int16_t left_front_3508, int16_t right_front_3508,int16_t right_back_3508, int16_t left_back_3508)
{
    
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
	MotorSendBuffer[(LEFT_FRONT_6020 - 0x201)*2 ]               =   left_front_6020 >> 8;
    MotorSendBuffer[(LEFT_FRONT_6020 - 0x201)*2+ 1  ]        	=   left_front_6020;
    MotorSendBuffer[(RIGHT_FRONT_6020 - 0x201)*2  ]            	=   right_front_6020 >> 8;
    MotorSendBuffer[(RIGHT_FRONT_6020 - 0x201)*2+ 1 ]        		=   right_front_6020;
	MotorSendBuffer[(RIGHT_BACK_6020 - 0x201)*2]            	=   right_back_6020 >> 8;
	MotorSendBuffer[(RIGHT_BACK_6020 - 0x201)*2 + 1]          =   right_back_6020 ;
	MotorSendBuffer[(LEFT_BACK_6020 - 0x201)*2 ]            	=   left_back_6020 >> 8;
	MotorSendBuffer[(LEFT_BACK_6020 - 0x201)*2 +1 ]           =   left_back_6020;
	MotorSendBuffer[(LEFT_FRONT_3508 - 0x201)*2 ]               =   left_front_3508 >> 8;
    MotorSendBuffer[(LEFT_FRONT_3508 - 0x201)*2+ 1  ]        	=   left_front_3508;
    MotorSendBuffer[(RIGHT_FRONT_3508 - 0x201)*2  ]            	=   right_front_3508 >> 8;
    MotorSendBuffer[(RIGHT_FRONT_3508 - 0x201)*2+ 1 ]        		=   right_front_3508;
	MotorSendBuffer[(RIGHT_BACK_3508 - 0x201)*2]            	=   right_back_3508 >> 8;
	MotorSendBuffer[(RIGHT_BACK_3508 - 0x201)*2 + 1]          =   right_back_3508 ;
	MotorSendBuffer[(LEFT_BACK_3508 - 0x201)*2 ]            	=   left_back_3508 >> 8;
	MotorSendBuffer[(LEFT_BACK_3508 - 0x201)*2 +1 ]           =   left_back_3508;			
	can_tx_message.StdId = 0x200;
    HAL_CAN_AddTxMessage(&CONTROL_CANPORT, &can_tx_message, MotorSendBuffer, &send_mail_box_1);
	osDelay(1);
	can_tx_message.StdId = 0x1FF;
    HAL_CAN_AddTxMessage(&CONTROL_CANPORT, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box_2);

	
}


#define LEFT_FRONT_6020         			    0x205
#define RIGHT_FRONT_6020         			    0x206
#define RIGHT_BACK_6020                         0x207
#define LEFT_BACK_6020                          0x208
#define LEFT_FRONT_3508         			    0x201
#define RIGHT_FRONT_3508         			    0x202
#define RIGHT_BACK_3508                         0x203
#define LEFT_BACK_3508                          0x204 

motor_measure_t LEFT_FRONT_6020_Measure;
motor_measure_t RIGHT_FRONT_6020_Measure;
motor_measure_t RIGHT_BACK_6020_Measure;
motor_measure_t LEFT_BACK_6020_Measure;
motor_measure_t LEFT_FRONT_3508_Measure;
motor_measure_t RIGHT_FRONT_3508_Measure;
motor_measure_t RIGHT_BACK_3508_Measure;
motor_measure_t LEFT_BACK_3508_Measure;
motor_measure_t YawMotorMeasure;

//motor data read

void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message)
{
    switch (MotorID){
//		case YawMotorId:
//			get_motor_measure(&YawMotorMeasure,message);
//			break; 
        case LEFT_FRONT_6020:
            get_motor_measure(&LEFT_FRONT_6020_Measure, message);
            break;
        case RIGHT_FRONT_6020:
            get_motor_measure(&RIGHT_FRONT_6020_Measure, message);
            break;
		case RIGHT_BACK_6020:
            get_motor_measure(&RIGHT_BACK_6020_Measure, message);
            break;
        case LEFT_BACK_6020:
            get_motor_measure(&LEFT_BACK_6020_Measure, message);
            break;
		case LEFT_FRONT_3508:
            get_motor_measure(&LEFT_FRONT_3508_Measure, message);
            break;
        case RIGHT_FRONT_3508:
            get_motor_measure(&RIGHT_FRONT_3508_Measure, message);
            break;
		case RIGHT_BACK_3508:
            get_motor_measure(&RIGHT_BACK_3508_Measure, message);
            break;
        case LEFT_BACK_3508:
            get_motor_measure(&LEFT_BACK_3508_Measure, message);
            break;
        default:
            break;
    }
}













