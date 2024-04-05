#include "CMS.h"
#include "CanPacket.h"
#include "setting.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern PTZ_t PTZ;

//CMS_t CMS;
CMS_Data_t CMS_Data;

static CAN_TxHeaderTypeDef  cms_buffer_tx_message;		
static uint8_t              cms_buffer_can_send_data[2];
static CAN_TxHeaderTypeDef  cms_power_tx_message;
static uint8_t              cms_power_can_send_data[8];
 //ָ���


//void CMS_Referee_Send(uint16_t charge_limit, uint8_t enable)
//{
//        uint32_t send_mail_box;
//    can_tx_message.StdId = CMSCurrentSendID;
//    can_tx_message.IDE = CAN_ID_STD;
//    can_tx_message.RTR = CAN_RTR_DATA;
//    can_tx_message.DLC = 0x03;
//    can_send_data[0] = charge_limit >> 8;
//    can_send_data[1] = charge_limit;
//    can_send_data[2] = enable;
//    HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
//}


//send buffer power
void CMS_BUFFER_SEND(int16_t buffer)
{
    uint32_t send_mail_box;
    cms_buffer_tx_message.StdId = CMSBufferPowerSendID;
    cms_buffer_tx_message.IDE = CAN_ID_STD;
    cms_buffer_tx_message.RTR = CAN_RTR_DATA;
    cms_buffer_tx_message.DLC = 0x02;
    cms_buffer_can_send_data[0] = (buffer >> 8);
    cms_buffer_can_send_data[1] = buffer;
    HAL_CAN_AddTxMessage(&hcan1, &cms_buffer_tx_message, cms_buffer_can_send_data, &send_mail_box);
}

//send data
void CMS_POWER_SEND(int16_t input_power_limit,int16_t output_powe_limit,int16_t cap_power_limit,int16_t cap_control)
{
    uint32_t send_mail_box;
    cms_power_tx_message.StdId = CMSDateSendID;
    cms_power_tx_message.IDE = CAN_ID_STD;
    cms_power_tx_message.RTR = CAN_RTR_DATA;
    cms_power_tx_message.DLC = 0x08;
    cms_power_can_send_data[0] = (input_power_limit >> 8);
    cms_power_can_send_data[1] = input_power_limit;
	cms_power_can_send_data[2] = (output_powe_limit >> 8);
    cms_power_can_send_data[3] = output_powe_limit;
	cms_power_can_send_data[4] = (cap_power_limit >> 8);
    cms_power_can_send_data[5] = cap_power_limit;
	cms_power_can_send_data[6] = (cap_control >> 8);
    cms_power_can_send_data[7] = cap_control;
    HAL_CAN_AddTxMessage(&hcan1, &cms_power_tx_message, cms_power_can_send_data, &send_mail_box);
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
	int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;
	//��0.5ʹ����ȡ�������������
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}