/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Fs_remote.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"



//ң����������������
#define RC_CHANNAL_ERROR_VALUE 100

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

FS_ctrl_t fs_ctrl;

//ȡ������
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
void ibus_to_rc(uint8_t DmaBufNmb);

//remote control data 
//ң�������Ʊ���

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t ibus_rx_buf[2][IBUS_RX_BUF_NUM];


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void fs_remote_control_init(void)
{
    usart6_init(ibus_rx_buf[0], ibus_rx_buf[1], IBUS_RX_BUF_NUM);
	usart1_tx_dma_init();
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const FS_ctrl_t *get_fs_remote_control_point(void)
{
    return &fs_ctrl;
}


void slove_FS_lost(void)
{
    usart6_rx_dma_restart(IBUS_RX_BUF_NUM);
}
void slove_FS_data_error(void)
{
    usart6_rx_dma_restart(IBUS_RX_BUF_NUM);
}


//ȡ������
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
void ibus_to_rc(uint8_t DmaBufNmb)
{
    uint16_t check_sum=0;
	if(ibus_rx_buf[DmaBufNmb][0] == 0x20 && ibus_rx_buf[DmaBufNmb][1] == 0x40){
		check_sum+=(ibus_rx_buf[DmaBufNmb][0]+ibus_rx_buf[DmaBufNmb][1]);
		for(uint8_t i=0;i<14;i++)
		{
			fs_ctrl.ch[i]= (ibus_rx_buf[DmaBufNmb][2*i+3]<<8) | ibus_rx_buf[DmaBufNmb][2*i+2];
			check_sum+=(ibus_rx_buf[DmaBufNmb][2*i+3]+ibus_rx_buf[DmaBufNmb][2*i+2]);
		}
	}
	
	
}    








fp32 NormalizedLimit_0(fp32 input) {
    if (input > 1.0f) {
        input = 1.0f;
    }
    else if (input < -1.0f) {
        input = -1.0f;
    }
    return input;
}

