/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef FS_REMOTE_CONTROL_H
#define FS_REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "main.h"
#define IBUS_RX_BUF_NUM 36u

#define FS_FRAME_LENGTH 32u

typedef __PACKED_STRUCT
{
	uint16_t ch[14];
	int16_t  user_ch[10];
}FS_ctrl_t;
void ibus_to_rc(uint8_t DmaBufNmb);
extern void fs_remote_control_init(void);
extern const FS_ctrl_t *get_fs_remote_control_point(void);
#endif
