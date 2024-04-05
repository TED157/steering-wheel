#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include "struct_typedef.h"

typedef struct
{
    // Motor
    uint32_t Motor[8];
    
    // CAN Bus Node
    uint32_t PTZnode;
    
} OfflineCounter_t;

typedef struct
{
    // Motor
    uint8_t Motor[8];
	//Can bus noode
    uint8_t PTZnode;
    
} OfflineMonitor_t;



void TimerTaskLoop1000Hz();
void TimerTaskLoop1000Hz();
void TimerTaskLoop500Hz_1();
void TimerTaskLoop500Hz_2();
void TimerTaskLoop100Hz();

extern void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor);
extern uint8_t left_flag,right_flag;
extern double t00,tk[4],t01,t02,ct,ct2,t03;
extern int receive_times,send_times;
#endif