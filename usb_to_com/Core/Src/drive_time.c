/*
 * drive_time.c
 *
 *  Created on: Mar 2, 2025
 *      Author: Hi
 */
#include"drive_time.h"
#include "drive.h"
#include"dvr_gpio.h"
#include "mgr_hmi.h"
Msg_Timer_Delay TID_Timer[TOTAL_TIMER_DELAY];
static volatile uint32_t _tGloabal_milis=0;

uint32_t millis(void)
{
	return _tGloabal_milis;
}


void TIM7_Interrupt(void)
{
	++_tGloabal_milis;
	MC_Control_Interrupt();
	if(_tGloabal_milis>=0x7FFFFFFFU)
	{
		_tGloabal_milis=0x00U;
		for(int i=0;i<TOTAL_TIMER_DELAY;i++)
		{
			TID_Timer[i].Time_Cur=0x00U;
			if(TID_Timer[i].End_Time>0x7FFFFFFFU)
			{
				TID_Timer[i].End_Time=TID_Timer[i].End_Time-0x7FFFFFFFU;
			}
			else
			{
				TID_Timer[i].End_Time=0x00U;
			}
		}
	}
}
void reset_timer(void)
{

	for(int i=0;i<TOTAL_TIMER_DELAY;i++)
	{
		TID_Timer[i].active=0x00U;
		TID_Timer[i].Time_Delay=0x00U;
		TID_Timer[i].Time_Cur=0x00U;
		TID_Timer[i].End_Time=0x00U;
		_tGloabal_milis=0x00U;
	}

}
void reset_timer_one_channel(uint8_t id)
{
	TID_Timer[id].active=0x00U;
	TID_Timer[id].Time_Delay=0x00U;
	TID_Timer[id].Time_Cur=0x00U;
	TID_Timer[id].End_Time=0x00U;
}
void Delay_SetTimer(uint8_t id,uint32_t timer)
{
	TID_Timer[id].active=0x01U;
	TID_Timer[id].Time_Delay=timer;
	TID_Timer[id].Time_Cur=_tGloabal_milis;
	TID_Timer[id].End_Time=TID_Timer[id].Time_Cur+TID_Timer[id].Time_Delay;
}
uint8_t Delay_GetTimer(uint8_t id)
{
	TID_Timer[id].Time_Cur=_tGloabal_milis;
	if(TID_Timer[id].active==0x01U)
	{
		if(TID_Timer[id].Time_Cur>=TID_Timer[id].End_Time)
		{
			Delay_SetTimer(id,TID_Timer[id].Time_Delay);
			return 0x01U;
		}
	}
	return 0x00U;
}


