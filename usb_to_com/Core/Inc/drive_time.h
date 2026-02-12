/*
 * drive_time.h
 *
 *  Created on: Mar 2, 2025
 *      Author: Hi
 */

#ifndef DRIVE_TIME_H_
#define DRIVE_TIME_H_
#include"main.h"


#define CY_SCB_SPI_ETH_H

#define TID_TIMER_1ms        0
#define TID_TIMER_2ms        1
#define TID_TIMER_500ms      2
#define TID_TIMER_1000ms     3
#define TID_TIMER_3000ms     4
#define TID_TIMER_UPDT       5
#define TOTAL_TIMER_DELAY    6

#define TIMER_UPDATE 0
#define TOTAL_TIMER 3
typedef struct
{
	uint32_t Time_Cur;
	uint8_t active;
	uint32_t Time_Delay;
	uint32_t End_Time;

}Msg_Timer_Delay;

void Delay_SetTimer(uint8_t id,uint32_t timer);
void reset_timer_one_channel(uint8_t id);
uint8_t Delay_GetTimer(uint8_t id);
void reset_timer(void);
uint8_t check_timedelay(uint8_t id, uint32_t timedelay);
void reset_delaytimer(uint8_t id);

void TIM7_Interrupt(void);

#endif /* DRIVE_TIME_H_ */
