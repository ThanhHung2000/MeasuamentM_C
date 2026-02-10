/*
 * dvr_gpio.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Admin
 */

#ifndef INC_DVR_GPIO_H_
#define INC_DVR_GPIO_H_

#include "main.h"
#define NUMBER_GPIO_SAMPLING 0x02U
#define NUM_SENSORS          18U

typedef struct
{
	volatile uint8_t Sensor_State[NUM_SENSORS];// Mảng chứa trạng thái hiện tại (đã được chống nhiễu)
	volatile uint8_t Last_Sensor_Reading[NUM_SENSORS];	// Mảng chứa trạng thái đọc được lần trước
	volatile uint8_t Sample_Counter[NUM_SENSORS] ;	// Mảng chứa bộ đếm chống nhiễu cho từng cảm biến
}Input_state_Sesor;
uint8_t Get_State_Sensor(uint8_t channel);
void Gpio_input();
void Task_gpio_input();
void Task_gpio_output(void);
#endif /* INC_DVR_GPIO_H_ */
