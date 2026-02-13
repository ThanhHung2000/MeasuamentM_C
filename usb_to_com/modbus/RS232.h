/*
 * RS232.h
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "stm32f4xx_hal.h"
#include "modbusSlave.h"
#ifndef INC_RS232_H_
#define INC_RS232_H_


extern UART_HandleTypeDef huart2;


extern DMA_HandleTypeDef hdma_usart2_rx;

extern volatile uint16_t hoding_new;
typedef enum {
	EngineerSetting,
	WorkerRunning,

} MachineState;
typedef struct {
	uint16_t x1;
	uint16_t y1;
	uint16_t x2;
	uint16_t y2;
	uint16_t x3;
	uint16_t y3;
} Tray2D;
typedef struct {
	uint16_t x;
	uint16_t y;
} Point2D;
typedef union {
    struct {
        uint8_t Left	 : 1;
        uint8_t Right	 : 1;
        uint8_t In		 : 1;
        uint8_t Out		 : 1;
        uint8_t Up		 : 1;
        uint8_t Down	 : 1;
        uint8_t Reseved1 : 1;
        uint8_t Reseved2 : 1;
    } bits;
    uint8_t all;
} Control_motor_t;


typedef union {
    struct {
    	uint8_t Go_Home   :1;
    	uint8_t SetPoint  :1;
        uint8_t Emergency :1;
        uint8_t Restart   :1;
        uint8_t Stop      :1;
        uint8_t Reseved1  :3;
    } bits;
    uint8_t all;
} Tab_Control_t;

extern Control_motor_t* Control_motor;
extern Tab_Control_t* Main_controler;
extern uint16_t* Mark;
extern Tray2D * Point2D_Tray1;
void HMI_Init(void);

//void uart2_receive_IDLE_DMA();
#endif /* INC_RS232_H_ */
