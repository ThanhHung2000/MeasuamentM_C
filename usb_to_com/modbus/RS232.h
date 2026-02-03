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

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

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

        uint8_t Left	: 1;
        uint8_t Right	: 1;
        uint8_t In		: 1;
        uint8_t Out		: 1;
        uint8_t Up		: 1;
        uint8_t Down	: 1;
        uint8_t Set		: 1;
        uint8_t GoHome	: 1;
    } bits;
    uint8_t all;
} Control_motor_t;
typedef union {
    struct {

    	uint16_t glass_1		: 1;
    	uint16_t glass_2		: 1;
    	uint16_t glass_3		: 1;
    	uint16_t Cover_1		: 1;
    	uint16_t Cover_2		: 1;
    	uint16_t Cover_3		: 1;
    	uint16_t save_glass	: 1;
    	uint16_t Save_Cover	: 1;
    	uint16_t Save_Tray	: 1;
    } bits;
    uint16_t all;
}Save_Tray_t;
typedef union {
    struct {
    	uint8_t Home :1;
    	uint8_t Motor :1;
        uint8_t reserved : 6;

    } bits;
    uint8_t all;
} Tab_Control_t;




typedef union {
    struct {
    	uint16_t ruber1_1 		:1;
    	uint16_t ruber1_2 		:1;
    	uint16_t ruber1_3	    :1;
    	uint16_t tray1_1		:1;
    	uint16_t tray1_2		:1;
    	uint16_t tray1_3    	:1;
    	uint16_t tray2_1		:1;
    	uint16_t tray2_2		:1;
    	uint16_t tray2_3		:1;
    	uint16_t led_move		:1;


    } bits;
    uint16_t all;
} Motor_Lamp_t;

typedef union
{
    struct {
    	uint8_t reset 		:1;
    	uint8_t start 		:1;
    	uint8_t stop	    :1;
    	uint8_t setX		:1;
    	uint8_t setY		:1;
    	uint8_t setZ    	:1;
    	uint8_t Manual_auto :1;
    	uint8_t ignor		:1;
    } bits;
    uint8_t all;
}Home_controller;

typedef union {
    struct {
    	uint8_t glass1 		:1;
    	uint8_t glass2 		:1;
    	uint8_t glass3 		:1;
		uint8_t glass4 		:1;
		uint8_t Run 		:1;
		uint8_t ScanGlass 	:1;
		uint8_t Reset 		:1;
        uint8_t reserved 	:1;
    } bits;
    uint8_t all;
} Worker_Control_t;
extern uint16_t Glass_Index;
extern uint8_t end_Cover;

extern uint8_t* Lamp_Cover_done;
extern uint16_t* Lamp_Cover_select;
extern uint16_t* Cover_Select;

extern uint16_t* Glass_Select;
extern uint8_t* Lamp_glass_empty;
extern uint16_t* Lamp_glass_select;
extern Worker_Control_t* Worker_Control;
extern Motor_Lamp_t* Motor_Lamp;
extern Control_motor_t* Control_motor;
extern Tab_Control_t* Tab;
extern Home_controller * Home_controller_hmi;
extern Save_Tray_t* Save_Tray;
extern Save_Tray_t* Save_Tray_Indicator;
extern uint16_t* Mark;
extern Tray2D * Point2D_Tray1;
void HMI_Init(void);

//void uart2_receive_IDLE_DMA();
#endif /* INC_RS232_H_ */
