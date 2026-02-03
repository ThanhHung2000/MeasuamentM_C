/*
 * RS232.c
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "RS232.h"
#include "modbusSlave.h"
#include "mgr_hmi.h"
#define RX_BUF_SIZE 256
uint8_t RxData[RX_BUF_SIZE];
uint8_t TxData[RX_BUF_SIZE];


Control_motor_t* Control_motor = (Control_motor_t*)&Coils_Database[1];
Tab_Control_t* Tab = (Tab_Control_t*)&Coils_Database[0];
Save_Tray_t* Save_Tray = (Save_Tray_t*)&Coils_Database[3];
Save_Tray_t* Save_Tray_Indicator = (Save_Tray_t*)&Coils_Database[3];
Motor_Lamp_t* Motor_Lamp = (Motor_Lamp_t*)&Inputs_Database[1];
Worker_Control_t* Worker_Control = (Worker_Control_t*)&Coils_Database[2];
Home_controller * Home_controller_hmi = (Home_controller *)&Coils_Database[5];

Tray2D * Point2D_Tray1 = (Tray2D *)&Holding_Registers_Database[12];

uint16_t* Cover_Select = &Holding_Registers_Database[16];
uint16_t* Lamp_Cover_select = &Input_Registers_Database[1];
uint8_t* Lamp_Cover_done = &Inputs_Database[26];


uint16_t* Glass_Select = &Holding_Registers_Database[15];
uint16_t* Lamp_glass_select = &Input_Registers_Database[0];
uint8_t* Lamp_glass_empty = &Inputs_Database[1];

uint16_t* Mark = &Holding_Registers_Database[3];
uint16_t Glass_Index = 0;
uint8_t end_Cover = 25;

void HMI_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
	Init_hmi();
}
uint8_t DecodeModbusRtu(const uint8_t *data, uint16_t length )
{
	uint8_t crc_low_byte = data[length - 2]; // Byte thấp được gửi trước
	uint8_t crc_high_byte = data[length - 1]; // Byte cao được gửi sau
	// Tái tạo giá trị CRC-16 (16-bit)
	uint16_t received_crc = (uint16_t) (crc_high_byte << 8) | (uint16_t)crc_low_byte;
	if(received_crc==crc16((uint8_t *)data,length-2))
	{
		if(data[0]==SLAVE_ID)
		{
			return 0x01;
		}
	}
	return 0x00;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart-> Instance == USART2){
		if (DecodeModbusRtu(RxData, Size))
				{
					switch (RxData[1]){
					case 0x03:
						readHoldingRegs();
						break;
					case 0x04:
						readInputRegs();
						break;
					case 0x01:
						readCoils();
						break;
					case 0x02:
						readInputs();
						break;
					case 0x06:
						writeSingleReg();
						break;
					case 0x10:
						writeHoldingRegs();
						break;
					case 0x05:
						writeSingleCoil();
						break;
					case 0x0F:
						writeMultiCoils();
						break;
					default:
						modbusException(ILLEGAL_FUNCTION);
						break;
					}
				}
			//uart2_receive_IDLE_DMA();
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
	}
}

