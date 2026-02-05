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

Tab_Control_t* Main_controler = (Tab_Control_t*)&Coils_Database[0];
Control_motor_t* Control_motor = (Control_motor_t*)&Coils_Database[1];
Tray2D * Point2D_Tray1 = (Tray2D *)&Holding_Registers_Database[12];

Axis_Config_t *Axis_Config = (Axis_Config_t *)&Holding_Registers_Database[0];


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

