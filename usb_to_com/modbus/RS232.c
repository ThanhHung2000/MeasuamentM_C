/*
 * RS232.c
 *
 *  Created on: Oct 27, 2025
 *      Author: MCNEX
 */
#include "RS232.h"
#include "modbusSlave.h"
#include "mgr_hmi.h"
#include <string.h>

uint8_t RxData[RX_BUF_SIZE];

uint8_t TxData[RX_BUF_SIZE];

volatile uint8_t is_new_frame = 0;
volatile uint16_t leng_size = 0U;
uint8_t ProcessBuf[RX_BUF_SIZE];

void HMI_Init(void)
{
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // Khóa lần 1 (Khởi tạo)
}
uint8_t DecodeModbusRtu(const uint8_t *data, uint16_t length )
{
//	uint8_t crc_low_byte = data[length - 2]; // Byte thấp được gửi trước
//	uint8_t crc_high_byte = data[length - 1]; // Byte cao được gửi sau
//	// Tái tạo giá trị CRC-16 (16-bit)
//	uint16_t received_crc = (uint16_t)(crc_high_byte << 8) | (uint16_t)crc_low_byte;
	uint16_t received_crc = (uint16_t)(data[length - 2]) | ((uint16_t)(data[length - 1]) << 8);
	if(received_crc==crc16((uint8_t *)data,length-2))
	{
		if(data[0]==SLAVE_ID)
		{
			return 0x01;
		}
	}
	return 0x00;
}
void Modbus_Rtu_Run(uint8_t *Rx_Uart, uint16_t Size)
{
	if (DecodeModbusRtu(Rx_Uart, Size))
			{
				switch (Rx_Uart[1]){
				case 0x03:
					readHoldingRegs(Rx_Uart);
					break;
				case 0x04:
					readInputRegs(Rx_Uart);
					break;
				case 0x01:
					readCoils(Rx_Uart);
					break;
				case 0x02:
					readInputs(Rx_Uart);
					break;
				case 0x06:
					writeSingleReg(Rx_Uart);
					break;
				case 0x10:
					writeHoldingRegs(Rx_Uart);
					break;
				case 0x05:
					writeSingleCoil(Rx_Uart);
					break;
				case 0x0F:
					writeMultiCoils(Rx_Uart);
					break;
				default:
					modbusException(Rx_Uart,ILLEGAL_FUNCTION);
					break;
				}
			}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart-> Instance == USART2)
	{
		memcpy(ProcessBuf, RxData, Size);//copy ra vùng đêm để xử lý
		active_port = PORT_UART; // Ghi nhận là đang dùng UART
#ifdef PROCES_IN_MAIN
		// 2. Đặt flag để main xử lý
		is_new_frame = 0x01U;
		leng_size=Size;
#else
		Modbus_Rtu_Run(ProcessBuf,Size);
#endif
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // Khóa lần 1 (Khởi tạo)
	}
}

