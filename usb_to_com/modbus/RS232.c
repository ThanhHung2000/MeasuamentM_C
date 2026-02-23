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

uint8_t MainBuf[RX_BUF_SIZE]; // Mảng tạm để xử lý logic

uint8_t TxData[RX_BUF_SIZE];

volatile uint16_t hoding_new=0x00U;



void HMI_Init(void)
{
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, MainBuf, RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // Khóa lần 1 (Khởi tạo)
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
		// 1. Copy cực nhanh sang mảng tạm (chỉ copy đúng số byte đã nhận)
		memcpy(RxData, MainBuf, Size);// copy RxData = MainBuf
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, MainBuf, RX_BUF_SIZE);
		// PHẢI CÓ DÒNG NÀY Ở ĐÂY: Vì hàm trên vừa mới tự động bật HT lại
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // Khóa lại sau mỗi lần nhận
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
						hoding_new=0x01U;
						break;
					case 0x10:
						writeHoldingRegs();
						hoding_new=0x01U;
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
	}
}

