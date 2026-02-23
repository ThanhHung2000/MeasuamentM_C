/*
 * modbusSlave.c
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#include "modbusSlave.h"
#include "string.h"
#include "RS232.h"
#include "mgr_hmi.h"
#define LENG_MODBUS_DATA 0x0FU
extern uint8_t RxData[256];
extern uint8_t TxData[256];
extern UART_HandleTypeDef huart2;

// Giả sử khung tin Modbus tối đa là 256 byte
static uint8_t tx_buffer[RX_BUF_SIZE];

static volatile uint16_t Holding_Registers_Database[LENG_MODBUS_DATA]={0,};
static volatile uint8_t Coils_Database[LENG_MODBUS_DATA]={0,};
static volatile uint8_t Inputs_Database[LENG_MODBUS_DATA]  = {0,};
volatile uint16_t Input_Registers_Database[LENG_MODBUS_DATA]={0,};

Tab_Control_t* Main_controler = (Tab_Control_t*)&Coils_Database[0];
Control_motor_t* Control_motor = (Control_motor_t*)&Coils_Database[1];
Tray2D * Point2D_Tray1 = (Tray2D *)&Holding_Registers_Database[12];
void Update_Input_Register(uint8_t index, uint16_t toa_do,uint16_t toc_do, uint16_t state)
{
	if(Get_home_done()==0x00U)
	{
		Input_Registers_Database[index]=0x00U;
		Input_Registers_Database[index + 1]=0x00U;
		Input_Registers_Database[index + 2]=0x00U;
		return ;
	}
	Input_Registers_Database[index]=toa_do;
	Input_Registers_Database[index + 1]=toc_do;
	if(Input_Registers_Database[index + 2] != state)
	{
		Input_Registers_Database[index + 2]=state;
	}

}
void Set_Input_Register(uint8_t index, uint16_t data)
{
	Input_Registers_Database[index]=data;
}
uint8_t Get_Coild(uint8_t index)
{
	if(index>50) return 0x00U;
	return Coils_Database[index];
}
void Set_Inputs_Database(uint8_t index,uint8_t data)
{
	Inputs_Database[index]=data;
}
uint16_t Get_Holding_Registers(uint8_t index)
{
	if(index>50) return 0x00U;
	return Holding_Registers_Database[index];
}
void Reset_Oxis(void)
{
	Set_Input_Register(0x00U,0x00U);
	Set_Input_Register(3U,0x00U);
	Set_Input_Register(6U,0x00U);
}
void Copy_Holding_Registers(uint8_t index,uint8_t index_coppy)
{
	Holding_Registers_Database[index_coppy]=Holding_Registers_Database[index];
	Holding_Registers_Database[index_coppy+1]=Holding_Registers_Database[index+1];
}
void Set_bit_Coilt_lamp(uint8_t)
{

}
void Reset_Tray(uint8_t index)
{
	Coils_Database[index] = 0x00U;
}
void sendData (uint8_t *data, int size)
{
	// 1. Kiểm tra DMA có đang rảnh không
	if (huart2.gState != HAL_UART_STATE_READY) {
		return; // Hoặc thêm cơ chế retry/log lỗi nếu cần
	}
	if (size + 2 > RX_BUF_SIZE) return;

	// 2. Copy dữ liệu gốc vào buffer an toàn
	memcpy(tx_buffer, data, size);

	uint16_t crc = crc16(tx_buffer, size);
	tx_buffer[size]   = (uint8_t)(crc & 0xFF);   // CRC LOW
	tx_buffer[size+1] = (uint8_t)((crc>>8)&0xFF);  // CRC HIGH
//	HAL_UART_Transmit(&huart2, data, size+2, 1000);
	HAL_UART_Transmit_DMA(&huart2, tx_buffer, size + 2);
}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1]|0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	sendData(TxData, 3);         // send Data... CRC will be calculated in the function
}


uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>LENG_MODBUS_DATA)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>LENG_MODBUS_DATA)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Input_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Input_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readCoils (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coil Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if ((numCoils<1)||(numCoils>2000))  // maximum no. of coils as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numCoils-1;  // Last coils address
	if (endAddr>(LENG_MODBUS_DATA*10))  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}


	//reset TxData buffer
	memset (TxData, '\0', 256);

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils/8) + ((numCoils%8)>0 ? 1:0);  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr/8;  // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte
	int indxPosition = 0;  // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)
	{
		TxData[indx] |= ((Coils_Database[startByte] >> bitPosition) &0x01) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition>7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)  // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition=0;
			startByte++;
		}
	}

	if (numCoils%8 != 0)indx++;  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if ((numCoils<1)||(numCoils>2000))  // maximum no. of coils as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numCoils-1;  // Last coils address
	if (endAddr>(LENG_MODBUS_DATA*10))  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}


	//reset TxData buffer
	memset (TxData, '\0', 256);

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils/8) + ((numCoils%8)>0 ? 1:0);  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr/8;  // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte
	int indxPosition = 0;  // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)
	{
		TxData[indx] |= ((Inputs_Database[startByte] >> bitPosition) &0x01) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition>7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)  // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition=0;
			startByte++;
		}
	}

	if (numCoils%8 != 0)indx++;  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address
    uint8_t index = 7U;  // we need to keep track of index in RxData
	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>123))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>LENG_MODBUS_DATA)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* start saving 16 bit data
	 * Data starts from RxData[7] and we need to combine 2 bytes together
	 * 16 bit Data = firstByte<<8|secondByte
	 */

//	for (int i=0; i<numRegs; i++)
//	{
//		Holding_Registers_Database[startAddr++] = (RxData[index++]<<8)|RxData[index++];
//	}
	for (int i = 0; i < numRegs; i++)
	{
	    Holding_Registers_Database[startAddr++] = (RxData[index] << 8) | RxData[index + 1];
	    index += 2; // Tăng index lên 2 đơn vị sau khi đã dùng xong cho cả 2 byte
	}
	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | num of Regs    | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES      | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // num of Regs HIGH Byte
	TxData[5] = RxData[5];   // num of Regs LOW Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeSingleReg (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	if (startAddr>49)  // The Register Address can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Save the 16 bit data
	 * Data is the combination of 2 bytes, RxData[4] and RxData[5]
	 */

	Holding_Registers_Database[startAddr] = (RxData[4]<<8)|RxData[5];

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // Reg Data HIGH Byte
	TxData[5] = RxData[5];   // Reg Data LOW  Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeSingleCoil (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coil Address

	if (startAddr>(LENG_MODBUS_DATA*10))  // The Coil Address can not be more than 199 as we only have record of 200 Coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = startAddr/8;  // which byte we have to start writing the data into
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte


	/* The next 2 bytes in the RxData determines the state of the coil
	 * A value of FF 00 hex requests the coil to be ON.
	 * A value of 00 00 requests it to be OFF.
	 * All other values are illegal and will not affect the coil.
	 */

	if ((RxData[4] == 0xFF) && (RxData[5] == 0x00))
	{
		Coils_Database[startByte] |= 1<<bitPosition; // Replace that bit with 1
	}

	else if ((RxData[4] == 0x00) && (RxData[5] == 0x00))
	{
		Coils_Database[startByte] &= ~(1<<bitPosition); // Replace that bit with 0
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // Coil Data HIGH Byte
	TxData[5] = RxData[5];   // Coil Data LOW  Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeMultiCoils (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coil Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if ((numCoils<1)||(numCoils>1968))  // maximum no. of coils as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numCoils-1;  // Last coils address
	if (endAddr>(LENG_MODBUS_DATA*10))  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Calculation for the bit in the database, where the modification will be done */
	int startByte = startAddr/8;  // which byte we have to start writing the data into
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte
	int indxPosition = 0;  // The shift position in the current indx of the RxData buffer

	int indx = 7;  // we need to keep track of index in RxData

	/* The approach is simple. We will read 1 bit (starting from the very first bit in the RxData Buffer)
	 * at a time and store them in the Database.
	 * First find the offset in the first byte we write into, for eg- if the start coil is 13,
	 * we will Write into database[1] with an offset of 5. This bit is read from the RxData[indx] at 0th indxposition.
	 * Then we will keep shifting the RxData[indx] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte and start modifying the next byte in the database
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy from the next byte of the RxData
	 * This keeps going until the number of coils required have been modified
	 */

	// Modify the bits as per the Byte received
	for (int i=0; i<numCoils; i++)
	{
		if (((RxData[indx]>>indxPosition)&0x01) == 1)
		{
			Coils_Database[startByte] |= 1<<bitPosition;  // replace that bit with 1
		}
		else
		{
			Coils_Database[startByte] &= ~(1<<bitPosition);  // replace that bit with 0
		}

		bitPosition++; indxPosition++;

		if (indxPosition>7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)  // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition=0;
			startByte++;
		}
	}

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // num of coils HIGH Byte
	TxData[5] = RxData[5];   // num of coils LOW  Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}
