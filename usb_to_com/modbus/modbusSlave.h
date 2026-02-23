/*
 * modbusSlave.h
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f4xx_hal.h"

#define SLAVE_ID 1

#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03


uint8_t readHoldingRegs (uint8_t *Rx_Uart);
uint8_t readInputRegs (uint8_t *Rx_Uart);
uint8_t readCoils (uint8_t *Rx_Uart);
uint8_t readInputs (uint8_t *Rx_Uart);

uint8_t writeSingleReg (uint8_t *Rx_Uart);
uint8_t writeHoldingRegs (uint8_t *Rx_Uart);
uint8_t writeSingleCoil (uint8_t *Rx_Uart);
uint8_t writeMultiCoils (uint8_t *Rx_Uart);

void modbusException (uint8_t *Rx_Uart, uint8_t exceptioncode);
uint16_t Get_Holding_Registers(uint8_t index);
uint8_t Get_Coild(uint8_t index);
void Set_Inputs_Database(uint8_t index,uint8_t data);
void Copy_Holding_Registers(uint8_t index,uint8_t index_coppy);
//void Update_Input_Register(uint8_t index, uint16_t toa_do,uint16_t toc_do, uint16_t state, uint16_t test);
void Set_Input_Register(uint8_t index, uint16_t data);
void Update_Input_Register(uint8_t index, uint16_t toa_do,uint16_t toc_do, uint16_t state);
void Reset_Tray(uint8_t index);
void Reset_Oxis(void);
#endif /* INC_MODBUSSLAVE_H_ */
