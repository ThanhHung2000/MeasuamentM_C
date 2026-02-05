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


extern uint16_t Holding_Registers_Database[50];
extern uint16_t Input_Registers_Database[50];
extern uint8_t Coils_Database[50];
extern uint8_t Inputs_Database[50];


uint8_t readHoldingRegs (void);
uint8_t readInputRegs (void);
uint8_t readCoils (void);
uint8_t readInputs (void);

uint8_t writeSingleReg (void);
uint8_t writeHoldingRegs (void);
uint8_t writeSingleCoil (void);
uint8_t writeMultiCoils (void);

void modbusException (uint8_t exceptioncode);
uint16_t Get_Holding_Registers(uint8_t index);
void Copy_Holding_Registers(uint8_t index,uint8_t index_coppy);
void Update_Input_Register(uint8_t index, uint16_t toa_do,uint16_t toc_do, uint16_t state );
void Reset_Tray(uint8_t index);
void Reset_Oxis(void);
#endif /* INC_MODBUSSLAVE_H_ */
