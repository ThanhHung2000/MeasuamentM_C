#include "stm32f4xx_hal.h"
#include "flash_data.h"
#include "RS232.h"
#define FLASH_STORAGE_ADDR  0x080E0000 // Sector 11
#define POINT_COUNT 18 // 9 điểm * 2 trục

void Save_Calibration_To_Flash(uint16_t index)
{
    HAL_FLASH_Unlock();
    uint16_t* data_array;
    data_array=&Holding_Registers_Database[index];
    // 1. Xóa Sector 11 trước khi ghi mới
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_11;
    // VoltageRange_3 phù hợp với điện áp 2.7V - 3.6V (phổ biến nhất)
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // 2. Ghi từng Half-Word (2 byte) vào Flash
    uint32_t addr = FLASH_STORAGE_ADDR;
    for (int i = 0; i < POINT_COUNT; i++)
    {
        // Ghi 2 byte (Half-Word)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint64_t)data_array[i]) == HAL_OK)
        {
            addr += 2; // Tăng địa chỉ lên 2 byte cho lần ghi tiếp theo
        }
    }

    HAL_FLASH_Lock();
}


void Load_Calibration_From_Flash(uint16_t index)
{
	uint16_t* data_array;
	data_array=&Holding_Registers_Database[index];
    // Ép kiểu con trỏ để đọc từng 2 byte một
	uint8_t j=0x00U;
    uint16_t* flash_ptr = (uint16_t*)FLASH_STORAGE_ADDR;
	for (int i = 0; i < POINT_COUNT; i++)
	{
		if(flash_ptr[i] != 0xffffU && flash_ptr[i] != 0x00U)
		{
			data_array[i] = flash_ptr[i];
			if(i%2==0)
			{
				Motor_Lamp->all |= (1<<j);
				j++;
			}
		}

	}
}
