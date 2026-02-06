
#include"dvr_gpio.h"
#include "modbusSlave.h"
void Out_put_Xilanh1(uint8_t status);
void Out_put_Xilanh2(uint8_t status);
void Out_put_Vacum_hut1(uint8_t status);
void Out_put_Vacum_hut2(uint8_t status);
void Out_put_Vacum_nha1(uint8_t status);
void Out_put_Vacum_nha2(uint8_t status);
void Out_put_Den_xanh(uint8_t status);
void Out_put_Den_do(uint8_t status);
void Out_put_Den_coi(uint8_t status);
void Out_put_Duphong1(uint8_t status);
void Out_put_Duphong2(uint8_t status);
void Out_put_Duphong3(uint8_t status);
void Out_put_Duphong4(uint8_t status);
void Out_put_Duphong5(uint8_t status);
void Out_put_Duphong6(uint8_t status);
void Out_put_Duphong7(uint8_t status);
void Out_put_Duphong8(uint8_t status);
void Out_put_Duphong9(uint8_t status);
typedef void (*Gpio_out_handle)(uint8_t);

typedef struct {
	Gpio_out_handle handler;
} Gpio_out_handle_t;

Gpio_out_handle_t Gpio_output[18]=
{
	{ Out_put_Xilanh1 },
	{ Out_put_Xilanh2  },
	{ Out_put_Vacum_hut1  },
	{ Out_put_Vacum_hut2  },
	{ Out_put_Vacum_nha1 },
	{ Out_put_Vacum_nha2  },
	{ Out_put_Den_xanh  },
	{ Out_put_Den_do  },
	{ Out_put_Den_coi  },
	{ Out_put_Duphong1  },
	{ Out_put_Duphong2  },
	{ Out_put_Duphong3  },
	{ Out_put_Duphong4  },
	{ Out_put_Duphong5  },
	{ Out_put_Duphong6  },
	{ Out_put_Duphong7  },
	{ Out_put_Duphong8  },
	{ Out_put_Duphong9  },
};

volatile static Input_state_Sesor   sensor;
uint32_t Gpio_read_input(void)
{
	uint32_t idr =GPIOC->IDR;
	uint32_t input=0x00U;
	if(idr & GPIO_PIN_0) input |= (1<<0);
	if(idr & GPIO_PIN_1) input |= (1<<1);
	if(idr & GPIO_PIN_2) input |= (1<<2);
	if(idr & GPIO_PIN_3) input |= (1<<3);
	if(idr & GPIO_PIN_4) input |= (1<<4);
	if(idr & GPIO_PIN_5) input |= (1<<5);
	idr =GPIOB->IDR;
	if(idr & GPIO_PIN_0) input |= (1<<6);
	if(idr & GPIO_PIN_1) input |= (1<<7);
	if(idr & GPIO_PIN_12) input |= (1<<17);
	idr =GPIOE->IDR;
	if(idr & GPIO_PIN_7) input |= (1<<8);
	if(idr & GPIO_PIN_8) input |= (1<<9);
	if(idr & GPIO_PIN_9) input |= (1<<10);
	if(idr & GPIO_PIN_10) input |= (1<<11);
	if(idr & GPIO_PIN_11) input |= (1<<12);
	if(idr & GPIO_PIN_12) input |= (1<<13);
	if(idr & GPIO_PIN_13) input |= (1<<14);
	if(idr & GPIO_PIN_14) input |= (1<<15);
	if(idr & GPIO_PIN_15) input |= (1<<16);
	return input;
}

void Gpio_input()
{
	uint32_t input_sensor_current;
	uint8_t input_read;
	input_sensor_current=Gpio_read_input();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    	input_read=(input_sensor_current &(1<<i)) ? 0x00U :0x01U;
        // 2. So sánh với lần đọc trước
        if (input_read == sensor.Last_Sensor_Reading[i])
        {
            // Tín hiệu vẫn GIỮ NGUYÊN trạng thái (không có rung dội)
            if (sensor.Sample_Counter[i] < NUMBER_GPIO_SAMPLING)
            {
            	sensor.Sample_Counter[i]++; // Tăng bộ đếm ổn định
            }
            else
            {
                // Đã đủ ngưỡng ổn định, CẬP NHẬT trạng thái chính thức
            	sensor.Sensor_State[i] = input_read;
            }
        }
        else
        {
            // Tín hiệu BỊ THAY ĐỔI (có thể do rung dội hoặc trạng thái thực)
            // Đặt bộ đếm về 0 và chờ xác nhận lại
        	sensor.Sample_Counter[i] = 0;
        }
        // 3. Cập nhật lần đọc trước cho chu kỳ tiếp theo
    	sensor.Last_Sensor_Reading[i] = input_read;
    }
}
void Task_gpio_input(void)// copy dữ liệu sang địa chỉ 10000
{
	uint32_t gpio_input=0x00U;
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		if(sensor.Sensor_State[i])
		{
			gpio_input |=1UL<<i;
		}
	}
	Set_Inputs_Database(0x00U,(uint8_t)(gpio_input>>0U));
	//Inputs_Database[0]=(uint8_t)(gpio_input>>0U);
	Set_Inputs_Database(0x01U,(uint8_t)((uint8_t)(gpio_input>>8U)));
	//Inputs_Database[1]=(uint8_t)(gpio_input>>8U);

	Set_Inputs_Database(0x02U,(uint8_t)((gpio_input>>16U)&(0x03U)));
	//Inputs_Database[2] |= (uint8_t)((gpio_input>>16U)&(0x03U));
	// gán gpio_input sang mảng Inputs_Database[0] -> 18 bit
}
void Task_gpio_output(void)// copy dữ liệu sang địa chỉ 10000
{
	uint32_t gpio_output = ( (uint32_t)Get_Coild(2) << 0  ) |
						 ( (uint32_t)Get_Coild(3) << 8  ) |
						 ( (uint32_t)Get_Coild(4) << 16 ) ;
	uint8_t state=0x00U;
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		state =(gpio_output&(1UL<<i)) ? 0x00U:0x01U;
		if (Gpio_output[i].handler != NULL)
		{
			Gpio_output[i].handler(state);
		}
	}
}
uint8_t Get_State_Sensor(uint8_t channel)
{
	return sensor.Sensor_State[channel];
}

void Out_put_Xilanh1(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, status );
}
void Out_put_Xilanh2(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4, status );
}
void Out_put_Vacum_hut1(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5, status );
}
void Out_put_Vacum_hut2(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6, status );
}
void Out_put_Vacum_nha1(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, status );
}
void Out_put_Vacum_nha2(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, status );
}
void Out_put_Den_xanh(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4, status );
}
void Out_put_Den_do(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, status );
}
void Out_put_Den_coi(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, status );
}
void Out_put_Duphong1(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9, status );
}
void Out_put_Duphong2(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0, status );
}
void Out_put_Duphong3(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1, status );
}
void Out_put_Duphong4(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, status );
}
void Out_put_Duphong5(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, status );
}
void Out_put_Duphong6(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, status );
}
void Out_put_Duphong7(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, status );
}
void Out_put_Duphong8(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6, status );
}
void Out_put_Duphong9(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, status );
}
