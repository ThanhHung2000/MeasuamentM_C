
#include"dvr_gpio.h"
volatile static Input_state_Sesor   sensor;
uint8_t Gpio_read_input(void)
{
	uint32_t idr =GPIOC->IDR;
	uint8_t input=0x00U;
	if(idr & GPIO_PIN_0) input |= (1<<0);
	if(idr & GPIO_PIN_1) input |= (1<<1);
	if(idr & GPIO_PIN_2) input |= (1<<2);
	return input;
}

void Task_Gpio_input()
{
	uint8_t input_sensor_current;
	uint8_t input_read;
	input_sensor_current=Gpio_read_input();
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    	input_read=(input_sensor_current &(1<<i)) ? 0x01U :0x00U;
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
uint8_t Get_State_Sensor(uint8_t channel)
{
	return sensor.Sensor_State[channel];
}

