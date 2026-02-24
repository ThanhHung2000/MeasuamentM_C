/*
 * drive.c
 *
 *  Created on: Jan 16, 2026
 *      Author: Admin
 */


#include "drive.h"
#include <stdlib.h>
#include "modbusSlave.h"
#include "mgr_hmi.h"
#include"dvr_gpio.h"
#include "mgr_hmi.h"
#include "modbusSlave.h"
// Mảng chứa 46 giá trị Jerk (đơn vị: Hz/ms^2)
// Tính theo công thức: (Fmax - 1000) / (200 * 401)
// với jerk_table[0] tương ứng với Fmax=1000 Hz
// với jerk_table[49] tương ứng với Fmax=50000 Hz
const float jerk_table[50] = {
	    0.000000, 0.198020, 0.396040, 0.594059, 0.792079, 0.990099, 1.188119, 1.386139, 1.584158, 1.782178,
	    1.980198, 2.178218, 2.376238, 2.574257, 2.772277, 2.970297, 3.168317, 3.366337, 3.564356, 3.762376,
	    3.960396, 4.158416, 4.356436, 4.554455, 4.752475, 4.950495, 5.148515, 5.346535, 5.544554, 5.742574,
	    5.940594, 6.138614, 6.336634, 6.534654, 6.732673, 6.930693, 7.128713, 7.326733, 7.524753, 7.722772,
	    7.920792, 8.118812, 8.316832, 8.514852, 8.712872, 8.910892, 9.108911, 9.306931, 9.504951, 9.702971
};
// Theo công thức F(n) =F0 + j*(n+1)*n/2 tăng tốc lên Fmax trong 400ms
const float triangle_array[TIME_RAMPING] = {
    0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 66, 78, 91, 105, 120, 136, 153, 171, 190, 210,
    231, 253, 276, 300, 325, 351, 378, 406, 435, 465, 496, 528, 561, 595, 630, 666, 703, 741, 780, 820, 861,
    903, 946, 990, 1035, 1081, 1128, 1176, 1225, 1275, 1326, 1378, 1431, 1485, 1540, 1596, 1653, 1711, 1770, 1830, 1891, 1953,
    2016, 2080, 2145, 2211, 2278, 2346, 2415, 2485, 2556, 2628, 2701, 2775, 2850, 2926, 3003, 3081, 3160, 3240, 3321, 3403, 3486,
    3570, 3655, 3741, 3828, 3916, 4005, 4095, 4186, 4278, 4371, 4465, 4560, 4656, 4753, 4851, 4950, 5050
};
uint8_t Set_Direction_OX(uint8_t status);
uint8_t Set_Direction_OY(uint8_t status);
uint8_t Set_Direction_OZ(uint8_t status);
volatile static  MC_Axis_t Rotbot_axis[NUM_AXIT_ROBOT];
volatile Axis_Config_t Rotbot_axis_target[NUM_AXIT_ROBOT]={0,};
void Init_Timer_chanal(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	/* 1. Reset Counter */
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
}

void Reset_position(void)
{
	for(int i=0x00U;i<NUM_AXIT_ROBOT;i++)
	{
		Rotbot_axis[i].current_pos =0x00U;
		Rotbot_axis[i].target_pos =0x00U;
		Rotbot_axis[i].current_speed=0x00U;
		Rotbot_axis[i].target_speed=0X00U;
		Rotbot_axis[i].accel =0X00U;
		Rotbot_axis[i].state =STANDSTILL;
		Rotbot_axis[i].ramp_time=0x00U;
		Rotbot_axis[i].counter_pos=0x00U;
		Rotbot_axis[i].direction=0x00U;
		Rotbot_axis[i].fulse_stop=0x00U;
		Rotbot_axis[i].offset=0x00U;
		*(Rotbot_axis[i].current_pos_shodow)   = 0x00U;
		*(Rotbot_axis[i].current_speed_shadow) = 0x00U;
	}
}
void Robot_Init(void)
{
	// TRỤC X
	Rotbot_axis[AXIT_X_ROBOT].htim= &htim3;//htim3
	Rotbot_axis[AXIT_X_ROBOT].htim_counter= &htim5;//htim5
	Rotbot_axis[AXIT_X_ROBOT].channel=TIM_CHANNEL_1;//TIM_CHANNEL_1
	Rotbot_axis[AXIT_X_ROBOT].channel_counter=TIM_CHANNEL_1;//TIM_CHANNEL_1
	Rotbot_axis[AXIT_X_ROBOT].Set_Direction_Pin=Set_Direction_OX;//Set_Direction_OY
	Rotbot_axis[AXIT_X_ROBOT].max_axis=MAX_Axis_OX;//&Rotbot_axis_target[AXIT_X_ROBOT].max_limit;
	Rotbot_axis[AXIT_X_ROBOT].indexaxis=0x00U;

	Rotbot_axis[AXIT_X_ROBOT].current_pos_shodow =&Input_Registers_Database[0];
	Rotbot_axis[AXIT_X_ROBOT].current_speed_shadow =&Input_Registers_Database[1];
	Rotbot_axis[AXIT_X_ROBOT].axis_busy_shadow =&Input_Registers_Database[2];
	// TRỤC Y
	Rotbot_axis[AXIT_Y_ROBOT].htim= &htim1;//htim1
	Rotbot_axis[AXIT_Y_ROBOT].htim_counter= &htim2;//htim2
	Rotbot_axis[AXIT_Y_ROBOT].channel=TIM_CHANNEL_1;//TIM_CHANNEL_1
	Rotbot_axis[AXIT_Y_ROBOT].channel_counter=TIM_CHANNEL_2;//TIM_CHANNEL_2
	Rotbot_axis[AXIT_Y_ROBOT].Set_Direction_Pin=Set_Direction_OY;//Set_Direction_OX
	Rotbot_axis[AXIT_Y_ROBOT].max_axis=MAX_Axis_OY;//&Rotbot_axis_target[AXIT_Y_ROBOT].max_limit;
	Rotbot_axis[AXIT_Y_ROBOT].indexaxis=0x03U;

	Rotbot_axis[AXIT_Y_ROBOT].current_pos_shodow =&Input_Registers_Database[3];
	Rotbot_axis[AXIT_Y_ROBOT].current_speed_shadow =&Input_Registers_Database[4];
	Rotbot_axis[AXIT_Y_ROBOT].axis_busy_shadow =&Input_Registers_Database[5];
	// TRỤC Z
	Rotbot_axis[AXIT_Z_ROBOT].htim= &htim8;
	Rotbot_axis[AXIT_Z_ROBOT].htim_counter= &htim4;
	Rotbot_axis[AXIT_Z_ROBOT].channel=TIM_CHANNEL_3;
	Rotbot_axis[AXIT_Z_ROBOT].channel_counter=TIM_CHANNEL_1;
	Rotbot_axis[AXIT_Z_ROBOT].Set_Direction_Pin=Set_Direction_OZ;
	Rotbot_axis[AXIT_Z_ROBOT].max_axis=MAX_Axis_OZ;//&Rotbot_axis_target[AXIT_Z_ROBOT].max_limit;
	Rotbot_axis[AXIT_Z_ROBOT].indexaxis=0x06U;
	Rotbot_axis[AXIT_Z_ROBOT].current_pos_shodow =&Input_Registers_Database[6];
	Rotbot_axis[AXIT_Z_ROBOT].current_speed_shadow =&Input_Registers_Database[7];
	Rotbot_axis[AXIT_Z_ROBOT].axis_busy_shadow =&Input_Registers_Database[8];
	Reset_position();
}
// MC_Axis_t quản lý thông số của truc x,y, cụ thể
//int32_t pos vị trí đích tính theo xung, vị trí tuyệt đối so với điểm gốc
// uint32_t speed Vận tốc mục tiêu, đơn vị tần số ( số xung / giây)
// uint16_t accel (Gia tốc - Acceleration) Độ dốc của quá trình tăng tốc và giảm tốc.Tham số này sẽ quyết định việc bạn thay đổi giá trị ARR nhanh hay chậm trong ngắt 1ms để đạt tới speed.
//QUAN TRỌNG: Ép cập nhật giá trị từ vùng đệm vào thanh ghi thực thi
//axis->htim->Instance->EGR = TIM_EGR_UG;
void MC_MoveAbsolute_old(MC_Axis_t* axis, int32_t pos, uint32_t speed)// mục đích Kích hoạt di chuyển đến vị trí tuyệt đối
{
// 1. Kiểm tra nếu trục đang bận hoặc có lỗi thì không nhận lệnh mới (Tùy logic)
	if(axis->state == AXIS_ERROR || axis->busy == 0x01) return;

	// 2. Gán các tham số mục tiêu
	if(pos >= axis->max_axis) pos=axis->max_axis;
	if(pos <= 0x00U) pos=0x00U;//
	axis->target_pos = pos;
	axis->target_speed = (speed>=50000U) ? 50000U : speed;
	speed = (speed <= 1000U) ? 1000U : speed;
	axis->accel = jerk_table[(speed/(uint32_t)1000)-1] ;// speed là 1k đến 50k
	// 3. Xác định hướng di chuyển
	if (axis->target_pos > axis->current_pos) {
		axis->direction = 0x00; // Chạy tiến
	} else if (axis->target_pos < axis->current_pos) {
		axis->direction = 0x01; // Chạy lùi
	} else
	{
		return; // Đã ở đúng vị trí
	}
    //  Chọn hướng đi cho chân DIR
	if(axis->Set_Direction_Pin(axis->direction) != axis->direction)
	{
		axis->Set_Direction_Pin(axis->direction);
	}
	axis->delta_pos = axis->target_pos > axis->current_pos ? (axis->target_pos - axis->current_pos):(axis->current_pos - axis->target_pos);
	// 4. Chuyển trạng thái sang Tăng tốc để bộ Handler bắt đầu làm việc
	axis->state = START_RUN;
	axis->busy = 0x01U;
	axis->done = 0x00U;
    axis->counter_pos=0x00U;
    axis->current_speed=SET_SPEED_1000HZ;
    // CƯỠNG BỨC cập nhật giá trị từ vùng đệm vào thanh ghi thực thi CỦA timer đếm xung
}
void MC_MoveAbsolute(volatile MC_Axis_t* axis, int32_t pos, uint32_t speed)// mục đích Kích hoạt di chuyển đến vị trí tuyệt đối
{
	// 1. Kiểm tra nếu trục đang bận hoặc có lỗi thì không nhận lệnh mới (Tùy logic)
		if(axis->state == AXIS_ERROR || axis->busy == 0x01) return;

		// 2. Gán các tham số mục tiêu
		if(pos >= axis->max_axis) pos= axis->max_axis;
		if(pos <= 0x00U) pos=0U;// tránh chạm home liên tục sinh ngắt
		axis->target_pos = pos;
		// 3. Xác định hướng di chuyển
		if (axis->target_pos > axis->current_pos) {
			axis->direction = 0x00; // Chạy tiến
		} else if (axis->target_pos < axis->current_pos) {
			axis->direction = 0x01; // Chạy lùi
		} else
		{
			return; // Đã ở đúng vị trí
		}
	    //  Chọn hướng đi cho chân DIR
		if(axis->Set_Direction_Pin(axis->direction) != axis->direction)
		{
			axis->Set_Direction_Pin(axis->direction);
		}
		speed = (speed<50000U) ? speed:50000U;
		speed = (speed>1000U) ? speed:1000U;
		axis->target_speed = speed;
		axis->accel = jerk_table[(speed/(uint32_t)1000U)-1] ;// speed là 1k đến 50k

		axis->delta_pos = axis->target_pos > axis->current_pos ? (axis->target_pos - axis->current_pos):(axis->current_pos - axis->target_pos);
		// 4. Chuyển trạng thái sang Tăng tốc để bộ Handler bắt đầu làm việc
		axis->state = START_RUN;
		axis->busy = 0x01U;
		*axis->axis_busy_shadow =0x01U;
		axis->done = 0x00U;
	    axis->counter_pos=0x00U;
	    axis->current_speed=SET_SPEED_500HZ;
	    // CƯỠNG BỨC cập nhật giá trị từ vùng đệm vào thanh ghi thực thi CỦA timer đếm xung
}
void Timer_PWM_Chanal_Start(volatile MC_Axis_t* axis)
{
	// RESET BỘ ĐẾM COUNTER CỦA TIMER PHÁT XUNG VÀ TIMER ĐẾM XUNG
	__HAL_TIM_SET_COUNTER(axis->htim, 0);
	__HAL_TIM_SET_COUNTER(axis->htim_counter, 0);
	//	 SET SỐ XUNG CẦN ĐẾM CỦA TIMER ĐẾN XUNG ĐỂ DỪNG XUNG CỦA TIMER PHÁT XUNG
	__HAL_TIM_SET_AUTORELOAD(axis->htim_counter, axis->delta_pos);
	__HAL_TIM_SET_COMPARE(axis->htim_counter, axis->channel_counter, axis->delta_pos);
	axis->htim_counter->Instance->EGR = TIM_EGR_UG;
    // set tần só ban đầu của timer phát xung là 15 hz, lúc khởi động coi tần số gần bằng min là 1kHz
    __HAL_TIM_SET_AUTORELOAD(axis->htim, SET_SPEED_500HZ_ARR);
    __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, (SET_SPEED_500HZ_ARR/2));
    axis->htim->Instance->EGR = TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
    axis->current_speed = SET_SPEED_500HZ;
}
void MC_Stop(MC_Axis_t* axis)// mục đích Kích hoạt dừng tuyệt đối sau 100ms khi giảm tốc
{
	if(axis->state != STANDSTILL && axis->state != DECELERATING)// khác 0 và khác 4
	{
		axis->state = DECELERATING;// ÉP dừng pwm sau khoảng time giảm tốc
		axis->offset=0x01U;
	}
}

void MC_MoveAbsoluteTest(uint32_t posx,uint32_t posy,uint32_t posz, uint32_t freq)// freq sẽ nhận giá trị 1k đến 50k
{
	// Tạo 1 mảng
	MC_MoveAbsolute(&Rotbot_axis[0],posx,freq);//max 50kHz
	MC_MoveAbsolute(&Rotbot_axis[1],posy,freq);//max 50kHz
	MC_MoveAbsolute(&Rotbot_axis[2],posz,freq);//max 50kHz
}

void MC_MoveRelative(MC_Axis_t* axis,int32_t distance,uint32_t freq )
{
	int32_t taget_move=0x00U;

	taget_move = (axis->current_pos + distance);
	MC_MoveAbsolute(axis,taget_move,freq);
}
uint8_t MC_MoveHomeAbsolute( volatile MC_Axis_t* axis)
{
	if(axis->busy != 0x00U)
	{
		__HAL_TIM_SET_COMPARE(axis->htim, axis->channel, 0x00u);
		axis->htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
		axis->ramp_time=0x00U;
		axis->state = STANDSTILL;
		axis->current_speed = 0x00U;
	}
	return 0x01U;
}
uint8_t MC_Errow_Axis(MC_Axis_t* axis)
{
	if(axis->busy != 0x00U)
	{
		__HAL_TIM_SET_COMPARE(axis->htim, axis->channel, 0x00u);
		axis->htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
		axis->ramp_time=0x00U;
		axis->current_speed = 0x00U;
	}
	return 0x01U;
}
void Emergency_Stop(void)
{
	for(int i=0;i<NUM_AXIT_ROBOT;i++)
	{
		MC_MoveHomeAbsolute(&Rotbot_axis[i]);// thay đổi tần số ở đây
	}
}
void Interrup_gpio_OX(void)
{
		GPIOA->ODR &=~(1<<9u);
        __HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_X_ROBOT].htim, Rotbot_axis[AXIT_X_ROBOT].channel, 0x00u);
        Rotbot_axis[AXIT_X_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        Rotbot_axis[AXIT_X_ROBOT].ramp_time=0x00U;
        Rotbot_axis[AXIT_X_ROBOT].state = STANDSTILL;
        Rotbot_axis[AXIT_X_ROBOT].current_speed=0x00U;
}
void Interrup_gpio_OY(void)
{
	GPIOC->ODR &=~(1<<7U);
    __HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_Y_ROBOT].htim, Rotbot_axis[AXIT_Y_ROBOT].channel, 0x00u);
    Rotbot_axis[AXIT_Y_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
    Rotbot_axis[AXIT_Y_ROBOT].ramp_time=0x00U;
    Rotbot_axis[AXIT_Y_ROBOT].state = STANDSTILL;
    Rotbot_axis[AXIT_Y_ROBOT].current_speed=0x00U;
}
void Interrup_gpio_OZ(void)
{
	GPIOC->ODR &=~(1<<9U);
	__HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_Z_ROBOT].htim, Rotbot_axis[AXIT_Z_ROBOT].channel, 0x00u);
    Rotbot_axis[AXIT_Z_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
    Rotbot_axis[AXIT_Z_ROBOT].ramp_time=0x00U;
    Rotbot_axis[AXIT_Z_ROBOT].state = STANDSTILL;
    Rotbot_axis[AXIT_Z_ROBOT].current_speed=0x00U;
}
void Interrup_gpio(uint16_t GPIO_Pin)// kích hoạt ngắt ngoài khi về home
{
	if(GPIO_Pin==GPIO_PIN_0)//trục X
	{
		GPIOC->ODR &=~(1<<7U);
//		__HAL_TIM_SET_COUNTER(&htim2, Rotbot_axis[AXIT_X_ROBOT].delta_pos);
//		 Rotbot_axis[AXIT_X_ROBOT].htim_counter->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        __HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_X_ROBOT].htim, Rotbot_axis[AXIT_X_ROBOT].channel, 0x00u);
        Rotbot_axis[AXIT_X_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        Rotbot_axis[AXIT_X_ROBOT].ramp_time=0x00U;
        Rotbot_axis[AXIT_X_ROBOT].state = STANDSTILL;
        Rotbot_axis[AXIT_X_ROBOT].current_speed=0x00U;
	}
	if(GPIO_Pin==GPIO_PIN_1)//trục Y
	{
		GPIOA->ODR &=~(1<<9u);
//		__HAL_TIM_SET_COUNTER(&htim5, Rotbot_axis[AXIT_Y_ROBOT].delta_pos);
//		 Rotbot_axis[AXIT_Y_ROBOT].htim_counter->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        __HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_Y_ROBOT].htim, Rotbot_axis[AXIT_Y_ROBOT].channel, 0x00u);
        Rotbot_axis[AXIT_Y_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        Rotbot_axis[AXIT_Y_ROBOT].ramp_time=0x00U;
        Rotbot_axis[AXIT_Y_ROBOT].state = STANDSTILL;
        Rotbot_axis[AXIT_Y_ROBOT].current_speed=0x00U;
	}
	if(GPIO_Pin==GPIO_PIN_2)//trục Y
	{
		GPIOC->ODR &=~(1<<9U);
//		__HAL_TIM_SET_COUNTER(&htim4, Rotbot_axis[AXIT_Z_ROBOT].delta_pos);
//		Rotbot_axis[AXIT_Z_ROBOT].htim_counter->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
		__HAL_TIM_SET_COMPARE(Rotbot_axis[AXIT_Z_ROBOT].htim, Rotbot_axis[AXIT_Z_ROBOT].channel, 0x00u);
        Rotbot_axis[AXIT_Z_ROBOT].htim->Instance->EGR |= TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
        Rotbot_axis[AXIT_Z_ROBOT].ramp_time=0x00U;
        Rotbot_axis[AXIT_Z_ROBOT].state = STANDSTILL;
        Rotbot_axis[AXIT_Z_ROBOT].current_speed=0x00U;
	}

}
uint8_t Motor_Busy(void)// kiểm tra xem 3 trục robot có đã goàn toàn dừng lại chưa
{
	return (Rotbot_axis[0].busy || Rotbot_axis[1].busy || Rotbot_axis[2].busy) ;
}
uint8_t Move_Home_3Step(volatile uint8_t * home_tep)// về home 3 giai đoạn
{
	//uint8_t result=0x00U;
	static uint16_t counter_100=0x00U;
	static uint8_t onetime=0x00U;
	static uint8_t step=0x00U;
	static uint8_t time=0x00U;
	if( *home_tep == 0x01)
	{
		step=0x01U;
		*home_tep = 0x02U;

	}
	switch(step)
	{
		case 0x01U:
		{
            // step 1 : đưa Z VỀ 0 trước
			if(onetime==0x00U)
			{
				if(Motor_Busy()==0x00U)
				{
					if(Get_home_done())
					{
						Rotbot_axis[2].current_pos +=200U;
					}
					else
					{
						Rotbot_axis[2].current_pos=13000U;
					}
					if(Get_State_Sensor(AXIT_Z_ROBOT)==0x00U) MC_MoveAbsolute(&Rotbot_axis[2],0x00U,3000U);// di chuyển về home
					if(++counter_100 >= 2U)
					{
						onetime=0x01U;
						counter_100=0x00U;
					}
				}
			}
			if(Get_State_Sensor(AXIT_Z_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_Z_ROBOT]);// nếu chạm home thì dừng
			if(Motor_Busy()==0x00U)
			{
				if(++time>=5U)
				{
					time =0x00U;
					step=0x02U;
					onetime=0x00U;
				}
			}
			else time=0x00U;
		}
		break;
		case 0x02U:
		{
			// step 2 : về home max 2 trục 550cm và 280cm, NẾU chạm home ở ngắt ngoài thì cho về MC_Stop
			if(onetime==0x00U)
			{
				if(Motor_Busy()==0x00U)
				{
					if(Get_home_done())
					{
						Rotbot_axis[0].current_pos +=1000U;
						Rotbot_axis[1].current_pos +=1000U;
						Rotbot_axis[2].current_pos +=1000U;
					}
					else
					{
						Rotbot_axis[0].current_pos =55000U;
						Rotbot_axis[1].current_pos =28000U;
						Rotbot_axis[2].current_pos +=200U;
					}
					if(Get_State_Sensor(AXIT_X_ROBOT) ==0x00U ) MC_MoveAbsolute(&Rotbot_axis[0],0x00U,3000U);
					if(Get_State_Sensor(AXIT_Y_ROBOT) ==0x00U) MC_MoveAbsolute(&Rotbot_axis[1],0x00U,1500U);
					if(Get_State_Sensor(AXIT_Z_ROBOT) ==0x00U) MC_MoveAbsolute(&Rotbot_axis[2],0x00U,2000U);
				    if(++counter_100 > 1U)
					{
				    	onetime=0x01U;
				    	counter_100=0x00U;
					}
				}

			}
			if(Get_State_Sensor(AXIT_X_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_X_ROBOT]);// NẾU K CHẠM HOME LÀ LỖI
			if(Get_State_Sensor(AXIT_Y_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_Y_ROBOT]);
			if(Get_State_Sensor(AXIT_Z_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_Z_ROBOT]);
			if(Motor_Busy()==0x00U)
			{
				if(++time>=5U)
				{
					time =0x00U;
					step=0x03U;
					onetime=0x00U;
				}

			}
			else time=0x00U;
		}
		break;
		case 0x03U:// đi xa khoảng 5CM mỗi trục
		{

			if(onetime==0x00U)
			{
				Rotbot_axis[0].current_pos=0x00U;
				Rotbot_axis[1].current_pos=0x00U;
				Rotbot_axis[2].current_pos=0x00U;
				MC_MoveAbsolute(&Rotbot_axis[0],3000U,4000U);
				MC_MoveAbsolute(&Rotbot_axis[1],3000U,4000U);
				MC_MoveAbsolute(&Rotbot_axis[2],3000U,4000U);
				onetime=0x01U;
			}
			if(Motor_Busy()==0x00U)
			{

				if(++time>=5U)
				{
					time =0x00U;
					step=0x04U;
					onetime=0x00U;
					Rotbot_axis[0].current_pos +=1000U;
					Rotbot_axis[1].current_pos +=1000U;
					Rotbot_axis[2].current_pos +=1000U;
				}
			}
			else time=0x00U;
		}
		break;
		case 0x04U://
		{
			// step 2 : về home max 2 trục 550cm và 280cm, NẾU chạm home ở ngắt ngoài thì cho về MC_Stop
			if(onetime==0x00U)
			{
				MC_MoveAbsolute(&Rotbot_axis[0],0x00U,1000U);
				MC_MoveAbsolute(&Rotbot_axis[1],0x00U,1000U);
				MC_MoveAbsolute(&Rotbot_axis[2],0x00U,1000U);
				Rotbot_axis[0].homing=0x01U;
				Rotbot_axis[1].homing=0x01U;
				Rotbot_axis[2].homing=0x01U;
				onetime=0x01U;
			}
			if(Get_State_Sensor(AXIT_X_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_X_ROBOT]);// NẾU K CHẠM HOME LÀ LỖI
			if(Get_State_Sensor(AXIT_Y_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_Y_ROBOT]);
			if(Get_State_Sensor(AXIT_Z_ROBOT)) MC_MoveHomeAbsolute(&Rotbot_axis[AXIT_Z_ROBOT]);
			if(Motor_Busy()==0x00U)
			{
				if(++time>=5U)
				{
					time =0x00U;
					onetime=0x00U;
					step=0x05U;
					Reset_position();
				}
			}
			else time=0x00U;
		}
		break;
		case 0x05U://
		{
			if(onetime==0x00U)
			{
				MC_MoveAbsolute(&Rotbot_axis[0],200U,1000U);
				MC_MoveAbsolute(&Rotbot_axis[1],200U,1000U);
				MC_MoveAbsolute(&Rotbot_axis[2],200U,1000U);
				onetime=0x01U;
			}
			if(Motor_Busy()==0x00U)
			{
				if(++time>=5U)
				{
					time =0x00U;
					onetime=0x00U;
					step=0x00U;
					return 0x01U;

				}
			}
			else time=0x00U;
		}
		break;
		default:
		break;
	}
	return 0x00U;
}
uint8_t MC_MoveLinear(int32_t posx,int32_t posy,int32_t posz )// thời điểm kết thúc gần bằng nhau tuyệt đối
{
	float deltaX=(float)( posx > Rotbot_axis[0].current_pos ? posx-Rotbot_axis[0].current_pos : Rotbot_axis[0].current_pos - posx);
	float deltaY=(float)( posy > Rotbot_axis[1].current_pos ? posy-Rotbot_axis[1].current_pos : Rotbot_axis[1].current_pos - posy);
	float Lmax = (deltaX > deltaY) ? deltaX : deltaY;
	MC_MoveAbsolute(&Rotbot_axis[2],posz,Rotbot_axis_target[2].target_speed);
	if(Lmax < 1.0f) return Motor_Busy();
	uint16_t speed0 = Rotbot_axis_target[0].target_speed;
	uint16_t speed1 = Rotbot_axis_target[1].target_speed;
	float freq_max = (float)((speed0 > speed1) ? speed0 : speed1);
	int32_t freqx=(int32_t)((freq_max*deltaX/Lmax));
	int32_t freqy=(int32_t)((freq_max*deltaY/Lmax));

	if(freqx > speed0) freqx=speed0;
	if(freqy > speed1) freqy=speed1;
	MC_MoveAbsolute(&Rotbot_axis[0],posx,freqx);
	MC_MoveAbsolute(&Rotbot_axis[1],posy,freqy);
	return Motor_Busy();
}
void MC_MoveHandle(uint8_t axis,uint8_t status, int dir)
{
	if((Get_home_done()==0x00U) || (Get_Go_home()==0x01U)) return ;
	switch(status)
	{
		case STATUS_JOGGING_OXIS:// jogging
		{
			MC_MoveAbsolute(&Rotbot_axis[axis],dir*(Rotbot_axis[axis].max_axis),5000U);
			if(Rotbot_axis[axis].timer_jogging1khz==0x00U) Rotbot_axis[axis].jogging =0x01U;
		}
		break;
		case STATUS_STEP_OXIS:// step
		{
			if(dir==0x00U)
			{
				int32_t value = (Rotbot_axis[axis].current_pos > 5U) ? Rotbot_axis[axis].current_pos - 5U : 0x00U;

				MC_MoveAbsolute(&Rotbot_axis[axis],value,5000U);
			}
			else
			{
				MC_MoveAbsolute(&Rotbot_axis[axis],Rotbot_axis[axis].current_pos + 5U,5000U);
			}
		}
		break;
		case STATUS_STOP_OXIS:// stop
		{
			MC_MoveHomeAbsolute(&Rotbot_axis[axis]);
		}
		break;

		default:
		break;
	}
}
void Rotbot_controler(volatile MC_Axis_t* axis,uint8_t index)
{
	int32_t curent_counter=0x00U;
	uint32_t new_arr=0x00U;
    switch (axis->state)
    {
		case START_RUN:
		{
			Timer_PWM_Chanal_Start(axis);
			if(axis->homing==0x01U)
			{
				axis->state =HOME_STOPPING;
			}
			else if(axis->jogging==0x01U)
			{
				axis->state =JOGGING_RUN;
				axis->jogging=0x00U;

				axis->timer_jogging1khz=0x01U;
			}
			else
			{
				axis->state = ACCELERATING;
			}

			axis->ramp_time++;
		}
		break;
        case ACCELERATING://Đang tăng tốc
            // Tăng vận tốc dần dần
        	if(axis->ramp_time>=TIME_RAMPING)
			{
        		axis->fulse_stop = axis->counter_pos;
        		axis->ramp_time --;
        		axis->state = CONSTANT_VEL;
        		if(axis->target_speed>SET_SPEED_1000HZ) axis->current_speed = axis->target_speed;
        		// TÌM Số xung đã được ramping, để khi còn lại số xung này thì giảm
        		// ĐÂY Chính là số xung còn lại axis->counter_pos
			}
        	else
        	{
        		axis->current_speed = SET_SPEED_1000HZ + (uint32_t)((axis->accel)*triangle_array[axis->ramp_time]);
        	}
        	axis->ramp_time++;
        	// có trường hợp mà thời gian chạy mà ramping chưa max mà đã dừng thì luôn luôn so sánh số xung hiện tại và tổng số xung cần băm
        	if(axis->delta_pos <= (2*axis->counter_pos))
        	{
        		axis->state = DECELERATING;
        	}
            break;

        case CONSTANT_VEL:// chạy với tần số cố định
        	if(axis->timer_jogging1khz)
        	{
        		if(axis->target_speed>SET_SPEED_1000HZ) axis->current_speed = axis->target_speed;
        	}
        	else if((axis->delta_pos-axis->counter_pos) <= axis->fulse_stop)
			{
        		axis->state = DECELERATING;
			}
        	break;
        case DECELERATING:
        	axis->ramp_time--;
			if(axis->ramp_time <=0 ) axis->ramp_time=0x00U;
			if(axis->ramp_time<TIME_RAMPING)
			{
				axis->current_speed =SET_SPEED_1000HZ + (uint32_t)((axis->accel)*triangle_array[axis->ramp_time]);
			}
        	break;
        case HOME_STOPPING:
			if(axis->current_pos > 1500U)
			{
				axis->current_speed =SET_SPEED_1000HZ;
			}
			else
			{
				axis->current_speed =SET_SPEED_500HZ;
			}
        	break;
        case JOGGING_RUN:
			{

				if(++axis->timer_jogging1khz>=40U)
				{
					axis->timer_jogging1khz=0x00U;
					axis->ramp_time++;
					axis->current_speed = SET_SPEED_500HZ + (uint32_t)((axis->accel)*triangle_array[axis->ramp_time]);

					if(axis->ramp_time>=TIME_RAMPING)
					{
						axis->ramp_time --;
						axis->timer_jogging1khz=0x01U;
						axis->state = CONSTANT_VEL;
					}
				}

			}
			break;
        default:
            break;
    }
    // CẬP NHẬT PHẦN CỨNG
    if ( axis->busy==0x01)
    {
    	if(axis->current_speed >= SET_SPEED_500HZ)
    	{
			 new_arr = (uint32_t)(1000000U / (uint32_t)(axis->current_speed)); // Giả sử Clock Timer 1MHz
			__HAL_TIM_SET_AUTORELOAD(axis->htim, (new_arr));
			__HAL_TIM_SET_COMPARE(axis->htim, axis->channel, (new_arr / 2));
    	}
        curent_counter=axis->htim_counter->Instance->CNT;
        if(axis->direction == 0x00)
        {
                axis->current_pos += ((curent_counter-axis->counter_pos));
                axis->current_pos +=axis->offset;
        }
        else
        {
        	axis->current_pos -= (curent_counter-axis->counter_pos);
        	axis->current_pos -=axis->offset;
        }
        axis->offset=0x00U;
        axis->counter_pos = curent_counter;
        if(axis->done == 0x01U)
        {
        	axis->homing=0x00U;
        	axis->timer_jogging1khz=0x00U;
        	axis->done = 0x00U;
            axis->busy = 0x00U;// lần sau mới cho busy về 0
            __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, 0x00u);
            __HAL_TIM_SET_AUTORELOAD(axis->htim, SET_SPEED_500HZ);
            axis->htim->Instance->EGR |= TIM_EGR_UG; // Ép cập nhật để ra 0V ngay lập tức
        }
        else if ( axis->current_pos == axis->target_pos || axis->ramp_time==0x00U)
        {
            axis->current_speed = 0x00U;
            if(axis->state != AXIS_ERROR) axis->state = STANDSTILL;
            axis->done = 0x01U;
            axis->ramp_time=0x00U;
            axis->fulse_stop=0x00U;
        }
    }
	*axis->current_pos_shodow = (uint16_t)(axis->current_pos);
	*axis->current_speed_shadow = (uint16_t)(axis->current_speed);
	*axis->axis_busy_shadow = (uint16_t)(axis->busy);
}
void Copy_Data_Target(void)
{
	// 1. Lưu lại trạng thái ngắt hiện tại vào thanh ghi PRIMASK
	uint32_t primask_bit = __get_PRIMASK();
	__disable_irq(); // Chỉ tốn 1 chu kỳ máy
	for(int i=0;i<NUM_AXIT_ROBOT;i++)
	{
			Rotbot_axis_target[i].target_position = Get_Holding_Registers(Rotbot_axis[i].indexaxis);
			Rotbot_axis_target[i].target_speed=     Get_Holding_Registers(Rotbot_axis[i].indexaxis +1);
	}
	__set_PRIMASK(primask_bit);
}
void Update_Input(void)
{
	Copy_data_output();
	Copy_Gpio_Input();
}

void  MC_Control_Interrupt(void)
{
	for(int i=0;i<NUM_AXIT_ROBOT;i++)
	{
		Rotbot_controler(&Rotbot_axis[i],i);// thay đổi tần số ở đây
	}
	Task_Main_Controler();
}

uint8_t Set_Direction_OY(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9, status );
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9, status );
	return HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) > 0x00U ? 0x01U:0x00U;
}
uint8_t Set_Direction_OX(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, status );
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, status );
	return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) > 0x00U ? 0x01U:0x00U;
}
uint8_t Set_Direction_OZ(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, status );
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, status );
	return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) > 0x00U ? 0x01U:0x00U;
}
uint8_t Reset_Errow_Axis(void)
{
	static uint16_t time_delay=0x00U;
	if(Rotbot_axis[0].state==AXIS_ERROR)
	{
		time_delay++;
		Out_put_Duphong1(0x00);
		if(time_delay>=3000)
		{
			Rotbot_axis[0].state=STANDSTILL;
			time_delay=0x00U;
			Out_put_Duphong1(0x01u);
			return 0x01U;
		}
	}
	if(Rotbot_axis[1].state==AXIS_ERROR)
	{
		time_delay++;
		Out_put_Duphong2(0x00);
		if(time_delay>=3000)
		{
			Rotbot_axis[1].state=STANDSTILL;
			time_delay=0x00U;
			Out_put_Duphong2(0x01);
			return 0x01U;
		}
	}
	return 0x00U;

}

