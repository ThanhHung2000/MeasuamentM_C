/*
 * drive.c
 *
 *  Created on: Jan 16, 2026
 *      Author: Admin
 */


#include "drive.h"
#include <stdlib.h>
#include "modbusSlave.h"
#define NUM_AXIT_ROBOT 0x03
#define AXIT_X_ROBOT   0x00
#define AXIT_Y_ROBOT   0x01
#define AXIT_Z_ROBOT   0x02
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
const uint16_t my_array[400]={0,};
void Set_Direction_OX(uint8_t status);
void Set_Direction_OY(uint8_t status);
void Set_Direction_OZ(uint8_t status);
MC_Axis_t Rotbot_axis[NUM_AXIT_ROBOT];
void Init_Timer_chanal(void)
{
	/* 1. Reset Counter */
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}
void Robot_Init(void)
{
	// TRỤC X
	Rotbot_axis[AXIT_X_ROBOT].htim= &htim1;
	Rotbot_axis[AXIT_X_ROBOT].htim_counter= &htim2;
	Rotbot_axis[AXIT_X_ROBOT].channel=TIM_CHANNEL_1;
	Rotbot_axis[AXIT_X_ROBOT].channel_counter=TIM_CHANNEL_2;
	Rotbot_axis[AXIT_X_ROBOT].Set_Direction_Pin=Set_Direction_OX;
	// TRỤC Y
	Rotbot_axis[AXIT_Y_ROBOT].htim= &htim3;
	Rotbot_axis[AXIT_Y_ROBOT].htim_counter= &htim5;
	Rotbot_axis[AXIT_Y_ROBOT].channel=TIM_CHANNEL_1;
	Rotbot_axis[AXIT_Y_ROBOT].channel_counter=TIM_CHANNEL_1;
	Rotbot_axis[AXIT_Y_ROBOT].Set_Direction_Pin=Set_Direction_OY;
	// TRỤC Z
	Rotbot_axis[AXIT_Z_ROBOT].htim= &htim8;
	Rotbot_axis[AXIT_Z_ROBOT].htim_counter= &htim4;
	Rotbot_axis[AXIT_Z_ROBOT].channel=TIM_CHANNEL_3;
	Rotbot_axis[AXIT_Z_ROBOT].channel_counter=TIM_CHANNEL_1;
	Rotbot_axis[AXIT_Z_ROBOT].Set_Direction_Pin=Set_Direction_OZ;
	for(int i=0x00U;i<NUM_AXIT_ROBOT;i++)
	{
		Rotbot_axis[i].current_pos =0x00U;
		Rotbot_axis[i].target_pos =0x00U;
		Rotbot_axis[i].current_speed=SET_SPEED_1000HZ;
		Rotbot_axis[i].target_speed=0X00U;
		Rotbot_axis[i].accel =0X00U;
		Rotbot_axis[i].state =STANDSTILL;
		Rotbot_axis[i].ramp_time=0x00U;
		Rotbot_axis[i].counter_pos=0x00U;
		Rotbot_axis[i].direction=0x00U;
		Rotbot_axis[i].indexaxis=i;
		Rotbot_axis[i].fulse_stop=0x00U;
		Rotbot_axis[i].offset=0x00U;
	}
}
// MC_Axis_t quản lý thông số của truc x,y, cụ thể
//int32_t pos vị trí đích tính theo xung, vị trí tuyệt đối so với điểm gốc
// uint32_t speed Vận tốc mục tiêu, đơn vị tần số ( số xung / giây)
// uint16_t accel (Gia tốc - Acceleration) Độ dốc của quá trình tăng tốc và giảm tốc.Tham số này sẽ quyết định việc bạn thay đổi giá trị ARR nhanh hay chậm trong ngắt 1ms để đạt tới speed.
//QUAN TRỌNG: Ép cập nhật giá trị từ vùng đệm vào thanh ghi thực thi
//axis->htim->Instance->EGR = TIM_EGR_UG;
void MC_MoveAbsolute(MC_Axis_t* axis, uint32_t pos, uint32_t speed)// mục đích Kích hoạt di chuyển đến vị trí tuyệt đối
{
// 1. Kiểm tra nếu trục đang bận hoặc có lỗi thì không nhận lệnh mới (Tùy logic)
	if(axis->state == AXIS_ERROR || axis->busy == 0x01) return;

	// 2. Gán các tham số mục tiêu
	axis->target_pos = pos;
	axis->target_speed = speed;
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
	axis->delta_pos = axis->target_pos > axis->current_pos ? (axis->target_pos - axis->current_pos):(axis->current_pos - axis->target_pos);
	// 4. Chuyển trạng thái sang Tăng tốc để bộ Handler bắt đầu làm việc
	axis->state = START_RUN;
	axis->busy = 0x01U;
	axis->done = 0x00U;
	// RESET BỘ ĐẾM COUNTER CỦA TIMER PHÁT XUNG VÀ TIMER ĐẾM XUNG
	__HAL_TIM_SET_COUNTER(axis->htim, 0);
	__HAL_TIM_SET_COUNTER(axis->htim_counter, 0);
    //  Chọn hướng đi cho chân DIR
	axis->Set_Direction_Pin(axis->direction);
//	 SET SỐ XUNG CẦN ĐẾM CỦA TIMER ĐẾN XUNG ĐỂ DỪNG XUNG CỦA TIMER PHÁT XUNG
    __HAL_TIM_SET_AUTORELOAD(axis->htim_counter, axis->delta_pos);
    __HAL_TIM_SET_COMPARE(axis->htim_counter, axis->channel_counter, axis->delta_pos);
    axis->counter_pos=0x00U;
    axis->current_speed=SET_SPEED_1000HZ;
    // CƯỠNG BỨC cập nhật giá trị từ vùng đệm vào thanh ghi thực thi CỦA timer đếm xung
    axis->htim_counter->Instance->EGR = TIM_EGR_UG;
    // set tần só ban đầu của timer phát xung là 15 hz, lúc khởi động coi tần số gần bằng min là 1kHz
    __HAL_TIM_SET_AUTORELOAD(axis->htim, SET_FREQ_1KHZ);
    __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, (SET_FREQ_1KHZ/2));
    axis->htim->Instance->EGR = TIM_EGR_UG;// ép buộc cập nhập giá trị mới vào vùng đệm
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
uint8_t MC_MoveHomeAbsolute(MC_Axis_t* axis)
{
	axis->state = DECELERATING;// ÉP dừng pwm sau khoảng time giảm tốc
	axis->ramp_time=0x00U;
	if(axis->state == STANDSTILL)
	{
		axis->current_pos=0x00U;
		return 0x01U;
	}
	return 0x00U;
}
uint8_t MC_MoveLinear(int32_t posx,int32_t posy,int32_t posz,float freq_max )// thời điểm kết thúc gần bằng nhau tuyệt đối
{
	float deltaX=(float)( posx > Rotbot_axis[0].current_pos ? posx-Rotbot_axis[0].current_pos : Rotbot_axis[0].current_pos - posx);
	float deltaY=(float)( posy > Rotbot_axis[1].current_pos ? posy-Rotbot_axis[1].current_pos : Rotbot_axis[1].current_pos - posy);
	float Lmax = (deltaX > deltaY) ? deltaX : deltaY;
	MC_MoveAbsolute(&Rotbot_axis[2],posz,freq_max);
	if(Lmax==0x00U) return 0;
	uint32_t freqx=(uint32_t)((freq_max*deltaX/Lmax));
	uint32_t freqy=(uint32_t)((freq_max*deltaY/Lmax));
	MC_MoveAbsolute(&Rotbot_axis[0],posx,freqx);
	MC_MoveAbsolute(&Rotbot_axis[1],posy,freqy);
	return 0x01U;
}
void Rotbot_controler(MC_Axis_t* axis)
{
	int32_t curent_counter=0x00U;
	uint32_t new_arr=0x00U;
    switch (axis->state)
    {
		case START_RUN:
		{
			// 5. Khởi động Timer PWM phát xung
			HAL_TIM_PWM_Start(axis->htim, axis->channel);
		    HAL_TIM_PWM_Start(axis->htim_counter, axis->channel_counter);
			axis->state = ACCELERATING;
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
        		axis->ramp_time--;
        	}
            break;

        case CONSTANT_VEL:// chạy với tần số cố định
        	axis->current_speed = axis->target_speed;
        	if((axis->delta_pos-axis->counter_pos) <= axis->fulse_stop)
			{
        		axis->state = DECELERATING;
        		axis->ramp_time--;
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
        default:
            break;
    }
    // CẬP NHẬT PHẦN CỨNG
    if (axis->current_speed > SET_SPEED_1000HZ || axis->busy==0x01)
    {
         new_arr = (uint32_t)(1000000U / (uint32_t)(axis->target_speed)); // Giả sử Clock Timer 1MHz
        __HAL_TIM_SET_AUTORELOAD(axis->htim, (new_arr-1));
        __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, (new_arr / 2));
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
        Holding_Registers_Database[axis->indexaxis]=axis->current_pos;
        axis->counter_pos = curent_counter;
        if ( axis->current_pos == axis->target_pos||axis->ramp_time==0x00U)
        {
            axis->current_speed = SET_SPEED_1000HZ;
            axis->state = STANDSTILL;
            axis->busy = 0x00U;
            axis->done = 0x01U;
            axis->ramp_time=0x00U;
            axis->fulse_stop=0x00U;
            // TH này là ép buộc dừng khi trong trường hợp MC_Stoping và dừng lại sau (TIME_RAMPING + 1)ms
            __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, SET_SPEED_1000HZ);
            HAL_TIM_PWM_Stop(axis->htim_counter, axis->channel_counter);

        }
    }
}
void  MC_Control_Interrupt(void)
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0==0x01))
	{
		MC_Stop(&Rotbot_axis[0]);
	}
	for(int i=0;i<NUM_AXIT_ROBOT;i++)
	{
		Rotbot_controler(&Rotbot_axis[i]);// thay đổi tần số ở đây
	}
}




void Set_Direction_OX(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9, status );
}
void Set_Direction_OY(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, status );
}
void Set_Direction_OZ(uint8_t status)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, status );
}


