/*
 * drive.h
 *
 *  Created on: Jan 16, 2026
 *      Author: Admin
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_
#include "main.h"
#define  SET_FREQ_1KHZ                   999U
#define  SET_SPEED_1000HZ                1000U  // 1
#define  SET_SPEED_500HZ                 499U  // 1
#define  TIME_RAMPING                    101U
#define  MAX_Axis_OX					54000U
#define  MAX_Axis_OY					27800U
#define  MAX_Axis_OZ					13000U

#define NUM_AXIT_ROBOT 0x03
#define AXIT_X_ROBOT   0x00
#define AXIT_Y_ROBOT   0x01
#define AXIT_Z_ROBOT   0x02

typedef struct {
    uint32_t target_freq;    // Tần số đích (Tốc độ)
    uint32_t total_pulses;   // Tổng số xung cần phát (Vị trí)
    uint16_t ramp_time_ms;   // Thời gian tăng tốc (ms)
    uint32_t current_pulse;  // Biến đếm số xung thực tế
    float acceleration;      // Gia tốc tính toán được
} Axis_Config;

typedef enum {
    STANDSTILL,      // Đang dừng
	START_RUN,
    ACCELERATING,    // Đang tăng tốc
    CONSTANT_VEL,    // Chạy tốc độ đều
    DECELERATING,    // Đang giảm tốc
	HOME_STOPPING,
    POSITION_REACHED, // Đã đến đích
	AXIS_ERROR,        // TRẠNG THÁI LỖI (Cần Reset mới chạy lại được)
	JOGGING_RUN
} AxisState;
typedef struct {
    // Cấu hình phần cứng
    TIM_HandleTypeDef* htim;
    TIM_HandleTypeDef* htim_counter;
    uint32_t channel;
    uint32_t channel_counter;
    uint8_t (* Set_Direction_Pin)(uint8_t);
    // Thông số chuyển động
    int32_t current_pos;   // Vị trí hiện tại (số xung)
    volatile int32_t target_pos;    // Vị trí đích
    volatile int32_t delta_pos;
    volatile uint32_t counter_pos;
    volatile int32_t current_speed;     // Vận tốc hiện tại (Hz)
    volatile int32_t target_speed;      // Vận tốc mục tiêu (Hz)
    volatile float  accel;           // Gia tốc (Hz/ms)
    uint8_t direction;
    volatile AxisState state;       // Trạng thái hiện tại
    uint8_t offset;
    volatile int32_t ramp_time;   // Bước tính toán ramp
    volatile uint32_t fulse_stop;   // Bước tính toán ramp
    // Các cờ trạng thái giống PLC
    volatile uint8_t busy;    // Bằng 1 khi lệnh đang thực thi (tăng tốc, chạy đều hoặc giảm tốc)
	uint8_t done;    // Bằng 1 trong 1 chu kỳ khi đã đến đúng vị trí đích
	uint8_t error;   // Bằng 1 khi trục gặp sự cố
	uint8_t active;  // Bằng 1 khi trục đang có quyền điều khiển hardware
	uint8_t indexaxis;
	int32_t max_axis;
	volatile uint8_t homing;
	volatile uint8_t jogging;
	uint16_t timer_jogging1khz;


} MC_Axis_t;
typedef struct {
    float target_position;    // Tọa độ mục tiêu (ví dụ: mm hoặc độ)
    float target_speed;       // Tốc độ mục tiêu (ví dụ: mm/s)
    int32_t max_limit;          // Giới hạn phần mềm tối đa (Soft Limit Max)
} Axis_Config_t;

void Init_Timer_chanal(void);
void Robot_Init(void);
void  MC_Control_Interrupt(void);
uint8_t MC_MoveLinear(int32_t posx,int32_t posy,int32_t posz );// thời điểm kết thúc gần bằng nhau tuyệt đối
void MC_MoveAbsoluteTest(uint32_t posx,uint32_t posy,uint32_t posz, uint32_t freq);
void MC_MoveHandle(uint8_t axis,uint8_t status, int dir);
uint8_t Move_Home_3Step(volatile uint8_t * home_tep);
void Reset_position(void);
void Emergency_Stop(void);
void Interrup_gpio(uint16_t GPIO_Pin);
void Interrup_gpio_OX(void);
void Interrup_gpio_OY(void);
void Interrup_gpio_OZ(void);
uint8_t Motor_Busy(void);
void Copy_target_fromPC(void);
extern Axis_Config_t Rotbot_axis_target[NUM_AXIT_ROBOT];
//uint8_t MC_MoveLinear(int32_t posx,int32_t posy,int32_t posz,float freq_max );

//void MC_MoveHandle(uint8_t axis,uint8_t status, int dir);
#endif /* INC_DRIVE_H_ */
