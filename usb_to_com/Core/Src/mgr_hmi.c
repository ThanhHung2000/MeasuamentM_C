/*
 * mgr_hmi.c
 *
 *  Created on: Jan 25, 2026
 *      Author: Admin
 */

#include "RS232.h"
#include "main.h"
#include "delay.h"
#include  "Common.h"
#include "mgr_hmi.h"
#include "drive.h"
#include "modbusSlave.h"
#include"dvr_gpio.h"
#include <math.h>
#include "flash_data.h"
#include "drive.h"


static int8_t fisrtbit=0x00U;
static uint8_t Emergency = 0;
uint8_t save=0x00U;
volatile static uint8_t home=0x00U;
volatile static uint8_t home_done=0x00U;
uint8_t start_run=0x00U;
uint8_t stop_run=0x00U;
static uint8_t state=0x00U;
static float dX, dY, Angle;
typedef struct {
    uint8_t last_state;    // Trạng thái Coil ở chu kỳ 1ms trước
    uint32_t press_timer;  // Bộ đếm thời gian (ms)
    uint8_t is_jogging;    // Cờ báo hiệu đã chuyển sang chế độ chạy liên tục
} HMI_Button_t;

HMI_Button_t hmi_btns[NUM_BUTTON_HOLD]; // Quản lý 6 nút
void Handle_Home(void);
void Handle_Set(void);
void Handle_Emergency(void);
void Main_Reset(void);
void Main_Stop(void);

void Handle_Left(uint8_t data);
void Handle_Right(uint8_t data);
void Handle_In(uint8_t data);
void Handle_Out(uint8_t data);
void Handle_Up(uint8_t data);
void Handle_Down(uint8_t data);

void Scanning_Task(void);


typedef void (*ActionHandler_t)(void);
typedef void (*ActionHandler_button)(uint8_t);
typedef struct {
    ActionHandler_t handler;
} Main_Handle_Controler;

typedef struct {
    ActionHandler_button handler;
} Map_button_t;
typedef struct {
	ActionHandler_t handler;
} Map_main_t;
Map_button_t Handle_buttonTable[NUM_BUTTON_HOLD] = {
	{ Handle_Left   },
	{ Handle_Right  },
	{ Handle_In     },
	{ Handle_Out    },
	{ Handle_Up     },
	{ Handle_Down   }
};
Main_Handle_Controler Main_Table[5] =
{
	{  Handle_Home       },
	{  Handle_Set        },//
	{  Handle_Emergency  },
	{  Main_Reset        },
	{  Main_Stop         }
};
void Init_hmi(void)
{
}
uint8_t Get_home_done(void)
{
	return home_done;
}
uint8_t Get_Go_home(void)
{
	return home;
}
uint16_t FindActiveBit(uint8_t *data,uint8_t numbyte)
{
    for (uint8_t byteIdx = 0; byteIdx < numbyte; ++byteIdx)
    {
        uint8_t val = data[byteIdx];
        if (val) // chỉ xử lý nếu có ít nhất 1 bit được set
        {
        	int8_t fisrtbit = __builtin_ffs(val)-1;
        	return (byteIdx * 8 + fisrtbit);
        }
    }
    return 0xffff; // không có bit nào = 1
}
// dữ liệu đã lưu ở Coils_Database[1]; 0-5: trái 0x, phải Ox, lên 0x, xuống 0y, lên 0z, xuống 0z

void Task_Move_Oxis()
{
	uint8_t current_state=0x00U;
	for(int i=0x00U; i< NUM_BUTTON_HOLD;i++)
	{
		current_state= ( (Control_motor->all) &(1<<i))==0x00U ? 0x00U : 0x1U;
		if (current_state == 0x1U)
		{
			// --- TRƯỜNG HỢP ĐANG NHẤN ---
			hmi_btns[i].press_timer++; // Mỗi lần gọi là 1ms, cứ thế cộng dồn lên

			// Nếu giữ đủ 500ms và chưa Jog thì kích hoạt Jog
			if (hmi_btns[i].press_timer >= 150u && hmi_btns[i].is_jogging == 0)
			{
				//MC_Jog(axis, direction, 10000, 5000);
				hmi_btns[i].is_jogging = 1;
				Handle_buttonTable[i].handler(STATUS_JOGGING_OXIS);
			}
		}
		else
		{
			// --- TRƯỜNG HỢP NHẢ TAY (current_state == 0) ---
			if (hmi_btns[i].last_state == 1)
			{ // Chỉ xử lý khi vừa mới nhả tay (Sườn xuống)

				if (hmi_btns[i].press_timer < 150u) {
					// Nhấn nhả nhanh: Nhích 100 xung
					//MC_MoveRelative(axis, direction * 100, 5000, 2000);
					Handle_buttonTable[i].handler(STATUS_STEP_OXIS);
				}
				else {
					// Nhả sau khi đã Jog: Dừng trục
					//MC_Stop(axis, 5000);
					Handle_buttonTable[i].handler(STATUS_STOP_OXIS);
				}
				// Reset các biến để chuẩn bị cho lần nhấn sau
				hmi_btns[i].press_timer = 0;
				hmi_btns[i].is_jogging = 0;
			}
		}
		// Lưu trạng thái để bắt sườn xuống ở chu kỳ 1ms sau
		hmi_btns[i].last_state = current_state;
	}
}
void Set_Emergency_Stop()
{
	Emergency=0x01U;
}
void Task_Run_HMI(void)
{
	static uint8_t timer_Emergency_off=0x00U;
	Copy_target_fromPC();
	if(Emergency == 0x01U )
	{
		Emergency_Stop();
		if(Get_State_Sensor(0x003U)==0x00U)
		{
			if(++timer_Emergency_off>=10U)
			{
				timer_Emergency_off=0x00U;
				Emergency=0x00U;
				home=0x01U;
			}

		}
		return;
	}
	uint8_t current_main = Main_controler->all;
	fisrtbit = __builtin_ffs(current_main)-1;
	if(fisrtbit >= 0)
	{
		if(Main_Table[fisrtbit].handler != NULL)
		{
			Main_Table[fisrtbit].handler();
		}

	}
	Task_Move_Oxis();
}
// Giả sử P1, P2, P3 và dX, dY, Angle đã được tính ở bước Calibration
Point2D Get_Target_Zigzag(int hang, int cot) {
    Point2D target;
    int cot_thuc_te;

    // Nếu hàng lẻ (1, 3, 5...), đảo ngược chiều quét cột (từ 12 về 0)
    if (hang % 2 != 0)
    {
        cot_thuc_te = 12 - cot;
    } else
    {
        cot_thuc_te = cot;
    }
    float offsetX = (float)cot_thuc_te * dX;
    float offsetY = (float)hang * dY;
    // Áp dụng ma trận xoay để bù góc nghiêng của khay
    target.x = Point2D_Tray1->x1 + (offsetX * cosf(Angle)) - (offsetY * sinf(Angle));
    target.y = Point2D_Tray1->y1 + (offsetX * sinf(Angle)) + (offsetY * cosf(Angle));
    return target;
}
uint8_t Task_Scan_Tray(uint16_t* time)
{
	static int8_t i=0x00U, j=0x00U;
	static int32_t pulseX;
	static int32_t pulseY;
	if(state==0x00U)
	{
		if(Motor_Busy()==0x00U)
		{
			Point2D pos = Get_Target_Zigzag(i, j);
			 pulseX = (int32_t)(pos.x );
			 pulseY = (int32_t)(pos.y );
			MC_MoveLinear(pulseX,pulseY, 0x00u);// di chuyển sang điểm mới
			state=0x01U;
		}

	}
	else
	{

		if(Motor_Busy()==0x00U)
		{
			if(++*time>=100U)
			{
				*time =0x00U;
				state=0x00u;
				//MC_MoveLinear(pulseX,pulseY, 0x00u, 30000U);// di chuyển trục Z lên
				j++;
				if(j>=13U)
				{
					j=0x00U;
					i++;
				}
				if(i>=13U)
				{
					i=0x00U;
					j=0x00U;
					return 0x01U;
				}
			}
			else if(*time==0x01U)
			{
				//MC_MoveLinear(pulseX,pulseY, 7000u, 30000U);// di chuyển Z xuống
			}
		}
	}
	return 0x00U;
}
void Scanning_Task(void)
{
	static uint8_t step_run=0x00U;
	static uint16_t time=0x00U;
	static  uint16_t sample_x=0x00U;
	static uint16_t sample_y=0x00U;
	switch(step_run)
	{
		case 0x00u:
		{
			sample_x=(Get_Holding_Registers(6) + Get_Holding_Registers(8))/2;
			sample_y=(Get_Holding_Registers(7) + Get_Holding_Registers(11u))/2;
			// di chuyển đến điểm chứa bàn cờ hiệu chỉnh, ví dụ tray rubber, di chuyển đến trung tâm của tray rubber
			if(sample_x != 0x00U && sample_y != 0x00U )
			{
				//MC_MoveLinear(sample_x,sample_y,0x00U,20000u);
			}
			else
			{
				return;
			}

			if(Motor_Busy()==0x00U)
			{

				if(++time>=100U)// 1S sau khi hiệu chuẩn mới di chuyển sang khay sản phẩm;
				{
					// Tính khoảng cách trung bình (Pitch) giữa 13 con hàng
					// 12 khoảng cách cho 13 con hàng
					dX = sqrtf(powf((float)Point2D_Tray1->x2 - (float)Point2D_Tray1->x1, 2) + powf((float)Point2D_Tray1->y2 - (float)Point2D_Tray1->y1, 2)) / 12.0f;
					dY = sqrtf(powf((float)Point2D_Tray1->x3 - (float)Point2D_Tray1->x1, 2) + powf((float)Point2D_Tray1->y3 - (float)Point2D_Tray1->y1, 2)) / 12.0f;
					// Tính góc nghiêng của khay so với trục X của Robot
					Angle = atan2f((float)Point2D_Tray1->y2 - (float)Point2D_Tray1->y1, (float)Point2D_Tray1->x2 - (float)Point2D_Tray1->x1);
					//MC_MoveLinear(sample_x,sample_y,0x00U,30000U);
					time =0x00U;
					step_run=0x01U;
				}
				else if(time==0x01U)
				{
					//MC_MoveLinear(sample_x,sample_y,7000U,30000U);
				}
			}
		}
		break;
		case 0x01u:// có hai khay sản phẩm được lấy vị trí 3 điểm là tray 1 và tray 2
		{
			if(Task_Scan_Tray(&time))
			{
				//MC_MoveLinear(sample_x,sample_y,0x00U,10000u);
				step_run=0x01U;
				start_run=0x00U;
				time=0x00U;
			}
		}
		break;
		default:
		break;
	}
}
void Task_Run_Home(void)
{
	Gpio_input();
	if(home!=0x00u)
	{
		start_run=0x00U;
		state=0X00u;// trạng thái để đưa z xuống con hàng cần reset khi về home
		if(Move_Home_3Step(&home))
		{
			Reset_position();
			home=0x00U;
			home_done=0x01U;
			Reset_Oxis();
		}
	}
}

void Main_Reset(void)
{

}

void Main_Stop(void)
{

}

// Nut sang trai Ox -> xa gốc tọa độ
void Handle_Left(uint8_t data)
{
	MC_MoveHandle(AXIT_X_ROBOT,data,0x00);
}
void Handle_Right(uint8_t data)
{
	MC_MoveHandle(AXIT_X_ROBOT,data,0x01u);
}
void Handle_In(uint8_t data)
{
	MC_MoveHandle(AXIT_Y_ROBOT,data, 0x00u);
}
void Handle_Out(uint8_t data)
{
	MC_MoveHandle(AXIT_Y_ROBOT,data, 0x01u);
}
void Handle_Up(uint8_t data)
{
	MC_MoveHandle(AXIT_Z_ROBOT,data, 0x00u);
}
void Handle_Down(uint8_t data)
{
	MC_MoveHandle(AXIT_Z_ROBOT,data, 0x01);
}

void Handle_Set(void)
{
	if(Get_home_done()==0x00U || Get_Go_home()==0x01U) return ;
	// lấy dữ liệu từ 4x Holding_Registers_Database để làm target
	MC_MoveLinear(Rotbot_axis_target[0].target_position,Rotbot_axis_target[1].target_position,Rotbot_axis_target[2].target_position);

}
void Handle_Home(void)
{
	if(home==0x00U)
	{
		home=0x01U;
	}
}
void Handle_Emergency(void)
{
	Emergency=0x01U;
}

