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
int8_t fisrtbit=0x00U;
uint8_t Emergency = 0;
uint8_t Glass_group = 0;
uint8_t save=0x00U;
volatile uint8_t home=0x01U;
uint8_t start_run=0x00U;
uint8_t stop_run=0x00U;
static uint8_t state=0x00U;
static float dX, dY, Angle;
#define HMI_STATE_BUTTON    0x01U
typedef struct {
    uint8_t last_state;    // Trạng thái Coil ở chu kỳ 1ms trước
    uint32_t press_timer;  // Bộ đếm thời gian (ms)
    uint8_t is_jogging;    // Cờ báo hiệu đã chuyển sang chế độ chạy liên tục
} HMI_Button_t;

HMI_Button_t hmi_btns[NUM_BUTTON_HOLD]; // Quản lý 6 nút
HMI_Button_t main_btns[NUM_BUTTON_MAIN]; // Quản lý 6 nút
void Handle_Set(void);
void Handle_Home(void);
void Handle_Left(uint8_t data);
void Handle_Right(uint8_t data);
void Handle_In(uint8_t data);
void Handle_Out(uint8_t data);
void Handle_Up(uint8_t data);
void Handle_Down(uint8_t data);


void Main_Reset(uint8_t data);
void Main_Start(uint8_t data);
void Main_Stop(uint8_t data);
void Main_SetX(uint8_t data);
void Main_SetY(uint8_t data);
void Main_SetZ(uint8_t data);
void Main_Manual(uint8_t data);

void Pick_handle1(void);
void Release_handle1(void);
void Pick_handle2(void);
void Release_handle2(void);
void Save_1(void);
void Save_2(void);
void Save_3(void);
void Setmove_point(void);

void Save_Glass_1(void);
void Save_Glass_2(void);
void Save_Glass_3(void);
void Save_Glass_All(void);
void Save_Cover_1(void);
void Save_Cover_2(void);
void Save_Cover_3(void);
void Save_Cover_All(void);

void Save_Tray_1(void);
void Save_Tray_2(void);
void Save_Tray_3(void);
void Save_Tray_All(void);
void Scanning_Task(void);


void Reset(void);

typedef void (*ActionHandler_t)(void);
typedef void (*ActionHandler_button)(uint8_t);
typedef struct {
    uint32_t bitMask;
    ActionHandler_t handler;
} Map_t;

typedef struct {
    ActionHandler_button handler;
} Map_button_t;
typedef struct {
	ActionHandler_t handler;
} Map_main_t;
Map_button_t Map_main[7]=
{
		{ Main_Reset  },
		{ Main_Start  },
		{ Main_Stop  },
		{ Main_SetX  },
		{ Main_SetY },
		{ Main_SetZ  },
		{ Main_Manual  }
};
Map_button_t Handel_buttonTable[6] = {
	{ Handle_Left  },
	{ Handle_Right  },
	{ Handle_In  },
	{ Handle_Out  },
	{ Handle_Up },
	{ Handle_Down  }
};

Map_t motorMoveTable[19] = {

	{ 1 << 0, Handle_Set      },
	{ 1 << 1, Handle_Home     },
	{ 1 << 2, Pick_handle1    },
	{ 1 << 3, Release_handle1 },
	{ 1 << 4, Pick_handle2    },
	{ 1 << 5, Release_handle2 },
	{ 1 << 6, Save_1          },
	{ 1 << 7, Save_2          },
	{ 1 << 8, Save_3          },
	{ 1 << 9, Setmove_point   },
	{ 1 << 10, Save_Glass_1  },
	{ 1 << 11, Save_Glass_2  },
	{ 1 << 12, Save_Glass_3 },
	{ 1 << 13, Save_Cover_1  },
	{ 1 << 14, Save_Cover_2  },
	{ 1 << 15, Save_Cover_3  },
	{ 1 << 16, Save_Tray_1  },
	{ 1 << 17, Save_Tray_2  },
	{ 1 << 18,Save_Tray_3  },
};
void Init_hmi(void)
{
	Tab->bits.Motor=0x01U;
	Tab->bits.Home =0x00U;
}
Map_t worker_controlTable[]={
	{ 1 << 3, Reset },
};
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

void Task_scan_HMI()
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
			if (hmi_btns[i].press_timer >= 300u && hmi_btns[i].is_jogging == 0)
			{
				//MC_Jog(axis, direction, 10000, 5000);
				hmi_btns[i].is_jogging = 1;
				Handel_buttonTable[i].handler(STATUS_JOGGING_OXIS);
			}
		}
		else
		{
			// --- TRƯỜNG HỢP NHẢ TAY (current_state == 0) ---
			if (hmi_btns[i].last_state == 1)
			{ // Chỉ xử lý khi vừa mới nhả tay (Sườn xuống)

				if (hmi_btns[i].press_timer < 300u) {
					// Nhấn nhả nhanh: Nhích 100 xung
					//MC_MoveRelative(axis, direction * 100, 5000, 2000);
					Handel_buttonTable[i].handler(STATUS_STEP_OXIS);
				}
				else {
					// Nhả sau khi đã Jog: Dừng trục
					//MC_Stop(axis, 5000);
					Handel_buttonTable[i].handler(STATUS_STOP_OXIS);
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
void Task_scan_maim()
{
	    uint8_t current_state=0x00U;
		current_state=  (Home_controller_hmi->bits.stop) ;//&(1<<i))==0x00U ? 0x00U : 0x1U;
		if (current_state == 0x1U)
		{
			// --- TRƯỜNG HỢP ĐANG NHẤN ---
			main_btns[2].press_timer++; // Mỗi lần gọi là 1ms, cứ thế cộng dồn lên

			// Nếu giữ đủ 500ms và chưa Jog thì kích hoạt Jog
			if (main_btns[2].press_timer >= 300u && main_btns[2].is_jogging == 0)
			{
				//MC_Jog(axis, direction, 10000, 5000);
				main_btns[2].is_jogging = 1;
//				Handel_buttonTable[i].handler(STATUS_JOGGING_OXIS);
				if(stop_run) stop_run=0x00U;
				else stop_run=0x01U;

			}
		}
		else
		{
			// --- TRƯỜNG HỢP NHẢ TAY (current_state == 0) ---
			if (main_btns[2].last_state == 1)
			{ // Chỉ xử lý khi vừa mới nhả tay (Sườn xuống)

				if (main_btns[2].press_timer < 300u) {
					// Nhấn nhả nhanh: Nhích 100 xung
					//MC_MoveRelative(axis, direction * 100, 5000, 2000);
//					Handel_buttonTable[i].handler(STATUS_STEP_OXIS);
					if(stop_run) stop_run=0x00U;
					else stop_run=0x01U;
				}
				else {
					// Nhả sau khi đã Jog: Dừng trục
					//MC_Stop(axis, 5000);
//					Handel_buttonTable[i].handler(STATUS_STOP_OXIS);
				}
				// Reset các biến để chuẩn bị cho lần nhấn sau
				main_btns[2].press_timer = 0;
				main_btns[2].is_jogging = 0;
			}
		}
		// Lưu trạng thái để bắt sườn xuống ở chu kỳ 1ms sau
		main_btns[2].last_state = current_state;
}
void Task_Run_HMI(void)
{
	if(start_run == 0x01U  && stop_run == 0x00U) Scanning_Task();
	if(Tab->bits.Home == 1)
	{
		// nếu có lệnh start thì di chuyển đến điểm lấy mẫu samplepoin hiệu chuẩn, sau đó di chuyển đến tray sản phẩm để do kích thước NG, Good
		uint8_t current_main = Home_controller_hmi->all;
		fisrtbit = __builtin_ffs(current_main)-1;
		if(fisrtbit >= 0)
		{
			Map_main[fisrtbit].handler(fisrtbit);
		}
		Task_scan_maim();

	}
	else if (Tab->bits.Motor == 1 && home==0x00U)
	{
		uint32_t current = ((Save_Tray -> all) << 16)|(Worker_Control->all<< 8)| (Control_motor->all);
		fisrtbit = __builtin_ffs(current)-7;
		if(fisrtbit >= 0)
		{
			motorMoveTable[fisrtbit].handler();
		}
		Task_scan_HMI();
	}

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
			MC_MoveLinear(pulseX,pulseY, 0x00u, 20000U);// di chuyển sang điểm mới
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
				MC_MoveLinear(pulseX,pulseY, 0x00u, 30000U);// di chuyển trục Z lên
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
				MC_MoveLinear(pulseX,pulseY, 7000u, 30000U);// di chuyển Z xuống
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
	if(Motor_Lamp->bits.tray1_1==0x00U || Motor_Lamp->bits.tray1_1==0x00U || Motor_Lamp->bits.tray1_1==0x00U)
	{
		start_run=0x00U;
		return ;
	}
	switch(step_run)
	{
		case 0x00u:
		{
			sample_x=(Get_Holding_Registers(6) + Get_Holding_Registers(8))/2;
			sample_y=(Get_Holding_Registers(7) + Get_Holding_Registers(11u))/2;
			// di chuyển đến điểm chứa bàn cờ hiệu chỉnh, ví dụ tray rubber, di chuyển đến trung tâm của tray rubber
			if(sample_x != 0x00U && sample_y != 0x00U )
			{
				MC_MoveLinear(sample_x,sample_y,0x00U,20000u);
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
					MC_MoveLinear(sample_x,sample_y,0x00U,30000U);
					time =0x00U;
					step_run=0x01U;
				}
				else if(time==0x01U)
				{
					MC_MoveLinear(sample_x,sample_y,7000U,30000U);
				}
			}
		}
		break;
		case 0x01u:// có hai khay sản phẩm được lấy vị trí 3 điểm là tray 1 và tray 2
		{
			if(Task_Scan_Tray(&time))
			{
				MC_MoveLinear(sample_x,sample_y,0x00U,10000u);
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
	Task_Gpio_input();
	if(home!=0x00u)
	{
		start_run=0x00U;
		state=0X00u;// trạng thái để đưa z xuống con hàng cần reset khi về home
		if(Move_Home_3Step(&home))
		{
			Reset_position();
			home=0x00U;
			Reset_Oxis();
		}
	}
}
void Choose_glass_group(uint8_t num){
	if(num == 0) return;
}
void Main_Reset(uint8_t data)
{

}
void Main_Start(uint8_t data)
{
	start_run=0x01U;
}
void Main_Stop(uint8_t data)
{
}
void Main_SetX(uint8_t data)
{

}
void Main_SetY(uint8_t data)
{

}
void Main_SetZ(uint8_t data)
{

}
void Main_Manual(uint8_t data)
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
void Pick_handle1(void)
{

}
void Release_handle1(void)
{

}
void Pick_handle2(void)
{

}
void Release_handle2(void)
{

}
void Save_1(void)
{
	switch(save)
	{
		case 0x01U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,0x06);
				// BẬT LED LAMP LÊN BẰNG SET BIT
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.ruber1_1=0x01U;
				save=0x00;
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x02U :
		{
			if(Motor_Busy()==0x00U)
			{
			Copy_Holding_Registers(0x00U,0x08);
			Save_Calibration_To_Flash(0x06U);
			Motor_Lamp->bits.ruber1_2=0x01U;
			save=0x00;
			Reset_Tray(0x03);
			}

		}
		break;
		case 0x03U :
		{
			if(Motor_Busy()==0x00U)
			{
			Copy_Holding_Registers(0x00U,0x0a);
			Save_Calibration_To_Flash(0x06U);
			Motor_Lamp->bits.ruber1_3=0x01U;
			save=0x00;
			Reset_Tray(0x03);
			}

		}
		break;
	}

}
void Save_2(void)
{
	switch(save)
	{
		case 0x04U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,12);
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray1_1=0x01U;
				save=0x00;
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x05U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,14);
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray1_2=0x01U;
				save=0x00;
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x06U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,16);
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray1_3=0x01U;
				save=0x00;
				Reset_Tray(0x03);
			}

		}
		break;
	}
}
void Save_3(void)
{
	switch(save)
	{
		case 0x07U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,18);
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray2_1=0x01U;
				Reset_Tray(0x03);
				save=0x00;
			}
		}
		break;
		case 0x08U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,20U);
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray2_2=0x01U;
				Reset_Tray(0x03);
				save=0x00;
			}

		}
		break;
		case 0x09U :
		{
			if(Motor_Busy()==0x00U)
			{
				Copy_Holding_Registers(0x00U,22U);
				if(Motor_Busy()==0x00U)
				Save_Calibration_To_Flash(0x06U);
				Motor_Lamp->bits.tray2_3=0x01U;
				Reset_Tray(0x04);
				save=0x00;
			}

		}
		break;
	}

}
void Setmove_point(void)
{
	switch(save)
	{
		case 0x01:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(6),Get_Holding_Registers(7),0x00U,20000U);
				Reset_Tray(0x03);
			}
		}
		break;
		case 0x02:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(8),Get_Holding_Registers(9),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x03:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(10),Get_Holding_Registers(11),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x04:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(12),Get_Holding_Registers(13),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x05:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(14),Get_Holding_Registers(15),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x06:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(16),Get_Holding_Registers(17),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x07:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(18),Get_Holding_Registers(19),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x08:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(20),Get_Holding_Registers(21),0x00U,20000U);
				Reset_Tray(0x03);
			}

		}
		break;
		case 0x09:
		{
			if(Motor_Busy()==0x00U)
			{
				MC_MoveLinear(Get_Holding_Registers(22),Get_Holding_Registers(23),0x00U,20000U);
				Reset_Tray(0x04);
			}

		}
		break;
		default:
		break;
	}
	Reset_Tray(0x02);
	save=0x00U;
	Motor_Lamp->bits.led_move=0x01U;

}
void Save_Glass_1(void)
{
	// Lưu giá trị glass_1 7 và 8
	save=0x01;
}
void Save_Glass_2(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến
	save=0x02;
}
void Save_Glass_3(void)
{
	// Lưu giá trị glass_3 vào flash và trên biến
	save=0x03;
}


void Save_Glass_All(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến

}
void Save_Cover_1(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến
	save=0x04;
}
void Save_Cover_2(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến
	save=0x05;
}
void Save_Cover_3(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến
	save=0x06;
}
void Save_Cover_All(void)
{
	// Lưu giá trị glass_2 vào flash và trên biến
}
void Save_Tray_1(void)
{
	save=0x07U;
}
void Save_Tray_2(void)
{

	save=0x08U;
}
void Save_Tray_3(void)
{

	save=0x09U;
}
void Handle_Set(void)
{
	MC_MoveLinear(Holding_Registers_Database[0],Holding_Registers_Database[1],Holding_Registers_Database[2],20000U);

}
void Handle_Home(void)
{
	if(home==0x00U)
	{
		home=0x01U;
	}
}
void  Reset(void)
{

}

