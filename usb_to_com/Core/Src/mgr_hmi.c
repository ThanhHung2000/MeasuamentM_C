/*
 * mgr_hmi.c
 *
 *  Created on: Jan 25, 2026
 *      Author: Admin
 */
#include "motor.h"
#include "RS232.h"
#include "main.h"
#include "delay.h"
#include  "Common.h"
#include "mgr_hmi.h"
#include "drive.h"
uint8_t Emergency = 0;
uint8_t Glass_group = 0;


void Handle_Set(void);

void Handle_Left(void);
void Handle_Right(void);
void Handle_In(void);
void Handle_Out(void);
void Handle_Up(void);
void Handle_Down(void);

void Save_Glass_1(void);
void Save_Glass_2(void);
void Save_Glass_3(void);
void Save_Glass_All(void);
void Save_Cover_1(void);
void Save_Cover_2(void);
void Save_Cover_3(void);
void Save_Cover_All(void);


void Reset(void);

typedef void (*ActionHandler_t)(void);
typedef struct {
    uint16_t bitMask;
    ActionHandler_t handler;
} Map_t;

Map_t motorMoveTable[16] = {
    { 1 << 0, Handle_Left  },
    { 1 << 1, Handle_Right },
    { 1 << 2, Handle_In    },
    { 1 << 3, Handle_Out   },
    { 1 << 4, Handle_Up    },
	{ 1 << 5, Handle_Down  },
	{ 1 << 6, Handle_Set   },
	{ 1 << 7, Handle_Home  },

	{ 1 << 8, Save_Glass_1  },
	{ 1 << 9, Save_Glass_2  },
	{ 1 << 10, Save_Glass_3 },
	{ 1 << 11, Save_Cover_1  },
	{ 1 << 12, Save_Cover_2  },
	{ 1 << 13, Save_Cover_3  },
	{ 1 << 14, Save_Glass_All  },
	{ 1 << 15, Save_Cover_All  },
};

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
void Task_scan_HMI(void)
{
	if(Emergency!=0) return;
	if(Tab->bits.Home == 1 && Home_Lamp->bits.Run == 0)
	{

	}
	else if (Tab->bits.Engine == 1)
	{
		uint16_t current = ((Save_Tray -> all) << 8) | (Control_motor->all);
		int8_t fisrtbit = __builtin_ffs(current)-1;
		if(fisrtbit >= 0)
		{
			motorMoveTable[fisrtbit].handler();
		}
	}
}
void Choose_glass_group(uint8_t num){
	if(num == 0) return;
//	Home_Lamp-> all &= ~(0x0fu << 2);
//	Home_Lamp-> all |= (num << 2);
//	Glass_group = __builtin_ffs(num)-1;
//	ClearBit(Lamp_glass_empty, 0, 196);
//	*Lamp_glass_select = 0;
//	for(uint16_t i = 0; i<196;i++){
//		if(Glass[Glass_group*196+i].State != OK){
//			SetBit(Lamp_glass_empty, i, 1);
//		}
//	}
}

void Handle_Left(void)
{

}
void Handle_Right(void)
{

}
void Handle_In(void)
{

}
void Handle_Out(void)
{

}
void Handle_Up(void)
{

}
void Handle_Down(void)
{

}

void Save_Glass_1(void)
{

}
void Save_Glass_2(void)
{

}
void Save_Glass_3(void)
{

}
void Save_Glass_All(void)
{

}
void Save_Cover_1(void)
{

}
void Save_Cover_2(void)
{

}
void Save_Cover_3(void)
{

}
void Save_Cover_All(void)
{

}
void Handle_Set(void)
{
	MC_MoveLinear(Holding_Registers_Database[0],Holding_Registers_Database[1],Holding_Registers_Database[2],10000);
}
void Handle_Home(void)
{

}
