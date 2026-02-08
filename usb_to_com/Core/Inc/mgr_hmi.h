/*
 * mgr_hmi.h
 *
 *  Created on: Jan 25, 2026
 *      Author: Admin
 */

#ifndef INC_MGR_HMI_H_
#define INC_MGR_HMI_H_
#define STATUS_IDLE_OXIS     0x00U
#define STATUS_JOGGING_OXIS  0x01U
#define STATUS_STEP_OXIS     0x02U
#define STATUS_STOP_OXIS     0x03U
#define NUM_BUTTON_HOLD      0x06U
#define NUM_BUTTON_MAIN      0x07U
void Task_Run_HMI(void);
void Task_Run_Home(void);
void Init_hmi(void);
void Set_Emergency_Stop();


uint8_t Get_home_done(void);
uint8_t Get_Go_home(void);
extern volatile uint8_t home;
#endif /* INC_MGR_HMI_H_ */
