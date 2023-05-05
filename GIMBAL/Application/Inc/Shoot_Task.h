//
// Created by Yuanbin on 22-10-3.
//

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "stdint.h"
#include "bsp_rc.h"
#include "motor.h"
#include "SystemState_Task.h"

typedef enum{
    Shoot_Off,
		Shoot_On,
    SHOOT_MODE_NUM,
}Shoot_Mode_e;


typedef struct 
{
    Shoot_Mode_e shoot_mode;
	
    uint8_t trigger_Buf;
	
		int8_t stuck_flag;
	
    int bulletNum;//·¢Éä¼ÆÊý
	
    float trigger_Angle;
	
	  DJI_Motor_Info_t *wheel_L,*wheel_R,*trigger;
	
		Rc_Ctrl_t *dr16;
	
	 void (*state_Setup)(DEVICE_STATE);
		DEVICE_STATE state;
	
}Shoot_Info_t;

/* Exported functions --------------------------------------------------------*/
static void Wheel_Ctrl(void);
static float Temp_BURST_Fix(void);
static float Temp_RATE_Fix(void);
static float SpeedAdapt_10M(float real_S , float min_S, float max_S,float up_num , float down_num);
static float SpeedAdapt_16M(float real_S , float min_S, float max_S,float up_num , float down_num);
static void trigger_Stall_Handle(void);
static void Trigger_Ctrl(void);

extern Shoot_Info_t Shoot;
extern float Temp_Fix_speed(uint16_t real_speed);
extern float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num);
extern bool Judge_IF_SingeStuck(void);
extern bool Judge_IF_AutoBlock(float speed_err);

#endif //SHOOT_TASK_H
