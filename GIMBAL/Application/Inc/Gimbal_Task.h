#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "motor.h"
#include "bsp_rc.h"
#include "SystemState_Task.h"
#include "bmi088_driver.h"
#include "Vision_Task.h"

typedef struct 
{
    struct{
        float pit_Angle;
        float yaw_Angle;
				float pit_hang_Angle;
				float yaw_hang_Angle;
    }Target;
	
		DEVICE_STATE state;
		bmi088_real_data_t *IMU;
    Rc_Ctrl_t *dr16;
		VISION *vision;
		DJI_Motor_Info_t *yaw, *pitch;
		
		void (*state_Setup)(DEVICE_STATE);
		
}Gimbal_Info_t;

extern Gimbal_Info_t Gimbal_Ctrl;

extern int Vision_Flag;
extern float pit_motor_angle;

static void gimbal_State_Handoff(DEVICE_STATE mode);
static void Gimbal_Posture_Ctrl(void);

#endif //GIMBAL_TASK_H

