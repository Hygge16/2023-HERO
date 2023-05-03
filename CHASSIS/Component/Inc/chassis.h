#ifndef CHASSIS_H
#define CHASSIS_H

#include "stm32f4xx.h"
#include "motor.h"
#include "filter.h"
#include "stdbool.h"
#include "kalman.h"
#include "main.h"

#define NATURAL_NUMBER  2.718281828f    //自然常数e

#define M_LW 	(32.f)
#define M_L 	(32.f) 
#define M_R 	(7.5f)	

typedef enum
{
    _X,
    _Y,
    _W,
    SPEED_TYPE_NUM,
}Chassis_Speed_Type;

#include "Chassis_Task.h"

extern int16_t CAN1_SendData[4];
extern float Expect[4];

float Speed_X_deliver(float speed_X , float speed_Y);
float speed_Y_deliver(float speed_X , float speed_Y);
float Speed_W_deliver(Chassis_Mode_e mode);
float KEY_X_Ctrl(void);
float KEY_Y_Ctrl(void);
void Chassis_Power_Limit(float* output);
void control_switch();
void CHASSIS_Handler(float speed[SPEED_TYPE_NUM]);
void CHASSIS_Stop(void);
float RAMP_float( float final, float now, float ramp);
void Chassis_Set_Pid(Chassis_Ctrl_Mode ctrl_mode);

#endif //CHASSIS_H
