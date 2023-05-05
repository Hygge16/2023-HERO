//
// Created by WangJinJing
//
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stm32f4xx.h"
#include "bsp_rc.h"
#include "kalman.h"
#include "main.h"
#include "motor.h"

typedef enum
{
	CHASSIS_INVA=0x00U, //卸  力
	CHASSIS_FOLO=0X01U, //跟  随
	CHASSIS_SPIN=0X02U, //小陀螺
	CHASSIS_MODE_NUM,
}Chassis_Mode_e;


typedef enum
{
  RC_CTRL,     //遥控器模式
  KEY_CTRL,    //键盘模式
  CTRL_MODE_NUM,
}Chassis_Ctrl_Mode;

#include "chassis.h"

typedef struct
{
	int16_t CH2,CH3;
  Chassis_Mode_e mode;
  Chassis_Ctrl_Mode ctrl_mode;

	rc_ctrl_t *rc_ctrl;
	
	float vx,vy,vw;
	uint16_t midangle;
	extKalman_t KF_FOLO_Angle;
	float speed[SPEED_TYPE_NUM];

}Chassis_Info_t;

extern Chassis_Info_t Chassis_Ctrl;

void CHASSIS_Ctrl(void);
//void chassis_mode_set(Chassis_Info_t *Chassis_Info);
void Chassis_AcclerateCurve(float *speed_X,float *speed_Y);
static float AcclerateCurve(float x , float k);
static float DecclerateCurve(float x , float k);

void Chassis_Mode_Set(Chassis_Info_t *Chassis_Info);
void Chassis_Info_Update(Chassis_Info_t *Chassis_Info);
void Chassis_Reset(Chassis_Info_t *mode);
void Chassis_Test(void);
void Chassis_Rc_Ctrl(void);
void Chassis_Key_Ctrl(void);

#endif //CHASSIS_TASK_H
