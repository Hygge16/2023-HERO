//
// Created by WangJinJing
//
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "stm32f4xx.h"
#include "bsp_rc.h"

typedef enum
{
	CHASSIS_INVA=0x00U, //Ð¶Á¦
	CHASSIS_FOLO=0X01U, //¸úËæ
	CHASSIS_SPIN=0X02U, //Ð¡ÍÓÂÝ
	CHASSIS_MODE_NUM,
}Chassis_Mode_e;

typedef struct
{
	Chassis_Mode_e mode;
	
	rc_ctrl_t *rc_ctrl;
	
	float vx,vy,vw;
	uint16_t midangle;

}Chassis_Info_t;



#endif //CHASSIS_TASK_H
