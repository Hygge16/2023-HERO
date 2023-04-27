//
// Created by YanYuanbin on 22-10-2.
//
#ifndef ROBOT_REF_H
#define ROBOT_REF_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* define the robot type */
#define BalanceInfantry

/* define development board type */

#define CHASSIS_BOARD //chassis type

//#define GIMBAL_BOARD //gimbal type

//failed define development board type
#if !(defined(CHASSIS_BOARD) ||  defined(GIMBAL_BOARD))
#error NULL board definition! Please define one board type at least.
#endif

//multi define development board type
#if (defined(CHASSIS_BOARD) &&  defined(GIMBAL_BOARD)) 
#error Conflict board definition! You can only define one board type.
#endif



#endif //ROBOT_REF_H
