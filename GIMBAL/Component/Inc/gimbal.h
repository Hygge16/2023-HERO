//
// Created by Yuanbin on 22-10-3.
//

#ifndef GIMBAL_H
#define GIMBAL_H

#include "stdint.h"
#include "bsp_rc.h"
#include "bsp_can.h"

//云台模式枚举
typedef enum
{
   Gimbal_WEEK = 0x00U,  //无力
	 Gimbal_INS = 0x01U, //imu模式
	 Gimbal_Rcshoot = 0x02U, //遥控器击发
}Gimbal_Mode_e;

typedef struct _Gimbal_Info
{
	//遥控信息
	rc_ctrl_t *rc_ctrl;
	
	Gimbal_Mode_e mode;
	
	//云台状态期望值
	struct{
		float pit_angle;
		float yaw_angle;
		float pit_gyro;
		float yaw_gyro;
	}Target;
	
	//云台状态观测量
	struct{
		float *pit_angle;
		float *yaw_angle;
		float *pit_gyro;
		float *yaw_gyro;
	}Measure;
	
	//pitch角限幅
	struct{
		float min;
		float max;
	}Limit_pitch;
	
	//控制电流发送
	int16_t SendValue[2];

}Gimbal_Info_t;

/* Exported functions --------------------------------------------------------*/
extern void gimbal_mode_set(Gimbal_Info_t *Gimbal_Info);
extern void gimbal_Info_update(Gimbal_Info_t *Gimbal_Info);
extern void rc_communicate(rc_ctrl_t *rc_ctrl,CAN_TxFrameTypeDef *TxHeader);
#endif //GIMBAL_H
