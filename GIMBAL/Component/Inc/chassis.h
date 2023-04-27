//
// Created by Yuanbin on 22-10-3.
//

#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdint.h"
#include "stdbool.h"
#include "robot_ref.h"

#include "bsp_rc.h"

#ifdef BalanceInfantry

#include "arm_math.h"

/* 驱动轮半径 */
#define Wheel_Radius  0.125f

/* 
 * 转子转速(dps)转化成底盘线速度(m/s)的比例
 * 计算公式:2*pi*m_r/(360.f*k) k为减速比
 */
#if (defined(Wheel_Radius) && defined(PI))
	#define MOTOR_RPM_TO_LINE_SPEED   PI*Wheel_Radius/180.f
#endif

/* 
 * 转子转速(rpm)转化成输出轴角速度(°/s)的比例
 * 计算公式:360/(60*2*pi*k) k为减速比
 */
#ifdef PI
	#define DJIGM6020_RPM_TO_OMEGA   3.f/PI
#endif

#endif


//底盘模式枚举
typedef enum
{
   CHASSIS_WEEK = 0x00U,  //无力
	 CHASSIS_FRONT= 0x01U,  //正面对敌
	 CHASSIS_FLY = 0x02U,  //飞坡
	 CHASSIS_SPIN = 0x03U,  //小陀螺
}Chassis_Mode_e;

typedef struct
{
//底盘模式
	Chassis_Mode_e mode;
	
	bool IF_SPIN_ENABLE;
	
//是否偏置期望角
	bool IF_YAW_ANGLE_OFFSET;
	bool IF_PIT_ANGLE_OFFSET;

//底盘期望值
	struct{
		float yaw_angle[2];
		float yaw_gyro;
		float pit_angle[2];
		float pit_gyro;
		float linespeed;
		float position;
	}Target;
	
//底盘状态观测量
	struct{
		float yaw_angle;
		float yaw_gyro;
		float yaw_err;
		float *pit_angle;
		float *pit_gyro;
		float *rol_gyro;
		float Wheelaccel[2];
		float linespeed;
		float position;
	}Measure;
	//数据限幅
	struct{
		float linespeed;
		float position;
	}Max;
	
	//遥控信息
	rc_ctrl_t *rc_ctrl;
	
	int16_t SendValue[2];

}Chassis_Info_t;

/* Exported functions --------------------------------------------------------*/
extern void chassis_mode_set(Chassis_Info_t *Chassis_Info);
extern void Chassis_Info_update(Chassis_Info_t *Chassis_Info);
extern float AcclerateCurve(float x , float k ,float x0);
extern void get_rc_ctrl_data(uint32_t *canId, uint8_t *rxBuf,rc_ctrl_t *rc_ctrl);

#endif //CHASSIS_H
