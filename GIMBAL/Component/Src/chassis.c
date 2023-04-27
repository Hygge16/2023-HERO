//
// Created by Yuanbin on 22-10-3.
//

#include "robot_ref.h"
#include "chassis.h"

#if defined(CHASSIS_BOARD)

#include "Chassis_Task.h"

#include "motor.h"
#include "assist.h"
#include "pid.h"

#define Euler_Number 2.718281828459045f

//以队列的形式求差分
QueueDiff_t Wheelspeed_Diff[2] = {
	[Left_Wheel] = {
		.queueLength = 10,
	},
	[Right_Wheel] = {
		.queueLength = 10,
	},
};


static float ramp_float( float final, float now, float ramp );
static float speed_control(float speed_in,Chassis_Info_t *Chassis_Info);

/**
  * @brief          底盘模式设置
  * @param[out]     Chassis_Info:底盘信息变量指针.
  * @retval         none
  */
void chassis_mode_set(Chassis_Info_t *Chassis_Info)
{
		if(Chassis_Info == NULL) return;
		
		//右拨杆位于中位，则为正面对敌模式
		if(Chassis_Info->rc_ctrl->rc.s[0] == 3)
		{
				Chassis_Info->mode = CHASSIS_FRONT;
		}
		//右拨杆位于低位，则为飞坡模式
		else if(Chassis_Info->rc_ctrl->rc.s[0] == 2)
		{
				Chassis_Info->mode = CHASSIS_FLY;
		}
		//右拨杆位于高位或离线，则为无力模式
		else
		{
				Chassis_Info->mode = CHASSIS_WEEK;
		}
		
		//拨轮位于低位，且底盘非无力模式，则切换为小陀螺模式
		if((true == Chassis_Info->IF_SPIN_ENABLE || Key_SHIFT() == true) && (Chassis_Info->mode != CHASSIS_WEEK || CHASSIS_FLY != Chassis_Info->mode))
		{
				Chassis_Info->mode = CHASSIS_SPIN;
		}
}

/**
  * @brief          底盘信息更新
  * @param[out]     Chassis_Info_update:底盘信息变量指针.
  * @retval         none
  */
void Chassis_Info_update(Chassis_Info_t *Chassis_Info)
{
		if(Chassis_Info == NULL) return;
	
		//更新底盘航向角
		Chassis_Info->Measure.yaw_angle = f_encoder_to_angle(&YawMotor.Data.encoder,8192.f);
		Chassis_Info->Measure.yaw_gyro  = (float)YawMotor.Data.velocity*DJIGM6020_RPM_TO_OMEGA;
		Chassis_Info->Measure.yaw_err   = f_angle_conversion(Chassis_Info->Target.yaw_angle[Chassis_Info->IF_YAW_ANGLE_OFFSET],Chassis_Info->Measure.yaw_angle,180);

		if(CHASSIS_WEEK != Chassis_Info->mode)
		{
			Chassis_Info->Target.linespeed = fabs(arm_cos_f32(Chassis_Info->Measure.yaw_err/180.f*PI))*speed_control(-Chassis_Info->rc_ctrl->rc.ch[3]*0.4f-(Key_W()-Key_S())*Chassis_Info->Max.linespeed,Chassis_Info);
			VAL_LIMIT(Chassis_Info->Target.linespeed,-Chassis_Info->Max.linespeed,Chassis_Info->Max.linespeed);
		}
		//小陀螺模式，偏置期望角
		if(Chassis_Info->mode == CHASSIS_FLY)
		{
				Chassis_Info->IF_PIT_ANGLE_OFFSET = true;
		}else{
				Chassis_Info->IF_YAW_ANGLE_OFFSET = false;
				Chassis_Info->IF_PIT_ANGLE_OFFSET = false;
		}
		
		//更新小陀螺转速，暂为定值，后续优化
		if(Chassis_Info->mode == CHASSIS_SPIN && Chassis_Info->Target.yaw_gyro == 0) 
		{
			Chassis_Info->Target.yaw_gyro = -60;
		}
		
		//驱动轮转速(DPS)转化成底盘线速度(cm/s)
		Chassis_Info->Measure.linespeed = (float)(-MT9025[Left_Wheel].Data.velocity + MT9025[Right_Wheel].Data.velocity)/2.f*MOTOR_RPM_TO_LINE_SPEED*100.f;
		
		//以差分形式获取驱动轮加速度(cm/s^2)
		Chassis_Info->Measure.Wheelaccel[Left_Wheel] = f_get_diff(2,&Wheelspeed_Diff[Left_Wheel], -MT9025[Left_Wheel].Data.velocity * MOTOR_RPM_TO_LINE_SPEED*100.f)*1000;
		Chassis_Info->Measure.Wheelaccel[Right_Wheel]= f_get_diff(2,&Wheelspeed_Diff[Right_Wheel], MT9025[Right_Wheel].Data.velocity* MOTOR_RPM_TO_LINE_SPEED*100.f)*1000;

		if(Chassis_Info->Target.linespeed <= 1)
		{
				//底盘线速度积分位移(cm)
				Chassis_Info->Measure.position -= Chassis_Info->Measure.position/1000.f;
				//底盘位移限幅
				VAL_LIMIT(Chassis_Info->Measure.position,-Chassis_Info->Max.position,Chassis_Info->Max.position);	
		}else
		{
				Chassis_Info->Measure.position = 0;
		}
}

///**
//*	@brief	速度控制
//*/
//static float speed_control(Chassis_Info_t *Chassis_Info)
//{
//	
//	if(Chassis_Info->rc_ctrl->rc.ch[3] != 0)
//	{
//		Chassis_Info->Target.linespeed = ramp_float(-Chassis_Info->rc_ctrl->rc.ch[3]*0.4f,Chassis_Info->Target.linespeed,0.8f);
//	}
//	else if(Key_W()!=false || Key_S()!=false)
//	{
//		Chassis_Info->Target.linespeed = ramp_float(-(Key_W()-Key_S())*Chassis_Info->Max.linespeed,Chassis_Info->Target.linespeed,0.08f);
//	}else
//	{
//		Chassis_Info->Target.linespeed = ramp_float(0,Chassis_Info->Target.linespeed,0.08f);
//	}
//	
//	return Chassis_Info->Target.linespeed;
//}

/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
	
static float speed_control(float speed_in,Chassis_Info_t *Chassis_Info)
{
	float accel_K1 = 0.4;
	float accel_K2 = 0.8;
	
	if (speed_in >= 0)
	{
		float accel_1 = accel_K1*(speed_in - Chassis_Info->Target.linespeed);
		float accel_2 = accel_K2*(speed_in - Chassis_Info->Target.linespeed);
		
		if (Chassis_Info->Target.linespeed >= 0)
			return Chassis_Info->Target.linespeed + accel_1;
		else if (Chassis_Info->Target.linespeed < 0)
			return Chassis_Info->Target.linespeed + accel_2;
	}
	else if (speed_in < 0)
	{
		float accel_1 = -accel_K1*(speed_in - Chassis_Info->Target.linespeed);
		float accel_2 = -accel_K2*(speed_in - Chassis_Info->Target.linespeed);
		
		if (Chassis_Info->Target.linespeed < 0)
			return Chassis_Info->Target.linespeed - accel_1;
		else if (Chassis_Info->Target.linespeed >= 0)
			return Chassis_Info->Target.linespeed - accel_2;
	}
	return Chassis_Info->Target.linespeed;
}


/**
 *	@brief	斜坡函数
 */
static float ramp_float( float final, float now, float ramp )
{
	float buffer = 0.f;

	buffer = final - now;

	if (buffer > 0){
		if (buffer > ramp){  
			now += ramp;
		}else{
			now += buffer;
		}
	}else{
		if (buffer < -ramp){
			now -= ramp;
		}else{
			now -= buffer;
		}
	}
	return now;
}

/**
 * @description: 公式原型 y = 1/(1+e^(-k*(x-x0)))
 */
float AcclerateCurve(float x , float k ,float x0)
{
	float y = 0;
	
	if(k == 0)return 1;
	
	y = 1/(1+pow(Euler_Number,(-k*(x-x0))));
	
	return y;
}

/**
  * @brief          遥控器数据接收
	* @param[out]     canId:CAN通信ID变量指针, rxBuf:接收数据帧变量指针,rc_ctrl:遥控器信息变量指针
  * @retval         none
  */
void get_rc_ctrl_data(uint32_t *canId, uint8_t *rxBuf,rc_ctrl_t *rc_ctrl)
{
	if(*canId != 0x302 || rc_ctrl == NULL) return;
	
	rc_ctrl->rc.s[0]  = (rxBuf[0] & 0xC0U) >> 6;
	rc_ctrl->rc.s[1]  = (rxBuf[0] & 0x30U) >> 4;
	rc_ctrl->rc_lost     = (rxBuf[0] & 0x08U) >> 3;
	Chassis_Ctrl.IF_SPIN_ENABLE = (rxBuf[0] & 0x04U) >> 2;
	
	rc_ctrl->rc.ch[3] = (int16_t)rxBuf[2] << 8 | rxBuf[3];
	rc_ctrl->rc.ch[2] = (int16_t)rxBuf[4] << 8 | rxBuf[5];
	rc_ctrl->key.v    = (int16_t)rxBuf[6] << 8 | rxBuf[7];
}

#endif
