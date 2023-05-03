//
// Created by Yuanbin on 22-10-3.
//

#include "robot_ref.h"

#include "cmsis_os.h"

#include "Gimbal_Task.h"
#include "INS_Task.h" 

#include "pid.h"
#include "motor.h"
#include "bsp_can.h"

//deadband, maxIntegral, max_out, kp, ki, kd
float f_gimbal_Pid_Para[2][2][PID_PARAMETER_CNT]={
	[0]={
		[0]={0.f,0.f,5000.f,44.f,0.f,0.f,},
		[1]={0.f,5000.f,30000.f,64.f,0.01f,0.f,},
	},
	[1]={
		[0]={0.f,0.f,5000.f,60.f,0.f,0.f,},
		[1]={0,5000,30000,80,0.12f,0.f,},
	},
};

//PID
PID_TypeDef_t Gimbal_PID[2][2];

//云台控制信息
Gimbal_Info_t Gimbal_Ctrl={
		.rc_ctrl = &rc_ctrl,
		.Measure.pit_angle = &INS_Info.pit_angle,
		.Measure.yaw_angle = &INS_Info.yaw_tolangle,
		.Measure.pit_gyro  = &INS_Info.pit_gyro,
		.Measure.yaw_gyro  = &INS_Info.yaw_gyro,
		.Limit_pitch.max = 30.f,
		.Limit_pitch.min = -24.f,
};

static void gimbal_init(void);
static void gimbal_posture_ctrl(Gimbal_Info_t *Gimbal_Info_control);
static void chassis_send_current(Gimbal_Info_t *Gimbal_Info_send,CAN_TxFrameTypeDef *TXFrame);

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the StartGinbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
	gimbal_init();
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/**
  * @brief          云台初始化
  * @param[out]     none
  * @retval         none
  */
static void gimbal_init(void)
{
	//PID Init
	PID_Init(&Gimbal_PID[Gimbal_Pitch][0],PID_POSITION,f_gimbal_Pid_Para[Gimbal_Pitch][0]);
	PID_Init(&Gimbal_PID[Gimbal_Pitch][1],PID_POSITION,f_gimbal_Pid_Para[Gimbal_Pitch][1]);
	PID_Init(&Gimbal_PID[Gimbal_Yaw][0]  ,PID_POSITION,f_gimbal_Pid_Para[Gimbal_Yaw][0]);
	PID_Init(&Gimbal_PID[Gimbal_Yaw][1]  ,PID_POSITION,f_gimbal_Pid_Para[Gimbal_Yaw][1]);
}


/**
  * @brief          云台姿态控制
  * @param[out]     Gimbal_Info_control:云台信息变量指针.
  * @retval         none
  */
static void gimbal_posture_ctrl(Gimbal_Info_t *Gimbal_Info_control)
{
		if(Gimbal_Info_control == NULL) return ;
		
		//更新期望角速度
		Gimbal_Info_control->Target.pit_gyro = f_PID_Calculate(&Gimbal_PID[Gimbal_Pitch][0],Gimbal_Info_control->Target.pit_angle,*Gimbal_Info_control->Measure.pit_angle);
		Gimbal_Info_control->Target.yaw_gyro = f_PID_Calculate(&Gimbal_PID[Gimbal_Yaw][0],  Gimbal_Info_control->Target.yaw_angle,*Gimbal_Info_control->Measure.yaw_angle);
		
		//更新控制电流值
		Gimbal_Info_control->SendValue[Gimbal_Pitch] = f_PID_Calculate(&Gimbal_PID[Gimbal_Pitch][1],Gimbal_Info_control->Target.pit_gyro,*Gimbal_Info_control->Measure.pit_gyro);
		Gimbal_Info_control->SendValue[Gimbal_Yaw] = f_PID_Calculate(&Gimbal_PID[Gimbal_Yaw][1],  Gimbal_Info_control->Target.yaw_gyro,*Gimbal_Info_control->Measure.yaw_gyro);
		
		//云台卸力
		if(rc_ctrl.rc.s[1]==1 || rc_ctrl.rc.s[1]==0)
		{
			Gimbal_Info_control->SendValue[Gimbal_Pitch] = 0;
			Gimbal_Info_control->SendValue[Gimbal_Yaw] = 0;
		}
}

/**
  * @brief          云台控制电流发送
	* @param[out]     Gimbal_Info_send:云台信息变量指针, TXFrame:CAN发送帧包变量指针
  * @retval         none
  */
static void chassis_send_current(Gimbal_Info_t *Gimbal_Info_send,CAN_TxFrameTypeDef *TXFrame)
{
	if(Gimbal_Info_send == NULL || TXFrame == NULL ) return;
	
	TXFrame[Gimbal_Pitch].data[2] = (uint8_t)(Gimbal_Info_send->SendValue[Gimbal_Pitch] >> 8);
	TXFrame[Gimbal_Pitch].data[3] = (uint8_t)(Gimbal_Info_send->SendValue[Gimbal_Pitch]);
	TXFrame[Gimbal_Yaw].data[0]   = (uint8_t)(Gimbal_Info_send->SendValue[Gimbal_Yaw] >> 8);
	TXFrame[Gimbal_Yaw].data[1]   = (uint8_t)(Gimbal_Info_send->SendValue[Gimbal_Yaw]);
	
	//CAN_Transmit
	USER_CAN_TxMessage(&TXFrame[Gimbal_Pitch]);
	USER_CAN_TxMessage(&TXFrame[Gimbal_Yaw]);
}
