//
// Created by WangJinJing
//
#include "cmsis_os.h"

#include "Chassis_Task.h"

#include "assist.h"
#include "pid.h"

#include "motor.h"

PID_TypeDef_t Chassis_Speed[4],Chassis_Folo[2];

//Deadband  IntegralMAX  OutputMAX  P  I  D 
float f_PID_Wheel_Para[PID_PARAMETER_CNT]=
{0,0,16000,7.f,0.f,0.f,};

float f_PID_Folo_Para[2][PID_PARAMETER_CNT]=
{ 
	[0]={0,0,500.f,0.8f,0.f,0.f,},
	[1]={0,1000.f,1200.f,3.8f,0.f,0.f,},
};
	
Chassis_Info_t Chassis_Ctrl={
		.mode = CHASSIS_INVA,
		.midangle = 809,
};

/* USER CODE BEGIN Header_ChassisTask */
/**
  * @brief  Function implementing the ChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisTask*/
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisTask */
  /* Infinite loop */
  for(;;)
  {
		
		
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

void chassis_mode_set(Chassis_Info_t *Chassis_Info)
{
	if(Chassis_Info->rc_ctrl->rc.s[1] == 1)
		Chassis_Info->mode = CHASSIS_INVA;
	else if(Chassis_Info->rc_ctrl->rc.s[1] == 3)
		Chassis_Info->mode = CHASSIS_FOLO;
	else if(Chassis_Info->rc_ctrl->rc.s[1] == 2)
		Chassis_Info->mode = CHASSIS_SPIN;
	else
		Chassis_Info->mode = CHASSIS_INVA;
	
	if(Key_SHIFT() == true && Chassis_Info->mode != CHASSIS_INVA) 
		Chassis_Info->mode = CHASSIS_SPIN;
}


void chassis_info_update(Chassis_Info_t *Chassis_Info)
{
	Chassis_Info->vx = Chassis_Info->rc_ctrl->rc.ch[2]+(Key_W()-Key_S())*660;
	Chassis_Info->vy = Chassis_Info->rc_ctrl->rc.ch[3]+(Key_A()-Key_D())*660;
	if(Chassis_Info->mode==CHASSIS_INVA)
		Chassis_Info->vw = 0;
	else if(Chassis_Info->mode==CHASSIS_FOLO)
	{
		f_PID_Calculate(&Chassis_Folo[0],f_angle_conversion(Chassis_Info->midangle ,DJI_Motor[Gimbal_Yaw].Data.encoder,4096),0);
		Chassis_Info->vw = f_PID_Calculate(&Chassis_Folo[1],Chassis_Folo[0].Output,DJI_Motor[Gimbal_Yaw].Data.velocity);
	}
	else if(Chassis_Info->mode == CHASSIS_SPIN)
		Chassis_Info->vw = f_PID_Calculate(&Chassis_Folo[1],500,DJI_Motor[Gimbal_Yaw].Data.velocity);
	
}
