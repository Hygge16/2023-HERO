//
// Created by YanYuanbin on 22-10-12.
//

#include "cmsis_os.h"

#include "Chassis_Task.h"
#include "INS_Task.h"

#include "bsp_can.h"
#include "motor.h"

#include "pid.h"

PID_TypeDef_t Position_Pid,linespeed_Pid,Pitch_Pid[2],Yaw_Pid[2],Offset_Pid;

//deadband,MaxIntegral,Maxoutput,kp,ki,kd
float f_chassis_Pid_Para[7][PID_PARAMETER_CNT]={
		[0] = {0,0,350,2.2361f,0,20.f,},
		[1] = {0,200,700,-4.1247f,-0.05f,0,},
		[2] = {0,0,1000,-46.6112f,0,0,},
		[3] = {0,0,700,-5.0312f,0.f,0,},
		[4] = {0,0,50,3.1623f,0,4.f,},
		[5] = {0,50,200,5.0355f,0.f,0,},
		[6] = {0,200,700,1.f,0.05f,0,},
};

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the StartChassisTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
	
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}


