#include "robot_ref.h"
#include "cmsis_os.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "bsp_can.h"
#include "kalman.h"
#include "Vision_Task.h"

int Vision_Flag;
float pit_motor_angle;
float ky=1.f,kp=1.f;
float pit_add;

int16_t SendValue[2];

mpu_data_t KF_Gimbal_Pitch;
mpu_data_t KF_Gimbal_Yaw;

Gimbal_Info_t Gimbal_Ctrl=
{
	.dr16 = &rc_ctrl,
	.vision = &vision,
	.IMU = &imu,
  .yaw = &DJI_Motor[Gimbal_Yaw],
  .pitch = &DJI_Motor[Gimbal_Pitch],
};

//deadband, maxIntegral, max_out, kp, ki, kd
float f_yaw_Pid_Param[2][PID_PARAMETER_CNT]=
{
	[0]={0.f, 2000.f, 32000.f, 150.f, 0.1f, 0,},
	[1]={0.f, 2000.f, 1200.f, 60.f, 0.f, 0,},
};

float f_pitch_Pid_Param[2][PID_PARAMETER_CNT]=
{
	[0]={0.f, 20000.f, 27200.f, 60.f, 0.f, 0.f,},
	[1]={0.f, 0.f, 1000.f, -56.f, 0.f, 0.5f,},
};

PID_TypeDef_t Gimbal_PID[2][2];

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
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间
		
		if(DJI_Motor[Gimbal_Pitch].Data.angle > 200.f)
			pit_motor_angle=(DJI_Motor[Gimbal_Pitch].Data.angle-360.f);
		if(DJI_Motor[Gimbal_Pitch].Data.angle < 100.f)
			pit_motor_angle=(DJI_Motor[Gimbal_Pitch].Data.angle);
		
		Key_Q();
		Key_E();

		Gimbal_Posture_Ctrl();
		
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/**
	* @name		
  * @brief  none        
  * @param	none
  * @retval none
  */
static void gimbal_State_Handoff(DEVICE_STATE state)
{
    if(state!=Gimbal_Ctrl.state)
		{
			  Gimbal_Ctrl.state=state;
        PID_Init_ByParamArray(&Gimbal_Ctrl.yaw->pid_Angle, &f_yaw_Pid_Param[state][1]);
        PID_Init_ByParamArray(&Gimbal_Ctrl.pitch->pid_Angle, &f_pitch_Pid_Param[state][1]);
        PID_Init_ByParamArray(&Gimbal_Ctrl.yaw->pid_Speed, &f_yaw_Pid_Param[state][0]);
        PID_Init_ByParamArray(&Gimbal_Ctrl.pitch->pid_Speed, &f_pitch_Pid_Param[state][0]);
    }
}

/**
	* @name		Gimbal_Posture_Ctrl
  * @brief  none        
  * @param	none
  * @retval none
  */
static void Gimbal_Posture_Ctrl(void)
{
	float angle_a;
  float angle_Err[2],speed_Err[2];
  int16_t Send_Value[2];
	
	VAL_LIMIT(Gimbal_Ctrl.Target.yaw_hang_Angle,-16.f,48.f);
	
	angle_Err[0] = Gimbal_Ctrl.Target.yaw_Angle - Gimbal_Ctrl.IMU->accel[0];
  angle_Err[1] = Gimbal_Ctrl.Target.pit_Angle - Gimbal_Ctrl.IMU->accel[1];
	
	if((rc_ctrl.rc.s[0]==2 && rc_ctrl.rc.s[1]==1)==1 || Key_mouse_r()==true)//遥控拨杆自瞄（左下右下）
	{
		Vision_Flag=1;
	}
	else
	{
		Vision_Flag=0;
	}
   
	if((Vision_Flag == 1) && Gimbal_Ctrl.vision->tx2->isFind == 1)
	{
			if(key_KF)
			{
				angle_Err[0] = Gimbal_Ctrl.vision->yaw * ky;
        angle_Err[1] = Gimbal_Ctrl.vision->tx2->pit_Err[0] * kp;
      }
      else
			{
        angle_Err[0] = Gimbal_Ctrl.vision->tx2->yaw_Err[0] * ky;
        angle_Err[1] = Gimbal_Ctrl.vision->tx2->pit_Err[0] * kp;
      }
			
        VAL_LIMIT(angle_Err[0],-1.5f,1.5f);
        VAL_LIMIT(angle_Err[1],-3.f,3.f);
        Gimbal_Ctrl.Target.yaw_Angle = imu.accel[0];
        Gimbal_Ctrl.Target.pit_Angle = imu.accel[1];
  }

		pit_add = Gimbal_Ctrl.IMU->accel[1] * 28.904f - 1898.9f;
		VAL_LIMIT(pit_add, -2500, -500);

		f_PID_Calculate(&Gimbal_Ctrl.yaw->pid_Angle,angle_Err[0],0);
		f_PID_Calculate(&Gimbal_Ctrl.pitch->pid_Angle,angle_Err[1],0);
		
    speed_Err[0] = Gimbal_Ctrl.yaw->pid_Angle.Err[0] - Gimbal_Ctrl.IMU->gyro[0];
	
		f_PID_Calculate(&Gimbal_Ctrl.yaw->pid_Speed, speed_Err[0],0);
		f_PID_Calculate(&Gimbal_Ctrl.pitch->pid_Speed, speed_Err[1],0);
	
    speed_Err[1] = Gimbal_Ctrl.pitch->pid_Angle.Err[1] - Gimbal_Ctrl.IMU->gyro[1];
		speed_Err[1] = KalmanFilter(&KF_Gimbal_Pitch, speed_Err[1]);
		
    Send_Value[0] = (int16_t)Gimbal_Ctrl.yaw->pid_Speed.Err[0];
    Send_Value[1] = (int16_t)Gimbal_Ctrl.pitch->pid_Speed.Err[1];

    Gimbal_Ctrl.yaw->txMsg->data[L[Gimbal_Ctrl.yaw->Data.StdId-0x205]] = (uint8_t)(Send_Value[0]>>8);
    Gimbal_Ctrl.yaw->txMsg->data[H[Gimbal_Ctrl.yaw->Data.StdId-0x205]] = (uint8_t)(Send_Value[0]);
    Gimbal_Ctrl.pitch->txMsg->data[L[Gimbal_Ctrl.pitch->Data.StdId-0x205]] = (uint8_t)(Send_Value[1]>>8);
    Gimbal_Ctrl.pitch->txMsg->data[H[Gimbal_Ctrl.pitch->Data.StdId-0x205]] = (uint8_t)(Send_Value[1]);
}
