#include "cmsis_os.h"
#include "Chassis_Task.h"
#include "bsp_can.h"
#include "motor.h"
#include "pid.h"


PID_TypeDef_t Position_Pid,linespeed_Pid,Pitch_Pid[2],Yaw_Pid[2],Offset_Pid;

Chassis_Info_t Chassis_Ctrl=
{
		.ctrl = 1,
    .mode = CHASSIS_FOLO,
	  .txMsg=&CAN_TxMsg[_CAN2][_0x300],
    .dr16 = &rc_ctrl,
		.mode_Setup = chassis_Mode_Handoff,
};

Head head=
{
		.shoot_ready = Shoot_Off,
		.auto_ready=0,
	  .txMsg=&CAN_TxMsg[_CAN2][_0x1ff],
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
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间

    Chassis_ctrl();
		
		rc_ctrl_monitor(&rc_ctrl);

    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/**
	* @name		chassis_State_Handoff
  * @brief  底盘状态切换(在dbus任务中调用)
  * @param  mode
  * @retval NONE
  */
static void chassis_State_Handoff(DEVICE_STATE state)
{
   if(state!=Chassis_Ctrl.state)
	{
		Chassis_Ctrl.state=state;    
	}
}


/**
	* @name		chassis_Mode_Handoff
  * @brief  底盘模式切换(在dbus任务中调用)
  * @param  mode
  * @retval NONE
  */
static void chassis_Mode_Handoff(Chassis_Mode_e mode)
{
    if(mode!=Chassis_Ctrl.mode)
			Chassis_Ctrl.mode=mode;
}


/**
	* @name		Chassis_ctrl
  * @brief  控制底盘的运动
  * @param  mode
  * @retval NONE
  */
static void Chassis_ctrl(void)
{
	  static uint8_t chassis_mode_L = 1,chassis_mode_R = 0; //拨杆
		static uint8_t chassis_act_L = 0,chassis_act_R = 0;		//l底盘 r云台
		static uint8_t stuck_signal=0;
		static uint8_t unlimited_signal=0;

		if(Key_SHIFT() == true)
			Chassis_Ctrl.mode = CHASSIS_SPIN;
		else
			Chassis_Ctrl.mode = CHASSIS_INVA;
		
		if(Chassis_Ctrl.mode == CHASSIS_INVA)
		{
			chassis_act_L = 0;chassis_act_R = 1;
		}
		else if(Chassis_Ctrl.mode == CHASSIS_FOLO)
		{		
			chassis_act_L = 1;chassis_act_R = 0;
		}
		else if(Chassis_Ctrl.mode == CHASSIS_SPIN)
		{
			chassis_act_L = 1;chassis_act_R = 1;
		}
		
		if(Shoot.shoot_mode == Shoot_On)
			head.shoot_ready = Shoot_On;
		else if(Shoot.shoot_mode == Shoot_Off)
			head.shoot_ready = Shoot_Off;
		
		if(Vision_Flag == 1)
			head.auto_ready = 1;
		else if(Vision_Flag == 0)
			head.auto_ready = 0;
		
		if(DJI_Motor[Trigger].stalled == 0)
			stuck_signal = 0;
		else if(DJI_Motor[Trigger].stalled == 1)
			stuck_signal = 1;
		
    Chassis_Ctrl.vx=rc_ctrl.rc.ch[2];
    Chassis_Ctrl.vy=rc_ctrl.rc.ch[3];
    Chassis_Ctrl.key=rc_ctrl.key.v;
//发送装载
		Chassis_Ctrl.txMsg->data[0] = (uint8_t)(chassis_mode_L)<<7 | (uint8_t)(chassis_mode_R)<< 6 | (uint8_t)(chassis_act_L)<<5 | (uint8_t)chassis_act_R<<4
															| (uint8_t)(head.shoot_ready)<<3 | (uint8_t)(head.auto_ready) << 2 | (uint8_t)(stuck_signal)<<1;
		Chassis_Ctrl.txMsg->data[1] = (uint8_t)unlimited_signal;
    Chassis_Ctrl.txMsg->data[2] = (uint8_t)(Chassis_Ctrl.vx>>8);
    Chassis_Ctrl.txMsg->data[3] = (uint8_t)(Chassis_Ctrl.vx);
    Chassis_Ctrl.txMsg->data[4] = (uint8_t)(Chassis_Ctrl.vy>>8);
    Chassis_Ctrl.txMsg->data[5] = (uint8_t)(Chassis_Ctrl.vy);
    Chassis_Ctrl.txMsg->data[6] = (uint8_t)(Chassis_Ctrl.key>>8);
    Chassis_Ctrl.txMsg->data[7] = (uint8_t)(Chassis_Ctrl.key);
		
		head.txMsg->data[4] = (uint8_t)((int)((pit_motor_angle+16.30f)*100) >> 8);
		head.txMsg->data[5] = (uint8_t)((int)((pit_motor_angle+16.30f)*100));

}
