//
// Created by YanYuanbin on 22-10-12.
//

#include "robot_ref.h"

#if defined(CHASSIS_BOARD)

#include "cmsis_os.h"

#include "Chassis_Task.h"
#include "INS_Task.h"

#include "referee_info.h"
#include "bsp_can.h"
#include "motor.h"

#include "pid.h"

//PID
PID_TypeDef_t Position_Pid,linespeed_Pid,Pitch_Pid[2],Yaw_Pid[2],Offset_Pid;

/*
   -2.2361   -4.1247   46.6112    5.0312   -3.1623   -5.0355
   -2.2361   -4.1247   46.6112    5.0312    3.1623    5.0355
	 */
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

//���̿�����Ϣ
Chassis_Info_t Chassis_Ctrl={
		.rc_ctrl = &rc_ctrl,
		.Target.pit_angle[0] = -1.5f,
		.Target.pit_angle[1] = 20.f,
		.Target.yaw_angle[0] = 270.61f,
		.Target.yaw_angle[1] = 0.61f,
		.Measure.pit_angle = &INS_Info.pit_angle,
		.Measure.pit_gyro  = &INS_Info.pit_gyro,
		.Measure.rol_gyro  = &INS_Info.rol_gyro,
		.Max.linespeed = 300.f,//cm/s
		.Max.position = 100.f,//cm
};

static void chassis_init(void);
static void chassis_posture_control(Chassis_Info_t *Chassis_Info);	
static void chassis_send_current(Chassis_Info_t *Chassis_Info_send,CAN_TxFrameTypeDef *TXFrame);
static float CHAS_Power_Control(float target_linespeed);

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
	chassis_init();
  /* Infinite loop */
  for(;;)
  {
		//����ģʽ����
		chassis_mode_set(&Chassis_Ctrl);
		
		//������Ϣ����
		Chassis_Info_update(&Chassis_Ctrl);
		
		//������̬����
		chassis_posture_control(&Chassis_Ctrl);
		
		//���̿��Ƶ�������
		chassis_send_current(&Chassis_Ctrl,MT_hcanTxFrame);

    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/**
  * @brief          ���̳�ʼ��
  * @param[out]     none
  * @retval         none
  */
static void chassis_init(void)
{		
	//PID init
	PID_Init(&Position_Pid, PID_POSITION,f_chassis_Pid_Para[0]);
	PID_Init(&linespeed_Pid,PID_POSITION,f_chassis_Pid_Para[1]);
	PID_Init(&Pitch_Pid[0], PID_POSITION,f_chassis_Pid_Para[2]);
	PID_Init(&Pitch_Pid[1], PID_POSITION,f_chassis_Pid_Para[3]);
	PID_Init(&Yaw_Pid[0],   PID_POSITION,f_chassis_Pid_Para[4]);
	PID_Init(&Yaw_Pid[1],   PID_POSITION,f_chassis_Pid_Para[5]);
	PID_Init(&Offset_Pid,   PID_POSITION,f_chassis_Pid_Para[6]);
}


/**
  * @brief          ������̬����
  * @param[out]     Chassis_Info:������Ϣ����ָ��.
  * @retval         none
  */
static void chassis_posture_control(Chassis_Info_t *Chassis_Info)	
{
		if(Chassis_Info == NULL) return;
			
		//·�̻�
		f_PID_Calculate(&Position_Pid,Chassis_Info->Target.position,Chassis_Info->Measure.position);
		//�ٶȻ�
		f_PID_Calculate(&linespeed_Pid,Chassis_Info->Target.linespeed,Chassis_Info->Measure.linespeed);		
		//�ǶȻ�
		f_PID_Calculate(&Pitch_Pid[0],Chassis_Info->Target.pit_angle[Chassis_Info->IF_PIT_ANGLE_OFFSET],*Chassis_Info->Measure.pit_angle);
		//���ٶȻ�
		f_PID_Calculate(&Pitch_Pid[1], Chassis_Info->Target.pit_gyro,*Chassis_Info->Measure.pit_gyro);
		//ת��
		if(Chassis_Info->mode != CHASSIS_SPIN)
		{
			f_PID_Calculate(&Yaw_Pid[0],Chassis_Info->Measure.yaw_err,0);
			f_PID_Calculate(&Yaw_Pid[1],Yaw_Pid[0].Output,Chassis_Info->Measure.yaw_gyro);
		}else
		{
			f_PID_Calculate(&Yaw_Pid[1],Chassis_Info->Target.yaw_gyro,Chassis_Info->Measure.yaw_gyro);
		}
		
		
		//��������ģʽ
		if(Chassis_Info->mode == CHASSIS_WEEK)
		{
				Chassis_Info->SendValue[Left_Wheel]  = 0;
				Chassis_Info->SendValue[Right_Wheel] = 0;
		}
}

/**
  * @brief          ���̿��Ƶ�������
  * @param[out]     Chassis_Info_send:������Ϣ����ָ��, TXFrame CAN����֡������ָ��
  * @retval         none
  */
static void chassis_send_current(Chassis_Info_t *Chassis_Info_send,CAN_TxFrameTypeDef *TXFrame)
{
	if(Chassis_Info_send == NULL || TXFrame == NULL) return;
	
		//���������ģʽ��ʹ��CANID�˲���ʹ��
	
		//ת�رջ���������(�ڵ��������ģʽʹ��)
		MT_hcanTxFrame[Left_Wheel].data[0]  = 0xA1;
		MT_hcanTxFrame[Right_Wheel].data[0] = 0xA1;

		TXFrame[Left_Wheel].data[4]  = (uint8_t)(Chassis_Info_send->SendValue[Left_Wheel]);
		TXFrame[Left_Wheel].data[5]  = (uint8_t)(Chassis_Info_send->SendValue[Left_Wheel] >> 8);
		TXFrame[Right_Wheel].data[4] = (uint8_t)(Chassis_Info_send->SendValue[Right_Wheel]);
		TXFrame[Right_Wheel].data[5] = (uint8_t)(Chassis_Info_send->SendValue[Right_Wheel] >> 8);
		
		USER_CAN_TxMessage(&TXFrame[Left_Wheel]);	  //0x144
		USER_CAN_TxMessage(&TXFrame[Right_Wheel]);	//0x143
}

#endif