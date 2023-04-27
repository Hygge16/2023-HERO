//
// Created by Yuanbin on 22-10-3.
//

#include "robot_ref.h"

#if defined(GIMBAL_BOARD)

#include "cmsis_os.h"
#include "Shoot_Task.h"

#include "motor.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "pid.h"

Shoot_Info_t Shoot_Ctrl = {
    .mode = AUTO,
    .trigger_Buf = 7,//拨盘叶数
    .rc_ctrl = &rc_ctrl,
		.stuck_flag = 1,//卡弹反转
		.wheel_Speed={
            [INITIAL]={//初始
                [LV_1]=SHOOT_SPEED_15M_S,
                [LV_2]=SHOOT_SPEED_15M_S,
                [LV_3]=SHOOT_SPEED_15M_S,
            },
            [BURST]={//爆发优先
                [LV_1]=SHOOT_SPEED_15M_S,
                [LV_2]=SHOOT_SPEED_15M_S,
                [LV_3]=SHOOT_SPEED_15M_S,
            },
            [COOLING]={//冷却优先
                [LV_1]=SHOOT_SPEED_15M_S,
                [LV_2]=SHOOT_SPEED_18M_S,
                [LV_3]=SHOOT_SPEED_18M_S,
            },
            [RATE]={//射速优先
                [LV_1]=SHOOT_SPEED_30M_S,
                [LV_2]=SHOOT_SPEED_30M_S,
                [LV_3]=SHOOT_SPEED_30M_S,
            },
					},
};

float wheel_PID_Param[PID_PARAMETER_CNT]={0,1000,15000,13,0.1f,0,};

float trigger_PID_Param[2][PID_PARAMETER_CNT]={
			[0]={0,1000,10000,16,0.24f,0,},
			[1]={0,0,3000, 60,0,0,},
};

PID_TypeDef_t Shoot_PID[2],Trigger_PID[2];

static void Shoot_Init(void);
static void wheel_Ctrl(void);
static void trigger_Ctrl(void);
static void trigger_Stall_Handle(void);
static uint16_t Trigger_Speed_deliver(uint16_t cooling_rate);

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the StartShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */\
	Shoot_Init();
  /* Infinite loop */
  for(;;)
  {
		
		wheel_Ctrl();
		
		trigger_Ctrl();
		
		trigger_Stall_Handle();
		
		USER_CAN_TxMessage(&GimbalTxFrame[2]);
		
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

static void Shoot_Init(void)
{
	PID_Init(&Shoot_PID[0],PID_POSITION,wheel_PID_Param);
	PID_Init(&Shoot_PID[1],PID_POSITION,wheel_PID_Param);
	PID_Init(&Trigger_PID[0],PID_POSITION,trigger_PID_Param[0]);
	PID_Init(&Trigger_PID[1],PID_POSITION,trigger_PID_Param[1]);
}


static void wheel_Ctrl(void)
{
	int16_t send_Value[2];
	float res = Shoot_Ctrl.wheel_Speed[robot.mode][robot.level];
	float speed_gain = 0;
	float real_speed = robot.bullet_speed;
	static float speed_last = 0; 
	//射速发生变化
	if(real_speed != speed_last)
	{
		//射速控制
		if(res==SHOOT_SPEED_15M_S)
		{
			speed_gain += SpeedAdapt(real_speed, 13.7f , 14.3f , 15 , 35);
		}else if(res==SHOOT_SPEED_18M_S)
		{
			speed_gain += SpeedAdapt(real_speed , 16.7f , 17.3f , 15 , 40);
		}else if(res==SHOOT_SPEED_30M_S)
		{
			speed_gain += SpeedAdapt(real_speed , 27.7f , 28.3f , 25 , 55);
		}
	}
	res += speed_gain + Temp_Fix_speed(real_speed);
	speed_last = real_speed;
			
//速度控制(PID控制方案)
    send_Value[0] = f_PID_Calculate(&Shoot_PID[0], res,Gimbal_Motor[Left_Friction].Data.velocity);
    send_Value[1] = f_PID_Calculate(&Shoot_PID[1],-res,Gimbal_Motor[Right_Friction].Data.velocity);
	
	if(rc_ctrl.rc.s[1]!=2)
	{
		send_Value[0] = 0;
		send_Value[1] = 0;
	}
	
//发送装载
		GimbalTxFrame[2].data[0] = (uint8_t)(send_Value[0] >> 8);
		GimbalTxFrame[2].data[1] = (uint8_t)(send_Value[0]);
		GimbalTxFrame[2].data[2] = (uint8_t)(send_Value[1] >> 8);
		GimbalTxFrame[2].data[3] = (uint8_t)(send_Value[1]);
}

static void trigger_Ctrl(void)
{
	int16_t send_value;
	
	if(rc_ctrl.rc.s[1] == 2)
	{
		send_value = f_PID_Calculate(&Trigger_PID[0],Shoot_Ctrl.stuck_flag*(rc_ctrl.rc.ch[1]*5 - Key_mouse_l()*Trigger_Speed_deliver(robot.shooter_id1_17mm.cooling_rate)),Gimbal_Motor[Trigger].Data.velocity);
	}else
	{
		send_value = 0;
	}
	
	GimbalTxFrame[2].data[4] = (uint8_t)(send_value >> 8);
	GimbalTxFrame[2].data[5] = (uint8_t)(send_value);
}

static uint16_t Trigger_Speed_deliver(uint16_t cooling_rate)
{
	float res = 0;
	
	switch(cooling_rate)//枪口每秒冷却值
	{
		case 15:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 20)//2发余量
				res = TRIGGER_FREQ_3_HZ;
			else 
				res = 0;
		break;
		case 25:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 20)
				res = TRIGGER_FREQ_5_HZ;
			else 
				res = 0;
		break;
		case 35:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 30)
				res = TRIGGER_FREQ_5_HZ;
			else 
				res = TRIGGER_FREQ_3_HZ;
		break;
		case 40:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 30)
				res = TRIGGER_FREQ_6_HZ;
			else 
				res = TRIGGER_FREQ_3_HZ;
		break;
		case 60:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 30)
				res = TRIGGER_FREQ_8_HZ;
			else
				res = TRIGGER_FREQ_5_HZ;
		break;
		case 80:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 30)
				res = TRIGGER_FREQ_10_HZ;
			else 
				res = TRIGGER_FREQ_7_HZ;							
		break;
		default:
			if((robot.shooter_id1_17mm.cooling_limit-robot.cooling_heat) >= 20)//2发余量
				res = TRIGGER_FREQ_3_HZ;
			else 
				res = 0;
		break;
	}
	return res;
}

//堵转判断和处理（全模式调用）
static void trigger_Stall_Handle(void)
{
	static uint16_t cnt = 0;
	
	if(Judge_IF_AutoBlock(Trigger_PID[0].Err[0])==true)
	{
		cnt++;
		if(cnt > 300)
		{
			Shoot_Ctrl.stuck_flag = -Shoot_Ctrl.stuck_flag;
			cnt = 0;
		}
	}else{
			cnt = 0;
	}
}


#endif
