//#include "robot_ref.h"

#include "cmsis_os.h"
#include "Shoot_Task.h"
#include "Chassis_Task.h"
#include "SystemState_Task.h"

#include "motor.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "pid.h"

int SHOOT_SPEED_16M_S = 6250,SHOOT_SPEED_10M_S = 4200;//荧光弹丸大概5560-5570比赛时候是5515
int Shoot_speed_mode = 0;
float speed_add,speed_tem;
int stuck_flag = 0;//不卡为0，卡弹为1
int stall_back_flag;
int Trigger_time_num;
int RC_shoot_flag = 0;
int RC_direction = 1;
int Key_shoot_flag = 0;
int Key_direction = 1;
float angle_Err1,speed_Err;

Shoot_Info_t Shoot_Ctrl = {
	  
	
    .shoot_mode = Shoot_Off,
    .dr16 = &rc_ctrl,
    .wheel_L = &DJI_Motor[Left_Shoot],
    .wheel_R = &DJI_Motor[Right_Shoot],
    .trigger = &DJI_Motor[Trigger],
};

Shoot_Info_t Shoot;

PID_TypeDef_t Shoot_PID[2],Trigger_PID[2];

//deadband, maxIntegral, max_out, kp, ki, kd
float wheel_PID_Param[PID_PARAMETER_CNT]={0, 10000, 15000, 10, 0.f, 0,};

float trigger_PID_Param[2][2][PID_PARAMETER_CNT] = 
{
			[0]={0, 10000, 15000, 16, 0.05f, 0,},
			[1]={0, 0, 10000, 60, 0, 0,},
};

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
	uint32_t currentTime;
	PID_Init_ByParamArray(&DJI_Motor[Left_Shoot].pid_Speed, &wheel_PID_Param[1]);
  PID_Init_ByParamArray(&DJI_Motor[Right_Shoot].pid_Speed, &wheel_PID_Param[1]);
  PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Speed,trigger_PID_Param[1][0]);
  PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Angle,trigger_PID_Param[1][1]);
	
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();
		
		if(Key_B() == true)
			Shoot_Ctrl.shoot_mode = Shoot_On;
		else if(Key_B() == false)
			Shoot_Ctrl.shoot_mode = Shoot_Off;

		Wheel_Ctrl();
		Trigger_Ctrl();
		trigger_Stall_Handle();	
		
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

static void shoot_State_Handoff(DEVICE_STATE state)
{
    if(state!=Shoot.state){//防止循环调用出错
    //参数重置（受当前模式影响）
		Shoot.state=state;
        PID_Init_ByParamArray(&Shoot.wheel_L->pid_Speed, &wheel_PID_Param[state]);
        PID_Init_ByParamArray(&Shoot.wheel_R->pid_Speed, &wheel_PID_Param[state]);
        PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Speed, trigger_PID_Param[state][0]);
        PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Angle, trigger_PID_Param[state][1]);
    }
}

uint32_t Report_Shoot_NUM(void)
{
  return robot.shooter_id1_42mm.bullet_remaining_num_42mm;
}

uint32_t Report_Shoot_SPEED(void)
{
  return robot.bullet_speed;
}



static void Wheel_Ctrl(void)
{
  int16_t send_Value[2];
  float speed_Err[2] = {0,},target_speed;

	if(Chassis_Ctrl.ctrl == 1)//遥控模式
	{
		if(rc_ctrl.rc.s[1] == 2 && rc_ctrl.rc.s[0] == 3)
			Shoot_speed_mode=0;
		if(rc_ctrl.rc.s[1] == 2 && rc_ctrl.rc.s[0] == 1)
			Shoot_speed_mode=1;
		
		if(Shoot_speed_mode==0)
		{
			target_speed = SHOOT_SPEED_10M_S + Temp_BURST_Fix();
		}
		if(Shoot_speed_mode==1)
		{
			target_speed = SHOOT_SPEED_16M_S + Temp_RATE_Fix();
		}
		
		speed_Err[0] = (  target_speed - Shoot_Ctrl.wheel_L->Data.velocity);
		speed_Err[1] = ( -target_speed - Shoot_Ctrl.wheel_R->Data.velocity);
	}
	else if(Chassis_Ctrl.ctrl == 2)//键盘模式
  {
		if(Shoot.shoot_mode == Shoot_On)
		{
			switch(robot.mode)
			{
				case INITIAL:
					
						speed_tem = Temp_RATE_Fix();
						target_speed = SHOOT_SPEED_16M_S + speed_tem;

						break;
				case BURST:
					
						target_speed = SHOOT_SPEED_10M_S + Temp_BURST_Fix();
				
						break;
				case RATE:

						speed_tem = Temp_RATE_Fix();
						target_speed = SHOOT_SPEED_16M_S + speed_tem;
				
						break;          
				default:
					
						target_speed=0;
				
						break;
			 }

			speed_Err[0] = (  target_speed - Shoot_Ctrl.wheel_L->Data.velocity);
			speed_Err[1] = ( -target_speed - Shoot_Ctrl.wheel_R->Data.velocity);
		}
		else if(Shoot.shoot_mode == Shoot_Off)
		{
			speed_Err[0] =0;
			speed_Err[1] =0;
		}
	}

	
	f_PID_Calculate(&DJI_Motor[Left_Shoot].pid_Speed, speed_Err[0],0);
	f_PID_Calculate(&DJI_Motor[Right_Shoot].pid_Speed, speed_Err[1],0);
	send_Value[0] = Shoot_Ctrl.wheel_L->pid_Speed.Err[0];
	send_Value[1] = Shoot_Ctrl.wheel_R->pid_Speed.Err[1];


	Shoot_Ctrl.wheel_L->txMsg->data[L[Shoot_Ctrl.wheel_L->Data.StdId-0x201]] = (uint8_t)(send_Value[0]>>8);
	Shoot_Ctrl.wheel_L->txMsg->data[H[Shoot_Ctrl.wheel_L->Data.StdId-0x201]] = (uint8_t)(send_Value[0]);
	Shoot_Ctrl.wheel_R->txMsg->data[L[Shoot_Ctrl.wheel_R->Data.StdId-0x201]] = (uint8_t)(send_Value[1]>>8);
	Shoot_Ctrl.wheel_R->txMsg->data[H[Shoot_Ctrl.wheel_R->Data.StdId-0x201]] = (uint8_t)(send_Value[1]);
}

static float Temp_BURST_Fix(void)
{
  float temp_scope = 35;//假设变化范围为35摄氏度
  float temp_low = 35;//初始温度设定为35摄氏度
  float res = 0;
  float temp_real;
  
  temp_real = ((float)Shoot_Ctrl.wheel_L->Data.temperature + (float)Shoot_Ctrl.wheel_R->Data.temperature)/2;
  
  if(temp_real >= temp_low)
    res = ((temp_real - temp_low)/temp_scope * (-35))*7;
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -35;
  
		return res;
}

static float Temp_RATE_Fix(void)
{
  float temp_scope = 15;//假设变化范围为25摄氏度
  float temp_low = 29;//初始温度设定为35摄氏度
  float res = 0;
  float temp_real;
	
  temp_real = ((float)Shoot_Ctrl.wheel_L->Data.temperature + (float)Shoot_Ctrl.wheel_R->Data.temperature)/2;
  
  if(temp_real >= temp_low)
	{
		if(temp_real - temp_low >0 && temp_real - temp_low <4.f){
			res=(temp_real - temp_low)*(-0.8f);}
		if(temp_real - temp_low >4.f && temp_real - temp_low <9.f){
			res=4*(-0.8f) + (temp_real - temp_low - 4)*(-1.8f);}
		if(temp_real - temp_low >9.f){
			res=4*(-0.8f) + 4*(-1.8f) + (temp_real - temp_low - 9)*(-2.8f);}
	}

  else if(temp_real < temp_low)res = 0;
  else if(temp_real > temp_low + temp_scope)res = -70;
  
	return res;
}

static float SpeedAdapt_10M(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;

  if(real_S < 8) res+=2*up_num;//射速太低
	else if(real_S < min_S && real_S > 8) res+=up_num;//射速偏低
  else if(real_S >= min_S && real_S <= max_S )res = 0;
	else if(real_S > max_S) res -= down_num;//射速偏高
	
  return res;
}

static float SpeedAdapt_16M(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;

  if(real_S < 14.3f) res+=1.8f*up_num;//射速太低
	else if(real_S < min_S && real_S > 14.3f) res+=up_num;//射速偏低
  else if(real_S >= min_S && real_S <= max_S )res = 0;
	else if(real_S > max_S && real_S <15.5f) res -= down_num;//射速稍微偏高
	else if(real_S >15.5f)res -=1.5f * down_num;
	
  return res;
}

bool Judge_IF_SingeStuck(void)
{
	bool res = false;
	
	float angle_err = DJI_Motor[Trigger].pid_Angle.Err[0];
	
	if(ABS(angle_err) >= 360.f/2.f)
	{
			res = true;
	}
	
	return res;
}

static void trigger_Stall_Handle(void)
{
	static uint16_t cnt = 0;
	
	if(Judge_IF_SingeStuck() == true)
	{
		cnt++;
		if(cnt > 100)
		{
			stuck_flag = 1;
			cnt = 0;
		}
	}else if(Judge_IF_SingeStuck() == false)
	{
		stuck_flag = 0;
		cnt = 0;
	}
}

static void Trigger_Ctrl(void)
{	
	  static uint32_t _time[DJI_MOTOR_NUM];
    static bool IF_TRIGGER_POSITIVE=false;
		static bool IF_TRIGGER_NEGATIVE=false;
    int16_t send_Value;
	
		if(DJI_Motor[Trigger].stalled == 1 && stall_back_flag == 1)
		{
		  PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Speed, trigger_PID_Param[1][0]);
      PID_Init_ByParamArray(&DJI_Motor[Trigger].pid_Angle, trigger_PID_Param[1][1]);
			Shoot.trigger_Angle = DJI_Motor[Trigger].Data.angle - ((31/12)*10.f);
			stall_back_flag = 0;
		}
		else if(DJI_Motor[Trigger].stalled==0)
		{
			if(Chassis_Ctrl.ctrl == 1 && rc_ctrl.rc.s[1] == 2)//遥控器控制
			{
					if(Trigger_time_num < 300)
				{
					Trigger_time_num++;
				}
			else if((rc_ctrl.rc.ch[1] != 0 && RC_shoot_flag == 0))
				{
					if(rc_ctrl.rc.ch[1] > 0)
						RC_direction = 1;
					if(rc_ctrl.rc.ch[1] < 0)
						RC_direction = -1;

					Shoot.trigger_Angle = DJI_Motor[Trigger].Data.angle	+ RC_direction * 186.f;//}12比31
					Trigger_time_num = 0;
					RC_shoot_flag = 1;
				}
				else if(rc_ctrl.rc.ch[1] == 0)
					RC_shoot_flag = 0;
			}
		}
		else if(Chassis_Ctrl.ctrl == 2 && Shoot.shoot_mode == Shoot_On)//键盘控制，点射
		{
			if(Trigger_time_num < 300)
			{
				Trigger_time_num++;
			}				
			else
			{
				if(Key_mouse_l() == false)
					IF_TRIGGER_POSITIVE = true;
				if(Key_R() == false)
					IF_TRIGGER_NEGATIVE = true;
					
				if(IF_TRIGGER_POSITIVE == true && Key_shoot_flag == 0)//鼠标左键打弹，拨弹盘正转
				{
					if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100)//发射受枪口热量限制
					{
						Shoot.trigger_Angle = DJI_Motor[Trigger].Data.angle	+ Key_direction * 186.f;  //((31/12)*72.f)
						Trigger_time_num = 0;
					}
						IF_TRIGGER_POSITIVE = false;
						Key_shoot_flag = 1;
				}

				if(IF_TRIGGER_NEGATIVE == true && Key_shoot_flag == 0)//按键R退弹，拨弹盘反转
				{
					if(robot.shooter_id1_42mm.cooling_limit - robot.cooling_heat >= 100)
					{
						Shoot.trigger_Angle = DJI_Motor[Trigger].Data.angle	- Key_direction * 186.f;
						Trigger_time_num=0;
					}
						IF_TRIGGER_NEGATIVE = false;
						Key_shoot_flag = 1;
				}

				if(Key_mouse_l() == true)
					Key_shoot_flag = 0;
				if(Key_R() == true)
					Key_shoot_flag = 0;
			}
		}

		angle_Err1 = Shoot.trigger_Angle - DJI_Motor[Trigger].Data.angle;

		speed_Err = f_PID_Calculate(&DJI_Motor[Trigger].pid_Angle,angle_Err1,0) - DJI_Motor[Trigger].Data.velocity;

		send_Value = f_PID_Calculate(&DJI_Motor[Trigger].pid_Speed,speed_Err,0);

    DJI_Motor[Trigger].txMsg->data[L[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(send_Value>>8);
    DJI_Motor[Trigger].txMsg->data[H[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(send_Value);
		
		if(DJI_Motor[Trigger].Data.temperature >= 100)
		{
			DJI_Motor[Trigger].txMsg->data[L[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(0>>8);
		  DJI_Motor[Trigger].txMsg->data[H[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(0);}
		
			if(DJI_Motor[Trigger].I_Level != HIGH)
			{
				_time[Trigger] = xTaskGetTickCount();
				DJI_Motor[Trigger].stalled = 0;
			}
			
			if(xTaskGetTickCount() - _time[Trigger] > 500)
			{
				DJI_Motor[Trigger].txMsg->data[L[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(0>>8);
				DJI_Motor[Trigger].txMsg->data[H[DJI_Motor[Trigger].Data.StdId-0x201]] = (uint8_t)(0);
				stall_back_flag = 1;
				DJI_Motor[Trigger].stalled = 1;
			}
}

