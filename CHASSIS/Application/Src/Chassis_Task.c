//
// Created by WangJinJing
//
#include "cmsis_os.h"
#include "Chassis_Task.h"
#include "assist.h"
#include "pid.h"
#include "bsp_pm01_api.h"
#include "referee_info.h"
#include "chassis.h"
#include "bsp_can.h"

PID_TypeDef_t Chassis_Speed[4],Chassis_Folo[2];

Chassis_Info_t Chassis_Ctrl={
	  .ctrl_mode = RC_CTRL,
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
	uint32_t currentTime;
	Chassis_Reset(&Chassis_Ctrl);
  /* Infinite loop */
  for(;;)
  {
		pm01_access_poll();	        
		control_switch();	
		
		Chassis_AcclerateCurve(&Chassis_Ctrl.speed[_X],&Chassis_Ctrl.speed[_Y]);
		CHASSIS_Handler(Chassis_Ctrl.speed);

		USER_CAN_TxMessage(&hcan1,&CAN_TxMsg[_CAN1][_0x200]);

    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/**
	* @name		Chassis_Mode_Set
  * @brief  设置底盘模式
  * @param  Chassis_Info
  * @retval NONE
  */
void Chassis_Mode_Set(Chassis_Info_t *Chassis_Info)
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

/**
	* @name		Chassis_Info_Update
  * @brief  更新底盘信息
  * @param  Chassis_Info
  * @retval NONE
  */
void Chassis_Info_Update(Chassis_Info_t *Chassis_Info)
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

/**
	* @name		CHASSIS_Reset
  * @brief  重置底盘,重新初始化 PID 控制器
  * @param  chassis
  * @retval NONE
  */
void Chassis_Reset(Chassis_Info_t *chassis)
{
	chassis->mode = CHASSIS_INVA;
	Chassis_Set_Pid(chassis->ctrl_mode);
	CHASSIS_Stop();
}

/**
	* @name		Chassis_Test
  * @brief  测试底盘的运动控制，根据遥控器的信号设置底盘的运动速度
  * @param  chassis
  * @retval NONE
  */
void Chassis_Test(void)
{
	Chassis_Ctrl.speed[_X] = rc_ctrl.rc.ch[3] * 50;
	Chassis_Ctrl.speed[_Y] = rc_ctrl.rc.ch[2] * 50;
	Chassis_Ctrl.speed[_W] = rc_ctrl.rc.ch[0] * 3;
}

/**
	* @name		Chassis_Rc_Ctrl
  * @brief  根据遥控器的信号来控制底盘的运动
  * @param  NONE
  * @retval NONE
  */
void Chassis_Rc_Ctrl(void)
{
	
	float speed_k=0.f;

	Chassis_Ctrl.speed[_X] = Speed_X_deliver(Chassis_Ctrl.CH2 * 50,
																			     Chassis_Ctrl.CH3 * 50);
	Chassis_Ctrl.speed[_Y] = speed_Y_deliver(Chassis_Ctrl.CH2 * 50,
																			     Chassis_Ctrl.CH3 * 50);
	Chassis_Ctrl.speed[_W] = Speed_W_deliver(Chassis_Ctrl.mode);

	#if IF_DR16_CONNECT
		CHASSIS_Test();
	#endif

	if(ABS(Chassis_Ctrl.speed[_W]) >= 300)
	{
		speed_k = (2000 - ABS(Chassis_Ctrl.speed[_W]))/2000;
		speed_k = powf(speed_k,1.5f);
		VAL_LIMIT(speed_k,0.f,1.f);
	}else{
		speed_k = 1.f;
	}
	Chassis_Ctrl.speed[_X] *= speed_k;
	Chassis_Ctrl.speed[_Y] *= speed_k;
}

/**
	* @name		Chassis_Key_Ctrl
  * @brief  根据键盘的信号来控制底盘的运动
  * @param  NONE
  * @retval NONE
  */
void Chassis_Key_Ctrl(void)
{
	float speed_k=0.f;
	uint16_t power_limit = game_robot_state.chassis_power_limit;
	
	if(power_limit==50)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 38, KEY_Y_Ctrl() * 45);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 38, KEY_Y_Ctrl() * 45);
	}
	else if(power_limit==55)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 48, KEY_Y_Ctrl() * 52);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 48, KEY_Y_Ctrl() * 52);
	}
	else if(power_limit==60)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 50, KEY_Y_Ctrl() * 54);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 50, KEY_Y_Ctrl() * 54);
	}						
	else if(power_limit==65)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 51, KEY_Y_Ctrl() * 57);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 51, KEY_Y_Ctrl() * 57);
	}									
	else if(power_limit==70)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 52.5, KEY_Y_Ctrl() * 58);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 52.5, KEY_Y_Ctrl() * 58);
	}		
	else if(power_limit==90)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 57.5, KEY_Y_Ctrl() * 62);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 57.5, KEY_Y_Ctrl() * 62);
	}
	else if(power_limit==120)
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 61.5, KEY_Y_Ctrl() * 65);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 61.5, KEY_Y_Ctrl() * 65);
	}
	else
	{
		Chassis_Ctrl.speed[_X] = Speed_X_deliver(KEY_X_Ctrl() * 20, KEY_Y_Ctrl() * 15);
		Chassis_Ctrl.speed[_Y] = speed_Y_deliver(KEY_X_Ctrl() * 20, KEY_Y_Ctrl() * 15);
	} 
	
	Chassis_Ctrl.speed[_W] = Speed_W_deliver(Chassis_Ctrl.mode);
	if(ABS(Chassis_Ctrl.speed[_W]) >= 300)
	{
		speed_k = (2000 - ABS(Chassis_Ctrl.speed[_W]))/2000;
		speed_k = powf(speed_k,1.2f);
		VAL_LIMIT(speed_k,0.f,1.f);
	}else{
		speed_k = 1.25f;
	}
	Chassis_Ctrl.speed[_X] *= speed_k*6;
	Chassis_Ctrl.speed[_Y] *= speed_k*6;	
}

/**
	* @name		AcclerateCurve
  * @brief  公式原型 y = 1/(1+e^(-k(x-2/k)))  当k = 4.2 , x = 1时 y = 0.9
  * @param  x, k
  * @retval 
  */
static float AcclerateCurve(float x , float k)
{
	float y;
	k = 4.2f / k;
	if(k == 0)
	{
		return 1;
	}
	y = 1/(1+powf(NATURAL_NUMBER,(-k*(x-2/k))));
	return y;
}

/**
	* @name		DecclerateCurve
  * @brief  公式原型 y =1/(1+e^(?k(-(x-(5/k))?2/k) ) ) 当k = 10 , x = 0.8时 y = 0.01
  * @param  x, k
  * @retval 
  */
static float DecclerateCurve(float x , float k)
{
	float y;
	if(k == 0)
	{
		return 1;
	}
	y = 1/(1+powf(NATURAL_NUMBER,(-k*(-x+3/k))));
	return y;
}


/**
	* @name		Chassis_AcclerateCurve
  * @brief  底盘加速曲线
  * @param  speed_X, speed_Y
  * @retval 
  */
void Chassis_AcclerateCurve(float *speed_X,float *speed_Y)
{
	static float speed_line = 0.f;
  static float speed_linelast=0.f;//上一次的线速度
  static bool  accelerating = false,decelerating = false;//速度增减情况
  static float accCnt=0.f;//自增计时
  static float speed_k = 0;//曲线增益
  static float acck = 2.5f,deck = 20;//加速、减速曲线系数
  static float deceleRecode_Y=0.f,deceleRecode_X=0.f;//减速曲线缓存速度
	
	/*线速度计算*/
	speed_line = sqrt(powf(*speed_X,2.f)+powf(*speed_Y,2.f));

	/*加减速判断*/
	if((ABS(speed_line) - ABS(speed_linelast)) > 1000)
	{
		accelerating = 1;
		decelerating = 0;
		accCnt = 0;
	}
	else if((ABS(speed_line) - ABS(speed_linelast)) < -1000)
	{
		accelerating = 0;
		decelerating = 1;
		accCnt = 0;
	}

	/*加速曲线*/
	if(accelerating == 1)
	{
		accCnt += 0.005f;
		speed_k = AcclerateCurve(accCnt,acck);
		if (speed_k > 0.999f)
        {
            accelerating = 0;
        }
	}
	else if(decelerating != 1)
    {
        speed_k = 1;
        accCnt = 0;
    }

	/*减速曲线*/
	if(decelerating == 1)
	{
		accCnt += 0.005f;
		speed_k = DecclerateCurve(accCnt,deck);
		if (speed_k < 0.01f)
        {
            decelerating = 0;
        }
	}
	else if(accelerating != 1)
    {
        speed_k = 1;
        accCnt = 0;
    }

	/*增益计算*/
	if(accelerating == 1)
	{
		*speed_Y *= speed_k;
		*speed_X *= speed_k;
	}
	else if(decelerating == 1)
	{
		*speed_Y = deceleRecode_Y * speed_k;
		*speed_X = deceleRecode_X * speed_k;
	}
	if(decelerating != 1)
	{
		deceleRecode_Y = *speed_Y;
		deceleRecode_X = *speed_X;
	}

	/*更新上一次的线速度*/
	speed_linelast = speed_line;
}

