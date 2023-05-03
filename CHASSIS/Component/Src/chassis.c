#include "cmsis_os.h"
#include "arm_math.h"
#include "assist.h"
#include "pid.h"
#include "bsp_can.h"
#include "motor.h"
#include "chassis.h"
#include "bsp_pm01_api.h"
#include "referee_info.h"
#include "bsp_imu.h"

/*****************************************************************/
int pm01_switch=1;
float limitk1;
float limitk2;
double scaling[4];
float chassis_pidout = 0.f;
float  klimit = 0.f,plimit = 0.f;
float  chassis_pidout_max = 61536.f;
int16_t CAN1_SendData[4]={0.f,};
float Expect[4]={0.f,};

/*****************************************************************/

//Deadband  IntegralMAX  OutputMAX  Kp  Ki  Kd 
float f_PID_Wheel_Para[2][PID_PARAMETER_CNT]=
{
	[0] = {0, 0, 16000, 7.f, 0.f, 0.f, },			//RC
	[1] = {0, 0, 16000, 7.f, 0.f, 0.f, },			//KEY
};

float f_PID_Folo_Para[2][PID_PARAMETER_CNT]=
{ 
	[0]={0, 0, 500.f, 0.8f, 0.f, 0.f, },				//angle
	[1]={0, 1000.f, 1200.f, 3.8f, 0.f, 0.f, },	//speed
};

/**
	* @name		Speed_X_deliver
  * @brief  将给定的底盘速度分解成X方向的速度并返回。
  * @param  speed_X, speed_Y
  * @retval X轴方向上的速度值
  */
float Speed_X_deliver(float speed_X , float speed_Y)
{
	float angle = DJI_Motor[Gimbal_Yaw].Data.angle - Chassis_Ctrl.midangle;
				angle = ABS(angle) > 180 ? angle - angle/ABS(angle)*360 :angle;
	
	float sin_yaw = arm_sin_f32(angle*2*3.1415926f/360.f);
	float cos_yaw = arm_cos_f32(angle*2*3.1415926f/360.f);
	
  return (cos_yaw * speed_X - sin_yaw * speed_Y);
}

/**
	* @name		speed_Y_deliver
  * @brief  将给定的底盘速度分解成Y方向的速度并返回。
  * @param  speed_X, speed_Y
  * @retval Y轴方向上的速度值
  */
float speed_Y_deliver(float speed_X , float speed_Y)
{
	float angle = DJI_Motor[Gimbal_Yaw].Data.angle - Chassis_Ctrl.midangle;
				angle = ABS(angle) > 180 ? angle - angle/ABS(angle)*360 :angle;
	
	float sin_yaw = arm_sin_f32(angle*2*3.1415926f/360.f);
	float cos_yaw = arm_cos_f32(angle*2*3.1415926f/360.f);
	
  return (sin_yaw * speed_X + cos_yaw * speed_Y);
}

/**
	* @name		Speed_W_deliver
  * @brief  控制底盘的旋转速度
  * @param  mode
  * @retval 底盘的旋转速度
  */
float Speed_W_deliver(Chassis_Mode_e mode)
{
	float res=0.f;
	uint16_t power_limit = game_robot_state.chassis_power_limit;
	
	if(mode == CHASSIS_FOLO)
	{
		float angle = Chassis_Ctrl.midangle - DJI_Motor[Gimbal_Yaw].Data.angle ;
					angle = ABS(angle) > 180 ? angle-angle/ABS(angle)*360 :angle;
		float KF_angle_err = KalmanFilter(&Chassis_Ctrl.KF_FOLO_Angle,angle);
		float speed_err = DJI_Motor[Gimbal_Yaw].pid_Angle.f_PID_Calculate(&DJI_Motor[Gimbal_Yaw].pid_Angle, KF_angle_err, 0); 
		
		res = DJI_Motor[Gimbal_Yaw].pid_Speed.f_PID_Calculate(&DJI_Motor[Gimbal_Yaw].pid_Speed, speed_err,0);
		
//		float speed_err = DJI_Motor[Gimbal_Yaw].pid_Angle.f_PID_Calculate(&DJI_Motor[Gimbal_Yaw].pid_Angle,KF_angle_err) \
//											-MPU6050_Real_Data.Gyro_Z;
//		res = DJI_Motor[Gimbal_Yaw].pid_Speed.f_PID_Calculate(&DJI_Motor[Gimbal_Yaw].pid_Speed,speed_err);


		VAL_LIMIT(res,-power_limit*9.f,power_limit*9.f);
		
	}
	
	if(mode == CHASSIS_SPIN)
	{
		if(power_limit==50) 		   res = 700;
		else if(power_limit==55) 	 res = 900;
		else if(power_limit==60)   res = 980;
		else if(power_limit==65)   res = 1024;
		else if(power_limit==70)   res = 1124;
		else if(power_limit==90)   res = 1350;
		else if(power_limit==120)  res = 1600;
		else res=980;
	}
	if(ABS(res) < 50) res = 0;
	
	return res;
}

/**
	* @name		KEY_Slope_deliver
  * @brief  The keyboard controls the acceleration rate
  * @param  speed_type
  * @retval 相应的斜率值
  */
static float KEY_Slope_deliver(Chassis_Speed_Type speed_type)
{
	float slope = 0.f;
	uint16_t power_limit = game_robot_state.chassis_power_limit;

	if(speed_type!=_X && speed_type!=_Y)
	{
		return 0.f;
	}
	else if(speed_type == _X)
	{
		if(power_limit == 50) 			slope = 0.0120f;
		else if(power_limit == 55)  slope = 0.0160f;
		else if(power_limit == 60)  slope = 0.0190f;
		else if(power_limit == 65)  slope = 0.0200f;
		else if(power_limit == 70)  slope = 0.0205f;
		else if(power_limit == 90)  slope = 0.0220f;
		else if(power_limit == 120) slope = 0.0245f;
		else slope = 0.0190f;
	}
	else if(speed_type == _Y)
	{
		if(power_limit == 50) 			slope = 0.014f;
		else if(power_limit == 55)  slope = 0.0175f;
		else if(power_limit == 60)  slope = 0.0205f;
		else if(power_limit == 65)  slope = 0.0215f;
		else if(power_limit == 70)  slope = 0.0225f;
		else if(power_limit == 90)  slope = 0.0250f;
		else if(power_limit == 120) slope = 0.0275f;
		else slope = 0.0205f;
	}
	return slope;
}

/**
	* @name		KEY_X_Ctrl
  * @brief  控制在X轴方向上的移动
  * @param  None
  * @retval X轴方向上的位置
  */
float slope_x = 0.f,slope_y = 0.f;
float KEY_X_Ctrl(void)
{
	float slope = KEY_Slope_deliver(_X);
	static float res_X = 0.f,KEY_MAX = 660.f;

	if(Key_A()==1)
	{
		res_X = RAMP_float(-KEY_MAX,res_X,slope);
	}else if(Key_D()==1){
		res_X = RAMP_float(KEY_MAX,res_X,slope);
	}else{
		res_X = 0;
	}

	return res_X;
}

/**
	* @name		KEY_Y_Ctrl
  * @brief  控制在Y轴方向上的移动
  * @param  None
  * @retval Y轴方向上的位置
  */
float KEY_Y_Ctrl(void)
{
	float slope = KEY_Slope_deliver(_Y);;
	static float res_Y = 0.f,KEY_MAX = 660.f;

	if(Key_W()==1)
	{
		res_Y = RAMP_float(KEY_MAX,res_Y,slope);
	}else if(Key_S()==1){
		res_Y = RAMP_float(-KEY_MAX,res_Y,slope);
	}else{
		res_Y = 0;
	}
	
	return res_Y;
}

/**
	* @name		Power_Limit
  * @brief  Chassis power control
  * @param  expect, sendData
  * @retval None
  */
static void Power_Limit(float *expect,int16_t *sendData)
{
	float output[4]={0.f,};
	
	for(int i = 0; i < Gimbal_Yaw; i++)
	{
		output[i] = f_PID_Calculate(&DJI_Motor[i].pid_Speed,expect[i], DJI_Motor[i].Data.velocity);
		Chassis_Power_Limit(output);
		sendData[i] = output[i]; 
	}
}

/**
	* @name		Chassis_Power_Limit
  * @brief  Chassis power control
  * @param  output
  * @retval 底盘的输出功率
  */
void Chassis_Power_Limit(float* output)
{
	if(power_heat_data.chassis_power>960)
	{
		for(int i=0;i<4;i++)
		{
			VAL_LIMIT(DJI_Motor[i].pid_Speed.Output,-4096,4096);
		}
	}
	
	else if(pm01_switch == 0)
	{
		chassis_pidout = ABS(DJI_Motor[0].pid_Speed.Err[0])
									 + ABS(DJI_Motor[1].pid_Speed.Err[0])
									 + ABS(DJI_Motor[2].pid_Speed.Err[0])
									 + ABS(DJI_Motor[3].pid_Speed.Err[0]);
		for(int i=0;i<4;i++)
		{
			scaling[i] = DJI_Motor[i].pid_Speed.Err[0]/chassis_pidout;
		}
		klimit = chassis_pidout/36080.0f;
		VAL_LIMIT(klimit,-1,1);
		
		if(power_heat_data.chassis_power_buffer<50
				&&power_heat_data.chassis_power_buffer>=40)	plimit=0.9;
		else if(power_heat_data.chassis_power_buffer<40
				&&power_heat_data.chassis_power_buffer>=35)	plimit=0.75;
		else if(power_heat_data.chassis_power_buffer<35
				&&power_heat_data.chassis_power_buffer>=30)	plimit=0.5;
		else if(power_heat_data.chassis_power_buffer<30
				&&power_heat_data.chassis_power_buffer>=20)	plimit=0.25;
		else if(power_heat_data.chassis_power_buffer<20
				&&power_heat_data.chassis_power_buffer>=10)	plimit=0.125;
		else if(power_heat_data.chassis_power_buffer<10)	plimit=0.05;
		
		else if(power_heat_data.chassis_power_buffer==60)	plimit=1;
		
		for(int i=0;i<4;i++)
		{
			output[i] = chassis_pidout_max * scaling[i] * klimit * plimit;
		}
	}
	
	else if(pm01_switch == 1)
	{
		limitk1 = power_heat_data.chassis_power /(game_robot_state.chassis_power_limit );

		VAL_LIMIT(limitk1,0.7f,0.8f);
		limitk2 = 0.5f;

		if(pm01_od.v_out > 2100)
		{
			limitk1=pow(limitk1,0.4);
		}else if(pm01_od.v_out > 1800 && pm01_od.v_out < 2100)
		{
			limitk1 = sqrt(limitk2);
		}

		for(int i = 0; i < 4; i++)
		{
			output[i] *= limitk1; 
		}
	}
	
}

/**
	* @name		control_switch
  * @brief  Supercap control switch
  * @param  None
  * @retval None
  */
void control_switch()
{	
		if(pm01_od.v_out >= 1800)
		{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
			pm01_switch=1;
		}
		else if(pm01_od.v_out < 1800 && Key_SHIFT() == true)
		{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
			pm01_switch=1;
		}
		else if(pm01_od.v_out < 1800 && Key_SHIFT() == false)
		{
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
			pm01_switch=0;
		}
}

/**
	* @name		CHASSIS_Handler
  * @brief  根据给定的速度指令，计算期望的四个轮子的转速，
						并通过CAN总线发送给底盘电机控制器。
  * @param  None
  * @retval None
  */
void CHASSIS_Handler(float speed[SPEED_TYPE_NUM])
{
	Expect[Left_Front_Wheel] = 1.0f/M_R*( speed[_X]+speed[_Y]+(M_L+M_LW)*speed[_W]/2);
	Expect[Right_Front_Wheel] = 1.0f/M_R*( speed[_X]-speed[_Y]+(M_L+M_LW)*speed[_W]/2);
	Expect[Left_Rear_Wheel] = 1.0f/M_R*(-speed[_X]+speed[_Y]+(M_L+M_LW)*speed[_W]/2);
	Expect[Right_Rear_Wheel] = 1.0f/M_R*(-speed[_X]-speed[_Y]+(M_L+M_LW)*speed[_W]/2);
	
	Power_Limit(Expect,CAN1_SendData);

	CAN_TxMsg[_CAN1][_0x200].Data[0] = (uint8_t)(CAN1_SendData[Left_Front_Wheel]>>8);
	CAN_TxMsg[_CAN1][_0x200].Data[1] = (uint8_t)(CAN1_SendData[Left_Front_Wheel]);
	CAN_TxMsg[_CAN1][_0x200].Data[4] = (uint8_t)(CAN1_SendData[Right_Front_Wheel]>>8);
	CAN_TxMsg[_CAN1][_0x200].Data[5] = (uint8_t)(CAN1_SendData[Right_Front_Wheel]);
	CAN_TxMsg[_CAN1][_0x200].Data[2] = (uint8_t)(CAN1_SendData[Left_Rear_Wheel]>>8);
	CAN_TxMsg[_CAN1][_0x200].Data[3] = (uint8_t)(CAN1_SendData[Left_Rear_Wheel]);
	CAN_TxMsg[_CAN1][_0x200].Data[6] = (uint8_t)(CAN1_SendData[Right_Rear_Wheel]>>8);
	CAN_TxMsg[_CAN1][_0x200].Data[7] = (uint8_t)(CAN1_SendData[Right_Rear_Wheel]);
}


/**
	* @name		CHASSIS_Stop
  * @brief  停止底盘运动
  * @param  None
  * @retval None
  */
void CHASSIS_Stop(void)
{
	for(int i = 0; i < Gimbal_Yaw; i++)
	{
		DJI_Motor[i].pid_Speed.PID_clear(&DJI_Motor[i].pid_Speed);
	}
	
	DJI_Motor[Gimbal_Yaw].pid_Speed.PID_clear(&DJI_Motor[Gimbal_Yaw].pid_Speed);
	DJI_Motor[Gimbal_Yaw].pid_Angle.PID_clear(&DJI_Motor[Gimbal_Yaw].pid_Angle);

	for(int i = 0; i < 8; i++)
	{
		CAN_TxMsg[_CAN1][_0x200].Data[i]=0;
	}
}

/**
	* @name		RAMP_float
  * @brief  加减速运动控制,避免突然的加速或减速造成的冲击
  * @param  final, now, ramp
  * @retval 当前位置 now 经过加减速运动后的新位置
  */
float RAMP_float( float final, float now, float ramp)
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
	* @name		Chassis_Set_Pid
  * @brief  根据底盘控制模式来初始化 PID 控制器和卡尔曼滤波器等相关变量
  * @param  ctrl_mode
  * @retval NONE
  */
void Chassis_Set_Pid(Chassis_Ctrl_Mode ctrl_mode)
{
		for(int i = 0; i < Gimbal_Yaw; i++)
	{
		PID_Init_ByParamArray(&DJI_Motor[i].pid_Speed,f_PID_Wheel_Para[ctrl_mode]);
	}
//	PID_Init_ByParamArray(&DJI_Motor->pid_Speed,f_PID_Wheel_Para[ctrl_mode]);
//	PID_Init_ByParamArray(&DJI_Motor->pid_Speed,f_PID_Wheel_Para[ctrl_mode]);
//	PID_Init_ByParamArray(&DJI_Motor->pid_Speed,f_PID_Wheel_Para[ctrl_mode]);
	
	PID_Init_ByParamArray(&DJI_Motor[Gimbal_Yaw].pid_Speed,f_PID_Folo_Para[1]);
	PID_Init_ByParamArray(&DJI_Motor[Gimbal_Yaw].pid_Angle,f_PID_Folo_Para[0]);
	KalmanCreate(&Chassis_Ctrl.KF_FOLO_Angle,1,20);
}
