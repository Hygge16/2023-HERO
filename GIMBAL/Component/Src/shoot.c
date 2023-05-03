//
// Created by Yuanbin on 22-10-3.
//
#include "robot_ref.h"


#include "shoot.h"

#include "pid.h"

#include "motor.h"


/**
 * @brief 检查射速
 * @note  摩擦轮变速，动态调节转速
 */
float Temp_Fix_speed(uint16_t real_speed)
{
  float temp_scope = 50;//假设变化范围为50摄氏度
  float temp_low = 35;//初始温度设定为35摄氏度
  float res = 0;
  float speed_bap = 0;
  
  if(13.5f<real_speed<=15.f)
  {
	speed_bap = -50.f;
  }else if(16.5f<real_speed<=18.f)
  {
	speed_bap = -70.f;
  }else if(27.5f<real_speed<=30.f)
  {
	speed_bap = -170.f;
  }
  	return res;
}

float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num)
{
	float res=0;
	uint8_t SpeedErr_cnt=0;

  if(real_S < min_S && real_S > 8)SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )SpeedErr_cnt = 0;
	
  if(SpeedErr_cnt == 1)//射速偏低
  {
    SpeedErr_cnt = 0;
    res += up_num;
  }
  if(real_S > max_S)//射速偏高
    res -= down_num;
  return res;
}

//卡弹判断
bool Judge_IF_SingeStuck(float angle_err)
{
	if(ABS(angle_err) >= 360.f/13.f)return true;
	
	return false;
}
