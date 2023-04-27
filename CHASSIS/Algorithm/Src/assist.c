//
// Created by Yuanbin on 22-10-5.
//

#include "assist.h"
#include "arm_math.h"
#include "pid.h"

/**
  * @name   encoder_To_Angle
  * @brief  角度转换
  * @param
  *         encoder:编码器角度        encoder_Max:编码器最大值（量程）
  * @retval result_Angle:周角角度
  * @attention
*/
float f_encoder_to_angle(volatile int16_t const *encoder,float encoder_Max)
{
    return (float)*encoder / encoder_Max * 360.0f;
}

/**
  * @name   encoder_To_Radian
  * @brief  弧度转换
  * @param
  *         encoder:编码器角度        encoder_Max:编码器最大值（量程）
  * @retval result_Radian:弧度角
  * @attention
*/
float f_encoder_to_radian(volatile int16_t const *encoder,float encoder_Max)
{
    return (float)*encoder / encoder_Max * 2 * PI;
}

/**
  * @brief          角度转换
	* @param[out]     target:期望值
										measure：观测量
										halfMax:观测量最大值的一半
  * @retval         none
  */
float f_angle_conversion(float target,float measure,float halfMax)
{
	float result = 0.f;
	float angle_err = 0.f;
	
	angle_err = target - measure;
	
	if(ABS(angle_err) > halfMax)
			result =  angle_err - angle_err/ABS(angle_err)*halfMax*2.f;
	else
			result = angle_err;
	
	return result;
}

/**
* @brief  获取目标的差分(以队列的逻辑)
* @param  uint8_t QueueObj float
* @return float
*/
float f_get_diff(uint8_t queue_len, QueueDiff_t *Data,float add_data)
{
    if(queue_len >= Data->queueLength)
        queue_len = Data->queueLength;
    //防止溢出
    Data->queueTotal -= Data->queue[Data->nowLength];
    Data->queueTotal += add_data;

    Data->queue[Data->nowLength]=add_data;
			
    Data->nowLength++;

    if(Data->full_flag==0)//初始队列未满
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }else if(Data->full_flag == 1)
		{
	    Data->aver_num=(Data->queueTotal)/queue_len;	
		}
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;
    return Data->Diff;
}
