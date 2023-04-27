//
// Created by Yuanbin on 22-10-5.
//

#ifndef ASSIST_H
#define ASSIST_H

#include "stm32f4xx.h"

/*差分队列结构体*/
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //长度
    float queue[100];
    //指针
    float aver_num;//平均值

    float Diff;//差分值

    uint8_t full_flag;
	
}QueueDiff_t;

extern float f_encoder_to_angle(volatile int16_t const *encoder,float encoder_Max);
extern float f_encoder_to_radian(volatile int16_t const *encoder,float encoder_Max);
extern float f_angle_conversion(float target,float measure,float halfMax);
extern float f_get_diff(uint8_t queue_len, QueueDiff_t *Data,float add_data);
#endif //ASSIST_H
