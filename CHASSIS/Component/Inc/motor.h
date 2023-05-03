//
// Created by Yuanbin on 22-10-3.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"
#include "pid.h"

/* �쳣���ö�� */
typedef enum
{
		Motor_OFFline  = 0x00U, //�������
		Motor_ERROR_NONE = 0x01U, //���쳣
    Motor_OverTmp  = 0x02U, //����
}Motor_ErrorType_e;

/* �쳣�ṹ�� */
typedef struct
{
    uint16_t ERRORCount;
    Motor_ErrorType_e ERRORType;
}Motor_ErrorHandler_t;

/* ͨ�õ����Ϣ */
typedef struct
{
		bool init;
		uint32_t StdId;
	  CAN_TypeDef *CANx;
		int16_t  current;
		int16_t  velocity;
		int16_t  encoder;
		int16_t  last_encoder;
		float    angle;
		uint8_t  temperature;
}General_Motor_Info_t;


/* DJI�����;ö�� */
typedef enum
{
		Left_Front_Wheel,
		Right_Front_Wheel,
		Left_Rear_Wheel,
		Right_Rear_Wheel,
		Gimbal_Yaw,
    DJI_MOTOR_NUM,
}DJI_Motor_usage_e;


/* DJI�������ö�� */
typedef enum
{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;


/* DJI�����װ */
typedef struct
{
		General_Motor_Info_t Data;
    DJI_Motor_Type_e Type;
		DJI_Motor_usage_e Usage;
		Motor_ErrorHandler_t ERRORHandler;
	  PID_TypeDef_t pid_Speed,pid_Angle;
}DJI_Motor_Info_t;



/* Exported functions --------------------------------------------------------*/
extern void get_DJI_Motor_Info(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_t *DJI_Motor);
extern DJI_Motor_Info_t DJI_Motor[DJI_MOTOR_NUM];

#endif //MOTOR_H

