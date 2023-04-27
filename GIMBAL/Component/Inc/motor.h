//
// Created by Yuanbin on 22-10-3.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#include "stdbool.h"
#include "robot_ref.h"

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

#if defined(CHASSIS_BOARD)
/* DJI�����;ö�� */
typedef enum{
		Yaw_Motor,
		Left_Momentum,
		Right_Momentum,
    DJI_MOTOR_NUM,
}DJI_Motor_usage_e;

/* RMD�����;ö�� */
typedef enum{
		Left_Wheel,
		Right_Wheel,
    RMD_MOTOR_NUM,
}RMD_Motor_usage_e;

/* RMD�������ö�� */
typedef enum{
	  RMD_L9025,
    RMD_MOTOR_TYPE_NUM,
}RMD_Motor_Type_e;

/* RMD_L_9025�����װ */
typedef struct
{
		int8_t order;
		General_Motor_Info_t Data;
    RMD_Motor_Type_e Type;
		Motor_ErrorHandler_t ERRORHandler;
}RMD_L9025_Info_t;

#endif

#if defined(GIMBAL_BOARD)
/* DJI�����;ö�� */
typedef enum{
		Pitch_Motor,
		Yaw_Motor,
		Left_Friction,
		Right_Friction,
		Trigger,
    DJI_MOTOR_NUM,
}DJI_Motor_usage_e;

#endif


/* DJI�������ö�� */
typedef enum{
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
		Motor_ErrorHandler_t ERRORHandler;
}DJI_Motor_Info_t;

#if defined(CHASSIS_BOARD)
extern RMD_L9025_Info_t MT9025[2];
extern DJI_Motor_Info_t YawMotor;
#endif
#if defined(GIMBAL_BOARD)
extern DJI_Motor_Info_t Gimbal_Motor[DJI_MOTOR_NUM];
#endif

/* Exported functions --------------------------------------------------------*/
extern void get_DJI_Motor_Info(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_t *DJI_Motor);
#if defined(CHASSIS_BOARD)
extern void get_RMD_Motor_Info(uint32_t *StdId, uint8_t *rxBuf,RMD_L9025_Info_t *RMD_Motor);
#endif

#endif //MOTOR_H

