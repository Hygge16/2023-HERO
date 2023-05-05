#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#include "stdbool.h"
#include "robot_ref.h"
#include "pid.h"
#include "bsp_can.h"

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
		uint8_t stalled;
}General_Motor_Info_t;

/* DJI�����;ö�� */
typedef enum
{
		Gimbal_Yaw,
		Gimbal_Pitch,
		Left_Shoot,
		Right_Shoot,
		Trigger,
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

typedef enum
{
    LOW,
    MID,
    HIGH,
    CURRENT_LEVEL_NUM,
}MOTOR_CURRENT_LEVEL;

/* DJI�����װ */
typedef struct
{
		General_Motor_Info_t Data;
    DJI_Motor_Type_e Type;
		DJI_Motor_usage_e Usage;
	  PID_TypeDef_t pid_Speed,pid_Angle;
    CAN_TxFrameTypeDef* txMsg;
		Motor_ErrorHandler_t ERRORHandler;
		uint8_t stalled;
		MOTOR_CURRENT_LEVEL I_Level;
}DJI_Motor_Info_t;

extern DJI_Motor_Info_t DJI_Motor[DJI_MOTOR_NUM];

/* Exported functions --------------------------------------------------------*/
extern void get_DJI_Motor_Info(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_t *DJI_Motor);



#endif //MOTOR_H

