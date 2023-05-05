
#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "stm32f4xx.h"
#include "bsp_rc.h"
#include "bsp_can.h"
#include "SystemState_Task.h"
#include "vision.h"
#include "Vision_Task.h"
#include "Shoot_Task.h"
#include "Gimbal_Task.h"


typedef enum
{
	CHASSIS_INVA = 0x00U, //Ð¶  Á¦
	CHASSIS_FOLO = 0X01U, //¸ú  Ëæ
	CHASSIS_SPIN = 0X02U, //Ð¡ÍÓÂÝ
	CHASSIS_MODE_NUM,
}Chassis_Mode_e;

typedef struct
{
	  uint8_t state;

    Chassis_Mode_e mode;
		
		uint8_t ctrl;

    CAN_TxFrameTypeDef *txMsg;
    
    int16_t vx,vy,key;

    Rc_Ctrl_t *dr16;
	
		void (*mode_Setup)(Chassis_Mode_e);
	  void (*state_Setup)(DEVICE_STATE);

}Chassis_Info_t;

typedef struct
{
	uint8_t shoot_ready;
	uint8_t auto_ready;
	
	CAN_TxFrameTypeDef *txMsg;
}Head;

extern Chassis_Info_t Chassis_Ctrl;
extern Head head;

static void chassis_State_Handoff(DEVICE_STATE mode);
static void chassis_Mode_Handoff(Chassis_Mode_e mode);
static void Chassis_ctrl(void);


#endif

