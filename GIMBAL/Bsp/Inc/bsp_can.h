//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"
#include "robot_ref.h"

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t				data[8];
}CAN_TxFrameTypeDef;


/* Exported functions --------------------------------------------------------*/
extern void bsp_can_init(void);
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);
#endif //BSP_CAN_H
