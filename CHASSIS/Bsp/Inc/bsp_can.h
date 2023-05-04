//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "can.h"

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

enum{
    _CAN1,
    _CAN2,
    CAN_PORT_NUM,
};

enum{
    _0x200,
    _0x1ff,
    _0x300,
    _0x301,
    _0x302,
    _0x303,
		_0x210,
    stdID_NUM,
};


/* Exported functions --------------------------------------------------------*/
extern void bsp_can_init(void);
extern void USER_CAN_TxMessage(CAN_HandleTypeDef *hcan,CAN_TxFrameTypeDef *TxHeader);
extern CAN_TxFrameTypeDef CAN_TxMsg[CAN_PORT_NUM][stdID_NUM];

#endif //BSP_CAN_H
