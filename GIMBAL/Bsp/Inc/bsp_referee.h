//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_REFEREE_H
#define BSP_REFEREE_H

#include "stm32f4xx.h"

/* Exported functions --------------------------------------------------------*/
extern void referee_receive_init(void);
extern void Referee_Receive_RxEvent(UART_HandleTypeDef *huart);

#endif //BSP_REFEREE_H
