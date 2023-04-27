//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "stdint.h"


/* Exported functions --------------------------------------------------------*/
extern void bsp_tim_init(void);
extern void Heat_Power_Ctl(uint16_t tempPWM);
extern void Cover_PWM_Ctrl(uint16_t tempPWM);

#endif //BSP_TIM_H

