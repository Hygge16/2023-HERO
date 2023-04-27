//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_tim.h"

#include "tim.h"

/**
  * @brief  Starts the PWM signal generation.
  * @param  None
  * @retval None
  */
void bsp_tim_init(void)
{
	//Heat_Power_Tim init
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	
	//Cover 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2200);
}

/**
  * @brief  Bmi088 Heat Power PWM Control
  * @param  None
  * @retval None
  */
void Heat_Power_Ctl(uint16_t tempPWM)
{
	  __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
}
/**
  * @brief  Cover PWM Control
  * @param  None
  * @retval None
  */
void Cover_PWM_Ctrl(uint16_t tempPWM)
{
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, tempPWM);
}
