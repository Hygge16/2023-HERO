//
// Created by Yuanbin on 22-10-2.
//
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

#include "robot_ref.h"

#include "bsp_tim.h"
#include "bsp_rc.h"
#include "bsp_can.h"
#include "bsp_referee.h"


#include "BMI088_Driver.h"
#include "IST8310_Driver.h"


/**
  * @brief  config board periph
  * @retval None
  */
void BSP_Init(void)
{
		/* starts the pwm signal generation. */
		bsp_tim_init();

#if defined(GIMBAL_BOARD)
		/* config remote control transfer*/
		remote_control_init();
#endif
	
#if defined(CHASSIS_BOARD)
		referee_receive_init();
#endif
	
		/* config the can module transfer*/
		bsp_can_init();
		
		/* config bmi088 transfer */
		while(BMI088_init())  
		{
				HAL_Delay(100);
		}
		/* config ist8310 transfer */
//    while(ist8310_init())
//    {
//        HAL_Delay(100);
//    }
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART3)Remote_Control_RxEvent(huart);
	
//	else if(huart->Instance == USART1)Referee_Receive_RxEvent(huart);
	
}


