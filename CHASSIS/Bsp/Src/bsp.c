//
// Created by Yuanbin on 22-10-2.
//
/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "bsp_rc.h"
#include "bsp_can.h"

/**
  * @brief  config board periph
  * @retval None
  */
void BSP_Init(void)
{

		/* config remote control transfer*/
		remote_control_init();
	
//		referee_receive_init();
	
		/* config the can module transfer*/
		bsp_can_init();
		
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
//	if(huart->Instance == USART3)Remote_Control_RxEvent(huart);
	
//	else if(huart->Instance == USART1)Referee_Receive_RxEvent(huart);
	
}


