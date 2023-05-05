//
// Created by YanYuanbin on 22-10-12.
//

#ifndef _MYUSART_H
#define _MYUSART_H

#include "stdint.h"
#include "usart.h"

#define UART1_MAX_RX_LEN    12
#define UART1_MAX_TX_LEN    14

extern uint8_t UART1_RX_BUF[UART1_MAX_RX_LEN];//定义串口1接收缓存大小为10
extern uint8_t UART1_TX_BUF[UART1_MAX_TX_LEN];//定义串口1发送缓存，大小为9

extern void myprintf(int16_t value_1,int16_t value_2);
/**
  * @brief  Starts the UART DMA RxEventCallback .
  */
extern HAL_StatusTypeDef UART_TOIDLE_Start(UART_HandleTypeDef* uartHandle,uint32_t DataLength);
/**
  * @brief  Starts the multi_buffer DMA Transfer.
  */
extern HAL_StatusTypeDef DMA_MultiBufferStart(DMA_HandleTypeDef *hdma,uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);

#endif

