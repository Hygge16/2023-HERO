//
// Created by Yuanbin on 22-03-31.
//
#include "bsp_serialplot.h"
#include "bsp_uart.h"

#define SERIALPlOT_TXLENGTH 6

uint8_t SERIALPLOT_TXBUF[SERIALPlOT_TXLENGTH]={0xAA,0xBB,};


/**
  * @brief  printf the variables waveforms
  * @param  value_1,value_2   specified variables
  * @retval NULL
  */
void myprintf(int16_t value_1,int16_t value_2)
{	
    SERIALPLOT_TXBUF[2] = (uint8_t)(value_1 >> 8);
    SERIALPLOT_TXBUF[3] = (uint8_t)(value_1);
    SERIALPLOT_TXBUF[4] = (uint8_t)(value_2 >> 8);
    SERIALPLOT_TXBUF[5] = (uint8_t)(value_2);

//		HAL_UART_Transmit_DMA(&huart1,SERIALPLOT_TXBUF,SERIALPlOT_TXLENGTH);
}



