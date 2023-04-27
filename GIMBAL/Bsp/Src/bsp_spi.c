//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_spi.h"

/**
  * @brief  Config the SPI transfer speed
  * @param  *spi_Handler pointer to a SPI_HandleTypeDef structure that contains
  *                      the configuration information for SPI module.
  * @param  SPI_BaudRatePrescaler  SPI Communication speed
  * @retval None
  */
static void USER_SPI_SetSpeed(SPI_HandleTypeDef *spi_Handler,uint8_t SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));

    __HAL_SPI_DISABLE(spi_Handler); //关闭 SPI
    spi_Handler->Instance->CR1 &= 0XFFC7; //位 3-5 清零，用来设置波特率
    spi_Handler->Instance->CR1 |= SPI_BaudRatePrescaler;//设置 SPI 速度
    __HAL_SPI_ENABLE(spi_Handler); //使能 SPI
}


