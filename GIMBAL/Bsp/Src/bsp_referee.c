//
// Created by Yuanbin on 22-10-3.
//
#include "crc.h"
#include "bsp_referee.h"
#include "referee_Info.h"
#include "bsp_uart.h"

#define REFEREE_DATALENGTH  200

uint8_t Referee_Rx_Buf[REFEREE_DATALENGTH];

void referee_receive_init(void)
{
		/* UART IDLE ENABLE */
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		
		UART_Start_Receive_DMA(&huart1,Referee_Rx_Buf,REFEREE_DATALENGTH);
}

void Referee_Receive_RxEvent(UART_HandleTypeDef *huart)
{
	uint16_t index=0;	//当前数据序号
	uint16_t data_length = 0;	//帧数据长度
	
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET)
	{
		
		__HAL_UART_CLEAR_IDLEFLAG(huart);
	
		__HAL_DMA_DISABLE(huart->hdmarx);

		//裁判系统接收数据选择处理
		while(Referee_Rx_Buf[index]==0XA5) //帧头SOF校验
		{
			if(Verify_CRC8_Check_Sum(&Referee_Rx_Buf[index],5) == 1)
			{
				data_length = Referee_Rx_Buf[index+2]<<8 | Referee_Rx_Buf[index+1] + 9;
				if(Verify_CRC16_Check_Sum(&Referee_Rx_Buf[index],data_length))	//CRC16校验（CRC8不必再校验）
				{
					//裁判系统数据解析
					RefereeInfo_Decode(&Referee_Rx_Buf[index]);	
				}
			}
			index += data_length;
		}
		//数据清空
		memset(Referee_Rx_Buf,0,REFEREE_DATALENGTH);
		index = 0;
		//重新填充DMA数据量
		__HAL_DMA_SET_COUNTER(huart->hdmarx,REFEREE_DATALENGTH);
	
		UART_Start_Receive_DMA(&huart1,Referee_Rx_Buf,REFEREE_DATALENGTH);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

