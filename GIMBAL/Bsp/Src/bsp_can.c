//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_can.h"
#include "motor.h"
#include "bsp_rc.h"
#include "chassis.h"
#include "referee_Info.h"

int L[8] = { 0,2,4,6 , 0,2,4,6 };
int H[8] = { 1,3,5,7 , 1,3,5,7 };

CAN_RxFrameTypeDef CAN_RxInstance ={0,};
CAN_TxFrameTypeDef CAN_TxMsg[CAN_PORT_NUM][stdID_NUM]= 
{
	[_CAN1]={
				[_0x200]={
							.header.StdId=0x200,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},

				[_0x1ff]={
							.header.StdId=0x1ff,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},
			},
	[_CAN2]={
				[_0x200]={
							.header.StdId=0x200,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},
		
				[_0x1ff]={
							.header.StdId=0x1ff,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},

				[_0x300]={
							.header.StdId=0x300,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},
				[_0x301]={
							.header.StdId=0x301,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
						},
			},
};

/**
  * @brief  Configures the CAN Module.
  * @param  None
  * @retval None
  */
void bsp_can_init(void)
{
    CAN_FilterTypeDef sFilterConfig={0};

		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.SlaveStartFilterBank = 0;

		// 配置CAN标识符滤波器
		if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		{	
		    Error_Handler();
		}
    // 开启CAN1
    HAL_CAN_Start(&hcan1);
		// 使能接收中断
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		sFilterConfig.SlaveStartFilterBank = 14;
    // 开启CAN2
    HAL_CAN_Start(&hcan2);
		// 使能接收中断
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          自定义CAN发送
  * @param[out]     TxHeader:CAN发送帧包变量指针.
  * @retval         none
  */
void USER_CAN_TxMessage(CAN_HandleTypeDef *hcan,CAN_TxFrameTypeDef *TxHeader)
{
	if(TxHeader->hcan == NULL) return;
	
	uint32_t TxMailbox = 0;

   while( HAL_CAN_GetTxMailboxesFreeLevel( TxHeader->hcan ) == 0 );
	
//调用库函数实现CAN发送
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->data, &TxMailbox);
}


void CAN1_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Gimbal_Pitch]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Left_Shoot]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Right_Shoot]);
}
void CAN2_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Gimbal_Yaw]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Trigger]);
}

/**
 *	@brief	重写 CAN RxFifo 中断接收函数
 *	@note	在stm32f4xx_hal_can.c中弱定义
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	// CAN1 接收中断
	if(hcan->Instance == CAN1)
	{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInstance.header, CAN_RxInstance.data);
			CAN1_rxDataHandler(&CAN_RxInstance.header.StdId,CAN_RxInstance.data);
	}
	// CAN2 接收中断
	else if(hcan->Instance == CAN2)
	{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInstance.header, CAN_RxInstance.data);
			CAN2_rxDataHandler(&CAN_RxInstance.header.StdId,CAN_RxInstance.data);
	}
}