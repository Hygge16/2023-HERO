//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_can.h"
#include "motor.h"
#include "bsp_rc.h"

CAN_RxFrameTypeDef CAN_RxInstance ={0,};
CAN_TxHeaderTypeDef CAN_TxMsg[CAN_PORT_NUM][stdID_NUM]= 
{
	[_CAN1]={
				[_0x200]={
							.StdId=0x200,
							.IDE=CAN_ID_STD,
							.RTR=CAN_RTR_DATA,
							.DLC=8,
						},

				[_0x1ff]={
							.StdId=0x1ff,
							.IDE=CAN_ID_STD,
							.RTR=CAN_RTR_DATA,
							.DLC=8,
						},
			},
	
	[_CAN2]={
				[_0x302]={
							.StdId=0x302,
							.IDE=CAN_ID_STD,
							.RTR=CAN_RTR_DATA,
							.DLC=8,
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

		// ����CAN��ʶ���˲���
		if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		{	
		    Error_Handler();
		}
    // ����CAN1
    HAL_CAN_Start(&hcan1);
		// ʹ�ܽ����ж�
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		sFilterConfig.FilterBank = 14;
		sFilterConfig.SlaveStartFilterBank = 14;
		// ����CAN��ʶ���˲���
		if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
		{	
		    Error_Handler();
		}
    // ����CAN2
    HAL_CAN_Start(&hcan2);
		// ʹ�ܽ����ж�
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          �Զ���CAN����
  * @param[out]     TxHeader:CAN����֡������ָ��.
  * @retval         none
  */
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	if(TxHeader->hcan == NULL) return;
	
	uint32_t TxMailbox = 0;

//   while( HAL_CAN_GetTxMailboxesFreeLevel( TxHeader->hcan ) == 0 );
	
//���ÿ⺯��ʵ��CAN����
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->data, &TxMailbox);
}


void CAN1_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Left_Front_Wheel]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Right_Front_Wheel]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Left_Rear_Wheel]);
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Right_Rear_Wheel]);
	
}

void CAN2_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
		get_DJI_Motor_Info(StdID,rxBuf,&DJI_Motor[Gimbal_Yaw]);
	
		get_rc_ctrl_data(StdID,rxBuf,&rc_ctrl);
}

/**
 *	@brief	��д CAN RxFifo �жϽ��պ���
 *	@note	��stm32f4xx_hal_can.c��������
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	// CAN1 �����ж�
	if(hcan->Instance == CAN1)
	{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInstance.header, CAN_RxInstance.data);
			CAN1_rxDataHandler(&CAN_RxInstance.header.StdId,CAN_RxInstance.data);
	}
	// CAN2 �����ж�
	else if(hcan->Instance == CAN2)
	{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInstance.header, CAN_RxInstance.data);
			CAN2_rxDataHandler(&CAN_RxInstance.header.StdId,CAN_RxInstance.data);
	}
}