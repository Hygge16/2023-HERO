//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_can.h"
#include "motor.h"
#include "bsp_rc.h"
#include "chassis.h"
#include "referee_Info.h"

CAN_RxFrameTypeDef CAN_RxInstance ={0,};


#if defined(CHASSIS_BOARD)
//9025单电机命令CAN发送帧包
CAN_TxFrameTypeDef MT_hcanTxFrame[2] =  {
		[Left_Wheel]={
				.hcan = &hcan1,
				.header.StdId=0x144,
				.header.IDE=CAN_ID_STD,
				.header.RTR=CAN_RTR_DATA,
				.header.DLC=8,
		},
		[Right_Wheel]={
				.hcan = &hcan1,
				.header.StdId=0x143,
				.header.IDE=CAN_ID_STD,
				.header.RTR=CAN_RTR_DATA,
				.header.DLC=8,
		},
};

CAN_TxFrameTypeDef REFEREE_TxFrame={
				.hcan = &hcan2,
				.header.StdId=0x600,
				.header.IDE=CAN_ID_STD,
				.header.RTR=CAN_RTR_DATA,
				.header.DLC=8,
};
#endif

#if defined(GIMBAL_BOARD)

CAN_TxFrameTypeDef GimbalTxFrame[3] = {
		[0]={
		.hcan = &hcan1,
		.header.StdId=0x1ff,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
		},
		[1]={
		.hcan = &hcan2,
		.header.StdId=0x1ff,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
		},
		[2]={
		.hcan = &hcan1,
		.header.StdId=0x200,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
		},
};

CAN_TxFrameTypeDef RBCTxFrame = {
		.hcan = &hcan2,
		.header.StdId=0x302,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};

CAN_TxFrameTypeDef REFEREETxFrame = {
		.hcan = &hcan2,
		.header.StdId=0x600,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
#endif

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
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	if(TxHeader->hcan == NULL) return;
	
	uint32_t TxMailbox = 0;

//   while( HAL_CAN_GetTxMailboxesFreeLevel( TxHeader->hcan ) == 0 );
	
//调用库函数实现CAN发送
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->data, &TxMailbox);
}


void CAN1_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
#if defined(CHASSIS_BOARD)
		get_RMD_Motor_Info(StdID, rxBuf,&MT9025[Left_Wheel]);
		get_RMD_Motor_Info(StdID, rxBuf,&MT9025[Right_Wheel]);
#endif
#if defined(GIMBAL_BOARD)
		get_DJI_Motor_Info(StdID, rxBuf,&Gimbal_Motor[Pitch_Motor]);
		get_DJI_Motor_Info(StdID, rxBuf,&Gimbal_Motor[Left_Friction]);
		get_DJI_Motor_Info(StdID, rxBuf,&Gimbal_Motor[Right_Friction]);
		get_DJI_Motor_Info(StdID, rxBuf,&Gimbal_Motor[Trigger]);
#endif	
}

void CAN2_rxDataHandler(uint32_t *StdID, uint8_t *rxBuf)
{
#if defined(CHASSIS_BOARD)
		get_DJI_Motor_Info(StdID,rxBuf,&YawMotor);
		get_rc_ctrl_data(StdID,rxBuf,&rc_ctrl);
#endif
#if defined(GIMBAL_BOARD)
		get_DJI_Motor_Info(StdID, rxBuf,&Gimbal_Motor[Yaw_Motor]);
		get_Referee_Data(StdID,rxBuf);
#endif	
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