//
// Created by Yuanbin on 22-3-30.
//
#include "motor.h"
#include "pid.h"

DJI_Motor_Info_t DJI_Motor[DJI_MOTOR_NUM]={
		[Gimbal_Yaw]={
				.Usage = Gimbal_Yaw,
				.Data.StdId = 0x205,
				.Data.CANx = CAN2,
				.Type = DJI_GM6020,
				.stalled = 0,
				.txMsg = &CAN_TxMsg[_CAN2][_0x1ff],
		},
		
		[Gimbal_Pitch]={
				.Usage = Gimbal_Pitch,
				.Data.StdId = 0x206,
				.Data.CANx = CAN1,
				.Type = DJI_GM6020,
				.stalled = 0,
				.txMsg = &CAN_TxMsg[_CAN1][_0x1ff],
		},
		
		[Left_Shoot]={
			.Usage = Left_Shoot,
				.Data.StdId = 0x201,
				.Data.CANx = CAN1,
				.Type = DJI_M3508,
				.stalled = 0,
				.txMsg = &CAN_TxMsg[_CAN1][_0x200],
		},
				
		[Right_Shoot]={
				.Usage = Right_Shoot,
				.Data.StdId = 0x202,
				.Data.CANx = CAN1,
				.Type = DJI_M3508,
				.stalled = 0,
				.txMsg = &CAN_TxMsg[_CAN1][_0x200],
		},
						
		[Trigger]={
				.Usage = Trigger,
				.Data.StdId = 0x208,
				.Data.CANx = CAN2,
				.Type = DJI_M3508,
				.stalled = 0,
				.txMsg=&CAN_TxMsg[_CAN2][_0x1ff],
		},

};
/**
  * @brief  transform the encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a General_Motor_Info_t structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval anglesum
  */
static float encoder_to_anglesum(General_Motor_Info_t *Info,float torque_ratio,uint16_t MAXencoder)
{
		int16_t res1 = 0,res2 =0;
		
		if(Info == NULL) return 0;
		
		/* check the motor init */
		if(Info->init != true)
		{
			/* config the init flag */
			Info->init = true;
			/* update the last encoder */
			Info->last_encoder = Info->encoder;
			/* reset the angle */
			Info->angle = 0;
		}
		
		/* get the possiable min encoder err */
		if(Info->encoder < Info->last_encoder)
		{
        res1 = Info->encoder-Info->last_encoder + MAXencoder;
		}
		else if(Info->encoder > Info->last_encoder)
		{
        res1 = Info->encoder-Info->last_encoder - MAXencoder;
		}
    res2 = Info->encoder-Info->last_encoder;
		
		/* update the last encoder */
		Info->last_encoder = Info->encoder;
		
		/* transforms the encoder data to tolangle */
		ABS(res1) > ABS(res2) ? (Info->angle += (float)res2/(MAXencoder*torque_ratio)*360.f) : (Info->angle += (float)res1/(MAXencoder*torque_ratio)*360.f);
		
		return Info->angle;
}

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)return Input;
		
		float len = 0.f;

    if (Input > maxValue)
    {
        len = maxValue - minValue;    
        do{
            Input -= len;
        }while (Input > maxValue);
    }
    else if (Input < minValue)
    {
        len = maxValue - minValue;
        do{
            Input += len;
        }while (Input < minValue);
    }
    return Input;
}

/**
  * @brief  transform the encoder(0-8192) to angle(-180-180)
  * @param  *Info        pointer to a General_Motor_Info_t structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXencoder   the specified motor max encoder number
  * @retval angle
  */
static float encoder_to_angle(General_Motor_Info_t *Info,float torque_ratio,uint16_t MAXencoder)
{
	
		if(Info == NULL) return 0.f;
	
		float encoder_err = 0.f;
		
		/* check the motor init */
		if(Info->init != true)
		{
			/* config the init flag */
			Info->init = true;
			/* update the last encoder */
			Info->last_encoder = Info->encoder;
			/* reset the angle */
			Info->angle = 0;
		}
		
		encoder_err = Info->encoder - Info->last_encoder;
		
		/* 0�� -> MAXencoder */		
		if(encoder_err > MAXencoder*0.5f)
		{
			Info->angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
		}
		/* MAXencoder�� -> 0 */		
		else if(encoder_err < -MAXencoder*0.5f)
		{
			Info->angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
		}
		/* δ��0�� */
		else
		{
			Info->angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;
		}
		
		/* update the last encoder */
		Info->last_encoder = Info->encoder;
		
		/* loop constrain */
		f_loop_constrain(Info->angle,-180,180);

		return Info->angle;
}

/** @brief  Check the DJI Motor state
  * @param  *DJI_Motor pointer to a DJI_Motor_Info_t structure that contains
  *                    the configuration information for the specified motor.  
  * @retval None
  */
static void DJI_Motor_ErrorHandle(DJI_Motor_Info_t *DJI_Motor)
{
	/* check the dji motor temperature */
	if(DJI_Motor->Data.temperature > 80)
	{
			DJI_Motor->ERRORHandler.ERRORCount++;
		
			if(DJI_Motor->ERRORHandler.ERRORCount > 200)
			{
				DJI_Motor->ERRORHandler.ERRORCount = 0;
				DJI_Motor->ERRORHandler.ERRORType = Motor_OverTmp;
			}
	}else
	{
			DJI_Motor->ERRORHandler.ERRORCount = 0;	
	}
}

/**
  * @brief  transform the DJI motor receive data
  * @param  StdId  specifies the standard identifier.
  * @param  *rxBuf can receive memory address
  * @param  *DJI_Motor pointer to a DJI_Motor_Info_t structure that contains the information of DJI motor
  * @retval None
  */
void get_DJI_Motor_Info(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_t *DJI_Motor)
{
	/* check the StdId */
	if(*StdId != DJI_Motor->Data.StdId) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.temperature = rxBuf[6];
	DJI_Motor->Data.encoder  = ((int16_t)rxBuf[0]<<8 | (int16_t)rxBuf[1]);
	DJI_Motor->Data.velocity = ((int16_t)rxBuf[2]<<8 | (int16_t)rxBuf[3]);
	DJI_Motor->Data.current  = ((int16_t)rxBuf[4]<<8 | (int16_t)rxBuf[5]);
	
	/* check the motor err	*/
	DJI_Motor_ErrorHandle(DJI_Motor);
	
	/* transform the encoder to anglesum */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,1.f,8192);
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,3591.f/3060.f,8192);//3591.f/187.f
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,36.f,8192);
		break;
		
		default:break;
	}
}

