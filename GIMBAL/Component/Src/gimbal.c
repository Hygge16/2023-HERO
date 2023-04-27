//
// Created by Yuanbin on 22-10-3.
//

#include "robot_ref.h"

#if defined(GIMBAL_BOARD)

#include "gimbal.h"
#include "bsp_rc.h"
#include "pid.h"
#include "motor.h"


/**
  * @brief          云台模式设置
  * @param[out]     Gimbal_Info:云台信息变量指针.
  * @retval         none
  */
void gimbal_mode_set(Gimbal_Info_t *Gimbal_Info)
{
		if(Gimbal_Info == NULL) return;
		
		//左拨杆位于中位，则为正面对敌模式
		if(Gimbal_Info->rc_ctrl->rc.s[1] == 3)
		{
				Gimbal_Info->mode = Gimbal_INS;
		//右拨杆位于低位，则为遥控击发模式
		}else if(Gimbal_Info->rc_ctrl->rc.s[1] == 2)
		{
				Gimbal_Info->mode = Gimbal_Rcshoot;		
		}
		//右拨杆位于高位或离线，则为无力模式
		else
		{
				Gimbal_Info->mode = Gimbal_WEEK;
		}
}



/**
  * @brief          云台信息更新
  * @param[out]     Gimbal_Info:云台信息变量指针.
  * @retval         none
  */
void gimbal_Info_update(Gimbal_Info_t *Gimbal_Info)
{
		if(Gimbal_Info == NULL) return;
	
		//更新期望角
		if(Gimbal_WEEK != Gimbal_Info->mode){
				Gimbal_Info->Target.pit_angle += (rc_ctrl.rc.s[1]!=2)*rc_ctrl.rc.ch[1]*0.0001f - Mouse_Y_Speed() * 0.002f;
				Gimbal_Info->Target.yaw_angle -= (rc_ctrl.rc.s[1]!=2)*rc_ctrl.rc.ch[0]*0.0003f + Mouse_X_Speed() * 0.0015f;
		}
		else{
				Gimbal_Info->Target.pit_angle = *Gimbal_Info->Measure.pit_angle;
				Gimbal_Info->Target.yaw_angle = *Gimbal_Info->Measure.yaw_angle;
		}
		//期望角限幅
		VAL_LIMIT(Gimbal_Info->Target.pit_angle,Gimbal_Info->Limit_pitch.min,Gimbal_Info->Limit_pitch.max);
}


/**
  * @brief          遥控器数据共享
	* @param[out]     rc_ctrl:遥控器信息变量指针, TxHeader:CAN发送帧包变量指针
  * @retval         none
  */
void rc_communicate(rc_ctrl_t *rc_ctrl,CAN_TxFrameTypeDef *TxHeader)
{
	if(rc_ctrl == NULL || TxHeader == NULL) return;
	
	TxHeader->data[0] = (uint8_t)(rc_ctrl->rc.s[0] << 6) | (uint8_t)(rc_ctrl->rc.s[1] << 4) | 
											(uint8_t)(rc_ctrl->rc_lost << 3 )| (uint8_t)(rc_ctrl->rc.ch[4]==660) << 2 ;
	
	TxHeader->data[2] = (uint8_t)(rc_ctrl->rc.ch[3] >> 8);
	TxHeader->data[3] = (uint8_t)(rc_ctrl->rc.ch[3]);
	TxHeader->data[4] = (uint8_t)(rc_ctrl->rc.ch[2] >> 8);
	TxHeader->data[5] = (uint8_t)(rc_ctrl->rc.ch[2]);
	TxHeader->data[6] = (uint8_t)(rc_ctrl->key.v >> 8);
	TxHeader->data[7] = (uint8_t)(rc_ctrl->key.v);

	USER_CAN_TxMessage(TxHeader);
}

#endif


