//
// Created by Yuanbin on 22-10-3.
//
#include "bsp_rc.h"

#include "bsp_uart.h"
#include "string.h"
#include "pid.h"
#include "kalman.h"

rc_ctrl_t rc_ctrl;
Key_Info_t Key_Info;
static uint8_t SBUS_RX_BUF[2][SBUS_RX_BUF_NUM];

extKalman_t KF_Mouse_X_Speed,KF_Mouse_Y_Speed;
static float sliding_mouse_x[SF_LENGTH]={0.f,},sliding_mouse_y[SF_LENGTH]={0.f};

void remote_control_init(void)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA    
    do{
        __HAL_DMA_DISABLE(huart3.hdmarx);
    }while(huart3.hdmarx->Instance->CR & DMA_SxCR_EN);

    huart3.hdmarx->Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    huart3.hdmarx->Instance->M0AR = (uint32_t)(SBUS_RX_BUF[0]);
    //memory buffer 2
    //内存缓冲区2
    huart3.hdmarx->Instance->M1AR = (uint32_t)(SBUS_RX_BUF[1]);
    //data length
    //数据长度
    huart3.hdmarx->Instance->NDTR = SBUS_RX_BUF_NUM;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(huart3.hdmarx->Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(huart3.hdmarx);
	
		KalmanCreate(&KF_Mouse_X_Speed,1,60);
    KalmanCreate(&KF_Mouse_Y_Speed,1,60);
}
/**
  * @brief          遥控器数据保护
  * @param[out]     rc_ctrl_offline:遥控器信息变量指针.
  * @retval         none
  */
static void rc_ctrl_protect(rc_ctrl_t *rc_ctrl_offline)
{
	if(rc_ctrl_offline == NULL)
	{
		return;
	}
//清空离线的遥控器数据
	memset(rc_ctrl_offline,0,sizeof(*rc_ctrl_offline));
}

/**
  * @brief          遥控器状态机
  * @param[out]     rc_ctrl:遥控器信息变量指针.
  * @retval         none
  */
void rc_ctrl_monitor(rc_ctrl_t *rc_ctrl)
{
	if(rc_ctrl == NULL)
	{
		return;
	}
//计数小于50，认为遥控器离线
	if(rc_ctrl->Online_Cnt <= 50)
	{
		rc_ctrl->rc_lost = 1;
//遥控器数据保护
		rc_ctrl_protect(rc_ctrl);
	}else
	{
		rc_ctrl->rc_lost = 0;		
	}
//计数自减，在接收中断内重新赋值
	if(rc_ctrl->Online_Cnt > 0)
	{
		rc_ctrl->Online_Cnt--;
	}
}
/**
  * @brief  Transform the DMA data to remote control infomation.
  * @param  *sbus_buf   The source memory Buffer address  
  * @param  *rc_ctrl    pointer to a rc_ctrl_t structure
  * @retval NULL
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, rc_ctrl_t  *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    rc_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    rc_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    rc_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    rc_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    rc_ctrl->rc.ch[4] = (  sbus_buf[16] 	 | (sbus_buf[17] << 8) ) & 0x07ff;                 			  //!< Channel 4
    /* Switch left, right */
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);      \
	//!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
		rc_ctrl->Online_Cnt = 250;
}

/**
  * @brief  remote control uart dma receive callback
  * @param  *huart   pointer to a UART_HandleTypeDef structure  
  * @retval NULL
  */
void Remote_Control_RxEvent(UART_HandleTypeDef *huart)
{
    uint16_t this_time_rx_len = 0;
		
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
		{
      __HAL_UART_CLEAR_IDLEFLAG(huart);
			/* Current memory buffer used is Memory 0 */
			if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT )== 0U)
			{
					//失能DMA中断
					__HAL_DMA_DISABLE(huart->hdmarx);
					//得到当前剩余数据长度
					this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart->hdmarx);
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
					huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
					__HAL_DMA_ENABLE(huart->hdmarx);
					if(this_time_rx_len == RC_FRAME_LENGTH)
					{
							//处理遥控器数据
							sbus_to_rc(SBUS_RX_BUF[0], &rc_ctrl);
					}
			}else
			{
					//失能DMA中断
					__HAL_DMA_DISABLE(huart->hdmarx);
					//得到当前剩余数据长度
					this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart->hdmarx);
					__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
					huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
					__HAL_DMA_ENABLE(huart->hdmarx);
					if(this_time_rx_len == RC_FRAME_LENGTH)
					{
							//处理遥控器数据
							sbus_to_rc(SBUS_RX_BUF[1], &rc_ctrl);
					}
			}
		}
}

/**
 * @brief 滑动滤波
 * @note  
 * @param 
 */
static float Sliding_Filter(float t,float *slideFilter)
{
    float res = 0.f;

    for(int i = SF_LENGTH-1; i > 0; i-- )
    {
        slideFilter[i] = slideFilter[i-1];
    }
    slideFilter[0] = t;
    for(int i = 0; i < SF_LENGTH-1; i++)
    {
        res +=slideFilter[i];
    }
    return (res/SF_LENGTH);
}

float Mouse_X_Speed(void)
{
    float res=0.f;

    if(ABS(rc_ctrl.mouse.x) > Xmax){
        res = 0;
    }else{
        res = Sliding_Filter(KalmanFilter(&KF_Mouse_X_Speed,(float)rc_ctrl.mouse.x),sliding_mouse_x);
    } 
    return (float)res;
}
float Mouse_Y_Speed(void)
{
    float res=0.f;

    if(ABS(rc_ctrl.mouse.y) > Ymax){
        res = 0;
    }else{
        res = Sliding_Filter(KalmanFilter(&KF_Mouse_Y_Speed,(float)rc_ctrl.mouse.y),sliding_mouse_y);
    }
    return (float)res;
}


/**
 * @brief 判断按键是否按下
 * @note 
 * @param 
 */
static void FirstGetInto_KEY_PRESS(Key_Set_Info_t *key_set_Info)
{
  if(key_set_Info->prev_KEY_PRESS != key_set_Info->KEY_PRESS)
  {
    key_set_Info->state_cnt = 0;
    key_set_Info->prev_KEY_PRESS = key_set_Info->KEY_PRESS;
  }
}
static void KEY_State_Judge(Key_Set_Info_t *key_set_Info , uint8_t KEY_PRESS , int change_tim ,int long_change_tim)
{
    key_set_Info->KEY_PRESS = KEY_PRESS;
    FirstGetInto_KEY_PRESS(key_set_Info);
    if(KEY_PRESS == KEY_UP)
    {
        if(key_set_Info->prev_State != UP)
        {
            key_set_Info->state_cnt++;
            if(key_set_Info->state_cnt >= change_tim + 1)
            {
                key_set_Info->State = UP;
                key_set_Info->prev_State = UP;
            }
            else if(key_set_Info->state_cnt >= change_tim)
            {
                key_set_Info->State = RELAX;
                key_set_Info->prev_State = RELAX;
            }
        }else
        {
            key_set_Info->state_cnt = 0;
        }
    }
    else if(KEY_PRESS == KEY_DOWN)
    {
        if(key_set_Info->prev_State != DOWN) 
        {
            key_set_Info->state_cnt++;
            if(key_set_Info->state_cnt >= long_change_tim)  
            {
                key_set_Info->State = DOWN;
                key_set_Info->prev_State = DOWN;
            }
            else if(key_set_Info->state_cnt >= change_tim + 1)
            {
                key_set_Info->State = SHORT_DOWN;
                key_set_Info->prev_State = SHORT_DOWN;
            }
            else if(key_set_Info->state_cnt >= change_tim)  
            {
				key_set_Info->State = PRESS;
				key_set_Info->prev_State = PRESS;
            }
        }else
        {
            key_set_Info->state_cnt = 0;
        }
    }
}

bool Key_W(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.W ,rc_ctrl.key.set.W, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.W.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_S(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.S ,rc_ctrl.key.set.S, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.S.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_A(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.A ,rc_ctrl.key.set.A, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.A.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_D(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.D ,rc_ctrl.key.set.D, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.D.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_CTRL(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.CTRL ,rc_ctrl.key.set.CTRL, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.CTRL.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_SHIFT(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.SHIFT ,rc_ctrl.key.set.SHIFT, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.SHIFT.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}

bool Key_Q(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.Q ,rc_ctrl.key.set.Q, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.Q.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_E(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.E ,rc_ctrl.key.set.E, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.E.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_R(void)
{
	
	bool res = false;

	KEY_State_Judge(&Key_Info.R ,rc_ctrl.key.set.R, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.R.State)
	{
	case UP:
	break;
	case PRESS:
		res = true;
	break;
	case SHORT_DOWN:
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_F(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.F ,rc_ctrl.key.set.F, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.F.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_G(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.G ,rc_ctrl.key.set.G, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.G.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_Z(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.Z ,rc_ctrl.key.set.Z, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.Z.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_X(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.X ,rc_ctrl.key.set.X, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.X.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_C(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.C ,rc_ctrl.key.set.C, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.C.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_V(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.V ,rc_ctrl.key.set.V, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.V.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_B(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.B ,rc_ctrl.key.set.B, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.B.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_mouse_l(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.MOUSE_L ,rc_ctrl.mouse.press_l, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.MOUSE_L.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}
bool Key_mouse_r(void)
{
	static bool res = false;

	KEY_State_Judge(&Key_Info.MOUSE_R ,rc_ctrl.mouse.press_r, SHORT_CHANGE_TIM , LONG_CHANGE_TIM_SHIFT);

	switch(Key_Info.MOUSE_R.State)
	{
	case UP:
	break;
	case PRESS:
	break;
	case SHORT_DOWN:
		res = true;
	break;
	case DOWN:
	break;
	case RELAX:
		res = false;
	break;
	}
	return res;
}