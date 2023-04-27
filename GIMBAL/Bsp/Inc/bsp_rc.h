//
// Created by Yuanbin on 22-10-3.
//

#ifndef BSP_RC_H
#define BSP_RC_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx.h"


/*鼠标速度最大值限制*/
#define Xmax    300
#define Ymax    300
/*鼠标滑动滤波长度*/
#define SF_LENGTH 30  


#define SBUS_RX_BUF_NUM     36u
#define RC_FRAME_LENGTH     18
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

#define SHORT_CHANGE_TIM                  5   	//ms
#define LONG_CHANGE_TIM_SHIFT             500   	//ms

/* 检测键盘按键状态 */
#define KEY_UP                    0x00
#define KEY_DOWN                  0x01


/*按键状态枚举*/
typedef enum
{
	UP,			//抬起
	SHORT_DOWN,	//短按
	DOWN,		//长按
	PRESS,		//0->1
	RELAX,		//1->0
	KEY_STATE_CNT,
}Key_Set_State_e;

/*单独按键信息*/
typedef struct 
{  
  uint16_t state_cnt;
  uint8_t State;
  Key_Set_State_e prev_State;
  bool prev_KEY_PRESS;
  bool KEY_PRESS;
}Key_Set_Info_t;

/*总体键盘按键信息*/
typedef struct
{
	Key_Set_Info_t W;
	Key_Set_Info_t S;
	Key_Set_Info_t A;
	Key_Set_Info_t D;
	Key_Set_Info_t SHIFT;
	Key_Set_Info_t CTRL;
	Key_Set_Info_t Q;
	Key_Set_Info_t E;
	Key_Set_Info_t R;
	Key_Set_Info_t F;
	Key_Set_Info_t G;
	Key_Set_Info_t Z;
	Key_Set_Info_t X;
	Key_Set_Info_t C;
	Key_Set_Info_t V;
	Key_Set_Info_t B;
	Key_Set_Info_t MOUSE_L;
	Key_Set_Info_t MOUSE_R;
}Key_Info_t;

extern Key_Info_t Key_Info;

typedef  struct
{
    struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    union
    {
        uint16_t v;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        }set;
    }key;
		
		bool rc_lost;
		uint8_t Online_Cnt;
}rc_ctrl_t;

extern rc_ctrl_t rc_ctrl;
/* Exported functions --------------------------------------------------------*/
extern void remote_control_init(void);
extern void Remote_Control_RxEvent(UART_HandleTypeDef *huart);
extern void rc_ctrl_monitor(rc_ctrl_t *rc_ctrl);
extern float Mouse_Y_Speed(void);
extern float Mouse_X_Speed(void);
extern bool Key_W(void);
extern bool Key_A(void);
extern bool Key_S(void);
extern bool Key_D(void);
extern bool Key_SHIFT(void);
extern bool Key_CTRL(void);
extern bool Key_Q(void);
extern bool Key_E(void);
extern bool Key_R(void);
extern bool Key_F(void);
extern bool Key_G(void);
extern bool Key_Z(void);
extern bool Key_X(void);
extern bool Key_C(void);
extern bool Key_V(void);
extern bool Key_B(void);
extern bool Key_mouse_l(void);
extern bool Key_mouse_r(void);

#endif //BSP_RC_H
