//
// Created by Yuanbin on 22-10-3.
//

#ifndef SHOOT_H
#define SHOOT_H

#include "stdint.h"
#include "bsp_rc.h"
#include "referee_Info.h"

#define SHOOT_SPEED_15M_S    4650
#define SHOOT_SPEED_18M_S    5100
#define SHOOT_SPEED_30M_S    7450

#define TRIGGER_FREQ_2_HZ     750
#define TRIGGER_FREQ_3_HZ     1000
#define TRIGGER_FREQ_4_HZ     1250
#define TRIGGER_FREQ_5_HZ     1500
#define TRIGGER_FREQ_6_HZ     1750
#define TRIGGER_FREQ_7_HZ     2000
#define TRIGGER_FREQ_8_HZ     2400
#define TRIGGER_FREQ_9_HZ     2700
#define TRIGGER_FREQ_10_HZ    3000

typedef enum{
    AI,
    AUTO,
    SINGLE,
    SHOOT_MODE_NUM,
}Shoot_Mode_e;


typedef struct 
{
    Shoot_Mode_e mode;
	
    uint8_t trigger_Buf;
	
		int8_t stuck_flag;
	
    int bulletNum;//·¢Éä¼ÆÊý
	
    float trigger_Angle;
	
    float wheel_Speed[ROBOT_MODE_NUM][ROBOT_LEVEL_NUM];
	
		rc_ctrl_t *rc_ctrl;
	
}Shoot_Info_t;

/* Exported functions --------------------------------------------------------*/
extern float Temp_Fix_speed(uint16_t real_speed);
extern float SpeedAdapt(float real_S , float min_S, float max_S,float up_num , float down_num);
extern bool Judge_IF_SingeStuck(float angle_err);
extern bool Judge_IF_AutoBlock(float speed_err);

#endif //SHOOT_H
