#ifndef SYSTEMSTATE_TASK_H
#define SYSTEMSTATE_TASK_H

#include "FreeRTOS.h"
#include "bsp_can.h"

#define IS_STANDARD

//设备工作状态
typedef enum{
    OFF,//关闭（卸力）
    ON,//工作
    CALIBRATING,//掉线
    DEVICE_ERROR,//堵转，掉线等
    STATE_NUM,
}DEVICE_STATE;


typedef enum{
    INITIAL=0,
    BURST,//爆发
    RATE,//弹速
    ROBOT_MODE_NUM,
}ROBOT_MODE;

typedef enum{
    LV_1,
    LV_2,
    LV_3,
    ROBOT_LEVEL_NUM,
}ROBOT_LEVEL;

typedef  struct
{
	uint16_t	cooling_rate;		//枪口每秒冷却值
	uint16_t	cooling_limit;  //枪口热量上限
	uint16_t	speed_limit;    //枪口上限速度 单位 m/s
	uint16_t  bullet_remaining_num_42mm;//42mm弹丸剩余发射数目
}ROBOT_SHOOTER_T;



typedef  struct 
{
		uint8_t id; //机器人Id 0红1蓝
    ROBOT_LEVEL level; //等级
		ROBOT_MODE mode; //发射机构类型
		uint8_t mains_power_shooter_output;//主控输出情况，0 无输出 1 24V
		uint16_t cooling_heat;   //枪口热量
		float bullet_speed;//实时射速
		ROBOT_SHOOTER_T shooter_id1_42mm;		//发射机构信息
}ROBOT;

extern ROBOT robot;

void Referee_get_Data(CAN_HandleTypeDef* CANx,CAN_RxFrameTypeDef * RxMsg);

#endif



