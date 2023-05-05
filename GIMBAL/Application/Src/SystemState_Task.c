#include "SystemState_Task.h"
#include "cmsis_os.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"

/*--------------------------   变量声明   ------------------------------*/

ROBOT robot=
{
    .mode=INITIAL,
    .level=LV_1,
};
/*--------------------------   变量声明   ------------------------------*/
int print_mode;

void SYSTEMSTATE_TASK(void *args)
{
    uint32_t currentTime;
    
	while(1)
	{
		currentTime = xTaskGetTickCount();//当前系统时间

		if(robot.mode == INITIAL)
		{
			robot.shooter_id1_42mm.cooling_limit = 100;
			robot.shooter_id1_42mm.cooling_rate = 20;
			robot.shooter_id1_42mm.speed_limit = 10;
		}
		else if(robot.mode == BURST)
		{
			if(robot.level == LV_1)
			{
				robot.shooter_id1_42mm.cooling_limit = 200;
				robot.shooter_id1_42mm.cooling_rate = 40;
				robot.shooter_id1_42mm.speed_limit = 10;
			}else if(robot.level == LV_2)
			{
				robot.shooter_id1_42mm.cooling_limit = 350;
				robot.shooter_id1_42mm.cooling_rate = 80;
				robot.shooter_id1_42mm.speed_limit = 10;
			}else if(robot.level == LV_3)
			{
				robot.shooter_id1_42mm.cooling_limit = 500;
				robot.shooter_id1_42mm.cooling_rate = 120;
				robot.shooter_id1_42mm.speed_limit = 10;
			}
		}else if(robot.mode == RATE)
		{
			if(robot.level == LV_1)
			{
				robot.shooter_id1_42mm.cooling_limit = 100;
				robot.shooter_id1_42mm.cooling_rate = 20;
				robot.shooter_id1_42mm.speed_limit = 16;
			}else if(robot.level == LV_2)
			{
				robot.shooter_id1_42mm.cooling_limit = 200;
				robot.shooter_id1_42mm.cooling_rate = 60;
				robot.shooter_id1_42mm.speed_limit = 16;
			}else if(robot.level == LV_3)
			{
				robot.shooter_id1_42mm.cooling_limit = 300;
				robot.shooter_id1_42mm.cooling_rate = 100;
				robot.shooter_id1_42mm.speed_limit = 16;
			}
		}

		vTaskDelayUntil(&currentTime, 10);//绝对延时
	}
}
void Referee_get_Data(CAN_HandleTypeDef* CANx, CAN_RxFrameTypeDef * RxMsg)
{
	if(RxMsg->header.StdId != 0x302)
	{
		return;
	}
	
	uint8_t shooter_power = (RxMsg->data[0] & (1<<2)) >> 2;
	uint8_t mode_L = (RxMsg->data[0] & (1<<3)) >> 3, mode_H = (RxMsg->data[0] & (1<<4)) >> 4;
	uint8_t robot_LV_L =(RxMsg->data[0] & (1<<5)) >> 5,  robot_LV_H = (RxMsg->data[0] & (1<<6)) >> 6;
	uint8_t robot_id = (RxMsg->data[0] & (1<<7)) >> 7;
	robot.id = robot_id;//0红，1蓝

	if(robot_LV_H == 0&&robot_LV_L == 0)
	{
		robot.level = LV_1;
	}else if(robot_LV_H == 1&&robot_LV_L == 0)
	{
		robot.level = LV_2;
	}else if(robot_LV_H == 0&&robot_LV_L == 1)
	{
		robot.level = LV_3;
	}

	if(mode_L == 0&& mode_H == 0)
	{
		robot.mode = INITIAL;
	}else if(mode_L == 0&& mode_H == 1)
	{
		robot.mode = BURST;
	}
	else if(mode_L == 1&& mode_H == 0)
	{
		robot.mode = RATE;
	}

	robot.mains_power_shooter_output = shooter_power;
	robot.cooling_heat = (int16_t)RxMsg->data[1] << 8 | (int16_t)RxMsg->data[2];//42mm 枪口热量
	robot.bullet_speed = ((int16_t)RxMsg->data[3] << 8 | (int16_t)RxMsg->data[4])/100.f; //42mm枪口射速
	robot.shooter_id1_42mm.bullet_remaining_num_42mm = ((int16_t)RxMsg->data[5] << 8 | (int16_t)RxMsg->data[6]);//42mm剩余发射数目
}

