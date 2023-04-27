//
// Created by Yuanbin on 22-10-3.
//
#include "referee_Info.h"

ext_game_status_t                   game_state;					          //0x0001
ext_game_robot_HP_t                 game_robot_HP;                //0x0003
ext_game_robot_status_t             game_robot_state;             //0x0201
ext_power_heat_data_t 	        	  power_heat_data;	      	    //0x0202
ext_buff_t                          buff_data;                    //0x0204
ext_shoot_data_t                    real_shoot_data;              //0x0207
projectile_allowance_t              bullet_remaining;             //0x0208

ext_shoot_t Shooter_BURST[ROBOT_LEVEL_NUM]=
{
	[LV_1]={
		.cooling_limit = 150,
		.cooling_rate = 15,
		.speed_limit = 15,
	},
	[LV_2]={
		.cooling_limit = 280,
		.cooling_rate = 25,
		.speed_limit = 15,
	},
	[LV_3]={
		.cooling_limit = 400,
		.cooling_rate = 35,
		.speed_limit = 15,
	},
};
ext_shoot_t Shooter_COOLING[ROBOT_LEVEL_NUM]=
{
	[LV_1]={
		.cooling_limit = 50,
		.cooling_rate = 40,
		.speed_limit = 15,
	},
	[LV_2]={
		.cooling_limit = 100,
		.cooling_rate = 60,
		.speed_limit = 18,
	},
	[LV_3]={
		.cooling_limit = 150,
		.cooling_rate = 80,
		.speed_limit = 18,
	},
};
ext_shoot_t Shooter_RATE[ROBOT_LEVEL_NUM]=
{
	[LV_1]={
		.cooling_limit = 75,
		.cooling_rate = 15,
		.speed_limit = 30,
	},
	[LV_2]={
		.cooling_limit = 150,
		.cooling_rate = 25,
		.speed_limit = 30,
	},
	[LV_3]={
		.cooling_limit = 200,
		.cooling_rate = 35,
		.speed_limit = 30,
	},
};

ROBOT_Info_t robot=
{
	.mode=INITIAL,
	.level=LV_1,
	.shooter_id1_17mm.cooling_limit = 50,
	.shooter_id1_17mm.cooling_rate = 10,
	.shooter_id1_17mm.speed_limit = 15,
};

/*
*@title：float8位转32位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
static float bit8TObit32(uint8_t change_info[4])
{
	union
	{
    	float f;
			char  byte[4];
	}u32val;
		
	memcpy(u32val.byte,change_info,4*sizeof(uint8_t));
	
	return u32val.f;
}

/*
*@title：int32位转8位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
static char bit32TObit8(int index_need,int bit32)
{
	union
	{
    float  f;
		char  byte[4];
	}u32val;
	
  u32val.f = bit32;
	
	return u32val.byte[index_need];
}
/*
*@title：int16_t8位转16位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
static int16_t bit8TObit16(uint8_t *change_info)
{
	union
	{
    int16_t f;
		char  byte[2];
	}u16val;
	
	memcpy(u16val.byte,change_info,2*sizeof(uint8_t));

	return u16val.f;
}
/*
*@title：int16位转8位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
static char bit16TObit8(int index_need,int bit16)
{
	union
	{
    float  f;
		char  byte[2];
	}u16val;
    u16val.f = bit16;
	return u16val.byte[index_need];
}

void RefereeInfo_Decode(uint8_t *UsartRx_Info)
{

	switch(bit8TObit16(&UsartRx_Info[5]))
	{
		case 0x0001:
		{
			game_state.game_progress = UsartRx_Info[7];
		}break;
		
		case 0x0003:
		{
			game_robot_HP.red_1_robot_HP = bit8TObit16(&UsartRx_Info[7]);
			game_robot_HP.red_2_robot_HP = bit8TObit16(&UsartRx_Info[9]);
			game_robot_HP.red_3_robot_HP = bit8TObit16(&UsartRx_Info[11]);
			game_robot_HP.red_4_robot_HP = bit8TObit16(&UsartRx_Info[13]);
			game_robot_HP.red_5_robot_HP = bit8TObit16(&UsartRx_Info[15]);
			game_robot_HP.red_7_robot_HP = bit8TObit16(&UsartRx_Info[17]);
			game_robot_HP.red_outpost_HP = bit8TObit16(&UsartRx_Info[19]);
			game_robot_HP.red_base_HP = bit8TObit16(&UsartRx_Info[21]);
			
			game_robot_HP.blue_1_robot_HP = bit8TObit16(&UsartRx_Info[23]);
			game_robot_HP.blue_2_robot_HP = bit8TObit16(&UsartRx_Info[25]);
			game_robot_HP.blue_3_robot_HP = bit8TObit16(&UsartRx_Info[27]);
			game_robot_HP.blue_4_robot_HP = bit8TObit16(&UsartRx_Info[29]);
			game_robot_HP.blue_5_robot_HP = bit8TObit16(&UsartRx_Info[31]);
			game_robot_HP.blue_7_robot_HP = bit8TObit16(&UsartRx_Info[33]);
			game_robot_HP.blue_outpost_HP = bit8TObit16(&UsartRx_Info[35]);
			game_robot_HP.blue_base_HP = bit8TObit16(&UsartRx_Info[37]);
		}break;

		case 0x0201:
		{
			game_robot_state.robot_id                        =  UsartRx_Info[7];//机器人ID
			game_robot_state.robot_level                     =  UsartRx_Info[8];//机器人等级
			game_robot_state.remain_HP                       =  bit8TObit16(&UsartRx_Info[9]);//机器人剩余血量
			game_robot_state.max_HP                          =  bit8TObit16(&UsartRx_Info[11]);//机器人上限血量
			game_robot_state.shooter_id1_17mm_cooling_rate      =  bit8TObit16(&UsartRx_Info[13]);//id1 17mm枪口每秒冷却值
			game_robot_state.shooter_id1_17mm_cooling_limit     =  bit8TObit16(&UsartRx_Info[15]);//id1 17mm枪口热量上限
			game_robot_state.shooter_id1_17mm_speed_limit      =  bit8TObit16(&UsartRx_Info[17]);//id1 17mm枪口上限速度
			game_robot_state.shooter_id2_17mm_cooling_rate     =  bit8TObit16(&UsartRx_Info[19]);//id2 17mm枪口每秒冷却值
			game_robot_state.shooter_id2_17mm_cooling_limit       =  bit8TObit16(&UsartRx_Info[21]);//id2 17mm枪口热量上限
			game_robot_state.shooter_id2_17mm_speed_limit      =  bit8TObit16(&UsartRx_Info[23]);//id2 17mm枪口上限速度
			game_robot_state.shooter_id1_42mm_cooling_rate      =  bit8TObit16(&UsartRx_Info[25]);//42mm枪口每秒冷却值
			game_robot_state.shooter_id1_42mm_cooling_limit      =  bit8TObit16(&UsartRx_Info[27]);//42mm枪口热量上限
			game_robot_state.shooter_id1_42mm_speed_limit      =  bit8TObit16(&UsartRx_Info[29]);//42mm 枪口上限速度
			game_robot_state.chassis_power_limit      =  bit8TObit16(&UsartRx_Info[31]);//机器人底盘功率限制上限
			game_robot_state.mains_power_gimbal_output       =  UsartRx_Info[33] & 0x01 ;
			game_robot_state.mains_power_chassis_output      = (UsartRx_Info[33] & (0x01 << 1)) >> 1;
			game_robot_state.mains_power_shooter_output      =  UsartRx_Info[33] >> 2;
		}break;
		
		case 0x0202:
		{
			power_heat_data.chassis_volt               =  bit8TObit16(&UsartRx_Info[7]);////底盘输出电压
			power_heat_data.chassis_current            =  bit8TObit16(&UsartRx_Info[9]);//底盘输出电流
			power_heat_data.chassis_power              =  bit8TObit32(&UsartRx_Info[11]);//底盘输出功率
			power_heat_data.chassis_power_buffer       =  bit8TObit16(&UsartRx_Info[15]);//底盘功率缓冲
			power_heat_data.shooter_id1_17mm_cooling_heat              =  bit8TObit16(&UsartRx_Info[17]);//id1 17mm 枪口热量
			power_heat_data.shooter_id2_17mm_cooling_heat              =  bit8TObit16(&UsartRx_Info[19]);//id2 17mm 枪口热量		
			power_heat_data.shooter_id1_42mm_cooling_heat              =  bit8TObit16(&UsartRx_Info[21]);//42mm 枪口热量	

		}break;

		case 0x0204:
		{
		  buff_data.power_rune_buff  =  UsartRx_Info[7];//机器人增益
		}break;
		
		case 0x0207:
		{
			real_shoot_data.bullet_freq = UsartRx_Info[9];
			real_shoot_data.bullet_speed = bit8TObit32(&UsartRx_Info[10]);
		}break;
		
		default:break;
	}
}

#if defined(GIMBAL_BOARD)
void get_Referee_Data(uint32_t *StdId, uint8_t *rxBuf)
{
		if(*StdId != 0x600) return;

		uint8_t mode_L = (rxBuf[0] & (uint8_t)(1 << 2)) >> 2,
				mode_H = (rxBuf[0] & (uint8_t)(1 << 3)) >> 3; //发射机构类型：00(初始)，01(爆发)，10(冷却)，11(弹速)
		uint8_t robot_LV_L = (rxBuf[0] & (uint8_t)(1 << 4)) >> 4,
				robot_LV_H = (rxBuf[0] & (uint8_t)(1 << 5)) >> 5;//等级情况，00(1级) 01(2级) 10(3级)
	
		robot.id = (rxBuf[0] & (uint8_t)(3 << 6)) >> 6;//0蓝2红
		robot.mains_power_shooter_output = (rxBuf[0] & (uint8_t)(1 << 1)) >> 1; //主控电源输出情况：1为24v，0为无输出

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
			robot.shooter_id1_17mm.cooling_limit = 50;
			robot.shooter_id1_17mm.cooling_rate = 10;
			robot.shooter_id1_17mm.speed_limit = 15;
		}else if(mode_L == 0&& mode_H == 1)
		{
			robot.mode = BURST;
			robot.shooter_id1_17mm = Shooter_BURST[robot.level];
		}else if(mode_L == 1&& mode_H == 0)
		{
			robot.mode = COOLING;
			robot.shooter_id1_17mm = Shooter_COOLING[robot.level];
		}else if(mode_L == 1&& mode_H == 1)
		{
			robot.mode = RATE;
			robot.shooter_id1_17mm = Shooter_RATE[robot.level];
		}
		
		robot.cooling_heat = (int16_t)rxBuf[1] << 8 | (int16_t)rxBuf[2];//17mm 枪口热量
		robot.bullet_speed = ((int16_t)rxBuf[3] << 8 | (int16_t)rxBuf[4])/100.f; //17mm枪口射速
}
#endif

#if defined(CHASSIS_BOARD)
void Report_Referee_Data(CAN_TxFrameTypeDef *CAN_TxHandler)
{
	static uint8_t robot_id = 0;//0红，1蓝
	static uint8_t robot_LV_L = 0,robot_LV_H = 0;//等级情况，00(1级) 01(2级) 10(3级)
	static uint8_t shoot_type_L = 0,shoot_type_H = 0; //发射机构类型：00(初始)，01(爆发)，10(冷却)，11(弹速)

	//id判断
	if(game_robot_state.robot_id > 99)
	{
		robot_id = 2;//红
	}else{
		robot_id = 0;//蓝	
	}
	//等级判断
	if(game_robot_state.robot_level==1)//1级
	{
		robot_LV_L = 0;
		robot_LV_H = 0;
		//发射机构类型判断
		if(game_robot_state.shooter_id1_17mm_cooling_limit == 150)
		{
			shoot_type_L = 0;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 75)
		{
			shoot_type_L = 1;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 50)
		{
			if(game_robot_state.shooter_id1_17mm_cooling_rate == 10)
			{
				shoot_type_L = 0;
				shoot_type_H = 0;
			}else if(game_robot_state.shooter_id1_17mm_cooling_rate == 40)
			{
				shoot_type_L = 1;
				shoot_type_H = 0;
			}
		}
	}
	else if(game_robot_state.robot_level==2)//2级
	{
			robot_LV_L = 0;
			robot_LV_H = 1;
		//发射机构类型判断
		if(game_robot_state.shooter_id1_17mm_cooling_limit == 280)
		{
			shoot_type_L = 0;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 150)
		{
			shoot_type_L = 1;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 100)
		{
			shoot_type_L = 1;
			shoot_type_H = 0;
		}
	}
	else if(game_robot_state.robot_level==3)//3级
	{
			robot_LV_L = 1;
			robot_LV_H = 0;
		//发射机构类型判断
		if(game_robot_state.shooter_id1_17mm_cooling_limit == 400)
		{
			shoot_type_L = 0;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 200)
		{
			shoot_type_L = 1;
			shoot_type_H = 1;
		}else if(game_robot_state.shooter_id1_17mm_cooling_limit == 150)
		{
			shoot_type_L = 1;
			shoot_type_H = 0;
		}
	}

	CAN_TxHandler->data[0] = robot_id << 6 | robot_LV_H << 5 | robot_LV_L << 4 | shoot_type_H << 3 | shoot_type_L << 2 ;
	CAN_TxHandler->data[1] = (uint8_t)(power_heat_data.shooter_id1_17mm_cooling_heat >> 8);
	CAN_TxHandler->data[2] = (uint8_t)(power_heat_data.shooter_id1_17mm_cooling_heat	 );
	CAN_TxHandler->data[3] = (uint8_t)((int16_t)(real_shoot_data.bullet_speed*100) >> 8);
	CAN_TxHandler->data[4] = (uint8_t)((int16_t)(real_shoot_data.bullet_speed*100)     );
	
	USER_CAN_TxMessage(CAN_TxHandler);
}
#endif
