#include "referee_info.h"


ext_game_robot_status_t             game_robot_state;             //0x0201
ext_power_heat_data_t 	        		power_heat_data;	      	  //0x0202
ext_buff_t                          buff_data;                    //0x0204
ext_shoot_data_t                    real_shoot_data;              //0x0207
ext_bullet_remaining_t              bullet_remaining;             //0x0208
/*
*@title：float8位转32位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
float bit8TObit32(uint8_t *change_info)
{
	union
	{
    	float f;
		char  byte[4];
	}u32val;
    u32val.byte[0]=change_info[0];
    u32val.byte[1]=change_info[1];
    u32val.byte[2]=change_info[2];
    u32val.byte[3]=change_info[3];
	return u32val.f;
}
/*
*@title：int32位转8位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
 char bit32TObit8(int index_need,int bit32)
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
int16_t bit8TObit16(uint8_t *change_info)
{
	union
	{
    int16_t f;
		char  byte[2];
	}u16val;
    u16val.byte[0]=change_info[0];
    u16val.byte[1]=change_info[1];
	return u16val.f;
}
/*
*@title：int16位转8位
*@description：
*@param 1：	
*@param 2：	
*@return:：	
*/
 char bit16TObit8(int index_need,int bit16)
{
	union
	{
    float  f;
		char  byte[2];
	}u16val;
    u16val.f = bit16;
	return u16val.byte[index_need];
}

uint16_t length;
void RefereeInfo_Decode(uint8_t *UsartRx_Info)
{

	switch(bit8TObit16(&UsartRx_Info[5]))
	{
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
		case 0x0208:
		{
			bullet_remaining.bullet_remaining_num_42mm=bit8TObit16(&UsartRx_Info[9]);
		}break;
		default:
		{
		}	
	}

}

