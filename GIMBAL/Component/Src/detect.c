//
// Created by Yuanbin on 22-10-3.
//
#include "detect.h"

#include "bsp_tim.h"

#include "bsp_rc.h"
#include "motor.h"

/**
  * @brief  检测数据初始化
  * @param  检测数据结构体指针，相位，敏感度
  */
void detect_slip_init(detect_slip_t* data, uint16_t phase,	float sen)
{
	data->rk_sen = sen;
	float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	data->phase = phase;
	LP_FilterInit(&data->LPF_1,fliter_num);
	LP_FilterInit(&data->LPF_2,fliter_num);
	Kalman_Init(&data->KF,0.1f,0.1f);
}


/**
  * @brief  检测数据更新
  * @param  检测数据结构体指针，检测数据输入
  */
void detect_slip_update(detect_slip_t* data, float new_data)
{
	LP_FilterCalc(&data->LPF_1,new_data * data->rk_sen);
	LP_FilterCalc(&data->LPF_2,data->LPF_1.out);
	KalmanFilterCalc(&data->KF,data->LPF_2.out);
	
	if(data->num_rk_now >= DETECT_NUM)data->num_rk_now = 0;
	data->KF_rk[data->num_rk_now]=data->KF.rk;
	int phase_num = data->num_rk_now - data->phase;
	if(phase_num<0)phase_num += DETECT_NUM;
	if(phase_num>=DETECT_NUM)phase_num -= DETECT_NUM;
	data->rk_adj = data->KF_rk[phase_num];
	data->num_rk_now++;
}

/**
  * @brief  判断是否打滑
  * @param  判断打滑预警值
  */
void check_slipped(chassis_detect_t *chassis_detect,detect_slip_t detect_slip[3])
{
	float slip_rk = detect_slip[0].rk_adj + detect_slip[1].rk_adj + detect_slip[2].rk_adj;
	
	if(chassis_detect->slip_num_rk_now >= CHECK_NUM)chassis_detect->slip_num_rk_now = 0;
	chassis_detect->slip_rk_sum -= chassis_detect->slip_rk[chassis_detect->slip_num_rk_now];
	chassis_detect->slip_rk[chassis_detect->slip_num_rk_now]=slip_rk;
	chassis_detect->slip_rk_sum += chassis_detect->slip_rk[chassis_detect->slip_num_rk_now];
	chassis_detect->slip_num_rk_now++;	
	
	if(chassis_detect->slip_rk_sum > chassis_detect->slip_warning_value)chassis_detect->isSlipped = 1;
	else chassis_detect->isSlipped = 0;
}


