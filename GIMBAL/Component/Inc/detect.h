//
// Created by Yuanbin on 22-10-3.
//

#ifndef DETECT_H
#define DETECT_H

#include "stdint.h"
#include "filter.h"

#define DETECT_NUM 50
#define CHECK_NUM 50

//底盘检测结构体
typedef struct
{
	uint16_t cnt_LoseCtrl;			//失控检测
	
	float slip_rk[CHECK_NUM];		//打滑检测
	float slip_warning_value;
	float slip_rk_sum;
	uint8_t slip_num_rk_now;
	uint8_t isSlipped;
	
}chassis_detect_t;

//打滑检测对象结构体
typedef struct
{
	lpf_data_t LPF_1,LPF_2;		//低通滤波
	kf_data_t KF;							//卡方检测
	float rk_sen;							//敏感度
	uint16_t phase;						//相位
	float KF_rk[DETECT_NUM];
	uint8_t num_rk_now;
	float rk_adj;
}detect_slip_t;


/* Exported functions --------------------------------------------------------*/
extern void detect_slip_init(detect_slip_t* data, uint16_t phase,	float sen);
extern void detect_slip_update(detect_slip_t* data, float new_data);
extern void check_slipped(chassis_detect_t *chassis_detect,detect_slip_t detect_slip[3]);
#endif //DETECT_H
