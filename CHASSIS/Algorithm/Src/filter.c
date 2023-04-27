//
// Created by Yuanbin on 22-10-3.
//
#include "filter.h"

/**
 *@brief	二阶低通滤波初始化
 *@param	Kalman_data_t *kf 卡尔曼结构体参数
 *@return	none
 */
void LP_FilterInit(lpf_data_t *lp,float *fliter_num)
{
	lp->fliter_num[0] = fliter_num[0];
	lp->fliter_num[1] = fliter_num[1];
	lp->fliter_num[2] = fliter_num[2];
}

/**
 *@brief	二阶低通滤波
 *@param	hpf_data_t *hp ,低通滤波结构体参数;input ,输入值
 *@return	滤波后的参数
 */
float LP_FilterCalc(lpf_data_t *lp,float input)
{
	lp->in = input;
	lp->fliter_1 = lp->fliter_2;
	lp->fliter_2 = lp->fliter_3;
	lp->fliter_3 = lp->fliter_2 * lp->fliter_num[0] + lp->fliter_1 * lp->fliter_num[1] +  input * lp->fliter_num[2];
	lp->out=lp->fliter_3;
	return lp->out;
}



/**
 *@brief	卡尔曼滤波器初始化
 *@param	Kalman_data_t *kf 卡尔曼结构体参数
 *@return	none
 */
void Kalman_Init(kf_data_t *kf,float Q,float R)
{
	kf->Last_P = 1;			
	kf->Now_P = 0;		
	kf->out = 0;			
	kf->Kg = 0;		
	kf->Q = Q;
	kf->R = R;
}

/**
 *@brief	卡尔曼滤波器
 *@param	Kalman_data_t *kf ,卡尔曼结构体参数;input ,输入值
 *@return	滤波后的参数
 */
float KalmanFilterCalc(kf_data_t *kf,float input)
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kf->Now_P = kf->Last_P + kf->Q;
		kf->ek = 1 * kf->out;
		kf->ek = input - kf->ek;
		kf->ek = kf->ek * 1.0f/(kf->Now_P + kf->R);
		kf->rk = kf->ek * kf->ek;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kf->Kg = kf->Now_P / (kf->Now_P + kf->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kf->out = kf->out + kf->Kg * (input -kf->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kf->LastP 威下一次运算准备。
   kf->Last_P = (1-kf->Kg) * kf->Now_P;
   return kf->out;
}


