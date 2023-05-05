//
// Created by Yuanbin on 22-10-3.
//

#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

#define mat         float

typedef struct 
{
	float fliter_num[3];
	float fliter_1,fliter_2,fliter_3;
	float out;
	float in;
	
}lpf_data_t;

typedef struct 
{
	float in;
	float Last_P;	//上次估算协方差
	float Now_P;	//当前估算协方差
	float out;		//卡尔曼滤波器输出
	float Kg;			//卡尔曼增益
	float Q;			//过程噪声协方差
	float R;			//观测噪声协方差
	float ek;
	float rk;			//卡方数据
}kf_data_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;


/* Exported functions --------------------------------------------------------*/

extern void LP_FilterInit(lpf_data_t *lp,float *fliter_num);
extern float LP_FilterCalc(lpf_data_t *lp,float input);
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
extern void Kalman_Init(kf_data_t *kf,float Q,float R);
extern float KalmanFilterCalc(kf_data_t *kf,float input);

#endif //FILTER_H
