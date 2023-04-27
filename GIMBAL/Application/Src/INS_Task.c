//
// Created by Yuanbin on 22-10-3.
//
#include "cmsis_os.h"
#include "INS_Task.h"
#include "robot_ref.h"

#include "pid.h"

#include "bsp_tim.h"

#include "bmi088_driver.h"

#include "AHRS_MiddleWare.h"


INS_Info_t INS_Info;

/* Private variables ---------------------------------------------------------*/
/* Algorithm variables */
PID_TypeDef_t imu_temp_pid;
float imu_temp_pid_para[PID_PARAMETER_CNT] = {0,0,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static imu_kalman_t accel_kalman;

/* INS variables */
bmi088_real_data_t bmi088_data = {0,};
float const *ist8310_data =0;

static INS_Recv_Data_t INS_Recv_Data;
		
static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

/* linear calibration scalar */
float gyro_scale_factor[3][3] = {    
		[0]={0.0f, 1.0f, 0.0f},                     
    [1]={-1.0f, 0.0f, 0.0f},                    
    [2]={0.0f, 0.0f, 1.0f},};

float accel_scale_factor[3][3] = {    
		[0]={0.0f, 1.0f, 0.0f},                     
    [1]={-1.0f, 0.0f, 0.0f},                    
    [2]={0.0f, 0.0f, 1.0f},};

static float mag_scale_factor[3][3] = {    
		[0]={1.0f, 0.0f, 0.0f},                     
    [1]={0.0f, 1.0f, 0.0f},                     
    [2]={0.0f, 0.0f, 1.0f}};

static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static float eff_[3];
static float Eff[80] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
												10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
												20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
												30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
												40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
												50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
												60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
												70, 71, 72, 73, 74, 75, 76, 77, 78, 79}; 

/* Private function prototypes -----------------------------------------------*/
static void INS_Init(INS_Recv_Data_t *recv_data);
static void imu_cali_slove(INS_Recv_Data_t *recv_data, bmi088_real_data_t *bmi088, float const *ist8310);
static void IMU_getValues(INS_Recv_Data_t *recv_data) ;
static void imu_temp_control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINS_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
	TickType_t systick = 0;
	
	INS_Init(&INS_Recv_Data);
	
  /* Infinite loop */
  for(;;)
  {
			systick = osKernelSysTick();

			IMU_getValues(&INS_Recv_Data);
			imu_cali_slove(&INS_Recv_Data, &bmi088_data, ist8310_data);
			imu_temp_control(bmi088_data.temp);
			
			memcpy(accel_fliter_1,accel_fliter_2,sizeof(accel_fliter_2));
			memcpy(accel_fliter_2,accel_fliter_3,sizeof(accel_fliter_3));
		
			for(uint8_t i=INS_X_ADDRESS_OFFSET;i<=INS_Z_ADDRESS_OFFSET;i++)
			{
					accel_fliter_3[i] = accel_fliter_2[i] * fliter_num[0] + accel_fliter_1[i] * fliter_num[1] + INS_Recv_Data.INS_accel[i] * fliter_num[2];
			}

			
			memcpy(INS_Info.accel,INS_Recv_Data.INS_accel,sizeof(INS_Recv_Data.INS_accel));
									
			INS_Info.yaw_angle = INS_Recv_Data.INS_angle[0] *180.f/PI;
			if(INS_Info.yaw_angle - INS_Info.last_yawangle >180.f)
			{
					INS_Info.YawRoundCount--;
			}
			else if(INS_Info.yaw_angle - INS_Info.last_yawangle <-180.f)
			{
					INS_Info.YawRoundCount++;
			}
			INS_Info.last_yawangle = INS_Info.yaw_angle;
			INS_Info.yaw_tolangle = INS_Info.YawRoundCount*360.f + INS_Info.yaw_angle;
			
#if defined(CHASSIS_BOARD)
			INS_Info.pit_angle = INS_Recv_Data.INS_angle[1] *180.f/PI;
			INS_Info.rol_angle = INS_Recv_Data.INS_angle[2] *180.f/PI;
			INS_Info.pit_gyro = INS_Recv_Data.INS_gyro[1] *180.f/PI;
			INS_Info.rol_gyro = INS_Recv_Data.INS_gyro[0] *180.f/PI;
#endif
#if defined(GIMBAL_BOARD)
			INS_Info.pit_angle = INS_Recv_Data.INS_angle[2] *180.f/PI;
			INS_Info.rol_angle = INS_Recv_Data.INS_angle[1] *180.f/PI;
			INS_Info.pit_gyro = INS_Recv_Data.INS_gyro[0] *180.f/PI;
			INS_Info.rol_gyro = INS_Recv_Data.INS_gyro[1] *180.f/PI;
#endif
			INS_Info.yaw_gyro = INS_Recv_Data.INS_gyro[2] *180.f/PI;
			
			osDelayUntil(&systick, 1);//绝对延时
  }
  /* USER CODE END INS_Task */
}

static void INS_Init(INS_Recv_Data_t *recv_data)
{
		BMI088_read(bmi088_data.gyro,bmi088_data.accel,&bmi088_data.temp);
    PID_Init(&imu_temp_pid,PID_VELOCITY,imu_temp_pid_para);
		imu_cali_slove(recv_data, &bmi088_data, ist8310_data);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = recv_data->INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = recv_data->INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = recv_data->INS_accel[2];
}

static void imu_cali_slove(INS_Recv_Data_t *recv_data, bmi088_real_data_t *bmi088, float const *ist8310)
{
    for (uint8_t i = INS_X_ADDRESS_OFFSET; i <= INS_Z_ADDRESS_OFFSET; i++)
    {
				if(bmi088 != NULL)
				{
					recv_data->INS_gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] ;
					recv_data->INS_accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] ;
				}
				if(ist8310 != NULL)
				{
					recv_data->INS_mag[i] = ist8310[0] * mag_scale_factor[i][0] + ist8310[1] * mag_scale_factor[i][1] + ist8310[2] * mag_scale_factor[i][2] ;
				}
    }
}

static void IMU_getValues(INS_Recv_Data_t *recv_data) 
{
    float bmi088_sum[2][3]={0.f,}; //[0]accel [1]gyro
		static float BMI088_FIFO[2][3][101] = {0};//1.[0]accel [1]gyro 3.[0]-[99]为最近100次数据 [100]为100次数据的平均值	
		
		BMI088_read(bmi088_data.gyro,bmi088_data.accel,&bmi088_data.temp);
		
		for(uint8_t i=INS_X_ADDRESS_OFFSET;i<=INS_Z_ADDRESS_OFFSET;i++)
		{
				for(uint8_t j=1;j<100;j++)
				{
						BMI088_FIFO[0][i][j-1]=BMI088_FIFO[0][i][j];
						BMI088_FIFO[1][i][j-1]=BMI088_FIFO[1][i][j];
				}
				
				BMI088_FIFO[0][i][99] = bmi088_data.accel[i];  
				BMI088_FIFO[1][i][99] = bmi088_data.gyro[i];

				for(uint8_t j=0;j<100;j++)//求当前数组（加速、陀螺仪的100个值）的和，再取平均值
				{
						bmi088_sum[0][i] += BMI088_FIFO[0][i][j];	
					 	bmi088_sum[1][i] += BMI088_FIFO[1][i][j];	
				}
				BMI088_FIFO[0][i][100] = bmi088_sum[0][i]/100.f;
				BMI088_FIFO[1][i][100] = bmi088_sum[1][i]/100.f;
			
				for(uint8_t j = 0; j <50; j++)
				{
						if(BMI088_FIFO[1][i][100] ==  Eff[j])
						{
								eff_[i] = Eff[j];
								break;
						}
						if(-BMI088_FIFO[1][i][100] ==  Eff[j])
						{
								eff_[i] = -Eff[j];
								break;
						}
				}
			
				accel_kalman.Xk_accel[i] = accel_kalman.Xk_accel[i];;   //先验估计  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
				accel_kalman.Q_accel[i] = 0.018f;
				accel_kalman.R_accel[i] = 0.542f;
				accel_kalman.pk_accel[i] = accel_kalman.pk_accel[i]+ accel_kalman.Q_accel[i];;   //先验误差 p(k|k-1) = A*p(k-1|k-1)*A'+Q
				accel_kalman.Kk_accel[i] = accel_kalman.pk_accel[i] / (accel_kalman.pk_accel[i] + accel_kalman.R_accel[i]);   //卡尔曼增益 kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
				accel_kalman.Xk_accel[i] = accel_kalman.Xk_accel[i] + accel_kalman.Kk_accel[i]*(BMI088_FIFO[0][i][100] - accel_kalman.Xk_accel[i]);  //卡尔曼增益 kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
				accel_kalman.pk_accel[i] = (1 - accel_kalman.Kk_accel[i])*accel_kalman.pk_accel[i];;	  //状态更新 p(k|k) = (I-kg(k)*H)*P(k|k-1)
				
				recv_data->INS_accel[i] = accel_kalman.Xk_accel[i];
				recv_data->INS_gyro[i] = BMI088_FIFO[1][i][100] - eff_[i];
		}
}

static void imu_temp_control(float temp)
{
    static uint8_t temp_constant_time = 0;
		static uint8_t first_temperate = 0;

    if (first_temperate)
    {
        f_PID_Calculate(&imu_temp_pid, 45.0f,temp);
        if (imu_temp_pid.Output < 0.0f)
        {
            imu_temp_pid.Output = 0.0f;
        }
        Heat_Power_Ctl((uint16_t)imu_temp_pid.Output);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 40.0f)
        {
					temp_constant_time++;
					if(temp_constant_time > 200)
					{
						first_temperate = 1;
					}
						//达到设置温度，将积分项设置为一半最大功率，加速收敛
						imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
        }
        Heat_Power_Ctl(MPU6500_TEMP_PWM_MAX - 1);
    }
}
