//
// Created by Yuanbin on 22-10-3.
//

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "stdint.h"
#include "string.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 800.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.1f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   2000.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT  2000.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 2000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_X_ADDRESS_OFFSET 0
#define INS_Y_ADDRESS_OFFSET 1
#define INS_Z_ADDRESS_OFFSET 2

typedef struct
{
	float Xk_accel[3];
	float pk_accel[3];
	float Q_accel[3];
	float R_accel[3];
	float Kk_accel[3];

	float Xk_mag[3];
	float Pk_mag[3];
	float Q_mag[3];
	float R_mag[3];
	float Kk_mag[3];
}imu_kalman_t;

typedef struct
{
	float INS_gyro[3];
	float INS_accel[3];
	float INS_mag[3];
	float INS_quat[4];
	float INS_angle[3];      //euler angle, unit rad.欧拉角 单位 rad   
}INS_Recv_Data_t;

typedef struct
{
	float pit_angle;
	float yaw_angle;
	float yaw_tolangle;
	float rol_angle;

	float pit_gyro;
	float yaw_gyro;
	float rol_gyro;
	
	float accel[3];
	
	float last_yawangle;
	int16_t YawRoundCount;

}INS_Info_t;

extern INS_Info_t INS_Info;

#endif //IMU_TASK_H
