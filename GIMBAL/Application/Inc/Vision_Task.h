#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "FreeRTOS.h"

#include "pid.h"
#include "assist.h"
#include "filter.h"
#include "bsp_uart.h"
#include "bmi088_driver.h"
#include "vision.h"
#include "INS_Task.h"


/*数据结构*/
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //长度
    float queue[100];
    //指针
    float aver_num;//平均值

    float Diff;//差分值

    uint8_t full_flag;
} QueueObj;


typedef struct{
//帧差时间
    TickType_t delta_T;
//发送结构体
    struct{
        uint8_t sign_Flag;//1++2+-3-+4--
        uint8_t inform[4];
    }send2uart;
//卡尔曼
    struct{
        float armor_Speed;//装甲板移动速度[0]yaw;[1]pitch
        float measurement;//测量值[0]yaw;[1]pitch
        float predict[2]; //预测值[0][0]yaw轴角度，[0][1]yaw轴速度;[1][0]pitch轴角度，[1][1]pitch轴速度
}KF;
//云台位置偏移量（误差量）
    float pit,yaw;
//tx2挂载
    TX2 *tx2;
// 陀螺仪挂载
    bmi088_real_data_t *imu;
}VISION;
extern VISION vision;

extern float predict_yaw_temp;
extern float yaw_accel;
extern float armor_position2vision[2];

extern float imu_yaw_angle;
extern bool IF_Fire_Ready;

extern int key_KF;
extern kf_data_t KF_YAW_FEED,KF_YAW_ACCEL,KF_YAW_DIS,KF_YAW_ANGLE,KF_YAW_SEED;


static TickType_t get_delta_time(void);
static void send2uart1(void);
static float* get_measurement(void);
static float* get_armor_speed(float* nowValue);
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data);
#endif



