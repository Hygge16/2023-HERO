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


/*���ݽṹ*/
typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //����
    float queue[100];
    //ָ��
    float aver_num;//ƽ��ֵ

    float Diff;//���ֵ

    uint8_t full_flag;
} QueueObj;


typedef struct{
//֡��ʱ��
    TickType_t delta_T;
//���ͽṹ��
    struct{
        uint8_t sign_Flag;//1++2+-3-+4--
        uint8_t inform[4];
    }send2uart;
//������
    struct{
        float armor_Speed;//װ�װ��ƶ��ٶ�[0]yaw;[1]pitch
        float measurement;//����ֵ[0]yaw;[1]pitch
        float predict[2]; //Ԥ��ֵ[0][0]yaw��Ƕȣ�[0][1]yaw���ٶ�;[1][0]pitch��Ƕȣ�[1][1]pitch���ٶ�
}KF;
//��̨λ��ƫ�������������
    float pit,yaw;
//tx2����
    TX2 *tx2;
// �����ǹ���
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



