#ifndef SYSTEMSTATE_TASK_H
#define SYSTEMSTATE_TASK_H

#include "FreeRTOS.h"
#include "bsp_can.h"

#define IS_STANDARD

//�豸����״̬
typedef enum{
    OFF,//�رգ�ж����
    ON,//����
    CALIBRATING,//����
    DEVICE_ERROR,//��ת�����ߵ�
    STATE_NUM,
}DEVICE_STATE;


typedef enum{
    INITIAL=0,
    BURST,//����
    RATE,//����
    ROBOT_MODE_NUM,
}ROBOT_MODE;

typedef enum{
    LV_1,
    LV_2,
    LV_3,
    ROBOT_LEVEL_NUM,
}ROBOT_LEVEL;

typedef  struct
{
	uint16_t	cooling_rate;		//ǹ��ÿ����ȴֵ
	uint16_t	cooling_limit;  //ǹ����������
	uint16_t	speed_limit;    //ǹ�������ٶ� ��λ m/s
	uint16_t  bullet_remaining_num_42mm;//42mm����ʣ�෢����Ŀ
}ROBOT_SHOOTER_T;



typedef  struct 
{
		uint8_t id; //������Id 0��1��
    ROBOT_LEVEL level; //�ȼ�
		ROBOT_MODE mode; //�����������
		uint8_t mains_power_shooter_output;//������������0 ����� 1 24V
		uint16_t cooling_heat;   //ǹ������
		float bullet_speed;//ʵʱ����
		ROBOT_SHOOTER_T shooter_id1_42mm;		//���������Ϣ
}ROBOT;

extern ROBOT robot;

void Referee_get_Data(CAN_HandleTypeDef* CANx,CAN_RxFrameTypeDef * RxMsg);

#endif



