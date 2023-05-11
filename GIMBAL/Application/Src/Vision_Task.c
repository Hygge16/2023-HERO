#include "cmsis_os.h"
#include "Vision_Task.h"
#include "SystemState_Task.h"
#include "Gimbal_Task.h"
#include "kalman.h"
#include "bsp_rc.h"
#include "vision.h"
#include "bsp_uart.h"

/*--------------------------   ��������   ------------------------------*/

VISION vision={
    .delta_T=0,
    .imu=&imu,
    .tx2=&tx2,
    .send2uart.sign_Flag=0,
};

kalman_filter_t KF_Vision[2];
kalman_filter_init_t KF_Param_Vision[2]={
    //YAW
    [0]={
        .P_data = {2, 0, 0, 2},
        .A_data = {1, 0.013/*0.001*/, 0, 1},//����ʱ����
        .H_data = {1, 0, 0, 1},
        .Q_data = {1, 0, 0, 1},
        .R_data = {500, 0, 0, 900}//500 1000
    },
    //PITCH
    [1]={
        .P_data = {2, 0, 0, 2},
        .A_data = {1, 0.001/*0.001*/, 0, 1},//����ʱ����
        .H_data = {1, 0, 0, 1},
        .Q_data = {1, 0, 0, 1},
        .R_data = {200, 0, 0, 400}
    }
};

float k2=1.f,k3=1.f;
float yaw_Predict;

//������ֵ����̨�����ڱ��������Ӧ�����ϴ�仯����tx2ֵ�������ڱ�������̨������¼����������仯��
//װ�װ�λ��0y1p
float armor_position2vision[2];
//װ�װ��ٶ�0y1p
float armor_velocity2vision[2];
//Ԥ����0y1p
float predictValue[2];
/*--------------------------   ��������   ------------------------------*/
float predict_yaw_temp;
//��������

int key_KF=0;

float yaw_feed=0.49f;
extKalman_t KF_YAW_FEED,KF_YAW_ACCEL,KF_YAW_DIS,KF_YAW_ANGLE,KF_YAW_SEED;
float yaw_tx2_last_err=0.f;
QueueObj yaw_Queue_speed={.queueLength=60 },yaw_Queue_accel={.queueLength=60 },yaw_Queue_dis={.queueLength=60 },yaw_Queue_angle={.queueLength=60 };

float yaw_accel=0.f;
float feed_dis=0.f;
uint32_t feed_big=0;
float accel_k=0.f,speed_k=0.32f,dis_k=0.;
float imu_yaw_angle;
float fedd111=1.f;
float speed_last;
float tx2_yaw_last;
bool IF_Fire_Ready=false;

void VISION_TASK(void *args)
{
    uint32_t currentTime;

	while(1)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
	
				if(Key_F() == true)
					tx2.kf_Flag = 1;
				else if(Key_F() == false)
					tx2.kf_Flag = 0;
				
        
    //����̨�����(������)����tx2
        send2uart1();
        vTaskDelayUntil(&currentTime, 1);//������ʱ
	}
}
/**
* @brief ��ȡĿ��Ĳ��
* @param void
* @return void
*	�Զ��е��߼�
*/
float Get_Diff(uint8_t queue_len, QueueObj *Data,float add_data)
{
    if(queue_len>=Data->queueLength)
        queue_len=Data->queueLength;
    //��ֹ���
    Data->queueTotal-=Data->queue[Data->nowLength];
    Data->queueTotal+=add_data;

    Data->queue[Data->nowLength]=add_data;
			
    Data->nowLength++;

    if(Data->full_flag==0)//��ʼ����δ��
    {
        Data->aver_num=Data->queueTotal/Data->nowLength;
    }else if(Data->full_flag == 1)
	{
	    Data->aver_num=(Data->queueTotal)/queue_len;	
		
	}
    if(Data->nowLength>=queue_len)
    {
        Data->nowLength=0;
        Data->full_flag=1;
    }

    Data->Diff=add_data - Data->aver_num;
    return Data->Diff;
}

static TickType_t get_delta_time(void)
{
   static  TickType_t now,pre,delta;

//�Ӿ������Ѿ�����
    if(vision.tx2->isUpdata==1){
//				vision.KF.measurement = get_measurement();
        pre = now;
        now = xTaskGetTickCount();
			
    //�Ӿ�������Ϊ������״̬
    }
    delta = now - pre-1;
    VAL_LIMIT(delta,0.f,15.f);
    return delta;
}
static void send2uart1(void)
{
    UART1_TX_BUF[0]  = 0x69;
    UART1_TX_BUF[1]  = tx2.color_Flag;
    UART1_TX_BUF[2]  = 1;
    UART1_TX_BUF[3]  = tx2.kf_Flag;
    UART1_TX_BUF[4]  = 14;
    UART1_TX_BUF[5]  = (int)(14.9f*100)%100;
    UART1_TX_BUF[6]  = (int16_t) (INS_angle[2]);
    UART1_TX_BUF[7]  = (int16_t)((INS_angle[2])*100.f)%100;
		UART1_TX_BUF[8]  = (int16_t)((INS_angle[2])*10000.f)%100;
    UART1_TX_BUF[9]  = (int16_t)(-INS_angle[0]);
    UART1_TX_BUF[10] = (int16_t)(-(INS_angle[0])*100.f)%100;
    UART1_TX_BUF[11] = (int16_t)(-(INS_angle[0])*10000.f)%100;
	
		UART1_TX_BUF[12] = (UART1_TX_BUF[1]+UART1_TX_BUF[2]+UART1_TX_BUF[3] + UART1_TX_BUF[4]+UART1_TX_BUF[5] + UART1_TX_BUF[6]
					 + UART1_TX_BUF[7]+UART1_TX_BUF[8] + UART1_TX_BUF[9]+UART1_TX_BUF[10]+UART1_TX_BUF[11])/11;
	
		UART1_TX_BUF[13] = 0x96;
		
		HAL_UART_Transmit_DMA(&huart1,UART1_TX_BUF,UART1_MAX_TX_LEN);

}


