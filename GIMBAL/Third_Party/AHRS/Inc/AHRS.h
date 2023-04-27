#ifndef AHRS_H
#define AHRS_H

#include "AHRS_MiddleWare.h"
#include "stdbool.h"
/**
  * @brief          ���ݼ��ٶȵ����ݣ������Ƶ����ݽ�����Ԫ����ʼ��
  * @param[in]      ��Ҫ��ʼ������Ԫ������
  * @param[in]      ���ڳ�ʼ���ļ��ٶȼ�,(x,y,z)��Ϊ�� ��λ m/s2 
  * @param[in]      ���ڳ�ʼ���Ĵ����Ƽ�,(x,y,z)��Ϊ�� ��λ uT
  * @retval         ���ؿ�
  */
extern void AHRS_init(float quat[4], const float accel[3], const float mag[3]);

/**
  * @brief          ���������ǵ����ݣ����ٶȵ����ݣ������Ƶ����ݽ�����Ԫ������
  * @param[in]      ��Ҫ���µ���Ԫ������
  * @param[in]      ���¶�ʱʱ�䣬�̶���ʱ���ã�����1000Hz�����������Ϊ0.001f,
  * @param[in]      ���ڸ��µ�����������,����˳��(x,y,z) ��λ rad
  * @param[in]      ���ڳ�ʼ���ļ��ٶ�����,����˳��(x,y,z) ��λ m/s2 
  * @param[in]      ���ڳ�ʼ���Ĵ���������,����˳��(x,y,z) ��λ uT
  * @retval         1:���³ɹ�, 0:����ʧ��
  */
extern bool AHRS_update(float quat[4], const float timing_time, const float gyro[3], const float accel[3], const float mag[3]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ����ƫ��yaw
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���ص�ƫ����yaw ��λ rad
  */
extern float get_yaw(const float quat[4]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ���Ǹ����� pitch
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���صĸ����� pitch ��λ rad
  */
extern float get_pitch(const float quat[4]);
/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ���Ǻ���� roll
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @retval         ���صĺ���� roll ��λ rad
  */
extern float get_roll(const float quat[4]);

/**
  * @brief          ������Ԫ����С�����Ӧ��ŷ����yaw��pitch��roll
  * @param[in]      ��Ԫ�����飬��ΪNULL
  * @param[in]      ���ص�ƫ����yaw ��λ rad
  * @param[in]      ���صĸ�����pitch  ��λ rad
  * @param[in]      ���صĺ����roll ��λ rad
  */
extern void get_angle(const float quat[4], float *yaw, float *pitch, float *roll);
/**
  * @brief          ���ص�ǰ���������ٶ�
  * @param[in]      ��
  * @retval         �����������ٶ� ��λ m/s2
  */
extern float get_carrier_gravity(void);

#endif
