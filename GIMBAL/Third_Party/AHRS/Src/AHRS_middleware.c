#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"
/**
 * @brief          ���ڻ�ȡ��ǰ�߶�
 * @author         RM
 * @param[in]      �߶ȵ�ָ�룬float
 * @retval         ���ؿ�
 */

void AHRS_get_height(float* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          ���ڻ�ȡ��ǰγ��
 * @author         RM
 * @param[in]      γ�ȵ�ָ�룬float
 * @retval         ���ؿ�
 */

void AHRS_get_latitude(float* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          ���ٿ���������
 * @author         RM
 * @param[in]      ������Ҫ�����ĸ�������float
 * @retval         ����1/sqrt ������ĵ���
 */

float AHRS_invSqrt(float num)
{
    return 1/sqrtf(num);

//    float halfnum = 0.5f * num;
//    float y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          sin����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�sinֵ
 */

float AHRS_sinf(float angle)
{
    return arm_sin_f32(angle);
}
/**
 * @brief          cos����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�cosֵ
 */

float AHRS_cosf(float angle)
{
    return arm_cos_f32(angle);
}

/**
 * @brief          tan����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�tanֵ
 */

float AHRS_tanf(float angle)
{
    return tanf(angle);
}
/**
 * @brief          ����32λ�������ķ����Ǻ��� asin����
 * @author         RM
 * @param[in]      ����sinֵ�����1.0f����С-1.0f
 * @retval         ���ؽǶ� ��λ����
 */

float AHRS_asinf(float sin)
{

    return asinf(sin);
}

/**
 * @brief          �����Ǻ���acos����
 * @author         RM
 * @param[in]      ����cosֵ�����1.0f����С-1.0f
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

float AHRS_acosf(float cos)
{

    return acosf(cos);
}

/**
 * @brief          �����Ǻ���atan����
 * @author         RM
 * @param[in]      ����tanֵ�е�yֵ ����������С������
 * @param[in]      ����tanֵ�е�xֵ ����������С������
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

float AHRS_atan2f(float y, float x)
{
    return atan2f(y, x);
}
