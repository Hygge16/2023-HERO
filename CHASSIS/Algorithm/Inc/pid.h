//
// Created by Yuanbin on 22-10-11.
//

#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "stm32f4xx.h"

#define VAL_LIMIT(x,min,max)  do{ \
                                    if (x > max) {x = max;} \
                                    else if (x < min) {x = min;} \
                                }while(0U)
#ifndef ABS
#define ABS(x) (((x) > 0) ? (x) : -(x))
#endif

/* define PID parameter cnt */																
#ifndef PID_PARAMETER_CNT 
#define PID_PARAMETER_CNT 6							
#endif
																
/* 异常情况枚举 */
typedef enum
{
    PID_ERROR_NONE = 0x00U, //无异常
    Motor_Blocked  = 0x01U, //堵转
		PID_Output_NAN = 0x02U,  //nan
}PID_ErrorType_e;

/* 异常结构体 */
typedef struct
{
    uint16_t ERRORCount;
    PID_ErrorType_e ERRORType;
}PID_ErrorHandler_t;

/* PID类型枚举 */
typedef enum
{
		PID_Type_None = 0x00U,
		PID_POSITION = 0x01U, //position pid
		PID_VELOCITY = 0x02U, //velocity pid
}PIDType_e;

/* PID参数结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

		float Deadband;
    float maxIntegral;
    float MaxOut;
}PID_param_t;

/* PID结构体 */
typedef struct _PID_TypeDef
{
		PIDType_e Type;
	
		float Target;
		float Measure;
	
    float Err[3];
		float Integral;

    float Pout;
    float Iout;
    float Dout;
    float Output;
	
		PID_param_t param;
    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
				float *para);
				
		void (*PID_clear)(
				struct _PID_TypeDef *pid);
				
}PID_TypeDef_t;

extern void PID_Init(PID_TypeDef_t *pid,PIDType_e type,float para[PID_PARAMETER_CNT]);
extern float f_PID_Calculate(PID_TypeDef_t *pid, float Target,float Measure);
#endif


