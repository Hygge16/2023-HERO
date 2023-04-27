//
// Created by Yuanbin on 22-10-11.
//
#include "pid.h"
#include "string.h"

/**
  * @brief  Config the specified pid parameters
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID.  
  * @param  *para pointer to a float array that contains the parameters for the specified PID
  * @retval None
  */
static void PID_param_init(PID_TypeDef_t *pid,float para[PID_PARAMETER_CNT])
{
		if(para == NULL) return;
	
		pid->PID_clear(pid);
	
    pid->param.Deadband = para[0];
    pid->param.maxIntegral = para[1];
    pid->param.MaxOut = para[2];

    pid->param.Kp = para[3];
    pid->param.Ki = para[4];
    pid->param.Kd = para[5];

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
	
		pid->Pout = 0;
		pid->Iout = 0;
		pid->Dout = 0;
    pid->Output = 0;
}
/**
  * @brief  Clear the specified pid parameters
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID.  
  * @retval None
  */
static void PID_clear(PID_TypeDef_t *pid)
{
	
	memset(pid->Err,0,sizeof(pid->Err));
	pid->Integral = 0;
		
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->Output = 0;
}
/**
  * @brief  Check the PID Caculate Errors
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID.  
  * @retval None
  */
static void PID_ErrorHandle(PID_TypeDef_t *pid)
{
		/*Output NAN Handle*/
		if(isnan(pid->Output) == true)
		{
				pid->ERRORHandler.ERRORType = PID_Output_NAN;
		}
}
/**
  * @brief  Caculate the PID Controller
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @param  Target  Target variable for the pid controller
  * @param  Measure Measure variable for the pid controller
  * @retval None
  */
float f_PID_Calculate(PID_TypeDef_t *pid, float Target,float Measure)
{
    if (pid == NULL)return 0; 
		
		PID_ErrorHandle(pid);
		if(pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
		{
			pid->PID_clear(pid);
			return 0;
		}
		
		/* update the target/measure variable */
		pid->Target = Target;
		pid->Measure = Measure;

		/* check the pid type */
		if(pid->Type == PID_POSITION)
		{
			pid->Err[1] = pid->Err[0];
			pid->Err[0] = pid->Target - pid->Measure;
			
			if(ABS(pid->Err[0]) > pid->param.Deadband)
			{
				ABS(pid->param.Ki) > 0 ? (pid->Integral += pid->Err[0]) : (pid->Integral = 0);
				
				VAL_LIMIT(pid->Integral,-pid->param.maxIntegral,pid->param.maxIntegral);
				
				pid->Pout = pid->param.Kp * pid->Err[0];
				pid->Iout = pid->param.Ki * pid->Integral;
				pid->Dout = pid->param.Kd * (pid->Err[0] - pid->Err[1]);
				
				pid->Output = pid->Pout + pid->Iout + pid->Dout;
			}
		}
		else if(pid->Type == PID_VELOCITY)
		{
			pid->Err[2] = pid->Err[1];
			pid->Err[1] = pid->Err[0];
			pid->Err[0] = pid->Target - pid->Measure;
			
			pid->Pout = pid->param.Kp * (pid->Err[0] - pid->Err[1]);
			pid->Iout = pid->param.Ki *  pid->Err[0];
			pid->Dout = pid->param.Kd * (pid->Err[0] - 2.f*pid->Err[1] + pid->Err[2]);
			
			pid->Output += pid->Pout + pid->Iout + pid->Dout;
		}
		VAL_LIMIT(pid->Output,-pid->param.MaxOut,pid->param.MaxOut);
		return pid->Output;
}
/**
  * @brief  Config the PID Controller
  * @param  *pid pointer to a PID_TypeDef_t structure that contains
  *              the configuration information for the specified PID. 
  * @param  type  the pid controller type
  * @param  *para pointer to a float array that contains the parameters for the specified PID
  * @retval None
  */
void PID_Init(PID_TypeDef_t *pid,PIDType_e type,float para[PID_PARAMETER_CNT])
{
		pid->PID_clear = PID_clear;
		pid->Type = type;
    pid->PID_param_init = PID_param_init;
		pid->PID_clear(pid);
    pid->PID_param_init(pid, para);
}

