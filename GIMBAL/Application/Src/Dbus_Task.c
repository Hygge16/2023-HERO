#include "Dbus_Task.h"
#include "cmsis_os.h"

/*--------------------------   变量声明   ------------------------------*/


/*--------------------------   变量声明   ------------------------------*/

void Dbus_Task(void *args)
{
    uint32_t currentTime;

	while(1)
	{
		currentTime = xTaskGetTickCount();
        if(currentTime-rc_ctrl.Offline_Cnt > 50)
				{
            Gimbal_Ctrl.state_Setup(OFF);
            Shoot.state_Setup(OFF);
            Chassis_Ctrl.state_Setup(OFF);
						Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
        }
				else if(Chassis_Ctrl.ctrl == 2)
				{
						Gimbal_Ctrl.state_Setup(ON);
						Shoot.state_Setup(ON);
						Chassis_Ctrl.state_Setup(ON);
						Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
				}
        else if(Chassis_Ctrl.ctrl==1)
        {
							switch (rc_ctrl.rc.s[0])//右档选项
							{
	/*------------------------------------------------*/ 
							case 1://右上（云台部分
											switch (rc_ctrl.rc.s[1])//左档选项
											{
											case 1://左上(全拨上，整车电机关闭卸力)
													Gimbal_Ctrl.state_Setup(OFF);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 3://左中
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 2://左下
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(ON);
													Chassis_Ctrl.state_Setup(ON);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											default:
													break;
											}
									break;
	/*------------------------------------------------*/                    
							case 3://右中
											switch (rc_ctrl.rc.s[1])//左档选项
											{
											case 1://左上
													Gimbal_Ctrl.state_Setup(OFF);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 3://左中
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 2://左下
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(ON);
													Chassis_Ctrl.state_Setup(ON);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											default:
													break;
											}
									break;
	/*------------------------------------------------*/                    
							case 2://右下
											switch (rc_ctrl.rc.s[1])//左档选项
											{
											case 1://左上
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 3://左中
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 2://左下
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(ON);
													Chassis_Ctrl.mode_Setup(CHASSIS_SPIN);
													break;
											
											default:
													break;
											}
									break;

							default:
									break;
							}
					}
        vTaskDelayUntil(&currentTime, 3);
    }
}

