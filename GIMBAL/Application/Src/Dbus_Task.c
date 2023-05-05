#include "Dbus_Task.h"
#include "cmsis_os.h"

/*--------------------------   ��������   ------------------------------*/


/*--------------------------   ��������   ------------------------------*/

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
							switch (rc_ctrl.rc.s[0])//�ҵ�ѡ��
							{
	/*------------------------------------------------*/ 
							case 1://���ϣ���̨����
											switch (rc_ctrl.rc.s[1])//��ѡ��
											{
											case 1://����(ȫ���ϣ���������ر�ж��)
													Gimbal_Ctrl.state_Setup(OFF);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 3://����
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 2://����
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
							case 3://����
											switch (rc_ctrl.rc.s[1])//��ѡ��
											{
											case 1://����
													Gimbal_Ctrl.state_Setup(OFF);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_INVA);
													break;
											
											case 3://����
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 2://����
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
							case 2://����
											switch (rc_ctrl.rc.s[1])//��ѡ��
											{
											case 1://����
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 3://����
													Gimbal_Ctrl.state_Setup(ON);
													Shoot.state_Setup(OFF);
													Chassis_Ctrl.state_Setup(OFF);
													Chassis_Ctrl.mode_Setup(CHASSIS_FOLO);
													break;
											
											case 2://����
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

