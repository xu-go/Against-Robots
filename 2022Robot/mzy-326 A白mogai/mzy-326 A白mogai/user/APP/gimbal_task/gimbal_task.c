#include "gimbal_task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "main.h"

#include "remote_control.h"
#include "CAN_Receive.h"
#include "limit.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"




GimbalCtrlMode  Gimbalmode;   //������̨����ģʽ

//����ֵ
float Cloud_Angle_Measure;
float Cloud_Speed_Measure;
float Cloud_Current_Measure;
float Cloud_Final_Output;
float kRc_Mech_Cloud_Standard;

//λ�û�����
float Cloud_Angle_Target_Sum;
float Cloud_Angle_Measure_Sum;
float Cloud_Angle_Measure_Prev;
float Cloud_Buff_Target_Sum;



//PID
float Cloud_Angle_kpid[2][3];
float Cloud_Angle_Error[2];
float pTermCloudAngle[2], iTermCloudAngle[2],pidTermCloudAngle[2];//  inner/outer
extern float iTermPosiMax;//λ�û������޷�


#define abs(x) ((x)>0? (x):(-(x)))
extern  RC_ctrl_t rc_ctrl;

void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		
		/* ����� */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//��ʼ��ģʽ
		{
       GIMBAL_InitCtrl();
		}
		else
		{
			if(SYSTEM_GetRemoteMode() == RC)
			{
        RC_Set_Mode();
			  GIMBAL_Set_Control();
			}

		}
		GIMBAL_PositionLoop();
		GIMBAL_CanSend();			
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

void GIMBAL_InitCtrl(void)
{
		Cloud_Angle_kpid[OUTER][KP] = 0.08;//0.005
		Cloud_Angle_kpid[OUTER][KI] = 0;//0.005
		Cloud_Angle_kpid[OUTER][KD] = 0;//0.005
		Cloud_Angle_kpid[INNER][KP] = 6;//0.005
		Cloud_Angle_kpid[INNER][KI] = 0;//0.005
		Cloud_Angle_kpid[INNER][KD] = 0;//0.005
	
		Cloud_Angle_Target_Sum = Cloud_Angle_Measure;//λ�û�Ŀ��Ƕ�����
	  Cloud_Angle_Measure_Sum = Cloud_Angle_Measure;//λ�û�ת���Ƕ�����
	  Cloud_Angle_Measure_Prev = Cloud_Angle_Measure;//�ϴ�λ������
		Cloud_Buff_Target_Sum = Cloud_Angle_Target_Sum;
	  iTermPosiMax   = 2500;
}

void RC_Set_Mode(void)
{
	if(IF_RC_SW1_DOWN)
	{
		Gimbalmode = CLOUD_MECH_MODE;
	}
	if(IF_RC_SW1_MID)
	{
		Cloud_Angle_Target_Sum   = RAMP_float( mid_angle, Cloud_Angle_Target_Sum, Slope_Begin_Yaw);
	}
}

void GIMBAL_Set_Control(void)
{
	if(Gimbalmode == CLOUD_MECH_MODE)
	{
	  kRc_Mech_Cloud_Standard = 122.f;  //����ҡ��������,�����ٶ�
	
		//Cloud_Angle_Target_Sum   = RAMP_float( mid_angle, Cloud_Angle_Target_Sum, Slope_Begin_Yaw);
		Cloud_Angle_Target_Sum = constrain_float( -kRc_Mech_Cloud_Standard*RC_CH2_LLR_OFFSET, -90000, 90000);//ǰ��
	}
	
}




/*----------------------------------------------λ�û�PID--------------------------------------------*/
void GIMBAL_PositionLoop(void)
{
	
	//��ȡת�����ܽǶ�ֵ
	CloudAngleSum();  
	//�⻷����
	Cloud_Angle_Error[OUTER] = Cloud_Angle_Target_Sum - Cloud_Angle_Measure_Sum;
	pTermCloudAngle[OUTER] = Cloud_Angle_Error[OUTER] * Cloud_Angle_kpid[OUTER][KP];
	
	//�ڻ�����
	Cloud_Angle_Error[INNER]  =  pTermCloudAngle[OUTER] - Cloud_Speed_Measure;
	pTermCloudAngle[INNER]   = Cloud_Angle_Error[INNER] * Cloud_Angle_kpid[INNER][KP];		
	iTermCloudAngle[INNER]  += Cloud_Angle_Error[INNER] * Cloud_Angle_kpid[INNER][KI] * 0.001f;
	iTermCloudAngle[INNER]   = constrain_float( iTermCloudAngle[INNER], -iTermPosiMax, iTermPosiMax );

	Cloud_Final_Output = constrain_float( pTermCloudAngle[INNER] + iTermCloudAngle[INNER] , -6000, 6000);
}



 

void CloudAngleSum(void)
{		 
	//�ٽ�ֵ�жϷ�
	if (abs(Cloud_Angle_Measure - Cloud_Angle_Measure_Prev) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Cloud_Angle_Measure < Cloud_Angle_Measure_Prev)//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Cloud_Angle_Measure_Sum += 8191 - Cloud_Angle_Measure_Prev + Cloud_Angle_Measure;
		}
		else
		{
			//������һȦ
			Cloud_Angle_Measure_Sum -= 8191 - Cloud_Angle_Measure + Cloud_Angle_Measure_Prev;
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Cloud_Angle_Measure_Sum += Cloud_Angle_Measure - Cloud_Angle_Measure_Prev;
	}

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Cloud_Angle_Measure_Prev = Cloud_Angle_Measure;
}



void GIMBAL_CanSend(void)
{
	CAN_CMD_GIMBAL(0,0,Cloud_Final_Output,0);
}


void GIMBAL_UpdateAngle(int16_t angle )
{
	Cloud_Angle_Measure = angle;
}


void GIMBAL_UpdateSpeed(int16_t speed )
{
	Cloud_Speed_Measure = speed;
}


void GIMBAL_UpdateCurrent(int16_t current )
{
	Cloud_Current_Measure = current;
}
