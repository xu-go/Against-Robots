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




GimbalCtrlMode  Gimbalmode;   //定义云台控制模式

//测量值
float Cloud_Angle_Measure;
float Cloud_Speed_Measure;
float Cloud_Current_Measure;
float Cloud_Final_Output;
float kRc_Mech_Cloud_Standard;

//位置环参数
float Cloud_Angle_Target_Sum;
float Cloud_Angle_Measure_Sum;
float Cloud_Angle_Measure_Prev;
float Cloud_Buff_Target_Sum;



//PID
float Cloud_Angle_kpid[2][3];
float Cloud_Angle_Error[2];
float pTermCloudAngle[2], iTermCloudAngle[2],pidTermCloudAngle[2];//  inner/outer
extern float iTermPosiMax;//位置环积分限幅


#define abs(x) ((x)>0? (x):(-(x)))
extern  RC_ctrl_t rc_ctrl;

void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{	
		currentTime = xTaskGetTickCount();//当前系统时间
		
		/* 代码段 */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化模式
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
		
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
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
	
		Cloud_Angle_Target_Sum = Cloud_Angle_Measure;//位置环目标角度重置
	  Cloud_Angle_Measure_Sum = Cloud_Angle_Measure;//位置环转过角度重置
	  Cloud_Angle_Measure_Prev = Cloud_Angle_Measure;//上次位置重置
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
	  kRc_Mech_Cloud_Standard = 122.f;  //调节摇杆灵敏度,调节速度
	
		//Cloud_Angle_Target_Sum   = RAMP_float( mid_angle, Cloud_Angle_Target_Sum, Slope_Begin_Yaw);
		Cloud_Angle_Target_Sum = constrain_float( -kRc_Mech_Cloud_Standard*RC_CH2_LLR_OFFSET, -90000, 90000);//前后
	}
	
}




/*----------------------------------------------位置环PID--------------------------------------------*/
void GIMBAL_PositionLoop(void)
{
	
	//获取转过的总角度值
	CloudAngleSum();  
	//外环计算
	Cloud_Angle_Error[OUTER] = Cloud_Angle_Target_Sum - Cloud_Angle_Measure_Sum;
	pTermCloudAngle[OUTER] = Cloud_Angle_Error[OUTER] * Cloud_Angle_kpid[OUTER][KP];
	
	//内环计算
	Cloud_Angle_Error[INNER]  =  pTermCloudAngle[OUTER] - Cloud_Speed_Measure;
	pTermCloudAngle[INNER]   = Cloud_Angle_Error[INNER] * Cloud_Angle_kpid[INNER][KP];		
	iTermCloudAngle[INNER]  += Cloud_Angle_Error[INNER] * Cloud_Angle_kpid[INNER][KI] * 0.001f;
	iTermCloudAngle[INNER]   = constrain_float( iTermCloudAngle[INNER], -iTermPosiMax, iTermPosiMax );

	Cloud_Final_Output = constrain_float( pTermCloudAngle[INNER] + iTermCloudAngle[INNER] , -6000, 6000);
}



 

void CloudAngleSum(void)
{		 
	//临界值判断法
	if (abs(Cloud_Angle_Measure - Cloud_Angle_Measure_Prev) > 4095)//转过半圈
	{		
		//本次测量角度小于上次测量角度且过了半圈,则说明本次过了零点
		if (Cloud_Angle_Measure < Cloud_Angle_Measure_Prev)//过半圈且过零点
		{
			//已经转到下一圈,则累计转过 8191(一圈) - 上次 + 本次
			Cloud_Angle_Measure_Sum += 8191 - Cloud_Angle_Measure_Prev + Cloud_Angle_Measure;
		}
		else
		{
			//超过了一圈
			Cloud_Angle_Measure_Sum -= 8191 - Cloud_Angle_Measure + Cloud_Angle_Measure_Prev;
		}
	}
	else      
	{
		//未过临界值,累加上转过的角度差
		Cloud_Angle_Measure_Sum += Cloud_Angle_Measure - Cloud_Angle_Measure_Prev;
	}

	//记录此时电机角度,下一次计算转过角度差用,用来判断是否转过1圈
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
