/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Start_Task.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "delay.h"
#include "led.h"
#include "motor_ctrl.h"
#include "stdbool.h"



#define Chassis_TASK_PRIO 8
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;


//#define GIMBAL_TASK_PRIO 7
//#define GIMBAL_STK_SIZE 512
//TaskHandle_t GIMBALTask_Handler;


#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define System_control_TASK_PRIO 6
#define System_control_STK_SIZE 512
static TaskHandle_t System_controlTask_Handler;


extern RC_ctrl_t rc_ctrl;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

	
//	  xTaskCreate((TaskFunction_t)GIMBAL_task,
//                (const char *)"GIMBAL_task",
//                (uint16_t)GIMBAL_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)GIMBAL_TASK_PRIO,
//                (TaskHandle_t *)&GIMBALTask_Handler);
								

    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);

								
		 xTaskCreate((TaskFunction_t)System_control,
                (const char *)"System_control",
                (uint16_t)System_control_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)System_control_TASK_PRIO,
                (TaskHandle_t *)&System_controlTask_Handler);																						

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}


void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}



/*******************************************************/
/*系统状态更新，启动延时，系统保护*/
//控制模式
eRemoteMode remoteMode = RC;

//系统状态
eSystemState systemState = SYSTEM_STARTING;

//返回控制模式
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//返回系统状态
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}



//系统更新状态任务函数
void System_control(void *pvParameters)
{
	static portTickType currentTime;
		
	
		
	currentTime = xTaskGetTickCount();	//获取当前系统时间	
	while(1)
	{
	  if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化模式
		{
		  MPU_Update_Starting();
      system_update();
//			lujinset();
		}
		else
		{
			//sensor_update();	
      MPU_Update();
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
  }

}

int key,rbutton,bbutton,rplan1,rplan2,rplan3,bplan1,bplan2,bplan3,key1;


#if MODE==TEST


void system_update(void)
{
	static uint32_t  ulInitCnt  =  0;
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;

		if (ulInitCnt > 2500)//启动延时,1ms*2k=2s,为了给MPU启动时间
		{
			ulInitCnt = 0;

			systemState = SYSTEM_RUNNING;//启动完成,转换成普通模式
		}
	}
}

#else


//整个系统启动延时2500ms
void system_update(void)
{
	static uint32_t  ulInitCnt  =  0;
	key=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1);
	
	
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;

		if (ulInitCnt > 2500)//启动延时,1ms*2k=2s,为了给MPU启动时间
		{
			if(key == 0)
			{
			ulInitCnt = 0;

			systemState = SYSTEM_RUNNING;//启动完成,转换成普通模式
			}
			else
			{
				systemState = SYSTEM_STARTING;
			}
		}
	}
}
//int red,blue,ii,keymode;
//int KEY_Scan(int mode)
//{	 
//	static int key_up=1;
//	if(mode)key_up=1; 		  
//	if(key_up&&(red==0||blue==0))
//	{
//		for(ii=0;ii<5000;ii++) 
//		{
//		}
//		key_up=0;
//		if(red==0)return 1;
//		else if(blue==0)return 2;
//	}else if(red==1&&blue==1)key_up=1; 	    
// 	return 0;
//}



//void lujinset(void)
//{
//    red=GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6);
//	  blue=GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_7);
//	  keymode=KEY_Scan(0);
//	  if(keymode==1)
//		{
//			 GPIO_SetBits(GPIOG, GPIO_Pin_1);
//			
////		  if(rplan1==0){
////			}
////			else if(rplan2==0){
////			}
////			else if(rplan3==0){
////			
////			}
//		
//		}
//		else if(keymode==2)
//		{
//			 GPIO_SetBits(GPIOG, GPIO_Pin_2);
////		   if(bplan1==0){
////			}
////			else if(bplan2==0){
////			}
////			else if(bplan3==0){
////			
////			}
//			
//		}

//}
	

#endif

void remote_StateControl(void)
{
	 remoteMode=RC;
}




