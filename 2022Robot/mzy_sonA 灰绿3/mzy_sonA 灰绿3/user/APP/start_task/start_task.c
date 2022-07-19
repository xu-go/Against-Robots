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
#include "key.h"
void Mode_Switch(void);

#define Chassis_TASK_PRIO 8
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;
extern int nbut;

//#define GIMBAL_TASK_PRIO 7
//#define GIMBAL_STK_SIZE 512
//TaskHandle_t GIMBALTask_Handler;


#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define System_control_TASK_PRIO 6
#define System_control_STK_SIZE 512
static TaskHandle_t System_controlTask_Handler;

//static TaskHandle_t Mode_Switch_Handler=NULL;


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
//	xTaskCreate(
//		(TaskFunction_t)Mode_Switch,
//		(const char*)"Mode_Switch",
//		(uint16_t) 512,
//		(void*)NULL,
//		(UBaseType_t) System_control_TASK_PRIO,
//		(TaskHandle_t *)&Mode_Switch_Handler
//	);																					

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

volatile int key,sw1,mode=0;

void Mode_Switch(void)
{
	while(1)
	{
		sw1=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);
		if(sw1)
		{
			vTaskDelay(20);
			if(sw1)
			{
				mode=(mode+1)%PLAN_NUM;
				while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2));
			}
		}
		GPIO_Write(GPIOG,(~(1<<mode))&0xff);
	}

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

int tmp;

//系统更新状态任务函数
void System_control(void *pvParameters)
{
	static portTickType currentTime;
	
	currentTime = xTaskGetTickCount();	//获取当前系统时间	
	while(1)
	{
	  if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化阶段
		{	
	    tmp = KEY_Scan(0);
			if(tmp!=0)
			{
				nbut=tmp;
			}
		  MPU_Update_Starting();
      system_update();
		
	}
		else
		{
			//sensor_update();	
      MPU_Update();
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//绝对延时
  }

}

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
				//vTaskDelete(Mode_Switch_Handler);
				systemState = SYSTEM_RUNNING;//启动完成,转换成普通模式
			}
			else
			{
				systemState = SYSTEM_STARTING;
			}
		}
	}
}


#endif

void remote_StateControl(void)
{
	 remoteMode=RC;
}




