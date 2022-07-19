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
#ifndef START_TASK_H
#define START_TASK_H
#include "main.h"


void startTast(void);


/*---------------------更新系统状态-------------------------*/
void System_control(void *pvParameters);

typedef enum
{
    RC   = 0,  

} eRemoteMode;  // 遥控方式


typedef enum
{
	  SYSTEM_STARTING  = 0,
	  SYSTEM_RUNNING   = 1,

} eSystemState;

eRemoteMode SYSTEM_GetRemoteMode( void );
eSystemState SYSTEM_GetSystemState( void );
void system_update(void);
void lujinset(void);
/*---------------------------------------------------*/
#endif
